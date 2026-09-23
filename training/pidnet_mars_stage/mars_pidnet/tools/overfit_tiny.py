#!/usr/bin/env python3
from __future__ import annotations
import argparse, csv, json, sys
from pathlib import Path
import numpy as np
from PIL import Image
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from mars_pidnet.datasets import MarsDataset
from mars_pidnet.modeling import build_pidnet_s, freeze_options, load_matching_weights

COLORS = np.array([[50,50,50],[100,80,50],[180,120,50],[70,20,20],[200,180,120],[180,0,0],[0,0,180],[220,190,0]], dtype=np.uint8)

def boundary_target(mask):
    valid = mask != 255; clean = mask.clone(); clean[~valid] = 0
    edge = torch.zeros_like(clean, dtype=torch.bool)
    edge[:,1:] |= clean[:,1:] != clean[:,:-1]; edge[:,:-1] |= clean[:,:-1] != clean[:,1:]
    edge[:,:,1:] |= clean[:,:,1:] != clean[:,:,:-1]; edge[:,:,:-1] |= clean[:,:,:-1] != clean[:,:,1:]
    return (edge & valid).float().unsqueeze(1)

def confusion(pred, target, classes):
    valid = target != 255; x = target[valid] * classes + pred[valid]
    return torch.bincount(x, minlength=classes*classes).reshape(classes, classes)

def colorize(mask):
    out = np.zeros((*mask.shape,3), np.uint8); valid = mask != 255; out[valid] = COLORS[mask[valid]]; return out

def main():
    p=argparse.ArgumentParser(); p.add_argument("--dataset", required=True); p.add_argument("--epochs", type=int, default=200)
    p.add_argument("--classes", type=int, choices=(6,8), default=6); p.add_argument("--output", required=True)
    p.add_argument("--split", default="train"); p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--weights"); p.add_argument("--freeze-backbone", action="store_true"); p.add_argument("--freeze-bn", action="store_true")
    p.add_argument("--max-samples", type=int, default=10); args=p.parse_args()
    if not torch.cuda.is_available(): raise RuntimeError("CUDA GPU is required")
    torch.manual_seed(304); device=torch.device("cuda:0"); out=Path(args.output); out.mkdir(parents=True, exist_ok=True)
    ds=MarsDataset(args.dataset,args.split,"coarse" if args.classes==6 else "fine",augment=True)
    if not 1 <= len(ds) <= args.max_samples: raise ValueError(f"tiny overfit expects 1..{args.max_samples} samples, found {len(ds)}")
    batch_size=min(2,len(ds)); loader=DataLoader(ds,batch_size=batch_size,shuffle=True,num_workers=0)
    model=build_pidnet_s(args.classes,training=True).to(device); load_info=None
    if args.weights: load_info=load_matching_weights(model,args.weights)
    freeze_options(model,args.freeze_backbone,args.freeze_bn)
    optimizer=torch.optim.AdamW([x for x in model.parameters() if x.requires_grad],lr=args.lr,weight_decay=1e-4)
    first=next(x.detach().clone() for x in model.parameters() if x.requires_grad); losses=[]
    for epoch in range(args.epochs):
        model.train(); total=0.0
        if args.freeze_bn or batch_size == 1: freeze_options(model,False,True)
        for batch in loader:
            image=batch["image"].to(device); target=batch["mask"].to(device); outputs=model(image)
            small=F.interpolate(target[:,None].float(),size=outputs[1].shape[-2:],mode="nearest")[:,0].long()
            seg=0.4*F.cross_entropy(outputs[0],small,ignore_index=255)+F.cross_entropy(outputs[1],small,ignore_index=255)
            bd=F.binary_cross_entropy_with_logits(outputs[2],boundary_target(small)); loss=seg+bd
            if not torch.isfinite(loss): raise RuntimeError(f"non-finite loss at epoch {epoch}")
            optimizer.zero_grad(set_to_none=True); loss.backward(); optimizer.step(); total += loss.item()
        losses.append(total/len(loader)); print(f"epoch={epoch+1} loss={losses[-1]:.6f}",flush=True)
    changed=not torch.equal(first,next(x.detach() for x in model.parameters() if x.requires_grad))
    if not changed: raise RuntimeError("no trainable parameter changed")
    ckpt=out/"checkpoint.pt"; torch.save({"state_dict":model.state_dict(),"classes":args.classes,"losses":losses},ckpt)
    reload_model=build_pidnet_s(args.classes,training=True).to(device); reload_model.load_state_dict(torch.load(ckpt,map_location=device,weights_only=False)["state_dict"]); reload_model.eval()
    matrix=torch.zeros(args.classes,args.classes,dtype=torch.int64); pred_dir=out/"predictions"; pred_dir.mkdir(exist_ok=True)
    with torch.no_grad():
        for batch in DataLoader(ds,batch_size=1,shuffle=False):
            image=batch["image"].to(device); target=batch["mask"][0]; logits=reload_model(image)[1]
            pred=F.interpolate(logits,size=target.shape,mode="bilinear",align_corners=False).argmax(1)[0].cpu(); matrix += confusion(pred,target,args.classes)
            stem=batch["id"][0]; raw=np.asarray(Image.open(batch["image_path"][0]).convert("RGB")); raw=np.pad(raw,((12,12),(0,0),(0,0)))
            gt=target.numpy().astype(np.uint8); pm=pred.numpy().astype(np.uint8); pc=colorize(pm); overlay=(0.6*raw+0.4*pc).astype(np.uint8)
            Image.fromarray(raw).save(pred_dir/f"{stem}_input.png"); Image.fromarray(colorize(gt)).save(pred_dir/f"{stem}_gt.png")
            Image.fromarray(pm).save(pred_dir/f"{stem}_pred.png"); Image.fromarray(overlay).save(pred_dir/f"{stem}_overlay.png")
    diag=matrix.diag().float(); denom=matrix.sum(0)+matrix.sum(1)-diag; iou=torch.where(denom>0,diag/denom,torch.nan)
    metrics={"losses":losses,"parameter_changed":changed,"per_class_iou":[None if torch.isnan(x) else x.item() for x in iou],"confusion_matrix":matrix.tolist(),"weight_load":load_info}
    (out/"metrics.json").write_text(json.dumps(metrics,indent=2));
    with (out/"loss.csv").open("w",newline="") as f: w=csv.writer(f); w.writerow(["epoch","loss"]); w.writerows(enumerate(losses,1))
    import matplotlib.pyplot as plt
    plt.plot(range(1,len(losses)+1),losses); plt.xlabel("epoch"); plt.ylabel("loss"); plt.tight_layout(); plt.savefig(out/"loss_curve.png"); plt.close()
    print(json.dumps(metrics,indent=2))
if __name__=="__main__": main()
