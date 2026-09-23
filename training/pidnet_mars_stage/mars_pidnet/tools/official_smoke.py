#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, sys, time
from pathlib import Path
import numpy as np
from PIL import Image
import torch
import torch.nn.functional as F
ROOT=Path(__file__).resolve().parents[2]; sys.path.insert(0,str(ROOT))
from mars_pidnet.modeling import build_pidnet_s, load_matching_weights

COLORS=np.array([[128,64,128],[244,35,232],[70,70,70],[102,102,156],[190,153,153],[153,153,153],[250,170,30],[220,220,0],[107,142,35],[152,251,152],[70,130,180],[220,20,60],[255,0,0],[0,0,142],[0,0,70],[0,60,100],[0,80,100],[0,0,230],[119,11,32]],dtype=np.uint8)
p=argparse.ArgumentParser(); p.add_argument("--image",required=True); p.add_argument("--weights"); p.add_argument("--output",required=True); p.add_argument("--iterations",type=int,default=100); args=p.parse_args()
if not torch.cuda.is_available(): raise RuntimeError("CUDA unavailable")
device=torch.device("cuda:0"); out=Path(args.output); out.mkdir(parents=True,exist_ok=True); model=build_pidnet_s(19,training=False); load_info=None
if args.weights: load_info=load_matching_weights(model,args.weights)
model.eval().to(device); rgb=np.asarray(Image.open(args.image).convert("RGB").resize((640,384)),dtype=np.uint8); mean=np.array([.485,.456,.406]); std=np.array([.229,.224,.225])
x=torch.from_numpy((((rgb/255)-mean)/std).transpose(2,0,1)).float()[None].to(device); torch.cuda.reset_peak_memory_stats()
with torch.no_grad():
    for _ in range(10): logits=model(x)
    torch.cuda.synchronize(); start=time.perf_counter()
    for _ in range(args.iterations): logits=model(x)
    torch.cuda.synchronize(); latency=(time.perf_counter()-start)*1000/args.iterations
    logits=F.interpolate(logits,size=(384,640),mode="bilinear",align_corners=False); pred=logits.argmax(1)[0].byte().cpu().numpy()
color=COLORS[pred]; overlay=(.6*rgb+.4*color).astype(np.uint8); Image.fromarray(rgb).save(out/"input.png"); Image.fromarray(pred).save(out/"mask.png"); Image.fromarray(color).save(out/"color_mask.png"); Image.fromarray(overlay).save(out/"overlay.png")
info={"weights_requested":args.weights,"weights_loaded":bool(args.weights),"load_info":load_info,"input_shape":list(x.shape),"output_shape":list(logits.shape),"dtype":str(logits.dtype),"device":str(logits.device),"iterations":args.iterations,"mean_latency_ms":latency,"peak_memory_mib":torch.cuda.max_memory_allocated()/2**20}
(out/"metrics.json").write_text(json.dumps(info,indent=2)); print(json.dumps(info,indent=2))
