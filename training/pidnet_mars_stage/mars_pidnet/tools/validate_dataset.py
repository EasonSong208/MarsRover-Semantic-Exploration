#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np
from PIL import Image

def digest(path):
    h=hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda:f.read(1024*1024),b""): h.update(block)
    return h.hexdigest()

p=argparse.ArgumentParser(); p.add_argument("dataset",type=Path); p.add_argument("--granularity",choices=("fine","coarse"),default="coarse")
p.add_argument("--report-dir",type=Path,default=Path("reports")); args=p.parse_args(); root=args.dataset; allowed=set(range(8 if args.granularity=="fine" else 6))|{255}
report={"granularity":args.granularity,"splits":{},"errors":[],"warnings":[],"duplicate_hashes":{},"cross_split_duplicates":[]}
hashes=defaultdict(list); split_stems={}
for split in ("train","val","test"):
    sf=root/"splits"/f"{split}.txt"; stems=[x.strip() for x in sf.read_text().splitlines() if x.strip()] if sf.is_file() else []
    split_stems[split]=set(stems); stats=Counter(); empty=[]; all_ignore=[]
    image_dir=root/"images"/split; mask_dir=root/("masks_fine" if args.granularity=="fine" else "masks_coarse")/split
    actual_images=[p for p in image_dir.glob("*") if p.suffix.lower() in {".png",".jpg",".jpeg"}] if image_dir.is_dir() else []
    actual_masks=list(mask_dir.glob("*.png")) if mask_dir.is_dir() else []
    for stem in stems:
        imgs=[p for p in actual_images if p.stem==stem]; mask=mask_dir/f"{stem}.png"
        if len(imgs)!=1 or not mask.is_file(): report["errors"].append(f"{split}/{stem}: pairing failure") ; continue
        try: image=np.asarray(Image.open(imgs[0]).convert("RGB")); raw=Image.open(mask); arr=np.asarray(raw)
        except Exception as exc: report["errors"].append(f"{split}/{stem}: unreadable: {exc}"); continue
        if raw.mode != "L" or arr.ndim != 2 or arr.dtype != np.uint8: report["errors"].append(f"{split}/{stem}: mask must be mode L uint8")
        if image.shape[:2] != arr.shape: report["errors"].append(f"{split}/{stem}: dimension mismatch")
        illegal=set(np.unique(arr).tolist())-allowed
        if illegal: report["errors"].append(f"{split}/{stem}: illegal values {sorted(illegal)}")
        values, frequencies=np.unique(arr,return_counts=True); counts=Counter({int(k):int(v) for k,v in zip(values,frequencies)}); stats.update(counts)
        if not any(counts[c] for c in allowed-{255}): empty.append(stem)
        if counts[255] == arr.size: all_ignore.append(stem)
        ih=digest(imgs[0]); hashes[ih].append(f"{split}/{stem}")
    total=sum(stats.values()); report["splits"][split]={"listed":len(stems),"images_on_disk":len(actual_images),"masks_on_disk":len(actual_masks),"class_pixels":{str(k):int(v) for k,v in sorted(stats.items())},"class_ratios":{str(k):float(v/total) for k,v in sorted(stats.items())} if total else {},"empty_labels":empty,"all_ignore":all_ignore}
for h,items in hashes.items():
    if len(items)>1:
        report["duplicate_hashes"][h]=items
        if len({x.split('/')[0] for x in items})>1: report["cross_split_duplicates"].append(items)
for a,aset in split_stems.items():
    for b,bset in split_stems.items():
        if a<b and aset&bset: report["errors"].append(f"same stems in {a}/{b}: {sorted(aset&bset)}")
numeric=[]
for split,stems in split_stems.items():
    for stem in stems:
        m=re.match(r"^(.*?)(\d+)$",stem)
        if m: numeric.append((m.group(1),int(m.group(2)),split,stem))
lookup={(p,n):(s,x) for p,n,s,x in numeric}
for p,n,s,x in numeric:
    for delta in (-1,1):
        other=lookup.get((p,n+delta))
        if other and other[0]!=s: report["warnings"].append(f"adjacent frames split: {x}({s}) / {other[1]}({other[0]})")
args.report_dir.mkdir(parents=True,exist_ok=True); (args.report_dir/"dataset_validation.json").write_text(json.dumps(report,indent=2))
lines=["# Dataset validation",f"- Status: {'FAIL' if report['errors'] else 'PASS'}",f"- Errors: {len(report['errors'])}",f"- Warnings: {len(report['warnings'])}","","## Errors"]+[f"- {x}" for x in report["errors"]]+["","## Warnings"]+[f"- {x}" for x in report["warnings"]]
(args.report_dir/"dataset_validation.md").write_text("\n".join(lines)+"\n"); print(json.dumps(report,indent=2)); raise SystemExit(bool(report["errors"]))
