#!/usr/bin/env python3
import argparse, subprocess, sys
from pathlib import Path
import numpy as np
from PIL import Image

p=argparse.ArgumentParser(); p.add_argument("--output",required=True); args=p.parse_args(); root=Path(args.output).resolve(); data=root/"dataset"
for split in ("train","val","test"):
    (data/"images"/split).mkdir(parents=True,exist_ok=True); (data/"masks_coarse"/split).mkdir(parents=True,exist_ok=True); (data/"splits").mkdir(exist_ok=True)
    stems=[]
    for i in range(2 if split=="train" else 1):
        stem=f"synthetic_{split}_{i:02d}"; stems.append(stem); yy,xx=np.mgrid[:360,:640]
        rgb=np.stack([(xx+i*31)%256,(yy*2+i*17)%256,((xx+yy)//3)%256],axis=-1).astype(np.uint8)
        mask=((xx//100+yy//90+i)%6).astype(np.uint8); mask[:12]=255; mask[100:110,200:260]=255
        Image.fromarray(rgb).save(data/"images"/split/f"{stem}.png"); Image.fromarray(mask).save(data/"masks_coarse"/split/f"{stem}.png")
    (data/"splits"/f"{split}.txt").write_text("\n".join(stems)+"\n")
cmd=[sys.executable,str(Path(__file__).with_name("overfit_tiny.py")),"--dataset",str(data),"--epochs","1","--classes","6","--max-samples","10","--output",str(root/"run")]
raise SystemExit(subprocess.call(cmd))
