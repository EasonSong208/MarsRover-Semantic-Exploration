#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np
from PIL import Image
from mars_pidnet.datasets.mars_dataset import ALLOWED_FINE, FINE_TO_COARSE

parser = argparse.ArgumentParser()
parser.add_argument("input", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
files = [args.input] if args.input.is_file() else sorted(args.input.rglob("*.png"))
for src in files:
    mask = np.asarray(Image.open(src))
    if mask.ndim != 2 or mask.dtype != np.uint8:
        raise ValueError(f"not a uint8 single-channel mask: {src}")
    illegal = set(np.unique(mask).tolist()) - ALLOWED_FINE
    if illegal:
        raise ValueError(f"illegal values {sorted(illegal)}: {src}")
    dst = args.output if args.input.is_file() else args.output / src.relative_to(args.input)
    dst.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(FINE_TO_COARSE[mask], mode="L").save(dst)
    print(f"{src} -> {dst}")
