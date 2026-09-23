#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np
from PIL import Image
from mars_pidnet.datasets.mars_dataset import pad_640x360

COLORS = np.array([[50,50,50],[100,80,50],[180,120,50],[70,20,20],[200,180,120],[180,0,0],[0,0,180],[220,190,0]], dtype=np.uint8)
parser = argparse.ArgumentParser()
parser.add_argument("image", type=Path); parser.add_argument("mask", type=Path); parser.add_argument("output", type=Path)
args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
rgb = np.asarray(Image.open(args.image).convert("RGB")); mask = np.asarray(Image.open(args.mask))
prgb, pmask = pad_640x360(rgb, mask)
color = np.zeros((*pmask.shape, 3), np.uint8); valid = pmask != 255; color[valid] = COLORS[pmask[valid]]
overlay = prgb.copy(); overlay[valid] = (0.55 * prgb[valid] + 0.45 * color[valid]).astype(np.uint8)
Image.fromarray(rgb).save(args.output / "original_image.png"); Image.fromarray(mask).save(args.output / "original_mask.png")
Image.fromarray(prgb).save(args.output / "padded_image.png"); Image.fromarray(pmask).save(args.output / "padded_mask.png")
Image.fromarray(overlay).save(args.output / "padded_overlay.png")
