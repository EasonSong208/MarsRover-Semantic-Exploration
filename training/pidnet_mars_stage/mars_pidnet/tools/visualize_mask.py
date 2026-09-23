#!/usr/bin/env python3
import argparse
import numpy as np
from PIL import Image
COLORS = np.array([[50,50,50],[100,80,50],[180,120,50],[70,20,20],[200,180,120],[180,0,0],[0,0,180],[220,190,0]], dtype=np.uint8)

parser = argparse.ArgumentParser(); parser.add_argument("mask"); parser.add_argument("output"); args = parser.parse_args()
mask = np.asarray(Image.open(args.mask)); out = np.zeros((*mask.shape,3), np.uint8); valid = mask != 255
if np.any(mask[valid] >= len(COLORS)): raise ValueError("mask contains unknown classes")
out[valid] = COLORS[mask[valid]]; Image.fromarray(out).save(args.output)
