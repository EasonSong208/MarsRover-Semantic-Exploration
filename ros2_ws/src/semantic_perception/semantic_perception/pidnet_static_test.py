"""Run bounded PIDNet-S inference on a few static RGB images."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from semantic_perception.pidnet_contract import (
    calculate_ratios,
    colorize_prediction,
    make_overlay,
)
from semantic_perception.pidnet_inference import PIDNetInference


SUPPORTED_SUFFIXES = {'.png', '.jpg', '.jpeg'}


def build_parser() -> argparse.ArgumentParser:
    """Build the bounded static-test CLI."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--model-path', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--limit', type=int, default=3)
    parser.add_argument('--device', default='cuda')
    parser.add_argument('--precision', choices=('fp16', 'fp32'), default='fp16')
    return parser


def discover_images(root: Path, limit: int):
    """Return a deterministic bounded image list."""
    if not root.is_dir():
        raise FileNotFoundError(root)
    if limit <= 0:
        raise ValueError('--limit must be positive')
    images = sorted(
        path for path in root.rglob('*')
        if path.is_file() and path.suffix.casefold() in SUPPORTED_SUFFIXES
    )
    if not images:
        raise RuntimeError(f'no supported images found under {root}')
    return images[:limit]


def main() -> int:
    """Load one model and save raw/mask/color/overlay artifacts."""
    args = build_parser().parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    engine = PIDNetInference(
        model_path=args.model_path,
        device=args.device,
        precision=args.precision,
        allow_fp32_fallback=True,
    )

    records = []
    for image_path in discover_images(args.input.resolve(), args.limit):
        bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError(f'failed to read image: {image_path}')
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        mask, inference_ms = engine.predict(rgb)
        values = sorted(int(value) for value in np.unique(mask))
        if len(values) <= 1:
            raise RuntimeError(
                f'single-class prediction is rejected: {image_path} values={values}')

        stem = image_path.stem
        raw_path = output / f'{stem}_raw.png'
        mask_path = output / f'{stem}_mask.png'
        color_path = output / f'{stem}_color.png'
        overlay_path = output / f'{stem}_overlay.png'
        color = colorize_prediction(mask)
        overlay = make_overlay(rgb, mask)
        if not cv2.imwrite(str(raw_path), bgr):
            raise RuntimeError(f'failed to write {raw_path}')
        if not cv2.imwrite(str(mask_path), mask):
            raise RuntimeError(f'failed to write {mask_path}')
        if not cv2.imwrite(
            str(color_path), cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
        ):
            raise RuntimeError(f'failed to write {color_path}')
        if not cv2.imwrite(
            str(overlay_path), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
        ):
            raise RuntimeError(f'failed to write {overlay_path}')
        records.append({
            'input': str(image_path),
            'raw': str(raw_path),
            'mask': str(mask_path),
            'color': str(color_path),
            'overlay': str(overlay_path),
            'mask_shape': list(mask.shape),
            'mask_values': values,
            'inference_ms': inference_ms,
            'ratios': calculate_ratios(mask),
        })

    report = {
        'status': 'PASS',
        'model': engine.metadata,
        'input_count': len(records),
        'precision': engine.precision,
        'fp16_fallback_reason': engine.last_fallback_reason,
        'records': records,
    }
    report_path = output / 'static_test_report.json'
    report_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, separators=(',', ':')))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
