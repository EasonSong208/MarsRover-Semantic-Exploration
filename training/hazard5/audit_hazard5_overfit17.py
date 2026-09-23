#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from PIL import Image

ALLOWED = {0, 1, 2, 3, 4, 255}
CLASS_NAMES = {0: "other", 1: "hill_candidate", 2: "crater_candidate",
               3: "step_candidate", 4: "rover", 255: "ignore"}


def index_pngs(root: Path) -> dict[str, list[Path]]:
    result: dict[str, list[Path]] = defaultdict(list)
    for path in root.rglob("*"):
        if path.is_file() and path.suffix.lower() == ".png":
            result[path.stem].append(path)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--report-dir", type=Path, required=True)
    args = parser.parse_args()
    dataset = args.dataset.resolve()
    report_dir = args.report_dir.resolve()
    manifest_path = dataset / "manifests/all_samples.csv"
    with manifest_path.open("r", encoding="utf-8-sig", newline="") as handle:
        manifest = list(csv.DictReader(handle))
    rgb_index = index_pngs(dataset / "rgb")
    mask_index = index_pngs(dataset / "masks_hazard5")
    manifest_ids = [row["sample_id"] for row in manifest]
    duplicate_manifest_ids = sorted(sid for sid, count in Counter(manifest_ids).items() if count != 1)
    issues: list[dict] = []
    pairs: list[dict] = []
    class_pixels = Counter()
    class_images = Counter()

    def problem(sample_id: str, message: str) -> None:
        issues.append({"sample_id": sample_id, "message": message})

    for row in manifest:
        sid = row["sample_id"]
        rgb_candidates = rgb_index.get(sid, [])
        mask_candidates = mask_index.get(sid, [])
        if len(rgb_candidates) != 1:
            problem(sid, f"RGB candidates={len(rgb_candidates)}: {[str(x) for x in rgb_candidates]}")
        if len(mask_candidates) != 1:
            problem(sid, f"mask candidates={len(mask_candidates)}: {[str(x) for x in mask_candidates]}")
        if len(rgb_candidates) != 1 or len(mask_candidates) != 1:
            continue
        rgb_path, mask_path = rgb_candidates[0], mask_candidates[0]
        sample_issues: list[str] = []
        with Image.open(rgb_path) as image:
            rgb_mode, rgb_size = image.mode, image.size
            rgb = np.asarray(image)
        with Image.open(mask_path) as image:
            mask_mode, mask_size = image.mode, image.size
            mask = np.asarray(image)
        if rgb_size != (640, 360):
            sample_issues.append(f"RGB size={rgb_size}")
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            sample_issues.append(f"RGB mode/shape={rgb_mode}/{rgb.shape}")
        if mask_size != (640, 360):
            sample_issues.append(f"mask size={mask_size}")
        if mask.ndim != 2 or mask_mode != "L":
            sample_issues.append(f"mask mode/shape={mask_mode}/{mask.shape}")
        if mask.dtype != np.uint8:
            sample_issues.append(f"mask dtype={mask.dtype}")
        actual = set(map(int, np.unique(mask)))
        if not actual <= ALLOWED:
            sample_issues.append(f"illegal mask values={sorted(actual - ALLOWED)}")
        if rgb.shape[:2] != mask.shape:
            sample_issues.append(f"RGB/mask mismatch={rgb.shape[:2]}/{mask.shape}")
        padded_rgb = np.pad(rgb, ((12, 12), (0, 0), (0, 0)), constant_values=0)
        padded_mask = np.pad(mask, ((12, 12), (0, 0)), constant_values=255)
        padding_ok = bool(padded_rgb.shape == (384, 640, 3) and padded_mask.shape == (384, 640)
                          and np.array_equal(padded_rgb[12:372], rgb)
                          and np.array_equal(padded_mask[12:372], mask)
                          and np.all(padded_mask[:12] == 255) and np.all(padded_mask[372:] == 255))
        if not padding_ok:
            sample_issues.append("symmetric padding alignment check failed")
        counts = {value: int(np.sum(mask == value)) for value in CLASS_NAMES}
        for value, count in counts.items():
            class_pixels[value] += count
            class_images[value] += int(count > 0)
        pairs.append({
            "sample_id": sid, "session_id": row["session_id"],
            "rgb_path": str(rgb_path), "mask_path": str(mask_path),
            "rgb_size": list(rgb_size), "rgb_mode": rgb_mode,
            "mask_size": list(mask_size), "mask_mode": mask_mode,
            "mask_dtype": str(mask.dtype), "mask_values": sorted(actual),
            "padding_alignment_ok": padding_ok, "status": "INVALID" if sample_issues else "VALID",
            "issues": sample_issues, "class_pixels": {str(k): v for k, v in counts.items()},
        })
        for message in sample_issues:
            problem(sid, message)

    extra_rgb = sorted(set(rgb_index) - set(manifest_ids))
    extra_masks = sorted(set(mask_index) - set(manifest_ids))
    missing_target_classes = [CLASS_NAMES[value] for value in range(1, 5) if class_pixels[value] == 0]
    report = {
        "dataset": str(dataset), "pair_method": "manifest sample_id -> unique recursive PNG stem",
        "manifest_rows": len(manifest), "rgb_png_count": sum(map(len, rgb_index.values())),
        "mask_png_count": sum(map(len, mask_index.values())), "valid_pairs": sum(x["status"] == "VALID" for x in pairs),
        "invalid_pairs": sum(x["status"] == "INVALID" for x in pairs),
        "duplicate_manifest_ids": duplicate_manifest_ids, "extra_rgb_ids": extra_rgb,
        "extra_mask_ids": extra_masks, "issues": issues,
        "allowed_mask_values": sorted(ALLOWED), "class_names": {str(k): v for k, v in CLASS_NAMES.items()},
        "class_pixels": {str(k): class_pixels[k] for k in CLASS_NAMES},
        "class_images": {str(k): class_images[k] for k in CLASS_NAMES},
        "missing_target_classes": missing_target_classes,
        "all_17_participate": len(manifest) == 17 and len(pairs) == 17 and not issues and not extra_rgb and not extra_masks,
        "pairs": pairs,
    }
    report_dir.mkdir(parents=True, exist_ok=True)
    (report_dir / "hazard5_overfit17_pair_audit.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    with (report_dir / "hazard5_overfit17_pairs.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = ["sample_id", "session_id", "rgb_path", "mask_path", "status", "mask_values", "padding_alignment_ok"]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for pair in pairs:
            writer.writerow({k: pair[k] for k in fields})
    print(json.dumps({k: report[k] for k in (
        "manifest_rows", "rgb_png_count", "mask_png_count", "valid_pairs", "invalid_pairs",
        "class_pixels", "class_images", "all_17_participate")}, ensure_ascii=False, indent=2))
    return 0 if report["all_17_participate"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
