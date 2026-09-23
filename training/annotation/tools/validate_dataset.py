from __future__ import annotations

from collections import Counter, defaultdict
import hashlib
from pathlib import Path

import numpy as np
from PIL import Image

from common import (COARSE_VALUES, FINE_VALUES, image_info, parser, read_csv,
                    write_csv, write_json)

FINE_NAMES = {0: "unknown_background", 1: "traversable_ground", 2: "hill_candidate",
              3: "crater_candidate", 4: "step_candidate", 5: "rover_red",
              6: "rover_blue", 7: "rover_yellow", 255: "ignore"}
COARSE_NAMES = {0: "unknown_background", 1: "traversable_ground", 2: "hill_candidate",
                3: "crater_candidate", 4: "step_candidate", 5: "rover", 255: "ignore"}
ISSUE_FIELDS = ["sample_id", "severity", "check", "message"]


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    args = parser("自动校验生成的数据集").parse_args()
    root = args.root.resolve()
    manifest_path = root / "dataset_generated/manifests/all_samples.csv"
    if not manifest_path.exists():
        raise SystemExit("缺少 all_samples.csv")
    manifest = read_csv(manifest_path)
    generation_path = root / "dataset_generated/reports/mask_generation.csv"
    generation = {r["sample_id"]: r for r in read_csv(generation_path)} if generation_path.exists() else {}
    issues: list[dict] = []
    per_sample: dict[str, list[str]] = defaultdict(list)
    pixel_counts = {"fine": Counter(), "coarse": Counter()}
    image_counts = {"fine": Counter(), "coarse": Counter()}

    def issue(sample: str, severity: str, check: str, message: str) -> None:
        issues.append({"sample_id": sample, "severity": severity, "check": check, "message": message})
        per_sample[sample].append(severity)

    id_counts = Counter(r["sample_id"] for r in manifest)
    for sid, count in id_counts.items():
        if count > 1:
            issue(sid, "INVALID", "unique_sample_id", f"sample_id 在 manifest 出现 {count} 次")

    for row in manifest:
        sid, session = row["sample_id"], row["session_id"]
        if row["pair_status"] != "matched":
            issue(sid, "INVALID", "pair_status", f"配对状态为 {row['pair_status']}")
        source_rgb = root / row["rgb_path"] if row["rgb_path"] else None
        source_json = root / row["json_path"] if row["json_path"] else None
        rgb_suffix = source_rgb.suffix.lower() if source_rgb else ".png"
        paths = {
            "rgb": root / f"dataset_generated/rgb/{session}/{sid}{rgb_suffix}",
            "json": root / f"dataset_generated/annotations_json/{session}/{sid}.json",
            "fine": root / f"dataset_generated/masks_fine/{session}/{sid}.png",
            "coarse": root / f"dataset_generated/masks_coarse/{session}/{sid}.png",
        }
        for kind, path in paths.items():
            if not path.is_file():
                issue(sid, "INVALID", "file_correspondence", f"缺少 {kind}: {path.relative_to(root)}")
        if source_rgb and paths["rgb"].is_file() and digest(source_rgb) != digest(paths["rgb"]):
            issue(sid, "INVALID", "rgb_copy_integrity", "生成目录中的 RGB 与原文件字节不一致")
        if source_json and paths["json"].is_file() and digest(source_json) != digest(paths["json"]):
            issue(sid, "INVALID", "json_copy_integrity", "生成目录中的 JSON 与原文件字节不一致")
        if not paths["rgb"].is_file():
            continue
        with Image.open(paths["rgb"]) as im:
            rgb_size = im.size
        for scale in ("fine", "coarse"):
            path = paths[scale]
            if not path.is_file():
                continue
            with Image.open(path) as im:
                mode, size = im.mode, im.size
                arr = np.asarray(im)
            if size != rgb_size:
                issue(sid, "INVALID", f"{scale}_size", f"mask {size} != RGB {rgb_size}")
            if arr.ndim != 2 or mode not in {"L", "P"}:
                issue(sid, "INVALID", f"{scale}_channels", f"mask mode={mode}, shape={arr.shape}")
            if arr.dtype != np.uint8:
                issue(sid, "INVALID", f"{scale}_dtype", f"mask dtype={arr.dtype}")
            allowed = FINE_VALUES if scale == "fine" else COARSE_VALUES
            values, counts = np.unique(arr, return_counts=True)
            invalid_values = sorted(set(map(int, values)) - allowed)
            if invalid_values:
                issue(sid, "INVALID", f"{scale}_values", f"非法像素值 {invalid_values}")
            for value, count in zip(values, counts):
                pixel_counts[scale][int(value)] += int(count)
                image_counts[scale][int(value)] += 1
            ignore_ratio = float(np.mean(arr == 255))
            if ignore_ratio == 1.0:
                issue(sid, "INVALID", f"{scale}_all_ignore", "mask 全部为 255")

        raw_unlabeled = generation.get(sid, {}).get("unlabeled_ratio", "")
        if raw_unlabeled:
            unlabeled_ratio = float(raw_unlabeled)
            if unlabeled_ratio > 0.30:
                issue(sid, "WARNING", "unlabeled_ratio", f"未被任何 shape 覆盖的像素比例 {unlabeled_ratio:.2%} > 30%")

        if row["depth_path"]:
            depth_src = root / row["depth_path"]
            depth_dst = root / f"dataset_generated/depth/{session}/{sid}{depth_src.suffix.lower()}"
            if not depth_dst.is_file():
                issue(sid, "INVALID", "depth_exists", f"缺少 Depth 副本: {depth_dst.relative_to(root)}")
            else:
                src_info, dst_info = image_info(depth_src), image_info(depth_dst)
                if src_info["dtype"] != dst_info["dtype"] or src_info["png_bit_depth"] != dst_info["png_bit_depth"]:
                    issue(sid, "INVALID", "depth_dtype", f"Depth 存储类型改变: {src_info} -> {dst_info}")
                if digest(depth_src) != digest(depth_dst):
                    issue(sid, "INVALID", "depth_copy_integrity", "Depth 副本与原文件字节不一致")
                if (str(src_info["image_width"]), str(src_info["image_height"])) != (row["rgb_width"], row["rgb_height"]):
                    issue(sid, "WARNING", "rgb_depth_resolution", "RGB/Depth 分辨率不一致")

    for scale, names in (("fine", FINE_NAMES), ("coarse", COARSE_NAMES)):
        for value, name in names.items():
            if value != 255 and pixel_counts[scale][value] == 0:
                issue("__dataset__", "WARNING", f"{scale}_class_empty", f"类别 {name}({value}) 像素数为 0")

    status_rows = []
    for sid in id_counts:
        severities = per_sample.get(sid, [])
        status = "INVALID" if "INVALID" in severities else ("WARNING" if "WARNING" in severities else "VALID")
        status_rows.append({"sample_id": sid, "status": status})
    statuses = Counter(r["status"] for r in status_rows)
    pixel_rows, image_rows = [], []
    for scale, names in (("fine", FINE_NAMES), ("coarse", COARSE_NAMES)):
        for value, name in names.items():
            pixel_rows.append({"mask_type": scale, "class_id": value, "class_name": name, "pixel_count": pixel_counts[scale][value]})
            image_rows.append({"mask_type": scale, "class_id": value, "class_name": name, "image_count": image_counts[scale][value]})
    reports = root / "dataset_generated/reports"
    write_csv(reports / "validation_issues.csv", issues, ISSUE_FIELDS)
    write_csv(reports / "class_pixel_statistics.csv", pixel_rows,
              ["mask_type", "class_id", "class_name", "pixel_count"])
    write_csv(reports / "class_image_statistics.csv", image_rows,
              ["mask_type", "class_id", "class_name", "image_count"])
    unmatched = sum(max(0, len(read_csv(reports / name)) if (reports / name).exists() else 0)
                    for name in ("unmatched_rgb.csv", "unmatched_depth.csv", "unmatched_json.csv"))
    invalid_labels = len(read_csv(reports / "invalid_labels.csv")) if (reports / "invalid_labels.csv").exists() else 0
    report = {"sample_counts": {k: statuses.get(k, 0) for k in ("VALID", "WARNING", "INVALID")},
              "unmatched_file_count": unmatched, "invalid_label_sample_count": invalid_labels,
              "issue_count": len(issues), "samples": status_rows}
    write_json(reports / "validation_report.json", report)
    print(f"VALID samples: {statuses.get('VALID', 0)}")
    print(f"WARNING samples: {statuses.get('WARNING', 0)}")
    print(f"INVALID samples: {statuses.get('INVALID', 0)}")
    print("各类别总像素数 (coarse):", dict(sorted(pixel_counts["coarse"].items())))
    print("各类别出现的图片数量 (coarse):", dict(sorted(image_counts["coarse"].items())))
    print(f"未配对文件数量: {unmatched}")
    print(f"非法标签数量: {invalid_labels}")
    return 0 if statuses.get("INVALID", 0) == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
