from __future__ import annotations

import json
import shutil
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from common import parser, read_csv, write_csv, write_json

LABEL_TO_ID = {
    "unknown_background": 0,
    "traversable_ground": 0,
    "hill_candidate": 1,
    "crater_candidate": 2,
    "step_candidate": 3,
    "rover_red": 4,
    "rover_blue": 4,
    "rover_yellow": 4,
    "__ignore__": 255,
    "ignore": 255,
}
CLASS_NAMES = {
    0: "other", 1: "hill_candidate", 2: "crater_candidate",
    3: "step_candidate", 4: "rover", 255: "ignore",
}
ALLOWED_VALUES = set(CLASS_NAMES)
PRIORITY_GROUPS = [
    (0, {"unknown_background", "traversable_ground"}),
    (1, {"hill_candidate"}),
    (2, {"crater_candidate"}),
    (3, {"step_candidate"}),
    (4, {"rover_red", "rover_blue", "rover_yellow"}),
    (255, {"__ignore__", "ignore"}),
]
COLORS = {
    1: (40, 110, 255), 2: (255, 50, 50), 3: (255, 220, 20),
    4: (30, 220, 80), 255: (180, 40, 210),
}


def rasterize(width: int, height: int, shape: dict) -> np.ndarray:
    canvas = Image.new("1", (width, height), 0)
    draw = ImageDraw.Draw(canvas)
    points = [tuple(float(v) for v in point[:2]) for point in shape.get("points", [])]
    kind = shape.get("shape_type", "polygon")
    if kind == "polygon":
        if len(points) < 3:
            raise ValueError("polygon 少于 3 个点")
        draw.polygon(points, fill=1)
    elif kind == "rectangle":
        if len(points) != 2:
            raise ValueError("rectangle 必须有 2 个点")
        x1, x2 = sorted((points[0][0], points[1][0]))
        y1, y2 = sorted((points[0][1], points[1][1]))
        draw.rectangle((x1, y1, x2, y2), fill=1)
    else:
        raise ValueError(f"不支持 shape_type: {kind}")
    return np.asarray(canvas, dtype=bool)


def render(data: dict, width: int, height: int) -> np.ndarray:
    coverage = {label: np.zeros((height, width), dtype=bool) for label in LABEL_TO_ID}
    for index, shape in enumerate(data.get("shapes", [])):
        label = str(shape.get("label"))
        if label not in LABEL_TO_ID:
            raise ValueError(f"shape {index} 含非法标签: {label}")
        try:
            coverage[label] |= rasterize(width, height, shape)
        except ValueError as exc:
            raise ValueError(f"shape {index}: {exc}") from exc

    # Uncovered pixels and explicit non-target labels are other=0.
    mask = np.zeros((height, width), dtype=np.uint8)
    for value, labels in PRIORITY_GROUPS:
        union = np.zeros((height, width), dtype=bool)
        for label in labels:
            union |= coverage[label]
        mask[union] = value
    return mask


def make_overlay(rgb_path: Path, mask: np.ndarray, output: Path) -> None:
    with Image.open(rgb_path) as image:
        rgb = np.asarray(image.convert("RGB"), dtype=np.float32)
    result = rgb.copy()
    for value, color in COLORS.items():
        selected = mask == value
        result[selected] = rgb[selected] * 0.6 + np.asarray(color, dtype=np.float32) * 0.4
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.clip(result, 0, 255).astype(np.uint8), mode="RGB").save(output)


def choose_tiny(root: Path, candidates: list[dict], limit: int = 10) -> dict:
    frequency = Counter()
    for item in candidates:
        frequency.update(item["target_classes"])
    remaining = list(candidates)
    negatives = [item for item in remaining if not item["target_classes"]]
    selected: list[dict] = []
    covered: set[int] = set()
    combos: set[tuple[int, ...]] = set()
    sessions: set[str] = set()

    # Reserve a real pure-negative sample first when one exists.
    if negatives:
        negative = max(negatives, key=lambda x: (x["counts"][0], x["sample_id"]))
        selected.append(negative)
        remaining.remove(negative)
        sessions.add(negative["session_id"])
        combos.add(())

    while remaining and len(selected) < limit:
        def score(item: dict) -> tuple:
            classes = item["target_classes"]
            combo = tuple(sorted(classes))
            new_classes = classes - covered
            rarity = sum(1.0 / frequency[value] for value in classes if frequency[value])
            diversity = int(item["session_id"] not in sessions) + int(combo not in combos)
            new_pixels = sum(item["counts"][value] for value in new_classes)
            return (len(new_classes), rarity, diversity, len(classes), new_pixels, item["sample_id"])
        best = max(remaining, key=score)
        remaining.remove(best)
        selected.append(best)
        covered |= best["target_classes"]
        sessions.add(best["session_id"])
        combos.add(tuple(sorted(best["target_classes"])))

    out = root / "dataset_generated/tiny_overfit_hazard5"
    for subdir in ("rgb", "masks_hazard5", "overlays_hazard5"):
        (out / subdir).mkdir(parents=True, exist_ok=True)
    csv_rows = []
    for item in selected:
        rgb_src = item["rgb_path"]
        mask_src = item["mask_path"]
        overlay_src = item["overlay_path"]
        shutil.copy2(rgb_src, out / "rgb" / rgb_src.name)
        shutil.copy2(mask_src, out / "masks_hazard5" / mask_src.name)
        shutil.copy2(overlay_src, out / "overlays_hazard5" / overlay_src.name)
        csv_rows.append({
            "sample_id": item["sample_id"], "session_id": item["session_id"],
            "is_pure_negative": not item["target_classes"],
            "classes_present": "|".join(CLASS_NAMES[v] for v in sorted(item["target_classes"])),
            **{f"pixels_{CLASS_NAMES[value]}": item["counts"][value] for value in CLASS_NAMES},
        })
    fields = ["sample_id", "session_id", "is_pure_negative", "classes_present"] + [
        f"pixels_{CLASS_NAMES[value]}" for value in CLASS_NAMES
    ]
    write_csv(out / "tiny_overfit_hazard5.csv", csv_rows, fields)
    selection_report = {
        "selected_count": len(selected),
        "selected_samples": [item["sample_id"] for item in selected],
        "target_classes_covered": sorted(covered),
        "pure_negative_available": bool(negatives),
        "pure_negative_included": any(not item["target_classes"] for item in selected),
        "requirement_note": (
            "已包含真实纯负样本。" if negatives else
            "当前 17 张图片全部包含至少一种 1～4 目标类别，无法在不篡改标注的情况下加入纯负样本；需补充一张确实没有山、坑、坎和小车的图片。"
        ),
    }
    write_json(out / "selection_report.json", selection_report)
    return selection_report


def main() -> int:
    args = parser("生成独立 hazard5 mask、overlay、校验报告和 tiny-overfit 变体").parse_args()
    root = args.root.resolve()
    manifest_path = root / "dataset_generated/manifests/all_samples.csv"
    if not manifest_path.exists():
        raise SystemExit("缺少 all_samples.csv，请先运行 inspect_dataset.py 和 build_manifest.py")
    manifest = read_csv(manifest_path)
    generated: list[dict] = []
    failures: list[dict] = []

    for row in manifest:
        sid, session = row["sample_id"], row["session_id"]
        if row["pair_status"] != "matched" or not row["rgb_path"] or not row["json_path"]:
            failures.append({"sample_id": sid, "reason": "缺少完整且唯一的 RGB/Depth/JSON 配对"})
            continue
        rgb_path = root / row["rgb_path"]
        json_path = root / row["json_path"]
        try:
            data = json.loads(json_path.read_text(encoding="utf-8"))
            with Image.open(rgb_path) as image:
                width, height = image.size
            if data.get("imageWidth") not in (None, width) or data.get("imageHeight") not in (None, height):
                raise ValueError("Labelme JSON 与 RGB 尺寸不一致")
            mask = render(data, width, height)
            mask_path = root / f"dataset_generated/masks_hazard5/{session}/{sid}.png"
            overlay_path = root / f"dataset_generated/overlays_hazard5/{session}/{sid}_overlay.png"
            mask_path.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(mask, mode="L").save(mask_path)
            make_overlay(rgb_path, mask, overlay_path)
            generated.append({"sample_id": sid, "session_id": session, "rgb_path": rgb_path,
                              "mask_path": mask_path, "overlay_path": overlay_path})
        except Exception as exc:
            failures.append({"sample_id": sid, "reason": f"{type(exc).__name__}: {exc}"})

    class_pixels = Counter()
    class_images = Counter()
    samples: list[dict] = []
    valid_items: list[dict] = []
    for item in generated:
        sid = item["sample_id"]
        sample_issues = []
        with Image.open(item["mask_path"]) as image:
            mode, size = image.mode, image.size
            mask = np.asarray(image)
        with Image.open(item["rgb_path"]) as image:
            rgb_size = image.size
        if mask.ndim != 2 or mode != "L":
            sample_issues.append(f"mask 必须为单通道 L，实际 mode={mode}, shape={mask.shape}")
        if mask.dtype != np.uint8:
            sample_issues.append(f"mask dtype 必须为 uint8，实际 {mask.dtype}")
        if size != rgb_size:
            sample_issues.append(f"mask 尺寸 {size} 与 RGB {rgb_size} 不一致")
        values, counts = np.unique(mask, return_counts=True)
        illegal = sorted(set(map(int, values)) - ALLOWED_VALUES)
        if illegal:
            sample_issues.append(f"非法像素值: {illegal}")
        counts_by_class = {value: int(np.sum(mask == value)) for value in CLASS_NAMES}
        for value, count in counts_by_class.items():
            class_pixels[value] += count
            if count:
                class_images[value] += 1
        target_classes = {value for value in range(1, 5) if counts_by_class[value] > 0}
        pure_negative = not target_classes
        sample_record = {
            "sample_id": sid, "session_id": item["session_id"],
            "status": "INVALID" if sample_issues else "VALID", "issues": sample_issues,
            "actual_values": sorted(map(int, values)), "explicit_ignore_pixels": counts_by_class[255],
            "explicit_ignore_ratio": counts_by_class[255] / mask.size,
            "target_classes_present": sorted(target_classes), "is_pure_negative": pure_negative,
        }
        samples.append(sample_record)
        if not sample_issues:
            valid_items.append({**item, "counts": counts_by_class, "target_classes": target_classes})

    missing_targets = [CLASS_NAMES[value] for value in range(1, 5) if class_pixels[value] == 0]
    pure_negative_samples = [sample["sample_id"] for sample in samples if sample["is_pure_negative"]]
    total_pixels = sum(class_pixels.values())
    tiny_report = choose_tiny(root, valid_items)
    report = {
        "class_definition": {str(value): name for value, name in CLASS_NAMES.items()},
        "allowed_values": sorted(ALLOWED_VALUES), "manifest_samples": len(manifest),
        "successful_samples": len(generated) - sum(sample["status"] == "INVALID" for sample in samples),
        "failed_samples": failures, "invalid_sample_count": sum(sample["status"] == "INVALID" for sample in samples),
        "actual_values": sorted(value for value, count in class_pixels.items() if count > 0),
        "explicit_ignore_pixels": class_pixels[255],
        "explicit_ignore_ratio": class_pixels[255] / total_pixels if total_pixels else 0.0,
        "pure_negative_samples": pure_negative_samples,
        "missing_target_classes": missing_targets,
        "annotation_completeness_warning": (
            "other 仅表示非目标区域，绝不表示可通行。未标出的山、坑、坎或小车会错误成为 other；人工标注必须穷尽所有可见目标。"
        ),
        "tiny_overfit": tiny_report, "samples": samples,
    }
    reports = root / "dataset_generated/reports"
    write_json(reports / "hazard5_validation.json", report)
    write_csv(reports / "hazard5_class_statistics.csv", [
        {"class_id": value, "class_name": name, "pixel_count": class_pixels[value],
         "image_count": class_images[value]}
        for value, name in CLASS_NAMES.items()
    ], ["class_id", "class_name", "pixel_count", "image_count"])
    generated_list = reports / "generated_files.txt"
    generated_root = root / "dataset_generated"
    all_files = sorted(path.relative_to(root).as_posix() for path in generated_root.rglob("*")
                       if path.is_file() and path != generated_list)
    all_files.append(generated_list.relative_to(root).as_posix())
    generated_list.write_text("\n".join(all_files) + "\n", encoding="utf-8")
    print(f"hazard5 成功: {report['successful_samples']}/{len(manifest)}; INVALID: {report['invalid_sample_count']}; 转换失败: {len(failures)}")
    print("实际像素值:", report["actual_values"])
    print("各类像素数:", dict(class_pixels))
    print("各类出现图片数:", dict(class_images))
    print(f"显式 ignore 比例: {report['explicit_ignore_ratio']:.8%}")
    print("纯负样本:", pure_negative_samples if pure_negative_samples else "无")
    print("tiny-overfit hazard5:", tiny_report["selected_samples"])
    if not tiny_report["pure_negative_included"]:
        print("提醒:", tiny_report["requirement_note"])
    return 0 if not failures and report["invalid_sample_count"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
