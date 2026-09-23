from __future__ import annotations

import json
import shutil
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from common import FINE_LABELS, FINE_VALUES, parser, read_csv, write_csv

INVALID_FIELDS = ["sample_id", "json_path", "invalid_labels"]
UNSUPPORTED_FIELDS = ["sample_id", "json_path", "shape_index", "label", "shape_type"]
GEN_FIELDS = ["sample_id", "session_id", "status", "fine_mask", "coarse_mask", "unlabeled_ratio", "ignore_ratio", "message"]
TIER = {
    "unknown_background": 0, "traversable_ground": 1, "hill_candidate": 2,
    "crater_candidate": 3, "step_candidate": 4,
    "rover_red": 5, "rover_blue": 5, "rover_yellow": 5,
    "__ignore__": 6, "ignore": 6,
}
COLORS = {
    0: (128, 128, 128), 1: (255, 255, 255), 2: (40, 110, 255),
    3: (255, 50, 50), 4: (255, 220, 20), 5: (30, 220, 80),
    255: (180, 40, 210),
}


def shape_mask(width: int, height: int, shape: dict) -> np.ndarray:
    temp = Image.new("1", (width, height), 0)
    draw = ImageDraw.Draw(temp)
    points = [tuple(float(v) for v in p[:2]) for p in shape.get("points", [])]
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
    return np.asarray(temp, dtype=bool)


def render(data: dict, width: int, height: int) -> tuple[np.ndarray, list[dict], np.ndarray]:
    coverage = {name: np.zeros((height, width), dtype=bool) for name in FINE_LABELS}
    unsupported: list[dict] = []
    for index, shape in enumerate(data.get("shapes", [])):
        label = shape.get("label")
        kind = shape.get("shape_type", "polygon")
        if kind not in {"polygon", "rectangle"}:
            unsupported.append({"shape_index": index, "label": label, "shape_type": kind})
            continue
        try:
            coverage[label] |= shape_mask(width, height, shape)
        except ValueError as exc:
            unsupported.append({"shape_index": index, "label": label, "shape_type": f"{kind}: {exc}"})
    if unsupported:
        return np.full((height, width), 255, dtype=np.uint8), unsupported, np.zeros((height, width), dtype=bool)

    result = np.full((height, width), 255, dtype=np.uint8)
    for tier in range(7):
        names = [name for name, value in TIER.items() if value == tier]
        masks = [(name, coverage[name]) for name in names if coverage[name].any()]
        for name, mask in masks:
            result[mask] = FINE_LABELS[name]
        # Same-tier different labels have no defined winner; make conflict explicit ignore.
        if len(masks) > 1:
            count = np.zeros((height, width), dtype=np.uint8)
            for _, mask in masks:
                count += mask
            result[count > 1] = 255
    annotated = np.zeros((height, width), dtype=bool)
    for mask in coverage.values():
        annotated |= mask
    return result, [], annotated


def overlay(rgb_path: Path, coarse: np.ndarray, output: Path) -> None:
    with Image.open(rgb_path) as im:
        rgb = np.asarray(im.convert("RGB"), dtype=np.float32)
    color = np.zeros_like(rgb)
    for value, triplet in COLORS.items():
        color[coarse == value] = triplet
    blended = np.clip(rgb * 0.6 + color * 0.4, 0, 255).astype(np.uint8)
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(blended, mode="RGB").save(output)


def main() -> int:
    args = parser("将 Labelme JSON 转为 fine/coarse mask 并生成复制与 overlay").parse_args()
    root = args.root.resolve()
    manifest_path = root / "dataset_generated/manifests/all_samples.csv"
    if not manifest_path.exists():
        raise SystemExit("请先运行 build_manifest.py")
    rows = read_csv(manifest_path)
    invalid_labels: list[dict] = []
    unsupported_rows: list[dict] = []
    generated: list[dict] = []

    for row in rows:
        sid, session = row["sample_id"], row["session_id"]
        if not row["rgb_path"] or not row["json_path"] or row["pair_status"] == "ambiguous":
            generated.append({"sample_id": sid, "session_id": session, "status": "skipped",
                              "fine_mask": "", "coarse_mask": "", "unlabeled_ratio": "", "ignore_ratio": "",
                              "message": "缺少唯一 RGB/JSON 配对"})
            continue
        rgb_src, json_src = root / row["rgb_path"], root / row["json_path"]
        try:
            data = json.loads(json_src.read_text(encoding="utf-8"))
        except Exception as exc:
            generated.append({"sample_id": sid, "session_id": session, "status": "failed", "fine_mask": "",
                              "coarse_mask": "", "unlabeled_ratio": "", "ignore_ratio": "", "message": f"JSON 读取失败: {exc}"})
            continue
        labels = {str(s.get("label")) for s in data.get("shapes", [])}
        illegal = sorted(labels - set(FINE_LABELS))
        if illegal:
            invalid_labels.append({"sample_id": sid, "json_path": row["json_path"], "invalid_labels": "|".join(illegal)})
            generated.append({"sample_id": sid, "session_id": session, "status": "failed", "fine_mask": "",
                              "coarse_mask": "", "unlabeled_ratio": "", "ignore_ratio": "", "message": "非法标签: " + "|".join(illegal)})
            continue
        try:
            with Image.open(rgb_src) as im:
                width, height = im.size
            if data.get("imageWidth") not in (None, width) or data.get("imageHeight") not in (None, height):
                raise ValueError(f"JSON 尺寸 {data.get('imageWidth')}x{data.get('imageHeight')} 与 RGB {width}x{height} 不一致")
            fine, unsupported, annotated = render(data, width, height)
        except Exception as exc:
            generated.append({"sample_id": sid, "session_id": session, "status": "failed", "fine_mask": "",
                              "coarse_mask": "", "unlabeled_ratio": "", "ignore_ratio": "", "message": str(exc)})
            continue
        if unsupported:
            for item in unsupported:
                unsupported_rows.append({"sample_id": sid, "json_path": row["json_path"], **item})
            generated.append({"sample_id": sid, "session_id": session, "status": "failed", "fine_mask": "",
                              "coarse_mask": "", "unlabeled_ratio": "", "ignore_ratio": "", "message": "存在不支持或无效的 shape"})
            continue
        if not set(np.unique(fine)).issubset(FINE_VALUES):
            raise RuntimeError(f"内部错误：{sid} fine mask 出现非法像素值")
        coarse = fine.copy()
        coarse[np.isin(coarse, [5, 6, 7])] = 5
        out = root / "dataset_generated"
        fine_path = out / "masks_fine" / session / f"{sid}.png"
        coarse_path = out / "masks_coarse" / session / f"{sid}.png"
        rgb_dst = out / "rgb" / session / f"{sid}{rgb_src.suffix.lower()}"
        json_dst = out / "annotations_json" / session / f"{sid}.json"
        overlay_path = out / "overlays" / session / f"{sid}_overlay.png"
        for p in (fine_path, coarse_path, rgb_dst, json_dst):
            p.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(fine, mode="L").save(fine_path)
        Image.fromarray(coarse, mode="L").save(coarse_path)
        shutil.copy2(rgb_src, rgb_dst)
        shutil.copy2(json_src, json_dst)
        if row["depth_path"]:
            depth_src = root / row["depth_path"]
            depth_dst = out / "depth" / session / f"{sid}{depth_src.suffix.lower()}"
            depth_dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(depth_src, depth_dst)
        overlay(rgb_src, coarse, overlay_path)
        generated.append({"sample_id": sid, "session_id": session, "status": "generated",
                          "fine_mask": fine_path.relative_to(root).as_posix(),
                          "coarse_mask": coarse_path.relative_to(root).as_posix(),
                          "unlabeled_ratio": f"{float(np.mean(~annotated)):.8f}",
                          "ignore_ratio": f"{float(np.mean(fine == 255)):.8f}", "message": ""})

    reports = root / "dataset_generated/reports"
    write_csv(reports / "invalid_labels.csv", invalid_labels, INVALID_FIELDS)
    write_csv(reports / "unsupported_shapes.csv", unsupported_rows, UNSUPPORTED_FIELDS)
    write_csv(reports / "mask_generation.csv", generated, GEN_FIELDS)
    print(f"mask 成功生成: {sum(r['status'] == 'generated' for r in generated)}")
    print(f"非法标签样本: {len(invalid_labels)}; 不支持 shape: {len(unsupported_rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
