from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
import struct
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from PIL import Image

RGB_WORDS = {"rgb", "color", "colour", "image", "images"}
DEPTH_WORDS = {"depth", "depth_raw", "aligned_depth", "depth_reference"}
IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
FINE_LABELS = {
    "unknown_background": 0,
    "traversable_ground": 1,
    "hill_candidate": 2,
    "crater_candidate": 3,
    "step_candidate": 4,
    "rover_red": 5,
    "rover_blue": 6,
    "rover_yellow": 7,
    "__ignore__": 255,
    "ignore": 255,
}
FINE_VALUES = set(FINE_LABELS.values())
COARSE_VALUES = {0, 1, 2, 3, 4, 5, 255}
PRIORITY = [
    "unknown_background", "traversable_ground", "hill_candidate",
    "crater_candidate", "step_candidate", "rover_red", "rover_blue",
    "rover_yellow", "__ignore__", "ignore",
]
INVENTORY_FIELDS = [
    "relative_path", "filename", "stem", "suffix", "parent_folder",
    "file_size", "image_width", "image_height", "image_mode", "dtype",
    "channels", "candidate_type", "classification_note", "png_bit_depth",
]


def parser(description: str) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=description)
    p.add_argument(
        "--root", type=Path,
        default=Path("/mnt/d/mars_annotation") if Path("/mnt/d/mars_annotation").is_dir()
        else Path(__file__).resolve().parents[1],
        help="数据根目录（WSL 示例：/mnt/d/mars_annotation；Windows 示例：D:\\mars_annotation）",
    )
    return p


def ensure_output_dirs(root: Path) -> None:
    for rel in (
        "dataset_generated/reports", "dataset_generated/manifests",
        "dataset_generated/rgb", "dataset_generated/depth",
        "dataset_generated/annotations_json", "dataset_generated/masks_fine",
        "dataset_generated/masks_coarse", "dataset_generated/overlays",
    ):
        (root / rel).mkdir(parents=True, exist_ok=True)


def relative_text(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def write_csv(path: Path, rows: Iterable[dict[str, Any]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def png_header(path: Path) -> tuple[int | None, int | None]:
    try:
        with path.open("rb") as f:
            header = f.read(29)
        if header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR":
            return None, None
        bit_depth, color_type = struct.unpack(">BB", header[24:26])
        channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}.get(color_type)
        return bit_depth, channels
    except OSError:
        return None, None


def image_info(path: Path) -> dict[str, Any]:
    info: dict[str, Any] = {
        "image_width": "", "image_height": "", "image_mode": "",
        "dtype": "", "channels": "", "png_bit_depth": "",
    }
    try:
        bit_depth, header_channels = png_header(path) if path.suffix.lower() == ".png" else (None, None)
        with Image.open(path) as im:
            width, height = im.size
            arr = np.asarray(im)
            channels = header_channels or (1 if arr.ndim == 2 else arr.shape[2])
            dtype = str(arr.dtype)
            # Pillow expands unsigned 16-bit PNG to mode I/int32. Report storage dtype.
            if bit_depth == 16 and channels == 1:
                dtype = "uint16"
            info.update(image_width=width, image_height=height, image_mode=im.mode,
                        dtype=dtype, channels=channels, png_bit_depth=bit_depth or "")
    except Exception as exc:  # corrupt/unsupported files remain inventory entries
        info["read_error"] = f"{type(exc).__name__}: {exc}"
    return info


def _words(path: Path) -> set[str]:
    words: set[str] = set()
    for part in path.parts:
        low = part.lower()
        words.add(low)
        words.update(x for x in re.split(r"[^a-z0-9]+", low) if x)
    return words


def classify(path: Path, info: dict[str, Any], under_annotations: bool = False) -> tuple[str, str]:
    if under_annotations and path.suffix.lower() == ".json":
        return "labelme_json", "annotations_json"
    if path.suffix.lower() not in IMAGE_SUFFIXES:
        return "unknown", "unsupported_suffix"
    words = _words(path)
    rgb_hits = words & RGB_WORDS
    depth_hits = words & DEPTH_WORDS
    # A dataset container commonly has the literal name "images".  It is weak
    # evidence and must not conflict with a more specific /depth/ component.
    if depth_hits and rgb_hits == {"images"}:
        rgb_hits = set()
    rgb_name = bool(rgb_hits)
    depth_name = bool(depth_hits)
    channels = info.get("channels")
    bit_depth = info.get("png_bit_depth")
    rgb_attr = channels in (3, 4)
    depth_attr = path.suffix.lower() == ".png" and channels == 1 and bit_depth == 16
    if rgb_name and depth_name:
        return "unknown", "rgb_and_depth_keywords_conflict"
    if rgb_name and depth_attr:
        return "unknown", "rgb_keyword_conflicts_with_16bit_single_channel"
    if depth_name and rgb_attr:
        return "unknown", "depth_keyword_conflicts_with_multichannel_image"
    if depth_name or depth_attr:
        return "depth", "depth_keyword" if depth_name else "16bit_single_channel_png"
    if rgb_name or rgb_attr:
        return "rgb", "rgb_keyword" if rgb_name else "multichannel_image"
    return "unknown", "insufficient_evidence"


def make_inventory_row(path: Path, root: Path, annotations: Path) -> dict[str, Any]:
    info = image_info(path) if path.suffix.lower() in IMAGE_SUFFIXES else {
        "image_width": "", "image_height": "", "image_mode": "",
        "dtype": "", "channels": "", "png_bit_depth": "",
    }
    candidate, note = classify(path, info, path.is_relative_to(annotations))
    return {
        "relative_path": relative_text(path, root), "filename": path.name,
        "stem": path.stem, "suffix": path.suffix.lower(),
        "parent_folder": path.parent.name, "file_size": path.stat().st_size,
        **info, "candidate_type": candidate,
        "classification_note": info.get("read_error", note),
    }


def sanitize(value: str) -> str:
    cleaned = re.sub(r"[^0-9A-Za-z._\-\u4e00-\u9fff]+", "_", value).strip("_.")
    return cleaned or "default"


def session_for(relative_path: str) -> str:
    parts = Path(relative_path).parts
    if parts and parts[0].lower() == "images":
        parts = parts[1:-1]
    else:
        parts = parts[:-1]
    filtered = [p for p in parts if p.lower() not in RGB_WORDS | DEPTH_WORDS]
    return sanitize("__".join(filtered) if filtered else "default")


def normalized_base(stem: str) -> str:
    return re.sub(r"(?i)(?:_aligned_depth|_depth_reference|_depth_raw|_depth|_colour|_color|_image|_rgb)$", "", stem)


def stable_suffix(text: str) -> str:
    return hashlib.sha1(text.encode("utf-8")).hexdigest()[:8]


def resolve_stored_path(root: Path, value: str) -> Path:
    p = Path(value)
    return p if p.is_absolute() else root / Path(*value.split("/"))
