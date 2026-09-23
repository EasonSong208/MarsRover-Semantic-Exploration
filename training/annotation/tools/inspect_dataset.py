from __future__ import annotations

from collections import Counter, defaultdict
from pathlib import Path
import json
import re

from common import INVENTORY_FIELDS, ensure_output_dirs, make_inventory_row, parser, write_csv, write_json


def tree_text(root: Path) -> str:
    lines = [f"{root.name}/"]
    bases = [base for base in (root / "images", root / "annotations") if base.exists()]
    items = sorted((p for base in bases for p in (base, *base.rglob("*"))),
                   key=lambda p: p.as_posix().lower())
    for p in items:
        rel = p.relative_to(root)
        lines.append(f"{'  ' * (len(rel.parts) - 1)}{'[D] ' if p.is_dir() else '[F] '}{rel.name}")
    return "\n".join(lines) + "\n"


def naming_patterns(rows: list[dict]) -> list[dict]:
    patterns: Counter[tuple[str, str]] = Counter()
    for r in rows:
        if r["candidate_type"] not in {"rgb", "depth"}:
            continue
        stem = str(r["stem"])
        if re.fullmatch(r"\d{10}\.\d{6,9}", stem):
            pattern = "unix_timestamp_fraction"
        elif re.search(r"(?i)_(rgb|color|image|depth|depth_raw|aligned_depth)$", stem):
            pattern = "semantic_suffix"
        elif re.search(r"\d+$", stem):
            pattern = "numeric_suffix"
        else:
            pattern = "other"
        patterns[(r["candidate_type"], pattern)] += 1
    return [{"candidate_type": k[0], "pattern": k[1], "count": v}
            for k, v in sorted(patterns.items())]


def main() -> int:
    args = parser("只读扫描 JetRover 数据目录").parse_args()
    root = args.root.resolve()
    images, annotations = root / "images", root / "annotations"
    missing = [str(p) for p in (images, annotations, root / "labels.txt", root / "annotation_rules.md") if not p.exists()]
    if missing:
        raise SystemExit("缺少必需输入：" + ", ".join(missing))
    ensure_output_dirs(root)
    files = sorted([p for base in (images, annotations) for p in base.rglob("*") if p.is_file()],
                   key=lambda p: p.as_posix().lower())
    rows = [make_inventory_row(p, root, annotations) for p in files]
    reports = root / "dataset_generated" / "reports"
    (reports / "tree.txt").write_text(tree_text(root), encoding="utf-8")
    write_csv(reports / "file_inventory.csv", rows, INVENTORY_FIELDS)

    counts = Counter(r["candidate_type"] for r in rows)
    folders: dict[str, dict] = defaultdict(lambda: {"file_count": 0, "properties": Counter()})
    for r in rows:
        if str(r["relative_path"]).startswith("images/"):
            folder = str(Path(str(r["relative_path"])).parent.as_posix())
            folders[folder]["file_count"] += 1
            key = f"{r['image_width']}x{r['image_height']}|{r['image_mode']}|{r['dtype']}|ch={r['channels']}"
            folders[folder]["properties"][key] += 1
    stem_paths: dict[str, list[str]] = defaultdict(list)
    for r in rows:
        stem_paths[str(r["stem"])].append(str(r["relative_path"]))
    duplicate_stems = {k: v for k, v in stem_paths.items() if len(v) > 1}
    json_refs = []
    for r in rows:
        if r["candidate_type"] == "labelme_json":
            try:
                data = json.loads((root / str(r["relative_path"])).read_text(encoding="utf-8"))
                json_refs.append({"json": r["relative_path"], "imagePath": data.get("imagePath", "")})
            except Exception as exc:
                json_refs.append({"json": r["relative_path"], "read_error": str(exc)})
    summary = {
        "root": str(root), "counts": {k: counts.get(k, 0) for k in ("rgb", "depth", "labelme_json", "unknown")},
        "folders": {k: {"file_count": v["file_count"], "properties": dict(v["properties"])} for k, v in sorted(folders.items())},
        "naming_patterns": naming_patterns(rows), "duplicate_stems": duplicate_stems,
        "json_image_paths": json_refs,
        "classification_conflicts": [r for r in rows if r["candidate_type"] == "unknown"],
    }
    write_json(reports / "scan_summary.json", summary)
    print((reports / "tree.txt").read_text(encoding="utf-8"))
    print("匹配策略：JSON imagePath > JSON/RGB 同 stem > 同 session 唯一候选/相邻 rgb-depth 目录同名 > 去语义后缀基础 ID；只接受唯一候选，不按顺序配对。")
    print(f"RGB 候选数量: {counts.get('rgb', 0)}")
    print(f"Depth 候选数量: {counts.get('depth', 0)}")
    print(f"JSON 数量: {counts.get('labelme_json', 0)}")
    print(f"unknown 数量: {counts.get('unknown', 0)}")
    for folder, value in summary["folders"].items():
        print(f"{folder}: {value['file_count']} files; {value['properties']}")
    print("推测出的命名模式:", summary["naming_patterns"])
    print(f"存在重名 stem: {'是' if duplicate_stems else '否'} ({len(duplicate_stems)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
