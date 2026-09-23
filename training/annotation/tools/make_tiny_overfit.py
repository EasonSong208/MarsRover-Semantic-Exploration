from __future__ import annotations

import shutil
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image

from common import parser, read_csv, write_csv

NAMES = {0: "unknown_background", 1: "traversable_ground", 2: "hill_candidate",
         3: "crater_candidate", 4: "step_candidate", 5: "rover"}


def main() -> int:
    args = parser("选择最多 10 张类别覆盖与场景多样性较高的 tiny-overfit 样本").parse_args()
    root = args.root.resolve()
    manifest = read_csv(root / "dataset_generated/manifests/all_samples.csv")
    # Read status from JSON without depending on pandas.
    import json
    report = json.loads((root / "dataset_generated/reports/validation_report.json").read_text(encoding="utf-8"))
    valid_ids = {r["sample_id"] for r in report.get("samples", []) if r["status"] != "INVALID"}
    candidates = []
    frequency = Counter()
    for row in manifest:
        if row["sample_id"] not in valid_ids:
            continue
        mask_path = root / f"dataset_generated/masks_coarse/{row['session_id']}/{row['sample_id']}.png"
        if not mask_path.is_file():
            continue
        arr = np.asarray(Image.open(mask_path))
        counts = {value: int(np.sum(arr == value)) for value in NAMES}
        classes = {value for value, count in counts.items() if count > 0}
        frequency.update(classes)
        candidates.append({"row": row, "counts": counts, "classes": classes})

    selected, covered, sessions, combos = [], set(), set(), set()
    while candidates and len(selected) < 10:
        def score(item: dict) -> tuple:
            classes = item["classes"]
            combo = tuple(sorted(classes))
            new_cover = classes - covered
            rare = sum(1.0 / frequency[c] for c in classes if frequency[c])
            complexity = len(classes)
            diversity = int(item["row"]["session_id"] not in sessions) + int(combo not in combos)
            pixels = sum(item["counts"][c] for c in new_cover)
            return (len(new_cover), rare, diversity, complexity, pixels, item["row"]["sample_id"])
        best = max(candidates, key=score)
        candidates.remove(best)
        selected.append(best)
        covered |= best["classes"]
        sessions.add(best["row"]["session_id"])
        combos.add(tuple(sorted(best["classes"])))

    out = root / "dataset_generated/tiny_overfit"
    for sub in ("rgb", "masks_coarse", "overlays"):
        (out / sub).mkdir(parents=True, exist_ok=True)
    csv_rows = []
    for item in selected:
        row, counts = item["row"], item["counts"]
        sid, session = row["sample_id"], row["session_id"]
        rgb_src = root / f"dataset_generated/rgb/{session}/{sid}{Path(row['rgb_path']).suffix.lower()}"
        mask_src = root / f"dataset_generated/masks_coarse/{session}/{sid}.png"
        overlay_src = root / f"dataset_generated/overlays/{session}/{sid}_overlay.png"
        shutil.copy2(rgb_src, out / "rgb" / rgb_src.name)
        shutil.copy2(mask_src, out / "masks_coarse" / mask_src.name)
        shutil.copy2(overlay_src, out / "overlays" / overlay_src.name)
        csv_rows.append({"sample_id": sid, "session_id": session,
                         "classes_present": "|".join(NAMES[c] for c in sorted(item["classes"])),
                         **{f"pixels_{NAMES[c]}": counts[c] for c in NAMES}})
    fields = ["sample_id", "session_id", "classes_present"] + [f"pixels_{NAMES[c]}" for c in NAMES]
    write_csv(out / "tiny_overfit.csv", csv_rows, fields)
    generated_root = root / "dataset_generated"
    generated_list = generated_root / "reports/generated_files.txt"
    all_files = sorted(p.relative_to(root).as_posix() for p in generated_root.rglob("*")
                       if p.is_file() and p != generated_list)
    all_files.append(generated_list.relative_to(root).as_posix())
    generated_list.write_text("\n".join(all_files) + "\n", encoding="utf-8")
    missing = [NAMES[c] for c in NAMES if frequency[c] == 0]
    print("tiny-overfit 样本:", [x["row"]["sample_id"] for x in selected])
    print("数据集中完全缺失的 coarse 类别:", missing if missing else "无")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
