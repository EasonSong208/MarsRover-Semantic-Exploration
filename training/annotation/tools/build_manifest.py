from __future__ import annotations

from collections import defaultdict
import json
from pathlib import Path, PureWindowsPath

from common import (normalized_base, parser, read_csv, sanitize, session_for,
                    stable_suffix, write_csv)

FIELDS = [
    "sample_id", "session_id", "rgb_path", "depth_path", "json_path",
    "rgb_width", "rgb_height", "depth_width", "depth_height", "depth_dtype",
    "pair_status", "pair_method",
]


def unique_or_state(candidates: list[dict]) -> tuple[dict | None, str]:
    by_path = {str(x["relative_path"]): x for x in candidates}
    values = list(by_path.values())
    return (values[0], "matched") if len(values) == 1 else (None, "unmatched" if not values else "ambiguous")


def resolve_image_path(root: Path, json_path: Path, image_path: str, rgbs: list[dict]) -> list[dict]:
    if not image_path:
        return []
    win_parts = PureWindowsPath(image_path).parts
    clean_parts = [p for p in win_parts if p not in ("\\", "/")]
    raw = Path(*clean_parts)
    direct = (json_path.parent / raw).resolve() if not raw.is_absolute() else raw.resolve()
    exact = [r for r in rgbs if (root / str(r["relative_path"])).resolve() == direct]
    if exact:
        return exact
    # Absolute Windows paths cannot be resolved in WSL; match a unique trailing path.
    normalized = "/".join(clean_parts).lower()
    trailing = [r for r in rgbs if normalized.endswith(str(r["relative_path"]).lower())]
    if trailing:
        return trailing
    return [r for r in rgbs if str(r["filename"]).lower() == raw.name.lower()]


def choose_rgb(root: Path, json_row: dict, rgbs: list[dict]) -> tuple[dict | None, str, str, list[dict]]:
    jp = root / str(json_row["relative_path"])
    try:
        image_path = str(json.loads(jp.read_text(encoding="utf-8")).get("imagePath", ""))
    except Exception:
        image_path = ""
    candidates = resolve_image_path(root, jp, image_path, rgbs)
    picked, state = unique_or_state(candidates)
    if state != "unmatched":
        return picked, state, "labelme_imagePath", candidates
    candidates = [r for r in rgbs if r["stem"] == json_row["stem"]]
    picked, state = unique_or_state(candidates)
    return picked, state, "exact_stem", candidates


def choose_depth(rgb: dict, depths: list[dict], rgbs: list[dict]) -> tuple[dict | None, str, str, list[dict]]:
    session = session_for(str(rgb["relative_path"]))
    in_session = [d for d in depths if session_for(str(d["relative_path"])) == session]
    same_stem = [d for d in in_session if d["stem"] == rgb["stem"]]
    picked, state = unique_or_state(same_stem)
    if state != "unmatched":
        return picked, state, "same_session_exact_stem", same_stem
    same_base = [d for d in in_session if normalized_base(str(d["stem"])) == normalized_base(str(rgb["stem"]))]
    picked, state = unique_or_state(same_base)
    if state != "unmatched":
        return picked, state, "same_session_normalized_base", same_base
    # A session with exactly one RGB and one Depth is a structural unique match,
    # independent of filename order. This covers synchronized timestamp pairs.
    session_rgbs = [r for r in rgbs if session_for(str(r["relative_path"])) == session]
    if len(session_rgbs) != 1:
        return None, "ambiguous" if in_session else "unmatched", "same_session_not_one_rgb_one_depth", in_session
    picked, state = unique_or_state(in_session)
    return picked, state, "same_session_unique_depth", in_session


def main() -> int:
    args = parser("建立严格的 RGB/Depth/JSON 配对清单").parse_args()
    root = args.root.resolve()
    reports = root / "dataset_generated" / "reports"
    inventory_path = reports / "file_inventory.csv"
    if not inventory_path.exists():
        raise SystemExit("请先运行 inspect_dataset.py")
    rows = read_csv(inventory_path)
    rgbs = [r for r in rows if r["candidate_type"] == "rgb"]
    depths = [r for r in rows if r["candidate_type"] == "depth"]
    jsons = [r for r in rows if r["candidate_type"] == "labelme_json"]
    used_rgb: set[str] = set()
    used_depth: set[str] = set()
    used_json: set[str] = set()
    manifest: list[dict] = []
    ambiguous: list[dict] = []

    for j in jsons:
        rgb, rgb_state, rgb_method, rgb_candidates = choose_rgb(root, j, rgbs)
        if rgb_state == "ambiguous":
            ambiguous.append({"source_type": "json", "source_path": j["relative_path"],
                              "candidate_type": "rgb", "candidate_paths": "|".join(str(x["relative_path"]) for x in rgb_candidates),
                              "method": rgb_method})
        depth = None
        depth_state, depth_method, depth_candidates = "unmatched", "not_attempted_without_rgb", []
        if rgb:
            depth, depth_state, depth_method, depth_candidates = choose_depth(rgb, depths, rgbs)
            used_rgb.add(str(rgb["relative_path"]))
            used_json.add(str(j["relative_path"]))
            if depth:
                used_depth.add(str(depth["relative_path"]))
            if depth_state == "ambiguous":
                ambiguous.append({"source_type": "rgb", "source_path": rgb["relative_path"],
                                  "candidate_type": "depth", "candidate_paths": "|".join(str(x["relative_path"]) for x in depth_candidates),
                                  "method": depth_method})
        session = session_for(str(rgb["relative_path"])) if rgb else "unresolved"
        stem = str(rgb["stem"] if rgb else j["stem"])
        status = "matched" if rgb and depth else ("ambiguous" if "ambiguous" in (rgb_state, depth_state) else "unmatched")
        manifest.append({
            "sample_id": sanitize(f"{session}__{stem}"), "session_id": session,
            "rgb_path": rgb["relative_path"] if rgb else "", "depth_path": depth["relative_path"] if depth else "",
            "json_path": j["relative_path"], "rgb_width": rgb["image_width"] if rgb else "",
            "rgb_height": rgb["image_height"] if rgb else "", "depth_width": depth["image_width"] if depth else "",
            "depth_height": depth["image_height"] if depth else "", "depth_dtype": depth["dtype"] if depth else "",
            "pair_status": status, "pair_method": f"rgb:{rgb_method};depth:{depth_method}",
        })

    # Inventory RGBs with no JSON are retained as explicit unmatched rows.
    for rgb in rgbs:
        if str(rgb["relative_path"]) in used_rgb:
            continue
        session = session_for(str(rgb["relative_path"]))
        depth, depth_state, depth_method, depth_candidates = choose_depth(rgb, depths, rgbs)
        if depth_state == "ambiguous":
            ambiguous.append({"source_type": "rgb", "source_path": rgb["relative_path"], "candidate_type": "depth",
                              "candidate_paths": "|".join(str(x["relative_path"]) for x in depth_candidates), "method": depth_method})
        manifest.append({"sample_id": sanitize(f"{session}__{rgb['stem']}"), "session_id": session,
                         "rgb_path": rgb["relative_path"], "depth_path": depth["relative_path"] if depth else "", "json_path": "",
                         "rgb_width": rgb["image_width"], "rgb_height": rgb["image_height"],
                         "depth_width": depth["image_width"] if depth else "", "depth_height": depth["image_height"] if depth else "",
                         "depth_dtype": depth["dtype"] if depth else "", "pair_status": "unmatched",
                         "pair_method": f"rgb:no_json;depth:{depth_method}"})
        if depth:
            used_depth.add(str(depth["relative_path"]))

    # Guarantee sample_id uniqueness deterministically without hiding collisions.
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in manifest:
        grouped[row["sample_id"]].append(row)
    for base, group in grouped.items():
        if len(group) > 1:
            for row in group:
                row["sample_id"] = f"{base}__{stable_suffix(row['rgb_path'] or row['json_path'])}"

    write_csv(root / "dataset_generated/manifests/all_samples.csv", manifest, FIELDS)
    write_csv(reports / "ambiguous_pairs.csv", ambiguous,
              ["source_type", "source_path", "candidate_type", "candidate_paths", "method"])
    unmatched_rgb = [{"rgb_path": r["relative_path"], "reason": "no_unique_json"}
                     for r in rgbs if str(r["relative_path"]) not in used_rgb]
    unmatched_depth = [{"depth_path": r["relative_path"], "reason": "no_unique_rgb"}
                       for r in depths if str(r["relative_path"]) not in used_depth]
    unmatched_json = [{"json_path": r["relative_path"], "reason": "no_unique_rgb"}
                      for r in jsons if str(r["relative_path"]) not in used_json]
    write_csv(reports / "unmatched_rgb.csv", unmatched_rgb, ["rgb_path", "reason"])
    write_csv(reports / "unmatched_depth.csv", unmatched_depth, ["depth_path", "reason"])
    write_csv(reports / "unmatched_json.csv", unmatched_json, ["json_path", "reason"])
    print(f"manifest 样本: {len(manifest)}; 完整匹配: {sum(r['pair_status'] == 'matched' for r in manifest)}")
    print(f"unmatched RGB/Depth/JSON: {len(unmatched_rgb)}/{len(unmatched_depth)}/{len(unmatched_json)}")
    print(f"ambiguous: {len(ambiguous)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
