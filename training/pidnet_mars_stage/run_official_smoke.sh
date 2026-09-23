#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEIGHTS="$ROOT/env/weights/PIDNet_S_Cityscapes_val.pt"
ARGS=(); [[ -f "$WEIGHTS" ]] && ARGS+=(--weights "$WEIGHTS")
conda run -n pidnet_mars env PYTHONPATH="$ROOT" python "$ROOT/mars_pidnet/tools/official_smoke.py" --image "$ROOT/upstream/PIDNet/samples/frankfurt_000000_002196_leftImg8bit.png" --output "$ROOT/outputs/official_smoke" "${ARGS[@]}" 2>&1 | tee "$ROOT/logs/official_inference.log"
