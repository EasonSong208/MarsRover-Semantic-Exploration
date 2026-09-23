#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 1 ]]; then echo "usage: $0 DATASET_ROOT [EPOCHS]" >&2; exit 2; fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; DATASET="$1"; EPOCHS="${2:-200}"
conda run -n pidnet_mars env PYTHONPATH="$ROOT" python "$ROOT/mars_pidnet/tools/validate_dataset.py" "$DATASET" --granularity coarse --report-dir "$ROOT/reports"
conda run -n pidnet_mars env PYTHONPATH="$ROOT" python "$ROOT/mars_pidnet/tools/overfit_tiny.py" --dataset "$DATASET" --epochs "$EPOCHS" --classes 6 --max-samples 10 --output "$ROOT/outputs/tiny_overfit"
