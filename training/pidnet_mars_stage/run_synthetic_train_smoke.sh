#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
conda run -n pidnet_mars env PYTHONPATH="$ROOT" python "$ROOT/mars_pidnet/tools/synthetic_train_smoke.py" --output "$ROOT/outputs/synthetic_smoke" 2>&1 | tee "$ROOT/logs/synthetic_train.log"
