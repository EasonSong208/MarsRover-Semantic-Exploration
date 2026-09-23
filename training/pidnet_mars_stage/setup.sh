#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -d "$ROOT/upstream/PIDNet/.git" ]]; then
  git clone https://github.com/XuJiacong/PIDNet.git "$ROOT/upstream/PIDNet"
fi
git -C "$ROOT/upstream/PIDNet" checkout 4c158cf24ce432f0a8cb43364fae38d93cee0dc3
conda create -y -n pidnet_mars python=3.10.20 pip=25.1
conda run -n pidnet_mars python -m pip install --index-url https://download.pytorch.org/whl/cu128 torch==2.7.1 torchvision==0.22.1
conda run -n pidnet_mars python -m pip install -r "$ROOT/requirements-app.txt"
echo "Environment and pinned official checkout are ready."
