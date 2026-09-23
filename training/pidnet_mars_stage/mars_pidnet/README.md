# Mars PIDNet-S baseline

This package wraps the pinned official checkout without modifying its network. It implements strict fine/coarse masks, fixed padding, validation, synthetic training and a 10-image overfit entry point. See `docs/dataset_format.md` and the root `FINAL_REPORT.md`.

From the root, use `run_official_smoke.sh`, `run_synthetic_train_smoke.sh`, or `run_tiny_overfit.sh /path/to/dataset`. All commands run in the isolated `pidnet_mars` Conda environment.
