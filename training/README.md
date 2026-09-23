# Training: PIDNet-S Mars semantic baseline

Training, validation, and annotation code for the five-class (hazard5) PIDNet-S
used by `ros2_ws/src/semantic_perception`. The Mars dataset and model weights are
**not** distributed in this repository (see "Data & weights" below).

## Layout

- `pidnet_mars_stage/` — integration/validation stage: the `mars_pidnet` wrapper
  around the pinned official PIDNet checkout, configs for the 6/8/hazard5-class
  heads, dataset validation, padding utilities, synthetic/overfit smoke entries,
  and the acceptance report `FINAL_REPORT.md`.
- `annotation/` — dataset preparation tooling (Labelme JSON → fine/coarse/hazard5
  masks, manifests, validators; see `annotation/DATASET_PREPARATION.md`).
- `hazard5/` — the 17-image hazard5 overfit experiment (training + audit scripts).

## Upstream

- https://github.com/XuJiacong/PIDNet (MIT license), pinned commit
  `4c158cf24ce432f0a8cb43364fae38d93cee0dc3`.
- `pidnet_mars_stage/setup.sh` clones it under `pidnet_mars_stage/upstream/PIDNet`.

## Environment

See `pidnet_mars_stage/reports/environment_audit.md`; `setup.sh` creates the
isolated Conda environment (`pidnet_mars`, Python 3.10, torch 2.7.1+cu128).

## Data & weights

- The Mars RGB/Depth dataset and derived masks are not distributed here;
  `pidnet_mars_stage/dataset/` only carries the directory layout, annotation
  rules, class palette, and splits.
- The V3 checkpoint (`best_miou_v3.pt`) and upstream pretrained weights are not
  included; provide your own at runtime.

## Quick start

```bash
cd pidnet_mars_stage
./setup.sh                                    # clone pinned upstream + build conda env
./run_synthetic_train_smoke.sh                # 1-epoch synthetic training smoke
./run_tiny_overfit.sh /path/to/dataset 200    # 10-image overfit
```
