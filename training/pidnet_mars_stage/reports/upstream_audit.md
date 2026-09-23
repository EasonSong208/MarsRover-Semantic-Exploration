# Official PIDNet audit

- Repository: `https://github.com/XuJiacong/PIDNet.git`
- Pinned commit: `4c158cf24ce432f0a8cb43364fae38d93cee0dc3`
- License: MIT (`upstream/PIDNet/LICENSE`)
- Upstream working tree was clean after clone and no upstream files were edited.
- Reviewed: README, Cityscapes PIDNet-S YAML, `models/pidnet.py`, `models/model_utils.py`, base/Cityscapes datasets, training entry point, criterion and training functions.
- PIDNet-S parameters are selected by `m=2,n=3,planes=32,ppm_planes=96,head_planes=128`. Training mode returns P auxiliary segmentation, I final segmentation, and D boundary outputs. The Mars wrapper retains all three and trains all losses.
- Upstream already performs shape-filtered ImageNet/segmentation checkpoint loading, but its prefix handling is positional. The Mars wrapper adds a non-invasive loader supporting raw and `module.` keys, logs loaded/skipped/missing keys, and reinitializes shape-mismatched class heads.

## Weight availability

README warns that old individual Google Drive links may no longer work. Initial direct download attempts failed because this server could not route to `drive.google.com` (`Errno 101: Network is unreachable`); those historical logs remain in `logs/cityscapes_weight_download.log` and `logs/imagenet_weight_download.log`. The official PIDNet-S Cityscapes validation checkpoint was subsequently uploaded to `env/weights/PIDNet_S_Cityscapes_val.pt`, verified at 31,145,857 bytes with SHA256 `b51aa935bdb64a0779d776f38267fd49f7cce59413910abbbf0a74934b3d7c01`, loaded successfully, and exercised on GPU. See `reports/official_smoke_report.md`.
