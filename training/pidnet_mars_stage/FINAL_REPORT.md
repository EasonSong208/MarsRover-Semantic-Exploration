# PIDNet-S Mars baseline final report

1. **Server environment:** Ubuntu 24.04.3, 8× RTX 4090 D, driver 570.153.02 (CUDA 12.8 compatibility), system nvcc 11.8, 503 GiB RAM. Details are in `reports/environment_audit.md`.
2. **Official commit:** `4c158cf24ce432f0a8cb43364fae38d93cee0dc3`.
3. **Independent environment:** Conda `pidnet_mars`, Python 3.10.20, torch 2.7.1+cu128 and torchvision 0.22.1+cu128. `setup.sh` recreates it; `requirements-lock.txt` records the resolved Python packages.
4. **Official PIDNet-S GPU forward:** PASS. The official network ran natively on GPU.
5. **Official weights:** PASS. The uploaded official PIDNet-S Cityscapes validation checkpoint loaded successfully: 453 parameters/buffers loaded, zero required missing keys, and zero unexpected keys. Its SHA256 is `b51aa935bdb64a0779d776f38267fd49f7cce59413910abbbf0a74934b3d7c01`.
6. **Official pretrained inference:** PASS. Input/output were 1×3×384×640 and 1×19×384×640 on RTX 4090 D CUDA. After 10 warmups, 100 forwards averaged 7.724823 ms; peak PyTorch allocation was 52.3125 MiB. This is a local measurement, not paper FPS.
7. **Mars heads:** 6-class and 8-class training models both build and GPU-forward successfully; output shapes were 1×6×48×80 and 1×8×48×80.
8. **Dataset loader:** PASS (3/3 unit tests). It enforces stem pairing, readable RGB, single-channel uint8 masks, exact class sets, dimensions, and preserves ignore=255.
9. **Padding:** PASS. Content remains 640×360 and receives exactly 12 rows top/bottom. RGB value is 0; mask value is 255. Alignment is unit-tested; there is no stretch.
10. **Synthetic training:** PASS for one epoch, finite loss 9.281750, trainable parameters changed, checkpoint reloaded, evaluation emitted per-class IoU and a confusion matrix. This does not show Mars-scene learning.
11. **Real 10-image overfit:** Ready but awaiting 1–10 annotated RGB/mask pairs in the documented dataset layout. Run `./run_tiny_overfit.sh /path/to/dataset [epochs]`.
12. **Upstream modifications:** none. All integration code is under `mars_pidnet/`.
13. **System pollution:** none. No sudo, system package, CUDA/driver, ROS2, vendor workspace, or existing environment changes were made.
14. **Next minimum action:** provide the annotated fine masks and RGB images, generate coarse masks, validate them, then run the tiny-overfit command. Deployment-stage acceptance is otherwise complete.

## Official pretrained checkpoint acceptance

- File: `env/weights/PIDNet_S_Cityscapes_val.pt`
- Size: 31,145,857 bytes (about 30 MiB)
- SHA256: `b51aa935bdb64a0779d776f38267fd49f7cce59413910abbbf0a74934b3d7c01`
- Loaded parameters/buffers: 453
- Required missing keys: 0
- Unexpected keys: 0
- Input: `1 × 3 × 384 × 640`
- Output: `1 × 19 × 384 × 640`
- Device: NVIDIA GeForce RTX 4090 D, CUDA
- 100-forward mean latency: 7.724823 ms
- Peak allocated GPU memory: 52.3125 MiB
- Smoke script exit status: 0

The 28 skipped checkpoint entries are training-only auxiliary-head and loss entries absent from the inference model; they are not missing required inference parameters. Therefore the official PIDNet-S Cityscapes pretrained checkpoint successfully loaded and completed GPU inference.

## Verification evidence

- `logs/cuda_validation.log`: isolated CUDA import and compute
- `logs/unit_tests.log`: 3 passed
- `logs/model_build_gpu.log`: 6/8-class GPU construction
- `logs/official_inference.log`: official Cityscapes-pretrained GPU inference and load details
- `logs/synthetic_train.log`: loss, parameter-change check, reload and evaluation
- `outputs/synthetic_smoke/run/`: checkpoint, curve, predictions, overlays and metrics

No ONNX, TensorRT, ROS2, Jetson, or network-structure adaptation was started.

## Deployment acceptance matrix

| Item | Status |
|---|---|
| Isolated environment | PASS |
| Native CUDA compute | PASS |
| Official PIDNet code | PASS |
| 6-class / 8-class forward | PASS |
| Dataset and ignore=255 | PASS |
| 640×360 → 640×384 padding | PASS |
| Unit tests | PASS |
| Synthetic training loop | PASS |
| Checkpoint reload | PASS |
| IoU / confusion matrix | PASS |
| Random-weight performance smoke | PASS |
| Official pretrained weight load | PASS |
| Official pretrained GPU inference | PASS |
