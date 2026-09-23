# Official PIDNet-S smoke report

- Official architecture at pinned commit: GPU forward PASS
- Checkpoint: official PIDNet-S Cityscapes validation checkpoint, 31,145,857 bytes
- SHA256: `b51aa935bdb64a0779d776f38267fd49f7cce59413910abbbf0a74934b3d7c01`
- Load result: PASS; 453 parameters/buffers loaded, zero required missing keys, zero unexpected keys
- Input: repository sample image converted to RGB and resized to 640×384 for this architecture-only benchmark
- Output: float32 CUDA tensor, shape 1×19×384×640 after bilinear upsampling
- Warm-up: 10; timed iterations: 100
- Mean latency: 7.724823 ms on GPU 0
- PyTorch peak allocated memory: 52.3125 MiB
- Artifacts: input, class mask, color mask, overlay and metrics under `outputs/official_smoke/`

The 28 skipped entries are training-only auxiliary-head and loss entries absent from the inference model. These measurements are local results and do not quote paper FPS. Mars images use the separate no-stretch 640×360→640×384 padding path.
