# Environment audit

- Audit date: 2026-07-21 (Asia/Shanghai)
- Host: `iair-svl`, Ubuntu 24.04.3 LTS, kernel 6.2.0-26-generic
- GPUs: 8 × NVIDIA GeForce RTX 4090 D, 49140 MiB each
- NVIDIA driver: 570.153.02; driver-reported CUDA compatibility: 12.8
- System CUDA compiler: nvcc 11.8.89 (left unchanged)
- Memory: 503 GiB; available during audit: 491 GiB
- Home filesystem: 1.6 TiB, 137 GiB available during audit
- Existing base: Miniconda, Python 3.13.13, torch 2.11.0+cu130; CUDA unavailable because that wheel requires a newer driver
- Isolated baseline: Conda `pidnet_mars`, Python 3.10.20, torch 2.7.1+cu128, torchvision 0.22.1+cu128
- Isolated CUDA check: available; GPU 0 RTX 4090 D; compute capability 8.9; GPU matrix multiply succeeded

No system package, driver, CUDA toolkit, ROS2 workspace, or existing Conda environment was modified. Full command output is in `environment_raw.txt`; isolated-environment verification is in `../logs/cuda_validation.log`.
