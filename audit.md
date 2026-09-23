# JetRover Jetson PIDNet-S 第一阶段审计

- 审计时间：2026-08-05（Asia/Shanghai）
- 目标：`ubuntu@10.x.x.x`，主机 `ubuntu-desktop`
- 范围：只读系统/文件元数据检查、现有 Python 模块导入和 CUDA 可见性查询
- 写入范围：仅新建 `/tmp/pidnet_audit` 及本报告所列四个输出
- 未执行：安装、Conda、APT/PIP、源码编译、ONNX 导出、TensorRT engine 生成、文件清理/移动、ROS2/服务启动停止、相机或运动控制

## 结论

| 判断 | 结果 | 依据 |
|---|---|---|
| A. 当前已有环境能否直接运行 PIDNet-S | **否** | 核心运行库齐全，但全盘未发现 PIDNet 代码、权重或配置；官方配置入口依赖的 `yacs` 不可导入；模型类别表也尚未确定。 |
| B. 能否不新建大型环境完成测试 | **有条件可以** | 系统 Python 3.10 已有 CUDA PyTorch、OpenCV、NumPy、TensorRT、PyCUDA、ONNX/ONNX Runtime。无需 Conda 或复制大型环境；只需后续以受控方式补齐小型代码/配置、匹配权重，并选择安装不足 1 MiB 的 `yacs` 或采用经过审查的无-yacs 最小推理入口。第一阶段没有做这些。 |
| C. 部署后能否至少保留 2 GiB 空闲 | **可以（按保守持久空间预算）** | 当前约 4.203 GiB 可用；最保守的“PT + ONNX + TensorRT engine 全保留”预算为 260 MiB，预计仍约 3.950 GiB。转换临时峰值另算，不能把这项结论当作允许本机生成 engine。 |
| D. 当前是否适合继续图片串测试 | **否** | 没有可加载的 PIDNet-S 资产，`yacs` 缺失，类别合法值集合未知；常规录制目录只有 71 张 RGB，未达到 100 张下限（回收站另有 563 张 RGB，但不应默认依赖）。因此第二阶段未执行。 |

第一阶段总评：**环境底座可用，但模型资产门槛未满足；停止在第一阶段。** 当前结果不是最终的 PASS / CONDITIONAL PASS / FAIL 性能判定，因为尚未进行任何 PIDNet 推理或 10 分钟稳定性测试。

## 磁盘与目录

- 根分区 `/dev/nvme0n1p1`：79 GiB 容量，71 GiB 已用，`df -h` 显示 **4.3 GiB 可用、95% 使用率**。
- 字节级基线：`4,513,808,384 B` 可用，即约 **4.203 GiB**。
- 报告生成后的 16:03 快照变为 `7,026,933,760 B` 可用，即 `df -h` 的 **6.6 GiB、92% 使用率**。本审计没有执行任何清理、删除或移动；约 2.34 GiB 的释放来自审计之外的并发活动，来源未追踪。为避免乐观偏差，以下预算统一使用窗口内最低值 4.203 GiB。
- `/home/ubuntu`：约 **31.57 GiB**；因一个 Open3D 构建子目录无读取权限，这一数值可能略低于实际值。
- `/home/ubuntu/third_party`：约 **11.46 GiB**。
- `/home/ubuntu/.cache`：约 **8.30 GiB**，其中 VS Code C/C++ 缓存约 7.24 GiB、pip 缓存约 **595.01 MiB**。按限制未清理。
- `/home/ubuntu/.vscode-server`：约 **4.21 GiB**；`/home/ubuntu/.local`：约 **3.02 GiB**。
- 厂商 ROS2 工作空间 `/home/ubuntu/ros2_ws`：约 **1.45 GiB**（`src` 728.31 MiB、`.git` 630.37 MiB、`build` 93.07 MiB、`install` 25.38 MiB、`log` 7.92 MiB）。未修改。
- 项目部署检出 `/home/ubuntu/hiwonder-jetson-robot`：约 **14.68 MiB**；其中 `models/` 和 `configs/` 都仅约 8 KiB，未含 PIDNet 资产。
- `/home/ubuntu/large_models`：约 **281.48 MiB**；扫描到多种现有 PT/ONNX/engine，但没有 PIDNet 命名资产。
- `/home/ubuntu/rgbd_recordings`：约 **29.62 MiB**，共 142 张图，其中抽样结构表明 71 张 640x360 RGB PNG + 71 张 16 位深度 PNG。
- 未发现 `pyvenv.cfg` 或 `conda-meta`，即未发现现有 venv/Conda 环境；也没有创建新环境。

注意：根分区已经达到 95%，虽然 PIDNet-S 本身较小，但任何后续下载、导出或转换都应先做精确的“预期写入量 + 2 GiB 保留线”门禁。现有大目录仅作为观察结果列出，不代表建议或授权清理。

## Python 与加速栈

默认解释器是 `/usr/bin/python3`，版本 **Python 3.10.12**；当前模块来自系统目录与 `/home/ubuntu/.local/lib/python3.10/site-packages` 的混合环境。

| 模块 | 导入 | 版本/关键信息 |
|---|---|---|
| `torch` | PASS | 2.7.0；CUDA build 12.6；cuDNN 90300；`torch.cuda.is_available() == True`；1 个设备 `Orin`，计算能力 8.7 |
| `torchvision` | PASS | 0.22.0 |
| `cv2` | PASS | 4.11.0；OpenCV CUDA 设备数 1 |
| `numpy` | PASS | 1.26.4 |
| `tensorrt` | PASS | 10.7.0 |
| `pycuda` | PASS | distribution 2024.1.2 |
| `onnxruntime` | PASS | 1.22.0（包名 `onnxruntime-gpu`）；providers 为 TensorRT、CUDA、CPU |
| `onnx`（补充） | PASS | 1.17.0 |
| `yacs`（官方配置入口） | **MISSING** | 预计新增持久空间小于 1 MiB；第一阶段未安装。建议后续获准后安装最小版本，或使用经过审查且不导入官方配置模块的最小推理入口。 |
| `onnxscript`（PyTorch 新 ONNX 导出器可选依赖） | **MISSING** | 不影响已有 ONNX Runtime 推理；若必须使用 PyTorch 2.7 的新导出路径，估计新增约 5–20 MiB。也可评估 `dynamo=False` 的旧导出路径。第一阶段未安装/导出。 |

CUDA 工具包存在于 `/usr/local/cuda-12.6`：版本文件为 12.6.11，`/usr/local/cuda/bin/nvcc` 为 12.6.68。非交互 PATH 未包含该目录，所以直接执行 `nvcc` 报未找到；这不是软件缺失，也不影响本次推理审计。

平台版本：

- JetPack **6.2**，L4T **36.4.3**，Ubuntu 22.04.5，内核 5.15.148-tegra。
- CUDA **12.6**（runtime 12.6.68）。
- cuDNN **9.3.0.75**。
- TensorRT **10.7.0.23**。
- 当前功耗模式 `MAXN_SUPER`。

## 内存、交换与温度基线

- 物理内存：15 GiB；采样时约 **9.7 GiB available**。
- 交换空间：约 **17.6 GiB**（10 GiB swapfile + 8 个约 0.956 GiB 的 zram），仅约 1 MiB 已用。
- 静态采样温度：GPU 约 61.1–61.3 °C、CPU/TJ 约 63.4–63.9 °C；GPU 利用率为 0%。这是审计基线，不是 10 分钟负载温度。

## 持久与临时空间预算

估算基于 PIDNet-S 约 8.06M 参数、FP32 模型约 29.1 MB，并对 PyTorch checkpoint 封装、代码、配置和报告留出余量。没有实际下载或转换，因此均为规划区间，不是实测产物大小。

| 项目 | 预计新增持久空间 |
|---|---:|
| 精简 PIDNet 代码 + 配置 | 2–10 MiB |
| PIDNet-S FP32 checkpoint | 30–60 MiB |
| benchmark/report + 最多 10 张预览 | 5–20 MiB |
| 基础资产合计 | **40–90 MiB** |
| PyTorch 方案（复用现有 runtime） | **50–120 MiB** |
| ONNX-only 方案 | **45–110 MiB** |
| ONNX 方案且保留 PyTorch checkpoint | **80–180 MiB** |
| TensorRT engine-only 部署 | **30–100 MiB** |
| PT + ONNX + engine 全部保留 | **110–260 MiB** |

预计部署后的根分区可用空间：

- PyTorch：约 **4.087–4.154 GiB**。
- ONNX（保留 checkpoint）：约 **4.028–4.125 GiB**。
- TensorRT（PT/ONNX/engine 全保留）：约 **3.950–4.095 GiB**。

转换峰值应把磁盘和统一内存分开看：

- ONNX 导出：磁盘临时峰值约 **0.1–0.5 GiB**，RAM 峰值约 **1–3 GiB**。
- TensorRT 构建：磁盘临时峰值约 **0.3–1.0 GiB**；builder/tactic 工作区主要消耗 RAM，保守按 **2–6 GiB** 规划。
- 若转换工具保留多份中间模型、详细 profiling 或失败转储，磁盘峰值可能超出上述区间。建议转换前强制检查实时 `df -B1 /`，设置产物上限，并在可行时在开发机完成 ONNX 导出；TensorRT engine 必须按目标 Orin、JetPack/TensorRT 兼容性重新确认，不能假定跨平台可移植。

按上述持久部署上限，2 GiB 保留线可满足；但若在 Jetson 同时进行超过约 1.9 GiB 的临时落盘，可能跌破 2 GiB。因此当前限制下不应生成 TensorRT engine，且本次没有生成。

## 第二阶段门禁

第二阶段没有执行。重新评估前至少需要：

1. 明确 PIDNet-S checkpoint 的任务与合法类别集合（例如 Cityscapes 19 类、CamVid 11 类或自定义类别），并核验来源、哈希与许可证。
2. 提供/确认代码、配置和权重的目标路径；不得放入现有 ROS2/vendor 工作空间中做临时修改。
3. 解决 `yacs` 缺失，或审查一个只依赖现有 torch/cv2/numpy 的最小推理入口；不得默认安装。
4. 指定 100–300 张稳定的本地 RGB 图片。常规录制目录只有 71 张 RGB；回收站中的 563 张 RGB 不应在未确认的情况下作为测试集。
5. 执行前再次确认可用磁盘不少于 2 GiB + 预计临时峰值，并把预览上限固定为 10 张。

满足这些门禁后，可优先做 PyTorch 图片串冒烟测试；ONNX/TensorRT 属于后续对比路径。该测试只读原图、仅写 CSV/JSON/Markdown 和最多 10 张预览，不启动 ROS2 或任何运动节点。

## 证据与估算来源

- 原始机器证据：`disk_report.txt`、`environment_report.txt`。
- PIDNet 官方仓库：`https://github.com/XuJiacong/PIDNet`；官方 `configs/default.py` 直接导入 `yacs.config.CfgNode`。
- 参数/浮点模型规模交叉参考：Qualcomm PIDNet model card（8.06M 参数、29.1 MB float model）。
- 所有未在 Jetson 上实际生成的大小均明确标为估算。
