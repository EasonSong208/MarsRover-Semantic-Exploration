# JetRover 火星语义分割数据准备

## 实际检测到的输入结构

2026-07-22 在 WSL 路径 `/mnt/d/mars_annotation` 检测到：

```text
D:\mars_annotation
├── annotations\                 17 个 Labelme JSON
├── images\                      17 个采集 session
│   └── <session>\
│       ├── rgb\                 每个 session 1 张 640×360 RGB PNG
│       └── depth\               每个 session 1 张 640×360、16 位单通道 PNG
├── labels.txt
├── annotation_rules.md
├── requirements-data.txt
├── run_prepare_dataset.ps1
└── tools\
```

其中一个目录名为 `到这里了recording_20260721_193529_910134`，脚本不会假定所有 session 名称遵循统一格式。最新、完整的递归结构由运行时生成的 `dataset_generated/reports/tree.txt` 给出。

## 配对规则

严格按以下优先级寻找唯一匹配：

1. 解析 Labelme JSON 的 `imagePath`（同时支持 `\` 与 `/`）；
2. JSON 与 RGB 的 stem 完全一致；
3. 同一 session 或相邻 `rgb/depth` 目录中的同名文件；
4. 去除 `_rgb`、`_color`、`_image`、`_depth`、`_depth_raw`、`_aligned_depth` 等后缀后比较基础 ID；
5. 当前数据的 RGB/Depth 时间戳不完全相同，因此仅在一个 session 中恰好存在唯一 Depth 时采用 `same_session_unique_depth`。

任何步骤出现多个候选都记为 ambiguous；没有候选记为 unmatched。脚本绝不按文件排序位置配对。

## Windows 一键运行

```powershell
conda activate mars-labelme
cd D:\mars_annotation
powershell -ExecutionPolicy Bypass -File .\run_prepare_dataset.ps1
```

脚本使用当前已激活环境的 `python`，检查 `numpy` 与 `Pillow`，任一步失败立即停止，不安装任何深度学习框架。所有 Python 工具还支持 `--root`，例如 WSL 静态扫描：

```bash
python3 /mnt/d/mars_annotation/tools/inspect_dataset.py --root /mnt/d/mars_annotation
```

已有 manifest 后，只生成独立 hazard5 变体且不触碰旧 fine/coarse mask：

```powershell
conda activate mars-labelme
cd D:\mars_annotation
powershell -ExecutionPolicy Bypass -File .\run_prepare_hazard5.ps1
```

## 输出目录

所有派生文件位于 `dataset_generated`：

- `rgb/<session>/`：以 `shutil.copy2` 复制的 RGB；
- `depth/<session>/`：逐字节原样复制的 Depth，不转换位深；
- `annotations_json/<session>/`：JSON 副本；
- `masks_fine/<session>/`：0～7/255 单通道 `uint8` mask；
- `masks_coarse/<session>/`：0～5/255 单通道 `uint8` mask；
- `masks_hazard5/<session>/`：独立的 0=other、1=hill、2=crater、3=step、4=rover、255=ignore mask；
- `overlays/<session>/`：RGB 与 coarse mask 的 0.4 透明度预览；
- `overlays_hazard5/<session>/`：hazard5 人工检查预览；
- `manifests/all_samples.csv`：权威配对清单；
- `reports/`：扫描、配对、转换和校验报告；
- `tiny_overfit/`：最多 10 张覆盖类别与复杂场景的样本。
- `tiny_overfit_hazard5/`：优先覆盖 hazard5 目标类别的独立小样本集；只有真实纯负图片存在时才加入纯负样本。

原始 `images`、`annotations`、RGB、Depth、JSON 永远只读。未被任何标注形状覆盖的像素为 255（ignore），不是 `unknown_background=0`。

hazard5 是独立任务定义：未被 shape 覆盖的像素为 `other=0`；显式 `ignore` 仍为 255。`other` 只表示非目标区域，绝不代表可通行。漏标的山、坑、坎或小车会被错误训练为 other，因此这些目标必须穷尽标注。可通行性后续由 Depth、elevation mapping 和几何规则判断。

## 检查问题与 overlay

优先查看：

- `reports/unmatched_rgb.csv`、`unmatched_depth.csv`、`unmatched_json.csv`；
- `reports/ambiguous_pairs.csv`；
- `reports/invalid_labels.csv`、`unsupported_shapes.csv`；
- `reports/validation_issues.csv` 和 `validation_report.json`。

逐张打开 `overlays/<session>/*_overlay.png` 检查边界、漏标与重叠。overlay 仅用于人工检查，不能作为训练标签。

## 重新运行

修正原始 Labelme JSON 后再次运行 PowerShell 脚本即可；派生文件按相同 sample ID 更新。配对报告和 manifest 每次都会重新生成。若曾移除样本，请以最新 manifest 为准，并人工清理不再引用的旧派生文件。

## 上传 tiny-overfit 到 Linux GPU 服务器

在 Windows PowerShell 中可使用（替换用户名和主机）：

```powershell
scp -r D:\mars_annotation\dataset_generated\tiny_overfit user@gpu-server:/data/mars_annotation/
```

也可用 WinSCP/SFTP。上传后核对 `tiny_overfit.csv` 与文件数量。

## 与 PIDNet/Depth 的边界

当前流水线只准备数据，不训练 PIDNet。本次在 PIDNet 工程中新增了 `pidnet_s_mars_hazard5.yaml`，配置为 `num_classes=5`、`ignore_label=255`，但没有启动训练。hazard5 仍只读取 RGB 与 hazard5 mask，Depth 暂不作为网络输入。Depth 保留原始 16 位数据，后续用于结合 CameraInfo、TF 等判断可通行性并把预测语义像素投影到三维空间。
