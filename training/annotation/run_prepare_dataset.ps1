$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$Root = "D:\mars_annotation"
$PythonCommand = Get-Command python -ErrorAction SilentlyContinue
if (-not $PythonCommand) {
    throw "未找到 python。请先执行 conda activate mars-labelme。"
}

& python -c "import numpy; import PIL"
if ($LASTEXITCODE -ne 0) {
    throw "缺少必要依赖。请运行: python -m pip install -r `"$Root\requirements-data.txt`""
}

$Scripts = @(
    "inspect_dataset.py",
    "build_manifest.py",
    "convert_labelme_masks.py",
    "validate_dataset.py",
    "make_tiny_overfit.py"
)

foreach ($Script in $Scripts) {
    Write-Host "`n=== Running $Script ===" -ForegroundColor Cyan
    & python (Join-Path $Root "tools\$Script") --root $Root
    if ($LASTEXITCODE -ne 0) {
        throw "$Script 失败，退出码 $LASTEXITCODE。流水线已停止。"
    }
}

Write-Host "`n数据准备完成。" -ForegroundColor Green
Write-Host "生成目录: $Root\dataset_generated"
Write-Host "清单: $Root\dataset_generated\manifests\all_samples.csv"
Write-Host "报告: $Root\dataset_generated\reports"
Write-Host "tiny-overfit: $Root\dataset_generated\tiny_overfit"
