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

$Manifest = Join-Path $Root "dataset_generated\manifests\all_samples.csv"
if (-not (Test-Path $Manifest -PathType Leaf)) {
    throw "缺少 $Manifest。请先建立 manifest；为保护旧 mask，本脚本不会自动运行旧 mask 转换。"
}

& python (Join-Path $Root "tools\convert_hazard5_masks.py") --root $Root
if ($LASTEXITCODE -ne 0) {
    throw "hazard5 转换或校验失败，退出码 $LASTEXITCODE。"
}

Write-Host "`nhazard5 数据准备完成。" -ForegroundColor Green
Write-Host "masks: $Root\dataset_generated\masks_hazard5"
Write-Host "overlays: $Root\dataset_generated\overlays_hazard5"
Write-Host "report: $Root\dataset_generated\reports\hazard5_validation.json"
Write-Host "tiny-overfit: $Root\dataset_generated\tiny_overfit_hazard5"
