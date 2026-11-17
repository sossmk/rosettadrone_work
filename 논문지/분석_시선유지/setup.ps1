param(
    # cu128: RTX 50 series (Blackwell) and newer. cu124 / cu121 for older GPUs.
    [string]$Cuda = "cu128"
)
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (-not (Test-Path ".venv")) {
    Write-Host "[1/5] Creating .venv"
    python -m venv .venv
}
$py = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
& $py -m pip install --upgrade pip

Write-Host "[2/5] Installing PyTorch ($Cuda)"
& $py -m pip install torch torchvision --index-url "https://download.pytorch.org/whl/$Cuda"

Write-Host "[3/5] Installing requirements (SAM2, rosbags, OpenCV ...)"
$env:SAM2_BUILD_CUDA = "0"   # skip optional CUDA extension build (fails on Windows without MSVC)
& $py -m pip install -r requirements.txt

Write-Host "[4/5] Downloading SAM2.1 small checkpoint"
New-Item -ItemType Directory -Force "checkpoints" | Out-Null
$ckpt = "checkpoints\sam2.1_hiera_small.pt"
if (-not (Test-Path $ckpt)) {
    $ProgressPreference = "SilentlyContinue"
    Invoke-WebRequest -Uri "https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_small.pt" -OutFile $ckpt
}

Write-Host "[5/5] Checking GPU"
& $py -c "import torch, sam2, rosbags, cv2; print('torch', torch.__version__, '| cuda', torch.cuda.is_available(), '|', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NO GPU')"
