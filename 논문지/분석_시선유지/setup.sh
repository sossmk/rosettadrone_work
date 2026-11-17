#!/usr/bin/env bash
# Linux equivalent of setup.ps1. Usage: bash setup.sh [cu128|cu124]
set -euo pipefail
cd "$(dirname "$0")"
# ROS 2 를 source 한 셸이면 PYTHONPATH 에 /opt/ros/... 가 들어 있다. 그대로 두면 pip 가
# 거기 설치된 numpy·cv2 를 보고 엉뚱한 의존성을 따지고, 분석 스크립트도 그쪽을 먼저 집는다.
unset PYTHONPATH
CUDA="${1:-cu128}"   # cu128: RTX 50 series (Blackwell) and newer

echo "[1/5] Creating .venv"
[ -d .venv ] || python3 -m venv .venv
PY=".venv/bin/python"
"$PY" -m pip install --upgrade -q pip

echo "[2/5] Installing PyTorch ($CUDA)"
"$PY" -m pip install -q torch torchvision --index-url "https://download.pytorch.org/whl/$CUDA"

echo "[3/5] Installing requirements (SAM2, rosbags, OpenCV ...)"
export SAM2_BUILD_CUDA=0
"$PY" -m pip install -q -r requirements.txt

echo "[4/5] Downloading SAM2.1 small checkpoint"
mkdir -p checkpoints
[ -f checkpoints/sam2.1_hiera_small.pt ] || \
  curl -fL --progress-bar -o checkpoints/sam2.1_hiera_small.pt \
    https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_small.pt

echo "[5/5] Checking GPU"
echo "    (스크립트를 돌릴 때도 PYTHONPATH 를 비운다: env -u PYTHONPATH .venv/bin/python ...)"
"$PY" -c "import torch, sam2, rosbags, cv2; print('torch', torch.__version__, '| cuda', torch.cuda.is_available(), '|', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NO GPU')"
