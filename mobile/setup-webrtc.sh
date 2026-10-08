#!/usr/bin/env bash
# Instala somente no projeto, sem pip global ou alterações no Python do ROS.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m venv --system-site-packages "$project_dir/.webrtc-venv"
touch "$project_dir/.webrtc-venv/COLCON_IGNORE"
env -u PYTHONPATH "$project_dir/.webrtc-venv/bin/python" -m pip install \
  -r "$project_dir/requirements-mobile-webrtc.txt"
