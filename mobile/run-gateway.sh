#!/usr/bin/env bash
# Execute após source do ROS/workspace. Mantém os caminhos ROS, mas impede que
# PYTHONPATH carregue cryptography do apt junto com pyOpenSSL do ambiente isolado.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="$project_dir/.webrtc-venv/bin/python"
if [[ ! -x "$python_bin" ]]; then
  echo "Prepare o vídeo WebRTC: bash $project_dir/mobile/setup-webrtc.sh" >&2
  exit 1
fi
python_packages="$(env -u PYTHONPATH "$python_bin" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
export PYTHONPATH="$python_packages:$project_dir${PYTHONPATH:+:$PYTHONPATH}"
exec "$python_bin" -m drone_inspetor.mobile_gateway.server --webrtc "$@"
