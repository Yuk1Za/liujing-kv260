#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if test "$EUID" != 0; then exec sudo -n bash "$0" "$@"; fi
source /opt/ros/humble/setup.bash
source /home/ubuntu/losam_live_20261021/runtime/software/install/setup.bash
source /home/ubuntu/sensor_bringup_20261021/lidar_ws/install/setup.bash
export AMENT_PREFIX_PATH="/home/ubuntu/losam_live_20261021/runtime/software/install:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="$ROOT/python:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/usr/local/lib:/home/ubuntu/losam_live_20261021/runtime/software/install/lib:${LD_LIBRARY_PATH:-}"
export PATH="/usr/local/share/pynq-venv/bin:$PATH"
exec /usr/local/share/pynq-venv/bin/python3 "$ROOT/tools/online_session.py" "$@"
