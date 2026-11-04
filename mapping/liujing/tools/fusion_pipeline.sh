#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source /home/ubuntu/losam_live_20261021/runtime/software/install/setup.bash
source /home/ubuntu/sensor_bringup_20261021/lidar_ws/install/setup.bash
export AMENT_PREFIX_PATH="/home/ubuntu/losam_live_20261021/runtime/software/install:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="$ROOT/python:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/usr/local/lib:/home/ubuntu/losam_live_20261021/runtime/software/install/lib:${LD_LIBRARY_PATH:-}"
export XILINX_XRT=/usr
exec /usr/local/share/pynq-venv/bin/python3 "$ROOT/tools/fusion_session.py" "$@" --engine native-v2 --fusion-reference pipeline --backend pl --mode static_diagnostic --calibration "$ROOT/configs/live_candidate_20261026.json" --native-build "$ROOT/native_pipeline_protocol_v1_build" --bit "${LIUJING_FUSION_ARTIFACT:-$ROOT/artifacts/opaque_v1}/liujing.bit" --bit-manifest "${LIUJING_FUSION_ARTIFACT:-$ROOT/artifacts/opaque_v1}/manifest.json"
