#!/usr/bin/env bash
# Attach a bounded camera/pose capture to an already running lidar-only session.
# Never launches a second camera, changes networking or reconfigures PL.
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
source /opt/ros/humble/setup.bash
set -u
out=${1:?new output directory required}
seconds=${2:-120}
test ! -e "$out"
if pgrep -f '(^|/)(liujing_camera|liujing_native|camera_ros2)( |$)' >/dev/null; then
  echo 'Camera already owned; stop the visual session before calibration capture.'; exit 2
fi
# Reuse the manager's process identity/boot checks instead of trusting a PID.
bash tools/online.sh status --engine native-v2 | python3 -c '
import json,sys
s=json.load(sys.stdin)
if not s.get("active") or not s.get("lidar_only"):raise SystemExit("An active lidar-only session is required")
if s.get("domain")!=42:raise SystemExit("Expected ROS domain 42")
'
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
log_dir="${out}_logs"
mkdir "$log_dir"
./sensors_build/liujing_camera >"$log_dir/camera.log" 2>&1 & camera_pid=$!
cleanup() { kill -INT "$camera_pid" 2>/dev/null || true; wait "$camera_pid" 2>/dev/null || true; }
trap cleanup EXIT
python3 tools/capture_alignment.py --out "$out" --seconds "$seconds" --sample-period .5 --record-poses
