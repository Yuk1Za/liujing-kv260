#!/usr/bin/env bash
set -eo pipefail
cd /home/ubuntu/liujing
source /opt/ros/humble/setup.bash
source /home/ubuntu/sensor_bringup_20261021/lidar_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1
if pgrep -f '(^|/)(liujing_camera|liujing_native|camera_ros2|lslidar_driver_node)( |$)' >/dev/null; then echo 'Existing sensor process: refusing duplicate'; exit 2; fi
out=${1:?new output path required}
test ! -e "$out"
log_dir="${out}_logs"
mkdir "$log_dir"
./sensors_build/liujing_camera >"$log_dir/camera.log" 2>&1 & camera_pid=$!
/home/ubuntu/sensor_bringup_20261021/lidar_ws/install/lslidar_driver/lib/lslidar_driver/lslidar_driver_node --ros-args --params-file /home/ubuntu/sensor_bringup_20261021/c16.yaml >"$log_dir/lidar.log" 2>&1 & lidar_pid=$!
cleanup() { kill -INT "$camera_pid" "$lidar_pid" 2>/dev/null || true; wait "$camera_pid" "$lidar_pid" 2>/dev/null || true; }
trap cleanup EXIT
python3 tools/capture_alignment.py --out "$out" --seconds 25
