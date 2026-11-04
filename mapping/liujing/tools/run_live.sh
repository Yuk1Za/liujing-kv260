#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=4 OMP_WAIT_POLICY=PASSIVE
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$project_root/python:${PYTHONPATH:-}"
# One LiuJing consumer; upstream LiDAR is separately started after calibration.
exec flock -n "$project_root/live.lock" python3 "$project_root/python/live.py" "$@"
