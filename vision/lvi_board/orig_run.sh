#!/bin/bash
# 对照组：用 lo_sam **自带** publisher（KITTI 时间戳、frame_id='velodyne'）跑原链，
# 只记录 map 的 odometry —— 用来判定"轨迹不动"是链路本身还是我们 publisher 的问题。
# 用法: sudo bash orig_run.sh [rate] [秒数]
set +u
source /home/ubuntu/lo_sam_env.sh >/dev/null 2>&1 || source /opt/ros/humble/setup.bash
set -u
export HOME=/home/ubuntu ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/build/lo_sam/rosidl_generator_py:$PYTHONPATH
export FAST=1 VMAP=1 VIS_FACTOR=0
RATE=${1:-2.0}
DUR=${2:-90}
cd /home/ubuntu/lvi_board || exit 1

setsid ros2 launch lo_sam lo_sam_demo.launch.py rate:="$RATE" > losam_orig.log 2>&1 &
LPID=$!
sleep 18
python3 rec_traj.py map_orig.txt "$DUR" > rec_orig.log 2>&1
kill -INT -"$LPID" 2>/dev/null; sleep 3; kill -9 -"$LPID" 2>/dev/null
for p in kitti_pointcloud_publisher feature_extract pynq_feature_extraction pldataprocess laserOdometry map_optimization; do
  for pid in $(ps -eo pid,comm | awk -v n="$p" '$2==n{print $1}'); do kill -9 "$pid" 2>/dev/null; done
done
tail -2 rec_orig.log
wc -l map_orig.txt
