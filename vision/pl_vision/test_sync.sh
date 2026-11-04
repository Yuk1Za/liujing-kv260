#!/bin/bash
# 隔离验证：kitti 点云发布器 + 视觉节点（雷达节拍模式），确认配对命中同序号帧
source /opt/ros/humble/setup.bash
source /home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/setup.bash 2>/dev/null
export AMENT_PREFIX_PATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install:$AMENT_PREFIX_PATH
export COLCON_PREFIX_PATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install
export LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib:/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/lib:${LD_LIBRARY_PATH:-}
export XILINX_XRT=/opt/xilinx/xrt
export PYTHONUNBUFFERED=1

V=/home/ubuntu/FPT_Competition/LO-SAM-ROS2/2011_09_30_drive_0027_sync/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points
L=/home/ubuntu/pl_vision/logs
mkdir -p "$L"

PLSRV=/usr/local/share/pynq-venv/lib/python3.10/site-packages/pynq/pl_server
rm -f $PLSRV/_current_metadata.pkl $PLSRV/global_pl_state.json

pkill -9 -f "kitti_pointcloud_[p]ublisher" 2>/dev/null
pkill -9 -f "pl_vision_nod[e]" 2>/dev/null
sleep 1

echo ">>> 起雷达点云发布器 (5 Hz)"
nohup setsid python3 /home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/lib/lo_sam/kitti_pointcloud_publisher.py \
  "$V/data" "$V/timestamps.txt" 5 > "$L/radar.log" 2>&1 < /dev/null &
sleep 4

echo ">>> 起视觉节点 (run_node.sh, 雷达节拍模式)"
nohup setsid bash /home/ubuntu/pl_vision/run_node.sh --ros-args \
  -p bit:=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/share/lo_sam/bitstreams/feature_extraction.bit \
  -p download:=true \
  -p sync_topic:=/velodyne_points \
  -p align:=stamp \
  -p velo_timestamps:="$V/timestamps.txt" \
  -p idle_exit_s:=25.0 > "$L/vision.log" 2>&1 < /dev/null &

sleep 4
echo ">>> 录 /pl_vision/sync 24 s"
timeout 24 ros2 topic echo /pl_vision/sync --field data > "$L/sync.log" 2>&1

echo ">>> 结束"
pkill -9 -f "kitti_pointcloud_[p]ublisher" 2>/dev/null
pkill -9 -f "pl_vision_nod[e]" 2>/dev/null
echo DONE
