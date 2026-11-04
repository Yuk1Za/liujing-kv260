#!/bin/bash
# 隔离测试：coop launch（雷达链）+ 我们的 kitti_pub_losam_vis.py，**不起视觉节点**。
# 目的：判定 co_run 里 PL "0 flat / 0 sharp" 是不是我们的 publisher 造成的。
# 用法: sudo bash pub_test.sh [TAG_INDEX] [frames] [rate]
set +u
source /home/ubuntu/lo_sam_env.sh >/dev/null 2>&1 || source /opt/ros/humble/setup.bash
set -u
export HOME=/home/ubuntu ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/build/lo_sam/rosidl_generator_py:$PYTHONPATH
export FAST=1 VMAP=1 VIS_FACTOR=0
cd /home/ubuntu/lvi_board || exit 1

TI=${1:-1}
N=${2:-40}
RATE=${3:-5}
TAG=pubt_ti${TI}
VDIR=/home/ubuntu/FPT_Competition/LO-SAM-ROS2/2011_09_30_drive_0027_sync/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points

setsid ros2 launch lo_sam_coop.launch.py rate:="$RATE" > "losam_${TAG}.log" 2>&1 &
LPID=$!
sleep 18

TAG_INDEX="$TI" VIS_STRIDE=8 VIS_TOPIC=/velodyne_points_vis \
  timeout -k 5 300 python3 kitti_pub_losam_vis.py "$VDIR/data" "$VDIR/timestamps.txt" "$RATE" "$N" \
  > "pub_${TAG}.log" 2>&1
echo "[pubt] TAG_INDEX=$TI 发布结束 rc=$?"

sleep 2
kill -INT -"$LPID" 2>/dev/null; sleep 4
kill -9 -"$LPID" 2>/dev/null
for p in kitti_pointcloud_publisher feature_extract pynq_feature_extraction pldataprocess laserOdometry map_optimization; do
  for pid in $(ps -eo pid,comm | awk -v n="$p" '$2==n{print $1}'); do kill -9 "$pid" 2>/dev/null; done
done

echo "=== TAG_INDEX=$TI 结果 ==="
echo "-- 前 3 帧 --"
grep -a "Published:" "losam_${TAG}.log" | head -3
echo "-- 0特征 / 总 --"
echo "$(grep -a 'Published:' "losam_${TAG}.log" | grep -c '0 flat') / $(grep -ac 'Published:' "losam_${TAG}.log")"
echo "-- 中间一帧 --"
grep -a "Published:" "losam_${TAG}.log" | sed -n '10p'
echo "-- ORD --"
grep -a "\[ORD\]" "losam_${TAG}.log" | head -2
echo "-- pub 尾 --"
tail -2 "pub_${TAG}.log"
