#!/bin/bash
# 共存归因：雷达链 + publisher + 视觉节点（按 mode 决定视觉是否碰 PL）
# 用法: sudo bash co_test.sh <mode> [frames] [rate]
#   mode=none  不起视觉（基线）
#   mode=cpu   视觉跑 cv2 前端 + Python NMS  ⇒ **完全不碰 PL**
#   mode=pl    视觉跑 stcv2 + 硬件 NMS     ⇒ 碰 PL（co_run 的配置）
set +u
source /home/ubuntu/lo_sam_env.sh >/dev/null 2>&1 || source /opt/ros/humble/setup.bash
set -u
export HOME=/home/ubuntu ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/build/lo_sam/rosidl_generator_py:$PYTHONPATH
export FAST=1 VMAP=1 VIS_FACTOR=0
cd /home/ubuntu/lvi_board || exit 1

MODE=${1:-none}
N=${2:-40}
RATE=${3:-5}
TAG=ct_${MODE}
VDIR=/home/ubuntu/FPT_Competition/LO-SAM-ROS2/2011_09_30_drive_0027_sync/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points
BIT=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/share/lo_sam/bitstreams/feature_extraction.bit

setsid ros2 launch lo_sam_coop.launch.py rate:="$RATE" > "losam_${TAG}.log" 2>&1 &
LPID=$!
sleep 18

VPID=0
if [ "$MODE" = "cpu" ]; then
  PL_DOWNLOAD=0 PL_CLEAR_CACHE=0 python3 vo_pnp_node.py --frontend cv2 --nms off \
    --every 1 --max_cnt 60 --min_dist 20 --img_pre /home/ubuntu/kitti_pre \
    --pl_q 4 --pl_bit "$BIT" --out_frame velo --pc_topic /velodyne_points_vis \
    --dump "vo_${TAG}.txt" > "ab_${TAG}.log" 2>&1 &
  VPID=$!
elif [ "$MODE" = "pl" ]; then
  PL_DOWNLOAD=0 PL_CLEAR_CACHE=0 python3 vo_pnp_node.py --frontend stcv2 --nms auto \
    --every 1 --max_cnt 60 --min_dist 20 --img_pre /home/ubuntu/kitti_pre \
    --pl_q 4 --pl_bit "$BIT" --out_frame velo --pc_topic /velodyne_points_vis \
    --dump "vo_${TAG}.txt" > "ab_${TAG}.log" 2>&1 &
  VPID=$!
fi
if [ "$VPID" != "0" ]; then
  for i in $(seq 1 90); do grep -q "vo_pnp 就绪" "ab_${TAG}.log" 2>/dev/null && break; sleep 1; done
  echo "[ct] 视觉就绪 ${i}s"
fi

TAG_INDEX=1 VIS_STRIDE=8 VIS_TOPIC=/velodyne_points_vis \
  timeout -k 5 300 python3 kitti_pub_losam_vis.py "$VDIR/data" "$VDIR/timestamps.txt" "$RATE" "$N" \
  > "pub_${TAG}.log" 2>&1
echo "[ct] mode=$MODE 回放结束"

sleep 2
[ "$VPID" != "0" ] && kill -INT "$VPID" 2>/dev/null
sleep 2
[ "$VPID" != "0" ] && kill -9 "$VPID" 2>/dev/null
kill -INT -"$LPID" 2>/dev/null; sleep 4
kill -9 -"$LPID" 2>/dev/null
for p in kitti_pointcloud_publisher feature_extract pynq_feature_extraction pldataprocess laserOdometry map_optimization; do
  for pid in $(ps -eo pid,comm | awk -v n="$p" '$2==n{print $1}'); do kill -9 "$pid" 2>/dev/null; done
done

echo "=== mode=$MODE ==="
echo "-- 前 3 帧 --"
grep -a "Published:" "losam_${TAG}.log" | head -3
echo "-- 0特征/总 --"
echo "$(grep -a 'Published:' "losam_${TAG}.log" | grep -c '0 flat') / $(grep -ac 'Published:' "losam_${TAG}.log")"
echo "-- ORD 前 2 --"
grep -a "\[ORD\]" "losam_${TAG}.log" | head -2
echo "-- 视觉汇总 --"
grep -a "汇总" "ab_${TAG}.log" | tail -1
