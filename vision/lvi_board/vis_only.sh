#!/bin/bash
# 隔离实验：只跑视觉节点 + losam publisher（抽稀话题），不启雷达链。
# 目的：判定"|Tvis| 只有应有值的 1/4~1/14"是**抽稀点云导致赋深变差**，还是共存 CPU 争抢。
# 用法: sudo bash vis_only.sh [frames] [vis_stride] [every] [tag]
set +u
source /home/ubuntu/lo_sam_env.sh >/dev/null 2>&1 || source /opt/ros/humble/setup.bash
set -u
export HOME=/home/ubuntu ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/build/lo_sam/rosidl_generator_py:$PYTHONPATH
cd /home/ubuntu/lvi_board || exit 1
N=${1:-60}; VS=${2:-8}; EV=${3:-1}; TAG=${4:-vo}
VDIR=/home/ubuntu/FPT_Competition/LO-SAM-ROS2/2011_09_30_drive_0027_sync/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points
rm -f "vo2_${TAG}.txt"
sudo python3 warm_pl.py /home/ubuntu/vis3/design_1_wrapper_fusion.bit >/dev/null 2>&1

PL_DOWNLOAD=1 PL_CLEAR_CACHE=0 python3 vo_pnp_node.py --frontend stcv2 --every "$EV" --max_cnt 60 \
  --min_dist 20 --nms auto --img_pre /home/ubuntu/kitti_pre --pl_q 4 \
  --pl_bit /home/ubuntu/vis3/design_1_wrapper_fusion.bit --pc_topic /velodyne_points_vis \
  --dump "vo2_${TAG}.txt" > "ab2_${TAG}.log" 2>&1 &
VPID=$!
for i in $(seq 1 90); do grep -q "vo_pnp 就绪" "ab2_${TAG}.log" 2>/dev/null && break; sleep 1; done
echo "[vo-only] 节点就绪（${i}s）"
VIS_STRIDE="$VS" timeout -k 5 200 python3 kitti_pub_losam_vis.py "$VDIR/data" "$VDIR/timestamps.txt" 5 "$N" > "pub2_${TAG}.log" 2>&1
sleep 2; kill -INT $VPID 2>/dev/null; sleep 3; kill -9 $VPID 2>/dev/null
grep -a "汇总" "ab2_${TAG}.log" | tail -1
