#!/bin/bash
# KV260 LO-SAM 演示一键启动（KITTI 回放）
# 用法: sudo ./run_demo.sh [rate] [kitti_dir] [timestamps]
#   rate       回放频率 Hz，默认 1.5
#   kitti_dir  bin 目录（如 /home/ubuntu/_kitti_incoming/velodyne_points/data），默认原 KITTI 0027
#   timestamps 时间戳文件，须与 kitti_dir 配套
# 演示配置：关回环 / mapping leaf size 0.8+0.4 / surroundingKeyframeSize=50
RATE="${1:-1.5}"
KITTI_DIR="${2:-}"
KITTI_TS="${3:-}"

source /opt/ros/humble/setup.bash
source /home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/setup.bash
export AMENT_PREFIX_PATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install:$AMENT_PREFIX_PATH
export COLCON_PREFIX_PATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install
export LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib:/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/lib:${LD_LIBRARY_PATH:-}

# final3 位流（含 s2m_accel_0 / sparse_lk）；不设则用老 feature_extraction.bit
export LOSAM_BIT="${LOSAM_BIT:-/home/ubuntu/final3/final3.bit}"

# 清掉上一轮残留，否则输入频率会翻倍、日志全乱
for p in "kitti_pointcloud_[p]ublisher" "feature_extrac[t]" "pynq_feature_[e]xtraction" \
         "pldataproces[s]" "laserOdometr[y]" "map_optimizatio[n]" "ros2 launc[h]"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 2

# [FAST=1] 启用 2026-10-18 全套已验证优化（A1-cor/A2/A3/A6，默认全部关闭 = 原版行为）
#   ODOM_SURF_LEAF=0.4  canonical LOAM 降采样（上游被删掉，属修实现缺陷）
#   ODOM_NN/LEAF        laserOdometry 1-NN 换体素栅格（8.9 倍建表、1.53 倍整帧）
#   MAP_COR=saes        corner 3x3 特征分解换 SelfAdjointEigenSolver（2.53x）
#   MAP_NN=voxel        scan-to-map 5-NN 换体素栅格（语义等价、轨迹逐字节相同）
#   MAP_ESKF=inc        局部地图增量维护（3.71x）
# 实测 8 Hz 端到端：leaf0.4 下 odom 2.27->2.91、map 1.29->2.09 帧/s
if [ "${FAST:-0}" = "1" ]; then
  export ODOM_SURF_LEAF=0.4 ODOM_NN=voxel ODOM_NN_LEAF=0.35
  export MAP_COR=saes MAP_NN=voxel MAP_ESKF=inc
  echo ">>> FAST=1：优化开关已启用（ODOM_SURF_LEAF=0.4 / ODOM_NN=voxel / MAP_COR=saes / MAP_NN=voxel / MAP_ESKF=inc）"
fi

# [VMAP=1] 体素平面/直线地图（A8/A9）：scan-to-map 的"5-NN + 每点现拟合"换成"读预存图元"
#   平面表从 A3 的增量桶里聚合 ⇒ **必须 MAP_ESKF=inc**（这里自动补上）
#   实测 1.5 Hz：map 单帧 160.7 → 132.7 ms（1.21×），轨迹差 61 mm / 165 m = 0.037%
#   实测 8 Hz  ：map 节点 CPU 96% → 57%，但出帧率基本不变（8 Hz 的瓶颈已在上游串行链）
#   增量/全量逐格对拍：miss=extra=bad=0、|Δd|=0、法向/方向差 ≤0.03°
if [ "${VMAP:-0}" = "1" ]; then
  export MAP_ESKF=${MAP_ESKF:-inc}
  export MAP_PLANE=vmap MAP_PLANE_NB=1 MAP_PLANE_PICK=1 MAP_PLANE_RMAX=0.4
  export MAP_CORELINE=vmap
  echo ">>> VMAP=1：体素平面(A8) + 体素直线(A9) 已启用（MAP_ESKF=inc 已自动保证）"
fi

# [MAP_PL_S2M=1] surf H/b 走 PL（s2m_accel_0）；须先起 s2m_srv（run_pls2m.sh 负责编排）
if [ "${MAP_PL_S2M:-0}" = "1" ]; then
  echo ">>> MAP_PL_S2M=1：surf 法方程走 PL（/dev/shm/s2m_shm）"
fi

LAUNCH_ARGS=(rate:="$RATE")
if [ -n "$KITTI_DIR" ]; then
  LAUNCH_ARGS+=(kitti_dir:="$KITTI_DIR")
  echo ">>> kitti_dir=$KITTI_DIR"
fi
if [ -n "$KITTI_TS" ]; then
  LAUNCH_ARGS+=(timestamps:="$KITTI_TS")
fi

echo ">>> 演示启动，回放频率 ${RATE} Hz  LOSAM_BIT=$LOSAM_BIT"
exec ros2 launch lo_sam lo_sam_demo.launch.py "${LAUNCH_ARGS[@]}"
