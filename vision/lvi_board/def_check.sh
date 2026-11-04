#!/bin/bash
# 验证 map_optimization 里 VIS_FACTOR 的**源码默认值**（本脚本刻意 unset 相关环境变量）：
#   ① `[VIS] enabled` 行是否出现   ⇒ 默认是否真的打开
#   ② 打印的 sigma 是否 0.060/0.090 ⇒ 默认值是不是 A/B 选出来的那组
#   ③ **视觉节点完全不在跑**时：雷达链是否照常出特征、轨迹是否照常移动
#      （q 为空 ⇒ 每次插值失败直接 return，必须"不崩、不报错、不变慢"）
# 用法: sudo bash def_check.sh [duration_s] [rate]
set +u
source /home/ubuntu/lo_sam_env.sh >/dev/null 2>&1 || source /opt/ros/humble/setup.bash
set -u
export HOME=/home/ubuntu ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/build/lo_sam/rosidl_generator_py:$PYTHONPATH
export FAST=1 VMAP=1
export ODOM_SURF_LEAF=0.4 ODOM_NN=voxel ODOM_NN_LEAF=0.35
export MAP_COR=saes MAP_NN=voxel MAP_ESKF=inc MAP_ITER_MAX=10
export MAP_PLANE=vmap MAP_PLANE_NB=1 MAP_PLANE_PICK=1 MAP_PLANE_RMAX=0.4
export MAP_CORELINE=vmap
# ★ 关键：全部 unset，让二进制走自己的默认值
unset VIS_FACTOR VIS_SIG_ROT VIS_SIG_TR VIS_ROBUST VIS_TOL VIS_MAX_TR VIS_STAT
cd /home/ubuntu/lvi_board || exit 1

DUR=${1:-40}
RATE=${2:-2}
rm -f map_def.txt

setsid ros2 launch lo_sam lo_sam_demo.launch.py rate:="$RATE" > losam_def.log 2>&1 &
LPID=$!
sleep 18
python3 rec_traj.py map_def.txt "$DUR" > rec_def.log 2>&1
kill -INT -"$LPID" 2>/dev/null; sleep 4
kill -9 -"$LPID" 2>/dev/null
for p in kitti_pointcloud_publisher feature_extract pynq_feature_extraction pldataprocess laserOdometry map_optimization; do
  for pid in $(ps -eo pid,comm | awk -v n="$p" '$2==n{print $1}'); do kill -9 "$pid" 2>/dev/null; done
done

echo "=== ① [VIS] 默认值（应见 enabled + sig_rot=0.060 sig_tr=0.090）==="
grep -a "\[VIS\]" losam_def.log | head -2 || echo "!! 没有 [VIS] 行 ⇒ 默认没打开"
echo "=== ② 雷达链特征（0 flat 只应出现在预热帧）==="
echo "0特征/总: $(grep -a 'Published:' losam_def.log | grep -c '0 flat') / $(grep -ac 'Published:' losam_def.log)"
echo "=== ③ 轨迹是否照常移动 ==="
tail -3 rec_def.log
echo "条数 $(wc -l < map_def.txt)  末位姿: $(tail -1 map_def.txt)"
echo "=== ④ 有没有因为缺视觉而报错 ==="
grep -ac "vo_pnp\|插值\|VISDBG" losam_def.log
