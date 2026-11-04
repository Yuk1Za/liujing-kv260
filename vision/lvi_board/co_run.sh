#!/bin/bash
# 共存运行：lo_sam 全链（雷达）+ 视觉节点，**共用同一颗融合位流**（雷达+ST+NMS）
# 用法: sudo bash co_run.sh <tag> [frames] [rate] [vis_factor]
#   例: sudo bash co_run.sh vf0 100 5 0
# 关键约束：
#  1) 两端必须走**同一个 bit 路径**，否则后启动方会重下位流、清掉先启动方的寄存器
#     （已由 deploy_fusion.sh 覆盖 lo_sam 的 bitstreams/feature_extraction.bit 保证）
#  2) 视觉必须在雷达把位流加载好之后再 attach（sleep 16），且 **PL_CLEAR_CACHE=0**
#     （清 PYNQ 元数据缓存会逼它重新解析位流）
#  3) 按 PID / 进程组 kill，不用 pkill -f（板上会匹配到 ssh 自己的命令行）
set +u
# ★ 必须 source lo_sam 的环境：它把 install/ 补进 AMENT_PREFIX_PATH/COLCON_PREFIX_PATH/LD_LIBRARY_PATH，
#   否则 `ros2 launch lo_sam ...` 直接报 "package 'lo_sam' not found"（这个工作空间不是 colcon 装的）。
source /home/ubuntu/lo_sam_env.sh >/dev/null 2>&1 || source /opt/ros/humble/setup.bash
set -u
# ★★ `pynq_feature_extraction.py` 要 `from lo_sam.msg import ...`，而 lo_sam 的 python 消息
#   只存在于 build/lo_sam/rosidl_generator_py（install 里没有）⇒ 不补 PYTHONPATH 就
#   ModuleNotFoundError: No module named 'lo_sam'，该节点一启动就死、整条雷达链哑掉
#   （且看起来像"launch 起来了"，只有翻日志才看得到）。
export PYTHONPATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/build/lo_sam/rosidl_generator_py:$PYTHONPATH
export HOME=/home/ubuntu
export ROS_LOCALHOST_ONLY=1
cd /home/ubuntu/lvi_board || exit 1

TAG=${1:?用法: co_run.sh <tag> [frames] [rate] [vis_factor]}
N=${2:-100}
RATE=${3:-2}
VF=${4:-1}                 # ★ 默认开（与 map_optimization 源码默认一致）；纯激光对照传 0
VEVERY=${VEVERY:-1}        # 视觉唤醒间隔（1=每帧；4=2.5 Hz，省 CPU/功耗）
CPU_KHZ=${CPU_KHZ:-0}      # >0 时把 4 个核锁到该频率（可选 333333/444444/666666/1333333）
BIT=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/share/lo_sam/bitstreams/feature_extraction.bit
VDIR=/home/ubuntu/FPT_Competition/LO-SAM-ROS2/2011_09_30_drive_0027_sync/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points
rm -f "map_${TAG}.txt" "vo_${TAG}.txt"

export FAST=1 VMAP=1
# ★★★ 只设 FAST=1/VMAP=1 **不生效**：这两个标志只在 `run_demo.sh` 里被翻译成真正的环境变量
#   （ODOM_NN / MAP_COR / MAP_ESKF / MAP_PLANE …）。之前 co_run 直接用 `ros2 launch`，
#   于是 map_optimization 打印 `[A1] iter_max=30 ... mode=0` = **基座配置**，
#   共存吞吐被白白压在 ~0.8 Hz。这里把 run_demo.sh 的翻译逻辑复制进来。
if [ "${FAST:-0}" = "1" ]; then
  export ODOM_SURF_LEAF=0.4 ODOM_NN=voxel ODOM_NN_LEAF=0.35
  export MAP_COR=saes MAP_NN=voxel MAP_ESKF=inc MAP_ITER_MAX=10
fi
if [ "${VMAP:-0}" = "1" ]; then
  export MAP_ESKF=${MAP_ESKF:-inc}
  export MAP_PLANE=vmap MAP_PLANE_NB=1 MAP_PLANE_PICK=1 MAP_PLANE_RMAX=0.4
  export MAP_CORELINE=vmap
fi
export VIS_FACTOR="$VF" VIS_STAT=1
echo "[co] tag=$TAG frames=$N rate=$RATE VIS_FACTOR=$VF bit=$(md5sum $BIT | cut -c1-8)"

if [ "$CPU_KHZ" -gt 0 ] 2>/dev/null; then
  gov=/sys/devices/system/cpu/cpufreq/policy0/scaling_governor
  echo userspace > $gov 2>/dev/null
  echo "$CPU_KHZ" > /sys/devices/system/cpu/cpufreq/policy0/scaling_setspeed 2>/dev/null
  echo "[co] CPU 锁到 $(cat /sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq 2>/dev/null) kHz"
fi

setsid ros2 launch lo_sam_coop.launch.py rate:="$RATE" > "losam_${TAG}.log" 2>&1 &
LPID=$!
sleep 16

PL_CLEAR_CACHE=0 PL_DOWNLOAD=0 python3 vo_pnp_node.py --frontend stcv2 --every "$VEVERY" --max_cnt 60 --min_dist 20 --nms auto --img_pre /home/ubuntu/kitti_pre --pl_q 4 --pl_bit "$BIT" --out_frame velo --pc_topic /velodyne_points_vis --dump "vo_${TAG}.txt" > "ab_${TAG}.log" 2>&1 &
VPID=$!

python3 rec_traj.py "map_${TAG}.txt" > "rec_${TAG}.log" 2>&1 &
RPID=$!

# ★★ 必须等视觉节点**真正就绪**再开始回放：它的启动要 ~17 s（PYNQ attach + cv2/numpy 导入），
#   而 60 帧 @10 Hz 只发 6 s ⇒ 之前几轮节点订阅时数据已经发完（现象："收到 0 帧"、q=0）。
#   这里轮询它的就绪日志，最多等 90 s。
for _i in $(seq 1 90); do
    grep -q "vo_pnp 就绪" "ab_${TAG}.log" 2>/dev/null && break
    sleep 1
done
echo "[co] 视觉节点就绪等待结束（$_i s）"

# ★ 用 lo_sam 自带 publisher 的补丁版（KITTI 时间戳 + frame_id 带帧号 + 抽稀话题）：
#   实测用 wall-clock 时间戳会让后端轨迹恒为 0（详见 kitti_pub_losam_vis.py 文件头）。
# ★ 回放频率必须跟 `rate` 一致：这里硬编码过 10，而链只有 ~4.8 Hz ⇒ RELIABLE depth 10
#   队列溢出、帧被丢，"2 Hz 全帧处理"的意图完全没实现（实测 80 帧 12 s 跑完）。
VIS_STRIDE=8 VIS_TOPIC=/velodyne_points_vis timeout -k 5 400 python3 kitti_pub_losam_vis.py "$VDIR/data" "$VDIR/timestamps.txt" "$RATE" "$N" > "pub_${TAG}.log" 2>&1
echo "[co] publisher 结束 rc=$?  尾 2 行:"; tail -2 "pub_${TAG}.log"

sleep 3
kill -INT "$VPID" "$RPID" 2>/dev/null; sleep 3
kill -9 "$VPID" "$RPID" 2>/dev/null
kill -INT -"$LPID" 2>/dev/null; sleep 4
kill -9 -"$LPID" 2>/dev/null
for p in kitti_pointcloud_publisher feature_extract pynq_feature_extraction pldataprocess laserOdometry map_optimization; do
  for pid in $(ps -eo pid,comm | awk -v n="$p" '$2==n{print $1}'); do kill -9 "$pid" 2>/dev/null; done
done

echo "=== 汇总 ==="
tail -2 "rec_${TAG}.log"
echo "-- 雷达链 PL 特征（0 flat 计数 / 总帧） --"
echo "$(grep -a 'Published:' "losam_${TAG}.log" | grep -c '0 flat') / $(grep -ac 'Published:' "losam_${TAG}.log")"
echo "-- 配置确认（应见 iter_max=10 / mode=2） --"
grep -a "\[A1\]" "losam_${TAG}.log" | head -2
echo "-- map 轨迹条数 / 末位姿 --"
wc -l < "map_${TAG}.txt"; tail -1 "map_${TAG}.txt"
grep -a "汇总" "ab_${TAG}.log" | tail -1
echo "[VIS] 行数: $(grep -ac '\[VIS\]' "losam_${TAG}.log")"
grep -a "\[VIS\]" "losam_${TAG}.log" | head -6
