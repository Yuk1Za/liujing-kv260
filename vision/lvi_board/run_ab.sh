#!/bin/bash
# 板上 A/B：kitti_pub 回放 → vo_pnp_node（--frontend cv2|pl）→ 逐条相对位姿
# 用法: sudo bash run_ab.sh <frontend> <tag> [frames] [every]
#   例: sudo bash run_ab.sh cv2 base 200 4
#       sudo bash run_ab.sh pl  pl3  200 4
# 环境: PL_LEVELS=3 / PL_ITERS=5 可覆盖
#
# 三个必须保留的细节（都踩过）：
#  1) 按 PID kill，不用 pkill -f（板上 pkill -f 会匹配到 ssh 自己的命令行）
#  2) export HOME=/home/ubuntu —— sudo 下 $HOME=/root，kitti_pub.py 用 ~ 展开找数据
#     会全部"缺数据，跳过"，节点收不到任何点云（现象：dump 空、进程不退）
#  3) ROS_LOCALHOST_ONLY=1 —— 点云 ~976 KB/帧 × 10 Hz 走 eth0 会把 sshd 饿死
set +u
source /opt/ros/humble/setup.bash
set -u
export HOME=/home/ubuntu
export ROS_LOCALHOST_ONLY=1

cd /home/ubuntu/lvi_board || exit 1
FE=${1:?用法: run_ab.sh <cv2|pl> <tag> [frames] [every]}
TAG=${2:?缺 tag}
N=${3:-200}
EVERY=${4:-4}
LV=${PL_LEVELS:-3}
IT=${PL_ITERS:-5}
LOG=/home/ubuntu/lvi_board/ab_${TAG}.log
DUMP=/home/ubuntu/lvi_board/traj_${TAG}.txt
PLOG=/home/ubuntu/lvi_board/pub_${TAG}.log
rm -f "$DUMP"

echo "[ab] frontend=$FE tag=$TAG frames=$N every=$EVERY pl_levels=$LV pl_iters=$IT pl_th=${PL_TH:-20} pl_q=${PL_Q:-4} st_th=${ST_TH:-0} max_cand=${PL_MC:-0} img_pre=${IMG_PRE_DIR:-none} nms=${NMS_MODE:-auto} bit=$(basename ${PL_BIT:-/home/ubuntu/vis3/design_1_wrapper.bit}) out_frame=${OUT_FRAME:-cam} pc=${PC_TOPIC:-/velodyne_points} pub_vs=${PUB_VS:-4}"
python3 vo_pnp_node.py --frontend "$FE" --every "$EVERY" --max_cnt 60 --min_dist 20 \
    --pl_levels "$LV" --pl_iters "$IT" --pl_th "${PL_TH:-20}" --pl_q "${PL_Q:-4}" \
    --st_th "${ST_TH:-0}" --max_cand "${PL_MC:-0}" --img_pre "${IMG_PRE_DIR:-}" --nms "${NMS_MODE:-auto}" \
    --pl_bit "${PL_BIT:-/home/ubuntu/vis3/design_1_wrapper.bit}" --dump "$DUMP" \
    --out_frame "${OUT_FRAME:-cam}" --pc_topic "${PC_TOPIC:-/velodyne_points}" > "$LOG" 2>&1 &
VO=$!
sleep 6
if ! kill -0 "$VO" 2>/dev/null; then
  echo "[错误] 节点启动即退出："
  tail -25 "$LOG"
  exit 1
fi

timeout -k 5 420 python3 kitti_pub.py --W "$N" --stride 2 --rate 10 \
    --vis_stride "${PUB_VS:-4}" --vis_topic /velodyne_points_vis > "$PLOG" 2>&1
echo "[ab] publisher 结束 rc=$?  回放日志尾 2 行:"
tail -2 "$PLOG"
sleep 2
kill -INT "$VO" 2>/dev/null
sleep 4
kill -9 "$VO" 2>/dev/null

echo "=== ab_${TAG}.log 尾 16 行 ==="
tail -16 "$LOG"
echo "=== dump: $(wc -l < "$DUMP" 2>/dev/null || echo 0) 行 ==="
