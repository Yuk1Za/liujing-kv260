#!/bin/bash
# 板上端到端测试：kitti_pub 回放 → vo_pnp_node 出相对位姿
# 用法: bash run_ros_test.sh [帧数=120] [every=4]
# 注意：按 PID kill，不用 pkill -f（板上 pkill -f 会匹配到 ssh 自己的命令行）
cd "$HOME/lvi_board" || exit 1
# ★ ROS 的 setup.bash 会引用未定义变量（AMENT_TRACE_SETUP_FILES），
#   与 set -u 冲突 ⇒ source 前后临时关掉 nounset
set +u
source /opt/ros/humble/setup.bash
set -u
N=${1:-120}
EVERY=${2:-4}

python3 vo_pnp_node.py --every "$EVERY" --dump "$HOME/lvi_board/traj.txt" > vo.log 2>&1 &
VO=$!
sleep 3
if ! kill -0 "$VO" 2>/dev/null; then
  echo "[错误] vo 节点启动即退出："
  tail -12 vo.log
  exit 1
fi

timeout 150 python3 kitti_pub.py --W "$N" --rate 10 > pub.log 2>&1
sleep 2
kill -INT "$VO" 2>/dev/null
sleep 3
kill -9 "$VO" 2>/dev/null

echo "=== vo_pnp_node.log（尾 20 行）==="
tail -20 vo.log
echo
echo "=== kitti_pub.log（尾 3 行）==="
tail -3 pub.log
