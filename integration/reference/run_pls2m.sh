#!/bin/bash
# run_pls2m.sh —— 起 s2m 常驻服务 + 跑回放（surf 的 H/b 走 PL）
#
# 顺序不能反：srv 先起（分配 DMA、mmap shm），PL 节点后起会重下位流清寄存器，
#             但 srv 每次请求都重写全部寄存器 ⇒ 能恢复。
# 用法: sudo bash run_pls2m.sh [rate] [kitti_dir]
#   rate 默认 1.5
#   kitti_dir 默认 /home/ubuntu/_kitti_incoming/velodyne_points/data（street1 C16 转换）
set -u
RATE="${1:-1.5}"
KITTI_DIR="${2:-/home/ubuntu/_kitti_incoming/velodyne_points/data}"
KITTI_TS="$(dirname "$KITTI_DIR")/timestamps.txt"
SRV=/dev/shm/s2m_srv.py

export MAP_PL_S2M=1
export VMAP="${VMAP:-1}"

# 清残留服务
for p in $(ps -eo pid,args --no-headers | grep "[s]2m_srv.py" | awk '{print $1}'); do
    kill -9 "$p" 2>/dev/null
done
sleep 1
rm -f /dev/shm/s2m_shm

echo ">>> 先起回放（PL 节点负责下载位流）  kitti_dir=$KITTI_DIR"
bash /home/ubuntu/run_demo.sh "$RATE" "$KITTI_DIR" "$KITTI_TS" > /home/ubuntu/run_demo_s2m.log 2>&1 &
DEMO_PID=$!

echo ">>> 等 PL 节点把位流加载完（22 s）"
sleep 22
if ! kill -0 "$DEMO_PID" 2>/dev/null; then
    echo "!! 回放已退出，日志尾部："
    tail -15 /home/ubuntu/run_demo_s2m.log
    exit 1
fi

echo ">>> 起 s2m 服务（--reuse 复用已加载位流，绝不重下）"
/usr/local/share/pynq-venv/bin/python3 "$SRV" --reuse > /home/ubuntu/s2m_srv.log 2>&1 &
SRV_PID=$!
sleep 8
if ! kill -0 "$SRV_PID" 2>/dev/null; then
    echo "!! s2m 服务没起来："
    tail -12 /home/ubuntu/s2m_srv.log
    kill -9 "$DEMO_PID" 2>/dev/null
    exit 1
fi
ls -l /dev/shm/s2m_shm 2>/dev/null || echo "!! shm 未创建"
echo ">>> s2m 服务 pid=$SRV_PID 就绪（map 有 25 s 预热窗，不会早调）"

wait "$DEMO_PID"
echo ">>> 回放结束"
echo "--- s2m 服务统计 ---"
grep -E "nblk|服务|PL|err" /home/ubuntu/s2m_srv.log 2>/dev/null | tail -5
echo "--- map 侧 PL-S2M 日志 ---"
grep "PL-S2M" /home/ubuntu/run_demo_s2m.log 2>/dev/null | tail -5
