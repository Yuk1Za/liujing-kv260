#!/bin/bash
# 用法: run_gate_ab.sh <npz> <秒数> <日志> [额外 launch 参数]
# 与 run_vis.sh 的区别：先起 rec_vision.py 录 flow/雷达真值，再用 PID 收尾（不用 pkill -f，避免误杀自己）
OUT=$1; SECS=$2; LOG=$3; shift 3; EXTRA="$*"
source /opt/ros/humble/setup.bash
source /home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/setup.bash 2>/dev/null
export AMENT_PREFIX_PATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install:$AMENT_PREFIX_PATH
export COLCON_PREFIX_PATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/lib:${LD_LIBRARY_PATH:-}
rm -f "$OUT"
python3 /home/ubuntu/pl_vision/rec_vision.py "$OUT" > "${OUT%.npz}.rec.log" 2>&1 &
RP=$!
sleep 4
/home/ubuntu/merge_st/run_vis.sh -1 "$SECS" "$LOG" $EXTRA
sleep 2
kill -INT $RP 2>/dev/null   # SIGTERM 会被 rclpy.spin 吞掉，必须 SIGINT
wait $RP 2>/dev/null
ls -la "$OUT" 2>&1
tail -2 "${OUT%.npz}.rec.log"
