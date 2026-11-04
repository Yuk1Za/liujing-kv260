#!/bin/bash
# PL 视觉前端节点启动器
# 关键：必须用 pynq-venv 的 python（系统 python3 没有 pynq），
#       并在脚本内部 source ROS2 环境（sudo 会丢环境变量，所以 sudo 放在最外层）。
# 用法： sudo bash run_node.sh [--ros-args -p xxx:=yyy ...]
source /opt/ros/humble/setup.bash
source /home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/setup.bash 2>/dev/null
export AMENT_PREFIX_PATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install:$AMENT_PREFIX_PATH
export COLCON_PREFIX_PATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install
export LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib:/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/lib:${LD_LIBRARY_PATH:-}
export XILINX_XRT=/opt/xilinx/xrt
exec /usr/local/share/pynq-venv/bin/python3 /home/ubuntu/pl_vision/pl_vision_node.py "$@"
