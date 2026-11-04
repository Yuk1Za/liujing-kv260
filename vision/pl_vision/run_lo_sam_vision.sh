#!/bin/bash
# 启动「雷达管线 + 视觉分支」：lo_sam 原管线 + PL 视觉节点（attach 同一个 PL）
# 用法（板端，需 root）： sudo bash run_lo_sam_vision.sh [rate]
RATE=${1:-1.5}

source /opt/ros/humble/setup.bash
source /home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/setup.bash 2>/dev/null
export AMENT_PREFIX_PATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install:$AMENT_PREFIX_PATH
export COLCON_PREFIX_PATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install
export LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib:/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/lib:${LD_LIBRARY_PATH:-}
export XILINX_XRT=/opt/xilinx/xrt

# 硬规矩：加载任何设计前先清 PYNQ 元数据缓存（否则寄存器地址错位 -> SoC 停等）
PLSRV=/usr/local/share/pynq-venv/lib/python3.10/site-packages/pynq/pl_server
rm -f $PLSRV/_current_metadata.pkl $PLSRV/global_pl_state.json

# 清残留
for p in "kitti_pointcloud_[p]ublisher" "feature_extrac[t]" "pynq_feature_[e]xtraction" \
         "pldataproces[s]" "laserOdometr[y]" "map_optimizatio[n]" "ros2 launc[h]" \
         "pl_vision_nod[e]"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 2

echo ">>> 启动 lo_sam + 视觉分支，rate=$RATE"
exec ros2 launch /home/ubuntu/pl_vision/lo_sam_vision.launch.py rate:="$RATE"
