#!/bin/bash
# 端到端：lo_sam 全管线 + 视觉分支（雷达节拍模式），跑约 95 s 后优雅停止
# 注意：ros2 topic echo 也要在 root 下运行，否则看不到 root 节点的话题（DDS 共享内存权限）
L=/home/ubuntu/pl_vision/logs
export PYTHONUNBUFFERED=1
mkdir -p "$L"
: > "$L/e2e.log"; : > "$L/sync_e2e.log"

source /opt/ros/humble/setup.bash
source /home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/setup.bash 2>/dev/null
export AMENT_PREFIX_PATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib:/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/lib:${LD_LIBRARY_PATH:-}

nohup setsid bash /home/ubuntu/pl_vision/run_lo_sam_vision.sh 1.5 > "$L/e2e.log" 2>&1 < /dev/null &
echo "launch started"

sleep 30
nohup setsid bash -c 'source /opt/ros/humble/setup.bash; timeout 95 ros2 topic echo /pl_vision/sync --field data > /home/ubuntu/pl_vision/logs/sync_e2e.log 2>&1' > /dev/null 2>&1 < /dev/null &

sleep 115
echo ">>> 优雅停止"
for p in "ros2 launc[h]" "pl_vision_nod[e]" "pynq_feature_[e]xtraction" "laserOdometr[y]" \
         "map_optimizatio[n]" "kitti_pointcloud_[p]ublisher" "feature_extrac[t]" "pldataproces[s]" "pl_vision"; do
  pkill -SIGTERM -f "$p" 2>/dev/null
done
sleep 6
for p in "ros2 launc[h]" "pl_vision_nod[e]" "pynq_feature_[e]xtraction" "laserOdometr[y]" \
         "map_optimizatio[n]" "kitti_pointcloud_[p]ublisher" "feature_extrac[t]" "pldataproces[s]"; do
  pkill -9 -f "$p" 2>/dev/null
done
echo DONE
