#!/bin/bash
# 端到端：雷达管线 + PL 视觉节点 + 视觉下游消费者，跑约 115 s
L=/home/ubuntu/pl_vision/logs
export PYTHONUNBUFFERED=1
mkdir -p "$L"
: > "$L/branch_e2e.log"; : > "$L/cmp.log"; : > "$L/vo.log"

source /opt/ros/humble/setup.bash
source /home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/setup.bash 2>/dev/null
export AMENT_PREFIX_PATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib:/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/lib:${LD_LIBRARY_PATH:-}

nohup setsid bash /home/ubuntu/pl_vision/run_lo_sam_vision.sh 1.5 > "$L/branch_e2e.log" 2>&1 < /dev/null &
echo "launch started"

sleep 25
nohup setsid bash -c 'source /opt/ros/humble/setup.bash; timeout 115 ros2 topic echo /pl_vision/vo_vs_lidar --field data > /home/ubuntu/pl_vision/logs/cmp.log 2>&1' > /dev/null 2>&1 < /dev/null &
nohup setsid bash -c 'source /opt/ros/humble/setup.bash; timeout 115 ros2 topic echo /pl_vision/vo --field pose.pose.orientation > /home/ubuntu/pl_vision/logs/vo.log 2>&1' > /dev/null 2>&1 < /dev/null &

sleep 130
echo ">>> 优雅停止"
for p in "ros2 launc[h]" "pl_vision_nod[e]" "pl_vision_branc[h]" "pynq_feature_[e]xtraction" "laserOdometr[y]" \
         "map_optimizatio[n]" "kitti_pointcloud_[p]ublisher" "feature_extrac[t]" "pldataproces[s]"; do
  pkill -SIGTERM -f "$p" 2>/dev/null
done
sleep 6
for p in "ros2 launc[h]" "pl_vision_nod[e]" "pl_vision_branc[h]" "pynq_feature_[e]xtraction" "laserOdometr[y]" \
         "map_optimizatio[n]" "kitti_pointcloud_[p]ublisher" "feature_extrac[t]" "pldataproces[s]"; do
  pkill -9 -f "$p" 2>/dev/null
done
echo DONE
