#!/bin/bash
# 采集 odom_init/laser_odom 的 header.stamp，与 velodyne timestamps.txt 比对：
# 判断 312 ms 偏差是「流水线滞后（时间戳同一集合）」还是「时间戳被加了固定偏移」
L=/home/ubuntu/pl_vision/logs
export PYTHONUNBUFFERED=1
: > "$L/lidar_stamp.log"
source /opt/ros/humble/setup.bash
source /home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/setup.bash 2>/dev/null
export AMENT_PREFIX_PATH=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib:/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/lib:${LD_LIBRARY_PATH:-}

nohup setsid bash /home/ubuntu/pl_vision/run_lo_sam_vision.sh 1.5 > "$L/lidar_stamp_e2e.log" 2>&1 < /dev/null &
sleep 25
nohup setsid bash -c 'source /opt/ros/humble/setup.bash; timeout 60 ros2 topic echo /odom_init/laser_odom --field header.stamp > /home/ubuntu/pl_vision/logs/lidar_stamp.log 2>&1' > /dev/null 2>&1 < /dev/null &
sleep 75
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
