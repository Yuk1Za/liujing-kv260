#!/bin/bash
MODE="$1"
R=/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install
source /opt/ros/humble/setup.bash
source $R/setup.bash
export AMENT_PREFIX_PATH=$R:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib:$R/lib:${LD_LIBRARY_PATH:-}
export LOSAM_BIT=/home/ubuntu/fusion_probe_pl/fusion.bit
export ODOM_SURF_LEAF=0.4 ODOM_NN=voxel ODOM_NN_LEAF=0.35
export MAP_COR=saes MAP_NN=voxel MAP_ESKF=inc
export VIS_FACTOR=1 VIS_SIG_ROT=0.06 VIS_SIG_TR=0.09
rm -f /dev/shm/fastrtps_* 2>/dev/null
for p in "[f]eature_extrac[t]" "[p]ynq_feature_extractio[n]" "[p]ldataproces[s]" "[l]aserOdometr[y]" "[m]ap_optimizatio[n]" "[c]pu_feature_extrac[t]" "[n]an_filter" "[r]os2 bag pla[y]" "[c]puprof"; do pkill -9 -f "$p" 2>/dev/null; done
sleep 2
O=/tmp/ab_$MODE
rm -f ${O}*
/usr/local/share/pynq-venv/bin/python3 /home/ubuntu/fusion_probe_pl/nan_filter.py > ${O}_nf.log 2>&1 &
sleep 2
if [ "$MODE" = "cpu" ]; then
  ros2 run lo_sam cpu_feature_extract --ros-args -p lidar_topic:=/rslidar_clean > ${O}_cpu.log 2>&1 &
else
  ros2 run lo_sam feature_extract --ros-args -p lidar_topic:=/rslidar_clean > ${O}_fe.log 2>&1 &
  sleep 2
  /usr/local/share/pynq-venv/bin/python3 $R/lib/lo_sam/pynq_feature_extraction.py > ${O}_pl.log 2>&1 &
  sleep 2
  ros2 run lo_sam pldataprocess --ros-args -p n_scans:=16 > ${O}_pld.log 2>&1 &
fi
sleep 4
ros2 run lo_sam laserOdometry > ${O}_odom.log 2>&1 &
ros2 run lo_sam map_optimization > ${O}_map.log 2>&1 &
sleep 14
env CLK_TCK=100 python3 /home/ubuntu/fusion_probe_pl/cpuprof.py 58 > ${O}_prof.txt 2>&1 &
ros2 bag play /home/ubuntu/m2dgr_plus/street1_ros2 --rate 1.0 > ${O}_bag.log 2>&1 &
sleep 62
echo "=== CPUFE frames (cpu mode) ==="; grep -c "CPUFE" ${O}_cpu.log 2>/dev/null
echo "=== PROF frames (pl mode) ==="; grep -c "\[PROF\]" ${O}_odom.log 2>/dev/null
echo "=== bag done ==="; tail -2 ${O}_bag.log 2>/dev/null
