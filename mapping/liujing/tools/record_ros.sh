#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
: "${1:?usage: record_ros.sh NEW_OUTPUT_DIRECTORY}"
# Recording only: does not start a driver, load a bitstream, or issue motion.
exec ros2 bag record -o "$1" /point_cloud_raw /camera/color/image_raw /camera/depth/image_raw /camera/color/camera_info /camera/depth/camera_info /camera/frame_metadata /odom_init/laser_odom /mapping/odometry /imu /tf /tf_static
