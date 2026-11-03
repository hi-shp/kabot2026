#!/usr/bin/env bash
set -e
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source /home/isv/ros2_ws/install/setup.bash
source /home/isv/shape_docking_red_triangle/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}"
export ROS_LOG_DIR=/home/isv/shape_docking_red_triangle/log/runtime
exec ros2 launch isv_shape_navigation shape_follow.launch.py "$@"
