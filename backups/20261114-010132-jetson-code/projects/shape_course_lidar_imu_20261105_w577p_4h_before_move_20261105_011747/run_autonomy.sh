#!/bin/bash
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=50 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_HOME="$ROOT/ros_home" ROS_LOG_DIR="$ROOT/logs/ros"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
source /opt/ros/humble/setup.bash
source /home/isv/ros2_ws/install/setup.bash
source "$ROOT/install/setup.bash"
cd "$ROOT"
exec ros2 launch isv_shape_navigation course_mission.launch.py params_file:="$ROOT/src/isv_shape_navigation/config/course_mission.yaml" enable_output:=true
