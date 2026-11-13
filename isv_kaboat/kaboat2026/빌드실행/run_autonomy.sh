#!/bin/bash
set -e
ROOT="$(cd "$(dirname "$0")" && cd .. && pwd)"
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=50 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_HOME="$ROOT/.runtime/ros_home" ROS_LOG_DIR="$ROOT/기록/logs/ros"
export DISPLAY="${DISPLAY:-:0}"
# Forwarded SSH displays use ssh/xauth credentials, not the Jetson desktop cookie.
if [[ "$DISPLAY" == :0 || "$DISPLAY" == :0.0 ]]; then
  export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
fi
source /opt/ros/humble/setup.bash
source /home/isv/ros2_ws/install/setup.bash
source "$ROOT/.runtime/install/setup.bash"
cd "$ROOT"
exec ros2 launch isv_shape_navigation course_mission.launch.py params_file:="$ROOT/공통모듈/isv_shape_navigation/config/course_mission.yaml" enable_output:=true
