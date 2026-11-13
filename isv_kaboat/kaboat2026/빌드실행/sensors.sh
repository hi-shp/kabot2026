#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="${ISV_ROS_WS:-$HOME/ros2_ws}"
export ROS_DOMAIN_ID=50 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
build_all() { source "$HOME/.config/isv/ros2_helpers.bash"; cb; }
load_workspace() { source "$WS/install/setup.bash"; }
start_sensors() { load_workspace; ros2 run isv_water_trials tank_session start sensors -- use_outdoor:=false "$@"; }
action="${1:-status}"
if [ "$#" -gt 0 ]; then shift; fi
case "$action" in
 build) build_all ;;
 start) start_sensors "$@" ;;
 setup) build_all; start_sensors "$@"; python3 "$TASK_DIR/sensor_check.py" ;;
 status) load_workspace; ros2 run isv_water_trials tank_session status sensors; python3 "$TASK_DIR/sensor_check.py" ;;
 stop) load_workspace; ros2 run isv_water_trials tank_session stop sensors ;;
 *) printf 'Usage: %s {build|start|setup|status|stop} [launch arguments]\n' "$0"; exit 2 ;;
esac
