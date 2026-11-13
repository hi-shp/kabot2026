#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [ "$#" -lt 1 ]; then printf 'Usage: bash sensor_test/run.sh {camera|lidar|gps|imu|arduino|thruster} [options]\n'; exit 2; fi
SENSOR="$1"
shift
case "$SENSOR" in camera|lidar|gps|imu|arduino|thruster) ;; *) printf 'Unknown sensor: %s\n' "$SENSOR" >&2; exit 2;; esac
export ROS_DOMAIN_ID=50 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
exec python3 "$TASK_DIR/${SENSOR}_test.py" "$@"
