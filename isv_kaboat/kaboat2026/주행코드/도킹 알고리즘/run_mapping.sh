#!/usr/bin/env bash
set -eo pipefail
MAPPING_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source /home/isv/ros2_ws/install/setup.bash
cd "$MAPPING_ROOT"
exec "$MAPPING_ROOT/.venv/bin/python" -m docking_mapping.node "$@"
