#!/usr/bin/env bash
# Run the V2 waypoint mission against the existing sensor bringup.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
    "") output=false ;;
    --autonomy) output=true; shift ;;
    -h|--help) printf 'Usage: bash run_V2.sh [--autonomy]\n'; exit 0 ;;
    *) printf 'Usage: bash run_V2.sh [--autonomy]\n' >&2; exit 2 ;;
esac
if (( $# != 0 )); then
    printf 'Usage: bash run_V2.sh [--autonomy]\n' >&2
    exit 2
fi
source /opt/ros/humble/setup.bash
source "$ROOT/../../../install/setup.bash"
source "$ROOT/.runtime/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
exec python3 -m isv_shape_navigation.course_mission_v2 --ros-args \
    --params-file "$ROOT/설정/course_mission.yaml" \
    -p "enable_output:=$output" -p gui_enabled:=false
