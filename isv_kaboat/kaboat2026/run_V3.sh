#!/usr/bin/env bash
# V3: avoid obstacles to WP1, then GPS-hop to and hold at WP2.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
output=false
wait_start=false
wp1_lat=''; wp1_lon=''; wp2_lat=''; wp2_lon=''
usage='Usage: bash run_V3.sh [--autonomy] [--wait-start] [--wp1 LAT LON --wp2 LAT LON]'
while (( $# )); do
    case "$1" in
        --autonomy) output=true; shift ;;
        --wait-start) wait_start=true; shift ;;
        --wp1)
            if (( $# < 3 )); then printf '%s\n' "$usage" >&2; exit 2; fi
            wp1_lat="$2"; wp1_lon="$3"; shift 3 ;;
        --wp2)
            if (( $# < 3 )); then printf '%s\n' "$usage" >&2; exit 2; fi
            wp2_lat="$2"; wp2_lon="$3"; shift 3 ;;
        -h|--help) printf '%s\n' "$usage"; exit 0 ;;
        *) printf '%s\n' "$usage" >&2; exit 2 ;;
    esac
done
if [[ -n "$wp1_lat" && -z "$wp2_lat" || -z "$wp1_lat" && -n "$wp2_lat" ]]; then
    printf '%s\n' "$usage" >&2; exit 2
fi
source /opt/ros/humble/setup.bash
source "$ROOT/../../../install/setup.bash"
source "$ROOT/.runtime/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
params=(-p "enable_output:=$output" -p gui_enabled:=false -p wp_radius:=1.5)
if [[ -n "$wp1_lat" ]]; then
    params+=(-p "wp_latitudes:=[$wp1_lat,$wp2_lat,$wp2_lat]"
             -p "wp_longitudes:=[$wp1_lon,$wp2_lon,$wp2_lon]")
fi
python3 "$ROOT/빌드실행/launcher_check.py" --duplicates --components mission
if [[ "$output" == true && "$wait_start" == false ]]; then
    exec python3 "$ROOT/빌드실행/v3_cli_runner.py" --ros-args \
        --params-file "$ROOT/설정/course_mission.yaml" "${params[@]}"
fi
exec python3 -m isv_shape_navigation.course_mission_v3 --ros-args \
    --params-file "$ROOT/설정/course_mission.yaml" "${params[@]}"
