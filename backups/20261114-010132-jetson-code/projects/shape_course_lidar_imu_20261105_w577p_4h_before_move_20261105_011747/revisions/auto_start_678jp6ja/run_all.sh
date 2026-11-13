#!/usr/bin/env bash
# Added launcher only; existing workspaces and launch files are not changed.
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODE=preview
DRY=false
for arg in "$@"; do
  case "$arg" in
    --preview) MODE=preview ;;
    --autonomy) MODE=autonomy ;;
    --dry-run) DRY=true ;;
    -h|--help)
      printf 'Usage: bash run_all.sh [--preview|--autonomy] [--dry-run]\n'
      printf 'Default: all sensors + YOLO + fusion + GUI, no actuator output.\n'
      printf 'Autonomy: output enabled, RC AUTO + GUI Start still required.\n'
      exit 0 ;;
    *) printf 'Unknown option: %s\n' "$arg" >&2; exit 2 ;;
  esac
done
export COURSE_WORKSPACE="$ROOT"
if [[ -f "$ROOT/all_in_one.env" ]]; then source "$ROOT/all_in_one.env"; fi
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
export ROS_HOME="$ROOT/ros_home"
export ROS_LOG_DIR="$ROOT/logs/ros"
export YOLO_CONFIG_DIR="$ROOT/yolo_config"
for setup in /opt/ros/humble/setup.bash "$HOME/ros2_ws/install/setup.bash" "$ROOT/install/setup.bash"; do
  if [[ ! -f "$setup" ]]; then printf 'Missing setup: %s\n' "$setup" >&2; exit 2; fi
  source "$setup"
done
if [[ "$DRY" == true ]]; then
  python3 "$ROOT/launcher_check.py" --plan "$MODE"
  ros2 launch "$ROOT/all_in_one.launch.py" --show-args
  exit 0
fi
mkdir -p "$ROOT/logs/ros" "$ROOT/ros_home" "$ROOT/yolo_config"
python3 "$ROOT/launcher_check.py" --preflight "$MODE"
python3 "$ROOT/launcher_check.py" --duplicates
exec flock --no-fork --nonblock "$ROOT/logs/all_in_one.lock" \
  ros2 launch "$ROOT/all_in_one.launch.py" mode:="$MODE"
