#!/usr/bin/env bash
# Added launcher only; existing workspaces and launch files are not changed.
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && cd .. && pwd)"
MODE=autonomy
DRY=false
COMPONENTS=all
GUI=auto
for arg in "$@"; do
  case "$arg" in
    --preview) MODE=preview ;;
    --autonomy) MODE=autonomy ;;
    --dry-run) DRY=true ;;
    --bringup-only) COMPONENTS=bringup ;;
    --mission-only) COMPONENTS=mission ;;
    --web) GUI=web ;;
    --desktop) GUI=desktop ;;
    --no-gui) GUI=none ;;
    -h|--help)
      printf 'Usage: bash run_all.sh [--preview|--autonomy] [--bringup-only|--mission-only] [--web|--desktop|--no-gui] [--dry-run]\n'
      printf 'Default: all sensors + YOLO + fusion + GUI, one-shot automatic start when RC AUTO and navigation data are ready.\n'
      printf 'Preview: same stack, no actuator output or automatic start.\n'
      printf 'Bringup-only: equipment + perception, no mission. Mission-only: existing equipment + GUI, operator start required.\n'
      exit 0 ;;
    *) printf 'Unknown option: %s\n' "$arg" >&2; exit 2 ;;
  esac
done
if [[ "$COMPONENTS" == bringup ]]; then MODE=preview; fi
export COURSE_WORKSPACE="$ROOT"
if [[ -f "$ROOT/설정/all_in_one.env" ]]; then source "$ROOT/설정/all_in_one.env"; fi
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export DISPLAY="${DISPLAY:-:0}"
if [[ "$DISPLAY" == :0 || "$DISPLAY" == :0.0 ]]; then
  export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
fi
export ROS_HOME="$ROOT/.runtime/ros_home"
export ROS_LOG_DIR="$ROOT/기록/logs/ros"
export YOLO_CONFIG_DIR="$ROOT/.runtime/yolo_config"
for setup in /opt/ros/humble/setup.bash "$HOME/ros2_ws/install/setup.bash" "$ROOT/.runtime/install/setup.bash"; do
  if [[ ! -f "$setup" ]]; then printf 'Missing setup: %s\n' "$setup" >&2; exit 2; fi
  source "$setup"
done
if [[ "$DRY" == true ]]; then
  python3 "$ROOT/빌드실행/launcher_check.py" --plan "$MODE" --components "$COMPONENTS" --gui "$GUI"
  ros2 launch "$ROOT/빌드실행/all_in_one.launch.py" --show-args
  exit 0
fi
mkdir -p "$ROOT/기록/logs/ros" "$ROOT/.runtime/ros_home" "$ROOT/.runtime/yolo_config"
python3 "$ROOT/빌드실행/launcher_check.py" --preflight "$MODE" --components "$COMPONENTS"
python3 "$ROOT/빌드실행/launcher_check.py" --duplicates --components "$COMPONENTS"
# Split components may coexist; the full stack must own this lock exclusively.
exec 9>"$ROOT/기록/logs/stack_lifecycle.lock"
if [[ "$COMPONENTS" == all ]]; then
  flock --nonblock --exclusive 9
else
  flock --nonblock --shared 9
fi
exec flock --no-fork --nonblock "$ROOT/기록/logs/stack_${COMPONENTS}.lock" \
  ros2 launch "$ROOT/빌드실행/all_in_one.launch.py" mode:="$MODE" components:="$COMPONENTS" gui:="$GUI"
