#!/usr/bin/env bash
set -e
if [[ -n "${SSH_CONNECTION:-}" ]]; then
  read -r view_client_ip view_client_port view_server_ip view_server_port <<< "$SSH_CONNECTION"
  if [[ "$view_client_ip" == "$view_server_ip" ]]; then
    printf 'GUI SSH tunnel must start in Windows PowerShell (PS C:\\...), not in the Jetson shell (isv@ubuntu).\n' >&2
    printf 'Open a NEW Windows PowerShell window and run the SSH -L command there.\n' >&2
    exit 2
  fi
fi
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && cd .. && pwd)"
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_HOME="$ROOT/.runtime/ros_home"
export ROS_LOG_DIR="$ROOT/기록/logs/ros"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
if [[ -f "$ROOT/.runtime/install/setup.bash" ]]; then
  source "$ROOT/.runtime/install/setup.bash"
else
  source "$ROOT/install/setup.bash"
fi
if [[ " $* " != *" --read-only "* ]]; then
  python3 "$ROOT/빌드실행/gui_runtime.py"
fi
if [[ " $* " != *" --read-only "* ]]; then
  set -- --mapping-controls "$@"
fi
exec python3 "$(dirname -- "${BASH_SOURCE[0]}")/windows_viewer.py" --reuse-existing "$@"
