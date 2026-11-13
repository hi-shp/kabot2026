#!/usr/bin/env bash
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && cd .. && pwd)"
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
cd "$ROOT"
colcon --log-base "$ROOT/기록/logs/build" build --build-base "$ROOT/.runtime/build" --install-base "$ROOT/.runtime/install" --base-paths "$ROOT/공통모듈" --symlink-install --packages-select isv_shape_navigation --event-handlers console_direct+
