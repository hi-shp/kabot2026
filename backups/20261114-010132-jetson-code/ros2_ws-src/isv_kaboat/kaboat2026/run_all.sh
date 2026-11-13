#!/usr/bin/env bash
# Stable compatibility entry; implementation lives under 빌드실행.
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$ROOT/기록/logs/ros" "$ROOT/.runtime/ros_home" "$ROOT/.runtime/yolo_config"
exec bash "$ROOT/빌드실행/run_all.sh" "$@"
