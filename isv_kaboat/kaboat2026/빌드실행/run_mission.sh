#!/usr/bin/env bash
set -e
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$TASK_DIR/run_all.sh" --mission-only "$@"
