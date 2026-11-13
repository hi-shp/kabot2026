#!/usr/bin/env bash
# All sensors + mission, no local desktop windows; operator starts from Windows.
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export KABOAT_GUI=windows
exec bash "$ROOT/run_all.sh" "$@"
