#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ACTION="${1:-help}"
if [ "$#" -gt 0 ]; then shift; fi
case "$ACTION" in
 build) bash "$ROOT/빌드실행/sensors.sh" build; bash "$ROOT/빌드실행/build_course.sh" ;;
 build-driving) exec bash "$ROOT/빌드실행/build_course.sh" ;;
 sensors) exec bash "$ROOT/빌드실행/sensors.sh" "$@" ;;
 bringup) exec bash "$ROOT/빌드실행/run_bringup.sh" "$@" ;;
 mission) exec bash "$ROOT/빌드실행/run_mission.sh" "$@" ;;
 preview) exec bash "$ROOT/빌드실행/run_all.sh" --preview "$@" ;;
 drive) exec bash "$ROOT/빌드실행/run_all.sh" --autonomy "$@" ;;
 plan) exec bash "$ROOT/빌드실행/run_all.sh" --preview --dry-run ;;
 help|-h|--help)
   printf 'Usage: bash 시작.sh {build|build-driving|sensors [start|status|stop|setup]|bringup|mission|preview|drive|plan}\n'
   printf 'bringup: equipment + YOLO + fusion only\nmission: mission + GUI only; reuse equipment; GUI start required\nmission --web: interactive web GUI on port 8770\n'
   printf 'build: all sensor packages + driving package\npreview: complete stack without thrust output\ndrive: existing autonomous stack (can start driving when ready)\nplan: inspect launch paths without starting nodes\n' ;;
 *) printf 'Unknown command: %s\n' "$ACTION" >&2; exit 2 ;;
esac
