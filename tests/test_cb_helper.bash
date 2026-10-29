#!/usr/bin/env bash
set -e
helper="${1:?helper file required}"
fixture="${2:?new fixture directory required}"
mkdir -p "$fixture/workspace/src" "$fixture/workspace/install" "$fixture/workspace/custom install"
printf 'export CB_UNDERLAY=loaded\n' > "$fixture/underlay.bash"
printf 'export CB_APPLIED=default\n' > "$fixture/workspace/install/setup.bash"
printf 'export CB_APPLIED=custom\n' > "$fixture/workspace/custom install/setup.bash"
export ISV_ROS_WS="$fixture/workspace"
export ISV_ROS_SETUP="$fixture/underlay.bash"
source "$helper"

function colcon() { CB_LAST_ARGS=("$@"); return "${CB_MOCK_EXIT:-0}"; }
cb
[[ "$CB_APPLIED" == default && "$CB_UNDERLAY" == loaded && "${CB_LAST_ARGS[*]}" == build ]]
[[ "$PWD" == "$ISV_ROS_WS" ]]
unset CB_APPLIED
cb --packages-select example
[[ "$CB_APPLIED" == default && "${CB_LAST_ARGS[*]}" == 'build --packages-select example' ]]
unset CB_APPLIED
cb --install-base 'custom install'
[[ "$CB_APPLIED" == custom && "${CB_LAST_ARGS[2]}" == 'custom install' ]]
unset CB_APPLIED
CB_MOCK_EXIT=7
if cb; then
    printf 'ERROR: build failure was hidden\n' >&2
    exit 1
else
    [[ "$?" == 7 && -z "${CB_APPLIED:-}" ]]
fi
printf 'CB_HELPER_TESTS_PASS: default build, argument forwarding, same-shell source, custom install, failure propagation\n'
