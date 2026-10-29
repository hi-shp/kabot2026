# Source this file from interactive Bash; cb must run in the current shell.
# No files are removed, no running nodes are stopped, and no motors are started.
unalias cb 2>/dev/null || true

function cb() {
    local _isv_cb_workspace="${ISV_ROS_WS:-$HOME/ros2_ws}"
    local _isv_cb_underlay="${ISV_ROS_SETUP:-/opt/ros/humble/setup.bash}"
    local _isv_cb_install="install"
    local _isv_cb_next_install=0
    local _isv_cb_arg
    local _isv_cb_status

    for _isv_cb_arg in "$@"; do
        if (( _isv_cb_next_install )); then
            _isv_cb_install="$_isv_cb_arg"
            _isv_cb_next_install=0
        elif [[ "$_isv_cb_arg" == --install-base ]]; then
            _isv_cb_next_install=1
        elif [[ "$_isv_cb_arg" == --install-base=* ]]; then
            _isv_cb_install="${_isv_cb_arg#--install-base=}"
        fi
    done

    if [[ ! -d "$_isv_cb_workspace/src" ]]; then
        printf '[cb] Workspace src not found: %s\n' "$_isv_cb_workspace" >&2
        return 2
    fi
    if [[ ! -f "$_isv_cb_underlay" ]]; then
        printf '[cb] ROS underlay not found: %s\n' "$_isv_cb_underlay" >&2
        return 2
    fi

    source "$_isv_cb_underlay" || return
    cd "$_isv_cb_workspace" || return
    printf '[cb] Workspace: %s\n' "$PWD"
    printf '[cb] colcon build (all packages unless selection arguments are supplied)\n'
    if colcon build "$@"; then
        if [[ ! -f "$_isv_cb_install/setup.bash" ]]; then
            printf '[cb] Build returned success but setup.bash is missing: %s\n' "$_isv_cb_install" >&2
            return 3
        fi
        source "$_isv_cb_install/setup.bash" || return
        printf '[cb] Build succeeded; sourced %s/setup.bash in this shell.\n' "$_isv_cb_install"
    else
        _isv_cb_status=$?
        printf '[cb] Build failed (%s); workspace setup was NOT sourced.\n' "$_isv_cb_status" >&2
        return "$_isv_cb_status"
    fi
}
