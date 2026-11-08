#!/usr/bin/env bash
# Start only the mission adapter; flight execution still requires explicit commands.
set -eo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.bash
if [[ "${AGRI_PRECISION_LANDING:-0}" == "1" ]]; then
    CV_BRIDGE_SETUP="/home/ly/Lib/cv_bridge_pkgs/devel/setup.bash"
    [[ -f "$CV_BRIDGE_SETUP" ]] || { echo 'OpenCV 4.5 cv_bridge is missing.' >&2; exit 1; }
    source "$CV_BRIDGE_SETUP"
fi
[[ -f "$WORKSPACE/devel/setup.bash" ]] || { echo 'Project has not been built.' >&2; exit 1; }
source "$WORKSPACE/devel/setup.bash"
set -u
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export AGRI_MAPPING_PROFILE="${AGRI_MAPPING_PROFILE:-$WORKSPACE/config/mapping_profiles/$(id -un).yaml}"
[[ -f "$AGRI_MAPPING_PROFILE" ]] || { echo 'Aircraft mapping profile is missing.' >&2; exit 1; }
if timeout 4 rosnode list 2>/dev/null | grep -E '^/(multipointplan|gcs_gateway)$' >/dev/null; then
    echo 'A mission adapter is already running; no duplicate instance started.' >&2
    exit 1
fi
cd "$WORKSPACE"
if [[ "${AGRI_PRECISION_LANDING:-0}" == "1" ]]; then
    exec roslaunch multipoint indoor_visual_roundtrip.launch
fi
exec roslaunch multipoint indoor_roundtrip.launch
