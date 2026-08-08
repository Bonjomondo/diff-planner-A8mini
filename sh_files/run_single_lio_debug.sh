#!/bin/zsh

# Run the normal LIO real-flight stack while preserving console output, ROS logs,
# configuration snapshots, and the key topics needed to diagnose mission/RC issues.

setopt PIPE_FAIL
unsetopt BG_NICE

SCRIPT_DIR="${0:A:h}"
WORKSPACE_DIR="${SCRIPT_DIR:h}"
RUN_SCRIPT="${SCRIPT_DIR}/run_single_lio.sh"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_ROOT="${UAV_DEBUG_LOG_ROOT:-${WORKSPACE_DIR}/flight_logs}"
LOG_DIR="${LOG_ROOT}/${TIMESTAMP}"

mkdir -p "${LOG_DIR}/ros"
export ROS_LOG_DIR="${LOG_DIR}/ros"

exec > >(tee -a "${LOG_DIR}/console.log") 2>&1

typeset -g RUN_PID=""
typeset -g BAG_PID=""
typeset -g SNAPSHOT_PID=""
typeset -gi CLEANED_UP=0

cleanup_debug_run() {
    if (( CLEANED_UP )); then
        return
    fi
    CLEANED_UP=1

    if [[ -n "${SNAPSHOT_PID}" ]] && kill -0 "${SNAPSHOT_PID}" 2>/dev/null; then
        kill -TERM "${SNAPSHOT_PID}" 2>/dev/null
        wait "${SNAPSHOT_PID}" 2>/dev/null
    fi
    if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
        echo "[debug] Finalizing rosbag..."
        kill -INT "${BAG_PID}" 2>/dev/null
        wait "${BAG_PID}" 2>/dev/null
    fi
    if [[ -n "${RUN_PID}" ]] && kill -0 "${RUN_PID}" 2>/dev/null; then
        kill -INT "${RUN_PID}" 2>/dev/null
    fi
    grep -E "RC channel 8|TAKEOFF|LAND|AUTO_HOVER|AUTO_LAND|CMD_CTRL|Loaded waypoint|Published waypoint|Waypoint [0-9]+|mission finished|gimbal" \
        "${LOG_DIR}/console.log" > "${LOG_DIR}/key_events.log" 2>/dev/null || true
    echo "end_time=$(date -Iseconds)" >> "${LOG_DIR}/metadata.txt"
    echo "[debug] Logs saved in: ${LOG_DIR}"
    echo "[debug] Key event summary: ${LOG_DIR}/key_events.log"
}

trap 'cleanup_debug_run; exit 130' INT TERM HUP
trap cleanup_debug_run EXIT

cd "${WORKSPACE_DIR}" || exit 1
if [[ ! -f "${WORKSPACE_DIR}/devel/setup.zsh" ]]; then
    echo "[debug] Missing devel/setup.zsh. Build and source the workspace first."
    exit 1
fi
source "${WORKSPACE_DIR}/devel/setup.zsh"

{
    echo "start_time=$(date -Iseconds)"
    echo "workspace=${WORKSPACE_DIR}"
    echo "hostname=$(hostname)"
    echo "kernel=$(uname -a)"
    echo "git_commit=$(git rev-parse HEAD 2>/dev/null || echo unknown)"
    echo "ros_distro=${ROS_DISTRO:-unknown}"
    echo "ros_master_uri=${ROS_MASTER_URI:-unknown}"
    echo
    echo "git_status:"
    git status --short 2>/dev/null
} > "${LOG_DIR}/metadata.txt"

cp "${RUN_SCRIPT}" "${LOG_DIR}/run_single_lio.sh.snapshot"
cp "${WORKSPACE_DIR}/src/user_command/multipoint/config/points.yaml" \
   "${LOG_DIR}/points.yaml.snapshot"
cp "${WORKSPACE_DIR}/src/user_command/multipoint/config/coverage_5x5.yaml" \
   "${LOG_DIR}/coverage_5x5.yaml.snapshot"
cp "${WORKSPACE_DIR}/src/user_command/multipoint/config/coverage_20x20.yaml" \
   "${LOG_DIR}/coverage_20x20.yaml.snapshot"
cp "${WORKSPACE_DIR}/src/user_command/multipoint/launch/multipointplan_exp_lio.launch" \
   "${LOG_DIR}/multipointplan_exp_lio.launch.snapshot"
git diff > "${LOG_DIR}/working_tree.diff" 2>/dev/null

echo "[debug] Starting normal flight stack"
echo "[debug] Console log: ${LOG_DIR}/console.log"
echo "[debug] ROS logs:    ${LOG_DIR}/ros/"
echo "[debug] Stop with Ctrl+C so the rosbag index is finalized cleanly"

"${RUN_SCRIPT}" &
RUN_PID=$!

integer master_wait=0
while ! rosnode list >/dev/null 2>&1; do
    if ! kill -0 "${RUN_PID}" 2>/dev/null; then
        echo "[debug] run_single_lio.sh exited before the ROS master became available"
        wait "${RUN_PID}"
        exit $?
    fi
    if (( master_wait >= 60 )); then
        echo "[debug] ROS master was not available after 60 seconds; continuing without rosbag"
        wait "${RUN_PID}"
        exit $?
    fi
    sleep 1
    (( master_wait += 1 ))
done

echo "[debug] ROS master detected; recording key flight topics"
rosbag record --tcpnodelay -O "${LOG_DIR}/flight_debug.bag" \
    /mavros/rc/in \
    /mavros/state \
    /mavros/extended_state \
    /mavros/battery \
    /mavros/local_position/odom \
    /ekf/ekf_odom \
    /setpoints_cmd \
    /goal \
    /move_base_simple/goal \
    /back_trigger \
    /px4ctrl/takeoff_land \
    /mission/gimbal_task \
    /mission/gimbal_done \
    /drone_0_diff_planner_node/planning/trajectory \
    /drone_0_traj_server/heartbeat \
    /diagnostics \
    > "${LOG_DIR}/rosbag.log" 2>&1 &
BAG_PID=$!

(
    sleep 35
    rosnode list > "${LOG_DIR}/rosnode_list.txt" 2>&1
    rostopic list -v > "${LOG_DIR}/rostopic_list.txt" 2>&1
    rosparam dump "${LOG_DIR}/rosparams.yaml" > "${LOG_DIR}/rosparam_dump.log" 2>&1
    rostopic info /px4ctrl/takeoff_land > "${LOG_DIR}/takeoff_land_topic.txt" 2>&1
    rostopic info /goal > "${LOG_DIR}/goal_topic.txt" 2>&1
) &
SNAPSHOT_PID=$!

wait "${RUN_PID}"
RUN_STATUS=$?
echo "[debug] run_single_lio.sh exited with status ${RUN_STATUS}"
exit "${RUN_STATUS}"
