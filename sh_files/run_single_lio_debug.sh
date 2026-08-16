#!/bin/zsh

# Run the normal LIO real-flight stack while preserving the information needed
# to reproduce and diagnose FCU/MAVROS, controller, planner, RC, and gimbal issues.
# Raw LiDAR and raw MAVLink traffic are optional because they can grow bags quickly.

setopt PIPE_FAIL
unsetopt BG_NICE

SCRIPT_DIR="${0:A:h}"
WORKSPACE_DIR="${SCRIPT_DIR:h}"
RUN_SCRIPT="${SCRIPT_DIR}/run_single_lio.sh"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_ROOT="${UAV_DEBUG_LOG_ROOT:-${WORKSPACE_DIR}/flight_logs}"
LOG_DIR="${LOG_ROOT}/${TIMESTAMP}"
MISSION_CSV_SOURCE="/tmp/a8mini_mission_timestamps.csv"

mkdir -p "${LOG_DIR}/ros" "${LOG_DIR}/topic_rates"
export ROS_LOG_DIR="${LOG_DIR}/ros"

exec > >(tee -a "${LOG_DIR}/console.log") 2>&1

typeset -g RUN_PID=""
typeset -g BAG_PID=""
typeset -g SNAPSHOT_PID=""
typeset -g RUN_STATUS="unknown"
typeset -gi CLEANED_UP=0

capture_topic_rate() {
    local topic="$1"
    local name="$2"
    local output="${LOG_DIR}/topic_rates/${name}.txt"

    if rostopic list 2>/dev/null | grep -Fxq "${topic}"; then
        timeout 8 rostopic hz -w 200 "${topic}" > "${output}" 2>&1 || true
    else
        echo "topic_not_available=${topic}" > "${output}"
    fi
}

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

    if [[ -f "${LOG_DIR}/flight_debug.bag" ]]; then
        rosbag info "${LOG_DIR}/flight_debug.bag" > "${LOG_DIR}/rosbag_info.txt" 2>&1 || true
    fi

    if [[ -n "${RUN_PID}" ]] && kill -0 "${RUN_PID}" 2>/dev/null; then
        kill -INT "${RUN_PID}" 2>/dev/null
    fi

    # The mission node writes this file only after a mission trigger. Preserve it
    # only when it was created/updated during this debug run, avoiding stale /tmp data.
    if [[ -f "${MISSION_CSV_SOURCE}" && "${MISSION_CSV_SOURCE}" -nt "${LOG_DIR}/metadata.txt" ]]; then
        cp "${MISSION_CSV_SOURCE}" "${LOG_DIR}/mission_timestamps.csv" 2>/dev/null || true
    fi

    grep -E "\[startup\]|RC channel (6|8)|TAKEOFF|LAND|AUTO_HOVER|AUTO_LAND|CMD_CTRL|planning stop|traj_server|Lost heartbeat|wait ack timeout|FCU|MAVROS|Loaded waypoint|Published waypoint|Waypoint [0-9]+|mission finished|gimbal|FATAL|ERROR" \
        "${LOG_DIR}/console.log" > "${LOG_DIR}/key_events.log" 2>/dev/null || true

    {
        echo "end_time=$(date -Iseconds)"
        echo "run_status=${RUN_STATUS}"
    } >> "${LOG_DIR}/metadata.txt"

    echo "[debug] Logs saved in: ${LOG_DIR}"
    echo "[debug] Key event summary: ${LOG_DIR}/key_events.log"
}

trap 'RUN_STATUS=130; cleanup_debug_run; exit 130' INT TERM HUP
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
    echo "git_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
    echo "ros_distro=${ROS_DISTRO:-unknown}"
    echo "ros_master_uri=${ROS_MASTER_URI:-unknown}"
    echo "record_lidar=${UAV_DEBUG_RECORD_LIDAR:-0}"
    echo "record_raw_mavlink=${UAV_DEBUG_RECORD_MAVLINK:-0}"
    echo
    echo "disk_space:"
    df -h "${LOG_ROOT}" 2>/dev/null || true
    echo
    echo "git_status:"
    git status --short 2>/dev/null
} > "${LOG_DIR}/metadata.txt"

{
    echo "captured_at=$(date -Iseconds)"
    echo "expected_fcu_device=/dev/ttyACM0"
    ls -l /dev/ttyACM0 2>&1 || true
    if [[ -e /dev/ttyACM0 ]] && command -v udevadm >/dev/null 2>&1; then
        echo
        echo "udev_properties:"
        udevadm info --query=property --name=/dev/ttyACM0 2>/dev/null || true
    fi
} > "${LOG_DIR}/fcu_device.txt"

cp "${0:A}" "${LOG_DIR}/run_single_lio_debug.sh.snapshot"
cp "${RUN_SCRIPT}" "${LOG_DIR}/run_single_lio.sh.snapshot"
cp "${WORKSPACE_DIR}/src/user_command/multipoint/config/points.yaml" \
   "${LOG_DIR}/points.yaml.snapshot" 2>/dev/null || true
cp "${WORKSPACE_DIR}/src/user_command/multipoint/launch/multipointplan_exp_lio.launch" \
   "${LOG_DIR}/multipointplan_exp_lio.launch.snapshot" 2>/dev/null || true
# HEAD includes both staged and unstaged tracked changes, unlike plain `git diff`.
git diff HEAD > "${LOG_DIR}/working_tree.diff" 2>/dev/null || true

echo "[debug] Starting normal flight stack"
echo "[debug] Console log: ${LOG_DIR}/console.log"
echo "[debug] ROS logs:    ${LOG_DIR}/ros/"
echo "[debug] Rosbag:      ${LOG_DIR}/flight_debug.bag"
echo "[debug] Stop with Ctrl+C so the rosbag index is finalized cleanly"

"${RUN_SCRIPT}" &
RUN_PID=$!

integer master_wait=0
while ! rosnode list >/dev/null 2>&1; do
    if ! kill -0 "${RUN_PID}" 2>/dev/null; then
        echo "[debug] run_single_lio.sh exited before the ROS master became available"
        wait "${RUN_PID}"
        RUN_STATUS=$?
        exit "${RUN_STATUS}"
    fi
    if (( master_wait >= 60 )); then
        echo "[debug] ROS master was not available after 60 seconds; aborting debug run"
        wait "${RUN_PID}"
        RUN_STATUS=$?
        exit "${RUN_STATUS}"
    fi
    sleep 1
    (( master_wait += 1 ))
done

echo "[debug] ROS master detected; recording key flight topics"

typeset -a BAG_TOPICS
BAG_TOPICS=(
    /tf
    /tf_static
    /mavros/rc/in
    /mavros/state
    /mavros/extended_state
    /mavros/battery
    /mavros/statustext/recv
    /mavros/timesync_status
    /mavros/imu/data
    /mavros/imu/data_raw
    /mavros/px4flow/raw/optical_flow_rad
    /mavros/local_position/odom
    /mavros/setpoint_raw/attitude
    /mavros/setpoint_raw/target_attitude
    /laserMapping/odometry
    /ekf/ekf_odom
    /setpoints_cmd
    /debugPx4ctrl
    /traj_start_trigger
    /goal
    /move_base_simple/goal
    /back_trigger
    /planning/stop
    /planning/yaw
    /px4ctrl/takeoff_land
    /mission/gimbal_task
    /mission/gimbal_done
    /drone_0_planning/trajectory
    /drone_0_traj_server/heartbeat
    /diagnostics
)

if [[ "${UAV_DEBUG_RECORD_LIDAR:-0}" == "1" ]]; then
    BAG_TOPICS+=(/livox/lidar)
    echo "[debug] Raw LiDAR recording enabled (/livox/lidar); expect a much larger bag"
fi

if [[ "${UAV_DEBUG_RECORD_MAVLINK:-0}" == "1" ]]; then
    BAG_TOPICS+=(/mavlink/from /mavlink/to)
    echo "[debug] Raw MAVLink recording enabled (/mavlink/from, /mavlink/to)"
fi

rosbag record --tcpnodelay -O "${LOG_DIR}/flight_debug.bag" \
    "${BAG_TOPICS[@]}" \
    > "${LOG_DIR}/rosbag.log" 2>&1 &
BAG_PID=$!

(
    # The normal stack has several staged startup sleeps. Wait until it should be
    # fully up, then take one reproducibility snapshot and short rate samples.
    sleep 35
    rosnode list > "${LOG_DIR}/rosnode_list.txt" 2>&1
    rostopic list -v > "${LOG_DIR}/rostopic_list.txt" 2>&1
    rosparam dump "${LOG_DIR}/rosparams.yaml" > "${LOG_DIR}/rosparam_dump.log" 2>&1

    capture_topic_rate /mavros/imu/data mavros_imu_data &
    capture_topic_rate /mavros/imu/data_raw mavros_imu_data_raw &
    capture_topic_rate /mavros/setpoint_raw/target_attitude mavros_target_attitude &
    capture_topic_rate /mavros/battery mavros_battery &
    capture_topic_rate /mavros/px4flow/raw/optical_flow_rad mavros_optical_flow_rad &
    capture_topic_rate /ekf/ekf_odom ekf_odom &
    capture_topic_rate /setpoints_cmd setpoints_cmd &
    capture_topic_rate /mavros/setpoint_raw/attitude mavros_attitude_command &
    wait
) &
SNAPSHOT_PID=$!

wait "${RUN_PID}"
RUN_STATUS=$?
echo "[debug] run_single_lio.sh exited with status ${RUN_STATUS}"
exit "${RUN_STATUS}"
