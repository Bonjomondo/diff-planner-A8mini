#!/bin/zsh

# Start the complete real-flight LIO stack and request one automatic takeoff.
#
# The script is intentionally a zsh script because the existing flight stack
# scripts use zsh. It can be launched from any directory, including by using
# its absolute path after boot:
#
#   /home/q/Documents/diff-planner-A8miniV2/sh_files/one_click_takeoff.sh
#
# Use --start-only when the stack should be brought up without sending a
# takeoff command. The stack stays in the foreground so Ctrl+C still performs
# the normal debug-log and rosbag cleanup.

setopt PIPE_FAIL
unsetopt BG_NICE

SCRIPT_DIR="${0:A:h}"
WORKSPACE_DIR="${SCRIPT_DIR:h}"
ROS_SETUP_FILE="${ROS_SETUP_FILE:-/opt/ros/noetic/setup.zsh}"
STACK_SCRIPT="${SCRIPT_DIR}/run_single_lio_debug.sh"

typeset -g STACK_PID=""
typeset -g STACK_EXIT_STATUS=0
typeset -gi CLEANED_UP=0
typeset -g RUN_MODE="takeoff"

integer READY_TIMEOUT="${ONE_CLICK_READY_TIMEOUT:-180}"
integer READY_STABLE_SEC="${ONE_CLICK_READY_STABLE_SEC:-3}"
integer TAKEOFF_TIMEOUT="${ONE_CLICK_TAKEOFF_TIMEOUT:-30}"

usage() {
    cat <<'EOF'
用法:
  ./sh_files/one_click_takeoff.sh              启动完整栈并自动起飞
  ./sh_files/one_click_takeoff.sh --start-only 只启动，不发送起飞指令

可选环境变量:
  ONE_CLICK_READY_TIMEOUT=180       等待系统就绪的最长秒数
  ONE_CLICK_READY_STABLE_SEC=3      安全条件连续满足的秒数
  ONE_CLICK_TAKEOFF_TIMEOUT=30      等待确认进入空中的最长秒数
  ROS_SETUP_FILE=/opt/ros/noetic/setup.zsh

起飞前必须满足：飞控已连接且未解锁、已收到 RC/IMU/电池/里程计，
RC 摇杆居中，模式和指令开关打开，RC 8 通道处于 DOWN。
EOF
}

while (( $# > 0 )); do
    case "$1" in
        --start-only|--no-takeoff)
            RUN_MODE="start-only"
            ;;
        --takeoff)
            RUN_MODE="takeoff"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[one-click] 未知参数: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

if (( READY_TIMEOUT <= 0 || READY_STABLE_SEC < 0 || TAKEOFF_TIMEOUT <= 0 )); then
    echo "[one-click] 超时参数必须是正数，READY_STABLE_SEC 可为 0。" >&2
    exit 2
fi

cleanup() {
    if (( CLEANED_UP )); then
        return
    fi
    CLEANED_UP=1
    trap '' INT TERM HUP

    if stack_process_running; then
        echo "[one-click] 正在停止飞行栈，等待日志和 rosbag 收尾..."
        kill -INT "${STACK_PID}" 2>/dev/null || true
        wait "${STACK_PID}" 2>/dev/null || true
    fi
}

trap 'exit 130' INT TERM HUP
trap cleanup EXIT

source_setup() {
    if [[ -r "${ROS_SETUP_FILE}" ]]; then
        echo "[one-click] 加载 ROS 环境: ${ROS_SETUP_FILE}"
        source "${ROS_SETUP_FILE}" || return 1
    elif [[ -z "${ROS_DISTRO:-}" ]]; then
        echo "[one-click] 找不到 ROS 环境: ${ROS_SETUP_FILE}" >&2
        echo "[one-click] 如 ROS 安装路径不同，请设置 ROS_SETUP_FILE。" >&2
        return 1
    fi

    if [[ ! -r "${WORKSPACE_DIR}/devel/setup.zsh" ]]; then
        echo "[one-click] 找不到 ${WORKSPACE_DIR}/devel/setup.zsh。" >&2
        echo "[one-click] 请先在该工作空间执行 catkin_make。" >&2
        return 1
    fi

    cd "${WORKSPACE_DIR}" || return 1
    echo "[one-click] 加载工作空间: ${WORKSPACE_DIR}"
    source "${WORKSPACE_DIR}/devel/setup.zsh" || return 1
    return 0
}

require_commands() {
    local command_name
    for command_name in roslaunch rosnode rostopic rosservice timeout ps; do
        if ! command -v "${command_name}" >/dev/null 2>&1; then
            echo "[one-click] 缺少命令: ${command_name}" >&2
            return 1
        fi
    done
}

stack_process_running() {
    local process_state
    [[ -n "${STACK_PID}" ]] || return 1
    kill -0 "${STACK_PID}" 2>/dev/null || return 1
    process_state="$(ps -o stat= -p "${STACK_PID}" 2>/dev/null)"
    [[ -n "${process_state}" && "${process_state}" != Z* ]]
}

topic_once() {
    local topic="$1"
    timeout 4 rostopic echo -n 1 "${topic}" 2>/dev/null
}

topic_has_message() {
    local topic="$1"
    local message
    message="$(topic_once "${topic}")" || return 1
    [[ -n "${message}" ]]
}

required_nodes_ready() {
    local nodes
    nodes="$(timeout 5 rosnode list 2>/dev/null)" || return 1

    local node_name
    for node_name in /px4ctrl /laserMapping /ekf /multipointplan; do
        print -r -- "${nodes}" | grep -Fxq "${node_name}" || return 1
    done
}

takeoff_subscriber_ready() {
    local topic_info
    topic_info="$(timeout 5 rostopic info /px4ctrl/takeoff_land 2>/dev/null)" || return 1
    print -r -- "${topic_info}" | grep -Eq '/px4ctrl([[:space:]]|$)' || return 1
}

rc_preflight_ready() {
    local rc_message
    rc_message="$(topic_once /mavros/rc/in)" || return 1

    # px4ctrl uses a 25% dead zone for channels 1..4, and requires both its
    # hover-mode and command-mode switches to be high for automatic takeoff.
    # multipointplan also requires RC channel 8 to start in DOWN; keeping it
    # there prevents this one-click command from starting a waypoint mission.
    print -r -- "${rc_message}" | awk '
        /channels:/ {
            line = $0
            sub(/^.*channels:[[:space:]]*\[/, "", line)
            sub(/\].*$/, "", line)
            count = split(line, value, /,[[:space:]]*/)
            if (count >= 8) {
                found = 1
                valid = 1
                for (i = 1; i <= 4; ++i) {
                    if ((value[i] + 0) < 1375 || (value[i] + 0) > 1625)
                        valid = 0
                }
                if ((value[5] + 0) <= 1750 || (value[6] + 0) <= 1750)
                    valid = 0
                if ((value[8] + 0) <= 1750)
                    valid = 0
            }
        }
        END { exit !(found && valid) }
    '
}

flight_state_ground_ready() {
    local state extended_state
    state="$(topic_once /mavros/state)" || return 1
    [[ "${state}" == *"connected: True"* ]] || return 1
    [[ "${state}" == *"armed: False"* ]] || return 1

    extended_state="$(topic_once /mavros/extended_state)" || return 1
    # MAVROS ExtendedState: LANDED_STATE_ON_GROUND == 1.
    [[ "${extended_state}" == *"landed_state: 1"* ]]
}

stack_ready() {
    local state
    required_nodes_ready || return 1
    flight_state_ground_ready || return 1
    takeoff_subscriber_ready || return 1
    rc_preflight_ready || return 1

    for state in /mavros/imu/data /mavros/battery /ekf/ekf_odom; do
        topic_has_message "${state}" || return 1
    done

    rosservice info /mavros/cmd/arming >/dev/null 2>&1 || return 1
    return 0
}

wait_for_stack_ready() {
    integer started=$SECONDS
    integer ready_since=-1
    integer last_report=-10

    echo "[one-click] 启动完整 LIO 飞行栈，等待系统就绪..."
    while (( SECONDS - started < READY_TIMEOUT )); do
        if ! stack_process_running; then
            wait "${STACK_PID}" 2>/dev/null
            STACK_EXIT_STATUS=$?
            echo "[one-click] 飞行栈提前退出，状态码: ${STACK_EXIT_STATUS}" >&2
            return 1
        fi

        if stack_ready; then
            if (( ready_since < 0 )); then
                ready_since=$SECONDS
                echo "[one-click] 关键节点、传感器、MAVROS 和 RC 已就绪，继续确认..."
            elif (( SECONDS - ready_since >= READY_STABLE_SEC )); then
                echo "[one-click] 安全条件已连续满足 ${READY_STABLE_SEC}s。"
                return 0
            fi
        else
            ready_since=-1
            if (( SECONDS - last_report >= 10 )); then
                echo "[one-click] 等待就绪：请确认飞控连接、定位、RC 开关/摇杆和 RC8 DOWN。"
                last_report=$SECONDS
            fi
        fi
        sleep 1
    done

    echo "[one-click] ${READY_TIMEOUT}s 内未达到启动安全条件，未发送起飞指令。" >&2
    return 1
}

send_takeoff() {
    echo "[one-click] 发送一次自动起飞指令。"
    timeout 10 rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand \
        "takeoff_land_cmd: 1"
}

wait_for_takeoff_confirmation() {
    integer started=$SECONDS
    integer last_report=-10

    echo "[one-click] 等待 PX4 确认已解锁并进入空中状态..."
    while (( SECONDS - started < TAKEOFF_TIMEOUT )); do
        if ! stack_process_running; then
            wait "${STACK_PID}" 2>/dev/null
            STACK_EXIT_STATUS=$?
            echo "[one-click] 起飞过程中飞行栈提前退出，状态码: ${STACK_EXIT_STATUS}" >&2
            return 1
        fi

        local state extended_state
        state="$(topic_once /mavros/state)" || state=""
        extended_state="$(topic_once /mavros/extended_state)" || extended_state=""
        if [[ "${state}" == *"armed: True"* ]] &&
           ([[ "${extended_state}" == *"landed_state: 2"* ]] ||
            [[ "${extended_state}" == *"landed_state: 3"* ]] ||
            [[ "${extended_state}" == *"landed_state: 4"* ]]); then
            echo "[one-click] 已确认：飞控 armed=True，ExtendedState 已离地/起飞。"
            return 0
        fi

        if (( SECONDS - last_report >= 5 )); then
            echo "[one-click] 起飞确认中（控制器会先进行约 3s 电机加速和爬升）..."
            last_report=$SECONDS
        fi
        sleep 1
    done

    echo "[one-click] 未在 ${TAKEOFF_TIMEOUT}s 内确认进入空中状态。" >&2
    echo "[one-click] 不会自动发送降落指令；请立即通过遥控器确认并接管。" >&2
    return 1
}

if ! source_setup || ! require_commands; then
    exit 1
fi

if [[ ! -x "${STACK_SCRIPT}" ]]; then
    echo "[one-click] 启动脚本不可执行或不存在: ${STACK_SCRIPT}" >&2
    exit 1
fi

if rosnode list 2>/dev/null | grep -Eq '^/(px4ctrl|laserMapping|ekf|multipointplan)$'; then
    echo "[one-click] 检测到已有飞行节点，拒绝重复启动。" >&2
    echo "[one-click] 请先确认上一轮任务已结束，再重试。" >&2
    exit 1
fi

"${STACK_SCRIPT}" &
STACK_PID=$!

if ! wait_for_stack_ready; then
    exit 1
fi

if [[ "${RUN_MODE}" == "start-only" ]]; then
    echo "[one-click] 仅启动模式：未发送起飞指令。按 Ctrl+C 结束并保存日志。"
    wait "${STACK_PID}"
    STACK_EXIT_STATUS=$?
    exit "${STACK_EXIT_STATUS}"
fi

if ! send_takeoff; then
    echo "[one-click] 起飞消息发布失败；未自动重试。" >&2
    exit 1
fi

if ! wait_for_takeoff_confirmation; then
    # Keep the ROS stack alive for immediate manual inspection/RC takeover.
    # The user can press Ctrl+C after handling the aircraft.
    wait "${STACK_PID}"
    STACK_EXIT_STATUS=$?
    exit "${STACK_EXIT_STATUS}"
fi

echo "[one-click] 起飞流程完成，系统继续运行。后续可用 RC8 UP 启动航点任务。"
wait "${STACK_PID}"
STACK_EXIT_STATUS=$?
exit "${STACK_EXIT_STATUS}"
