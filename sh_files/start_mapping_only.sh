#!/usr/bin/env bash
# Sensors and odometry only. Never starts a planner, controller or mission.
set -eo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/agri-gcs/mapping-start"
mkdir -p "$STATE_DIR"
exec 9>"$STATE_DIR/start.lock"
flock -n 9 || { echo 'ERROR: 另一个建图启动请求正在执行'; exit 1; }
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
source /opt/ros/noetic/setup.bash
if [[ ! -f "$WORKSPACE/devel/setup.bash" ]]; then
    echo 'ERROR: 新工作目录尚未编译'; exit 1
fi
source "$WORKSPACE/devel/setup.bash"
set -u
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export BD_LIST=47MCP3J0031590
export ROS_HOME="$STATE_DIR/ros"
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"
cd "$WORKSPACE"
for package in faster_lio livox_ros_driver2; do
    package_path=$(rospack find "$package")
    case "$package_path" in "$WORKSPACE"/src/*) ;; *) echo "ERROR: $package 未来自新工作目录"; exit 1;; esac
done
for binary in devel/lib/faster_lio/run_mapping_online devel/lib/livox_ros_driver2/livox_ros_driver2_node; do
    [[ -x "$binary" ]] || { echo "ERROR: 缺少建图二进制 $binary"; exit 1; }
done

started=()
cleanup_failure() {
    code=$?
    if (( code != 0 )); then
        # Only clean up process groups created by this invocation.
        for child in "${started[@]}"; do kill -- "-$child" 2>/dev/null || true; done
    fi
    exit "$code"
}
trap cleanup_failure EXIT
trap 'exit 1' TERM INT
nodes() { timeout 4 rosnode list 2>/dev/null; }
start_child() {
    local name="$1"; shift
    nohup setsid "$@" </dev/null >"$STATE_DIR/$name.log" 2>&1 9>&- &
    started+=("$!")
    printf '%s\n' "$!" >"$STATE_DIR/$name.pid"
}
wait_topic() { timeout "$1" rostopic echo -n 1 "$2" >/dev/null 2>&1; }

echo 'STAGE: 正在检查 ROS 建图环境'
if ! nodes >/dev/null; then
    start_child roscore roscore -p 11311
    for attempt in $(seq 1 20); do nodes >/dev/null && break; sleep 0.5; done
    nodes >/dev/null || { echo 'ERROR: ROS master 启动失败'; exit 1; }
fi
if nodes | grep -Fxq /laserMapping; then
    if wait_topic 8 /laserMapping/cloud_registered_body/header && wait_topic 5 /laserMapping/odometry/header; then
        echo 'STAGE: 建图已在运行，直接连接'; exit 0
    fi
    echo 'ERROR: 已有建图节点缺少新点云或位姿，请检查传感器；未自动重启'; exit 1
fi
if nodes | grep -Fxq /livox_lidar_publisher2; then
    echo 'ERROR: 雷达驱动已被其他进程使用，未重复启动'; exit 1
fi
echo 'STAGE: 正在连接飞控并等待 IMU'
if ! nodes | grep -Fxq /mavros; then
    start_child mavros roslaunch mavros px4.launch gcs_url:=''
fi
connected=false
for attempt in $(seq 1 12); do
    if timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q '^connected: True'; then connected=true; break; fi
    sleep 0.5
done
[[ "$connected" == true ]] || { echo 'ERROR: 飞控未连接，请检查供电和 USB'; exit 1; }
# /imu/data follows ATTITUDE_QUATERNION; HIGHRES_IMU alone leaves it at 50 Hz.
# Match the existing full-flight startup requests without starting flight nodes.
timeout 8 rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 >/dev/null 2>&1 || true
timeout 8 rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 >/dev/null 2>&1 || true
wait_topic 8 /mavros/imu/data/header || { echo 'ERROR: 未收到飞控 IMU'; exit 1; }
echo 'STAGE: 正在启动 MID-360 和 Faster-LIO'
start_child mapping roslaunch faster_lio mapping_mid360.launch rviz:=false
wait_topic 25 /laserMapping/cloud_registered_body/header || { echo 'ERROR: 雷达点云超时，请检查雷达供电和网线'; exit 1; }
wait_topic 5 /laserMapping/odometry/header || { echo 'ERROR: 未收到建图位姿'; exit 1; }
echo 'STAGE: 建图已启动，正在传输点云'
