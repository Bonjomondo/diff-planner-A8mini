#!/usr/bin/env bash
# All simulated FCU messages stay on a separate master. Never use live 11311.
set -eo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_IP=127.0.0.1
STATE="${AGRI_GCS_TEST_STATE:-$HOME/.local/state/agri-gcs/roundtrip-tests}"
mkdir -p "$STATE"
export ROS_HOME="$STATE/ros"
export ROS_LOG_DIR="$ROS_HOME/log"
python3 - <<'PY'
import socket
with socket.socket() as peer:
    if peer.connect_ex(('127.0.0.1',11411)) == 0:
        raise SystemExit('Test port 11411 is occupied; will not reuse or stop an unknown master')
PY
nohup setsid roscore -p 11411 </dev/null >"$STATE/master.log" 2>&1 &
test_master=$!
trap 'kill -INT "$test_master" 2>/dev/null || true; wait "$test_master" 2>/dev/null || true' EXIT
for attempt in $(seq 1 20); do timeout 2 rostopic list >/dev/null 2>&1 && break; sleep .3; done
cd "$WORKSPACE"
python3 -B -m unittest discover -s src/user_command/multipoint/test -p 'test_gcs*.py'
./devel/lib/multipoint/test_remote_mission
./devel/lib/multipoint/test_mission_guard
g++ -std=c++11 -Wall -Wextra -Werror -I src/diff_planner/plan_env/include \
    src/diff_planner/plan_env/test/test_mission_box.cpp -o "$STATE/test_mission_box"
"$STATE/test_mission_box"
g++ -std=c++11 -Wall -Wextra -Werror -I src/realflight_modules/px4ctrl/src \
    src/realflight_modules/px4ctrl/test/test_thrust_estimation_guard.cpp \
    -o "$STATE/test_thrust_estimation_guard"
"$STATE/test_thrust_estimation_guard"
tests=("$@")
if (( ${#tests[@]} == 0 )); then tests=(remote_mission.test remote_cancel.test point_roundtrip_planner.test automatic_cancel_takeoff.test automatic_takeoff_odom_loss.test automatic_takeoff_rc_loss.test automatic_takeoff_rejected.test automatic_cancel_landing.test automatic_landing_timeout.test automatic_landing_blocked.test automatic_roundtrip.test automatic_controller.test automatic_controller_delayed.test automatic_roundtrip_delayed.test automatic_takeoff_corridor_blocked.test automatic_position_limit_removed.test spool_attitude_baseline.test spool_attitude_hold.test); fi
for launch in "${tests[@]}"; do
    echo "Running isolated test: $launch"
    # roslaunch leaves node parameters behind on a reused master. A preceding
    # automatic test must not turn a generic regression into an automatic task.
    for test_namespace in /multipointplan /px4ctrl /remote_integration /automatic_roundtrip /automatic_controller /automatic_controller_delayed; do
        rosparam delete "$test_namespace" >/dev/null 2>&1 || true
    done
    if ! timeout 90 rostest --reuse-master multipoint "$launch" >"$STATE/$launch.log" 2>&1; then
        tail -n 90 "$STATE/$launch.log"
        exit 1
    fi
    tail -n 12 "$STATE/$launch.log"
done
