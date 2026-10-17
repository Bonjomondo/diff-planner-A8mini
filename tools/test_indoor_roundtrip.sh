#!/usr/bin/env bash
# All simulated FCU messages stay on a separate master. Never use live 11311.
set -eo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_IP=127.0.0.1
STATE="$HOME/.local/state/agri-gcs/roundtrip-tests"
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
tests=("$@")
if (( ${#tests[@]} == 0 )); then tests=(remote_mission.test remote_cancel.test point_roundtrip_planner.test); fi
for launch in "${tests[@]}"; do
    echo "Running isolated test: $launch"
    if ! timeout 90 rostest --reuse-master multipoint "$launch" >"$STATE/$launch.log" 2>&1; then
        tail -n 90 "$STATE/$launch.log"
        exit 1
    fi
    tail -n 12 "$STATE/$launch.log"
done
