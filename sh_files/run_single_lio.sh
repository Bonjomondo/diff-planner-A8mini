#!/bin/zsh

echo '1' | sudo -S chmod 777 /dev/tty* & sleep 1
export DRONE_ID=0

wait_for_mavros_connection() {
  local timeout_s=30
  local start_time=$SECONDS

  echo "[startup] Waiting for MAVROS to connect to FCU..."
  while (( SECONDS - start_time < timeout_s )); do
    if timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q '^connected: True'; then
      echo "[startup] MAVROS is connected to FCU."
      return 0
    fi
    sleep 1
  done

  echo "[startup] ERROR: MAVROS did not connect to FCU within ${timeout_s}s." >&2
  return 1
}

wait_for_mavros_command_service() {
  local timeout_s=15
  local start_time=$SECONDS

  echo "[startup] Waiting for MAVROS command service..."
  while (( SECONDS - start_time < timeout_s )); do
    if rosservice info /mavros/cmd/command >/dev/null 2>&1; then
      echo "[startup] MAVROS command service is ready."
      return 0
    fi
    sleep 1
  done

  echo "[startup] ERROR: /mavros/cmd/command was not ready within ${timeout_s}s." >&2
  return 1
}

set_message_interval() {
  local message_id=$1
  local message_name=$2
  local interval_us=${3:-5000}
  local max_attempts=5
  local attempt

  for (( attempt=1; attempt<=max_attempts; attempt++ )); do
    echo "[startup] Setting ${message_name} (${message_id}) interval to ${interval_us} us, attempt ${attempt}/${max_attempts}..."
    if rosrun mavros mavcmd long 511 "${message_id}" "${interval_us}" 0 0 0 0 0; then
      echo "[startup] ${message_name} stream configured successfully."
      return 0
    fi

    echo "[startup] WARN: Failed to configure ${message_name}; retrying..." >&2
    sleep 1
  done

  echo "[startup] ERROR: Failed to configure ${message_name} after ${max_attempts} attempts." >&2
  return 1
}

roslaunch mavros px4.launch &
MAVROS_LAUNCH_PID=$!

if ! wait_for_mavros_connection || ! wait_for_mavros_command_service; then
  echo "[startup] Aborting because the FCU/MAVROS link is not ready." >&2
  kill "${MAVROS_LAUNCH_PID}" 2>/dev/null
  exit 1
fi

set_message_interval 31  ATTITUDE_QUATERNION 5000 || exit 1
set_message_interval 105 HIGHRES_IMU         5000 || exit 1
set_message_interval 83  ATTITUDE_TARGET     5000 || exit 1
set_message_interval 147 BATTERY_STATUS      5000 || exit 1
set_message_interval 106 OPTICAL_FLOW_RAD    5000 || exit 1

source devel/setup.zsh
roslaunch faster_lio mapping_mid360.launch & sleep 10
roslaunch ekf ekf_lidar.launch & sleep 5
roslaunch diff_planner run_exp_single_lio.launch & sleep 3
roslaunch px4ctrl run_ctrl_lio.launch & sleep 3
roslaunch multipoint multipointplan_exp_lio.launch & sleep 2
roslaunch diff_planner exp_rviz.launch & sleep 1
wait
