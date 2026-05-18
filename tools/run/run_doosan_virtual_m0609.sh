#!/usr/bin/env bash
set -euo pipefail

# Start the Doosan M0609 virtual MoveIt stack.
# This is the pre-real-robot motion stack. It does not connect to the real arm
# unless MODE/HOST/PORT are changed away from the defaults.

ROBOT_NAME="${ROBOT_NAME:-}"
ROBOT_HOST="${ROBOT_HOST:-127.0.0.1}"
ROBOT_PORT="${ROBOT_PORT:-12345}"
MODE="${MODE:-virtual}"
MODEL="${MODEL:-m0609}"
COLOR="${COLOR:-white}"
RT_HOST="${RT_HOST:-192.168.137.50}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
set -u

echo "[Azas] Starting Doosan ${MODEL} MoveIt stack"
echo "[Azas] mode=${MODE} name=${ROBOT_NAME:-<none>} host=${ROBOT_HOST} port=${ROBOT_PORT}"
echo "[Azas] Keep MODE=virtual for non-hardware validation."

launch_args=(
  host:="${ROBOT_HOST}" \
  port:="${ROBOT_PORT}" \
  mode:="${MODE}" \
  model:="${MODEL}" \
  color:="${COLOR}" \
  rt_host:="${RT_HOST}"
)

if [[ -n "${ROBOT_NAME}" ]]; then
  launch_args=(name:="${ROBOT_NAME}" "${launch_args[@]}")
fi

exec ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py "${launch_args[@]}"
