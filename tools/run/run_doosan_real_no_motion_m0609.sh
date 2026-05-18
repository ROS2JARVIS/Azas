#!/usr/bin/env bash
set -euo pipefail

# Start Doosan M0609 ROS 2 / MoveIt bringup against a real controller without
# sending Azas motion or RG2 commands. Use this after the robot is connected,
# before robot_connection_acceptance.sh or run_robot_real.sh.

ROBOT_NAME="${ROBOT_NAME:-}"
ROBOT_HOST="${ROBOT_HOST:-}"
ROBOT_PORT="${ROBOT_PORT:-12345}"
MODEL="${MODEL:-m0609}"
COLOR="${COLOR:-white}"
RT_HOST="${RT_HOST:-192.168.137.50}"
DOOSAN_NO_MOTION_CONFIRM="${DOOSAN_NO_MOTION_CONFIRM:-}"
SHOW_ARGS_ONLY="${SHOW_ARGS_ONLY:-false}"

if [[ "${SHOW_ARGS_ONLY}" == "true" ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  source /home/ssu/ros2_ws/install/setup.bash
  set -u
  exec ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py --show-args
fi

if [[ -z "${ROBOT_HOST}" ]]; then
  echo "[Azas] Refusing Doosan real no-motion bringup: ROBOT_HOST is required."
  echo "[Azas] Example:"
  echo "  ROBOT_HOST=192.168.1.100 DOOSAN_NO_MOTION_CONFIRM=CONNECT_DOOSAN_NO_MOTION $0"
  exit 1
fi

if [[ "${ROBOT_HOST}" == "127.0.0.1" || "${ROBOT_HOST}" == "localhost" ]]; then
  echo "[Azas] Refusing Doosan real no-motion bringup: ROBOT_HOST points to localhost."
  echo "[Azas] Use /home/ssu/Azas/tools/run/run_doosan_virtual_m0609.sh for virtual mode."
  exit 1
fi

if [[ "${DOOSAN_NO_MOTION_CONFIRM}" != "CONNECT_DOOSAN_NO_MOTION" ]]; then
  echo "[Azas] Refusing Doosan real no-motion bringup without explicit confirmation."
  echo "[Azas] This should not move the arm, but it connects to the real robot controller."
  echo "[Azas] Re-run with:"
  echo "  DOOSAN_NO_MOTION_CONFIRM=CONNECT_DOOSAN_NO_MOTION"
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
set -u

echo "[Azas] Starting Doosan ${MODEL} real no-motion bringup"
echo "[Azas] mode=real name=${ROBOT_NAME:-<none>} host=${ROBOT_HOST} port=${ROBOT_PORT}"
echo "[Azas] This entrypoint does not launch Azas hardware motion or RG2 commands."
echo "[Azas] After startup, run /home/ssu/Azas/tools/checks/robot_connection_acceptance.sh in another terminal."

launch_args=(
  host:="${ROBOT_HOST}" \
  port:="${ROBOT_PORT}" \
  mode:=real \
  model:="${MODEL}" \
  color:="${COLOR}" \
  rt_host:="${RT_HOST}"
)

if [[ -n "${ROBOT_NAME}" ]]; then
  launch_args=(name:="${ROBOT_NAME}" "${launch_args[@]}")
fi

exec ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py "${launch_args[@]}"
