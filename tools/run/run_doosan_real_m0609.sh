#!/usr/bin/env bash
set -euo pipefail

# Start Doosan M0609 ROS 2 / MoveIt bringup against the real controller for
# supervised real-motion panel runs.  This intentionally avoids the legacy
# "no_motion" entrypoint name so panel logs cannot be mistaken for a virtual or
# motion-blocked run.

ROBOT_NAME="${ROBOT_NAME:-}"
ROBOT_HOST="${ROBOT_HOST:-}"
ROBOT_PORT="${ROBOT_PORT:-12345}"
MODEL="${MODEL:-m0609}"
COLOR="${COLOR:-white}"
RT_HOST="${RT_HOST:-0.0.0.0}"
DOOSAN_REAL_MOTION_CONFIRM="${DOOSAN_REAL_MOTION_CONFIRM:-}"
SHOW_ARGS_ONLY="${SHOW_ARGS_ONLY:-false}"

if [[ "${SHOW_ARGS_ONLY}" == "true" ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  source /home/ssu/ros2_ws/install/setup.bash
  source /home/ssu/Azas/install/setup.bash
  set -u
  exec ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py --show-args
fi

if [[ -z "${ROBOT_HOST}" ]]; then
  echo "[Azas] Refusing Doosan real bringup: ROBOT_HOST is required."
  echo "[Azas] Example:"
  echo "  ROBOT_HOST=192.168.1.100 DOOSAN_REAL_MOTION_CONFIRM=ENABLE_DOOSAN_REAL_MOTION_BRINGUP $0"
  exit 1
fi

if [[ "${ROBOT_HOST}" == "127.0.0.1" || "${ROBOT_HOST}" == "localhost" ]]; then
  echo "[Azas] Refusing Doosan real bringup: ROBOT_HOST points to localhost."
  echo "[Azas] Use /home/ssu/Azas/tools/run/run_doosan_virtual_m0609.sh for virtual mode."
  exit 1
fi

if [[ "${DOOSAN_REAL_MOTION_CONFIRM}" != "ENABLE_DOOSAN_REAL_MOTION_BRINGUP" ]]; then
  echo "[Azas] Refusing Doosan real bringup without explicit real-motion confirmation."
  echo "[Azas] Re-run with:"
  echo "  DOOSAN_REAL_MOTION_CONFIRM=ENABLE_DOOSAN_REAL_MOTION_BRINGUP"
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
source /home/ssu/Azas/install/setup.bash
set -u

echo "[Azas] Starting Doosan ${MODEL} REAL MOTION bringup"
echo "[Azas] mode=real name=${ROBOT_NAME:-<none>} host=${ROBOT_HOST} port=${ROBOT_PORT}"
echo "[Azas] This entrypoint is for supervised real robot motion. Keep E-stop reachable."

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
