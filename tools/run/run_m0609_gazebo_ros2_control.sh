#!/usr/bin/env bash
set -euo pipefail

# Start an M0609 Gazebo Classic simulation that is actually driven by ros2_control.
# This is for visual motion verification only. It does not connect to hardware.

ROBOT_NAME="${ROBOT_NAME:-dsr01}"
MODEL="${MODEL:-m0609}"
COLOR="${COLOR:-white}"
GUI="${GUI:-false}"
X="${X:-0.0}"
Y="${Y:-0.0}"
Z="${Z:-0.1525}"
ROLL="${ROLL:-0.0}"
PITCH="${PITCH:-0.0}"
YAW="${YAW:-0.0}"
REMAP_TF="${REMAP_TF:-false}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
source /home/ssu/Azas/install/setup.bash 2>/dev/null || true
set -u

echo "[Azas] Starting Gazebo Classic ros2_control M0609 simulation"
echo "[Azas] name=${ROBOT_NAME} model=${MODEL} gui=${GUI} pose=(${X}, ${Y}, ${Z}, ${ROLL}, ${PITCH}, ${YAW})"
echo "[Azas] Motion command topic after startup:"
echo "[Azas]   /${ROBOT_NAME}/gz/dsr_position_controller/commands"
echo "[Azas] Controller checks:"
echo "[Azas]   ros2 control list_controllers -c /${ROBOT_NAME}/gz/controller_manager"

if ! ros2 pkg prefix azas_bringup >/dev/null 2>&1; then
  echo "[Azas] ERROR: azas_bringup is not built/sourced. Run: colcon build --symlink-install --packages-select azas_bringup" >&2
  exit 1
fi

exec ros2 launch azas_bringup m0609_gazebo_classic_ros2_control.launch.py \
  name:="${ROBOT_NAME}" \
  model:="${MODEL}" \
  color:="${COLOR}" \
  x:="${X}" \
  y:="${Y}" \
  z:="${Z}" \
  R:="${ROLL}" \
  P:="${PITCH}" \
  Y:="${YAW}"
