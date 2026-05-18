#!/usr/bin/env bash
set -euo pipefail

# RViz-only preview: move a simulated cup to a target XYZ, then show the
# existing high-shake task around the target area. This never arms hardware.

TARGET_X="${TARGET_X:-0.43}"
TARGET_Y="${TARGET_Y:-0.08}"
TARGET_Z="${TARGET_Z:-0.135}"
SHAKE_DELAY_SEC="${SHAKE_DELAY_SEC:-8.0}"
SHAKE_CENTER_X="${SHAKE_CENTER_X:-${TARGET_X}}"
SHAKE_CENTER_Y="${SHAKE_CENTER_Y:-${TARGET_Y}}"
SHAKE_CENTER_Z="${SHAKE_CENTER_Z:-0.62}"

set +u
source /opt/ros/humble/setup.bash
if [[ -f /home/ssu/ros2_ws/install/setup.bash ]]; then
  source /home/ssu/ros2_ws/install/setup.bash
fi
if [[ -f /home/ssu/ws_moveit/install/setup.bash ]]; then
  source /home/ssu/ws_moveit/install/setup.bash
fi
source /home/ssu/Azas/install/setup.bash
set -u

exec ros2 launch azas_bringup cup_target_then_shake_rviz.launch.py \
  target_x:="${TARGET_X}" \
  target_y:="${TARGET_Y}" \
  target_z:="${TARGET_Z}" \
  shake_center_x:="${SHAKE_CENTER_X}" \
  shake_center_y:="${SHAKE_CENTER_Y}" \
  shake_center_z:="${SHAKE_CENTER_Z}" \
  shake_delay_sec:="${SHAKE_DELAY_SEC}" \
  enable_hardware:=false
