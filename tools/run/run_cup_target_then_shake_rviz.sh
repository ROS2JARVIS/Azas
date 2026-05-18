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
SHAKE_AMPLITUDE_X="${SHAKE_AMPLITUDE_X:-0.100}"
SHAKE_AMPLITUDE_Y="${SHAKE_AMPLITUDE_Y:-0.040}"
SHAKE_AMPLITUDE_Z="${SHAKE_AMPLITUDE_Z:-0.055}"
SHAKE_CYCLES="${SHAKE_CYCLES:-4}"
SHAKE_TWIST_RX_DEG="${SHAKE_TWIST_RX_DEG:-6.0}"
SHAKE_TWIST_RZ_DEG="${SHAKE_TWIST_RZ_DEG:-22.0}"
APPROACH_LINE_TIME="${APPROACH_LINE_TIME:-3.5}"
SHAKE_LINE_TIME="${SHAKE_LINE_TIME:-0.40}"
MIN_SHAKE_Z="${MIN_SHAKE_Z:-0.55}"
DISPENSER_KEEPOUT_RADIUS="${DISPENSER_KEEPOUT_RADIUS:-0.0}"

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
  shake_amplitude_x:="${SHAKE_AMPLITUDE_X}" \
  shake_amplitude_y:="${SHAKE_AMPLITUDE_Y}" \
  shake_amplitude_z:="${SHAKE_AMPLITUDE_Z}" \
  shake_cycles:="${SHAKE_CYCLES}" \
  shake_twist_rx_deg:="${SHAKE_TWIST_RX_DEG}" \
  shake_twist_rz_deg:="${SHAKE_TWIST_RZ_DEG}" \
  approach_line_time:="${APPROACH_LINE_TIME}" \
  shake_line_time:="${SHAKE_LINE_TIME}" \
  min_shake_z:="${MIN_SHAKE_Z}" \
  dispenser_keepout_radius:="${DISPENSER_KEEPOUT_RADIUS}" \
  shake_delay_sec:="${SHAKE_DELAY_SEC}" \
  enable_hardware:=false
