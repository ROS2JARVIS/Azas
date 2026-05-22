#!/usr/bin/env bash
set -euo pipefail

# RViz-only simulation of the chained rule-based workflow:
# side-grasp transfer to the dispenser outlet while holding the cup, then high-shake.

SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID:-2}"
USE_RVIZ="${USE_RVIZ:-true}"
USE_ROBOT_URDF="${USE_ROBOT_URDF:-true}"
ANIMATE_ROBOT_JOINTS="${ANIMATE_ROBOT_JOINTS:-true}"
ENABLE_IK_PREVIEW="${ENABLE_IK_PREVIEW:-true}"
SHOW_SEQUENCE_MARKERS="${SHOW_SEQUENCE_MARKERS:-false}"
SHOW_DISPENSER_MARKERS="${SHOW_DISPENSER_MARKERS:-false}"
SHOW_ANIMATED_CUP="${SHOW_ANIMATED_CUP:-false}"
SHOW_DEMO_ARM="${SHOW_DEMO_ARM:-false}"
USE_SHAKE_VISUALIZER="${USE_SHAKE_VISUALIZER:-false}"
SHAKE_DELAY_SEC="${SHAKE_DELAY_SEC:-10.0}"
GRASP_X="${GRASP_X:-0.42}"
GRASP_Y="${GRASP_Y:--0.24}"
GRASP_Z="${GRASP_Z:-0.05}"
MOUTH_X="${MOUTH_X:-${GRASP_X}}"
MOUTH_Y="${MOUTH_Y:-${GRASP_Y}}"
MOUTH_Z="${MOUTH_Z:-0.22}"
SHAKE_CENTER_X="${SHAKE_CENTER_X:-0.28}"
SHAKE_CENTER_Y="${SHAKE_CENTER_Y:--0.30}"
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
DISPENSER_KEEPOUT_RADIUS="${DISPENSER_KEEPOUT_RADIUS:-0.20}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
source /home/ssu/Azas/install/setup.bash
set -u

exec ros2 launch azas_bringup tumbler_dispenser_then_shake_demo.launch.py \
  selected_dispenser_id:="${SELECTED_DISPENSER_ID}" \
  use_rviz:="${USE_RVIZ}" \
  use_robot_urdf:="${USE_ROBOT_URDF}" \
  animate_robot_joints:="${ANIMATE_ROBOT_JOINTS}" \
  enable_ik_preview:="${ENABLE_IK_PREVIEW}" \
  show_sequence_markers:="${SHOW_SEQUENCE_MARKERS}" \
  show_dispenser_markers:="${SHOW_DISPENSER_MARKERS}" \
  show_animated_cup:="${SHOW_ANIMATED_CUP}" \
  show_demo_arm:="${SHOW_DEMO_ARM}" \
  use_shake_visualizer:="${USE_SHAKE_VISUALIZER}" \
  shake_delay_sec:="${SHAKE_DELAY_SEC}" \
  grasp_x:="${GRASP_X}" \
  grasp_y:="${GRASP_Y}" \
  grasp_z:="${GRASP_Z}" \
  mouth_x:="${MOUTH_X}" \
  mouth_y:="${MOUTH_Y}" \
  mouth_z:="${MOUTH_Z}" \
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
  dispenser_keepout_radius:="${DISPENSER_KEEPOUT_RADIUS}"
