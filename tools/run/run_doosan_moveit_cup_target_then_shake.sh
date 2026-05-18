#!/usr/bin/env bash
set -euo pipefail

# One-command RViz validation:
# official Doosan M0609 MoveIt/RViz + measured dispenser collision objects
# + optional detected-cup approach + current side-grip assumption
# + relative lift to safe shake space + pose shake.
# Defaults are virtual and stay inside /home/ssu/Azas overlays only.

TARGET_X="${TARGET_X:-0.43}"
TARGET_Y="${TARGET_Y:-0.08}"
TRANSPORT_Z="${TRANSPORT_Z:-0.50}"
SHAKE_CENTER_Z="${SHAKE_CENTER_Z:-0.62}"
SHAKE_AMPLITUDE_X="${SHAKE_AMPLITUDE_X:-0.03}"
SHAKE_AMPLITUDE_Y="${SHAKE_AMPLITUDE_Y:-0.02}"
SHAKE_CYCLES="${SHAKE_CYCLES:-4}"
START_DELAY_SEC="${START_DELAY_SEC:-14.0}"
MOVE_TO_INITIAL_SIDE_GRIP="${MOVE_TO_INITIAL_SIDE_GRIP:-false}"
MOVE_TO_DETECTED_CUP="${MOVE_TO_DETECTED_CUP:-false}"
SHAKE_MODE="${SHAKE_MODE:-relative_pose}"
LIFT_BEFORE_SAFE_SHAKE_SPACE="${LIFT_BEFORE_SAFE_SHAKE_SPACE:-true}"
MOVE_TO_SAFE_SHAKE_SPACE="${MOVE_TO_SAFE_SHAKE_SPACE:-true}"
RELATIVE_LIFT_Z="${RELATIVE_LIFT_Z:-0.25}"
SAFE_MIN_Z="${SAFE_MIN_Z:-0.55}"
SAFE_MAX_Z="${SAFE_MAX_Z:-0.85}"
USE_FIXED_SAFE_XY="${USE_FIXED_SAFE_XY:-false}"

set +u
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
set -u

exec ros2 launch azas_bringup doosan_moveit_cup_target_then_shake.launch.py \
  mode:=virtual \
  model:=m0609 \
  host:=127.0.0.1 \
  target_x:="${TARGET_X}" \
  target_y:="${TARGET_Y}" \
  transport_z:="${TRANSPORT_Z}" \
  shake_center_z:="${SHAKE_CENTER_Z}" \
  shake_amplitude_x:="${SHAKE_AMPLITUDE_X}" \
  shake_amplitude_y:="${SHAKE_AMPLITUDE_Y}" \
  shake_cycles:="${SHAKE_CYCLES}" \
  start_delay_sec:="${START_DELAY_SEC}" \
  move_to_initial_side_grip:="${MOVE_TO_INITIAL_SIDE_GRIP}" \
  move_to_detected_cup:="${MOVE_TO_DETECTED_CUP}" \
  shake_mode:="${SHAKE_MODE}" \
  lift_before_safe_shake_space:="${LIFT_BEFORE_SAFE_SHAKE_SPACE}" \
  move_to_safe_shake_space:="${MOVE_TO_SAFE_SHAKE_SPACE}" \
  relative_lift_z:="${RELATIVE_LIFT_Z}" \
  safe_min_z:="${SAFE_MIN_Z}" \
  safe_max_z:="${SAFE_MAX_Z}" \
  use_fixed_safe_xy:="${USE_FIXED_SAFE_XY}" \
  execute_motion:=true
