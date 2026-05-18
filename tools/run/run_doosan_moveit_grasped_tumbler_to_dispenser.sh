#!/usr/bin/env bash
set -euo pipefail

# Official Doosan M0609 MoveIt/RViz + MoveItPy execution.
# Starts from the provided side-grasp joint posture and moves to dispenser front.

SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID:-2}"
J1_DEG="${J1_DEG:-159.0}"
J2_DEG="${J2_DEG:--43.0}"
J3_DEG="${J3_DEG:--105.0}"
J4_DEG="${J4_DEG:--81.0}"
J5_DEG="${J5_DEG:-85.0}"
J6_DEG="${J6_DEG:-31.0}"
FRONT_APPROACH_OFFSET_X="${FRONT_APPROACH_OFFSET_X:-0.12}"
OUTLET_FRONT_OFFSET_X="${OUTLET_FRONT_OFFSET_X:-0.02}"
TRANSFER_Z_OVERRIDE="${TRANSFER_Z_OVERRIDE:-0.20}"
ENABLE_SAFE_LIFT_TRANSFER="${ENABLE_SAFE_LIFT_TRANSFER:-true}"
SAFE_LIFT_MIN_Z="${SAFE_LIFT_MIN_Z:-0.40}"
SAFE_LIFT_DELTA_Z="${SAFE_LIFT_DELTA_Z:-0.15}"
SAFE_LIFT_MAX_Z="${SAFE_LIFT_MAX_Z:-0.55}"
DISPENSER_ABOVE_Z="${DISPENSER_ABOVE_Z:-0.40}"
ENABLE_DEMO_OBSTACLE="${ENABLE_DEMO_OBSTACLE:-true}"
ENABLE_OBSTACLE_DETOUR="${ENABLE_OBSTACLE_DETOUR:-true}"
DETOUR_Y="${DETOUR_Y:--0.24}"
MOVEIT_READY_WAIT_SEC="${MOVEIT_READY_WAIT_SEC:-5.0}"
CONTROLLER_ACTION_WAIT_SEC="${CONTROLLER_ACTION_WAIT_SEC:-90.0}"
EXECUTION_BACKEND="${EXECUTION_BACKEND:-controller_action}"
TASK_MODE="${TASK_MODE:-dispenser_front}"
ASSUME_ALREADY_AT_SIDE_GRIP="${ASSUME_ALREADY_AT_SIDE_GRIP:-false}"
FLOOR_TARGET_X="${FLOOR_TARGET_X:-0.42}"
FLOOR_TARGET_Y="${FLOOR_TARGET_Y:--0.22}"
FLOOR_TARGET_Z="${FLOOR_TARGET_Z:-0.20}"
FLOOR_APPROACH_Z="${FLOOR_APPROACH_Z:-0.28}"
ALLOW_DISPENSER_ORIENTATION_FALLBACK="${ALLOW_DISPENSER_ORIENTATION_FALLBACK:-true}"
STATE_PLANNER_ID="${STATE_PLANNER_ID:-PTP}"
POSE_PLANNER_ID="${POSE_PLANNER_ID:-LIN}"
START_DELAY_SEC="${START_DELAY_SEC:-14.0}"
EXECUTE_MOTION="${EXECUTE_MOTION:-true}"
ENABLE_MEASURED_COLLISION_SCENE="${ENABLE_MEASURED_COLLISION_SCENE:-true}"
COLLISION_CONFIG_PATH="${COLLISION_CONFIG_PATH:-/home/ssu/Azas/install/azas_bringup/share/azas_bringup/config/measured_dispenser_collision.yaml}"
COLLISION_PUBLISH_PERIOD_SEC="${COLLISION_PUBLISH_PERIOD_SEC:-1.0}"

set +u
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset GAZEBO_MODEL_PATH
unset GZ_SIM_RESOURCE_PATH
unset GZ_SIM_SYSTEM_PLUGIN_PATH
unset IGN_GAZEBO_RESOURCE_PATH
unset IGN_GAZEBO_SYSTEM_PLUGIN_PATH
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash
set -u

required_pkgs=(
  azas_bringup
  azas_motion
  dsr_bringup2
  dsr_common2
  dsr_controller2
  dsr_description2
  dsr_msgs2
  dsr_moveit_config_m0609
  moveit_py
  moveit_ros_move_group
)

for pkg in "${required_pkgs[@]}"; do
  if ! prefix="$(ros2 pkg prefix "${pkg}" 2>/dev/null)"; then
    echo "ERROR: required package '${pkg}' is not available from the strict ~/Azas environment." >&2
    echo "Build/install the Doosan and MoveIt dependencies into /home/ssu/Azas before running this script." >&2
    exit 2
  fi

  case "${prefix}" in
    /home/ssu/Azas/*|/opt/ros/humble|/opt/ros/humble/*)
      ;;
    *)
      echo "ERROR: package '${pkg}' resolves outside ~/Azas: ${prefix}" >&2
      echo "This script intentionally refuses external ROS workspace overlays." >&2
      exit 2
      ;;
  esac
done

exec ros2 launch azas_bringup doosan_moveit_grasped_tumbler_to_dispenser.launch.py \
  mode:=virtual \
  model:=m0609 \
  host:=127.0.0.1 \
  task_mode:="${TASK_MODE}" \
  assume_already_at_side_grip:="${ASSUME_ALREADY_AT_SIDE_GRIP}" \
  selected_dispenser_id:="${SELECTED_DISPENSER_ID}" \
  joint_1_deg:="${J1_DEG}" \
  joint_2_deg:="${J2_DEG}" \
  joint_3_deg:="${J3_DEG}" \
  joint_4_deg:="${J4_DEG}" \
  joint_5_deg:="${J5_DEG}" \
  joint_6_deg:="${J6_DEG}" \
  front_approach_offset_x:="${FRONT_APPROACH_OFFSET_X}" \
  outlet_front_offset_x:="${OUTLET_FRONT_OFFSET_X}" \
  transfer_z_override:="${TRANSFER_Z_OVERRIDE}" \
  enable_safe_lift_transfer:="${ENABLE_SAFE_LIFT_TRANSFER}" \
  safe_lift_min_z:="${SAFE_LIFT_MIN_Z}" \
  safe_lift_delta_z:="${SAFE_LIFT_DELTA_Z}" \
  safe_lift_max_z:="${SAFE_LIFT_MAX_Z}" \
  dispenser_above_z:="${DISPENSER_ABOVE_Z}" \
  enable_demo_obstacle:="${ENABLE_DEMO_OBSTACLE}" \
  enable_obstacle_detour:="${ENABLE_OBSTACLE_DETOUR}" \
  detour_y:="${DETOUR_Y}" \
  floor_target_x:="${FLOOR_TARGET_X}" \
  floor_target_y:="${FLOOR_TARGET_Y}" \
  floor_target_z:="${FLOOR_TARGET_Z}" \
  floor_approach_z:="${FLOOR_APPROACH_Z}" \
  execution_backend:="${EXECUTION_BACKEND}" \
  moveit_ready_wait_sec:="${MOVEIT_READY_WAIT_SEC}" \
  controller_action_wait_sec:="${CONTROLLER_ACTION_WAIT_SEC}" \
  allow_dispenser_orientation_fallback:="${ALLOW_DISPENSER_ORIENTATION_FALLBACK}" \
  state_planner_id:="${STATE_PLANNER_ID}" \
  pose_planner_id:="${POSE_PLANNER_ID}" \
  start_delay_sec:="${START_DELAY_SEC}" \
  enable_measured_collision_scene:="${ENABLE_MEASURED_COLLISION_SCENE}" \
  collision_config_path:="${COLLISION_CONFIG_PATH}" \
  collision_publish_period_sec:="${COLLISION_PUBLISH_PERIOD_SEC}" \
  execute_motion:="${EXECUTE_MOTION}"
