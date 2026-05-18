#!/usr/bin/env bash
set -euo pipefail

# RViz validation sequence:
# 1. Move the virtual robot into the assumed side-grip posture and hold it.
# 2. From that current held posture, move the grasped tumbler to the selected dispenser front.

LOCK_FILE="${LOCK_FILE:-/tmp/azas_side_grip_to_dispenser.lock}"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "ERROR: another side-grip-to-dispenser validation is already running." >&2
  exit 1
fi

LOG_DIR="${LOG_DIR:-/tmp}"
SETUP_LOG="${SETUP_LOG:-${LOG_DIR}/azas_side_grip_hold.log}"
DISPENSER_LOG="${DISPENSER_LOG:-${LOG_DIR}/azas_side_grip_current_to_dispenser.log}"
RVIZ_LOG="${RVIZ_LOG:-${LOG_DIR}/azas_side_grip_current_to_dispenser_rviz.log}"
COLLISION_LOG="${COLLISION_LOG:-${LOG_DIR}/azas_side_grip_current_to_dispenser_collision.log}"

export SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID:-2}"
export ENABLE_DEMO_OBSTACLE="${ENABLE_DEMO_OBSTACLE:-false}"
export ENABLE_OBSTACLE_DETOUR="${ENABLE_OBSTACLE_DETOUR:-false}"
export FRONT_APPROACH_OFFSET_X="${FRONT_APPROACH_OFFSET_X:-0.12}"
export OUTLET_FRONT_OFFSET_X="${OUTLET_FRONT_OFFSET_X:-0.02}"
export TRANSFER_Z_OVERRIDE="${TRANSFER_Z_OVERRIDE:-0.20}"
export CONTROLLER_ACTION_WAIT_SEC="${CONTROLLER_ACTION_WAIT_SEC:-90.0}"
export TASK_MODE="${TASK_MODE:-dispenser_front}"
export DISPENSER_SEQUENCE_IDS="${DISPENSER_SEQUENCE_IDS:-[1,2,3,4]}"
export POSE_PLANNER_ID="${POSE_PLANNER_ID:-LIN}"
export MAX_SINGLE_SEGMENT_JOINT_MOTION_DEG="${MAX_SINGLE_SEGMENT_JOINT_MOTION_DEG:-170.0}"
export MAX_VELOCITY_SCALING_FACTOR="${MAX_VELOCITY_SCALING_FACTOR:-0.03}"
export MAX_ACCELERATION_SCALING_FACTOR="${MAX_ACCELERATION_SCALING_FACTOR:-0.03}"
export WAYPOINT_HOLD_SEC="${WAYPOINT_HOLD_SEC:-1.5}"
export DISPENSER_TRANSFER_WAIT_SEC="${DISPENSER_TRANSFER_WAIT_SEC:-120}"
export DISPENSER_SEQUENCE_WAIT_SEC="${DISPENSER_SEQUENCE_WAIT_SEC:-240}"

cleanup_stale_ros() {
  if [[ "${SKIP_STALE_CLEANUP:-false}" == "true" ]]; then
    return
  fi

  local patterns=(
    "doosan_moveit_grasped_tumbler_to_dispenser_node"
    "dsr_bringup2"
    "run_emulator"
    "rviz2"
    "move_group"
    "ros2_control_node"
    "robot_state_publisher"
  )
  local pattern
  local pid
  for pattern in "${patterns[@]}"; do
    while read -r pid; do
      [[ -z "${pid}" ]] && continue
      [[ "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]] && continue
      kill -TERM "${pid}" 2>/dev/null || true
    done < <(pgrep -f "${pattern}" || true)
  done

  sleep 2
  for pattern in "${patterns[@]}"; do
    while read -r pid; do
      [[ -z "${pid}" ]] && continue
      [[ "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]] && continue
      kill -KILL "${pid}" 2>/dev/null || true
    done < <(pgrep -f "${pattern}" || true)
  done

  while read -r container_id; do
    [[ -z "${container_id}" ]] && continue
    docker rm -f "${container_id}" 2>/dev/null || true
  done < <(
    docker ps -a --format '{{.ID}} {{.Names}} {{.Image}}' |
      awk '/emulator|dsr|doosanrobot|drcf/ {print $1}'
  )
}

source_azas_ros_env() {
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
}

cleanup() {
  if [[ "${KEEP_STACK_AFTER_DONE:-false}" == "true" && "${SCRIPT_DONE:-false}" == "true" ]]; then
    return
  fi
  if [[ -n "${RVIZ_PID:-}" ]] && kill -0 "${RVIZ_PID}" 2>/dev/null; then
    kill -TERM "${RVIZ_PID}" 2>/dev/null || true
    wait "${RVIZ_PID}" 2>/dev/null || true
  fi
  if [[ -n "${DISPENSER_PID:-}" ]] && kill -0 "${DISPENSER_PID}" 2>/dev/null; then
    kill -TERM "${DISPENSER_PID}" 2>/dev/null || true
    wait "${DISPENSER_PID}" 2>/dev/null || true
  fi
  if [[ -n "${COLLISION_PID:-}" ]] && kill -0 "${COLLISION_PID}" 2>/dev/null; then
    kill -TERM "${COLLISION_PID}" 2>/dev/null || true
    wait "${COLLISION_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}

stop_kept_stack() {
  KEEP_STACK_AFTER_DONE=false
  cleanup
  exit 0
}
trap cleanup EXIT
trap stop_kept_stack INT TERM

cleanup_stale_ros
rm -f "${SETUP_LOG}" "${DISPENSER_LOG}" "${RVIZ_LOG}" "${COLLISION_LOG}"
source_azas_ros_env

DISPLAY= \
TASK_MODE=side_grip_hold \
ASSUME_ALREADY_AT_SIDE_GRIP=false \
ENABLE_MEASURED_COLLISION_SCENE=false \
START_DELAY_SEC="${START_DELAY_SEC:-18}" \
/home/ssu/Azas/tools/run/run_doosan_moveit_grasped_tumbler_to_dispenser.sh \
  > >(tee "${SETUP_LOG}") 2>&1 &
LAUNCH_PID=$!

deadline=$((SECONDS + 90))
while ! grep -q "DONE: robot is holding the assumed side-grip posture" "${SETUP_LOG}" 2>/dev/null; do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "ERROR: side-grip setup launch exited before reaching hold posture." >&2
    exit 1
  fi
  if (( SECONDS > deadline )); then
    echo "ERROR: timed out waiting for side-grip hold posture." >&2
    exit 1
  fi
  sleep 1
done

source_azas_ros_env

ros2 run azas_motion measured_dispenser_collision_scene_node \
  --ros-args \
  -p config_path:="${COLLISION_CONFIG_PATH:-/home/ssu/Azas/install/azas_bringup/share/azas_bringup/config/measured_dispenser_collision.yaml}" \
  -p publish_period_sec:="${COLLISION_PUBLISH_PERIOD_SEC:-1.0}" \
  > >(tee "${COLLISION_LOG}") 2>&1 &
COLLISION_PID=$!

ros2 launch azas_bringup doosan_moveit_rviz_only.launch.py model:=m0609 \
  > >(tee "${RVIZ_LOG}") 2>&1 &
RVIZ_PID=$!
sleep "${RVIZ_START_WAIT_SEC:-3}"

ros2 run azas_motion doosan_moveit_grasped_tumbler_to_dispenser_node \
  --ros-args \
  -p task_mode:="${TASK_MODE}" \
  -p assume_already_at_side_grip:=true \
  -p execute_motion:=true \
  -p start_delay_sec:=0.0 \
  -p joint_1_deg:="${J1_DEG:-159.0}" \
  -p joint_2_deg:="${J2_DEG:--43.0}" \
  -p joint_3_deg:="${J3_DEG:--105.0}" \
  -p joint_4_deg:="${J4_DEG:--81.0}" \
  -p joint_5_deg:="${J5_DEG:-85.0}" \
  -p joint_6_deg:="${J6_DEG:-31.0}" \
  -p selected_dispenser_id:="${SELECTED_DISPENSER_ID}" \
  -p dispenser_sequence_ids:="${DISPENSER_SEQUENCE_IDS}" \
  -p enable_demo_obstacle:="${ENABLE_DEMO_OBSTACLE}" \
  -p enable_obstacle_detour:="${ENABLE_OBSTACLE_DETOUR}" \
  -p front_approach_offset_x:="${FRONT_APPROACH_OFFSET_X}" \
  -p outlet_front_offset_x:="${OUTLET_FRONT_OFFSET_X}" \
  -p transfer_z_override:="${TRANSFER_Z_OVERRIDE}" \
  -p pose_planner_id:="${POSE_PLANNER_ID}" \
  -p max_single_segment_joint_motion_deg:="${MAX_SINGLE_SEGMENT_JOINT_MOTION_DEG}" \
  -p max_velocity_scaling_factor:="${MAX_VELOCITY_SCALING_FACTOR}" \
  -p max_acceleration_scaling_factor:="${MAX_ACCELERATION_SCALING_FACTOR}" \
  -p waypoint_hold_sec:="${WAYPOINT_HOLD_SEC}" \
  -p controller_action_wait_sec:="${CONTROLLER_ACTION_WAIT_SEC}" \
  > >(tee "${DISPENSER_LOG}") 2>&1 &
DISPENSER_PID=$!

if [[ "${TASK_MODE}" == "dispenser_sequence" || "${TASK_MODE}" == "all_dispensers" ]]; then
  DONE_PATTERN="DONE: grasped tumbler visited dispenser sequence while staying side-grasped"
  TRANSFER_WAIT_SEC="${DISPENSER_SEQUENCE_WAIT_SEC}"
else
  DONE_PATTERN="DONE: grasped tumbler moved to dispenser front"
  TRANSFER_WAIT_SEC="${DISPENSER_TRANSFER_WAIT_SEC}"
fi

deadline=$((SECONDS + TRANSFER_WAIT_SEC))
while ! grep -q "${DONE_PATTERN}" "${DISPENSER_LOG}" 2>/dev/null; do
  if ! kill -0 "${DISPENSER_PID}" 2>/dev/null; then
    echo "ERROR: dispenser transfer node exited before completing dispenser front motion." >&2
    exit 1
  fi
  if (( SECONDS > deadline )); then
    echo "ERROR: timed out waiting for side-grip to dispenser transfer." >&2
    exit 1
  fi
  sleep 1
done

echo "DONE: RViz side-grip current posture to ${TASK_MODE} transfer completed."
SCRIPT_DONE=true
if [[ "${KEEP_STACK_AFTER_DONE:-false}" == "true" ]]; then
  echo "KEEP_STACK_AFTER_DONE=true: RViz and ROS stack remain running. Press Ctrl+C in this terminal to stop."
  wait
fi
