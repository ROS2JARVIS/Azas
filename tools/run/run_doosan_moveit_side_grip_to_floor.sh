#!/usr/bin/env bash
set -euo pipefail

# RViz validation sequence:
# 1. Move the virtual robot into the assumed side-grip posture and hold it.
# 2. From that current held posture, run only the floor transfer motion.

LOCK_FILE="${LOCK_FILE:-/tmp/azas_side_grip_to_floor.lock}"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "ERROR: another side-grip-to-floor validation is already running." >&2
  echo "Stop the existing run before starting a new one." >&2
  exit 1
fi

LOG_DIR="${LOG_DIR:-/tmp}"
SETUP_LOG="${SETUP_LOG:-${LOG_DIR}/azas_side_grip_hold.log}"
FLOOR_LOG="${FLOOR_LOG:-${LOG_DIR}/azas_side_grip_current_to_floor.log}"
RVIZ_LOG="${RVIZ_LOG:-${LOG_DIR}/azas_side_grip_current_to_floor_rviz.log}"

export ENABLE_DEMO_OBSTACLE="${ENABLE_DEMO_OBSTACLE:-false}"
export ENABLE_OBSTACLE_DETOUR="${ENABLE_OBSTACLE_DETOUR:-false}"
export FLOOR_TARGET_X="${FLOOR_TARGET_X:-0.42}"
export FLOOR_TARGET_Y="${FLOOR_TARGET_Y:--0.22}"
export FLOOR_TARGET_Z="${FLOOR_TARGET_Z:-0.20}"
export FLOOR_APPROACH_Z="${FLOOR_APPROACH_Z:-0.28}"
export CONTROLLER_ACTION_WAIT_SEC="${CONTROLLER_ACTION_WAIT_SEC:-90.0}"

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

  docker rm -f dsr_emulator 2>/dev/null || true
}

cleanup() {
  if [[ "${KEEP_STACK_AFTER_DONE:-false}" == "true" && "${SCRIPT_DONE:-false}" == "true" ]]; then
    return
  fi
  if [[ -n "${RVIZ_PID:-}" ]] && kill -0 "${RVIZ_PID}" 2>/dev/null; then
    kill -TERM "${RVIZ_PID}" 2>/dev/null || true
    wait "${RVIZ_PID}" 2>/dev/null || true
  fi
  if [[ -n "${FLOOR_PID:-}" ]] && kill -0 "${FLOOR_PID}" 2>/dev/null; then
    kill -TERM "${FLOOR_PID}" 2>/dev/null || true
    wait "${FLOOR_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

cleanup_stale_ros
rm -f "${SETUP_LOG}" "${FLOOR_LOG}" "${RVIZ_LOG}"

DISPLAY= \
TASK_MODE=side_grip_hold \
ASSUME_ALREADY_AT_SIDE_GRIP=false \
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
source /home/ssu/Azas/install/setup.bash
set -u

ros2 launch azas_bringup doosan_moveit_rviz_only.launch.py model:=m0609 \
  > >(tee "${RVIZ_LOG}") 2>&1 &
RVIZ_PID=$!
sleep "${RVIZ_START_WAIT_SEC:-3}"

ros2 run azas_motion doosan_moveit_grasped_tumbler_to_dispenser_node \
  --ros-args \
  -p task_mode:=floor \
  -p assume_already_at_side_grip:=true \
  -p execute_motion:=true \
  -p start_delay_sec:=0.0 \
  -p joint_1_deg:="${J1_DEG:-159.0}" \
  -p joint_2_deg:="${J2_DEG:--43.0}" \
  -p joint_3_deg:="${J3_DEG:--105.0}" \
  -p joint_4_deg:="${J4_DEG:--81.0}" \
  -p joint_5_deg:="${J5_DEG:-85.0}" \
  -p joint_6_deg:="${J6_DEG:-31.0}" \
  -p enable_demo_obstacle:=false \
  -p enable_obstacle_detour:=false \
  -p floor_target_x:="${FLOOR_TARGET_X}" \
  -p floor_target_y:="${FLOOR_TARGET_Y}" \
  -p floor_target_z:="${FLOOR_TARGET_Z}" \
  -p floor_approach_z:="${FLOOR_APPROACH_Z}" \
  -p controller_action_wait_sec:="${CONTROLLER_ACTION_WAIT_SEC}" \
  > >(tee "${FLOOR_LOG}") 2>&1 &
FLOOR_PID=$!

deadline=$((SECONDS + 90))
while ! grep -q "DONE: side-grasped tumbler moved to demo floor target" "${FLOOR_LOG}" 2>/dev/null; do
  if ! kill -0 "${FLOOR_PID}" 2>/dev/null; then
    echo "ERROR: floor transfer node exited before completing floor target motion." >&2
    exit 1
  fi
  if (( SECONDS > deadline )); then
    echo "ERROR: timed out waiting for side-grip to floor transfer." >&2
    exit 1
  fi
  sleep 1
done

echo "DONE: RViz side-grip current posture to floor transfer completed."
SCRIPT_DONE=true
if [[ "${KEEP_STACK_AFTER_DONE:-false}" == "true" ]]; then
  echo "KEEP_STACK_AFTER_DONE=true: RViz and ROS stack remain running. Press Ctrl+C in this terminal to stop."
  wait
fi
