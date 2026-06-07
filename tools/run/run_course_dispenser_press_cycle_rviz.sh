#!/usr/bin/env bash
set -euo pipefail

# Course-material execution path for the requested dispenser cycle:
# 1) Doosan MoveIt bringup as in 25장 (virtual now, real later by MODE/HOST)
# 2) Azas MoveItPy node follows 26~28장: plan() -> robot.execute(blocking=True)
# 3) RViz robot motion is controller-backed /joint_states. No fake joint publisher.
# 4) Default RVIZ_MODE=bringup keeps the course/MoveIt RViz, including the orange planned/goal robot display.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/log/manual}"
MODE="${MODE:-virtual}"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-12347}"
MODEL="${MODEL:-m0609}"
COLOR="${COLOR:-white}"
RT_HOST="${RT_HOST:-192.168.137.50}"
START_DELAY_SEC="${START_DELAY_SEC:-22}"
JOINT_WAIT_SEC="${JOINT_WAIT_SEC:-60}"
DISPENSER_ID="${DISPENSER_ID:-1}"
PRESS_COUNT="${PRESS_COUNT:-2}"
RVIZ_MODE="${RVIZ_MODE:-bringup}"   # bringup|clean|none
RVIZ_CONFIG="${RVIZ_CONFIG:-${ROOT_DIR}/src/azas_bringup/rviz/m0609_robot_only.rviz}"
DISPENSER_COLLISION_ENABLED="${DISPENSER_COLLISION_ENABLED:-1}"
# The measured combined box is the glass-bottle/body area, not the press button/head.
# Keep markers visible in RViz by default, but do not feed this draft body box into
# MoveIt collision checking for the press stroke unless explicitly requested.
DISPENSER_COLLISION_OBJECTS="${DISPENSER_COLLISION_OBJECTS:-1}"
DISPENSER_COLLISION_CONFIG="${DISPENSER_COLLISION_CONFIG:-${ROOT_DIR}/install/azas_bringup/share/azas_bringup/config/measured_dispenser_collision.yaml}"
if [[ ! -f "${DISPENSER_COLLISION_CONFIG}" ]]; then
  DISPENSER_COLLISION_CONFIG="${ROOT_DIR}/src/azas_bringup/config/measured_dispenser_collision.yaml"
fi
mkdir -p "${LOG_DIR}"

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT
PIDS=()

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ws_moveit/install/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  source "${ROOT_DIR}/install/setup.bash"
fi
set -u


STRICT_SINGLE_SESSION="${STRICT_SINGLE_SESSION:-1}"
if [[ "${STRICT_SINGLE_SESSION}" == "1" ]]; then
  existing="$(pgrep -af 'dsr_bringup2_moveit|move_group|ros2_control_node|run_emulator' | grep -v "$$" || true)"
  if [[ -n "${existing}" ]]; then
    echo '[Azas] Refusing: an existing Doosan/MoveIt session is running. Stop it first to avoid RViz state jumping.' >&2
    echo "${existing}" >&2
    exit 2
  fi
fi

if pgrep -af 'm0609_shake_joint_state_node|side_grasp_ik_preview_node' >/dev/null; then
  echo '[Azas] Refusing: fake RViz joint publisher is still running.' >&2
  pgrep -af 'm0609_shake_joint_state_node|side_grasp_ik_preview_node' >&2 || true
  exit 1
fi

before_rviz="$(pgrep -x rviz2 || true)"

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  mode:="${MODE}" \
  model:="${MODEL}" \
  host:="${HOST}" \
  port:="${PORT}" \
  color:="${COLOR}" \
  rt_host:="${RT_HOST}" \
  >"${LOG_DIR}/course_dispenser_bringup.log" 2>&1 &
PIDS+=("$!")

sleep "${START_DELAY_SEC}"

joint_deadline=$((SECONDS + JOINT_WAIT_SEC))
while (( SECONDS < joint_deadline )); do
  if timeout 3 ros2 topic echo /joint_states --once >"${LOG_DIR}/course_dispenser_joint_state_once.txt" 2>/dev/null; then
    if grep -q '^header:' "${LOG_DIR}/course_dispenser_joint_state_once.txt"; then
      break
    fi
  fi
  sleep 1
done
if ! grep -q '^header:' "${LOG_DIR}/course_dispenser_joint_state_once.txt" 2>/dev/null; then
  echo '[Azas] No fresh /joint_states. MoveItPy cannot mirror the robot in RViz.' >&2
  tail -100 "${LOG_DIR}/course_dispenser_bringup.log" >&2 || true
  exit 1
fi

if [[ "${DISPENSER_COLLISION_ENABLED}" == "1" || "${DISPENSER_COLLISION_ENABLED}" == "true" ]]; then
  if [[ "${DISPENSER_COLLISION_OBJECTS}" == "1" || "${DISPENSER_COLLISION_OBJECTS}" == "true" ]]; then
    DISPENSER_COLLISION_OBJECTS_BOOL=true
  else
    DISPENSER_COLLISION_OBJECTS_BOOL=false
  fi
  ros2 run azas_motion measured_dispenser_collision_scene_node \
    --ros-args \
    -p config_path:="${DISPENSER_COLLISION_CONFIG}" \
    -p publish_period_sec:=1.0 \
    -p publish_collision_objects:="${DISPENSER_COLLISION_OBJECTS_BOOL}" \
    -p publish_markers:=true \
    >"${LOG_DIR}/measured_dispenser_collision_scene.log" 2>&1 &
  PIDS+=("$!")
  echo "[Azas] Dispenser combined box represents bottle/body only; press pre/contact is derived from press_contact_joints_deg FK, not from this box."
  echo "[Azas] DISPENSER_COLLISION_OBJECTS=${DISPENSER_COLLISION_OBJECTS} (1=add to MoveIt collision scene, 0=RViz markers only)."
  sleep 2
  if [[ "${DISPENSER_COLLISION_OBJECTS}" == "1" || "${DISPENSER_COLLISION_OBJECTS}" == "true" ]]; then
    timeout 8 ros2 topic echo /collision_object >"${LOG_DIR}/collision_object_samples.txt" 2>/dev/null || true
    if grep -q 'dispenser_combined_body_box' "${LOG_DIR}/collision_object_samples.txt"; then
      echo '[Azas] Published measured dispenser collision object: dispenser_combined_body_box'
    elif grep -q 'Publishing measured dispenser collision objects: .*dispenser_combined_body_box' "${LOG_DIR}/measured_dispenser_collision_scene.log"; then
      echo '[Azas] Collision node is publishing dispenser_combined_body_box; RViz should show PlanningScene/marker display when enabled.'
    else
      echo '[Azas] Warning: dispenser_combined_body_box was not observed in collision samples/log.' >&2
      tail -80 "${LOG_DIR}/measured_dispenser_collision_scene.log" >&2 || true
      tail -120 "${LOG_DIR}/collision_object_samples.txt" >&2 || true
    fi
  else
    timeout 8 ros2 topic echo /azas/measured_dispenser_collision/markers >"${LOG_DIR}/measured_dispenser_collision_markers.txt" 2>/dev/null || true
    if grep -q 'dispenser_combined_body_box' "${LOG_DIR}/measured_dispenser_collision_markers.txt"; then
      echo '[Azas] Published RViz marker for measured dispenser body box: dispenser_combined_body_box'
    else
      echo '[Azas] Collision markers enabled; marker sample did not capture label yet.' >&2
      tail -80 "${LOG_DIR}/measured_dispenser_collision_scene.log" >&2 || true
    fi
  fi
fi

if [[ "${RVIZ_MODE}" == "clean" ]]; then
  # dsr_bringup2_moveit launches its default RViz unconditionally. Replace only
  # the RViz processes that appeared after this script started, preserving any
  # pre-existing RViz windows.
  before_lines="$(printf '%s\n' ${before_rviz:-})"
  for pid in $(pgrep -x rviz2 || true); do
    if ! grep -qx "${pid}" <<<"${before_lines}"; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
  rviz2 -d "${RVIZ_CONFIG}" >"${LOG_DIR}/course_dispenser_clean_rviz.log" 2>&1 &
  PIDS+=("$!")
elif [[ "${RVIZ_MODE}" == "none" ]]; then
  before_lines="$(printf '%s\n' ${before_rviz:-})"
  for pid in $(pgrep -x rviz2 || true); do
    if ! grep -qx "${pid}" <<<"${before_lines}"; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
fi

ros2 launch azas_bringup dispenser_press_cycle_moveit.launch.py \
  dispenser_id:="${DISPENSER_ID}" \
  press_count:="${PRESS_COUNT}" \
  trajectory_time_scale:="${TRAJECTORY_TIME_SCALE:-8.0}" \
  press_up_m:="${PRESS_UP_M:-0.02}" \
  cup_pre_grasp_backoff_m:="${CUP_PRE_GRASP_BACKOFF_M:-0.08}" \
  cup_release_retract_m:="${CUP_RELEASE_RETRACT_M:-0.05}" \
  planning_time_sec:="${PLANNING_TIME_SEC:-5.0}" \
  >"${LOG_DIR}/course_dispenser_cycle.log" 2>&1

if grep -qE 'process has died|FAILED:|ABORT|GOAL_TOLERANCE_VIOLATED|No motion plan found' "${LOG_DIR}/course_dispenser_cycle.log"; then
  echo '[Azas] Dispenser cycle failed. See log:' >&2
  tail -120 "${LOG_DIR}/course_dispenser_cycle.log" >&2 || true
  exit 3
fi
if ! grep -q 'DONE:' "${LOG_DIR}/course_dispenser_cycle.log"; then
  echo '[Azas] Dispenser cycle did not report DONE. See log:' >&2
  tail -120 "${LOG_DIR}/course_dispenser_cycle.log" >&2 || true
  exit 4
fi

echo '[Azas] Dispenser press cycle finished: MoveItPy plan -> robot.execute -> controller /joint_states -> RViz RobotModel.'
echo "[Azas] Logs: ${LOG_DIR}/course_dispenser_bringup.log ${LOG_DIR}/course_dispenser_cycle.log ${LOG_DIR}/measured_dispenser_collision_scene.log ${LOG_DIR}/collision_object_samples.txt ${LOG_DIR}/measured_dispenser_collision_markers.txt"
wait
