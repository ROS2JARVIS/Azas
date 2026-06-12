#!/usr/bin/env bash
set -euo pipefail

# Course-material execution path for the requested dispenser cycle:
# 1) Doosan MoveIt bringup as in 25장 (virtual now, real later by MODE/HOST)
# 2) Azas MoveItPy node follows 26~28장: plan() -> robot.execute(blocking=True)
# 3) RViz robot motion is controller-backed /joint_states. No fake joint publisher.
# 4) Default RVIZ_MODE=clean replaces noisy MoveIt MotionPlanning RViz with a
#    lean RobotModel/marker view so the planned-path ghost robot does not flicker.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/log/manual}"
MODE="${MODE:-virtual}"
ROBOT_NAME="${ROBOT_NAME:-dsr01}"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-12345}"
MODEL="${MODEL:-m0609}"
COLOR="${COLOR:-white}"
RT_HOST="${RT_HOST:-192.168.137.50}"
JOINT_STATE_RELAY_INPUT_TOPIC="${JOINT_STATE_RELAY_INPUT_TOPIC:-/${ROBOT_NAME}/joint_states}"
JOINT_STATES_TOPIC="${JOINT_STATES_TOPIC:-/joint_states}"
MOVEIT_CONTROLLER_ACTION="${MOVEIT_CONTROLLER_ACTION:-/${ROBOT_NAME}/dsr_moveit_controller/follow_joint_trajectory}"
CONTROLLER_SETTLE_SEC="${CONTROLLER_SETTLE_SEC:-5}"
START_DOOSAN="${START_DOOSAN:-auto}"  # auto|1|0; auto reuses an existing Doosan/MoveIt session.
RVIZ_ONLY="${RVIZ_ONLY:-0}"            # 1 forces virtual/sim bringup so robot.execute cannot command real hardware.
START_DELAY_SEC="${START_DELAY_SEC:-22}"
JOINT_WAIT_SEC="${JOINT_WAIT_SEC:-60}"
DISPENSER_ID="${DISPENSER_ID:-1}"
PRESS_COUNT="${PRESS_COUNT:-2}"
PRESS_ONLY="${PRESS_ONLY:-0}"     # 1 = measured press joints + Z-only pump only; skips cup place/return IK.
RVIZ_MODE="${RVIZ_MODE:-clean}"   # bringup|clean|none
KEEP_RVIZ_ON_FAIL="${KEEP_RVIZ_ON_FAIL:-0}"
KEEP_ALIVE_AFTER_DONE="${KEEP_ALIVE_AFTER_DONE:-1}"
PRESERVE_PREVIEW_SESSION_AFTER_DONE="${PRESERVE_PREVIEW_SESSION_AFTER_DONE:-0}"
REPLACE_EXISTING_RVIZ="${REPLACE_EXISTING_RVIZ:-0}"
RESET_EXISTING_VIRTUAL_PREVIEW="${RESET_EXISTING_VIRTUAL_PREVIEW:-0}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${ROOT_DIR}/src/azas_bringup/rviz/azas_cocktail_collision_preview.rviz}"
COURSE_RVIZ_CONFIG="${COURSE_RVIZ_CONFIG:-/home/ssu/ros2_ws/install/dsr_moveit_config_m0609/share/dsr_moveit_config_m0609/launch/moveit.rviz}"
DISPENSER_COLLISION_ENABLED="${DISPENSER_COLLISION_ENABLED:-1}"
# The measured combined box is the glass-bottle/body area, not the press button/head.
# Keep markers visible in RViz by default, but do not feed this draft body box into
# MoveIt collision checking for the press stroke unless explicitly requested.
DISPENSER_COLLISION_OBJECTS="${DISPENSER_COLLISION_OBJECTS:-1}"
DISPENSER_COLLISION_EXCLUDE_IDS="${DISPENSER_COLLISION_EXCLUDE_IDS:-dispenser_head_nozzle_merged_horizontal_spout_box}"
REMOVE_COURSE_WORKSPACE_WALLS="${REMOVE_COURSE_WORKSPACE_WALLS:-0}"
WORKSPACE_COLLISION_ENABLED="${WORKSPACE_COLLISION_ENABLED:-1}"
FULL_COLLISION_SCENE_ENABLED="${FULL_COLLISION_SCENE_ENABLED:-1}"
FULL_COLLISION_SHOW_CEILING="${FULL_COLLISION_SHOW_CEILING:-0}"
SHOW_LINK6_GRIPPER="${SHOW_LINK6_GRIPPER:-1}"
START_JOINT_STATE_RELAY="${START_JOINT_STATE_RELAY:-auto}"
DISPENSER_COLLISION_CONFIG="${DISPENSER_COLLISION_CONFIG:-${ROOT_DIR}/install/azas_bringup/share/azas_bringup/config/measured_dispenser_collision.yaml}"
if [[ ! -f "${DISPENSER_COLLISION_CONFIG}" ]]; then
  DISPENSER_COLLISION_CONFIG="${ROOT_DIR}/src/azas_bringup/config/measured_dispenser_collision.yaml"
fi
SAFETY_CONFIG="${SAFETY_CONFIG:-${ROOT_DIR}/install/azas_bringup/share/azas_bringup/config/safety.yaml}"
if [[ ! -f "${SAFETY_CONFIG}" ]]; then
  SAFETY_CONFIG="${ROOT_DIR}/src/azas_bringup/config/safety.yaml"
fi
CALIBRATION_CONFIG="${CALIBRATION_CONFIG:-${ROOT_DIR}/install/azas_bringup/share/azas_bringup/config/calibration.yaml}"
if [[ ! -f "${CALIBRATION_CONFIG}" ]]; then
  CALIBRATION_CONFIG="${ROOT_DIR}/src/azas_bringup/config/calibration.yaml"
fi
mkdir -p "${LOG_DIR}"

if [[ "${RVIZ_ONLY}" == "1" || "${RVIZ_ONLY}" == "true" ]]; then
  MODE=virtual
  HOST=127.0.0.1
  if [[ "${KEEP_RVIZ_ON_FAIL}" == "0" ]]; then
    KEEP_RVIZ_ON_FAIL=1
  fi
  echo "[Azas] RVIZ_ONLY=${RVIZ_ONLY}: forcing MODE=virtual HOST=127.0.0.1 START_DOOSAN=${START_DOOSAN}"
  echo "[Azas] RVIZ_ONLY=${RVIZ_ONLY}: RVIZ_MODE=${RVIZ_MODE}; KEEP_RVIZ_ON_FAIL=${KEEP_RVIZ_ON_FAIL}"
  pkill -f 'workspace_collision_scene_node' 2>/dev/null || true
  echo "[Azas] RVIZ_ONLY=${RVIZ_ONLY}: stopped stale workspace_collision_scene_node publishers."
fi

cleanup() {
  if [[ "${PRESERVE_PREVIEW_SESSION_AFTER_DONE}" == "1" || "${PRESERVE_PREVIEW_SESSION_AFTER_DONE}" == "true" ]]; then
    echo "[Azas] PRESERVE_PREVIEW_SESSION_AFTER_DONE=${PRESERVE_PREVIEW_SESSION_AFTER_DONE}: keeping virtual Doosan/RViz preview session for the next group."
    return 0
  fi
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
existing="$(pgrep -af 'dsr_bringup2_moveit|move_group|ros2_control_node|run_emulator|DRCF' | grep -v "$$" | grep -v 'pgrep -af' || true)"
if [[ -n "${existing}" && ( "${RVIZ_ONLY}" == "1" || "${RVIZ_ONLY}" == "true" ) && "${RVIZ_MODE}" == "bringup" && ( "${START_DOOSAN}" == "auto" || "${START_DOOSAN}" == "1" || "${START_DOOSAN}" == "true" ) && ( "${RESET_EXISTING_VIRTUAL_PREVIEW}" == "1" || "${RESET_EXISTING_VIRTUAL_PREVIEW}" == "true" ) ]]; then
  if echo "${existing}" | grep -qE 'mode:=virtual|run_emulator|DRCF'; then
    echo "[Azas] Resetting existing virtual Doosan/MoveIt preview so teaching RViz gets robot_description parameters."
    echo "${existing}"
    KILL_RVIZ=1 "${ROOT_DIR}/tools/run/stop_cocktail_motion_preview.sh" || true
    sleep 3
    existing="$(pgrep -af 'dsr_bringup2_moveit|move_group|ros2_control_node|run_emulator|DRCF' | grep -v "$$" | grep -v 'pgrep -af' || true)"
    START_DOOSAN=1
    before_rviz="$(pgrep -x rviz2 || true)"
  else
    echo "[Azas] Existing Doosan/MoveIt session does not look virtual; refusing to reset it from RVIZ_ONLY preview." >&2
  fi
fi
if [[ "${STRICT_SINGLE_SESSION}" == "1" ]]; then
  if [[ -n "${existing}" ]]; then
    if [[ "${START_DOOSAN}" == "auto" || "${START_DOOSAN}" == "0" || "${START_DOOSAN}" == "false" ]]; then
      START_DOOSAN=0
      echo "[Azas] Reusing existing Doosan/MoveIt session; waiting on ${JOINT_STATES_TOPIC}."
      echo "${existing}"
    else
      echo '[Azas] Refusing: an existing Doosan/MoveIt session is running. Stop it first to avoid RViz state jumping.' >&2
      echo "${existing}" >&2
      exit 2
    fi
  fi
fi
if [[ "${START_DOOSAN}" == "auto" ]]; then
  START_DOOSAN=1
fi

if pgrep -af 'm0609_shake_joint_state_node|side_grasp_ik_preview_node' | grep -v 'pgrep -af' >/dev/null; then
  echo '[Azas] Refusing: fake RViz joint publisher is still running.' >&2
  pgrep -af 'm0609_shake_joint_state_node|side_grasp_ik_preview_node' | grep -v 'pgrep -af' >&2 || true
  exit 1
fi

before_rviz="$(pgrep -x rviz2 || true)"

if [[ "${START_DOOSAN}" == "1" || "${START_DOOSAN}" == "true" ]]; then
  ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
    name:="${ROBOT_NAME}" \
    mode:="${MODE}" \
    model:="${MODEL}" \
    host:="${HOST}" \
    port:="${PORT}" \
    color:="${COLOR}" \
    rt_host:="${RT_HOST}" \
    >"${LOG_DIR}/course_dispenser_bringup.log" 2>&1 &
  PIDS+=("$!")

  echo "[Azas] Doosan launch: name=${ROBOT_NAME} mode=${MODE} host=${HOST} port=${PORT} rt_host=${RT_HOST}"
  sleep "${START_DELAY_SEC}"
else
  echo "[Azas] Doosan launch skipped: START_DOOSAN=${START_DOOSAN}"
  : >"${LOG_DIR}/course_dispenser_bringup.log"
fi
echo "[Azas] Waiting for controller joint states on ${JOINT_STATES_TOPIC}"
echo "[Azas] Joint-state relay source: ${JOINT_STATE_RELAY_INPUT_TOPIC}"
echo "[Azas] MoveIt controller action: ${MOVEIT_CONTROLLER_ACTION}"
if [[ "${PRESS_ONLY}" == "1" || "${PRESS_ONLY}" == "true" ]]; then
  echo "[Azas] PRESS_ONLY=${PRESS_ONLY}: RViz will show measured press joints + Z-only pump strokes only."
  echo "[Azas] PRESS_ONLY=${PRESS_ONLY}: skipping cup placement/return IK paths so press motion can be judged directly."
fi

if [[ "${START_JOINT_STATE_RELAY}" == "1" || "${START_JOINT_STATE_RELAY}" == "true" || "${START_JOINT_STATE_RELAY}" == "auto" ]]; then
  if [[ "${JOINT_STATES_TOPIC}" == "/joint_states" && "${JOINT_STATE_RELAY_INPUT_TOPIC}" != "/joint_states" ]]; then
    if ! pgrep -af "joint_state_relay.py.*input_topic:=${JOINT_STATE_RELAY_INPUT_TOPIC}.*output_topic:=/joint_states|joint_state_relay.py.*output_topic:=/joint_states.*input_topic:=${JOINT_STATE_RELAY_INPUT_TOPIC}" | grep -v 'pgrep -af' >/dev/null; then
      python3 "${ROOT_DIR}/src/dsr_practice/dsr_practice/joint_state_relay.py" \
        --ros-args \
        -r __node:=azas_course_joint_state_relay \
        -p input_topic:="${JOINT_STATE_RELAY_INPUT_TOPIC}" \
        -p output_topic:="${JOINT_STATES_TOPIC}" \
        >"${LOG_DIR}/course_joint_state_relay.log" 2>&1 &
      PIDS+=("$!")
      echo "[Azas] Started joint-state relay: ${JOINT_STATE_RELAY_INPUT_TOPIC} -> ${JOINT_STATES_TOPIC}"
    else
      echo "[Azas] Reusing existing joint-state relay: ${JOINT_STATE_RELAY_INPUT_TOPIC} -> ${JOINT_STATES_TOPIC}"
    fi
  fi
fi

joint_deadline=$((SECONDS + JOINT_WAIT_SEC))
while (( SECONDS < joint_deadline )); do
  if timeout 3 ros2 topic echo "${JOINT_STATES_TOPIC}" --once >"${LOG_DIR}/course_dispenser_joint_state_once.txt" 2>/dev/null; then
    if grep -q '^header:' "${LOG_DIR}/course_dispenser_joint_state_once.txt"; then
      break
    fi
  fi
  sleep 1
done
if ! grep -q '^header:' "${LOG_DIR}/course_dispenser_joint_state_once.txt" 2>/dev/null; then
  echo "[Azas] No fresh ${JOINT_STATES_TOPIC}. MoveItPy cannot mirror the robot in RViz." >&2
  if grep -qE 'Failed to initialize hardware|Wrong state or command interface configuration|INITIAL STATE CALL FAILURE|process has died' "${LOG_DIR}/course_dispenser_bringup.log" 2>/dev/null; then
    echo '[Azas] Doosan virtual bringup failed before joint_state_broadcaster became available.' >&2
    echo "[Azas] Current launch args: NAME=${ROBOT_NAME} MODE=${MODE} HOST=${HOST} PORT=${PORT} MODEL=${MODEL} RT_HOST=${RT_HOST}" >&2
    echo '[Azas] If an emulator was already running, stop stale Doosan emulator/controller processes and rerun.' >&2
  fi
  tail -100 "${LOG_DIR}/course_dispenser_bringup.log" >&2 || true
  exit 1
fi

action_deadline=$((SECONDS + 30))
while (( SECONDS < action_deadline )); do
  if timeout 3 ros2 action list >"${LOG_DIR}/course_dispenser_action_list.txt" 2>/dev/null; then
    if grep -qx "${MOVEIT_CONTROLLER_ACTION}" "${LOG_DIR}/course_dispenser_action_list.txt"; then
      break
    fi
  fi
  sleep 1
done
if ! grep -qx "${MOVEIT_CONTROLLER_ACTION}" "${LOG_DIR}/course_dispenser_action_list.txt" 2>/dev/null; then
  echo "[Azas] Warning: ${MOVEIT_CONTROLLER_ACTION} was not observed before cycle launch." >&2
  tail -80 "${LOG_DIR}/course_dispenser_action_list.txt" >&2 || true
else
  echo "[Azas] Controller action observed; settling ${CONTROLLER_SETTLE_SEC}s before MoveItPy execution."
  sleep "${CONTROLLER_SETTLE_SEC}"
fi

if [[ "${WORKSPACE_COLLISION_ENABLED}" == "1" || "${WORKSPACE_COLLISION_ENABLED}" == "true" ]]; then
  ros2 launch azas_bringup workspace_collision_scene.launch.py \
    publish_period_sec:=1.0 \
    publish_collision_objects:=true \
    table_collision_enabled:=true \
    workspace_boundary_collision_enabled:=true \
    dispenser_collision_enabled:=false \
    >"${LOG_DIR}/workspace_collision_scene.log" 2>&1 &
  PIDS+=("$!")
  echo "[Azas] WORKSPACE_COLLISION_ENABLED=${WORKSPACE_COLLISION_ENABLED}: publishing floor/table + side safety walls on /collision_object and /azas/workspace_collision/markers."
  sleep 2
  timeout 8 ros2 topic echo /azas/workspace_collision/markers >"${LOG_DIR}/workspace_collision_markers.txt" 2>/dev/null || true
  if grep -q 'side_grip_workspace_.*_wall\|side_grip_table' "${LOG_DIR}/workspace_collision_markers.txt"; then
    echo '[Azas] Published workspace safety markers: floor/table + side walls.'
  else
    echo '[Azas] Warning: workspace safety marker sample did not capture table/walls yet.' >&2
    tail -80 "${LOG_DIR}/workspace_collision_scene.log" >&2 || true
  fi
fi

if [[ "${DISPENSER_COLLISION_ENABLED}" == "1" || "${DISPENSER_COLLISION_ENABLED}" == "true" ]]; then
  if [[ "${DISPENSER_COLLISION_OBJECTS}" == "1" || "${DISPENSER_COLLISION_OBJECTS}" == "true" ]]; then
    DISPENSER_COLLISION_OBJECTS_BOOL=true
  else
    DISPENSER_COLLISION_OBJECTS_BOOL=false
  fi
  if [[ "${REMOVE_COURSE_WORKSPACE_WALLS}" == "1" || "${REMOVE_COURSE_WORKSPACE_WALLS}" == "true" ]]; then
    REMOVE_COURSE_WORKSPACE_WALLS_BOOL=true
  else
    REMOVE_COURSE_WORKSPACE_WALLS_BOOL=false
  fi
  ros2 run azas_motion measured_dispenser_collision_scene_node \
    --ros-args \
    -p config_path:="${DISPENSER_COLLISION_CONFIG}" \
    -p publish_period_sec:=1.0 \
    -p publish_collision_objects:="${DISPENSER_COLLISION_OBJECTS_BOOL}" \
    -p collision_object_exclude_ids:="${DISPENSER_COLLISION_EXCLUDE_IDS}" \
    -p remove_course_workspace_collision_objects:="${REMOVE_COURSE_WORKSPACE_WALLS_BOOL}" \
    -p clear_markers_before_publish:=false \
    -p publish_markers:=true \
    >"${LOG_DIR}/measured_dispenser_collision_scene.log" 2>&1 &
  PIDS+=("$!")
  echo "[Azas] Dispenser combined box represents bottle/body only; press pre/contact is derived from press_contact_joints_deg FK, not from this box."
  echo "[Azas] DISPENSER_COLLISION_OBJECTS=${DISPENSER_COLLISION_OBJECTS} (1=add to MoveIt collision scene, 0=RViz markers only)."
  echo "[Azas] DISPENSER_COLLISION_EXCLUDE_IDS=${DISPENSER_COLLISION_EXCLUDE_IDS} (marker-only IDs; not used to block press-contact planning)."
  echo "[Azas] REMOVE_COURSE_WORKSPACE_WALLS=${REMOVE_COURSE_WORKSPACE_WALLS} (0=keep safety walls visible/active; 1=remove only if a legacy path is blocked)."
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

if [[ "${FULL_COLLISION_SCENE_ENABLED}" == "1" || "${FULL_COLLISION_SCENE_ENABLED}" == "true" ]]; then
  if [[ "${FULL_COLLISION_SHOW_CEILING}" == "1" || "${FULL_COLLISION_SHOW_CEILING}" == "true" ]]; then
    FULL_COLLISION_SHOW_CEILING_BOOL=true
  else
    FULL_COLLISION_SHOW_CEILING_BOOL=false
  fi
  if pgrep -af 'collision_scene_rviz_publisher.py|collision_scene_rviz_publisher' | grep -v 'pgrep -af' >/dev/null; then
    echo "[Azas] Reusing existing full collision RViz publisher on /azas/collision_scene/markers."
  else
    python3 "${ROOT_DIR}/src/azas_bringup/azas_bringup/collision_scene_rviz_publisher.py" \
      --ros-args \
      -p safety_config_path:="${SAFETY_CONFIG}" \
      -p dispenser_collision_config_path:="${DISPENSER_COLLISION_CONFIG}" \
      -p calibration_path:="${CALIBRATION_CONFIG}" \
      -p publish_workspace_ceiling:="${FULL_COLLISION_SHOW_CEILING_BOOL}" \
      >"${LOG_DIR}/full_collision_scene_rviz_publisher.log" 2>&1 &
    PIDS+=("$!")
    echo "[Azas] FULL_COLLISION_SCENE_ENABLED=${FULL_COLLISION_SCENE_ENABLED}: publishing table/walls/dispenser/front-hold markers on /azas/collision_scene/markers."
  fi
fi

if [[ "${SHOW_LINK6_GRIPPER}" == "1" || "${SHOW_LINK6_GRIPPER}" == "true" ]]; then
  ros2 launch azas_bringup rg2_link6_tcp.launch.py \
    publish_gripper_collision:=false \
    >"${LOG_DIR}/rg2_link6_tcp.log" 2>&1 &
  PIDS+=("$!")
  echo "[Azas] SHOW_LINK6_GRIPPER=${SHOW_LINK6_GRIPPER}: publishing RG2 link_6 TF only; MoveIt uses the mesh-based RG2 URDF."
fi

if [[ "${DISPENSER_COLLISION_OBJECTS}" == "0" || "${DISPENSER_COLLISION_OBJECTS}" == "false" ]]; then
  python3 "${ROOT_DIR}/tools/run/remove_moveit_collision_objects.py" \
    >"${LOG_DIR}/remove_moveit_collision_objects.log" 2>&1 || {
      echo "[Azas] Warning: failed to remove stale MoveIt collision objects." >&2
      tail -80 "${LOG_DIR}/remove_moveit_collision_objects.log" >&2 || true
    }
  sleep 1
fi

if [[ "${RVIZ_MODE}" == "clean" ]]; then
  # dsr_bringup2_moveit launches its default RViz unconditionally. Replace only
  # the RViz processes that appeared after this script started, preserving any
  # pre-existing RViz windows.
  before_lines="$(printf '%s\n' ${before_rviz:-})"
  for pid in $(pgrep -x rviz2 || true); do
    if [[ "${REPLACE_EXISTING_RVIZ}" == "1" || "${REPLACE_EXISTING_RVIZ}" == "true" ]] || ! grep -qx "${pid}" <<<"${before_lines}"; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
  rviz2 -d "${RVIZ_CONFIG}" >"${LOG_DIR}/course_dispenser_clean_rviz.log" 2>&1 &
  PIDS+=("$!")
elif [[ "${RVIZ_MODE}" == "bringup" && ! ( "${START_DOOSAN}" == "1" || "${START_DOOSAN}" == "true" ) ]]; then
  # Reusing an already-running Doosan/MoveIt session does not reopen the
  # teaching-material RViz. Open that exact config so the preview shows the
  # course-style orange MoveIt robot, not the clean debug RobotModel view.
  if [[ "${REPLACE_EXISTING_RVIZ}" == "1" || "${REPLACE_EXISTING_RVIZ}" == "true" ]]; then
    for pid in $(pgrep -x rviz2 || true); do
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    done
  fi
  if pgrep -x rviz2 >/dev/null; then
    echo "[Azas] Reusing existing RViz window for course-material orange robot view."
  else
    rviz2 -d "${COURSE_RVIZ_CONFIG}" >"${LOG_DIR}/course_dispenser_bringup_rviz.log" 2>&1 &
    PIDS+=("$!")
  fi
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
  press_only:="${PRESS_ONLY}" \
  joint_states_topic:="${JOINT_STATES_TOPIC}" \
  moveit_controller_action:="${MOVEIT_CONTROLLER_ACTION}" \
  trajectory_time_scale:="${TRAJECTORY_TIME_SCALE:-8.0}" \
  press_up_m:="${PRESS_UP_M:-0.05}" \
  cup_pre_grasp_backoff_m:="${CUP_PRE_GRASP_BACKOFF_M:-0.08}" \
  cup_release_retract_m:="${CUP_RELEASE_RETRACT_M:-0.05}" \
  planning_time_sec:="${PLANNING_TIME_SEC:-5.0}" \
  >"${LOG_DIR}/course_dispenser_cycle.log" 2>&1

if grep -qE 'process has died|FAILED:|ABORT|GOAL_TOLERANCE_VIOLATED|No motion plan found|Action client not connected to action server|Failed to send trajectory|MoveIt execution failed' "${LOG_DIR}/course_dispenser_cycle.log"; then
  echo '[Azas] Dispenser cycle failed. See log:' >&2
  tail -120 "${LOG_DIR}/course_dispenser_cycle.log" >&2 || true
  if [[ "${KEEP_RVIZ_ON_FAIL}" == "1" || "${KEEP_RVIZ_ON_FAIL}" == "true" ]]; then
    echo '[Azas] KEEP_RVIZ_ON_FAIL is enabled; leaving RViz/bringup open for inspection. Press Ctrl+C in this terminal to close.' >&2
    trap - EXIT
    wait
  fi
  exit 3
fi
if ! grep -q 'DONE:' "${LOG_DIR}/course_dispenser_cycle.log"; then
  echo '[Azas] Dispenser cycle did not report DONE. See log:' >&2
  tail -120 "${LOG_DIR}/course_dispenser_cycle.log" >&2 || true
  if [[ "${KEEP_RVIZ_ON_FAIL}" == "1" || "${KEEP_RVIZ_ON_FAIL}" == "true" ]]; then
    echo '[Azas] KEEP_RVIZ_ON_FAIL is enabled; leaving RViz/bringup open for inspection. Press Ctrl+C in this terminal to close.' >&2
    trap - EXIT
    wait
  fi
  exit 4
fi

echo "[Azas] Dispenser press cycle finished: MoveItPy plan -> robot.execute -> controller ${JOINT_STATES_TOPIC} -> RViz RobotModel."
echo "[Azas] Logs: ${LOG_DIR}/course_dispenser_bringup.log ${LOG_DIR}/course_dispenser_cycle.log ${LOG_DIR}/measured_dispenser_collision_scene.log ${LOG_DIR}/collision_object_samples.txt ${LOG_DIR}/measured_dispenser_collision_markers.txt"
if [[ "${KEEP_ALIVE_AFTER_DONE}" == "1" || "${KEEP_ALIVE_AFTER_DONE}" == "true" ]]; then
  echo "[Azas] KEEP_ALIVE_AFTER_DONE=${KEEP_ALIVE_AFTER_DONE}: leaving RViz/virtual bringup open. Press Ctrl+C to close."
  wait
fi
