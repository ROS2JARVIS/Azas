#!/usr/bin/env bash
set -euo pipefail

# One-command REAL robot path for the integrated cocktail dispenser cycle.
# Sequence:
#   1) connect/reuse real Doosan M0609 service namespace,
#   2) connect/reuse RG2 set_width service,
#   3) publish measured dispenser/tumbler collision scene,
#   4) run cup-place -> full-open gripper -> safe lift -> close empty gripper
#      -> measured press pump(s) -> re-grasp/lift cup.
#
# This script intentionally does not ask for cup coordinates. The cup pose is
# supplied by the existing vision/pose pipeline and the measured dispenser poses.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/log/manual}"
INTEGRATED_LOG="${INTEGRATED_LOG:-${LOG_DIR}/one_click_real_integrated_recipe.log}"
ROBOT_HOST="${ROBOT_HOST:-192.168.1.100}"
RT_HOST="${RT_HOST:-0.0.0.0}"
RG2_IP="${RG2_IP:-192.168.1.1}"
RG2_PORT="${RG2_PORT:-502}"
ROBOT_PORT="${ROBOT_PORT:-12345}"
TCP_CHECK_SEC="${TCP_CHECK_SEC:-2}"
RECIPE_DISPENSER_IDS="${RECIPE_DISPENSER_IDS:-${DISPENSER_IDS:-1x1}}"
SERVICE_PREFIX="${SERVICE_PREFIX:-${ROBOT_NAME:-dsr01}}"
ROBOT_NAME="${ROBOT_NAME:-${SERVICE_PREFIX}}"
REAL_COCKTAIL_CONFIRM="${REAL_COCKTAIL_CONFIRM:-}"
KEEP_CONNECTION_AFTER_DONE="${KEEP_CONNECTION_AFTER_DONE:-1}"
DRY_RUN="${DRY_RUN:-0}"
WAIT_SERVICE_SEC="${WAIT_SERVICE_SEC:-45}"
COLLISION_CONFIG="${COLLISION_CONFIG:-${ROOT_DIR}/src/azas_bringup/config/measured_dispenser_collision.yaml}"
RG2_OPEN_SETTLE_SECONDS="${RG2_OPEN_SETTLE_SECONDS:-5.0}"
GRIPPER_SETTLE_SECONDS="${GRIPPER_SETTLE_SECONDS:-2.0}"
PRESS_PRE_LIFT_M="${PRESS_PRE_LIFT_M:-0.300}"
PRESS_TRANSIT_HEIGHT_M="${PRESS_TRANSIT_HEIGHT_M:-0.300}"
PRESS_DEPTH_M="${PRESS_DEPTH_M:-0.080}"
ONE_CLICK_STAGE="init"

mkdir -p "${LOG_DIR}"

summarize_failure() {
  local rc="$1"
  if [[ "${rc}" == "0" ]]; then
    return 0
  fi
  echo "[Azas] FAILED real one-click cocktail cycle rc=${rc} stage=${ONE_CLICK_STAGE}" >&2
  echo "[Azas] Evidence logs:" >&2
  echo "  integrated=${INTEGRATED_LOG}" >&2
  echo "  doosan=${LOG_DIR}/one_click_real_doosan.log" >&2
  echo "  gripper=${LOG_DIR}/one_click_real_gripper.log" >&2
  echo "  collision=${LOG_DIR}/one_click_real_collision_scene.log" >&2
  if [[ -f "${INTEGRATED_LOG}" ]]; then
    echo "--- integrated tail ---" >&2
    tail -80 "${INTEGRATED_LOG}" >&2 || true
  fi
  if [[ -f "${LOG_DIR}/one_click_real_doosan.log" ]]; then
    if grep -qE 'Timeout: connect timed out|Connect Failed Please check network state|DRCF connecting ERROR' "${LOG_DIR}/one_click_real_doosan.log"; then
      echo "[Azas] DIAGNOSIS: Doosan controller connection timed out before motion services were usable." >&2
      echo "[Azas] CHECK: ROBOT_HOST=${ROBOT_HOST}, controller network, pendant state, and stop virtual preview before retry." >&2
      echo "[Azas] NEXT: bash tools/run/stop_cocktail_motion_preview.sh" >&2
      echo "[Azas] NEXT: RECIPE_DISPENSER_IDS=${RECIPE_DISPENSER_IDS} bash tools/run/check_one_click_cocktail_ready.sh || true" >&2
    elif grep -qE 'Wrong state or command interface configuration|missing state interfaces|missing command interfaces' "${LOG_DIR}/one_click_real_doosan.log"; then
      echo "[Azas] DIAGNOSIS: Doosan ros2_control hardware interfaces did not initialize; usually caused by connection failure or stale/aborted bringup." >&2
    fi
  fi
}

usage() {
  cat <<USAGE
Usage:
  REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE \\
  RECIPE_DISPENSER_IDS=1x2 \\
  bash tools/run/run_one_click_cocktail_real.sh

Options via env:
  RECIPE_DISPENSER_IDS=1x1,3x2,4x1   physical dispenser/count sequence
  ROBOT_HOST=192.168.1.100           real Doosan controller IP
  ROBOT_NAME=dsr01                   Doosan ROS namespace; defaults to SERVICE_PREFIX
  SERVICE_PREFIX=dsr01               Doosan service namespace used by motion calls
  RG2_IP=192.168.1.1                 RG2 Modbus IP
  DRY_RUN=1                          print commands without starting motion
  KEEP_CONNECTION_AFTER_DONE=1       leave robot/gripper/collision nodes running
USAGE
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

if [[ "${REAL_COCKTAIL_CONFIRM}" != "ENABLE_REAL_COCKTAIL_SEQUENCE" ]]; then
  echo "[Azas] Refusing real cocktail cycle without explicit confirmation." >&2
  echo "[Azas] This can move the real robot, actuate RG2, and press the dispenser." >&2
  echo "[Azas] Re-run with: REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE" >&2
  exit 2
fi

if [[ "${ROBOT_HOST}" == "127.0.0.1" || "${ROBOT_HOST}" == "localhost" ]]; then
  echo "[Azas] Refusing real cocktail cycle: ROBOT_HOST=${ROBOT_HOST} is not a real controller IP." >&2
  exit 2
fi

trap 'summarize_failure "$?"' EXIT

run_or_print() {
  if [[ "${DRY_RUN}" == "1" || "${DRY_RUN}" == "true" ]]; then
    printf '[DRY_RUN] %q ' "$@"
    printf '\n'
  else
    "$@"
  fi
}

source_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  source /home/ssu/ws_moveit/install/setup.bash 2>/dev/null || true
  source /home/ssu/ros2_ws/install/setup.bash
  if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
    source "${ROOT_DIR}/install/setup.bash"
  else
    source "${ROOT_DIR}/install/local_setup.bash"
  fi
  set -u
}

wait_for_ros_service() {
  local service="$1"
  local label="$2"
  local deadline=$((SECONDS + WAIT_SERVICE_SEC))
  while (( SECONDS < deadline )); do
    if ros2 service list 2>/dev/null | grep -qx "${service}"; then
      echo "[Azas] ${label} ready: ${service}"
      return 0
    fi
    sleep 1
  done
  echo "[Azas] Timeout waiting for ${label}: ${service}" >&2
  return 1
}

wait_for_motion_services() {
  wait_for_ros_service "/${SERVICE_PREFIX}/motion/move_joint" "Doosan move_joint"
  wait_for_ros_service "/${SERVICE_PREFIX}/motion/move_line" "Doosan move_line"
  wait_for_ros_service "/${SERVICE_PREFIX}/motion/move_wait" "Doosan move_wait"
  wait_for_ros_service "/${SERVICE_PREFIX}/motion/fkin" "Doosan fkin"
  wait_for_ros_service "/${SERVICE_PREFIX}/motion/ikin" "Doosan ikin"
  wait_for_ros_service "/${SERVICE_PREFIX}/motion/check_motion" "Doosan check_motion"
  wait_for_ros_service "/${SERVICE_PREFIX}/system/get_robot_state" "Doosan get_robot_state"
  wait_for_ros_service "/${SERVICE_PREFIX}/aux_control/get_current_posj" "Doosan get_current_posj"
  wait_for_ros_service "/${SERVICE_PREFIX}/aux_control/get_current_posx" "Doosan get_current_posx"
}

check_robot_tcp_before_bringup() {
  if [[ "${DRY_RUN}" == "1" || "${DRY_RUN}" == "true" ]]; then
    echo "[DRY_RUN] check TCP ${ROBOT_HOST}:${ROBOT_PORT} before starting real Doosan bringup"
    return 0
  fi
  if command -v nc >/dev/null 2>&1; then
    if timeout "${TCP_CHECK_SEC}s" nc -z "${ROBOT_HOST}" "${ROBOT_PORT}" >/dev/null 2>&1; then
      echo "[Azas] Doosan TCP reachable: ${ROBOT_HOST}:${ROBOT_PORT}"
      return 0
    fi
    echo "[Azas] Refusing to start real Doosan bringup: ${ROBOT_HOST}:${ROBOT_PORT} is not reachable." >&2
    echo "[Azas] Check controller IP/network/pendant state, then rerun readiness." >&2
    return 2
  fi
  echo "[Azas] nc not installed; skipping Doosan TCP preflight."
}

service_exists() {
  local service="$1"
  ros2 service list 2>/dev/null | grep -qx "${service}"
}

call_empty_service() {
  local service="$1"
  local type="$2"
  python3 "${ROOT_DIR}/tools/run/ros_call_empty_service.py" "${service}" "${type}" --timeout 8.0
}

verify_doosan_motion_ready() {
  if [[ "${DRY_RUN}" == "1" || "${DRY_RUN}" == "true" ]]; then
    echo "[DRY_RUN] verify /${SERVICE_PREFIX}/system/get_robot_state == robot_state=1 and /${SERVICE_PREFIX}/motion/check_motion status=0"
    return 0
  fi
  local state_output motion_output
  echo "[Azas] Verifying Doosan robot state before integrated motion."
  state_output="$(call_empty_service "/${SERVICE_PREFIX}/system/get_robot_state" "dsr_msgs2/srv/GetRobotState")"
  echo "--- get_robot_state ---"
  echo "${state_output}"
  if ! grep -Eq '(^|[[:space:]])(robot_state|state)=1($|[[:space:]])' <<<"${state_output}"; then
    echo "[Azas] Refusing integrated motion: robot_state is not STATE_STANDBY(1)." >&2
    return 2
  fi
  motion_output="$(call_empty_service "/${SERVICE_PREFIX}/motion/check_motion" "dsr_msgs2/srv/CheckMotion")"
  echo "--- check_motion ---"
  echo "${motion_output}"
  if ! grep -Eq '(^|[[:space:]])status=0($|[[:space:]])' <<<"${motion_output}"; then
    echo "[Azas] Refusing integrated motion: check_motion status is not 0." >&2
    return 2
  fi
}

start_real_doosan_if_needed() {
  local virtual_matches
  virtual_matches="$(pgrep -af 'dsr_bringup2_moveit|run_emulator|DRCF|ros2_control_node' | grep -v "$$" | grep -v 'run_one_click_cocktail_real.sh' | grep -v 'pgrep -af' | grep -v 'grep -E' | grep -E 'mode:=virtual|run_emulator|DRCF' || true)"
  if [[ -n "${virtual_matches}" ]]; then
    echo "[Azas] Refusing real cocktail cycle: an active Doosan session looks VIRTUAL/emulated." >&2
    echo "[Azas] Stop the RViz/virtual preview before real motion, then rerun this script." >&2
    echo "[Azas] Command: bash tools/run/stop_cocktail_motion_preview.sh" >&2
    echo "${virtual_matches}" >&2
    return 2
  fi

  if service_exists "/${SERVICE_PREFIX}/motion/move_joint"; then
    echo "[Azas] Reusing existing non-virtual Doosan services under /${SERVICE_PREFIX}; checking full service set."
    wait_for_motion_services
    return 0
  fi
  check_robot_tcp_before_bringup
  echo "[Azas] Starting real Doosan bringup: ROBOT_HOST=${ROBOT_HOST} ROBOT_NAME=${ROBOT_NAME} RT_HOST=${RT_HOST}"
  if [[ "${DRY_RUN}" == "1" || "${DRY_RUN}" == "true" ]]; then
    echo "[DRY_RUN] ROBOT_HOST=${ROBOT_HOST} ROBOT_NAME=${ROBOT_NAME} RT_HOST=${RT_HOST} DOOSAN_REAL_MOTION_CONFIRM=ENABLE_DOOSAN_REAL_MOTION_BRINGUP tools/run/run_doosan_real_m0609.sh &"
    return 0
  fi
  (
    cd "${ROOT_DIR}"
    ROBOT_HOST="${ROBOT_HOST}" ROBOT_NAME="${ROBOT_NAME}" RT_HOST="${RT_HOST}" \
      DOOSAN_REAL_MOTION_CONFIRM=ENABLE_DOOSAN_REAL_MOTION_BRINGUP \
      tools/run/run_doosan_real_m0609.sh
  ) >"${LOG_DIR}/one_click_real_doosan.log" 2>&1 &
  DOOSAN_PID=$!
  echo "[Azas] Doosan pid=${DOOSAN_PID} log=${LOG_DIR}/one_click_real_doosan.log"
  wait_for_motion_services
}

start_gripper_if_needed() {
  if service_exists "/jarvis/rg2/set_width"; then
    echo "[Azas] Reusing existing RG2 services; checking full service set."
    wait_for_ros_service "/jarvis/rg2/set_width" "RG2 set_width"
    wait_for_ros_service "/jarvis/rg2/open" "RG2 open"
    wait_for_ros_service "/jarvis/rg2/close" "RG2 close"
    return 0
  fi
  echo "[Azas] Starting RG2 service wrapper: ${RG2_IP}:${RG2_PORT}"
  if [[ "${DRY_RUN}" == "1" || "${DRY_RUN}" == "true" ]]; then
    echo "[DRY_RUN] ros2 launch azas_gripper rg2_trigger.launch.py ip:=${RG2_IP} port:=${RG2_PORT} connect:=true open_width:=1100 close_width:=0 force:=300 settle_seconds:=0.6 &"
    return 0
  fi
  (
    cd "${ROOT_DIR}"
    source_ros
    source "${ROOT_DIR}/install/azas_gripper/share/azas_gripper/package.bash" 2>/dev/null || true
    ros2 launch "${ROOT_DIR}/install/azas_gripper/share/azas_gripper/launch/rg2_trigger.launch.py" \
      ip:="${RG2_IP}" port:="${RG2_PORT}" connect:=true open_width:=1100 close_width:=0 force:=300 settle_seconds:=0.6
  ) >"${LOG_DIR}/one_click_real_gripper.log" 2>&1 &
  GRIPPER_PID=$!
  echo "[Azas] RG2 pid=${GRIPPER_PID} log=${LOG_DIR}/one_click_real_gripper.log"
  wait_for_ros_service "/jarvis/rg2/set_width" "RG2 set_width"
  wait_for_ros_service "/jarvis/rg2/open" "RG2 open"
  wait_for_ros_service "/jarvis/rg2/close" "RG2 close"
}

start_collision_scene() {
  echo "[Azas] Starting measured dispenser/tumbler collision publishers."
  if [[ "${DRY_RUN}" == "1" || "${DRY_RUN}" == "true" ]]; then
    echo "[DRY_RUN] measured_dispenser_collision_scene_node + tumbler_collision_scene_node &"
    return 0
  fi
  pkill -f 'measured_dispenser_collision_scene_node' 2>/dev/null || true
  pkill -f 'tumbler_collision_scene_node' 2>/dev/null || true
  sleep 0.5
  (
    cd "${ROOT_DIR}"
    source_ros
    python3 -m azas_motion.measured_dispenser_collision_scene_node \
      --ros-args \
      -p config_path:="${COLLISION_CONFIG}" \
      -p publish_period_sec:=2.0 \
      -p collision_object_exclude_ids:=dispenser_head_nozzle_merged_horizontal_spout_box \
      -p remove_course_workspace_collision_objects:=true &
    python3 -m azas_motion.tumbler_collision_scene_node \
      --ros-args \
      -p action:=publish_detected \
      -p object_id:=detected_tumbler \
      -p use_lidded_height:=true
  ) >"${LOG_DIR}/one_click_real_collision_scene.log" 2>&1 &
  COLLISION_PID=$!
  echo "[Azas] collision pid=${COLLISION_PID} log=${LOG_DIR}/one_click_real_collision_scene.log"
  sleep 2
}

run_integrated_recipe() {
  echo "[Azas] Running integrated cocktail dispenser cycle: ${RECIPE_DISPENSER_IDS}"
  echo "[Azas] Cycle: cup-place -> RG2 full-open -> high lift -> close empty gripper -> press pump(s) -> re-grasp/lift."
  echo "[Azas] press_pre_lift_m=${PRESS_PRE_LIFT_M} press_transit_height_m=${PRESS_TRANSIT_HEIGHT_M} press_depth_m=${PRESS_DEPTH_M}"
  echo "[Azas] gripper_open_settle_seconds=${RG2_OPEN_SETTLE_SECONDS} gripper_settle_seconds=${GRIPPER_SETTLE_SECONDS}"
  if [[ "${DRY_RUN}" == "1" || "${DRY_RUN}" == "true" ]]; then
    run_or_print \
      python3 "${ROOT_DIR}/tools/run/run_measured_dispenser_recipe_sequence.py" \
        --dispenser-ids "${RECIPE_DISPENSER_IDS}" \
        --service-prefix "${SERVICE_PREFIX}" \
        --press-pre-lift-m "${PRESS_PRE_LIFT_M}" \
        --press-transit-height-m "${PRESS_TRANSIT_HEIGHT_M}" \
        --press-depth-m "${PRESS_DEPTH_M}" \
        --gripper-open-settle-seconds "${RG2_OPEN_SETTLE_SECONDS}" \
        --gripper-settle-seconds "${GRIPPER_SETTLE_SECONDS}" \
        --execute \
        --confirm ENABLE_MEASURED_DISPENSER_RECIPE_SEQUENCE
    return 0
  fi

  set +e
  python3 "${ROOT_DIR}/tools/run/run_measured_dispenser_recipe_sequence.py" \
    --dispenser-ids "${RECIPE_DISPENSER_IDS}" \
    --service-prefix "${SERVICE_PREFIX}" \
    --press-pre-lift-m "${PRESS_PRE_LIFT_M}" \
    --press-transit-height-m "${PRESS_TRANSIT_HEIGHT_M}" \
    --press-depth-m "${PRESS_DEPTH_M}" \
    --gripper-open-settle-seconds "${RG2_OPEN_SETTLE_SECONDS}" \
    --gripper-settle-seconds "${GRIPPER_SETTLE_SECONDS}" \
    --execute \
    --confirm ENABLE_MEASURED_DISPENSER_RECIPE_SEQUENCE 2>&1 | tee "${INTEGRATED_LOG}"
  local rc="${PIPESTATUS[0]}"
  set -e
  return "${rc}"
}

print_post_run_evidence() {
  if [[ "${DRY_RUN}" == "1" || "${DRY_RUN}" == "true" ]]; then
    echo "[DRY_RUN] post-run evidence would sample current posj/posx and integrated log."
    return 0
  fi
  echo "[Azas] POST-RUN EVIDENCE: integrated sequence returned success."
  if [[ -f "${INTEGRATED_LOG}" ]] && grep -q '\[PASS\] measured dispenser recipe sequence completed' "${INTEGRATED_LOG}"; then
    echo "[Azas] PASS marker found in ${INTEGRATED_LOG}"
  else
    echo "[Azas] WARN: integrated command returned 0 but PASS marker was not found in ${INTEGRATED_LOG}" >&2
  fi
  SAMPLE_CURRENT_POSE=0 \
    INTEGRATED_LOG="${INTEGRATED_LOG}" \
    SERVICE_PREFIX="${SERVICE_PREFIX}" \
    bash "${ROOT_DIR}/tools/run/check_one_click_cocktail_result.sh"
  echo "--- final current_posj sample ---"
  call_empty_service "/${SERVICE_PREFIX}/aux_control/get_current_posj" "dsr_msgs2/srv/GetCurrentPosj" || true
  echo "--- final current_posx sample ---"
  call_empty_service "/${SERVICE_PREFIX}/aux_control/get_current_posx" "dsr_msgs2/srv/GetCurrentPosx" || true
}

source_ros
ONE_CLICK_STAGE="config_preflight"
RECIPE_DISPENSER_IDS="${RECIPE_DISPENSER_IDS}" \
  "${ROOT_DIR}/tools/run/check_one_click_cocktail_config.sh"
ONE_CLICK_STAGE="start_real_doosan"
start_real_doosan_if_needed
ONE_CLICK_STAGE="verify_doosan_motion_ready"
verify_doosan_motion_ready
ONE_CLICK_STAGE="start_gripper"
start_gripper_if_needed
ONE_CLICK_STAGE="start_collision_scene"
start_collision_scene
ONE_CLICK_STAGE="run_integrated_recipe"
run_integrated_recipe
ONE_CLICK_STAGE="post_run_evidence"
print_post_run_evidence
ONE_CLICK_STAGE="done"

if [[ "${KEEP_CONNECTION_AFTER_DONE}" == "1" || "${KEEP_CONNECTION_AFTER_DONE}" == "true" ]]; then
  echo "[Azas] DONE. Real robot/RG2/collision background nodes were left running for inspection/reuse."
  echo "[Azas] Logs: ${INTEGRATED_LOG} ${LOG_DIR}/one_click_real_doosan.log ${LOG_DIR}/one_click_real_gripper.log ${LOG_DIR}/one_click_real_collision_scene.log"
else
  echo "[Azas] DONE. KEEP_CONNECTION_AFTER_DONE=0 requested; stopping nodes started by this script."
  for pid in "${COLLISION_PID:-}" "${GRIPPER_PID:-}" "${DOOSAN_PID:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
fi
