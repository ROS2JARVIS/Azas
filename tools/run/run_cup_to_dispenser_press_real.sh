#!/usr/bin/env bash
set -euo pipefail

# DSR-style real sequence behind Azas gates:
# 1. live camera detects cup and publishes base_link tumbler pose,
# 2. Jarvis floor-place grasps the cup and places it below the selected outlet,
# 3. Jarvis dispense/lid sequence runs only the dispenser press primitive.
#
# This can move the real robot. It refuses to start unless the same strict
# live-gate stamp and measured config gate used by run_robot_real.sh pass.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHECKS_DIR="${ROOT_DIR}/tools/checks"
SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID:-2}"
RG2_IP="${RG2_IP:-192.168.1.1}"
COLOR_TOPIC="${COLOR_TOPIC:-/camera/camera/color/image_raw}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
PLACE_MOUTH_UNDER_OUTLET="${PLACE_MOUTH_UNDER_OUTLET:-true}"
OUTLET_MOUTH_CLEARANCE="${OUTLET_MOUTH_CLEARANCE:-0.0}"
SERVICE_PREFIX="${SERVICE_PREFIX:-}"
LIVE_GATE_STAMP="${LIVE_GATE_STAMP:-/tmp/azas_live_hardware_gates_passed}"
LIVE_GATE_MAX_AGE_SEC="${LIVE_GATE_MAX_AGE_SEC:-600}"
REAL_MOTION_CONFIG_CHECK="${REAL_MOTION_CONFIG_CHECK:-${CHECKS_DIR}/check_real_motion_config.sh}"
MOTION_HOLD_FILE="${MOTION_HOLD_FILE:-/tmp/azas_motion_hold}"
FLOOR_STATUS_FILE="${FLOOR_STATUS_FILE:-/tmp/azas_cup_to_dispenser_floor_status.txt}"
FLOOR_LOG_FILE="${FLOOR_LOG_FILE:-/tmp/azas_cup_to_dispenser_floor.log}"
PRESS_STATUS_FILE="${PRESS_STATUS_FILE:-/tmp/azas_cup_to_dispenser_press_status.txt}"
PRESS_LOG_FILE="${PRESS_LOG_FILE:-/tmp/azas_cup_to_dispenser_press.log}"

cleanup() {
  if [[ -n "${FLOOR_LAUNCH_PID:-}" ]] && kill -0 "${FLOOR_LAUNCH_PID}" 2>/dev/null; then
    kill "${FLOOR_LAUNCH_PID}" 2>/dev/null || true
    wait "${FLOOR_LAUNCH_PID}" 2>/dev/null || true
  fi
  if [[ -n "${FLOOR_STATUS_PID:-}" ]] && kill -0 "${FLOOR_STATUS_PID}" 2>/dev/null; then
    kill "${FLOOR_STATUS_PID}" 2>/dev/null || true
    wait "${FLOOR_STATUS_PID}" 2>/dev/null || true
  fi
  if [[ -n "${PRESS_LAUNCH_PID:-}" ]] && kill -0 "${PRESS_LAUNCH_PID}" 2>/dev/null; then
    kill "${PRESS_LAUNCH_PID}" 2>/dev/null || true
    wait "${PRESS_LAUNCH_PID}" 2>/dev/null || true
  fi
  if [[ -n "${PRESS_STATUS_PID:-}" ]] && kill -0 "${PRESS_STATUS_PID}" 2>/dev/null; then
    kill "${PRESS_STATUS_PID}" 2>/dev/null || true
    wait "${PRESS_STATUS_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

require_real_motion_gates() {
  if [[ -f "${MOTION_HOLD_FILE}" ]]; then
    echo "[Azas] Refusing cup-to-dispenser press: motion hold is active."
    echo "[Azas] Hold file: ${MOTION_HOLD_FILE}"
    sed -n '1,40p' "${MOTION_HOLD_FILE}" 2>/dev/null || true
    exit 1
  fi

  if [[ ! -f "${LIVE_GATE_STAMP}" ]]; then
    echo "[Azas] Refusing cup-to-dispenser press: missing strict live gate stamp."
    echo "[Azas] Run: STRICT=true GATE_STAMP=${LIVE_GATE_STAMP} ${CHECKS_DIR}/check_live_hardware_gates.sh"
    exit 1
  fi
  if ! grep -qx "strict=true" "${LIVE_GATE_STAMP}"; then
    echo "[Azas] Refusing cup-to-dispenser press: gate stamp is not strict."
    exit 1
  fi

  now_sec="$(date +%s)"
  stamp_sec="$(stat -c %Y "${LIVE_GATE_STAMP}")"
  age_sec=$((now_sec - stamp_sec))
  if (( age_sec > LIVE_GATE_MAX_AGE_SEC )); then
    echo "[Azas] Refusing cup-to-dispenser press: live gate stamp is stale (${age_sec}s)."
    exit 1
  fi

  if ! "${REAL_MOTION_CONFIG_CHECK}"; then
    echo "[Azas] Refusing cup-to-dispenser press: measured calibration/safety config gate failed."
    exit 1
  fi

  echo "[Azas] WARNING: this can move the real robot, actuate RG2, and press the dispenser."
  echo "[Azas] selected_dispenser_id=${SELECTED_DISPENSER_ID}"
  echo "[Azas] outlet alignment: place_mouth_under_outlet=${PLACE_MOUTH_UNDER_OUTLET}, clearance=${OUTLET_MOUTH_CLEARANCE}m"
  echo "[Azas] Continue only if live detection, hand-eye TF, outlet/press pose, workspace, and e-stop are verified."
  read -r -p "Type ENABLE_REAL_ROBOT_MOTION to continue: " CONFIRM
  if [[ "${CONFIRM}" != "ENABLE_REAL_ROBOT_MOTION" ]]; then
    echo "[Azas] Confirmation did not match. Refusing cup-to-dispenser press."
    exit 1
  fi
}

wait_for_status_done() {
  local label="$1"
  local status_file="$2"
  local log_file="$3"
  local loops="$4"

  for _ in $(seq 1 "${loops}"); do
    if grep -q "DONE" "${status_file}" 2>/dev/null || grep -q "DONE" "${log_file}" 2>/dev/null; then
      echo "[Azas] ${label}: DONE"
      return 0
    fi
    if grep -q "FAILED\\|REJECTED\\|STALE" "${status_file}" 2>/dev/null || grep -q "FAILED\\|REJECTED\\|STALE" "${log_file}" 2>/dev/null; then
      echo "[Azas] ${label}: failed"
      sed -n '1,220p' "${log_file}" 2>/dev/null || true
      return 1
    fi
    sleep 0.5
  done

  echo "[Azas] ${label}: timeout waiting for DONE"
  sed -n '1,220p' "${log_file}" 2>/dev/null || true
  return 1
}

require_real_motion_gates
rm -f "${FLOOR_STATUS_FILE}" "${FLOOR_LOG_FILE}" "${PRESS_STATUS_FILE}" "${PRESS_LOG_FILE}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
source /home/ssu/ros2_ws/install/setup.bash
set -u

echo "[Azas] Stage 1/2: detect, side-grasp, place cup below selected outlet"
ros2 launch azas_bringup robot_connection_control.launch.py \
  selected_dispenser_id:="${SELECTED_DISPENSER_ID}" \
  enable_realsense:=true \
  enable_rg2:=true \
  ip:="${RG2_IP}" \
  color_topic:="${COLOR_TOPIC}" \
  depth_topic:="${DEPTH_TOPIC}" \
  camera_info_topic:="${CAMERA_INFO_TOPIC}" \
  place_mouth_under_outlet:="${PLACE_MOUTH_UNDER_OUTLET}" \
  outlet_mouth_clearance:="${OUTLET_MOUTH_CLEARANCE}" \
  service_prefix:="${SERVICE_PREFIX}" \
  execution_stage:=full \
  enable_hardware:=true \
  hardware_confirm:=ENABLE_REAL_ROBOT_MOTION \
  allow_service_control_without_moveit:=true \
  >"${FLOOR_LOG_FILE}" 2>&1 &
FLOOR_LAUNCH_PID=$!

timeout 120s ros2 topic echo /jarvis/tumbler_floor_place/status --field data --no-daemon >"${FLOOR_STATUS_FILE}" &
FLOOR_STATUS_PID=$!
wait_for_status_done "cup placement below outlet" "${FLOOR_STATUS_FILE}" "${FLOOR_LOG_FILE}" 200

kill "${FLOOR_LAUNCH_PID}" 2>/dev/null || true
wait "${FLOOR_LAUNCH_PID}" 2>/dev/null || true
kill "${FLOOR_STATUS_PID}" 2>/dev/null || true
wait "${FLOOR_STATUS_PID}" 2>/dev/null || true

echo "[Azas] Stage 2/2: press selected dispenser"
ros2 launch jarvis dispense_lid_sequence.launch.py \
  selected_dispenser_id:="${SELECTED_DISPENSER_ID}" \
  enable_hardware:=true \
  hardware_confirm:=ENABLE_REAL_ROBOT_MOTION \
  allow_service_control_without_moveit:=true \
  service_prefix:="${SERVICE_PREFIX}" \
  execution_stage:=press \
  >"${PRESS_LOG_FILE}" 2>&1 &
PRESS_LAUNCH_PID=$!

timeout 60s ros2 topic echo /jarvis/dispense_lid_sequence/status --field data --no-daemon >"${PRESS_STATUS_FILE}" &
PRESS_STATUS_PID=$!
wait_for_status_done "dispenser press" "${PRESS_STATUS_FILE}" "${PRESS_LOG_FILE}" 120

echo "[Azas] DONE: cup was placed under the selected outlet and dispenser press completed."
