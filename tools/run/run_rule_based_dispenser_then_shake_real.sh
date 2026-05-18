#!/usr/bin/env bash
set -euo pipefail

# Real robot sequence:
# 1. detect cup from camera
# 2. grasp cup and move to selected dispenser outlet-front hold pose without opening RG2
# 3. move to the lifted shake volume and run the high-shake path
#
# This script intentionally keeps the same strict gates as run_robot_real.sh.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHECKS_DIR="${ROOT_DIR}/tools/checks"
SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID:-2}"
RG2_IP="${RG2_IP:-192.168.1.1}"
COLOR_TOPIC="${COLOR_TOPIC:-/camera/camera/color/image_raw}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
PLACE_MOUTH_UNDER_OUTLET="${PLACE_MOUTH_UNDER_OUTLET:-false}"
OUTLET_MOUTH_CLEARANCE="${OUTLET_MOUTH_CLEARANCE:-0.0}"
SERVICE_PREFIX="${SERVICE_PREFIX:-}"
LIVE_GATE_STAMP="${LIVE_GATE_STAMP:-/tmp/azas_live_hardware_gates_passed}"
LIVE_GATE_MAX_AGE_SEC="${LIVE_GATE_MAX_AGE_SEC:-600}"
REAL_MOTION_CONFIG_CHECK="${REAL_MOTION_CONFIG_CHECK:-${CHECKS_DIR}/check_real_motion_config.sh}"
MOTION_HOLD_FILE="${MOTION_HOLD_FILE:-/tmp/azas_motion_hold}"
OUTLET_HOLD_STATUS_FILE="${OUTLET_HOLD_STATUS_FILE:-/tmp/azas_dispenser_then_shake_outlet_hold_status.txt}"
OUTLET_HOLD_LOG_FILE="${OUTLET_HOLD_LOG_FILE:-/tmp/azas_dispenser_then_shake_outlet_hold.log}"
SHAKE_STATUS_FILE="${SHAKE_STATUS_FILE:-/tmp/azas_dispenser_then_shake_status.txt}"
SHAKE_LOG_FILE="${SHAKE_LOG_FILE:-/tmp/azas_dispenser_then_shake.log}"
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

cleanup() {
  if [[ -n "${OUTLET_HOLD_LAUNCH_PID:-}" ]] && kill -0 "${OUTLET_HOLD_LAUNCH_PID}" 2>/dev/null; then
    kill "${OUTLET_HOLD_LAUNCH_PID}" 2>/dev/null || true
    wait "${OUTLET_HOLD_LAUNCH_PID}" 2>/dev/null || true
  fi
  if [[ -n "${OUTLET_HOLD_STATUS_PID:-}" ]] && kill -0 "${OUTLET_HOLD_STATUS_PID}" 2>/dev/null; then
    kill "${OUTLET_HOLD_STATUS_PID}" 2>/dev/null || true
    wait "${OUTLET_HOLD_STATUS_PID}" 2>/dev/null || true
  fi
  if [[ -n "${SHAKE_LAUNCH_PID:-}" ]] && kill -0 "${SHAKE_LAUNCH_PID}" 2>/dev/null; then
    kill "${SHAKE_LAUNCH_PID}" 2>/dev/null || true
    wait "${SHAKE_LAUNCH_PID}" 2>/dev/null || true
  fi
  if [[ -n "${SHAKE_STATUS_PID:-}" ]] && kill -0 "${SHAKE_STATUS_PID}" 2>/dev/null; then
    kill "${SHAKE_STATUS_PID}" 2>/dev/null || true
    wait "${SHAKE_STATUS_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

require_real_motion_gates() {
  if [[ -f "${MOTION_HOLD_FILE}" ]]; then
    echo "[Azas] Refusing real robot sequence: motion hold is active."
    echo "[Azas] Hold file: ${MOTION_HOLD_FILE}"
    sed -n '1,40p' "${MOTION_HOLD_FILE}" 2>/dev/null || true
    exit 1
  fi

  if [[ ! -f "${LIVE_GATE_STAMP}" ]]; then
    echo "[Azas] Refusing real robot sequence: missing strict live gate stamp."
    echo "[Azas] Run this after dry-run/live bringup passes:"
    echo "  STRICT=true GATE_STAMP=${LIVE_GATE_STAMP} ${CHECKS_DIR}/check_live_hardware_gates.sh"
    exit 1
  fi

  if ! grep -qx "strict=true" "${LIVE_GATE_STAMP}"; then
    echo "[Azas] Refusing real robot sequence: gate stamp is not from STRICT=true."
    exit 1
  fi

  now_sec="$(date +%s)"
  stamp_sec="$(stat -c %Y "${LIVE_GATE_STAMP}")"
  age_sec=$((now_sec - stamp_sec))
  if (( age_sec > LIVE_GATE_MAX_AGE_SEC )); then
    echo "[Azas] Refusing real robot sequence: live gate stamp is too old (${age_sec}s > ${LIVE_GATE_MAX_AGE_SEC}s)."
    echo "[Azas] Re-run: STRICT=true GATE_STAMP=${LIVE_GATE_STAMP} ${CHECKS_DIR}/check_live_hardware_gates.sh"
    exit 1
  fi

  if ! "${REAL_MOTION_CONFIG_CHECK}"; then
    echo "[Azas] Refusing real robot sequence: measured calibration/safety config gate failed."
    exit 1
  fi

  echo "[Azas] WARNING: this can move the real robot through cup grasp, dispenser transfer, and lifted shake."
  echo "[Azas] Strict live gate stamp: ${LIVE_GATE_STAMP} age=${age_sec}s"
  echo "[Azas] Continue only if ALL are true:"
  echo "  - /azas/cup_detection status starts with detected:upright for the tumbler body"
  echo "  - detected:lid, rejected:*, or ambiguous detections are not accepted as cup poses"
  echo "  - /jarvis/tumbler_dispenser/tumbler_pose is from real camera detection, not demo"
  echo "  - outlet-front transfer keeps low side-grasp height unless PLACE_MOUTH_UNDER_OUTLET=true"
  echo "  - e-stop is reachable"
  echo "  - no person is inside the robot workspace"
  echo "  - cup, dispenser, table, cable, camera mount, and lifted shake volume were checked"
  echo "  - RG2 can keep the cup grasped during transfer and shake"
  echo
  read -r -p "Type ENABLE_REAL_ROBOT_MOTION to continue: " CONFIRM
  if [[ "${CONFIRM}" != "ENABLE_REAL_ROBOT_MOTION" ]]; then
    echo "[Azas] Confirmation did not match. Refusing real robot sequence."
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
rm -f "${OUTLET_HOLD_STATUS_FILE}" "${OUTLET_HOLD_LOG_FILE}" "${SHAKE_STATUS_FILE}" "${SHAKE_LOG_FILE}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/local_setup.bash"
set -u

echo "[Azas] Stage 1/2: grasp cup and hold it in front of the selected dispenser outlet"
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
  delivery_mode:=hold_under_outlet \
  execution_stage:=full \
  enable_hardware:=true \
  hardware_confirm:=ENABLE_REAL_ROBOT_MOTION \
  allow_service_control_without_moveit:=true \
  >"${OUTLET_HOLD_LOG_FILE}" 2>&1 &
OUTLET_HOLD_LAUNCH_PID=$!

timeout 90s ros2 topic echo /jarvis/tumbler_floor_place/status --field data --no-daemon >"${OUTLET_HOLD_STATUS_FILE}" &
OUTLET_HOLD_STATUS_PID=$!

wait_for_status_done "dispenser outlet-front hold transfer" "${OUTLET_HOLD_STATUS_FILE}" "${OUTLET_HOLD_LOG_FILE}" 140

kill "${OUTLET_HOLD_LAUNCH_PID}" 2>/dev/null || true
wait "${OUTLET_HOLD_LAUNCH_PID}" 2>/dev/null || true
kill "${OUTLET_HOLD_STATUS_PID}" 2>/dev/null || true
wait "${OUTLET_HOLD_STATUS_PID}" 2>/dev/null || true

echo "[Azas] Stage 2/2: run lifted high-shake while cup remains grasped"
ros2 launch azas_bringup tumbler_shake_sequence.launch.py \
  enable_hardware:=true \
  hardware_confirm:=ENABLE_REAL_ROBOT_MOTION \
  allow_service_control_without_moveit:=true \
  service_prefix:="${SERVICE_PREFIX}" \
  use_visualizer:=false \
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
  >"${SHAKE_LOG_FILE}" 2>&1 &
SHAKE_LAUNCH_PID=$!

timeout 60s ros2 topic echo /jarvis/tumbler_shake_sequence/status --field data --no-daemon >"${SHAKE_STATUS_FILE}" &
SHAKE_STATUS_PID=$!

wait_for_status_done "lifted high-shake" "${SHAKE_STATUS_FILE}" "${SHAKE_LOG_FILE}" 100

echo "[Azas] DONE: cup moved to dispenser outlet-front hold and lifted shake completed."
