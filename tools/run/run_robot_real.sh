#!/usr/bin/env bash
set -euo pipefail

# Real robot motion entrypoint. This is intentionally separate from dry-run.
# Use only after /azas/cup_detection is detected:upright and the operator has verified e-stop/workspace safety.

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

if [[ -f "${MOTION_HOLD_FILE}" ]]; then
  echo "[Azas] Refusing real robot motion: motion hold is active."
  echo "[Azas] Hold file: ${MOTION_HOLD_FILE}"
  sed -n '1,40p' "${MOTION_HOLD_FILE}" 2>/dev/null || true
  echo "[Azas] Remove the hazard, then delete the hold file only after safety review."
  exit 1
fi

if [[ ! -f "${LIVE_GATE_STAMP}" ]]; then
  echo "[Azas] Refusing real robot motion: missing strict live gate stamp."
  echo "[Azas] Run this after dry-run/live bringup passes:"
  echo "  STRICT=true GATE_STAMP=${LIVE_GATE_STAMP} ${CHECKS_DIR}/check_live_hardware_gates.sh"
  exit 1
fi

if ! grep -qx "strict=true" "${LIVE_GATE_STAMP}"; then
  echo "[Azas] Refusing real robot motion: gate stamp is not from STRICT=true."
  exit 1
fi

now_sec="$(date +%s)"
stamp_sec="$(stat -c %Y "${LIVE_GATE_STAMP}")"
age_sec=$((now_sec - stamp_sec))
if (( age_sec > LIVE_GATE_MAX_AGE_SEC )); then
  echo "[Azas] Refusing real robot motion: live gate stamp is too old (${age_sec}s > ${LIVE_GATE_MAX_AGE_SEC}s)."
  echo "[Azas] Re-run: STRICT=true GATE_STAMP=${LIVE_GATE_STAMP} ${CHECKS_DIR}/check_live_hardware_gates.sh"
  exit 1
fi

if ! "${REAL_MOTION_CONFIG_CHECK}"; then
  echo "[Azas] Refusing real robot motion: measured calibration/safety config gate failed."
  exit 1
fi

echo "[Azas] WARNING: this can move the real robot."
echo "[Azas] Strict live gate stamp: ${LIVE_GATE_STAMP} age=${age_sec}s"
echo "[Azas] Continue only if ALL are true:"
echo "  - /azas/cup_detection status starts with detected:upright for the tumbler body"
echo "  - detected:lid, rejected:*, or ambiguous detections are not accepted as cup poses"
echo "  - /jarvis/tumbler_dispenser/tumbler_pose is from real camera detection, not demo"
echo "  - cup mouth alignment is intended: place_mouth_under_outlet=${PLACE_MOUTH_UNDER_OUTLET}, clearance=${OUTLET_MOUTH_CLEARANCE}m"
echo "  - e-stop is reachable"
echo "  - no person is inside the robot workspace"
echo "  - cup, dispenser, table, cable, and camera mount collision risks were checked"
echo
read -r -p "Type ENABLE_REAL_ROBOT_MOTION to continue: " CONFIRM
if [[ "${CONFIRM}" != "ENABLE_REAL_ROBOT_MOTION" ]]; then
  echo "[Azas] Confirmation did not match. Refusing real robot motion."
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
source /home/ssu/ros2_ws/install/setup.bash
set -u

exec ros2 launch azas_bringup robot_connection_control.launch.py \
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
  enable_hardware:=true \
  hardware_confirm:=ENABLE_REAL_ROBOT_MOTION \
  allow_service_control_without_moveit:=true
