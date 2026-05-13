#!/usr/bin/env bash
set -euo pipefail

# Real robot high-shake entrypoint. This intentionally uses the same gates as
# run_robot_real.sh before allowing MoveLine service calls.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHECKS_DIR="${ROOT_DIR}/tools/checks"
SERVICE_PREFIX="${SERVICE_PREFIX:-}"
LIVE_GATE_STAMP="${LIVE_GATE_STAMP:-/tmp/azas_live_hardware_gates_passed}"
LIVE_GATE_MAX_AGE_SEC="${LIVE_GATE_MAX_AGE_SEC:-600}"
REAL_MOTION_CONFIG_CHECK="${REAL_MOTION_CONFIG_CHECK:-${CHECKS_DIR}/check_real_motion_config.sh}"
MOTION_HOLD_FILE="${MOTION_HOLD_FILE:-/tmp/azas_motion_hold}"
SHAKE_CENTER_X="${SHAKE_CENTER_X:-0.28}"
SHAKE_CENTER_Y="${SHAKE_CENTER_Y:--0.30}"
SHAKE_CENTER_Z="${SHAKE_CENTER_Z:-0.62}"
SHAKE_AMPLITUDE_X="${SHAKE_AMPLITUDE_X:-0.100}"
SHAKE_AMPLITUDE_Y="${SHAKE_AMPLITUDE_Y:-0.040}"
SHAKE_AMPLITUDE_Z="${SHAKE_AMPLITUDE_Z:-0.055}"
MIN_SHAKE_Z="${MIN_SHAKE_Z:-0.55}"
DISPENSER_KEEPOUT_RADIUS="${DISPENSER_KEEPOUT_RADIUS:-0.20}"

if [[ -f "${MOTION_HOLD_FILE}" ]]; then
  echo "[Azas] Refusing real robot shake: motion hold is active."
  echo "[Azas] Hold file: ${MOTION_HOLD_FILE}"
  sed -n '1,40p' "${MOTION_HOLD_FILE}" 2>/dev/null || true
  exit 1
fi

if [[ ! -f "${LIVE_GATE_STAMP}" ]]; then
  echo "[Azas] Refusing real robot shake: missing strict live gate stamp."
  echo "[Azas] Run this after dry-run/live bringup passes:"
  echo "  STRICT=true GATE_STAMP=${LIVE_GATE_STAMP} ${CHECKS_DIR}/check_live_hardware_gates.sh"
  exit 1
fi

if ! grep -qx "strict=true" "${LIVE_GATE_STAMP}"; then
  echo "[Azas] Refusing real robot shake: gate stamp is not from STRICT=true."
  exit 1
fi

now_sec="$(date +%s)"
stamp_sec="$(stat -c %Y "${LIVE_GATE_STAMP}")"
age_sec=$((now_sec - stamp_sec))
if (( age_sec > LIVE_GATE_MAX_AGE_SEC )); then
  echo "[Azas] Refusing real robot shake: live gate stamp is too old (${age_sec}s > ${LIVE_GATE_MAX_AGE_SEC}s)."
  echo "[Azas] Re-run: STRICT=true GATE_STAMP=${LIVE_GATE_STAMP} ${CHECKS_DIR}/check_live_hardware_gates.sh"
  exit 1
fi

if ! "${REAL_MOTION_CONFIG_CHECK}"; then
  echo "[Azas] Refusing real robot shake: measured calibration/safety config gate failed."
  exit 1
fi

echo "[Azas] WARNING: this can move the real robot through a lifted shake path."
echo "[Azas] Strict live gate stamp: ${LIVE_GATE_STAMP} age=${age_sec}s"
echo "[Azas] Continue only if ALL are true:"
echo "  - cup is already grasped securely"
echo "  - e-stop is reachable"
echo "  - no person is inside the robot workspace"
echo "  - dispenser, tumbler, cable, table, and camera mount collision risks were checked"
echo "  - lifted shake volume is clear around x=${SHAKE_CENTER_X}, y=${SHAKE_CENTER_Y}, z=${SHAKE_CENTER_Z}"
echo
read -r -p "Type ENABLE_REAL_ROBOT_MOTION to continue: " CONFIRM
if [[ "${CONFIRM}" != "ENABLE_REAL_ROBOT_MOTION" ]]; then
  echo "[Azas] Confirmation did not match. Refusing real robot shake."
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
source /home/ssu/ros2_ws/install/setup.bash
set -u

exec ros2 launch jarvis tumbler_shake_sequence.launch.py \
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
  min_shake_z:="${MIN_SHAKE_Z}" \
  dispenser_keepout_radius:="${DISPENSER_KEEPOUT_RADIUS}"
