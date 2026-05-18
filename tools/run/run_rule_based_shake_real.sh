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
GRASPED_CUP_TEST_MODE="${GRASPED_CUP_TEST_MODE:-false}"
USE_CURRENT_TCP_AS_SHAKE_CENTER="${USE_CURRENT_TCP_AS_SHAKE_CENTER:-false}"
REQUIRE_JOINT_LIMITS="${REQUIRE_JOINT_LIMITS:-true}"
REQUIRE_ROBOT_STANDBY="${REQUIRE_ROBOT_STANDBY:-true}"
JOINT5_MIN_DEG="${JOINT5_MIN_DEG:--135.0}"
JOINT5_MAX_DEG="${JOINT5_MAX_DEG:-135.0}"
CURRENT_SHAKE_CENTER_Z_OFFSET_M="${CURRENT_SHAKE_CENTER_Z_OFFSET_M:-0.0}"
SHAKE_CENTER_X="${SHAKE_CENTER_X:-0.28}"
SHAKE_CENTER_Y="${SHAKE_CENTER_Y:--0.30}"
SHAKE_CENTER_Z="${SHAKE_CENTER_Z:-0.62}"
SHAKE_AMPLITUDE_X="${SHAKE_AMPLITUDE_X:-0.100}"
SHAKE_AMPLITUDE_Y="${SHAKE_AMPLITUDE_Y:-0.040}"
SHAKE_AMPLITUDE_Z="${SHAKE_AMPLITUDE_Z:-0.055}"
SHAKE_CYCLES="${SHAKE_CYCLES:-4}"
SHAKE_APPROACH_HEIGHT="${SHAKE_APPROACH_HEIGHT:-0.10}"
MIN_SHAKE_Z="${MIN_SHAKE_Z:-0.55}"
DISPENSER_KEEPOUT_RADIUS="${DISPENSER_KEEPOUT_RADIUS:-0.20}"
LINE_TIME="${LINE_TIME:-0.0}"
SERVICE_WAIT_TIMEOUT_SEC="${SERVICE_WAIT_TIMEOUT_SEC:-5.0}"
MOTION_RESPONSE_TIMEOUT_SEC="${MOTION_RESPONSE_TIMEOUT_SEC:-10.0}"
RX="${RX:-180.0}"
RY="${RY:-0.0}"
RZ="${RZ:-180.0}"

if [[ -f "${MOTION_HOLD_FILE}" ]]; then
  echo "[Azas] Refusing real robot shake: motion hold is active."
  echo "[Azas] Hold file: ${MOTION_HOLD_FILE}"
  sed -n '1,40p' "${MOTION_HOLD_FILE}" 2>/dev/null || true
  exit 1
fi

age_sec="n/a"
if [[ "${GRASPED_CUP_TEST_MODE}" == "true" ]]; then
  echo "[Azas] Grasped-cup test mode: skipping camera/dispenser calibration gates."
  echo "[Azas] This mode assumes the cup is already grasped and only tests a local shake around current TCP."
else
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
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
source /home/ssu/ros2_ws/install/setup.bash
set -u

parse_robot_state() {
  python3 -c '
import re
import sys
text = sys.stdin.read()
match = re.search(r"robot_state=(\d+)|robot_state:\s*(\d+)", text)
if not match:
    raise SystemExit("could not parse robot_state from ros2 service output")
print(next(group for group in match.groups() if group is not None))
'
}

if [[ "${REQUIRE_ROBOT_STANDBY}" == "true" ]]; then
  echo "[Azas] Checking Doosan robot state before real motion."
  if ! robot_state_output="$(timeout 5s ros2 service call /system/get_robot_state dsr_msgs2/srv/GetRobotState "{}")"; then
    echo "[Azas] Refusing real robot shake: /system/get_robot_state did not respond."
    echo "[Azas] Start Doosan real bringup and confirm the robot network is connected."
    exit 1
  fi
  robot_state="$(printf '%s' "${robot_state_output}" | parse_robot_state)"
  if [[ "${robot_state}" != "1" ]]; then
    echo "[Azas] Refusing real robot shake: robot_state=${robot_state}, expected STATE_STANDBY(1)."
    echo "[Azas] Clear SAFE_OFF/SAFE_STOP/recovery state on the controller before motion."
    exit 1
  fi
fi

parse_first_array() {
  python3 -c '
import re
import sys
text = sys.stdin.read()
match = re.search(r"(?:data|pos)[:=]\s*(?:array\()?\[([^\]]+)\]", text, re.S)
if match:
    values = re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", match.group(1))
else:
    values = []
    in_array = False
    for line in text.splitlines():
        stripped = line.strip()
        if re.match(r"(?:data|pos):\s*$", stripped):
            in_array = True
            continue
        if in_array:
            item = re.match(r"-\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)", stripped)
            if item:
                values.append(item.group(1))
                if len(values) >= 7:
                    break
                continue
            if stripped and not stripped.startswith("-"):
                break
if len(values) < 6:
    raise SystemExit("could not parse numeric array from ros2 service output")
print(" ".join(values[:7]))
'
}

if [[ "${USE_CURRENT_TCP_AS_SHAKE_CENTER}" == "true" ]]; then
  echo "[Azas] Reading current robot TCP/joints for grasped-cup shake."
  if ! current_posx_output="$(timeout 5s ros2 service call /aux_control/get_current_posx dsr_msgs2/srv/GetCurrentPosx "{ref: 0}")"; then
    echo "[Azas] Refusing grasped-cup shake: /aux_control/get_current_posx did not respond."
    echo "[Azas] Check that the Doosan real bringup is connected and the robot is still on the network."
    exit 1
  fi
  if ! current_posj_output="$(timeout 5s ros2 service call /aux_control/get_current_posj dsr_msgs2/srv/GetCurrentPosj "{}")"; then
    echo "[Azas] Refusing grasped-cup shake: /aux_control/get_current_posj did not respond."
    echo "[Azas] Check that the Doosan real bringup is connected and the robot is still on the network."
    exit 1
  fi
  read -r px_mm py_mm pz_mm RX RY RZ _ <<<"$(printf '%s' "${current_posx_output}" | parse_first_array)"
  read -r _ _ _ _ joint5_deg _ <<<"$(printf '%s' "${current_posj_output}" | parse_first_array)"

  if [[ "${REQUIRE_JOINT_LIMITS}" == "true" ]]; then
    python3 - "$joint5_deg" "$JOINT5_MIN_DEG" "$JOINT5_MAX_DEG" <<'PY'
import sys
joint5 = float(sys.argv[1])
lower = float(sys.argv[2])
upper = float(sys.argv[3])
if not lower <= joint5 <= upper:
    print(
        f"[Azas] Refusing grasped-cup shake: joint_5={joint5:.3f} deg is outside "
        f"[{lower:.3f}, {upper:.3f}] deg."
    )
    print("[Azas] Put joint 5 back inside the robot's normal range with pendant/recovery, then rerun.")
    raise SystemExit(1)
PY
  fi

  SHAKE_CENTER_X="$(python3 -c 'import sys; print(f"{float(sys.argv[1]) / 1000.0:.6f}")' "${px_mm}")"
  SHAKE_CENTER_Y="$(python3 -c 'import sys; print(f"{float(sys.argv[1]) / 1000.0:.6f}")' "${py_mm}")"
  SHAKE_CENTER_Z="$(python3 -c 'import sys; print(f"{float(sys.argv[1]) / 1000.0 + float(sys.argv[2]):.6f}")' "${pz_mm}" "${CURRENT_SHAKE_CENTER_Z_OFFSET_M}")"
  MIN_SHAKE_Z="$(python3 -c 'import sys; print(f"{max(0.0, float(sys.argv[1]) - 0.05):.6f}")' "${SHAKE_CENTER_Z}")"
  echo "[Azas] Current TCP shake center: x=${SHAKE_CENTER_X} y=${SHAKE_CENTER_Y} z=${SHAKE_CENTER_Z} rx=${RX} ry=${RY} rz=${RZ}"
fi

echo "[Azas] WARNING: this can move the real robot through a lifted shake path."
if [[ "${age_sec}" == "n/a" ]]; then
  echo "[Azas] Strict live gate stamp: skipped for grasped-cup test mode"
else
  echo "[Azas] Strict live gate stamp: ${LIVE_GATE_STAMP} age=${age_sec}s"
fi
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

exec ros2 launch azas_bringup tumbler_shake_sequence.launch.py \
  enable_hardware:=true \
  hardware_confirm:=ENABLE_REAL_ROBOT_MOTION \
  allow_service_control_without_moveit:=true \
  service_prefix:="${SERVICE_PREFIX}" \
  use_visualizer:=false \
  shake_approach_height:="${SHAKE_APPROACH_HEIGHT}" \
  shake_center_x:="${SHAKE_CENTER_X}" \
  shake_center_y:="${SHAKE_CENTER_Y}" \
  shake_center_z:="${SHAKE_CENTER_Z}" \
  shake_amplitude_x:="${SHAKE_AMPLITUDE_X}" \
  shake_amplitude_y:="${SHAKE_AMPLITUDE_Y}" \
  shake_amplitude_z:="${SHAKE_AMPLITUDE_Z}" \
  shake_cycles:="${SHAKE_CYCLES}" \
  min_shake_z:="${MIN_SHAKE_Z}" \
  dispenser_keepout_radius:="${DISPENSER_KEEPOUT_RADIUS}" \
  rx:="${RX}" \
  ry:="${RY}" \
  rz:="${RZ}" \
  line_time:="${LINE_TIME}" \
  service_wait_timeout_sec:="${SERVICE_WAIT_TIMEOUT_SEC}" \
  motion_response_timeout_sec:="${MOTION_RESPONSE_TIMEOUT_SEC}"
