#!/usr/bin/env bash
set -euo pipefail

# Real robot high-shake entrypoint. This intentionally uses the same gates as
# run_robot_real.sh before allowing Doosan motion service calls.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHECKS_DIR="${ROOT_DIR}/tools/checks"
SERVICE_PREFIX="${SERVICE_PREFIX:-}"
LIVE_GATE_STAMP="${LIVE_GATE_STAMP:-/tmp/azas_live_hardware_gates_passed}"
LIVE_GATE_MAX_AGE_SEC="${LIVE_GATE_MAX_AGE_SEC:-600}"
REAL_MOTION_CONFIG_CHECK="${REAL_MOTION_CONFIG_CHECK:-${CHECKS_DIR}/check_real_motion_config.sh}"
MOTION_HOLD_FILE="${MOTION_HOLD_FILE:-/tmp/azas_motion_hold}"
GRASPED_CUP_TEST_MODE="${GRASPED_CUP_TEST_MODE:-false}"
SKIP_CUP_HOLDER_PICK="${SKIP_CUP_HOLDER_PICK:-false}"
CUP_HOLDER_PICK_CONFIG="${CUP_HOLDER_PICK_CONFIG:-${ROOT_DIR}/install/azas_bringup/share/azas_bringup/config/calibration.yaml}"
CUP_HOLDER_PLACE_FINAL_Z_OFFSET_M="${CUP_HOLDER_PLACE_FINAL_Z_OFFSET_M:-0.0}"
# Operational-only offset for the pre-shake re-grasp from cup_holder.side_grip_place.
# Negative values lower the final grasp pose; calibration.yaml is not modified.
CUP_HOLDER_PICK_Z_OFFSET_M="${CUP_HOLDER_PICK_Z_OFFSET_M:--0.020}"
CUP_HOLDER_PICK_WIDTH_M="${CUP_HOLDER_PICK_WIDTH_M:-0.068}"
CUP_HOLDER_PICK_FORCE_N="${CUP_HOLDER_PICK_FORCE_N:-35.0}"
USE_CURRENT_TCP_AS_SHAKE_CENTER="${USE_CURRENT_TCP_AS_SHAKE_CENTER:-false}"
REQUIRE_JOINT_LIMITS="${REQUIRE_JOINT_LIMITS:-true}"
REQUIRE_ROBOT_STANDBY="${REQUIRE_ROBOT_STANDBY:-true}"
SHAKE_CONTROL_MODE="${SHAKE_CONTROL_MODE:-joint}"
JOINT5_MIN_DEG="${JOINT5_MIN_DEG:-40.0}"
JOINT5_MAX_DEG="${JOINT5_MAX_DEG:-100.0}"
ENFORCE_WRIST_JOINT_LIMITS="${ENFORCE_WRIST_JOINT_LIMITS:-false}"
WRIST_MIN_DEG="${WRIST_MIN_DEG:--135.0}"
WRIST_MAX_DEG="${WRIST_MAX_DEG:-135.0}"
CURRENT_SHAKE_CENTER_Z_OFFSET_M="${CURRENT_SHAKE_CENTER_Z_OFFSET_M:-0.0}"
SHAKE_CENTER_X="${SHAKE_CENTER_X:-0.28}"
SHAKE_CENTER_Y="${SHAKE_CENTER_Y:--0.30}"
SHAKE_CENTER_Z="${SHAKE_CENTER_Z:-0.62}"
SHAKE_AMPLITUDE_X="${SHAKE_AMPLITUDE_X:-0.100}"
SHAKE_AMPLITUDE_Y="${SHAKE_AMPLITUDE_Y:-0.040}"
SHAKE_AMPLITUDE_Z="${SHAKE_AMPLITUDE_Z:-0.055}"
SHAKE_CYCLES="${SHAKE_CYCLES:-3}"
SHAKE_TWIST_RX_DEG="${SHAKE_TWIST_RX_DEG:-6.0}"
SHAKE_TWIST_RY_DEG="${SHAKE_TWIST_RY_DEG:-3.0}"
SHAKE_TWIST_RZ_DEG="${SHAKE_TWIST_RZ_DEG:-22.0}"
SHAKE_APPROACH_HEIGHT="${SHAKE_APPROACH_HEIGHT:-0.10}"
MIN_SHAKE_Z="${MIN_SHAKE_Z:-0.55}"
DISPENSER_KEEPOUT_RADIUS="${DISPENSER_KEEPOUT_RADIUS:-0.20}"
LINE_TIME="${LINE_TIME:-0.0}"
APPROACH_LINE_TIME="${APPROACH_LINE_TIME:-3.5}"
SHAKE_LINE_TIME="${SHAKE_LINE_TIME:-0.40}"
SERVICE_WAIT_TIMEOUT_SEC="${SERVICE_WAIT_TIMEOUT_SEC:-5.0}"
MOTION_RESPONSE_TIMEOUT_SEC="${MOTION_RESPONSE_TIMEOUT_SEC:-10.0}"
RX="${RX:-180.0}"
RY="${RY:-0.0}"
RZ="${RZ:-180.0}"
JOINT_SHAKE_BASE_J1_DEG="${JOINT_SHAKE_BASE_J1_DEG:-0.0}"
JOINT_SHAKE_BASE_J2_DEG="${JOINT_SHAKE_BASE_J2_DEG:--35.0}"
JOINT_SHAKE_BASE_J3_DEG="${JOINT_SHAKE_BASE_J3_DEG:-50.0}"
JOINT_SHAKE_BASE_J4_DEG="${JOINT_SHAKE_BASE_J4_DEG:-0.0}"
JOINT_SHAKE_BASE_J5_DEG="${JOINT_SHAKE_BASE_J5_DEG:-70.0}"
JOINT_SHAKE_BASE_J6_DEG="${JOINT_SHAKE_BASE_J6_DEG:-0.0}"
JOINT_SHAKE_J3_AMPLITUDE_DEG="${JOINT_SHAKE_J3_AMPLITUDE_DEG:-0.0}"
JOINT_SHAKE_J4_AMPLITUDE_DEG="${JOINT_SHAKE_J4_AMPLITUDE_DEG:-18.0}"
JOINT_SHAKE_J5_AMPLITUDE_DEG="${JOINT_SHAKE_J5_AMPLITUDE_DEG:-20.0}"
JOINT_SHAKE_J6_AMPLITUDE_DEG="${JOINT_SHAKE_J6_AMPLITUDE_DEG:-24.0}"
JOINT_SHAKE_J1_MIN_DEG="${JOINT_SHAKE_J1_MIN_DEG:--20.0}"
JOINT_SHAKE_J1_MAX_DEG="${JOINT_SHAKE_J1_MAX_DEG:-5.0}"
JOINT_SHAKE_J2_MIN_DEG="${JOINT_SHAKE_J2_MIN_DEG:--80.0}"
JOINT_SHAKE_J2_MAX_DEG="${JOINT_SHAKE_J2_MAX_DEG:-5.0}"
JOINT_SHAKE_J3_MIN_DEG="${JOINT_SHAKE_J3_MIN_DEG:-0.0}"
JOINT_SHAKE_J3_MAX_DEG="${JOINT_SHAKE_J3_MAX_DEG:-135.0}"
JOINT_SHAKE_MAX_SINGLE_DELTA_DEG="${JOINT_SHAKE_MAX_SINGLE_DELTA_DEG:-75.0}"
APPROACH_JOINT_VELOCITY="${APPROACH_JOINT_VELOCITY:-18.0}"
APPROACH_JOINT_ACCELERATION="${APPROACH_JOINT_ACCELERATION:-22.0}"
APPROACH_JOINT_TIME="${APPROACH_JOINT_TIME:-2.6}"
SHAKE_JOINT_VELOCITY="${SHAKE_JOINT_VELOCITY:-90.0}"
SHAKE_JOINT_ACCELERATION="${SHAKE_JOINT_ACCELERATION:-120.0}"
SHAKE_JOINT_TIME="${SHAKE_JOINT_TIME:-0.0}"
JOINT_SHAKE_PEAK_VELOCITY_LIMIT_DEG_S="${JOINT_SHAKE_PEAK_VELOCITY_LIMIT_DEG_S:-130.0}"
VERIFY_JOINT_TARGETS="${VERIFY_JOINT_TARGETS:-true}"
JOINT_TARGET_TOLERANCE_DEG="${JOINT_TARGET_TOLERANCE_DEG:-8.0}"
JOINT_TARGET_WAIT_EXTRA_SEC="${JOINT_TARGET_WAIT_EXTRA_SEC:-3.0}"
JOINT_TARGET_POLL_SEC="${JOINT_TARGET_POLL_SEC:-0.05}"
REQUIRE_STATE_VALIDITY_FOR_JOINT_SHAKE="${REQUIRE_STATE_VALIDITY_FOR_JOINT_SHAKE:-true}"
STATE_VALIDITY_SERVICE="${STATE_VALIDITY_SERVICE:-/check_state_validity}"
PLANNING_GROUP="${PLANNING_GROUP:-manipulator}"

echo "[Azas] SHAKE START 설명: 컵홀더에 놓인 닫힌 컵을 side grip으로 다시 잡은 뒤 흔드는 단계입니다."
echo "[Azas] 순서: 컵홀더 place 완료 확인 -> 컵홀더 측정 pose로 RG2 side-grip 픽업 -> 들어 올림 -> 관절 쉐이킹 실행."
echo "[Azas] 주의: 이 스크립트는 컵 좌표를 새로 만들지 않으며, calibration.yaml cup_holder.side_grip_place 측정값만 사용합니다."

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
source /home/ssu/ros2_ws/install/setup.bash
source "${ROOT_DIR}/install/setup.bash"
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

prefixed_service() {
  local suffix="${1#/}"
  local prefix="${SERVICE_PREFIX#/}"
  if [[ -n "${prefix}" ]]; then
    printf '/%s/%s' "${prefix}" "${suffix}"
  else
    printf '/%s' "${suffix}"
  fi
}

if [[ "${REQUIRE_ROBOT_STANDBY}" == "true" ]]; then
  echo "[Azas] Checking Doosan robot state before real motion."
  robot_state_service="$(prefixed_service system/get_robot_state)"
  if ! robot_state_output="$(timeout 5s ros2 service call "${robot_state_service}" dsr_msgs2/srv/GetRobotState "{}")"; then
    echo "[Azas] Refusing real robot shake: ${robot_state_service} did not respond."
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
  current_posx_service="$(prefixed_service aux_control/get_current_posx)"
  current_posj_service="$(prefixed_service aux_control/get_current_posj)"
  if ! current_posx_output="$(timeout 5s ros2 service call "${current_posx_service}" dsr_msgs2/srv/GetCurrentPosx "{ref: 0}")"; then
    echo "[Azas] Refusing grasped-cup shake: ${current_posx_service} did not respond."
    echo "[Azas] Check that the Doosan real bringup is connected and the robot is still on the network."
    exit 1
  fi
  if ! current_posj_output="$(timeout 5s ros2 service call "${current_posj_service}" dsr_msgs2/srv/GetCurrentPosj "{}")"; then
    echo "[Azas] Refusing grasped-cup shake: ${current_posj_service} did not respond."
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
echo "  - cup-holder pick completed and cup is grasped securely"
echo "  - e-stop is reachable"
echo "  - no person is inside the robot workspace"
echo "  - dispenser, tumbler, cable, table, and camera mount collision risks were checked"
if [[ "${SHAKE_CONTROL_MODE}" == "joint" || "${SHAKE_CONTROL_MODE}" == "joint_space" || "${SHAKE_CONTROL_MODE}" == "move_joint" ]]; then
  echo "  - joint-space shake is clear around base joints [${JOINT_SHAKE_BASE_J1_DEG}, ${JOINT_SHAKE_BASE_J2_DEG}, ${JOINT_SHAKE_BASE_J3_DEG}, ${JOINT_SHAKE_BASE_J4_DEG}, ${JOINT_SHAKE_BASE_J5_DEG}, ${JOINT_SHAKE_BASE_J6_DEG}]"
  echo "  - joint_1/joint_2 stay near safe space and joint_5 stays inside [${JOINT5_MIN_DEG}, ${JOINT5_MAX_DEG}]"
else
  echo "  - lifted shake volume is clear around x=${SHAKE_CENTER_X}, y=${SHAKE_CENTER_Y}, z=${SHAKE_CENTER_Z}"
fi
echo
read -r -p "Type ENABLE_REAL_ROBOT_MOTION to continue: " CONFIRM
if [[ "${CONFIRM}" != "ENABLE_REAL_ROBOT_MOTION" ]]; then
  echo "[Azas] Confirmation did not match. Refusing real robot shake."
  exit 1
fi

if [[ "${SKIP_CUP_HOLDER_PICK}" != "true" ]]; then
  echo "[Azas] Cup-holder pick is required before shake. Starting measured holder side-grip pickup."
  echo "[Azas] Cup-holder pick Z offset: ${CUP_HOLDER_PICK_Z_OFFSET_M} m (negative lowers grasp pose; calibration unchanged)."
  echo "[Azas] Cup-holder grasp: width=${CUP_HOLDER_PICK_WIDTH_M} m force=${CUP_HOLDER_PICK_FORCE_N} N; shake is conservative to reduce drop risk."
  python3 "${ROOT_DIR}/tools/run/pick_from_cup_holder_side_grip.py" \
    --service-prefix "${SERVICE_PREFIX}" \
    --config "${CUP_HOLDER_PICK_CONFIG}" \
    --approach-velocity 12.0 --approach-acceleration 16.0 \
    --descend-velocity 6.0 --descend-acceleration 10.0 \
    --lift-velocity 12.0 --lift-acceleration 16.0 \
    --place-final-z-offset-m "${CUP_HOLDER_PICK_Z_OFFSET_M}" \
    --timeout-sec 90.0 --target-tolerance-mm 12.0 --verify-timeout-sec 45.0 \
    --ikin-timeout-sec 20.0 --ikin-retries 2 \
    --gripper-grasp-width-m "${CUP_HOLDER_PICK_WIDTH_M}" \
    --gripper-force-n "${CUP_HOLDER_PICK_FORCE_N}" \
    --post-grasp-settle-sec 0.8 \
    --z-max 0.28 \
    --execute --confirm ENABLE_CUP_HOLDER_PICK
  echo "[Azas] Cup-holder pick completed; continuing to shake with grasped cup."
else
  echo "[Azas] Cup-holder pick skipped only because SKIP_CUP_HOLDER_PICK=true was set by a wrapper that already completed it."
fi

exec ros2 launch azas_bringup tumbler_shake_sequence.launch.py \
  enable_hardware:=true \
  hardware_confirm:=ENABLE_REAL_ROBOT_MOTION \
  allow_service_control_without_moveit:=true \
  service_prefix:="${SERVICE_PREFIX}" \
  use_visualizer:=false \
  shake_control_mode:="${SHAKE_CONTROL_MODE}" \
  shake_approach_height:="${SHAKE_APPROACH_HEIGHT}" \
  shake_center_x:="${SHAKE_CENTER_X}" \
  shake_center_y:="${SHAKE_CENTER_Y}" \
  shake_center_z:="${SHAKE_CENTER_Z}" \
  shake_amplitude_x:="${SHAKE_AMPLITUDE_X}" \
  shake_amplitude_y:="${SHAKE_AMPLITUDE_Y}" \
  shake_amplitude_z:="${SHAKE_AMPLITUDE_Z}" \
  shake_cycles:="${SHAKE_CYCLES}" \
  shake_twist_rx_deg:="${SHAKE_TWIST_RX_DEG}" \
  shake_twist_ry_deg:="${SHAKE_TWIST_RY_DEG}" \
  shake_twist_rz_deg:="${SHAKE_TWIST_RZ_DEG}" \
  min_shake_z:="${MIN_SHAKE_Z}" \
  dispenser_keepout_radius:="${DISPENSER_KEEPOUT_RADIUS}" \
  rx:="${RX}" \
  ry:="${RY}" \
  rz:="${RZ}" \
  line_time:="${LINE_TIME}" \
  approach_line_time:="${APPROACH_LINE_TIME}" \
  shake_line_time:="${SHAKE_LINE_TIME}" \
  service_wait_timeout_sec:="${SERVICE_WAIT_TIMEOUT_SEC}" \
  motion_response_timeout_sec:="${MOTION_RESPONSE_TIMEOUT_SEC}" \
  precheck_ikin_joint5:=true \
  enforce_wrist_joint_limits:="${ENFORCE_WRIST_JOINT_LIMITS}" \
  ikin_sol_space:=2 \
  joint5_min_deg:="${JOINT5_MIN_DEG}" \
  joint5_max_deg:="${JOINT5_MAX_DEG}" \
  wrist_min_deg:="${WRIST_MIN_DEG}" \
  wrist_max_deg:="${WRIST_MAX_DEG}" \
  joint_shake_base_j1_deg:="${JOINT_SHAKE_BASE_J1_DEG}" \
  joint_shake_base_j2_deg:="${JOINT_SHAKE_BASE_J2_DEG}" \
  joint_shake_base_j3_deg:="${JOINT_SHAKE_BASE_J3_DEG}" \
  joint_shake_base_j4_deg:="${JOINT_SHAKE_BASE_J4_DEG}" \
  joint_shake_base_j5_deg:="${JOINT_SHAKE_BASE_J5_DEG}" \
  joint_shake_base_j6_deg:="${JOINT_SHAKE_BASE_J6_DEG}" \
  joint_shake_j3_amplitude_deg:="${JOINT_SHAKE_J3_AMPLITUDE_DEG}" \
  joint_shake_j4_amplitude_deg:="${JOINT_SHAKE_J4_AMPLITUDE_DEG}" \
  joint_shake_j5_amplitude_deg:="${JOINT_SHAKE_J5_AMPLITUDE_DEG}" \
  joint_shake_j6_amplitude_deg:="${JOINT_SHAKE_J6_AMPLITUDE_DEG}" \
  joint_shake_j1_min_deg:="${JOINT_SHAKE_J1_MIN_DEG}" \
  joint_shake_j1_max_deg:="${JOINT_SHAKE_J1_MAX_DEG}" \
  joint_shake_j2_min_deg:="${JOINT_SHAKE_J2_MIN_DEG}" \
  joint_shake_j2_max_deg:="${JOINT_SHAKE_J2_MAX_DEG}" \
  joint_shake_j3_min_deg:="${JOINT_SHAKE_J3_MIN_DEG}" \
  joint_shake_j3_max_deg:="${JOINT_SHAKE_J3_MAX_DEG}" \
  joint_shake_max_single_delta_deg:="${JOINT_SHAKE_MAX_SINGLE_DELTA_DEG}" \
  approach_joint_velocity:="${APPROACH_JOINT_VELOCITY}" \
  approach_joint_acceleration:="${APPROACH_JOINT_ACCELERATION}" \
  approach_joint_time:="${APPROACH_JOINT_TIME}" \
  shake_joint_velocity:="${SHAKE_JOINT_VELOCITY}" \
  shake_joint_acceleration:="${SHAKE_JOINT_ACCELERATION}" \
  shake_joint_time:="${SHAKE_JOINT_TIME}" \
  joint_shake_peak_velocity_limit_deg_s:="${JOINT_SHAKE_PEAK_VELOCITY_LIMIT_DEG_S}" \
  verify_joint_targets:="${VERIFY_JOINT_TARGETS}" \
  joint_target_tolerance_deg:="${JOINT_TARGET_TOLERANCE_DEG}" \
  joint_target_wait_extra_sec:="${JOINT_TARGET_WAIT_EXTRA_SEC}" \
  joint_target_poll_sec:="${JOINT_TARGET_POLL_SEC}" \
  require_state_validity_for_joint_shake:="${REQUIRE_STATE_VALIDITY_FOR_JOINT_SHAKE}" \
  state_validity_service:="${STATE_VALIDITY_SERVICE}" \
  planning_group:="${PLANNING_GROUP}"
