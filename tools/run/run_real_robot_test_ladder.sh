#!/usr/bin/env bash
set -euo pipefail

# Staged real-robot test ladder for Azas.
#
# Default behavior is diagnostic only. The only stage that can command motion is
# STAGE=pick-real, and it still delegates to the supervised one-shot script with
# the explicit confirmation phrase required by that script.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHECKS_DIR="${ROOT_DIR}/tools/checks"
PICK_DIR="${ROOT_DIR}/tools/pick"
STAGE="${STAGE:-status}"
GATE_STAMP="${GATE_STAMP:-/tmp/azas_live_hardware_gates_passed}"
GATE_MAX_AGE_SEC="${GATE_MAX_AGE_SEC:-600}"
REAL_MOTION_CONFIG_CHECK="${REAL_MOTION_CONFIG_CHECK:-${CHECKS_DIR}/check_real_motion_config.sh}"
CONFIRM_PHRASE="I_UNDERSTAND_THIS_WILL_MOVE_THE_ROBOT"
CONFIRM="${CONFIRM:-}"
RUN_LID_STABILITY="${RUN_LID_STABILITY:-false}"
RUN_CUP_STABILITY="${RUN_CUP_STABILITY:-false}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"

usage() {
  cat <<EOF
Azas real robot test ladder

Usage:
  STAGE=<stage> $0

Stages:
  status       Explain why real robot execution is currently blocked.
  no-hardware  Run repository no-hardware verifier.
  field        Run field no-motion report without writing a real-motion stamp.
  live-gate    Run strict live hardware gate and write ${GATE_STAMP} only on pass.
  observe-dry  Run supervised observe-only flow without real motion.
  pick-dry     Run supervised one-shot pick planning flow without real motion.
  pick-real    Run supervised one-shot real pick. Requires:
                 CONFIRM=${CONFIRM_PHRASE}

Common options via environment:
  GATE_STAMP=${GATE_STAMP}
  RUN_LID_STABILITY=${RUN_LID_STABILITY}
  RUN_CUP_STABILITY=${RUN_CUP_STABILITY}
  COLOR_TOPIC, DEPTH_TOPIC, CAMERA_INFO_TOPIC, SERVICE_PREFIX

Recommended order:
  STAGE=status $0
  STAGE=no-hardware $0
  STAGE=field RUN_LID_STABILITY=true RUN_CUP_STABILITY=true $0
  STAGE=live-gate RUN_LID_STABILITY=true RUN_CUP_STABILITY=true $0
  STAGE=observe-dry $0
  STAGE=pick-dry $0
  STAGE=pick-real CONFIRM=${CONFIRM_PHRASE} $0
EOF
}

source_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  source "${ROOT_DIR}/install/setup.bash" 2>/dev/null || true
  source /home/ssu/ros2_ws/install/setup.bash 2>/dev/null || true
  set -u
}

require_fresh_gate_stamp() {
  if [[ ! -f "${GATE_STAMP}" ]] || ! grep -qx "strict=true" "${GATE_STAMP}"; then
    echo "[BLOCKED] Missing strict live gate stamp: ${GATE_STAMP}"
    echo "[NEXT] STAGE=live-gate RUN_LID_STABILITY=true RUN_CUP_STABILITY=true $0"
    return 1
  fi
  now_sec="$(date +%s)"
  stamp_sec="$(stat -c %Y "${GATE_STAMP}")"
  age_sec=$((now_sec - stamp_sec))
  if (( age_sec > GATE_MAX_AGE_SEC )); then
    echo "[BLOCKED] Strict live gate stamp is stale: ${age_sec}s > ${GATE_MAX_AGE_SEC}s"
    echo "[NEXT] STAGE=live-gate RUN_LID_STABILITY=true RUN_CUP_STABILITY=true $0"
    return 1
  fi
  if ! "${REAL_MOTION_CONFIG_CHECK}"; then
    echo "[BLOCKED] Real-motion calibration/safety config gate failed."
    return 1
  fi
}

run_status() {
  "${CHECKS_DIR}/explain_real_robot_blockers.sh"
}

run_no_hardware() {
  "${CHECKS_DIR}/verify_control_readiness.sh"
}

run_field() {
  env \
    RUN_LID_STABILITY="${RUN_LID_STABILITY}" \
    RUN_CUP_STABILITY="${RUN_CUP_STABILITY}" \
    STRICT_LIVE_GATE=false \
    GATE_STAMP="${GATE_STAMP}" \
    "${ROOT_DIR}/tools/run/field_no_motion_report.sh"
}

run_live_gate() {
  env \
    RUN_LID_STABILITY="${RUN_LID_STABILITY}" \
    RUN_CUP_STABILITY="${RUN_CUP_STABILITY}" \
    STRICT_LIVE_GATE=true \
    GATE_STAMP="${GATE_STAMP}" \
    "${ROOT_DIR}/tools/run/field_no_motion_report.sh"
}

run_observe_dry() {
  source_ros
  "${PICK_DIR}/run_supervised_real_single_cup_pick.py" \
    --observe-only \
    --dry-run
}

run_pick_dry() {
  source_ros
  "${PICK_DIR}/run_supervised_real_single_cup_pick.py" \
    --dry-run
}

run_pick_real() {
  if [[ "${CONFIRM}" != "${CONFIRM_PHRASE}" ]]; then
    echo "[BLOCKED] pick-real requires CONFIRM=${CONFIRM_PHRASE}"
    return 1
  fi
  require_fresh_gate_stamp
  source_ros
  "${PICK_DIR}/run_supervised_real_single_cup_pick.py" \
    --enable-real-motion \
    --confirm "${CONFIRM_PHRASE}"
}

case "${STAGE}" in
  help|-h|--help)
    usage
    ;;
  status)
    run_status
    ;;
  no-hardware)
    run_no_hardware
    ;;
  field)
    run_field
    ;;
  live-gate)
    run_live_gate
    ;;
  observe-dry)
    run_observe_dry
    ;;
  pick-dry)
    run_pick_dry
    ;;
  pick-real)
    run_pick_real
    ;;
  *)
    echo "[FAIL] Unknown STAGE=${STAGE}"
    usage
    exit 2
    ;;
esac
