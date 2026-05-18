#!/usr/bin/env bash
set -euo pipefail

# One-command no-motion acceptance check after connecting camera, Doosan, and RG2.
# This script does not command Doosan motion and does not call RG2 open/close.

REPORT="${REPORT:-/tmp/azas_robot_connection_acceptance_report.txt}"
STRICT_LIVE_GATE="${STRICT_LIVE_GATE:-true}"
RUN_DEPTH_SAMPLE="${RUN_DEPTH_SAMPLE:-true}"
RUN_LID_STABILITY="${RUN_LID_STABILITY:-true}"
RUN_CUP_STABILITY="${RUN_CUP_STABILITY:-true}"
RUN_HAND_EYE="${RUN_HAND_EYE:-true}"
FIELD_REPORT="${FIELD_REPORT:-/tmp/azas_field_no_motion_report.txt}"
HAND_EYE_REPORT="${HAND_EYE_REPORT:-/tmp/azas_robot_connection_hand_eye_report.txt}"
LIVE_GATE_MAX_AGE_SEC="${LIVE_GATE_MAX_AGE_SEC:-600}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"

mkdir -p "${ROS_LOG_DIR}"
rm -f "${REPORT}"

failures=0

section() {
  local title="$1"
  {
    echo
    echo "## ${title}"
  } | tee -a "${REPORT}"
}

run_step() {
  local label="$1"
  shift
  section "${label}"
  if "$@" 2>&1 | tee -a "${REPORT}"; then
    echo "[RESULT] ${label}: PASS" | tee -a "${REPORT}"
  else
    echo "[RESULT] ${label}: FAIL" | tee -a "${REPORT}"
    failures=$((failures + 1))
    return 1
  fi
}

{
  echo "# Azas Robot Connection Acceptance Report"
  date -Is
  echo
  echo "Scope: post-connection acceptance diagnostics only."
  echo "No Doosan motion command and no RG2 open/close request is sent."
  echo "strict_live_gate=${STRICT_LIVE_GATE}"
  echo "run_depth_sample=${RUN_DEPTH_SAMPLE}"
  echo "run_lid_stability=${RUN_LID_STABILITY}"
  echo "run_cup_stability=${RUN_CUP_STABILITY}"
  echo "run_hand_eye=${RUN_HAND_EYE}"
  echo "live_gate_max_age_sec=${LIVE_GATE_MAX_AGE_SEC}"
} | tee -a "${REPORT}"

run_step "Strict field no-motion report" env \
  REPORT="${FIELD_REPORT}" \
  STRICT_LIVE_GATE="${STRICT_LIVE_GATE}" \
  RUN_DEPTH_SAMPLE="${RUN_DEPTH_SAMPLE}" \
  RUN_LID_STABILITY="${RUN_LID_STABILITY}" \
  RUN_CUP_STABILITY="${RUN_CUP_STABILITY}" \
  LIVE_GATE_MAX_AGE_SEC="${LIVE_GATE_MAX_AGE_SEC}" \
  /home/ssu/Azas/tools/run/field_no_motion_report.sh || true

if [[ "${RUN_HAND_EYE}" == "true" ]]; then
  section "Hand-eye readiness"
  if /home/ssu/Azas/tools/checks/check_hand_eye_readiness.sh >"${HAND_EYE_REPORT}" 2>&1; then
    cat "${HAND_EYE_REPORT}" | tee -a "${REPORT}" >/dev/null || true
    echo "[RESULT] Hand-eye readiness: PASS" | tee -a "${REPORT}"
  else
    cat "${HAND_EYE_REPORT}" | tee -a "${REPORT}" >/dev/null || true
    echo "[RESULT] Hand-eye readiness: FAIL" | tee -a "${REPORT}"
    failures=$((failures + 1))
  fi
else
  section "Hand-eye readiness"
  echo "[SKIP] Hand-eye readiness disabled" | tee -a "${REPORT}"
fi

{
  echo
  echo "## Verdict"
  if [[ "${failures}" -eq 0 ]]; then
    echo "[PASS] Robot connection acceptance passed."
    echo "Next allowed entrypoint remains gated: /home/ssu/Azas/tools/run/run_robot_real.sh"
  else
    echo "[BLOCKED] Robot connection acceptance failed: failures=${failures}"
    echo "Do not run real robot motion. Fix the failed gate(s), then rerun this script."
  fi
  echo "field_report=${FIELD_REPORT}"
  echo "hand_eye_report=${HAND_EYE_REPORT}"
  echo "report=${REPORT}"
} | tee -a "${REPORT}"

if [[ "${failures}" -ne 0 ]]; then
  exit 1
fi
