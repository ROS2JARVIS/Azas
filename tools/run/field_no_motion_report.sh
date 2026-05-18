#!/usr/bin/env bash
set -euo pipefail

# Field no-motion report for deciding whether the robot/RG2 can move to the
# strict live gate. This script does not command Doosan motion or RG2 services.

REPORT="${REPORT:-/tmp/azas_field_no_motion_report.txt}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHECKS_DIR="${ROOT_DIR}/tools/checks"
RUN_DIR="${ROOT_DIR}/tools/run"
RUN_DEPTH_SAMPLE="${RUN_DEPTH_SAMPLE:-true}"
RUN_LID_STABILITY="${RUN_LID_STABILITY:-false}"
RUN_CUP_STABILITY="${RUN_CUP_STABILITY:-false}"
USE_CUP_LID_SEQUENCE="${USE_CUP_LID_SEQUENCE:-true}"
STABILITY_DURATION="${STABILITY_DURATION:-5}"
STABILITY_MIN_SAMPLES="${STABILITY_MIN_SAMPLES:-5}"
STABILITY_MIN_DETECTED_RATIO="${STABILITY_MIN_DETECTED_RATIO:-0.7}"
STRICT_LIVE_GATE="${STRICT_LIVE_GATE:-false}"
GATE_STAMP="${GATE_STAMP:-/tmp/azas_live_hardware_gates_passed}"
LIVE_GATE_MAX_AGE_SEC="${LIVE_GATE_MAX_AGE_SEC:-600}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
failures=0

mkdir -p "${ROS_LOG_DIR}"
rm -f "${REPORT}"

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

run_optional_step() {
  local enabled="$1"
  local label="$2"
  shift 2
  if [[ "${enabled}" != "true" ]]; then
    section "${label}"
    echo "[SKIP] ${label}: disabled" | tee -a "${REPORT}"
    return 0
  fi
  run_step "${label}" "$@"
}

{
  echo "# Azas Field No-Motion Report"
  date -Is
  echo
  echo "Scope: camera/perception/service/config diagnostics only."
  echo "No Doosan motion command and no RG2 open/close request is sent."
  echo "strict_live_gate=${STRICT_LIVE_GATE}"
  echo "run_depth_sample=${RUN_DEPTH_SAMPLE}"
  echo "run_lid_stability=${RUN_LID_STABILITY}"
  echo "run_cup_stability=${RUN_CUP_STABILITY}"
  echo "use_cup_lid_sequence=${USE_CUP_LID_SEQUENCE}"
  echo "gate_stamp=${GATE_STAMP}"
  echo "live_gate_max_age_sec=${LIVE_GATE_MAX_AGE_SEC}"
} | tee -a "${REPORT}"

run_step "Connection stage" "${CHECKS_DIR}/check_connection_stage.sh" || true

run_optional_step "${RUN_DEPTH_SAMPLE}" "Depth projection sample" \
  "${CHECKS_DIR}/check_depth_projection_sample.sh" || true

if [[ "${RUN_LID_STABILITY}" == "true" && "${RUN_CUP_STABILITY}" == "true" && "${USE_CUP_LID_SEQUENCE}" == "true" ]]; then
  run_step "Cup/lid detection stability sequence" env \
    STABILITY_DURATION="${STABILITY_DURATION}" \
    STABILITY_MIN_SAMPLES="${STABILITY_MIN_SAMPLES}" \
    STABILITY_MIN_DETECTED_RATIO="${STABILITY_MIN_DETECTED_RATIO}" \
    "${CHECKS_DIR}/check_cup_lid_sequence.sh" || true

  if grep -q '^\[RESULT\] lid: PASS$' "${REPORT}" 2>/dev/null; then
    echo "[RESULT] Lid detection stability: PASS" | tee -a "${REPORT}"
  else
    echo "[RESULT] Lid detection stability: FAIL" | tee -a "${REPORT}"
    failures=$((failures + 1))
  fi

  if grep -q '^\[RESULT\] cup/tumbler body: PASS$' "${REPORT}" 2>/dev/null; then
    echo "[RESULT] Cup detection stability: PASS" | tee -a "${REPORT}"
  else
    echo "[RESULT] Cup detection stability: FAIL" | tee -a "${REPORT}"
    failures=$((failures + 1))
  fi
else
  run_optional_step "${RUN_LID_STABILITY}" "Lid detection stability" \
    "${CHECKS_DIR}/check_detection_stability.sh" \
    --expect-class lid \
    --duration "${STABILITY_DURATION}" \
    --min-samples "${STABILITY_MIN_SAMPLES}" \
    --min-detected-ratio "${STABILITY_MIN_DETECTED_RATIO}" || true

  run_optional_step "${RUN_CUP_STABILITY}" "Cup detection stability" \
    "${CHECKS_DIR}/check_detection_stability.sh" \
    --expect-class cup \
    --duration "${STABILITY_DURATION}" \
    --min-samples "${STABILITY_MIN_SAMPLES}" \
    --min-detected-ratio "${STABILITY_MIN_DETECTED_RATIO}" || true
fi

run_step "Live hardware gate" env \
  STRICT="${STRICT_LIVE_GATE}" \
  GATE_STAMP="${GATE_STAMP}" \
  "${CHECKS_DIR}/check_live_hardware_gates.sh" || true

{
  echo
  echo "## Decision"
  if [[ "${STRICT_LIVE_GATE}" == "true" && -f "${GATE_STAMP}" && "$(grep -x "strict=true" "${GATE_STAMP}" || true)" == "strict=true" ]]; then
    now_sec="$(date +%s)"
    stamp_sec="$(stat -c %Y "${GATE_STAMP}")"
    age_sec=$((now_sec - stamp_sec))
    if (( age_sec <= LIVE_GATE_MAX_AGE_SEC )); then
      echo "[PASS] Fresh strict live gate stamp exists: ${GATE_STAMP} age=${age_sec}s"
      echo "Next allowed entrypoint: ${RUN_DIR}/run_robot_real.sh"
    else
      echo "[BLOCKED] Strict live gate stamp is stale: ${GATE_STAMP} age=${age_sec}s > ${LIVE_GATE_MAX_AGE_SEC}s"
      failures=$((failures + 1))
    fi
  else
    echo "[BLOCKED] Real motion remains blocked until strict live gate passes and writes ${GATE_STAMP}."
    echo "Typical next step: connect/start Doosan and RG2 services, fill measured calibration/safety config, then rerun:"
    echo "  STRICT_LIVE_GATE=true RUN_LID_STABILITY=true RUN_CUP_STABILITY=true ${RUN_DIR}/field_no_motion_report.sh"
    if [[ "${STRICT_LIVE_GATE}" == "true" ]]; then
      failures=$((failures + 1))
    fi
  fi
  echo
  echo "failures=${failures}"
  echo "Report: ${REPORT}"
} | tee -a "${REPORT}"

if [[ "${STRICT_LIVE_GATE}" == "true" && "${failures}" -ne 0 ]]; then
  exit 1
fi
