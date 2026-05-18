#!/usr/bin/env bash
set -euo pipefail

# Operator-guided no-hardware sequence for checking live lid and cup detection.
# This script only samples /azas/cup_detection through check_detection_stability.sh.
# It sends no Doosan motion command and no RG2 open/close request.

STABILITY_DURATION="${STABILITY_DURATION:-5}"
STABILITY_MIN_SAMPLES="${STABILITY_MIN_SAMPLES:-5}"
STABILITY_MIN_DETECTED_RATIO="${STABILITY_MIN_DETECTED_RATIO:-0.7}"
STABILITY_TOPIC="${STABILITY_TOPIC:-/azas/cup_detection}"
ASSUME_READY="${ASSUME_READY:-false}"

wait_for_object() {
  local label="$1"
  echo
  echo "[Azas] Place the ${label} clearly in the camera view."
  echo "[Azas] Press Enter to sample ${label} stability."
  if [[ "${ASSUME_READY}" != "true" ]]; then
    read -r _
  fi
}

run_stability_check() {
  local label="$1"
  local expected_class="$2"
  shift 2

  wait_for_object "${label}"
  echo "[Azas] Checking ${label} detection stability; no robot/RG2 commands will be sent."

  set +e
  /home/ssu/Azas/tools/checks/check_detection_stability.sh \
    "$@" \
    --topic "${STABILITY_TOPIC}" \
    --duration "${STABILITY_DURATION}" \
    --min-samples "${STABILITY_MIN_SAMPLES}" \
    --min-detected-ratio "${STABILITY_MIN_DETECTED_RATIO}" \
    --expect-class "${expected_class}"
  local status=$?
  set -e

  if [[ ${status} -eq 0 ]]; then
    echo "[RESULT] ${label}: PASS"
  else
    echo "[RESULT] ${label}: FAIL"
  fi
  return "${status}"
}

echo "[Azas] Cup/lid detection sequence"
echo "[Azas] Scope: live perception sampling only; no Doosan motion and no RG2 command."
echo "[Azas] topic=${STABILITY_TOPIC} duration=${STABILITY_DURATION}s min_samples=${STABILITY_MIN_SAMPLES} min_detected_ratio=${STABILITY_MIN_DETECTED_RATIO}"

failures=0
run_stability_check "lid" "lid" "$@" || failures=$((failures + 1))
run_stability_check "cup/tumbler body" "cup" "$@" || failures=$((failures + 1))

if [[ ${failures} -eq 0 ]]; then
  echo
  echo "[PASS] Lid and cup/tumbler-body detection stability passed."
else
  echo
  echo "[FAIL] ${failures} stability check(s) failed."
fi

exit "${failures}"
