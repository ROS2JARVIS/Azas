#!/usr/bin/env bash
set -euo pipefail

# Fail-closed config gate for real robot motion.
# This script sends no ROS commands and does not move hardware.

CALIBRATION_FILE="${CALIBRATION_FILE:-/home/ssu/Azas/src/azas_bringup/config/calibration.yaml}"
SAFETY_FILE="${SAFETY_FILE:-/home/ssu/Azas/src/azas_bringup/config/safety.yaml}"
MAX_ALLOWED_VELOCITY_SCALE="${MAX_ALLOWED_VELOCITY_SCALE:-0.25}"
MAX_ALLOWED_ACCELERATION_SCALE="${MAX_ALLOWED_ACCELERATION_SCALE:-0.25}"

failures=0
warnings=0

pass() {
  echo "[PASS] $1"
}

warn() {
  echo "[WARN] $1"
  warnings=$((warnings + 1))
}

fail() {
  echo "[FAIL] $1"
  failures=$((failures + 1))
}

require_file() {
  local file="$1"
  local label="$2"
  if [[ -s "${file}" ]]; then
    pass "${label}: ${file}"
  else
    fail "${label} missing or empty: ${file}"
  fi
}

check_no_placeholders() {
  local file="$1"
  local label="$2"

  if grep -nE ':[[:space:]]*null([[:space:]]*(#.*)?)?$' "${file}" >/tmp/azas_config_nulls.txt; then
    fail "${label} still contains null calibration/safety values"
    sed -n '1,80p' /tmp/azas_config_nulls.txt
  else
    pass "${label} contains no null values"
  fi

  if grep -n '확인 필요' "${file}" >/tmp/azas_config_unconfirmed.txt; then
    fail "${label} still contains unconfirmed values"
    sed -n '1,80p' /tmp/azas_config_unconfirmed.txt
  else
    pass "${label} contains no unconfirmed-value markers"
  fi
}

require_literal() {
  local file="$1"
  local pattern="$2"
  local label="$3"
  if grep -Eq "${pattern}" "${file}"; then
    pass "${label}"
  else
    fail "${label}"
  fi
}

require_dispenser_mapping() {
  local file="$1"
  local dispenser_id="$2"
  local block
  block="$(awk -v id="\"${dispenser_id}\":" '
    $0 ~ "^[[:space:]]*" id "[[:space:]]*$" { capture=1; print; next }
    capture && $0 ~ /^[[:space:]]*"[^"]+":[[:space:]]*$/ { exit }
    capture { print }
  ' "${file}")"

  if [[ -z "${block}" ]]; then
    fail "dispenser ${dispenser_id} calibration block missing"
    return
  fi

  local missing=0
  for key in outlet_pose_xyz_m outlet_pose_rpy_rad press_pose_xyz_m press_pose_rpy_rad clearance_m; do
    if grep -Eq "^[[:space:]]*${key}:[[:space:]]*(\\[[^]]+\\]|[0-9])" <<<"${block}"; then
      :
    else
      missing=1
      fail "dispenser ${dispenser_id} ${key} is not measured"
    fi
  done

  if [[ "${missing}" -eq 0 ]]; then
    pass "dispenser ${dispenser_id} outlet/press pose mapping is measured"
  fi
}

numeric_value() {
  local file="$1"
  local key="$2"
  awk -F: -v key="${key}" '
    $1 ~ "^[[:space:]]*" key "[[:space:]]*$" {
      value=$2
      sub(/#.*/, "", value)
      gsub(/[[:space:]]/, "", value)
      print value
      exit
    }
  ' "${file}"
}

check_scale_limit() {
  local file="$1"
  local key="$2"
  local max_value="$3"
  local label="$4"
  local value
  value="$(numeric_value "${file}" "${key}")"

  if [[ -z "${value}" ]]; then
    fail "${label} missing"
    return
  fi

  if awk -v value="${value}" -v max_value="${max_value}" 'BEGIN { exit !(value > 0 && value <= max_value) }'; then
    pass "${label}: ${value} <= ${max_value}"
  else
    fail "${label} out of allowed range: ${value} > ${max_value} or <= 0"
  fi
}

echo "[Azas] Real-motion config gate. No motion commands will be sent."
echo "[Azas] calibration=${CALIBRATION_FILE}"
echo "[Azas] safety=${SAFETY_FILE}"

require_file "${CALIBRATION_FILE}" "calibration config"
require_file "${SAFETY_FILE}" "safety config"

if [[ -s "${CALIBRATION_FILE}" ]]; then
  check_no_placeholders "${CALIBRATION_FILE}" "calibration config"
  require_literal "${CALIBRATION_FILE}" '^[[:space:]]*base_frame:[[:space:]]*base_link([[:space:]]*(#.*)?)?$' "base frame is explicit"
  require_literal "${CALIBRATION_FILE}" '^[[:space:]]*parent_frame:[[:space:]]*base_link([[:space:]]*(#.*)?)?$' "hand-eye parent frame is explicit"
  require_literal "${CALIBRATION_FILE}" '^[[:space:]]*clearance_m:[[:space:]]*[0-9]' "outlet clearance is numeric"
  for dispenser_id in 1 2 3 4; do
    require_dispenser_mapping "${CALIBRATION_FILE}" "${dispenser_id}"
  done
  if "${CHECKS_DIR:-/home/ssu/Azas/tools/checks}/check_measured_dispenser_geometry.py"; then
    pass "measured dispenser geometry matches launch geometry"
  else
    fail "measured dispenser geometry does not match launch geometry"
  fi
fi

if [[ -s "${SAFETY_FILE}" ]]; then
  check_no_placeholders "${SAFETY_FILE}" "safety config"
  check_scale_limit "${SAFETY_FILE}" "max_velocity_scale" "${MAX_ALLOWED_VELOCITY_SCALE}" "max velocity scale"
  check_scale_limit "${SAFETY_FILE}" "max_acceleration_scale" "${MAX_ALLOWED_ACCELERATION_SCALE}" "max acceleration scale"
  require_literal "${SAFETY_FILE}" '^[[:space:]]*on_detection_failure:[[:space:]]*abort_without_motion([[:space:]]*(#.*)?)?$' "detection failure aborts without motion"
  require_literal "${SAFETY_FILE}" '^[[:space:]]*on_tf_failure:[[:space:]]*abort_without_motion([[:space:]]*(#.*)?)?$' "TF failure aborts without motion"
  require_literal "${SAFETY_FILE}" '^[[:space:]]*on_plan_failure:[[:space:]]*stop_before_execution([[:space:]]*(#.*)?)?$' "planning failure stops before execution"
fi

echo "[Azas] Result: failures=${failures} warnings=${warnings}"
if [[ "${failures}" -ne 0 ]]; then
  exit 1
fi
