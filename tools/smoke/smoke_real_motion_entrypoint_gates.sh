#!/usr/bin/env bash
set -euo pipefail

# Verify run_robot_real.sh refuses unsafe entry states.
# This script does not allow reaching ros2 launch and sends no motion commands.

STAMP="${STAMP:-/tmp/azas_smoke_real_motion_gate_stamp}"
rm -f "${STAMP}"

expect_fail() {
  local label="$1"
  shift
  if "$@" >/tmp/azas_smoke_real_motion_gate.out 2>/tmp/azas_smoke_real_motion_gate.err; then
    echo "[FAIL] ${label}: command unexpectedly passed"
    sed -n '1,80p' /tmp/azas_smoke_real_motion_gate.out
    return 1
  fi
  echo "[PASS] ${label}: refused as expected"
}

expect_fail "missing strict stamp" env \
  LIVE_GATE_STAMP="${STAMP}" \
  /home/ssu/Azas/tools/run/run_robot_real.sh

expect_fail "cup-to-dispenser press missing strict stamp" env \
  LIVE_GATE_STAMP="${STAMP}" \
  /home/ssu/Azas/tools/run/run_cup_to_dispenser_press_real.sh

expect_fail "shake missing strict stamp" env \
  LIVE_GATE_STAMP="${STAMP}" \
  /home/ssu/Azas/tools/run/run_rule_based_shake_real.sh

{
  echo "strict=false"
  echo "timestamp=$(date -Is)"
} >"${STAMP}"
expect_fail "non-strict stamp" env \
  LIVE_GATE_STAMP="${STAMP}" \
  /home/ssu/Azas/tools/run/run_robot_real.sh

expect_fail "cup-to-dispenser press non-strict stamp" env \
  LIVE_GATE_STAMP="${STAMP}" \
  /home/ssu/Azas/tools/run/run_cup_to_dispenser_press_real.sh

expect_fail "shake non-strict stamp" env \
  LIVE_GATE_STAMP="${STAMP}" \
  /home/ssu/Azas/tools/run/run_rule_based_shake_real.sh

{
  echo "strict=true"
  echo "timestamp=$(date -Is)"
} >"${STAMP}"
expect_fail "strict stamp but placeholder config" env \
  LIVE_GATE_STAMP="${STAMP}" \
  LIVE_GATE_MAX_AGE_SEC=600 \
  /home/ssu/Azas/tools/run/run_robot_real.sh

expect_fail "cup-to-dispenser press strict stamp but placeholder config" env \
  LIVE_GATE_STAMP="${STAMP}" \
  LIVE_GATE_MAX_AGE_SEC=600 \
  /home/ssu/Azas/tools/run/run_cup_to_dispenser_press_real.sh

expect_fail "shake strict stamp but placeholder config" env \
  LIVE_GATE_STAMP="${STAMP}" \
  LIVE_GATE_MAX_AGE_SEC=600 \
  /home/ssu/Azas/tools/run/run_rule_based_shake_real.sh

echo "[PASS] real-motion entrypoint fail-closed checks passed"
