#!/usr/bin/env bash
set -euo pipefail

# Single-command field entrypoint for cup pick readiness.
#
# Intended state: Doosan, RG2, RealSense, and Azas perception bringup are already
# running. This script sends no motion until strict live gates and a dry pick
# pass. Real motion still requires the explicit CONFIRM phrase.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHECKS_DIR="${ROOT_DIR}/tools/checks"
RUN_DIR="${ROOT_DIR}/tools/run"
GATE_STAMP="${GATE_STAMP:-/tmp/azas_live_hardware_gates_passed}"
GATE_MAX_AGE_SEC="${GATE_MAX_AGE_SEC:-600}"
CONFIRM_PHRASE="I_UNDERSTAND_THIS_WILL_MOVE_THE_ROBOT"
CONFIRM="${CONFIRM:-}"
RUN_CUP_STABILITY="${RUN_CUP_STABILITY:-true}"
RUN_LID_STABILITY="${RUN_LID_STABILITY:-false}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"

usage() {
  cat <<EOF
Azas connected cup-pick runner

Prerequisite in other terminals:
  - Doosan bringup is connected
  - RG2 bridge is connected
  - RealSense/Azas perception is publishing live cup pose

Dry readiness only:
  $0

Real one-shot cup pick after all gates pass:
  CONFIRM=${CONFIRM_PHRASE} $0

Environment:
  GATE_STAMP=${GATE_STAMP}
  GATE_MAX_AGE_SEC=${GATE_MAX_AGE_SEC}
  RUN_CUP_STABILITY=${RUN_CUP_STABILITY}
  RUN_LID_STABILITY=${RUN_LID_STABILITY}
  COLOR_TOPIC, DEPTH_TOPIC, CAMERA_INFO_TOPIC, SERVICE_PREFIX
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

echo "[Azas] Stage 1/4: explain current real-robot blockers"
"${CHECKS_DIR}/explain_real_robot_blockers.sh" || true

echo "[Azas] Stage 2/4: strict live gate"
env \
  STRICT=true \
  GATE_STAMP="${GATE_STAMP}" \
  RUN_CUP_STABILITY="${RUN_CUP_STABILITY}" \
  RUN_LID_STABILITY="${RUN_LID_STABILITY}" \
  "${CHECKS_DIR}/check_live_hardware_gates.sh"

echo "[Azas] Stage 3/4: dry one-shot cup pick"
env \
  STAGE=pick-dry \
  GATE_STAMP="${GATE_STAMP}" \
  GATE_MAX_AGE_SEC="${GATE_MAX_AGE_SEC}" \
  "${RUN_DIR}/run_real_robot_test_ladder.sh"

if [[ "${CONFIRM}" != "${CONFIRM_PHRASE}" ]]; then
  echo "[Azas] Dry readiness passed. Real motion was not requested."
  echo "[Azas] To run exactly one real cup pick, rerun:"
  echo "  CONFIRM=${CONFIRM_PHRASE} $0"
  exit 0
fi

echo "[Azas] Stage 4/4: real one-shot cup pick"
env \
  STAGE=pick-real \
  CONFIRM="${CONFIRM_PHRASE}" \
  GATE_STAMP="${GATE_STAMP}" \
  GATE_MAX_AGE_SEC="${GATE_MAX_AGE_SEC}" \
  "${RUN_DIR}/run_real_robot_test_ladder.sh"
