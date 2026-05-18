#!/usr/bin/env bash
set -euo pipefail

# One-command field orchestration after the robot and camera are connected.
# It starts Doosan real no-motion bringup, starts Azas safe dry-run, runs the
# strict acceptance gates. It does not enter real motion unless
# RUN_REAL_AFTER_ACCEPTANCE=true is set explicitly. The final real-motion
# entrypoint still enforces its own motion hold, strict-stamp, config, and
# typed-confirmation checks.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHECKS_DIR="${ROOT_DIR}/tools/checks"
RUN_DIR="${ROOT_DIR}/tools/run"
ROBOT_HOST="${ROBOT_HOST:-}"
ROBOT_NAME="${ROBOT_NAME:-}"
SERVICE_PREFIX="${SERVICE_PREFIX:-${ROBOT_NAME}}"
RG2_IP="${RG2_IP:-192.168.1.1}"
SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID:-2}"
START_DOOSAN="${START_DOOSAN:-true}"
START_DRYRUN="${START_DRYRUN:-true}"
RUN_ACCEPTANCE="${RUN_ACCEPTANCE:-true}"
RUN_REAL_AFTER_ACCEPTANCE="${RUN_REAL_AFTER_ACCEPTANCE:-false}"
DRYRUN_STARTUP_SEC="${DRYRUN_STARTUP_SEC:-12}"
DOOSAN_STARTUP_SEC="${DOOSAN_STARTUP_SEC:-12}"
LOG_DIR="${LOG_DIR:-/tmp/azas_connected_robot_control}"

mkdir -p "${LOG_DIR}"

doosan_pid=""
dryrun_pid=""

cleanup() {
  if [[ -n "${dryrun_pid}" ]] && kill -0 "${dryrun_pid}" 2>/dev/null; then
    kill "${dryrun_pid}" 2>/dev/null || true
    wait "${dryrun_pid}" 2>/dev/null || true
  fi
  if [[ "${RUN_REAL_AFTER_ACCEPTANCE}" != "true" ]]; then
    if [[ -n "${doosan_pid}" ]] && kill -0 "${doosan_pid}" 2>/dev/null; then
      kill "${doosan_pid}" 2>/dev/null || true
      wait "${doosan_pid}" 2>/dev/null || true
    fi
  fi
}
trap cleanup EXIT

echo "[Azas] Connected robot control orchestration"
echo "[Azas] robot_host=${ROBOT_HOST:-<required when START_DOOSAN=true>} robot_name=${ROBOT_NAME:-<none>} service_prefix=${SERVICE_PREFIX:-<none>}"
echo "[Azas] rg2_ip=${RG2_IP} selected_dispenser_id=${SELECTED_DISPENSER_ID}"
echo "[Azas] start_doosan=${START_DOOSAN} start_dryrun=${START_DRYRUN} run_acceptance=${RUN_ACCEPTANCE} run_real_after_acceptance=${RUN_REAL_AFTER_ACCEPTANCE}"

if [[ "${START_DOOSAN}" == "true" ]]; then
  if [[ -z "${ROBOT_HOST}" ]]; then
    echo "[Azas] ROBOT_HOST is required for connected robot control."
    echo "[Azas] Example:"
    echo "  ROBOT_HOST=192.168.137.100 RG2_IP=192.168.1.1 ${RUN_DIR}/run_connected_robot_control.sh"
    exit 1
  fi
  echo "[Azas] Starting Doosan real no-motion bringup in background."
    ROBOT_HOST="${ROBOT_HOST}" \
    ROBOT_NAME="${ROBOT_NAME}" \
    DOOSAN_NO_MOTION_CONFIRM=CONNECT_DOOSAN_NO_MOTION \
    "${RUN_DIR}/run_doosan_real_no_motion_m0609.sh" \
    >"${LOG_DIR}/doosan_real_no_motion.log" 2>&1 &
  doosan_pid="$!"
  sleep "${DOOSAN_STARTUP_SEC}"
  if ! kill -0 "${doosan_pid}" 2>/dev/null; then
    echo "[Azas] Doosan no-motion bringup exited early. Last log lines:"
    tail -n 80 "${LOG_DIR}/doosan_real_no_motion.log" || true
    exit 1
  fi
fi

if [[ "${START_DRYRUN}" == "true" ]]; then
  echo "[Azas] Starting Azas safe dry-run in background."
  SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID}" \
    RG2_IP="${RG2_IP}" \
    "${RUN_DIR}/run_robot_dryrun.sh" \
    >"${LOG_DIR}/robot_dryrun.log" 2>&1 &
  dryrun_pid="$!"
  sleep "${DRYRUN_STARTUP_SEC}"
  if ! kill -0 "${dryrun_pid}" 2>/dev/null; then
    echo "[Azas] Safe dry-run exited early. Last log lines:"
    tail -n 80 "${LOG_DIR}/robot_dryrun.log" || true
    exit 1
  fi
fi

if [[ "${RUN_ACCEPTANCE}" == "true" ]]; then
  echo "[Azas] Running strict no-motion acceptance gates."
  SERVICE_PREFIX="${SERVICE_PREFIX}" \
    RG2_IP="${RG2_IP}" \
    "${CHECKS_DIR}/robot_connection_acceptance.sh"
fi

if [[ "${RUN_REAL_AFTER_ACCEPTANCE}" == "true" ]]; then
  echo "[Azas] Acceptance passed. Stopping dry-run before real-motion entrypoint."
  if [[ -n "${dryrun_pid}" ]] && kill -0 "${dryrun_pid}" 2>/dev/null; then
    kill "${dryrun_pid}" 2>/dev/null || true
    wait "${dryrun_pid}" 2>/dev/null || true
    dryrun_pid=""
  fi
  echo "[Azas] Handing off to gated real-motion entrypoint."
  echo "[Azas] If motion hold, config, or typed confirmation fails, run_robot_real.sh will refuse."
  trap - EXIT
  SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID}" \
    RG2_IP="${RG2_IP}" \
    SERVICE_PREFIX="${SERVICE_PREFIX}" \
    exec "${RUN_DIR}/run_robot_real.sh"
fi

echo "[Azas] Connected robot control checks completed without entering real motion."
