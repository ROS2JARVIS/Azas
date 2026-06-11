#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/log/manual}"
SERVICE_PREFIX="${SERVICE_PREFIX:-dsr01}"
TAIL_LINES="${TAIL_LINES:-80}"
SAMPLE_CURRENT_POSE="${SAMPLE_CURRENT_POSE:-1}"

print_file_tail() {
  local label="$1" path="$2"
  echo "--- ${label}: ${path} ---"
  if [[ -f "${path}" ]]; then
    tail -n "${TAIL_LINES}" "${path}" || true
  else
    echo "[missing] ${path}"
  fi
}

diagnose_doosan_log() {
  local path="$1"
  echo "--- diagnosis ---"
  if [[ ! -f "${path}" ]]; then
    echo "[INFO] Doosan log is missing; real one-click has not started Doosan in this log dir."
    return 0
  fi
  if grep -qE 'Timeout: connect timed out|Connect Failed Please check network state|DRCF connecting ERROR' "${path}"; then
    echo "[FAIL] Doosan controller connection failed before motion services became usable."
    echo "[CAUSE] ROS tried to connect to the configured ROBOT_HOST:12345 but timed out."
    echo "[CHECK] Verify robot controller IP, Ethernet route, pendant state, and that no virtual preview is running."
    echo "[NEXT] bash tools/run/stop_cocktail_motion_preview.sh"
    echo "[NEXT] RECIPE_DISPENSER_IDS=1x1 bash tools/run/check_one_click_cocktail_ready.sh || true"
    return 0
  fi
  if grep -qE 'Wrong state or command interface configuration|missing state interfaces|missing command interfaces' "${path}"; then
    echo "[FAIL] Doosan ros2_control failed to initialize hardware interfaces."
    echo "[CAUSE] This usually follows a controller connection failure or an aborted/stale bringup."
    echo "[NEXT] Stop stale Doosan processes, verify controller network, then rerun real NOW."
    return 0
  fi
  if grep -qE 'mode:=virtual|run_emulator|DRCF' "${path}"; then
    echo "[WARN] Doosan log contains virtual/emulator markers. Real one-click must not use virtual motion services."
    echo "[NEXT] bash tools/run/stop_cocktail_motion_preview.sh"
    return 0
  fi
  echo "[INFO] No common Doosan failure pattern detected in the displayed log."
}

echo "[Azas] Cocktail NOW status report"
echo "[Azas] log_dir=${LOG_DIR} service_prefix=${SERVICE_PREFIX}"

echo "--- process snapshot ---"
pgrep -af 'run_cocktail_now_real|run_one_click_cocktail_real|run_measured_dispenser_recipe_sequence|dsr_bringup2_moveit|run_emulator|DRCF|ros2_control_node|rg2_gripper_node|measured_dispenser_collision_scene_node|tumbler_collision_scene_node' \
  | grep -v "$$" \
  | grep -v 'pgrep -af' || true

print_file_tail "integrated recipe" "${LOG_DIR}/one_click_real_integrated_recipe.log"
print_file_tail "doosan" "${LOG_DIR}/one_click_real_doosan.log"
diagnose_doosan_log "${LOG_DIR}/one_click_real_doosan.log"
print_file_tail "gripper" "${LOG_DIR}/one_click_real_gripper.log"
print_file_tail "collision" "${LOG_DIR}/one_click_real_collision_scene.log"

if [[ -f "${LOG_DIR}/one_click_real_integrated_recipe.log" ]]; then
  SAMPLE_CURRENT_POSE="${SAMPLE_CURRENT_POSE}" \
  SERVICE_PREFIX="${SERVICE_PREFIX}" \
  INTEGRATED_LOG="${LOG_DIR}/one_click_real_integrated_recipe.log" \
  bash "${ROOT_DIR}/tools/run/check_one_click_cocktail_result.sh" || true
else
  echo "[Azas] Result checker skipped: integrated log missing."
fi
