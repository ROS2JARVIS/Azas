#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SERVICE_PREFIX="${SERVICE_PREFIX:-${ROBOT_NAME:-dsr01}}"
ROBOT_NAME="${ROBOT_NAME:-${SERVICE_PREFIX}}"
ROBOT_HOST="${ROBOT_HOST:-192.168.1.100}"
RECIPE_DISPENSER_IDS="${RECIPE_DISPENSER_IDS:-${DISPENSER_IDS:-1x1}}"
CHECK_TIMEOUT_SEC="${CHECK_TIMEOUT_SEC:-3}"
ROBOT_PORT="${ROBOT_PORT:-12345}"
TCP_CHECK_SEC="${TCP_CHECK_SEC:-2}"
TCP_HARD_BLOCK="${TCP_HARD_BLOCK:-0}"
STRICT_REAL="${STRICT_REAL:-0}"

source_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  source /home/ssu/ws_moveit/install/setup.bash 2>/dev/null || true
  source /home/ssu/ros2_ws/install/setup.bash 2>/dev/null || true
  if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
    source "${ROOT_DIR}/install/setup.bash" 2>/dev/null || true
  elif [[ -f "${ROOT_DIR}/install/local_setup.bash" ]]; then
    source "${ROOT_DIR}/install/local_setup.bash" 2>/dev/null || true
  fi
  set -u
}

has_service() {
  ros2 service list 2>/dev/null | grep -qx "$1"
}

show_service() {
  local service="$1" label="$2"
  if has_service "${service}"; then
    echo "[OK] ${label}: ${service}"
    return 0
  fi
  echo "[MISSING] ${label}: ${service}"
  return 1
}

tcp_check_robot_host() {
  if [[ "${ROBOT_HOST}" == "127.0.0.1" || "${ROBOT_HOST}" == "localhost" ]]; then
    echo "[BLOCKED] ROBOT_HOST=${ROBOT_HOST} is localhost; real one-click requires the real controller IP."
    return 2
  fi
  if command -v nc >/dev/null 2>&1; then
    if timeout "${TCP_CHECK_SEC}s" nc -z "${ROBOT_HOST}" "${ROBOT_PORT}" >/dev/null 2>&1; then
      echo "[OK] Doosan TCP reachable: ${ROBOT_HOST}:${ROBOT_PORT}"
      return 0
    fi
    echo "[WARN] Doosan TCP not reachable now: ${ROBOT_HOST}:${ROBOT_PORT}"
    echo "[WARN] If real Doosan services are absent, one-click bringup will likely fail until network/controller is ready."
    return 1
  fi
  echo "[INFO] nc not installed; skipping Doosan TCP reachability check."
  return 0
}

virtual_matches() {
  pgrep -af 'dsr_bringup2_moveit|run_emulator|DRCF|ros2_control_node' \
    | grep -v "$$" \
    | grep -v 'check_one_click_cocktail_ready.sh' \
    | grep -v 'pgrep -af' \
    | grep -v 'grep -E' \
    | grep -E 'mode:=virtual|run_emulator|DRCF' || true
}

real_matches() {
  pgrep -af 'dsr_bringup2_moveit' \
    | grep -v "$$" \
    | grep -v 'check_one_click_cocktail_ready.sh' \
    | grep -v 'pgrep -af' \
    | grep -v 'grep -E' \
    | grep -E 'mode:=real' || true
}

source_ros

echo "[Azas] One-click cocktail readiness"
echo "[Azas] expected robot_name=${ROBOT_NAME} service_prefix=/${SERVICE_PREFIX} robot_host=${ROBOT_HOST}"
echo "[Azas] recipe_dispenser_ids=${RECIPE_DISPENSER_IDS}"

rc=0
if RECIPE_DISPENSER_IDS="${RECIPE_DISPENSER_IDS}" "${ROOT_DIR}/tools/run/check_one_click_cocktail_config.sh"; then
  :
else
  rc=1
fi
vm="$(virtual_matches)"
rm="$(real_matches)"
if [[ -n "${vm}" ]]; then
  echo "[BLOCKED] Virtual/emulator Doosan session is active. Stop preview before real motion:"
  echo "  bash tools/run/stop_cocktail_motion_preview.sh"
  echo "--- virtual matches ---"
  echo "${vm}"
  rc=2
elif [[ -n "${rm}" ]]; then
  echo "[OK] Real Doosan launch process detected."
  echo "${rm}"
elif [[ "${STRICT_REAL}" == "1" || "${STRICT_REAL}" == "true" ]]; then
  echo "[MISSING] No real Doosan launch process detected. one-click script can start it, but STRICT_REAL requested an existing real session."
  rc=1
else
  echo "[INFO] No existing real Doosan launch process detected. one-click script will start it if services are absent."
fi

if ! has_service "/${SERVICE_PREFIX}/motion/move_joint"; then
  tcp_rc=0
  tcp_check_robot_host || tcp_rc=$?
  if [[ "${tcp_rc}" -eq 2 ]]; then
    rc=2
  elif [[ "${tcp_rc}" -ne 0 && "${rc}" -ne 2 ]]; then
    if [[ "${TCP_HARD_BLOCK}" == "1" || "${TCP_HARD_BLOCK}" == "true" ]]; then
      echo "[BLOCKED] Doosan TCP is required for real one-click startup but is not reachable."
      rc=2
    else
      rc=1
    fi
  fi
fi

show_service "/${SERVICE_PREFIX}/motion/move_joint" "Doosan move_joint" || { [[ "${rc}" -eq 2 ]] || rc=1; }
show_service "/${SERVICE_PREFIX}/motion/move_line" "Doosan move_line" || { [[ "${rc}" -eq 2 ]] || rc=1; }
show_service "/${SERVICE_PREFIX}/motion/move_wait" "Doosan move_wait" || { [[ "${rc}" -eq 2 ]] || rc=1; }
show_service "/${SERVICE_PREFIX}/motion/fkin" "Doosan fkin" || { [[ "${rc}" -eq 2 ]] || rc=1; }
show_service "/${SERVICE_PREFIX}/motion/ikin" "Doosan ikin" || { [[ "${rc}" -eq 2 ]] || rc=1; }
show_service "/${SERVICE_PREFIX}/motion/check_motion" "Doosan check_motion" || { [[ "${rc}" -eq 2 ]] || rc=1; }
show_service "/${SERVICE_PREFIX}/system/get_robot_state" "Doosan get_robot_state" || { [[ "${rc}" -eq 2 ]] || rc=1; }
show_service "/${SERVICE_PREFIX}/aux_control/get_current_posj" "Doosan get_current_posj" || { [[ "${rc}" -eq 2 ]] || rc=1; }
show_service "/${SERVICE_PREFIX}/aux_control/get_current_posx" "Doosan get_current_posx" || { [[ "${rc}" -eq 2 ]] || rc=1; }
show_service "/jarvis/rg2/set_width" "RG2 set_width" || { [[ "${rc}" -eq 2 ]] || rc=1; }
show_service "/jarvis/rg2/open" "RG2 open" || { [[ "${rc}" -eq 2 ]] || rc=1; }
show_service "/jarvis/rg2/close" "RG2 close" || { [[ "${rc}" -eq 2 ]] || rc=1; }

if has_service "/${SERVICE_PREFIX}/aux_control/get_current_posj"; then
  echo "[Azas] Sampling current joints..."
  if timeout "${CHECK_TIMEOUT_SEC}s" ros2 service call "/${SERVICE_PREFIX}/aux_control/get_current_posj" dsr_msgs2/srv/GetCurrentPosj "{}" 2>&1 | sed -n '1,12p'; then
    :
  else
    echo "[WARN] get_current_posj sample failed or timed out."
    rc=1
  fi
fi

if [[ "${rc}" -eq 0 ]]; then
  echo "[PASS] one-click cocktail stack is ready to run now."
elif [[ "${rc}" -eq 2 ]]; then
  echo "[FAIL] hard real-motion block is active; see [BLOCKED] lines above."
else
  echo "[WARN] not fully ready yet; run_one_click_cocktail_real.sh can start missing robot/gripper nodes when confirmed."
fi
exit "${rc}"
