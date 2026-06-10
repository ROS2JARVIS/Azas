#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/log/manual}"
INTEGRATED_LOG="${INTEGRATED_LOG:-${LOG_DIR}/one_click_real_integrated_recipe.log}"
SERVICE_PREFIX="${SERVICE_PREFIX:-dsr01}"
SAMPLE_CURRENT_POSE="${SAMPLE_CURRENT_POSE:-1}"

source_ros() {
  set +u
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  source /home/ssu/ws_moveit/install/setup.bash 2>/dev/null || true
  source /home/ssu/ros2_ws/install/setup.bash 2>/dev/null || true
  if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
    source "${ROOT_DIR}/install/setup.bash" 2>/dev/null || true
  elif [[ -f "${ROOT_DIR}/install/local_setup.bash" ]]; then
    source "${ROOT_DIR}/install/local_setup.bash" 2>/dev/null || true
  fi
  set -u
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<USAGE
Usage:
  bash tools/run/check_one_click_cocktail_result.sh

Env:
  INTEGRATED_LOG=log/manual/one_click_real_integrated_recipe.log
  SERVICE_PREFIX=dsr01
  SAMPLE_CURRENT_POSE=1   # sample current posj/posx if services are available
USAGE
  exit 0
fi

echo "[Azas] Checking one-click cocktail result"
echo "[Azas] integrated_log=${INTEGRATED_LOG}"

if [[ ! -f "${INTEGRATED_LOG}" ]]; then
  echo "[FAIL] integrated log not found. Run real one-click first or set INTEGRATED_LOG." >&2
  exit 1
fi

rc=0
if grep -q '\[PASS\] measured dispenser recipe sequence completed' "${INTEGRATED_LOG}"; then
  echo "[PASS] integrated measured dispenser recipe sequence completed"
else
  echo "[FAIL] PASS marker missing from integrated log" >&2
  rc=1
fi

for needle in \
  'RG2 full-open release complete; continuing only after open settle wait' \
  'RG2 close empty gripper for dispenser press' \
  'move to measured press contact joints exactly' \
  'press dispenser pump' \
  'RG2 soft side-grasp' \
  'post-grasp lift'; do
  if grep -q "${needle}" "${INTEGRATED_LOG}"; then
    echo "[OK] found: ${needle}"
  else
    echo "[WARN] not found: ${needle}"
    rc=1
  fi
done

if grep -q '\[FAIL\]\|\[BLOCKED\]\|target verification timeout\|joint target verification timeout\|response timeout' "${INTEGRATED_LOG}"; then
  echo "[FAIL] failure/blocking marker detected in integrated log" >&2
  grep -n '\[FAIL\]\|\[BLOCKED\]\|target verification timeout\|joint target verification timeout\|response timeout' "${INTEGRATED_LOG}" | tail -20 >&2 || true
  rc=1
fi

if [[ "${SAMPLE_CURRENT_POSE}" == "1" || "${SAMPLE_CURRENT_POSE}" == "true" ]]; then
  source_ros
  if ros2 service list 2>/dev/null | grep -qx "/${SERVICE_PREFIX}/aux_control/get_current_posj"; then
    echo "--- current_posj sample ---"
    python3 "${ROOT_DIR}/tools/run/ros_call_empty_service.py" "/${SERVICE_PREFIX}/aux_control/get_current_posj" dsr_msgs2/srv/GetCurrentPosj --timeout 5.0 || true
  fi
  if ros2 service list 2>/dev/null | grep -qx "/${SERVICE_PREFIX}/aux_control/get_current_posx"; then
    echo "--- current_posx sample ---"
    python3 "${ROOT_DIR}/tools/run/ros_call_empty_service.py" "/${SERVICE_PREFIX}/aux_control/get_current_posx" dsr_msgs2/srv/GetCurrentPosx --timeout 5.0 || true
  fi
fi

if [[ "${rc}" -eq 0 ]]; then
  echo "[PASS] one-click cocktail result log satisfies the expected cup-place -> press -> re-grasp evidence."
else
  echo "[WARN] one-click cocktail result is not fully proven by the log. See tail below."
  echo "--- integrated tail ---"
  tail -80 "${INTEGRATED_LOG}" || true
fi
exit "${rc}"
