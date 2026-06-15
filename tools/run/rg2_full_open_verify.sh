#!/usr/bin/env bash
set -euo pipefail

# Command RG2 to a full-open target and verify the ROS service accepted it.
#
# Safety/validation note:
# - azas_gripper/rg2_gripper_node does not expose actual finger-position feedback.
# - This script verifies the strongest available software evidence:
#   the full-open set_width command was sent and the service returned success=True.
# - Physical confirmation still requires watching the gripper or adding a feedback source.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SERVICE="${RG2_SET_WIDTH_SERVICE:-/jarvis/rg2/set_width}"
WIDTH_M="${RG2_FULL_OPEN_WIDTH_M:-0.110}"
FORCE_N="${RG2_OPEN_FORCE_N:-25.0}"
TIMEOUT_SEC="${RG2_OPEN_TIMEOUT_SEC:-12}"
RETRIES="${RG2_OPEN_RETRIES:-3}"
RETRY_SLEEP_SEC="${RG2_OPEN_RETRY_SLEEP_SEC:-1.0}"

cd "${ROOT_DIR}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
source /home/ssu/Azas/install/setup.bash
set -u

echo "[Azas] RG2 full-open request"
echo "[Azas] service=${SERVICE}"
echo "[Azas] command=open width_m=${WIDTH_M} force_n=${FORCE_N}"

output=""
for attempt in $(seq 1 "${RETRIES}"); do
  echo "[Azas] RG2 full-open attempt ${attempt}/${RETRIES}"
  if output="$(
    python3 tools/run/rg2_set_width_verify.py \
      --service "${SERVICE}" \
      --command open \
      --width-m "${WIDTH_M}" \
      --force-n "${FORCE_N}" \
      --timeout-sec "${TIMEOUT_SEC}" \
      --ready-timeout-sec "${RG2_READY_TIMEOUT_SEC:-18}" \
      --rg2-ip "${RG2_IP:-192.168.1.1}" 2>&1
  )"; then
    break
  fi
  echo "${output}"
  if [[ "${attempt}" -lt "${RETRIES}" ]]; then
    echo "[WARN] RG2 full-open service call failed; retrying after ${RETRY_SLEEP_SEC}s"
    sleep "${RETRY_SLEEP_SEC}"
  else
    echo "[FAIL] RG2 full-open service call failed after ${RETRIES} attempts"
    exit 1
  fi
done

echo "${output}"

RG2_OPEN_RESPONSE="${output}" python3 - "${WIDTH_M}" <<'PY'
import os
import re
import sys

width_m = float(sys.argv[1])
text = os.environ.get("RG2_OPEN_RESPONSE", "")
if not re.search(r"success\s*[:=]\s*True|success\s*[:=]\s*true", text):
    print("[FAIL] RG2 full-open response did not contain success=True")
    raise SystemExit(1)

print(
    "[PASS] RG2 full-open command accepted "
    f"(target_width_m={width_m:.3f}; actual finger feedback is not exposed)"
)
PY
