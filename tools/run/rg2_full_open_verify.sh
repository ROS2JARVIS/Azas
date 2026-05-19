#!/usr/bin/env bash
set -euo pipefail

# Command RG2 to a full-open target and verify the ROS service accepted it.
#
# Safety/validation note:
# - jarvis/rg2_trigger_node does not expose actual finger-position feedback.
# - This script verifies the strongest available software evidence:
#   the full-open set_width command was sent and the service returned success=True.
# - Physical confirmation still requires watching the gripper or adding a feedback source.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SERVICE="${RG2_SET_WIDTH_SERVICE:-/jarvis/rg2/set_width}"
WIDTH_M="${RG2_FULL_OPEN_WIDTH_M:-0.110}"
FORCE_N="${RG2_OPEN_FORCE_N:-12.0}"
TIMEOUT_SEC="${RG2_OPEN_TIMEOUT_SEC:-12}"

cd "${ROOT_DIR}"

echo "[Azas] RG2 full-open request"
echo "[Azas] service=${SERVICE}"
echo "[Azas] command=open width_m=${WIDTH_M} force_n=${FORCE_N}"

if ! output="$(
  timeout "${TIMEOUT_SEC}s" \
    ros2 service call "${SERVICE}" azas_interfaces/srv/SetGripper \
      "{command: 'open', width_m: ${WIDTH_M}, force_n: ${FORCE_N}}" 2>&1
)"; then
  echo "${output}"
  echo "[FAIL] RG2 full-open service call failed"
  exit 1
fi

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
