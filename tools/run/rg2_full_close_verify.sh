#!/usr/bin/env bash
set -euo pipefail

# Command RG2 to a full-close target and verify the ROS service accepted it.
#
# Safety/validation note:
# - Use this for dispenser pressing after the cup has already been released.
# - Do NOT use this as the large-cup grasp command; use gripper_soft_grasp instead.
# - jarvis/rg2_trigger_node does not expose actual finger-position feedback.
# - This verifies the strongest available software evidence: full-close set_width
#   command was sent and the service returned success=True.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SERVICE="${RG2_SET_WIDTH_SERVICE:-/jarvis/rg2/set_width}"
WIDTH_M="${RG2_FULL_CLOSE_WIDTH_M:-0.000}"
FORCE_N="${RG2_CLOSE_FORCE_N:-30.0}"
TIMEOUT_SEC="${RG2_CLOSE_TIMEOUT_SEC:-12}"

cd "${ROOT_DIR}"

echo "[Azas] RG2 full-close request"
echo "[Azas] service=${SERVICE}"
echo "[Azas] command=close width_m=${WIDTH_M} force_n=${FORCE_N}"

if ! output="$(
  python3 tools/run/rg2_set_width_verify.py \
    --service "${SERVICE}" \
    --command close \
    --width-m "${WIDTH_M}" \
    --force-n "${FORCE_N}" \
    --timeout-sec "${TIMEOUT_SEC}" \
    --ready-timeout-sec "${RG2_READY_TIMEOUT_SEC:-18}" \
    --rg2-ip "${RG2_IP:-192.168.1.1}" 2>&1
)"; then
  echo "${output}"
  echo "[FAIL] RG2 full-close service call failed"
  exit 1
fi

echo "${output}"

RG2_CLOSE_RESPONSE="${output}" python3 - "${WIDTH_M}" <<'PY'
import os
import re
import sys

width_m = float(sys.argv[1])
text = os.environ.get("RG2_CLOSE_RESPONSE", "")
if not re.search(r"success\s*[:=]\s*True|success\s*[:=]\s*true", text):
    print("[FAIL] RG2 full-close response did not contain success=True")
    raise SystemExit(1)

print(
    "[PASS] RG2 full-close command accepted "
    f"(target_width_m={width_m:.3f}; actual finger feedback is not exposed)"
)
PY
