#!/usr/bin/env bash
set -euo pipefail

# Short final entrypoint for the real integrated cocktail dispenser cycle.
# It intentionally delegates to the guarded one-click script instead of
# duplicating motion logic.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RECIPE_DISPENSER_IDS="${RECIPE_DISPENSER_IDS:-${1:-1x1}}"
ROBOT_HOST="${ROBOT_HOST:-192.168.1.100}"
SERVICE_PREFIX="${SERVICE_PREFIX:-dsr01}"
ROBOT_NAME="${ROBOT_NAME:-${SERVICE_PREFIX}}"
SKIP_PREVIEW_STOP="${SKIP_PREVIEW_STOP:-0}"
DRY_RUN="${DRY_RUN:-0}"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<USAGE
Usage:
  REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE \\
  bash tools/run/run_cocktail_now_real.sh 1x2

This runs the real integrated cycle:
  preview cleanup -> config/readiness guard -> real Doosan/RG2/collision setup ->
  cup-place -> RG2 full-open -> safe lift -> close empty gripper -> measured press pump(s) -> re-grasp/lift -> result check.

Env:
  RECIPE_DISPENSER_IDS=1x2    same as first positional argument
  ROBOT_HOST=192.168.1.100
  ROBOT_NAME=dsr01            defaults to SERVICE_PREFIX
  SERVICE_PREFIX=dsr01
  SKIP_PREVIEW_STOP=1         do not run preview cleanup first
  DRY_RUN=1                   print real one-click commands without motion
USAGE
  exit 0
fi

if [[ "${REAL_COCKTAIL_CONFIRM:-}" != "ENABLE_REAL_COCKTAIL_SEQUENCE" ]]; then
  echo "[Azas] Refusing real cocktail-now run without explicit confirmation." >&2
  echo "[Azas] Re-run with: REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE" >&2
  exit 2
fi

cd "${ROOT_DIR}"

echo "[Azas] Cocktail NOW real cycle: ${RECIPE_DISPENSER_IDS}"
echo "[Azas] robot_host=${ROBOT_HOST} robot_name=${ROBOT_NAME} service_prefix=${SERVICE_PREFIX}"

if [[ "${SKIP_PREVIEW_STOP}" != "1" && "${SKIP_PREVIEW_STOP}" != "true" ]]; then
  bash tools/run/stop_cocktail_motion_preview.sh
fi

set +e
if [[ "${DRY_RUN}" == "1" || "${DRY_RUN}" == "true" ]]; then
  TCP_HARD_BLOCK_FOR_READY="${TCP_HARD_BLOCK:-0}"
else
  TCP_HARD_BLOCK_FOR_READY="${TCP_HARD_BLOCK:-1}"
fi
RECIPE_DISPENSER_IDS="${RECIPE_DISPENSER_IDS}" \
ROBOT_HOST="${ROBOT_HOST}" \
ROBOT_NAME="${ROBOT_NAME}" \
SERVICE_PREFIX="${SERVICE_PREFIX}" \
TCP_HARD_BLOCK="${TCP_HARD_BLOCK_FOR_READY}" \
bash tools/run/check_one_click_cocktail_ready.sh
READY_RC=$?
set -e

if [[ "${READY_RC}" -eq 2 ]]; then
  echo "[Azas] Refusing to continue: readiness reported a hard real-motion block." >&2
  exit 2
fi
if [[ "${READY_RC}" -ne 0 ]]; then
  echo "[Azas] Readiness is not fully green yet (rc=${READY_RC}); continuing because one-click can start missing real Doosan/RG2 nodes after its own guards."
fi

REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE \
RECIPE_DISPENSER_IDS="${RECIPE_DISPENSER_IDS}" \
ROBOT_HOST="${ROBOT_HOST}" \
ROBOT_NAME="${ROBOT_NAME}" \
SERVICE_PREFIX="${SERVICE_PREFIX}" \
DRY_RUN="${DRY_RUN}" \
bash tools/run/run_one_click_cocktail_real.sh
