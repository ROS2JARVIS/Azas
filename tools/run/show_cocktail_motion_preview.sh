#!/usr/bin/env bash
set -euo pipefail

# Short operator command for the full cocktail dispenser motion preview.
# This is RViz/virtual only: it never commands the real robot.
# Sequence shown: cup-place -> open gripper -> safe lift -> close empty gripper
# -> measured press pump(s) -> re-grasp/lift.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RECIPE_DISPENSER_IDS="${RECIPE_DISPENSER_IDS:-${1:-1x1}}"
DISPENSER_COLLISION_OBJECTS="${DISPENSER_COLLISION_OBJECTS:-1}"
KEEP_ALIVE_AFTER_DONE="${KEEP_ALIVE_AFTER_DONE:-1}"
RESET_EXISTING_VIRTUAL_PREVIEW="${RESET_EXISTING_VIRTUAL_PREVIEW:-1}"
REPLACE_EXISTING_RVIZ="${REPLACE_EXISTING_RVIZ:-1}"
# Default to the Doosan teaching-material MoveIt RViz ("orange robot") view.
# Operators can still request the lean debug RobotModel view with RVIZ_MODE=clean.
RVIZ_MODE="${RVIZ_MODE:-bringup}"

usage() {
  cat <<USAGE
Usage:
  bash tools/run/show_cocktail_motion_preview.sh [sequence]

Examples:
  bash tools/run/show_cocktail_motion_preview.sh 1x1
  bash tools/run/show_cocktail_motion_preview.sh 1x2
  bash tools/run/show_cocktail_motion_preview.sh 1x1,3x2,4x1

Env:
  DISPENSER_COLLISION_OBJECTS=1    include measured dispenser collision in MoveIt
  RVIZ_MODE=bringup                show course-material orange MoveIt robot view
  RVIZ_MODE=clean                  show lean RobotModel/marker debug view
  KEEP_ALIVE_AFTER_DONE=1          keep RViz open after the preview finishes
USAGE
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

echo "[Azas] Full cocktail motion RViz preview: ${RECIPE_DISPENSER_IDS}"
echo "[Azas] RViz/virtual only; real robot will not move."
echo "[Azas] Motion: cup-place -> RG2 open -> safe Z lift -> RG2 close -> press pump(s) -> re-grasp/lift."

cd "${ROOT_DIR}"
RECIPE_DISPENSER_IDS="${RECIPE_DISPENSER_IDS}" \
DISPENSER_COLLISION_OBJECTS="${DISPENSER_COLLISION_OBJECTS}" \
KEEP_ALIVE_AFTER_DONE="${KEEP_ALIVE_AFTER_DONE}" \
RESET_EXISTING_VIRTUAL_PREVIEW="${RESET_EXISTING_VIRTUAL_PREVIEW}" \
REPLACE_EXISTING_RVIZ="${REPLACE_EXISTING_RVIZ}" \
RVIZ_MODE="${RVIZ_MODE}" \
bash tools/run/run_cocktail_collision_rviz_preview.sh
