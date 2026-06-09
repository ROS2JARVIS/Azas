#!/usr/bin/env bash
set -euo pipefail

# RViz-only collision-aware preview for the measured dispenser course cycle.
#
# This is intentionally a simulation/preview entrypoint:
# - forces virtual Doosan bringup through run_course_dispenser_press_cycle_rviz.sh
# - publishes measured dispenser collision objects into the MoveIt PlanningScene
# - runs the full cup-place -> press -> re-grasp course cycle, not PRESS_ONLY
#
# Input:
#   RECIPE_DISPENSER_IDS=1x1,3x2,4x1  # preferred
#   or DISPENSER_ID=1 PRESS_COUNT=2    # single fallback

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
COURSE_SCRIPT="${ROOT_DIR}/tools/run/run_course_dispenser_press_cycle_rviz.sh"

parse_sequence() {
  local raw="$1"
  local normalized part item did count
  normalized="${raw//;/,}"
  IFS=',' read -r -a parts <<<"${normalized}" || true
  for part in "${parts[@]}"; do
    item="$(echo "${part}" | tr '[:upper:]' '[:lower:]' | xargs)"
    [[ -z "${item}" ]] && continue
    if [[ "${item}" == *x* ]]; then
      did="${item%%x*}"
      count="${item#*x}"
    elif [[ "${item}" == *:* ]]; then
      did="${item%%:*}"
      count="${item#*:}"
    else
      did="${item}"
      count="1"
    fi
    did="$(echo "${did}" | xargs)"
    count="$(echo "${count}" | xargs)"
    if [[ ! "${did}" =~ ^[1-4]$ || ! "${count}" =~ ^[0-9]+$ || "${count}" -lt 1 ]]; then
      echo "invalid dispenser sequence item: ${item}" >&2
      return 2
    fi
    printf '%s %s\n' "${did}" "${count}"
  done
}

RAW_SEQUENCE="${RECIPE_DISPENSER_IDS:-}"
if [[ -z "${RAW_SEQUENCE}" ]]; then
  RAW_SEQUENCE="${DISPENSER_ID:-1}x${PRESS_COUNT:-1}"
fi

mapfile -t SEQUENCE_GROUPS < <(parse_sequence "${RAW_SEQUENCE}") || true
if [[ "${#SEQUENCE_GROUPS[@]}" -lt 1 ]]; then
  echo "[Azas] No dispenser sequence to preview. Set RECIPE_DISPENSER_IDS=1x1,3x2." >&2
  exit 2
fi

echo "[Azas] RViz cocktail collision preview sequence: ${RAW_SEQUENCE}"
echo "[Azas] Full cycle mode: cup-place -> press -> re-grasp, collision objects enabled."

first=1
last_index=$(("${#SEQUENCE_GROUPS[@]}" - 1))
for index in "${!SEQUENCE_GROUPS[@]}"; do
  group="${SEQUENCE_GROUPS[$index]}"
  read -r did count <<<"${group}"
  if [[ "${first}" == "1" ]]; then
    start_doosan="${START_DOOSAN:-auto}"
    first=0
  else
    # Each course-script invocation owns and cleans up its bringup unless it is
    # kept alive at the end, so later groups must be allowed to auto-start or
    # reuse the virtual session instead of assuming the first one still exists.
    start_doosan="${START_DOOSAN:-auto}"
  fi
  if [[ "${index}" -eq "${last_index}" ]]; then
    keep_after="${KEEP_ALIVE_AFTER_DONE:-0}"
    preserve_after=0
  else
    # Do not block between groups; keep RViz/virtual bringup alive only after
    # the final group so a full recipe such as 1x1,3x2,4x1 can actually play.
    keep_after=0
    preserve_after=1
  fi
  if [[ "${index}" -eq 0 ]]; then
    reset_existing="${RESET_EXISTING_VIRTUAL_PREVIEW:-1}"
    replace_rviz="${REPLACE_EXISTING_RVIZ:-1}"
  else
    # Reuse the virtual Doosan/RViz session preserved by the previous group.
    # Resetting/replacing here makes the orange robot disappear between steps.
    reset_existing=0
    replace_rviz=0
  fi
  echo "[Azas] Preview dispenser ${did} x${count}"
  RVIZ_ONLY=1 \
  PRESS_ONLY=0 \
  DISPENSER_COLLISION_ENABLED=1 \
  DISPENSER_COLLISION_OBJECTS="${DISPENSER_COLLISION_OBJECTS:-1}" \
  REMOVE_COURSE_WORKSPACE_WALLS="${REMOVE_COURSE_WORKSPACE_WALLS:-0}" \
  DISPENSER_ID="${did}" \
  PRESS_COUNT="${count}" \
  START_DOOSAN="${start_doosan}" \
  KEEP_ALIVE_AFTER_DONE="${keep_after}" \
  PRESERVE_PREVIEW_SESSION_AFTER_DONE="${preserve_after}" \
  KEEP_RVIZ_ON_FAIL="${KEEP_RVIZ_ON_FAIL:-1}" \
  RVIZ_MODE="${RVIZ_MODE:-clean}" \
  REPLACE_EXISTING_RVIZ="${replace_rviz}" \
  RESET_EXISTING_VIRTUAL_PREVIEW="${reset_existing}" \
  bash "${COURSE_SCRIPT}"
done

echo "[Azas] RViz cocktail collision preview completed."
