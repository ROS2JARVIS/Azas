#!/usr/bin/env bash
set -euo pipefail

# Stop only the RViz/virtual cocktail preview stack. This is intended before
# real robot execution so /dsr01 services are not accidentally backed by the
# virtual Doosan emulator.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DRY_RUN="${DRY_RUN:-0}"
KILL_RVIZ="${KILL_RVIZ:-1}"

kill_tree() {
  local pid="$1"
  local child
  for child in $(pgrep -P "${pid}" 2>/dev/null || true); do
    kill_tree "${child}"
  done
  if kill -0 "${pid}" 2>/dev/null; then
    if [[ "${DRY_RUN}" == "1" || "${DRY_RUN}" == "true" ]]; then
      echo "[DRY_RUN] kill ${pid} $(ps -p "${pid}" -o comm= 2>/dev/null || true)"
    else
      kill "${pid}" 2>/dev/null || true
    fi
  fi
}

collect_roots() {
  {
    # Use ps instead of pgrep -f for launch processes because long ROS launch
    # argv lines can be truncated differently by pgrep on some systems.
    ps -eo pid=,args= \
      | grep -E 'run_cocktail_collision_rviz_preview.sh|run_course_dispenser_press_cycle_rviz.sh' \
      | grep -v "$$" \
      | grep -v 'stop_cocktail_motion_preview.sh' \
      | grep -v 'grep -E' \
      | awk '{print $1}' || true
    ps -eo pid=,args= \
      | grep 'dsr_bringup2_moveit.launch.py' \
      | grep 'mode:=virtual' \
      | grep -v "$$" \
      | grep -v 'stop_cocktail_motion_preview.sh' \
      | grep -v 'grep ' \
      | awk '{print $1}' || true
    ps -eo pid=,args= \
      | grep -E 'run_emulator|./DRCF M0609|/DRCF M0609' \
      | grep -v "$$" \
      | grep -v 'stop_cocktail_motion_preview.sh' \
      | grep -v 'grep -E' \
      | awk '{print $1}' || true
    if [[ "${KILL_RVIZ}" == "1" || "${KILL_RVIZ}" == "true" ]]; then
      ps -eo pid=,args= \
        | grep 'rviz2' \
        | grep -E 'azas_cocktail_collision_preview|dsr_moveit_config_m0609.*/moveit.rviz' \
        | grep -v "$$" \
        | grep -v 'stop_cocktail_motion_preview.sh' \
        | grep -v 'grep ' \
        | awk '{print $1}' || true
    fi
  } | sort -n | uniq | grep -v "^$$$" || true
}

echo "[Azas] Stopping virtual/RViz cocktail preview stack. Real robot processes are not targeted."
mapfile -t roots < <(collect_roots)
if [[ "${#roots[@]}" -eq 0 ]]; then
  echo "[Azas] No cocktail preview processes found."
  exit 0
fi

for pid in "${roots[@]}"; do
  if kill -0 "${pid}" 2>/dev/null; then
    echo "[Azas] stopping preview pid=${pid} cmd=$(ps -p "${pid}" -o args= 2>/dev/null || true)"
    kill_tree "${pid}"
  fi
done

if [[ "${DRY_RUN}" != "1" && "${DRY_RUN}" != "true" ]]; then
  sleep 2
  # Escalate only matching preview/emulator remnants, not arbitrary real bringup.
  for pid in $(collect_roots); do
    if kill -0 "${pid}" 2>/dev/null; then
      echo "[Azas] force stopping lingering preview pid=${pid}"
      kill -9 "${pid}" 2>/dev/null || true
    fi
  done
  sleep 1
  lingering="$(collect_roots)"
  if [[ -n "${lingering}" ]]; then
    echo "[Azas] Warning: preview processes still visible after stop:" >&2
    for pid in ${lingering}; do
      echo "  ${pid} $(ps -p "${pid}" -o args= 2>/dev/null || true)" >&2
    done
    exit 1
  fi
fi

echo "[Azas] Preview stop complete."
