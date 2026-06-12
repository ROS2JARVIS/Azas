#!/usr/bin/env bash
set -euo pipefail

# Stop the entire Azas field stack in one shot: the azas tmux logic sessions,
# every ROS-related process (robot bringup, gripper, camera, relays, MoveIt,
# RViz, perception/preview nodes, stray ros2 CLI zombies), the ros2 daemon,
# and stale FastDDS shared-memory segments left behind by killed nodes.
#
# Protected and never killed: the control panel (unless KILL_PANEL=1), the
# tmux server itself, and Codex/OMX/Claude agent processes plus this script's
# own ancestry.
#
# Usage:
#   bash tools/run/stop_azas_all.sh              # stop everything + shm clean
#   DRY_RUN=1 bash tools/run/stop_azas_all.sh    # only print what would die
#   KILL_PANEL=1 bash tools/run/stop_azas_all.sh # also stop the control panel
#   CLEAN_FASTDDS_SHM=0 ...                      # skip /dev/shm cleanup

DRY_RUN="${DRY_RUN:-0}"
KILL_PANEL="${KILL_PANEL:-0}"
CLEAN_FASTDDS_SHM="${CLEAN_FASTDDS_SHM:-1}"
SESSIONS="${SESSIONS:-azas-logic azas-rviz-exact}"
GRACE_SEC="${GRACE_SEC:-6}"

ROS_PATTERN='run_doosan_real_m0609\.sh|dsr_bringup2|run_emulator|/DRCF|ros2_control_node|robot_state_publisher|move_group|rviz2|rg2_trigger|rg2_gripper_node|rs_launch\.py|realsense2_camera_node|joint_state_relay\.py|yolo_cup_pick_node|hand_eye_static_tf_node|static_transform_publisher|link6_gripper_collision_node|measured_dispenser_collision_scene_node|collision_scene_rviz_publisher\.py|publish_color_recipe_sequence_rviz_preview\.py|publish_collision_scene_rviz\.py|lid_sticker_detector_node|lid_grip_planner_node|lid_detection_pose_bridge_node|dispenser_sequence|run_changhyun_side_grip_direct\.sh|run_kang_lid_grip_close_direct\.sh|run_somyeong_cup_uprighting_direct\.sh|run_tmux_logic_sequence\.sh|run_color_recipe_sequence\.py|run_measured_dispenser_recipe_sequence\.py|run_minimal_dispenser_cycle\.py|/opt/ros/humble/bin/ros2 |ros2cli\.daemon'

PROTECT_PATTERN='codex|oh-my-codex|omx|claude|bwrap|stop_azas_all\.sh|(^|[ /])tmux( |$|:)'
if [[ "${KILL_PANEL}" != "1" && "${KILL_PANEL}" != "true" ]]; then
  PROTECT_PATTERN="${PROTECT_PATTERN}|robot_pipeline_control_server\.py|run_robot_pipeline_control_panel\.sh"
fi

self_and_ancestors() {
  local pid=$$
  while [[ -n "${pid}" && "${pid}" -gt 1 ]]; do
    echo "${pid}"
    pid="$(ps -o ppid= -p "${pid}" 2>/dev/null | tr -d ' ')" || break
  done
}
PROTECTED_PIDS=" $(self_and_ancestors | tr '\n' ' ') "

collect_pids() {
  ps -eo pid=,stat=,args= | grep -E "${ROS_PATTERN}" | grep -Ev "${PROTECT_PATTERN}" \
    | while read -r pid stat args; do
        # Defunct children cannot be killed; counting them as live ROS processes
        # prevents FastDDS SHM cleanup and makes reconnect look stuck.
        [[ "${stat}" == Z* ]] && continue
        [[ "${PROTECTED_PIDS}" == *" ${pid} "* ]] && continue
        echo "${pid}"
      done
}

signal_pids() {
  local sig="$1"
  shift
  local pid
  for pid in "$@"; do
    if [[ "${DRY_RUN}" == "1" || "${DRY_RUN}" == "true" ]]; then
      echo "[DRY_RUN] kill -${sig} ${pid} :: $(ps -p "${pid}" -o args= 2>/dev/null | cut -c1-140)"
    else
      kill "-${sig}" "${pid}" 2>/dev/null || true
    fi
  done
}

# 1) Gracefully stop the azas tmux logic sessions (C-c, brief wait, kill).
for session in ${SESSIONS}; do
  if tmux has-session -t "${session}" >/dev/null 2>&1; then
    echo "[Azas] stopping tmux session: ${session}"
    if [[ "${DRY_RUN}" == "1" || "${DRY_RUN}" == "true" ]]; then
      tmux list-windows -t "${session}" -F "[DRY_RUN] would kill window ${session}:#{window_name}"
      continue
    fi
    tmux list-panes -s -t "${session}" -F '#{pane_id}' | while read -r pane; do
      [[ -n "${pane}" ]] && tmux send-keys -t "${pane}" C-c >/dev/null 2>&1 || true
    done
    sleep 2
    tmux kill-session -t "${session}" >/dev/null 2>&1 || true
  fi
done

# 2) TERM every matched ROS process, wait, then KILL survivors.
mapfile -t targets < <(collect_pids)
if [[ "${#targets[@]}" -gt 0 ]]; then
  echo "[Azas] stopping ${#targets[@]} ROS-related processes"
  signal_pids TERM "${targets[@]}"
  if [[ "${DRY_RUN}" != "1" && "${DRY_RUN}" != "true" ]]; then
    deadline=$((SECONDS + GRACE_SEC))
    while [[ ${SECONDS} -lt ${deadline} ]]; do
      mapfile -t remaining < <(collect_pids)
      [[ "${#remaining[@]}" -eq 0 ]] && break
      sleep 0.5
    done
    mapfile -t remaining < <(collect_pids)
    if [[ "${#remaining[@]}" -gt 0 ]]; then
      echo "[Azas] force-killing ${#remaining[@]} survivors"
      signal_pids KILL "${remaining[@]}"
      sleep 0.5
    fi
  fi
else
  echo "[Azas] no ROS-related processes found"
fi

# 3) Clean stale FastDDS shared memory once nothing ROS-related is left.
if [[ "${CLEAN_FASTDDS_SHM}" == "1" || "${CLEAN_FASTDDS_SHM}" == "true" ]]; then
  mapfile -t remaining < <(collect_pids)
  if [[ "${#remaining[@]}" -gt 0 ]]; then
    echo "[Azas] SKIP FastDDS SHM cleanup: ${#remaining[@]} ROS processes still alive" >&2
  elif [[ "${DRY_RUN}" == "1" || "${DRY_RUN}" == "true" ]]; then
    echo "[DRY_RUN] would remove $(ls /dev/shm 2>/dev/null | grep -c '^fastrtps_' || true) /dev/shm/fastrtps_* segments"
  else
    count="$(ls /dev/shm 2>/dev/null | grep -c '^fastrtps_' || true)"
    rm -f /dev/shm/fastrtps_* >/dev/null 2>&1 || true
    echo "[Azas] removed ${count} stale /dev/shm/fastrtps_* segments"
  fi
fi

echo "[Azas] stop_azas_all done. Restart with: bash tools/run/start_azas_tmux_stack.sh"
