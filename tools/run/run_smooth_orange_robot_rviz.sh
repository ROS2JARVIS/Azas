#!/usr/bin/env bash
set -euo pipefail

# RViz robot-motion preview only: no MoveIt path display, no controller, no fake high-frequency shake.
# Shows the orange M0609 model itself moving smoothly from /joint_states.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/log/manual}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${ROOT_DIR}/src/azas_bringup/rviz/azas_dispenser_sequence_clean.rviz}"
PUBLISH_RATE="${PUBLISH_RATE:-60.0}"
SHAKE_CYCLES_PER_SECOND="${SHAKE_CYCLES_PER_SECOND:-0.55}"
PREVIEW_MODE="${PREVIEW_MODE:-side_grasp_move_then_shake}"
LOOP_MOTION="${LOOP_MOTION:-true}"
ROBOT_COLOR="${ROBOT_COLOR:-orange}"
mkdir -p "${LOG_DIR}"

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT
PIDS=()

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

# Kill only old RViz-only demo joint publishers so there is exactly one /joint_states source.
pkill -f 'm0609_shake_joint_state_node' 2>/dev/null || true
pkill -f 'side_grasp_ik_preview_node' 2>/dev/null || true

ros2 launch azas_bringup hardware_free_demo.launch.py \
  use_rviz:=false \
  use_robot_urdf:=true \
  robot_color:="${ROBOT_COLOR}" \
  enable_ik_preview:=false \
  run_live_stt:=false \
  run_recipe_mapper:=false \
  use_llm:=false \
  show_sequence_markers:=false \
  show_dispenser_markers:=false \
  show_animated_cup:=false \
  show_demo_arm:=false \
  >"${LOG_DIR}/smooth_robot_description.log" 2>&1 &
PIDS+=("$!")

ros2 run azas_motion m0609_shake_joint_state_node \
  --ros-args \
  -p publish_rate:="${PUBLISH_RATE}" \
  -p shake_cycles_per_second:="${SHAKE_CYCLES_PER_SECOND}" \
  -p preview_mode:="${PREVIEW_MODE}" \
  -p loop_motion:="${LOOP_MOTION}" \
  >"${LOG_DIR}/smooth_robot_joint_states.log" 2>&1 &
PIDS+=("$!")

rviz2 -d "${RVIZ_CONFIG}" >"${LOG_DIR}/smooth_robot_rviz.log" 2>&1 &
PIDS+=("$!")

echo "[Azas] Smooth orange robot motion is running in RViz."
echo "[Azas] Visual source: robot_state_publisher + one /joint_states publisher. No path display, no controller simulation."
echo "[Azas] Logs: ${LOG_DIR}/smooth_robot_*.log"
wait
