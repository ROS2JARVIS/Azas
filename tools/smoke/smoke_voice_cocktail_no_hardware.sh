#!/usr/bin/env bash
set -euo pipefail

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
VOICE_LOG_FILE="${VOICE_LOG_FILE:-/tmp/azas_smoke_voice_no_hardware_voice.log}"
TASK_LOG_FILE="${TASK_LOG_FILE:-/tmp/azas_smoke_voice_no_hardware_task.log}"
mkdir -p "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
set -u

rm -f "${VOICE_LOG_FILE}" "${TASK_LOG_FILE}"

ros2 run azas_voice recipe_mapper_node >"${VOICE_LOG_FILE}" 2>&1 &
VOICE_PID=$!
ros2 run azas_task_manager cocktail_dryrun_sequence_node \
  --ros-args \
  -p require_cup:=false \
  -p require_lid:=false \
  >"${TASK_LOG_FILE}" 2>&1 &
TASK_PID=$!

cleanup() {
  kill -9 "${VOICE_PID}" "${TASK_PID}" 2>/dev/null || true
}
trap cleanup EXIT

python3 /home/ssu/Azas/tools/smoke/smoke_voice_cocktail_no_hardware.py "$@"
