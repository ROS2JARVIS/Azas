#!/usr/bin/env bash
set -euo pipefail

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
NODE_LOG_FILE="${NODE_LOG_FILE:-/tmp/azas_smoke_cocktail_dryrun_node.log}"
mkdir -p "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
set -u

rm -f "${NODE_LOG_FILE}"
ros2 run azas_task_manager cocktail_dryrun_sequence_node >"${NODE_LOG_FILE}" 2>&1 &
NODE_PID=$!
cleanup() {
  kill -9 "${NODE_PID}" 2>/dev/null || true
}
trap cleanup EXIT

python3 /home/ssu/Azas/tools/smoke/smoke_cocktail_dryrun_sequence.py
