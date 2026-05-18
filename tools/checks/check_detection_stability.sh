#!/usr/bin/env bash
set -euo pipefail

# Sample live YOLO CupDetection messages and summarize cup/lid stability.
# This script sends no robot or gripper commands.

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
set -u

exec python3 /home/ssu/Azas/tools/checks/check_detection_stability.py "$@"
