#!/usr/bin/env bash
set -euo pipefail

# RViz-only preview of the dispenser color-classification camera pose.
# It publishes visual /joint_states for [0, 10, 32, 0, 100, 90] deg and
# never calls a Doosan motion service.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-79}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

set +u
source /opt/ros/humble/setup.bash
if [[ -f /home/ssu/ros2_ws/install/setup.bash ]]; then
  source /home/ssu/ros2_ws/install/setup.bash
fi
if [[ -f /home/ssu/ws_moveit/install/setup.bash ]]; then
  source /home/ssu/ws_moveit/install/setup.bash
fi
if [[ -f "${ROOT}/install/setup.bash" ]]; then
  source "${ROOT}/install/setup.bash"
else
  source "${ROOT}/install/local_setup.bash"
fi
set -u

echo "[Azas] RViz color scan pose preview"
echo "[Azas] joints_deg=[0, 10, 32, 0, 100, 90]"
echo "[Azas] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[Azas] RViz-only: robot model loops HOME -> color scan pose -> HOME; no real robot motion command will be sent"

exec ros2 launch "${ROOT}/src/azas_bringup/launch/color_scan_pose_rviz.launch.py" \
  use_rviz:="${USE_RVIZ:-true}" \
  preview_mode:="${PREVIEW_MODE:-color_scan_pose_move}"
