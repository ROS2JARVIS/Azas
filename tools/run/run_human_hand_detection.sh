#!/usr/bin/env bash
# Perception-only human hand detection for the post-shake handover plan.
# Publishes /azas/human_hand_detection (PointStamped, camera optical frame),
# /azas/human_hand_detection/status (JSON), and an overlay image.
# This NEVER sends a robot motion command (gate: no_motion_hri_perception_only).
# Note: no `set -u`; ROS setup.bash references unset vars.
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/humble/setup.bash
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  source "${ROOT_DIR}/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-15}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

exec python3 "${ROOT_DIR}/tools/perception/human_hand_detection_node.py" "$@"
