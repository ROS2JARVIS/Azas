#!/usr/bin/env bash
set -euo pipefail

# Live camera depth-projection gate. This does not command robot motion.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
TIMEOUT="${TIMEOUT:-10.0}"
PATCH_RADIUS="${PATCH_RADIUS:-3}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

exec python3 "${ROOT_DIR}/tools/checks/check_depth_projection_sample.py" \
  --depth-topic "${DEPTH_TOPIC}" \
  --camera-info-topic "${CAMERA_INFO_TOPIC}" \
  --timeout "${TIMEOUT}" \
  --patch-radius "${PATCH_RADIUS}"
