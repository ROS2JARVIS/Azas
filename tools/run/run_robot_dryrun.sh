#!/usr/bin/env bash
set -euo pipefail

# Safe default entrypoint for camera + YOLO + RG2 service + floor-place dry-run.
# This does NOT move the real robot because enable_hardware remains false.

SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID:-2}"
RG2_IP="${RG2_IP:-192.168.1.1}"
ENABLE_REALSENSE="${ENABLE_REALSENSE:-true}"
ENABLE_RG2="${ENABLE_RG2:-true}"
COLOR_TOPIC="${COLOR_TOPIC:-/camera/camera/color/image_raw}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
TUMBLER_POSE_TARGET_FRAME="${TUMBLER_POSE_TARGET_FRAME:-base_link}"
SOURCE_FRAME="${SOURCE_FRAME:-camera_color_optical_frame}"
REQUIRE_TUMBLER_POSE_TF="${REQUIRE_TUMBLER_POSE_TF:-true}"
TARGET_CLASS_NAMES="${TARGET_CLASS_NAMES:-cup,tumbler,bottle}"
SELECTION_POLICY="${SELECTION_POLICY:-largest_bbox}"
DEPTH_WINDOW_SIZE="${DEPTH_WINDOW_SIZE:-7}"
MIN_DEPTH_M="${MIN_DEPTH_M:-0.15}"
MAX_DEPTH_M="${MAX_DEPTH_M:-2.0}"
PLACE_MOUTH_UNDER_OUTLET="${PLACE_MOUTH_UNDER_OUTLET:-true}"
OUTLET_MOUTH_CLEARANCE="${OUTLET_MOUTH_CLEARANCE:-0.0}"
PUBLISH_CAMERA_BASE_TF="${PUBLISH_CAMERA_BASE_TF:-false}"
CAMERA_BASE_TF_PARENT_FRAME="${CAMERA_BASE_TF_PARENT_FRAME:-base_link}"
CAMERA_BASE_TF_CHILD_FRAME="${CAMERA_BASE_TF_CHILD_FRAME:-camera_color_optical_frame}"
CAMERA_BASE_TF_X="${CAMERA_BASE_TF_X:-0.0}"
CAMERA_BASE_TF_Y="${CAMERA_BASE_TF_Y:-0.0}"
CAMERA_BASE_TF_Z="${CAMERA_BASE_TF_Z:-0.0}"
CAMERA_BASE_TF_ROLL="${CAMERA_BASE_TF_ROLL:-0.0}"
CAMERA_BASE_TF_PITCH="${CAMERA_BASE_TF_PITCH:-0.0}"
CAMERA_BASE_TF_YAW="${CAMERA_BASE_TF_YAW:-0.0}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
set -u

echo "[Azas] Starting SAFE dry-run. Real robot motion is disabled."
echo "[Azas] selected_dispenser_id=${SELECTED_DISPENSER_ID} rg2_ip=${RG2_IP}"
echo "[Azas] camera topics: ${COLOR_TOPIC}, ${DEPTH_TOPIC}, ${CAMERA_INFO_TOPIC}"
echo "[Azas] outlet alignment: place_mouth_under_outlet=${PLACE_MOUTH_UNDER_OUTLET} clearance=${OUTLET_MOUTH_CLEARANCE}m"
if [[ "${PUBLISH_CAMERA_BASE_TF}" == "true" ]]; then
  echo "[Azas] publishing explicit camera->base static TF for no-motion validation:"
  echo "[Azas]   ${CAMERA_BASE_TF_PARENT_FRAME} -> ${CAMERA_BASE_TF_CHILD_FRAME}"
  echo "[Azas]   xyz=(${CAMERA_BASE_TF_X}, ${CAMERA_BASE_TF_Y}, ${CAMERA_BASE_TF_Z}) rpy=(${CAMERA_BASE_TF_ROLL}, ${CAMERA_BASE_TF_PITCH}, ${CAMERA_BASE_TF_YAW})"
  echo "[Azas]   Use measured hand-eye values only; this remains enable_hardware=false."
fi

exec ros2 launch azas_bringup robot_connection_control.launch.py \
  selected_dispenser_id:="${SELECTED_DISPENSER_ID}" \
  enable_realsense:="${ENABLE_REALSENSE}" \
  enable_rg2:="${ENABLE_RG2}" \
  ip:="${RG2_IP}" \
  color_topic:="${COLOR_TOPIC}" \
  depth_topic:="${DEPTH_TOPIC}" \
  camera_info_topic:="${CAMERA_INFO_TOPIC}" \
  tumbler_pose_target_frame:="${TUMBLER_POSE_TARGET_FRAME}" \
  source_frame:="${SOURCE_FRAME}" \
  require_tumbler_pose_tf:="${REQUIRE_TUMBLER_POSE_TF}" \
  target_class_names:="${TARGET_CLASS_NAMES}" \
  selection_policy:="${SELECTION_POLICY}" \
  depth_window_size:="${DEPTH_WINDOW_SIZE}" \
  min_depth_m:="${MIN_DEPTH_M}" \
  max_depth_m:="${MAX_DEPTH_M}" \
  place_mouth_under_outlet:="${PLACE_MOUTH_UNDER_OUTLET}" \
  outlet_mouth_clearance:="${OUTLET_MOUTH_CLEARANCE}" \
  publish_camera_base_tf:="${PUBLISH_CAMERA_BASE_TF}" \
  camera_base_tf_parent_frame:="${CAMERA_BASE_TF_PARENT_FRAME}" \
  camera_base_tf_child_frame:="${CAMERA_BASE_TF_CHILD_FRAME}" \
  camera_base_tf_x:="${CAMERA_BASE_TF_X}" \
  camera_base_tf_y:="${CAMERA_BASE_TF_Y}" \
  camera_base_tf_z:="${CAMERA_BASE_TF_Z}" \
  camera_base_tf_roll:="${CAMERA_BASE_TF_ROLL}" \
  camera_base_tf_pitch:="${CAMERA_BASE_TF_PITCH}" \
  camera_base_tf_yaw:="${CAMERA_BASE_TF_YAW}" \
  enable_hardware:=false
