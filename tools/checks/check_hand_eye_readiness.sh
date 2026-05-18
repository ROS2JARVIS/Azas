#!/usr/bin/env bash
set -euo pipefail

# No-motion hand-eye/calibration readiness helper.
# Inspects ROS topic and TF evidence only; it does not command Doosan motion,
# call gripper services, or start hardware nodes.

BASE_FRAME="${BASE_FRAME:-base_link}"
CAMERA_FRAME="${CAMERA_FRAME:-}"
COLOR_TOPIC="${COLOR_TOPIC:-/camera/camera/color/image_raw}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
CUP_DETECTION_TOPIC="${CUP_DETECTION_TOPIC:-/azas/cup_detection}"
TUMBLER_POSE_TOPIC="${TUMBLER_POSE_TOPIC:-/jarvis/tumbler_dispenser/tumbler_pose}"
CAMERA_INFO_TIMEOUT="${CAMERA_INFO_TIMEOUT:-5}"
TF_TIMEOUT="${TF_TIMEOUT:-6}"
ROS2_DAEMON_FLAG="${ROS2_DAEMON_FLAG:---no-daemon}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"

mkdir -p "${ROS_LOG_DIR}"

TOPICS_FILE="/tmp/azas_hand_eye_topics.txt"
CAMERA_INFO_FILE="/tmp/azas_hand_eye_camera_info.txt"
TF_ECHO_FILE="/tmp/azas_hand_eye_tf_echo.txt"

failures=0
warnings=0

pass() {
  echo "[PASS] $1"
}

warn() {
  echo "[WARN] $1"
  warnings=$((warnings + 1))
}

fail() {
  echo "[FAIL] $1"
  failures=$((failures + 1))
}

info() {
  echo "[INFO] $1"
}

topic_exists() {
  local topic="$1"
  grep -qx "${topic}" "${TOPICS_FILE}"
}

require_topic() {
  local topic="$1"
  local label="$2"
  if topic_exists "${topic}"; then
    pass "${label}: ${topic}"
    return 0
  fi

  fail "${label} missing: ${topic}"
  return 1
}

optional_topic() {
  local topic="$1"
  local label="$2"
  if topic_exists "${topic}"; then
    pass "${label}: ${topic}"
    return 0
  fi

  warn "${label} missing: ${topic}"
  return 1
}

camera_frame_from_info() {
  sed -n \
    "s/^[[:space:]]*frame_id:[[:space:]]*['\"]\\{0,1\\}\\([^'\"]*\\)['\"]\\{0,1\\}[[:space:]]*$/\\1/p" \
    "${CAMERA_INFO_FILE}" | sed -n '1p'
}

tf_stream_available() {
  topic_exists "/tf" || topic_exists "/tf_static"
}

transform_evidence_present() {
  grep -Eq "Translation:|transformStamped|TransformStamped" "${TF_ECHO_FILE}" \
    && ! grep -Eiq "Invalid frame ID|Could not transform|Lookup would require extrapolation" "${TF_ECHO_FILE}"
}

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -f /home/ssu/Azas/install/setup.bash ]]; then
  source /home/ssu/Azas/install/setup.bash
fi
if [[ -f /home/ssu/ros2_ws/install/setup.bash ]]; then
  source /home/ssu/ros2_ws/install/setup.bash
fi
set -u

echo "[Azas] Hand-eye/calibration readiness check. No motion commands will be sent."
echo "[Azas] base_frame=${BASE_FRAME}"
echo "[Azas] camera_frame=${CAMERA_FRAME:-<from CameraInfo>}"
echo "[Azas] camera_info_topic=${CAMERA_INFO_TOPIC}"
echo

rm -f "${TOPICS_FILE}" "${CAMERA_INFO_FILE}" "${TF_ECHO_FILE}"

if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
  topic_list_cmd=(ros2 topic list "${ROS2_DAEMON_FLAG}")
else
  topic_list_cmd=(ros2 topic list)
fi

if ! "${topic_list_cmd[@]}" >"${TOPICS_FILE}" 2>/tmp/azas_hand_eye_topic.err; then
  fail "could not list ROS topics"
  sed -n '1,20p' /tmp/azas_hand_eye_topic.err 2>/dev/null || true
else
  pass "ROS topic list available"
fi

require_topic "${COLOR_TOPIC}" "color image topic" || true
require_topic "${DEPTH_TOPIC}" "aligned depth topic" || true

camera_info_ok=false
if require_topic "${CAMERA_INFO_TOPIC}" "camera info topic"; then
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    camera_info_cmd=(ros2 topic echo --once "${CAMERA_INFO_TOPIC}" "${ROS2_DAEMON_FLAG}")
  else
    camera_info_cmd=(ros2 topic echo --once "${CAMERA_INFO_TOPIC}")
  fi
  if timeout "${CAMERA_INFO_TIMEOUT}s" "${camera_info_cmd[@]}" \
    >"${CAMERA_INFO_FILE}" 2>/tmp/azas_hand_eye_camera_info.err; then
    pass "camera info sample received"
    if grep -q "k:" "${CAMERA_INFO_FILE}"; then
      pass "camera info includes intrinsics"
    else
      fail "camera info sample lacks intrinsics"
    fi
    sampled_camera_frame="$(camera_frame_from_info)"
    if [[ -n "${sampled_camera_frame}" ]]; then
      pass "camera info frame_id: ${sampled_camera_frame}"
      camera_info_ok=true
      if [[ -z "${CAMERA_FRAME}" ]]; then
        CAMERA_FRAME="${sampled_camera_frame}"
      elif [[ "${CAMERA_FRAME}" != "${sampled_camera_frame}" ]]; then
        warn "CAMERA_FRAME=${CAMERA_FRAME} differs from CameraInfo frame_id=${sampled_camera_frame}"
      fi
    else
      fail "camera info sample lacks header.frame_id"
    fi
  else
    fail "camera info sample timeout: ${CAMERA_INFO_TOPIC}"
    sed -n '1,20p' /tmp/azas_hand_eye_camera_info.err 2>/dev/null || true
  fi
fi

optional_topic "${CUP_DETECTION_TOPIC}" "cup detection topic" || true
optional_topic "${TUMBLER_POSE_TOPIC}" "tumbler pose topic" || true

if tf_stream_available; then
  pass "TF topic available: /tf or /tf_static"
else
  fail "TF topics missing: /tf and /tf_static"
fi

if [[ -z "${CAMERA_FRAME}" ]]; then
  fail "camera frame unknown; set CAMERA_FRAME or publish CameraInfo.header.frame_id"
else
  info "checking TF evidence between ${BASE_FRAME} and ${CAMERA_FRAME}"
  timeout "${TF_TIMEOUT}s" ros2 run tf2_ros tf2_echo "${BASE_FRAME}" "${CAMERA_FRAME}" \
    >"${TF_ECHO_FILE}" 2>&1 || true
  if transform_evidence_present; then
    pass "TF transform evidence present: ${BASE_FRAME} <-> ${CAMERA_FRAME}"
    sed -n '1,18p' "${TF_ECHO_FILE}"
  else
    fail "TF transform evidence missing: ${BASE_FRAME} <-> ${CAMERA_FRAME}"
    sed -n '1,30p' "${TF_ECHO_FILE}" 2>/dev/null || true
  fi
fi

echo
echo "[Azas] Result: failures=${failures} warnings=${warnings} camera_info_sample=${camera_info_ok}"
if [[ "${failures}" -ne 0 ]]; then
  echo "[Azas] NOT READY: camera-to-base transform evidence is missing or prerequisite topics are unavailable."
  exit 1
fi

echo "[Azas] READY: camera-to-base TF evidence is present for the sampled camera frame."
