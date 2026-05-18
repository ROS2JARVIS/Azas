#!/usr/bin/env bash
set -euo pipefail

# Field gate checker for moving from non-hardware smoke tests to real robot dry-run.
# This script does not command robot motion and does not call RG2 open/close services.
#
# Expected use:
#   1. Start the relevant bringup in another terminal, normally run_robot_dryrun.sh.
#   2. Put the target tumbler/lid in the camera view.
#   3. Run this script and inspect PASS/WARN/FAIL.

COLOR_TOPIC="${COLOR_TOPIC:-/camera/camera/color/image_raw}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
CUP_DETECTION_TOPIC="${CUP_DETECTION_TOPIC:-/azas/cup_detection}"
TUMBLER_POSE_TOPIC="${TUMBLER_POSE_TOPIC:-/jarvis/tumbler_dispenser/tumbler_pose}"
SERVICE_PREFIX="${SERVICE_PREFIX:-}"
STRICT="${STRICT:-false}"
GATE_STAMP="${GATE_STAMP:-/tmp/azas_live_hardware_gates_passed}"
POSE_MAX_AGE_SEC="${POSE_MAX_AGE_SEC:-5}"
REAL_MOTION_CONFIG_CHECK="${REAL_MOTION_CONFIG_CHECK:-/home/ssu/Azas/tools/checks/check_real_motion_config.sh}"
ROS2_DAEMON_FLAG="${ROS2_DAEMON_FLAG:---no-daemon}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"

mkdir -p "${ROS_LOG_DIR}"

failures=0
warnings=0
color_topic_ok=false
depth_topic_ok=false
camera_info_topic_ok=false
cup_detection_topic_ok=false
tumbler_pose_topic_ok=false

rm -f \
  /tmp/azas_live_gate_camera_info.txt \
  /tmp/azas_live_gate_detection.txt \
  /tmp/azas_live_gate_pose.txt \
  /tmp/azas_live_gate_topic.err \
  /tmp/azas_live_gate_service.err \
  /tmp/azas_live_gate_echo.err

motion_service() {
  local name="$1"
  local prefix="${SERVICE_PREFIX#/}"
  if [[ -z "${prefix}" ]]; then
    printf '/motion/%s\n' "${name}"
  else
    printf '/%s/motion/%s\n' "${prefix}" "${name}"
  fi
}

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

strict_fail_or_warn() {
  local message="$1"
  if [[ "${STRICT}" == "true" ]]; then
    fail "${message}"
  else
    warn "${message}"
  fi
}

validate_detection_sample() {
  local file="$1"
  local status
  status="$(sed -n 's/^[[:space:]]*status:[[:space:]]*//p' "${file}" | sed -n '1p')"
  status="${status%\"}"
  status="${status#\"}"
  status="${status%\'}"
  status="${status#\'}"

  if [[ "${status}" == detected:upright* ]]; then
    pass "cup detection reports detected:upright"
  else
    strict_fail_or_warn "cup detection is not motion-ready upright status: ${status:-unknown}"
    sed -n '1,80p' "${file}"
  fi
}

validate_tumbler_pose_sample() {
  local file="$1"
  local frame_id
  local sec
  local nanosec
  local now_sec
  local age_sec
  frame_id="$(sed -n 's/^[[:space:]]*frame_id:[[:space:]]*//p' "${file}" | sed -n '1p')"
  frame_id="${frame_id%\"}"
  frame_id="${frame_id#\"}"
  frame_id="${frame_id%\'}"
  frame_id="${frame_id#\'}"
  sec="$(sed -n 's/^[[:space:]]*sec:[[:space:]]*//p' "${file}" | sed -n '1p')"
  nanosec="$(sed -n 's/^[[:space:]]*nanosec:[[:space:]]*//p' "${file}" | sed -n '1p')"

  if [[ "${frame_id}" == "base_link" ]]; then
    pass "tumbler pose frame is base_link"
  else
    strict_fail_or_warn "tumbler pose frame is not base_link: ${frame_id:-unknown}"
  fi

  if ! [[ "${sec}" =~ ^[0-9]+$ && "${nanosec}" =~ ^[0-9]+$ ]]; then
    strict_fail_or_warn "tumbler pose stamp is missing or invalid"
    return
  fi
  if (( sec == 0 && nanosec == 0 )); then
    strict_fail_or_warn "tumbler pose stamp is zero"
    return
  fi

  now_sec="$(date +%s)"
  age_sec=$((now_sec - sec))
  if (( age_sec < 0 )); then
    strict_fail_or_warn "tumbler pose stamp is in the future: age=${age_sec}s"
  elif (( age_sec <= POSE_MAX_AGE_SEC )); then
    pass "tumbler pose stamp fresh: age=${age_sec}s <= ${POSE_MAX_AGE_SEC}s"
  else
    strict_fail_or_warn "tumbler pose stamp stale: age=${age_sec}s > ${POSE_MAX_AGE_SEC}s"
  fi
}

topic_exists() {
  local topic="$1"
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    ros2 topic list "${ROS2_DAEMON_FLAG}" 2>/tmp/azas_live_gate_topic.err | grep -qx "${topic}"
  else
    ros2 topic list 2>/tmp/azas_live_gate_topic.err | grep -qx "${topic}"
  fi
}

service_exists() {
  local service="$1"
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    ros2 service list "${ROS2_DAEMON_FLAG}" 2>/tmp/azas_live_gate_service.err | grep -qx "${service}"
  else
    ros2 service list 2>/tmp/azas_live_gate_service.err | grep -qx "${service}"
  fi
}

service_type() {
  local service="$1"
  local escaped
  escaped="$(printf '%s\n' "${service}" | sed 's/[.[\*^$()+?{}|]/\\&/g')"
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    timeout 5s ros2 service list -t "${ROS2_DAEMON_FLAG}" 2>/tmp/azas_live_gate_service_type.err \
      | sed -n "s#^${escaped}[[:space:]]*\\[\\(.*\\)\\]#\\1#p" \
      | sed -n '1p'
  else
    timeout 5s ros2 service list -t 2>/tmp/azas_live_gate_service_type.err \
      | sed -n "s#^${escaped}[[:space:]]*\\[\\(.*\\)\\]#\\1#p" \
      | sed -n '1p'
  fi
}

require_topic() {
  local topic="$1"
  local label="$2"
  if topic_exists "${topic}"; then
    pass "${label}: ${topic}"
    return 0
  else
    fail "${label} missing: ${topic}"
    return 1
  fi
}

optional_topic() {
  local topic="$1"
  local label="$2"
  if topic_exists "${topic}"; then
    pass "${label}: ${topic}"
    return 0
  else
    warn "${label} missing: ${topic}"
    return 1
  fi
}

require_service() {
  local service="$1"
  local label="$2"
  if service_exists "${service}"; then
    pass "${label}: ${service}"
  else
    fail "${label} missing: ${service}"
  fi
}

require_service_contract() {
  local service="$1"
  local label="$2"
  local expected_type="$3"
  local actual_type
  if service_exists "${service}"; then
    pass "${label}: ${service}"
    actual_type="$(service_type "${service}")"
    if [[ "${actual_type}" == "${expected_type}" ]]; then
      pass "${label} type: ${actual_type}"
    else
      fail "${label} type mismatch: expected ${expected_type}, got ${actual_type:-unknown}"
    fi
  else
    fail "${label} missing: ${service}"
  fi
}

optional_service() {
  local service="$1"
  local label="$2"
  if service_exists "${service}"; then
    pass "${label}: ${service}"
  else
    warn "${label} missing: ${service}"
  fi
}

optional_service_contract() {
  local service="$1"
  local label="$2"
  local expected_type="$3"
  local actual_type
  if service_exists "${service}"; then
    pass "${label}: ${service}"
    actual_type="$(service_type "${service}")"
    if [[ "${actual_type}" == "${expected_type}" ]]; then
      pass "${label} type: ${actual_type}"
    else
      fail "${label} type mismatch: expected ${expected_type}, got ${actual_type:-unknown}"
    fi
  else
    warn "${label} missing: ${service}"
  fi
}

sample_topic() {
  local topic="$1"
  local label="$2"
  local file="$3"
  local echo_cmd
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    echo_cmd=(ros2 topic echo --once "${topic}" "${ROS2_DAEMON_FLAG}")
  else
    echo_cmd=(ros2 topic echo --once "${topic}")
  fi
  if timeout 8s "${echo_cmd[@]}" >"${file}" 2>/tmp/azas_live_gate_echo.err; then
    pass "${label} sample received"
  else
    fail "${label} sample timeout: ${topic}"
  fi
}

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
set -u

echo "[Azas] Live hardware gate check. No motion commands will be sent."
echo "[Azas] topics: ${COLOR_TOPIC}, ${DEPTH_TOPIC}, ${CAMERA_INFO_TOPIC}"
echo "[Azas] detection: ${CUP_DETECTION_TOPIC} -> ${TUMBLER_POSE_TOPIC}"
echo "[Azas] service_prefix=${SERVICE_PREFIX:-<none>} strict=${STRICT}"
echo "[Azas] RG2 note: Trigger open/close service existence is not RG2 actuation proof."
echo "[Azas] RG2 note: fake service pass is not real RG2 readiness; real RG2 needs hardware confirmation and operator clearance."

require_topic "${COLOR_TOPIC}" "color image topic" && color_topic_ok=true
require_topic "${DEPTH_TOPIC}" "aligned depth topic" && depth_topic_ok=true
require_topic "${CAMERA_INFO_TOPIC}" "camera info topic" && camera_info_topic_ok=true
optional_topic "${CUP_DETECTION_TOPIC}" "cup detection topic" && cup_detection_topic_ok=true
optional_topic "${TUMBLER_POSE_TOPIC}" "tumbler pose topic" && tumbler_pose_topic_ok=true

if [[ "${camera_info_topic_ok}" == "true" ]]; then
  sample_topic "${CAMERA_INFO_TOPIC}" "camera info" /tmp/azas_live_gate_camera_info.txt
  if [[ -s /tmp/azas_live_gate_camera_info.txt ]]; then
    if grep -q "frame_id:" /tmp/azas_live_gate_camera_info.txt && grep -q "k:" /tmp/azas_live_gate_camera_info.txt; then
      pass "camera info includes frame_id and intrinsics"
    else
      fail "camera info sample lacks frame_id or intrinsics"
    fi
  fi
fi

if [[ "${depth_topic_ok}" == "true" && "${camera_info_topic_ok}" == "true" ]]; then
  if DEPTH_TOPIC="${DEPTH_TOPIC}" CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC}" TIMEOUT=8.0 \
    /home/ssu/Azas/tools/checks/check_depth_projection_sample.sh >/tmp/azas_live_gate_depth_projection.txt 2>/tmp/azas_live_gate_depth_projection.err; then
    pass "depth projection sample"
    sed -n '1,20p' /tmp/azas_live_gate_depth_projection.txt
  else
    fail "depth projection sample failed"
    sed -n '1,40p' /tmp/azas_live_gate_depth_projection.txt 2>/dev/null || true
    sed -n '1,40p' /tmp/azas_live_gate_depth_projection.err 2>/dev/null || true
  fi
fi

if [[ "${cup_detection_topic_ok}" == "true" ]]; then
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    detection_cmd=(ros2 topic echo --once "${CUP_DETECTION_TOPIC}" "${ROS2_DAEMON_FLAG}")
  else
    detection_cmd=(ros2 topic echo --once "${CUP_DETECTION_TOPIC}")
  fi
  if timeout 10s "${detection_cmd[@]}" >/tmp/azas_live_gate_detection.txt 2>/tmp/azas_live_gate_detection.err; then
    validate_detection_sample /tmp/azas_live_gate_detection.txt
  else
    warn "no cup detection sample within 10 seconds"
  fi
fi

if [[ "${tumbler_pose_topic_ok}" == "true" ]]; then
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    pose_cmd=(ros2 topic echo --once "${TUMBLER_POSE_TOPIC}" "${ROS2_DAEMON_FLAG}")
  else
    pose_cmd=(ros2 topic echo --once "${TUMBLER_POSE_TOPIC}")
  fi
  if timeout 10s "${pose_cmd[@]}" >/tmp/azas_live_gate_pose.txt 2>/tmp/azas_live_gate_pose.err; then
    pass "tumbler pose sample received"
    validate_tumbler_pose_sample /tmp/azas_live_gate_pose.txt
  else
    warn "no tumbler pose sample within 10 seconds"
  fi
fi

require_service_contract "$(motion_service move_line)" "Doosan MoveLine service" "dsr_msgs2/srv/MoveLine"
require_service_contract "$(motion_service move_joint)" "Doosan MoveJoint service" "dsr_msgs2/srv/MoveJoint"
optional_service_contract "/jarvis/rg2/open" "RG2 open Trigger service" "std_srvs/srv/Trigger"
optional_service_contract "/jarvis/rg2/close" "RG2 close Trigger service" "std_srvs/srv/Trigger"
optional_service_contract "/jarvis/rg2/set_width" "RG2 set_width no-motion/custom width service" "azas_interfaces/srv/SetGripper"
echo "[Azas] RG2 contract note: /jarvis/rg2/set_width checks the SetGripper service shape only; it does not command or validate real RG2 motion."

if [[ "${STRICT}" == "true" ]]; then
  if "${REAL_MOTION_CONFIG_CHECK}"; then
    pass "real-motion calibration/safety config gate"
  else
    fail "real-motion calibration/safety config gate"
  fi
fi

if [[ "${STRICT}" == "true" && "${warnings}" -ne 0 ]]; then
  failures=$((failures + warnings))
fi

echo "[Azas] Result: failures=${failures} warnings=${warnings} strict=${STRICT}"
if [[ "${failures}" -ne 0 ]]; then
  exit 1
fi

if [[ "${STRICT}" == "true" ]]; then
  {
    echo "strict=true"
    echo "timestamp=$(date -Is)"
    echo "color_topic=${COLOR_TOPIC}"
    echo "depth_topic=${DEPTH_TOPIC}"
    echo "camera_info_topic=${CAMERA_INFO_TOPIC}"
    echo "cup_detection_topic=${CUP_DETECTION_TOPIC}"
    echo "tumbler_pose_topic=${TUMBLER_POSE_TOPIC}"
    echo "service_prefix=${SERVICE_PREFIX}"
  } >"${GATE_STAMP}"
  echo "[Azas] Strict gate stamp written: ${GATE_STAMP}"
else
  echo "[Azas] Non-strict run: no real-motion gate stamp written."
fi
