#!/usr/bin/env bash
set -euo pipefail

# Quick no-motion stage report for deciding what to connect next.
# This script does not start cameras, call gripper services, or command robot motion.

COLOR_TOPIC="${COLOR_TOPIC:-/camera/camera/color/image_raw}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
CUP_DETECTION_TOPIC="${CUP_DETECTION_TOPIC:-/azas/cup_detection}"
TUMBLER_POSE_TOPIC="${TUMBLER_POSE_TOPIC:-/jarvis/tumbler_dispenser/tumbler_pose}"
SERVICE_PREFIX="${SERVICE_PREFIX:-}"
REAL_MOTION_CONFIG_CHECK="${REAL_MOTION_CONFIG_CHECK:-/home/ssu/Azas/tools/checks/check_real_motion_config.sh}"
ROS2_DAEMON_FLAG="${ROS2_DAEMON_FLAG:---no-daemon}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"

mkdir -p "${ROS_LOG_DIR}"

motion_service() {
  local name="$1"
  local prefix="${SERVICE_PREFIX#/}"
  if [[ -z "${prefix}" ]]; then
    printf '/motion/%s\n' "${name}"
  else
    printf '/%s/motion/%s\n' "${prefix}" "${name}"
  fi
}

has_topic() {
  local topic="$1"
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    ros2 topic list "${ROS2_DAEMON_FLAG}" 2>/tmp/azas_stage_topic.err | grep -qx "${topic}"
  else
    ros2 topic list 2>/tmp/azas_stage_topic.err | grep -qx "${topic}"
  fi
}

has_service() {
  local service="$1"
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    ros2 service list "${ROS2_DAEMON_FLAG}" 2>/tmp/azas_stage_service.err | grep -qx "${service}"
  else
    ros2 service list 2>/tmp/azas_stage_service.err | grep -qx "${service}"
  fi
}

service_type() {
  local service="$1"
  local escaped
  escaped="$(printf '%s\n' "${service}" | sed 's/[.[\*^$()+?{}|]/\\&/g')"
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    timeout 5s ros2 service list -t "${ROS2_DAEMON_FLAG}" 2>/tmp/azas_stage_service_type.err \
      | sed -n "s#^${escaped}[[:space:]]*\\[\\(.*\\)\\]#\\1#p" \
      | sed -n '1p'
  else
    timeout 5s ros2 service list -t 2>/tmp/azas_stage_service_type.err \
      | sed -n "s#^${escaped}[[:space:]]*\\[\\(.*\\)\\]#\\1#p" \
      | sed -n '1p'
  fi
}

line() {
  printf '%-42s %s\n' "$1" "$2"
}

set +u
source /opt/ros/humble/setup.bash
if [[ -f /home/ssu/Azas/install/setup.bash ]]; then
  source /home/ssu/Azas/install/setup.bash
fi
if [[ -f /home/ssu/ros2_ws/install/setup.bash ]]; then
  source /home/ssu/ros2_ws/install/setup.bash
fi
set -u

echo "[Azas] Connection stage report. No motion commands will be sent."
echo "[Azas] service_prefix=${SERVICE_PREFIX:-<none>}"
echo

camera_topics_ok=true
for item in \
  "color image:${COLOR_TOPIC}" \
  "aligned depth:${DEPTH_TOPIC}" \
  "camera info:${CAMERA_INFO_TOPIC}"; do
  label="${item%%:*}"
  topic="${item#*:}"
  if has_topic "${topic}"; then
    line "${label}" "OK ${topic}"
  else
    line "${label}" "MISSING ${topic}"
    camera_topics_ok=false
  fi
done

detection_ok=false
if has_topic "${CUP_DETECTION_TOPIC}"; then
  line "cup detection topic" "OK ${CUP_DETECTION_TOPIC}"
  detection_ok=true
else
  line "cup detection topic" "MISSING ${CUP_DETECTION_TOPIC}"
fi

pose_ok=false
if has_topic "${TUMBLER_POSE_TOPIC}"; then
  line "tumbler pose topic" "OK ${TUMBLER_POSE_TOPIC}"
  pose_ok=true
else
  line "tumbler pose topic" "MISSING ${TUMBLER_POSE_TOPIC}"
fi

detection_sample_ok=false
if [[ "${detection_ok}" == "true" ]]; then
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    detection_cmd=(ros2 topic echo --once "${CUP_DETECTION_TOPIC}" "${ROS2_DAEMON_FLAG}")
  else
    detection_cmd=(ros2 topic echo --once "${CUP_DETECTION_TOPIC}")
  fi
  if timeout 5s "${detection_cmd[@]}" >/tmp/azas_stage_detection.out 2>/tmp/azas_stage_detection.err; then
    detection_status="$(sed -n 's/^[[:space:]]*status:[[:space:]]*//p' /tmp/azas_stage_detection.out | sed -n '1p')"
    detection_confidence="$(sed -n 's/^[[:space:]]*confidence:[[:space:]]*//p' /tmp/azas_stage_detection.out | sed -n '1p')"
    if [[ "${detection_status}" == detected:upright* ]]; then
      line "cup detection sample" "OK ${detection_status} confidence=${detection_confidence:-unknown}"
      detection_sample_ok=true
    elif [[ "${detection_status}" == detected:* ]]; then
      line "cup detection sample" "DETECTED BUT NOT MOTION-READY ${detection_status} confidence=${detection_confidence:-unknown}"
    else
      line "cup detection sample" "NOT DETECTED ${detection_status:-unknown} confidence=${detection_confidence:-unknown}"
    fi
  else
    line "cup detection sample" "NO SAMPLE"
  fi
fi

move_line_service="$(motion_service move_line)"
move_joint_service="$(motion_service move_joint)"
robot_services_ok=true
for item in \
  "Doosan MoveLine:${move_line_service}" \
  "Doosan MoveJoint:${move_joint_service}" \
  "RG2 open:/jarvis/rg2/open" \
  "RG2 close:/jarvis/rg2/close"; do
  label="${item%%:*}"
  service="${item#*:}"
  if has_service "${service}"; then
    type_name="$(service_type "${service}")"
    if [[ -n "${type_name}" ]]; then
      line "${label}" "OK ${service} [${type_name}]"
    else
      line "${label}" "OK ${service} [type unknown]"
    fi
  else
    line "${label}" "MISSING ${service}"
    robot_services_ok=false
  fi
done

config_ok=false
if "${REAL_MOTION_CONFIG_CHECK}" >/tmp/azas_stage_config.out 2>/tmp/azas_stage_config.err; then
  line "real-motion config gate" "OK"
  config_ok=true
else
  line "real-motion config gate" "BLOCKED"
fi

echo
echo "[Azas] Recommended next step:"
if [[ "${camera_topics_ok}" != "true" ]]; then
  echo "  Connect/start the RealSense camera first, then run:"
  echo "    /home/ssu/Azas/tools/run/run_robot_dryrun.sh"
  echo "    /home/ssu/Azas/tools/checks/check_robot_detection.sh"
  echo "    /home/ssu/Azas/tools/checks/check_depth_projection_sample.sh"
elif [[ "${detection_ok}" != "true" || "${pose_ok}" != "true" || "${detection_sample_ok}" != "true" ]]; then
  echo "  Camera topics exist, but live cup/lid detection is not confirmed yet."
  echo "  Put the tumbler/lid clearly in the color camera view and rerun:"
  echo "    /home/ssu/Azas/tools/run/run_robot_dryrun.sh"
  echo "    /home/ssu/Azas/tools/checks/check_robot_detection.sh"
  echo "    /home/ssu/Azas/tools/checks/check_connection_stage.sh"
elif [[ "${robot_services_ok}" != "true" ]]; then
  echo "  Camera/perception graph is present. Next connect Doosan/RG2 for no-motion service checks."
  echo "  Do not run real motion yet."
elif [[ "${config_ok}" != "true" ]]; then
  echo "  Hardware services are present, but measured calibration/safety config is still blocking real motion."
  echo "  Fill calibration.yaml and safety.yaml from measured values, then run strict live gates."
else
  echo "  All quick gates are present. Run the strict live gate before any real motion:"
  echo "    STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed /home/ssu/Azas/tools/checks/check_live_hardware_gates.sh"
fi
