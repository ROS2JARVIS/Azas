#!/usr/bin/env bash
set -euo pipefail

# Explain why real robot execution is currently blocked. This script sends no
# robot motion, no RG2 command, and no dispenser command.

MOTION_HOLD_FILE="${MOTION_HOLD_FILE:-/tmp/azas_motion_hold}"
GATE_STAMP="${GATE_STAMP:-/tmp/azas_live_hardware_gates_passed}"
LIVE_GATE_MAX_AGE_SEC="${LIVE_GATE_MAX_AGE_SEC:-600}"
SERVICE_PREFIX="${SERVICE_PREFIX:-}"
COLOR_TOPIC="${COLOR_TOPIC:-/camera/camera/color/image_raw}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
ROS2_DAEMON_FLAG="${ROS2_DAEMON_FLAG:---no-daemon}"

blockers=0

block() {
  echo "[BLOCKED] $1"
  blockers=$((blockers + 1))
}

ok() {
  echo "[OK] $1"
}

info() {
  echo "[INFO] $1"
}

motion_service() {
  local name="$1"
  local prefix="${SERVICE_PREFIX#/}"
  if [[ -z "${prefix}" ]]; then
    printf '/motion/%s\n' "${name}"
  else
    printf '/%s/motion/%s\n' "${prefix}" "${name}"
  fi
}

ros_topic_exists() {
  local topic="$1"
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    ros2 topic list "${ROS2_DAEMON_FLAG}" 2>/tmp/azas_explain_topics.err | grep -qx "${topic}"
  else
    ros2 topic list 2>/tmp/azas_explain_topics.err | grep -qx "${topic}"
  fi
}

ros_service_exists() {
  local service="$1"
  if [[ -n "${ROS2_DAEMON_FLAG}" ]]; then
    ros2 service list "${ROS2_DAEMON_FLAG}" 2>/tmp/azas_explain_services.err | grep -qx "${service}"
  else
    ros2 service list 2>/tmp/azas_explain_services.err | grep -qx "${service}"
  fi
}

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash 2>/dev/null
source /home/ssu/ros2_ws/install/setup.bash 2>/dev/null
set -u

echo "[Azas] Real robot execution blocker report"
echo "[Azas] No motion or gripper commands will be sent."
echo "[Azas] service_prefix=${SERVICE_PREFIX:-<none>}"

if [[ -f "${MOTION_HOLD_FILE}" ]]; then
  block "motion hold is active: ${MOTION_HOLD_FILE}"
  sed -n '1,20p' "${MOTION_HOLD_FILE}" 2>/dev/null || true
else
  ok "no motion hold file"
fi

if [[ -f "${GATE_STAMP}" ]] && grep -qx "strict=true" "${GATE_STAMP}"; then
  now_sec="$(date +%s)"
  stamp_sec="$(stat -c %Y "${GATE_STAMP}")"
  age_sec=$((now_sec - stamp_sec))
  if (( age_sec <= LIVE_GATE_MAX_AGE_SEC )); then
    ok "fresh strict live gate stamp: ${GATE_STAMP} age=${age_sec}s"
  else
    block "strict live gate stamp is stale: ${GATE_STAMP} age=${age_sec}s > ${LIVE_GATE_MAX_AGE_SEC}s"
  fi
else
  block "missing fresh strict live gate stamp: ${GATE_STAMP}"
fi

if /home/ssu/Azas/tools/checks/check_real_motion_config.sh >/tmp/azas_explain_config.out 2>&1; then
  ok "measured calibration/safety config passes"
else
  block "measured calibration/safety config does not pass"
  sed -n '1,80p' /tmp/azas_explain_config.out || true
fi

for topic in "${COLOR_TOPIC}" "${DEPTH_TOPIC}" "${CAMERA_INFO_TOPIC}"; do
  if ros_topic_exists "${topic}"; then
    ok "camera topic present: ${topic}"
  else
    block "camera topic missing: ${topic}"
  fi
done

move_line_service="$(motion_service move_line)"
move_joint_service="$(motion_service move_joint)"
if ros_service_exists "${move_line_service}"; then
  ok "Doosan MoveLine service present: ${move_line_service}"
else
  block "Doosan MoveLine service missing: ${move_line_service}"
fi
if ros_service_exists "${move_joint_service}"; then
  ok "Doosan MoveJoint service present: ${move_joint_service}"
else
  block "Doosan MoveJoint service missing: ${move_joint_service}"
fi

if ros_service_exists /jarvis/rg2/open && ros_service_exists /jarvis/rg2/close; then
  ok "RG2 Trigger services present"
else
  block "RG2 Trigger services missing: /jarvis/rg2/open and/or /jarvis/rg2/close"
fi

if [[ "${blockers}" -eq 0 ]]; then
  echo "[READY] Real robot entrypoint prerequisites appear satisfied."
  echo "[NEXT] /home/ssu/Azas/tools/run/run_robot_real.sh"
else
  echo "[NOT READY] blockers=${blockers}"
  echo "[NEXT] Fix the blockers above, then rerun this script or /home/ssu/Azas/tools/checks/robot_connection_acceptance.sh."
  exit 1
fi
