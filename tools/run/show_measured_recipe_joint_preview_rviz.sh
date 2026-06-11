#!/usr/bin/env bash
set -euo pipefail

# RViz-only measured joint preview.  This starts robot_state_publisher + RViz and
# publishes /joint_states from calibration.yaml. It does not call Doosan motion,
# MoveJoint, MoveLine, or gripper services.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/log/manual}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${ROOT_DIR}/src/azas_bringup/rviz/azas_dispenser_sequence_clean.rviz}"
SAFETY_CONFIG="${SAFETY_CONFIG:-${ROOT_DIR}/src/azas_bringup/config/safety.yaml}"
DISPENSER_COLLISION_CONFIG="${DISPENSER_COLLISION_CONFIG:-${ROOT_DIR}/src/azas_bringup/config/measured_dispenser_collision.yaml}"
CALIBRATION_CONFIG="${CALIBRATION_CONFIG:-${ROOT_DIR}/src/azas_bringup/config/calibration.yaml}"
ROBOT_COLOR="${ROBOT_COLOR:-white}"
PUBLISH_RATE="${PUBLISH_RATE:-60.0}"
SEGMENT_SECONDS="${SEGMENT_SECONDS:-4.0}"
JOINT_VELOCITY_DEG_S="${JOINT_VELOCITY_DEG_S:-40.0}"
HOLD_SECONDS="${HOLD_SECONDS:-1.0}"
SHOW_WORKSPACE_SAFETY="${SHOW_WORKSPACE_SAFETY:-true}"
SHOW_FULL_COLLISION_SCENE="${SHOW_FULL_COLLISION_SCENE:-true}"
SHOW_MEASURED_DISPENSER_COLLISION="${SHOW_MEASURED_DISPENSER_COLLISION:-true}"
SHOW_LINK6_GRIPPER="${SHOW_LINK6_GRIPPER:-true}"
PUBLISH_WORKSPACE_COLLISION_OBJECTS="${PUBLISH_WORKSPACE_COLLISION_OBJECTS:-true}"
PUBLISH_DISPENSER_COLLISION_OBJECTS="${PUBLISH_DISPENSER_COLLISION_OBJECTS:-true}"
RESET_STALE_PREVIEW_NODES="${RESET_STALE_PREVIEW_NODES:-true}"
START_FULL_COLLISION_MARKERS="${START_FULL_COLLISION_MARKERS:-false}"
PREVIEW_ARGS=("$@")
if [[ ${#PREVIEW_ARGS[@]} -gt 0 && "${PREVIEW_ARGS[0]}" != --* ]]; then
  PREVIEW_ARGS=("--dispenser-ids" "${PREVIEW_ARGS[@]}")
fi
SESSION_NAME="${SESSION_NAME:-azas-measured-joint-preview}"

mkdir -p "${LOG_DIR}"

DESC_PID=""
COLLISION_PID=""
JOINT_PID=""
RVIZ_PID=""

cleanup() {
  for pid in "${JOINT_PID}" "${COLLISION_PID}" "${DESC_PID}" "${RVIZ_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

cd "${ROOT_DIR}"
set +u
source /opt/ros/humble/setup.bash
if [[ -f /home/ssu/ws_moveit/install/setup.bash ]]; then
  source /home/ssu/ws_moveit/install/setup.bash
fi
if [[ -f /home/ssu/ros2_ws/install/setup.bash ]]; then
  source /home/ssu/ros2_ws/install/setup.bash
fi
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  source "${ROOT_DIR}/install/setup.bash"
else
  source "${ROOT_DIR}/install/local_setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-9}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

pkill -f 'publish_measured_recipe_joint_rviz_preview.py' 2>/dev/null || true
pkill -f 'preview_measured_dispenser_recipe_rviz.py' 2>/dev/null || true
pkill -f 'rule_motion_joint_preview_node' 2>/dev/null || true
pkill -f 'm0609_shake_joint_state_node' 2>/dev/null || true
if [[ "${RESET_STALE_PREVIEW_NODES}" == "true" || "${RESET_STALE_PREVIEW_NODES}" == "1" ]]; then
  pkill -f 'dispenser_sequence_preview_node' 2>/dev/null || true
  pkill -f 'workspace_collision_scene_node' 2>/dev/null || true
  pkill -f 'measured_dispenser_collision_scene_node' 2>/dev/null || true
  pkill -f 'collision_scene_rviz_publisher' 2>/dev/null || true
  pkill -f 'link6_gripper_collision_node' 2>/dev/null || true
  pkill -f '__node:=m0609_robot_state_publisher' 2>/dev/null || true
fi

ros2 launch "${ROOT_DIR}/src/azas_bringup/launch/measured_joint_preview_display.launch.py" \
  use_rviz:=false \
  robot_color:="${ROBOT_COLOR}" \
  show_workspace_safety:="${SHOW_WORKSPACE_SAFETY}" \
  show_measured_dispenser_collision:="${SHOW_MEASURED_DISPENSER_COLLISION}" \
  show_full_collision_scene:="${SHOW_FULL_COLLISION_SCENE}" \
  show_link6_gripper:="${SHOW_LINK6_GRIPPER}" \
  publish_workspace_collision_objects:="${PUBLISH_WORKSPACE_COLLISION_OBJECTS}" \
  publish_dispenser_collision_objects:="${PUBLISH_DISPENSER_COLLISION_OBJECTS}" \
  safety_config_path:="${SAFETY_CONFIG}" \
  dispenser_collision_config_path:="${DISPENSER_COLLISION_CONFIG}" \
  calibration_path:="${CALIBRATION_CONFIG}" \
  >"${LOG_DIR}/measured_joint_preview_description.log" 2>&1 &
DESC_PID=$!

COLLISION_PID=""
if [[ "${START_FULL_COLLISION_MARKERS}" == "true" ]]; then
  python3 tools/run/publish_collision_scene_rviz.py \
    >"${LOG_DIR}/measured_joint_preview_collision.log" 2>&1 &
  COLLISION_PID=$!
fi

python3 tools/run/preview_measured_dispenser_recipe_rviz.py \
  --rate-hz "${PUBLISH_RATE}" \
  --segment-seconds "${SEGMENT_SECONDS}" \
  --joint-velocity-deg-s "${JOINT_VELOCITY_DEG_S}" \
  --hold-seconds "${HOLD_SECONDS}" \
  "${PREVIEW_ARGS[@]}" \
  >"${LOG_DIR}/measured_joint_preview_joints.log" 2>&1 &
JOINT_PID=$!

rviz2 -d "${RVIZ_CONFIG}" \
  >"${LOG_DIR}/measured_joint_preview_rviz.log" 2>&1 &
RVIZ_PID=$!

echo "[Azas] RViz measured joint preview started."
echo "[Azas] Session marker: ${SESSION_NAME}"
echo "[Azas] Args: ${PREVIEW_ARGS[*]:-(default recipe/color map)}"
echo "[Azas] safety_config=${SAFETY_CONFIG}"
echo "[Azas] dispenser_collision_config=${DISPENSER_COLLISION_CONFIG}"
echo "[Azas] calibration_config=${CALIBRATION_CONFIG}"
echo "[Azas] preview_joint_velocity_deg_s=${JOINT_VELOCITY_DEG_S}"
echo "[Azas] PIDs: description=${DESC_PID} collision=${COLLISION_PID} joints=${JOINT_PID} rviz=${RVIZ_PID}"
echo "[Azas] Logs: ${LOG_DIR}/measured_joint_preview_*.log"

wait "${RVIZ_PID}"
