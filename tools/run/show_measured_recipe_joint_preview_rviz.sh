#!/usr/bin/env bash
set -euo pipefail

# RViz-only measured joint preview.  This does not call Doosan motion or gripper
# services; it only publishes /joint_states from calibration.yaml.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/log/manual}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${ROOT_DIR}/src/azas_bringup/rviz/azas_rule_motion_preview.rviz}"
ROBOT_COLOR="${ROBOT_COLOR:-white}"
SHOW_WORKSPACE_SAFETY="${SHOW_WORKSPACE_SAFETY:-false}"
SHOW_FULL_COLLISION_SCENE="${SHOW_FULL_COLLISION_SCENE:-false}"
SHOW_MEASURED_DISPENSER_COLLISION="${SHOW_MEASURED_DISPENSER_COLLISION:-true}"
START_FULL_COLLISION_MARKERS="${START_FULL_COLLISION_MARKERS:-false}"
PREVIEW_ARGS=("$@")
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
pkill -f 'rule_motion_joint_preview_node' 2>/dev/null || true
pkill -f 'm0609_shake_joint_state_node' 2>/dev/null || true

ros2 launch azas_bringup hardware_free_demo.launch.py \
  use_rviz:=false \
  use_robot_urdf:=true \
  robot_color:="${ROBOT_COLOR}" \
  enable_ik_preview:=false \
  run_live_stt:=false \
  run_recipe_mapper:=false \
  use_llm:=false \
  show_sequence_markers:=false \
  show_dispenser_markers:=false \
  show_animated_cup:=false \
  show_demo_arm:=false \
  show_workspace_safety:="${SHOW_WORKSPACE_SAFETY}" \
  show_measured_dispenser_collision:="${SHOW_MEASURED_DISPENSER_COLLISION}" \
  show_full_collision_scene:="${SHOW_FULL_COLLISION_SCENE}" \
  show_link6_gripper:=true \
  enable_rule_joint_preview:=false \
  >"${LOG_DIR}/measured_joint_preview_description.log" 2>&1 &
DESC_PID=$!

COLLISION_PID=""
if [[ "${START_FULL_COLLISION_MARKERS}" == "true" ]]; then
  python3 tools/run/publish_collision_scene_rviz.py \
    >"${LOG_DIR}/measured_joint_preview_collision.log" 2>&1 &
  COLLISION_PID=$!
fi

python3 tools/run/publish_measured_recipe_joint_rviz_preview.py "${PREVIEW_ARGS[@]}" \
  >"${LOG_DIR}/measured_joint_preview_joints.log" 2>&1 &
JOINT_PID=$!

rviz2 -d "${RVIZ_CONFIG}" \
  >"${LOG_DIR}/measured_joint_preview_rviz.log" 2>&1 &
RVIZ_PID=$!

echo "[Azas] RViz measured joint preview started."
echo "[Azas] Session marker: ${SESSION_NAME}"
echo "[Azas] Args: ${PREVIEW_ARGS[*]:-(default recipe/color map)}"
echo "[Azas] PIDs: description=${DESC_PID} collision=${COLLISION_PID} joints=${JOINT_PID} rviz=${RVIZ_PID}"
echo "[Azas] Logs: ${LOG_DIR}/measured_joint_preview_*.log"

wait "${RVIZ_PID}"
