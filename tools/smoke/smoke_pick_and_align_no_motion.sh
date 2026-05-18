#!/usr/bin/env bash
set -euo pipefail

# No-motion PickAndAlign side-grasp smoke:
# fake base_link PoseStamped -> /azas/pick_and_align -> side no-motion states.
# This does not start YOLO, RealSense, Doosan, MoveIt, or real RG2 control.

ACTION_LOG="${ACTION_LOG:-/tmp/azas_smoke_pick_and_align_no_motion_action.log}"
SERVER_LOG="${SERVER_LOG:-/tmp/azas_smoke_pick_and_align_no_motion_server.log}"
POSE_TOPIC="${POSE_TOPIC:-/jarvis/tumbler_dispenser/tumbler_pose}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"

rm -f "${ACTION_LOG}" "${SERVER_LOG}"
mkdir -p "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
set -u

cleanup() {
  if [[ -n "${SERVER_PID:-}" ]] && kill -0 "${SERVER_PID}" 2>/dev/null; then
    kill "${SERVER_PID}" 2>/dev/null || true
    wait "${SERVER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[Azas] Starting PickAndAlign no-motion action server"
ros2 run azas_task_manager pick_and_align_action_server \
  --ros-args \
  -p execution_mode:=no_motion \
  -p grasp_mode:=side \
  -p side_grasp_orientation_source:=parameter \
  -p side_grasp_qx:=0.0 \
  -p side_grasp_qy:=0.0 \
  -p side_grasp_qz:=0.0 \
  -p side_grasp_qw:=1.0 \
  -p tumbler_pose_topic:="${POSE_TOPIC}" \
  -p pose_wait_timeout_sec:=3.0 \
  -p enable_gripper_service_calls:=false \
  >"${SERVER_LOG}" 2>&1 &
SERVER_PID=$!

timeout 12s bash -lc '
  while true; do
    ros2 action list >/tmp/azas_smoke_pick_and_align_actions.txt 2>/tmp/azas_smoke_pick_and_align_actions.err || true
    if grep -qx "/azas/pick_and_align" /tmp/azas_smoke_pick_and_align_actions.txt; then
      exit 0
    fi
    sleep 0.2
  done
'

echo "[Azas] Publishing fake base_link tumbler pose into ${POSE_TOPIC}"
ros2 topic pub --once "${POSE_TOPIC}" geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.32, y: -0.22, z: 0.05},
    orientation: {w: 1.0}
  }
}" >/tmp/azas_smoke_pick_and_align_pose_pub.log

echo "[Azas] Sending no-motion PickAndAlign goal"
ros2 action send_goal /azas/pick_and_align azas_interfaces/action/PickAndAlign "{}" --feedback \
  >"${ACTION_LOG}" 2>&1

for expected in \
  "COMPUTE_SIDE_GRASP" \
  "SIDE_APPROACH_NO_MOTION" \
  "SIDE_PICK_NO_MOTION" \
  "GRIPPER_CLOSE_NO_MOTION" \
  "SIDE_LIFT_NO_MOTION" \
  "DONE_NO_MOTION" \
  "NO_MOTION_SIDE_GRASP_OK"; do
  if ! grep -q "${expected}" "${ACTION_LOG}"; then
    echo "[FAIL] PickAndAlign no-motion action did not report ${expected}"
    echo "--- action log ---"
    sed -n '1,220p' "${ACTION_LOG}" 2>/dev/null || true
    echo "--- server log ---"
    sed -n '1,220p' "${SERVER_LOG}" 2>/dev/null || true
    exit 1
  fi
done

if grep -q "Calling RG2 gripper" "${SERVER_LOG}"; then
  echo "[FAIL] PickAndAlign no-motion attempted an RG2 service call"
  sed -n '1,220p' "${SERVER_LOG}" 2>/dev/null || true
  exit 1
fi

echo "[OK] PickAndAlign side-grasp no-motion action reached DONE_NO_MOTION"
