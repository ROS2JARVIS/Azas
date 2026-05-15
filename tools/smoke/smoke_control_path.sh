#!/usr/bin/env bash
set -euo pipefail

# End-to-end non-hardware smoke test:
# CupDetection -> cup_detection_pose_bridge_node -> jarvis tumbler pose topic
# -> tumbler_floor_place_node -> DONE status.
#
# This does not start YOLO, RealSense, RG2, or real Doosan motion.

STATUS_FILE="${STATUS_FILE:-/tmp/azas_smoke_control_path_status.txt}"
LOG_FILE="${LOG_FILE:-/tmp/azas_smoke_control_path_launch.log}"
SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID:-2}"
SMOKE_CUP_DETECTION_TOPIC="${SMOKE_CUP_DETECTION_TOPIC:-/azas/smoke/cup_detection}"
SMOKE_TUMBLER_POSE_TOPIC="${SMOKE_TUMBLER_POSE_TOPIC:-/azas/smoke/tumbler_pose}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"

case "${SELECTED_DISPENSER_ID}" in
  1) EXPECTED_PLACE_Y="0.180" ;;
  2) EXPECTED_PLACE_Y="0.080" ;;
  3) EXPECTED_PLACE_Y="-0.020" ;;
  4) EXPECTED_PLACE_Y="-0.120" ;;
  *)
    echo "[FAIL] unsupported SELECTED_DISPENSER_ID=${SELECTED_DISPENSER_ID}"
    exit 1
    ;;
esac

rm -f "${STATUS_FILE}" "${LOG_FILE}"
mkdir -p "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
set -u

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  if [[ -n "${STATUS_PID:-}" ]] && kill -0 "${STATUS_PID}" 2>/dev/null; then
    kill "${STATUS_PID}" 2>/dev/null || true
    wait "${STATUS_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

assert_log_contains() {
  local pattern="$1"
  local description="$2"
  if grep -Eq "$pattern" "${LOG_FILE}"; then
    echo "[OK] ${description}"
    return 0
  fi
  echo "[FAIL] missing expected plan evidence: ${description}"
  echo "--- launch log ---"
  sed -n '1,220p' "${LOG_FILE}" 2>/dev/null || true
  exit 1
}

echo "[Azas] Starting non-hardware smoke launch"
ros2 launch azas_bringup yolo_to_floor_place.launch.py \
  selected_dispenser_id:="${SELECTED_DISPENSER_ID}" \
  cup_detection_topic:="${SMOKE_CUP_DETECTION_TOPIC}" \
  tumbler_pose_topic:="${SMOKE_TUMBLER_POSE_TOPIC}" \
  run_yolo:=false \
  source_frame:=base_link \
  enable_hardware:=false \
  allow_service_control_without_moveit:=false \
  tumbler_pose_wait_timeout:=8.0 \
  >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

echo "[Azas] Waiting for floor-place status topic"
timeout 12s bash -lc '
  while true; do
    ros2 topic list --no-daemon >/tmp/azas_smoke_control_path_topics.txt 2>/tmp/azas_smoke_control_path_topics.err || true
    if grep -qx "/jarvis/tumbler_floor_place/status" /tmp/azas_smoke_control_path_topics.txt; then
      exit 0
    fi
    sleep 0.2
  done
'

timeout 20s ros2 topic echo /jarvis/tumbler_floor_place/status --field data --no-daemon >"${STATUS_FILE}" &
STATUS_PID=$!

echo "[Azas] Publishing fake CupDetection into ${SMOKE_CUP_DETECTION_TOPIC}"
sleep 1
ros2 topic pub --once "${SMOKE_CUP_DETECTION_TOPIC}" azas_interfaces/msg/CupDetection "{
  header: {frame_id: 'base_link'},
  grasp_pose: {
    position: {x: 0.32, y: -0.22, z: 0.05},
    orientation: {w: 1.0}
  },
  cup_mouth_center: {
    position: {x: 0.32, y: -0.22, z: 0.22},
    orientation: {w: 1.0}
  },
  confidence: 0.95,
  status: 'detected:upright class=smoke_tumbler',
  source: 'smoke_control_path'
}" >/tmp/azas_smoke_control_path_pub.log

echo "[Azas] Waiting for DONE status"
for _ in {1..30}; do
  if grep -q "DONE" "${STATUS_FILE}" 2>/dev/null; then
    assert_log_contains "plan side_pre_grasp: x=0\\.238 y=-0\\.163 z=0\\.135 gripper=preopen width_m=0\\.095 force_n=8\\.0" "gripper preopens for tapered cup before side approach"
    assert_log_contains "plan side_grasp_tumbler: x=0\\.320 y=-0\\.220 z=0\\.135 gripper=close width_m=0\\.064 force_n=12\\.0" "grasp closes to tapered cup target width"
    assert_log_contains "plan lift_tumbler: x=0\\.320 y=-0\\.220 z=0\\.175 gripper=none" "side-grasped cup is lifted only slightly"
    assert_log_contains "plan pre_floor_place: x=0\\.430 y=${EXPECTED_PLACE_Y} z=0\\.145 gripper=none" "cup approaches fixed place point at slight clearance"
    assert_log_contains "plan floor_place: x=0\\.430 y=${EXPECTED_PLACE_Y} z=0\\.085 gripper=open" "cup moves to fixed selected-dispenser position"
    echo "[OK] smoke control path reached DONE"
    exit 0
  fi
  if grep -q "FAILED" "${STATUS_FILE}" 2>/dev/null; then
    echo "[FAIL] smoke control path reached FAILED"
    sed -n '1,120p' "${LOG_FILE}"
    exit 1
  fi
  sleep 0.5
done

echo "[FAIL] smoke control path did not reach DONE"
echo "--- status ---"
sed -n '1,120p' "${STATUS_FILE}" 2>/dev/null || true
echo "--- launch log ---"
sed -n '1,160p' "${LOG_FILE}" 2>/dev/null || true
exit 1
