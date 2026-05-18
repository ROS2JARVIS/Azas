#!/usr/bin/env bash
set -euo pipefail

# End-to-end fake-hardware smoke test:
# fake CupDetection -> pose bridge -> tumbler_floor_place_node with
# enable_hardware=true -> fake Doosan MoveLine + fake RG2 Trigger services.
#
# This sends service requests only to tools/smoke/fake_hardware_services.py.

STATUS_FILE="${STATUS_FILE:-/tmp/azas_smoke_fake_hardware_status.txt}"
LOG_FILE="${LOG_FILE:-/tmp/azas_smoke_fake_hardware_launch.log}"
FAKE_LOG_FILE="${FAKE_LOG_FILE:-/tmp/azas_smoke_fake_hardware_services.log}"
SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID:-2}"
SERVICE_PREFIX="${SERVICE_PREFIX:-}"
SMOKE_CUP_DETECTION_TOPIC="${SMOKE_CUP_DETECTION_TOPIC:-/azas/smoke/fake_hardware_cup_detection}"
SMOKE_TUMBLER_POSE_TOPIC="${SMOKE_TUMBLER_POSE_TOPIC:-/azas/smoke/fake_hardware_tumbler_pose}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"

case "${SELECTED_DISPENSER_ID}" in
  1) EXPECTED_PLACE_Y="180\\.0"; EXPECTED_PLAN_Y="0.180" ;;
  2) EXPECTED_PLACE_Y="80\\.0"; EXPECTED_PLAN_Y="0.080" ;;
  3) EXPECTED_PLACE_Y="-20\\.0"; EXPECTED_PLAN_Y="-0.020" ;;
  4) EXPECTED_PLACE_Y="-120\\.0"; EXPECTED_PLAN_Y="-0.120" ;;
  *)
    echo "[FAIL] unsupported SELECTED_DISPENSER_ID=${SELECTED_DISPENSER_ID}"
    exit 1
    ;;
esac

rm -f "${STATUS_FILE}" "${LOG_FILE}" "${FAKE_LOG_FILE}"
mkdir -p "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
set -u

assert_no_preexisting_fake_targets() {
  ros2 service list --no-daemon >/tmp/azas_smoke_fake_hardware_pre_services.txt 2>/tmp/azas_smoke_fake_hardware_pre_services.err || true
  for service in /jarvis/rg2/open /jarvis/rg2/close /jarvis/rg2/set_width; do
    if grep -qx "${service}" /tmp/azas_smoke_fake_hardware_pre_services.txt; then
      echo "[FAIL] refusing fake smoke: ${service} already exists before fake_hardware_services.py starts"
      echo "[FAIL] This smoke must only talk to the local fake/no-motion RG2 services."
      exit 1
    fi
  done
}

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  if [[ -n "${FAKE_PID:-}" ]] && kill -0 "${FAKE_PID}" 2>/dev/null; then
    kill "${FAKE_PID}" 2>/dev/null || true
    wait "${FAKE_PID}" 2>/dev/null || true
  fi
  if [[ -n "${STATUS_PID:-}" ]] && kill -0 "${STATUS_PID}" 2>/dev/null; then
    kill "${STATUS_PID}" 2>/dev/null || true
    wait "${STATUS_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

assert_log_contains() {
  local file="$1"
  local pattern="$2"
  local description="$3"
  if grep -Eq "$pattern" "${file}"; then
    echo "[OK] ${description}"
    return 0
  fi
  echo "[FAIL] missing expected evidence: ${description}"
  echo "--- ${file} ---"
  sed -n '1,220p' "${file}" 2>/dev/null || true
  exit 1
}

echo "[Azas] Starting fake hardware services"
assert_no_preexisting_fake_targets
if [[ -n "${SERVICE_PREFIX}" ]]; then
  python3 /home/ssu/Azas/tools/smoke/fake_hardware_services.py \
    --ros-args -p service_prefix:="${SERVICE_PREFIX}" \
    >"${FAKE_LOG_FILE}" 2>&1 &
else
  python3 /home/ssu/Azas/tools/smoke/fake_hardware_services.py \
    >"${FAKE_LOG_FILE}" 2>&1 &
fi
FAKE_PID=$!

timeout 12s bash -lc '
  while true; do
    ros2 service list --no-daemon >/tmp/azas_smoke_fake_hardware_services.txt 2>/tmp/azas_smoke_fake_hardware_services.err || true
    if grep -qx "/jarvis/rg2/open" /tmp/azas_smoke_fake_hardware_services.txt &&
       grep -qx "/jarvis/rg2/close" /tmp/azas_smoke_fake_hardware_services.txt &&
       grep -qx "/jarvis/rg2/set_width" /tmp/azas_smoke_fake_hardware_services.txt &&
       grep -q "/motion/move_line\\|/.*/motion/move_line" /tmp/azas_smoke_fake_hardware_services.txt; then
      exit 0
    fi
    sleep 0.2
  done
'

echo "[Azas] Starting hardware-armed floor-place launch against fake services"
LAUNCH_ARGS=(
  selected_dispenser_id:="${SELECTED_DISPENSER_ID}"
  cup_detection_topic:="${SMOKE_CUP_DETECTION_TOPIC}"
  tumbler_pose_topic:="${SMOKE_TUMBLER_POSE_TOPIC}"
  run_yolo:=false
  source_frame:=base_link
  enable_hardware:=true
  hardware_confirm:=ENABLE_REAL_ROBOT_MOTION
  allow_service_control_without_moveit:=true
  gripper_open_service:=/jarvis/rg2/open
  gripper_close_service:=/jarvis/rg2/close
  tumbler_pose_wait_timeout:=8.0
)
if [[ -n "${SERVICE_PREFIX}" ]]; then
  LAUNCH_ARGS+=(service_prefix:="${SERVICE_PREFIX}")
fi

ros2 launch azas_bringup yolo_to_floor_place.launch.py "${LAUNCH_ARGS[@]}" \
  >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

timeout 20s ros2 topic echo /jarvis/tumbler_floor_place/status --field data --no-daemon >"${STATUS_FILE}" &
STATUS_PID=$!

sleep 1
echo "[Azas] Publishing fake CupDetection into ${SMOKE_CUP_DETECTION_TOPIC}"
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
  status: 'detected:upright class=fake_hardware_tumbler',
  source: 'smoke_fake_hardware_path'
}" >/tmp/azas_smoke_fake_hardware_pub.log

echo "[Azas] Waiting for DONE status"
for _ in {1..40}; do
  if grep -q "DONE" "${STATUS_FILE}" 2>/dev/null || grep -q "tumbler_floor_place_node.*DONE" "${LOG_FILE}" 2>/dev/null; then
    assert_log_contains "${LOG_FILE}" "plan side_pre_grasp: x=0\\.238 y=-0\\.163 z=0\\.135 gripper=preopen width_m=0\\.095 force_n=8\\.0" "hardware-armed path preopens for tapered cup before side approach"
    assert_log_contains "${LOG_FILE}" "plan side_grasp_tumbler: x=0\\.320 y=-0\\.220 z=0\\.135 gripper=close width_m=0\\.064 force_n=12\\.0" "hardware-armed path closes to tapered cup target width"
    assert_log_contains "${LOG_FILE}" "plan lift_tumbler: x=0\\.320 y=-0\\.220 z=0\\.175 gripper=none" "hardware-armed path uses slight lift after side grasp"
    assert_log_contains "${LOG_FILE}" "plan pre_floor_place: x=0\\.430 y=${EXPECTED_PLAN_Y} z=0\\.145 gripper=none" "hardware-armed path uses slight place approach clearance"
    assert_log_contains "${LOG_FILE}" "plan floor_place: x=0\\.430 y=${EXPECTED_PLAN_Y} z=0\\.085 gripper=open" "hardware-armed path targets fixed selected-dispenser position"
    assert_log_contains "${FAKE_LOG_FILE}" "fake move_line: pos=\\[(np\\.float64\\()?237\\.[0-9]+\\)?, (np\\.float64\\()?-163\\.[0-9]+\\)?, (np\\.float64\\()?135\\.0" "fake Doosan MoveLine received robot-side radial pre-grasp waypoint in mm"
    assert_log_contains "${FAKE_LOG_FILE}" "fake RG2 set_width: command=preopen width_m=0\\.095 force_n=8\\.0" "fake RG2 received tapered preopen width command"
    assert_log_contains "${FAKE_LOG_FILE}" "fake RG2 set_width: command=grasp width_m=0\\.064 force_n=12\\.0" "fake RG2 received tapered grasp width command"
    assert_log_contains "${FAKE_LOG_FILE}" "fake move_line: pos=\\[(np\\.float64\\()?320\\.0\\)?, (np\\.float64\\()?-220\\.0\\)?, (np\\.float64\\()?175\\.0" "fake Doosan MoveLine received slight-lift waypoint in mm"
    assert_log_contains "${FAKE_LOG_FILE}" "fake move_line: pos=\\[(np\\.float64\\()?430\\.0\\)?, (np\\.float64\\()?${EXPECTED_PLACE_Y}\\)?, (np\\.float64\\()?85\\.0" "fake Doosan MoveLine received fixed floor-place waypoint in mm"
    echo "[OK] fake hardware path reached DONE"
    exit 0
  fi
  if grep -q "FAILED" "${STATUS_FILE}" 2>/dev/null; then
    echo "[FAIL] fake hardware path reached FAILED"
    sed -n '1,160p' "${LOG_FILE}"
    sed -n '1,160p' "${FAKE_LOG_FILE}"
    exit 1
  fi
  sleep 0.5
done

echo "[FAIL] fake hardware path did not reach DONE"
echo "--- status ---"
sed -n '1,120p' "${STATUS_FILE}" 2>/dev/null || true
echo "--- launch log ---"
sed -n '1,180p' "${LOG_FILE}" 2>/dev/null || true
echo "--- fake service log ---"
sed -n '1,180p' "${FAKE_LOG_FILE}" 2>/dev/null || true
exit 1
