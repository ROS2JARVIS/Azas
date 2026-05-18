#!/usr/bin/env bash
set -euo pipefail

# Fake-hardware smoke for the dispenser press primitive used after cup placement.
# This never talks to real hardware: MoveLine requests go only to
# tools/smoke/fake_hardware_services.py.

STATUS_FILE="${STATUS_FILE:-/tmp/azas_smoke_dispenser_press_status.txt}"
LOG_FILE="${LOG_FILE:-/tmp/azas_smoke_dispenser_press_launch.log}"
FAKE_LOG_FILE="${FAKE_LOG_FILE:-/tmp/azas_smoke_dispenser_press_services.log}"
SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID:-2}"
SERVICE_PREFIX="${SERVICE_PREFIX:-azas_verify_press}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"

case "${SELECTED_DISPENSER_ID}" in
  1) EXPECTED_Y="0.180"; EXPECTED_Y_MM="180\\.0" ;;
  2) EXPECTED_Y="0.080"; EXPECTED_Y_MM="80\\.0" ;;
  3) EXPECTED_Y="-0.020"; EXPECTED_Y_MM="-20\\.0" ;;
  4) EXPECTED_Y="-0.120"; EXPECTED_Y_MM="-120\\.0" ;;
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
source /home/ssu/ros2_ws/install/setup.bash
set -u

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

echo "[Azas] Starting fake hardware services for dispenser press"
python3 /home/ssu/Azas/tools/smoke/fake_hardware_services.py \
  --ros-args -p service_prefix:="${SERVICE_PREFIX}" \
  >"${FAKE_LOG_FILE}" 2>&1 &
FAKE_PID=$!

timeout 12s bash -lc '
  while true; do
    ros2 service list --no-daemon >/tmp/azas_smoke_dispenser_press_services.txt 2>/tmp/azas_smoke_dispenser_press_services.err || true
    if grep -qx "/'"${SERVICE_PREFIX}"'/motion/move_line" /tmp/azas_smoke_dispenser_press_services.txt; then
      exit 0
    fi
    sleep 0.2
  done
'

echo "[Azas] Starting hardware-armed dispenser press against fake MoveLine"
ros2 launch jarvis dispense_lid_sequence.launch.py \
  selected_dispenser_id:="${SELECTED_DISPENSER_ID}" \
  enable_hardware:=true \
  hardware_confirm:=ENABLE_REAL_ROBOT_MOTION \
  allow_service_control_without_moveit:=true \
  service_prefix:="${SERVICE_PREFIX}" \
  execution_stage:=press \
  >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

timeout 20s ros2 topic echo /jarvis/dispense_lid_sequence/status --field data --no-daemon >"${STATUS_FILE}" &
STATUS_PID=$!

echo "[Azas] Waiting for dispenser press DONE status"
for _ in {1..40}; do
  if grep -q "DONE" "${STATUS_FILE}" 2>/dev/null || grep -q "dispense_lid_sequence_node.*DONE" "${LOG_FILE}" 2>/dev/null; then
    assert_log_contains "${LOG_FILE}" "execution_stage=press: generated 4 of 7 sequence steps" "press stage excludes lid-close waypoints"
    assert_log_contains "${LOG_FILE}" "plan extended_press_approach: x=0\\.510 y=${EXPECTED_Y} z=0\\.472" "press approach uses selected current outlet geometry"
    assert_log_contains "${LOG_FILE}" "plan extended_press_top: x=0\\.510 y=${EXPECTED_Y} z=0\\.392" "press top aligns with selected outlet"
    assert_log_contains "${LOG_FILE}" "plan extended_press_down: x=0\\.510 y=${EXPECTED_Y} z=0\\.367" "press down applies configured depth"
    assert_log_contains "${FAKE_LOG_FILE}" "fake move_line: pos=\\[(np\\.float64\\()?510\\.0\\)?, (np\\.float64\\()?${EXPECTED_Y_MM}\\)?, (np\\.float64\\()?367\\.0" "fake Doosan MoveLine received dispenser press-down waypoint in mm"
    echo "[OK] dispenser press fake hardware path reached DONE"
    exit 0
  fi
  if grep -q "FAILED" "${STATUS_FILE}" 2>/dev/null || grep -q "FAILED" "${LOG_FILE}" 2>/dev/null; then
    echo "[FAIL] dispenser press path reached FAILED"
    sed -n '1,180p' "${LOG_FILE}"
    sed -n '1,180p' "${FAKE_LOG_FILE}"
    exit 1
  fi
  sleep 0.5
done

echo "[FAIL] dispenser press path did not reach DONE"
echo "--- status ---"
sed -n '1,120p' "${STATUS_FILE}" 2>/dev/null || true
echo "--- launch log ---"
sed -n '1,220p' "${LOG_FILE}" 2>/dev/null || true
echo "--- fake service log ---"
sed -n '1,220p' "${FAKE_LOG_FILE}" 2>/dev/null || true
exit 1
