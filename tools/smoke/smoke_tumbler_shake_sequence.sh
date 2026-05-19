#!/usr/bin/env bash
set -euo pipefail

# Fake-hardware high-shake smoke test:
# tumbler_shake_sequence_node with enable_hardware=true -> fake Doosan MoveJoint.
#
# This verifies the hardware-armed code path without commanding real hardware.

STATUS_FILE="${STATUS_FILE:-/tmp/azas_smoke_tumbler_shake_status.txt}"
LOG_FILE="${LOG_FILE:-/tmp/azas_smoke_tumbler_shake_launch.log}"
FAKE_LOG_FILE="${FAKE_LOG_FILE:-/tmp/azas_smoke_tumbler_shake_fake_services.log}"
SERVICE_PREFIX="${SERVICE_PREFIX:-}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"

rm -f "${STATUS_FILE}" "${LOG_FILE}" "${FAKE_LOG_FILE}"
mkdir -p "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
set -u

assert_no_preexisting_motion_target() {
  local prefix="${SERVICE_PREFIX#/}"
  prefix="${prefix%/}"
  local motion_service="/motion/move_joint"
  if [[ -n "${prefix}" ]]; then
    motion_service="/${prefix}/motion/move_joint"
  fi

  for _ in {1..20}; do
    ros2 service list --no-daemon >/tmp/azas_smoke_tumbler_shake_pre_services.txt 2>/tmp/azas_smoke_tumbler_shake_pre_services.err || true
    if ! grep -qx "${motion_service}" /tmp/azas_smoke_tumbler_shake_pre_services.txt; then
      return 0
    fi
    sleep 0.5
  done

  echo "[FAIL] refusing fake shake smoke: ${motion_service} already exists before fake_hardware_services.py starts"
  echo "[FAIL] This smoke must only talk to the local fake/no-motion Doosan service."
  exit 1
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
  if [[ -n "${BAD_LAUNCH_PID:-}" ]] && kill -0 "${BAD_LAUNCH_PID}" 2>/dev/null; then
    kill "${BAD_LAUNCH_PID}" 2>/dev/null || true
    wait "${BAD_LAUNCH_PID}" 2>/dev/null || true
  fi
  if [[ -n "${BAD_STATUS_PID:-}" ]] && kill -0 "${BAD_STATUS_PID}" 2>/dev/null; then
    kill "${BAD_STATUS_PID}" 2>/dev/null || true
    wait "${BAD_STATUS_PID}" 2>/dev/null || true
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

echo "[Azas] Starting fake motion service for high-shake smoke"
assert_no_preexisting_motion_target
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
    ros2 service list --no-daemon >/tmp/azas_smoke_tumbler_shake_services.txt 2>/tmp/azas_smoke_tumbler_shake_services.err || true
    if grep -q "/motion/move_joint\\|/.*/motion/move_joint" /tmp/azas_smoke_tumbler_shake_services.txt; then
      exit 0
    fi
    sleep 0.2
  done
'

echo "[Azas] Starting hardware-armed high-shake launch against fake service"
LAUNCH_ARGS=(
  enable_hardware:=true
  hardware_confirm:=ENABLE_REAL_ROBOT_MOTION
  allow_service_control_without_moveit:=true
  use_visualizer:=false
  shake_control_mode:=joint
  shake_cycles:=4
  joint_shake_base_j1_deg:=0.0
  joint_shake_base_j2_deg:=-35.0
  joint_shake_base_j3_deg:=50.0
  joint_shake_base_j4_deg:=0.0
  joint_shake_base_j5_deg:=70.0
  joint_shake_base_j6_deg:=0.0
  joint_shake_j3_amplitude_deg:=0.0
  joint_shake_j4_amplitude_deg:=18.0
  joint_shake_j5_amplitude_deg:=30.0
  joint_shake_j6_amplitude_deg:=36.0
  joint_shake_j1_min_deg:=-20.0
  joint_shake_j1_max_deg:=5.0
  joint_shake_j2_min_deg:=-80.0
  joint_shake_j2_max_deg:=5.0
  joint_shake_j3_min_deg:=0.0
  joint_shake_j3_max_deg:=135.0
  joint_shake_max_single_delta_deg:=75.0
  joint5_min_deg:=40.0
  joint5_max_deg:=100.0
  require_state_validity_for_joint_shake:=false
)
if [[ -n "${SERVICE_PREFIX}" ]]; then
  LAUNCH_ARGS+=(service_prefix:="${SERVICE_PREFIX}")
fi

ros2 launch azas_bringup tumbler_shake_sequence.launch.py "${LAUNCH_ARGS[@]}" \
  >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

timeout 20s ros2 topic echo /jarvis/tumbler_shake_sequence/status --field data --no-daemon >"${STATUS_FILE}" &
STATUS_PID=$!

echo "[Azas] Waiting for high-shake DONE status"
DONE_OK=false
for _ in {1..50}; do
  if grep -q "DONE" "${STATUS_FILE}" 2>/dev/null || grep -q "tumbler_shake_sequence_node.*DONE" "${LOG_FILE}" 2>/dev/null; then
    assert_log_contains "${LOG_FILE}" "Joint shake safety validated: .*joint_5 range=\\[40\\.0, 100\\.0\\]" "joint-space shake passed joint_5 safety validation"
    assert_log_contains "${LOG_FILE}" "plan joint_shake_safe_ready: joints_deg=\\[0\\.0, -35\\.0, 50\\.0, 0\\.0, 70\\.0, 0\\.0\\]" "joint shake starts from J3-positive safe base"
    assert_log_contains "${LOG_FILE}" "plan joint_shake_cycle_1_j5_j6_plus: joints_deg=\\[0\\.0, -35\\.0, 50\\.0, -18\\.0, 100\\.0, 36\\.0\\]" "joint shake drives J5/J6 positive dynamically without moving J3 negative"
    assert_log_contains "${LOG_FILE}" "plan joint_shake_cycle_1_j5_j6_minus: joints_deg=\\[0\\.0, -35\\.0, 50\\.0, 18\\.0, 40\\.0, -36\\.0\\]" "joint shake drives J5/J6 negative dynamically without moving J3 negative"
    assert_log_contains "${FAKE_LOG_FILE}" "fake move_joint: pos=.*0\\.0.*-35\\.0.*50\\.0.*0\\.0.*70\\.0.*0\\.0" "fake Doosan received J3-positive joint safe base"
    assert_log_contains "${FAKE_LOG_FILE}" "fake move_joint: pos=.*0\\.0.*-35\\.0.*50\\.0.*-18\\.0.*100\\.0.*36\\.0" "fake Doosan received dynamic J5/J6 plus waypoint"
    assert_log_contains "${FAKE_LOG_FILE}" "fake move_joint: pos=.*0\\.0.*-35\\.0.*50\\.0.*18\\.0.*40\\.0.*-36\\.0" "fake Doosan received dynamic J5/J6 minus waypoint"
    echo "[OK] high-shake fake hardware path reached DONE"
    DONE_OK=true
    break
  fi
  if grep -q "FAILED" "${STATUS_FILE}" 2>/dev/null; then
    echo "[FAIL] high-shake path reached FAILED"
    sed -n '1,180p' "${LOG_FILE}" 2>/dev/null || true
    sed -n '1,180p' "${FAKE_LOG_FILE}" 2>/dev/null || true
    exit 1
  fi
  sleep 0.5
done

if [[ "${DONE_OK}" != "true" ]]; then
  echo "[FAIL] high-shake path did not reach DONE"
  echo "--- status ---"
  sed -n '1,120p' "${STATUS_FILE}" 2>/dev/null || true
  echo "--- launch log ---"
  sed -n '1,220p' "${LOG_FILE}" 2>/dev/null || true
  echo "--- fake service log ---"
  sed -n '1,220p' "${FAKE_LOG_FILE}" 2>/dev/null || true
  exit 1
fi

kill "${LAUNCH_PID}" 2>/dev/null || true
wait "${LAUNCH_PID}" 2>/dev/null || true
kill "${STATUS_PID}" 2>/dev/null || true
wait "${STATUS_PID}" 2>/dev/null || true

BAD_LOG_FILE="${BAD_LOG_FILE:-/tmp/azas_smoke_tumbler_shake_bad_launch.log}"
BAD_STATUS_FILE="${BAD_STATUS_FILE:-/tmp/azas_smoke_tumbler_shake_bad_status.txt}"
rm -f "${BAD_LOG_FILE}" "${BAD_STATUS_FILE}"

echo "[Azas] Checking that unsafe shake space fails closed"
ros2 launch azas_bringup tumbler_shake_sequence.launch.py \
  use_visualizer:=false \
  shake_control_mode:=joint \
  joint_shake_base_j1_deg:=30.0 \
  >"${BAD_LOG_FILE}" 2>&1 &
BAD_LAUNCH_PID=$!

timeout 12s ros2 topic echo /jarvis/tumbler_shake_sequence/status --field data --no-daemon >"${BAD_STATUS_FILE}" 2>/tmp/azas_smoke_tumbler_shake_bad_topic.err &
BAD_STATUS_PID=$!

for _ in {1..24}; do
  if grep -q "FAILED" "${BAD_STATUS_FILE}" 2>/dev/null || grep -q "tumbler_shake_sequence_node.*FAILED" "${BAD_LOG_FILE}" 2>/dev/null; then
    assert_log_contains "${BAD_LOG_FILE}" "joint_1=.*outside safe shake range" "unsafe joint_1 shake plan fails closed"
    echo "[OK] unsafe shake space was rejected"
    exit 0
  fi
  sleep 0.5
done

echo "[FAIL] unsafe shake space did not fail closed"
sed -n '1,180p' "${BAD_LOG_FILE}" 2>/dev/null || true
exit 1
