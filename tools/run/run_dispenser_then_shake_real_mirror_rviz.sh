#!/usr/bin/env bash
set -euo pipefail

# RViz preview that mirrors the real dispenser-then-shake execution path.
# It intentionally does NOT publish fake /joint_states. If the robot model moves
# in RViz, that movement comes from the connected real/virtual Doosan driver.
# With no robot connected, this shows the exact dry-run Path messages produced
# by the same nodes used by the real script.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID:-2}"
USE_DEMO_CUP_POSE="${USE_DEMO_CUP_POSE:-true}"
START_RVIZ="${START_RVIZ:-true}"
START_ROBOT_DESCRIPTION="${START_ROBOT_DESCRIPTION:-false}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${ROOT_DIR}/src/azas_bringup/rviz/azas_real_mirror_dispenser.rviz}"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/log/manual}"
GRASP_X="${GRASP_X:-0.42}"
GRASP_Y="${GRASP_Y:--0.24}"
GRASP_Z="${GRASP_Z:-0.05}"
MOUTH_X="${MOUTH_X:-${GRASP_X}}"
MOUTH_Y="${MOUTH_Y:-${GRASP_Y}}"
MOUTH_Z="${MOUTH_Z:-0.22}"
SHAKE_CENTER_X="${SHAKE_CENTER_X:-0.28}"
SHAKE_CENTER_Y="${SHAKE_CENTER_Y:--0.30}"
SHAKE_CENTER_Z="${SHAKE_CENTER_Z:-0.62}"
SHAKE_AMPLITUDE_X="${SHAKE_AMPLITUDE_X:-0.100}"
SHAKE_AMPLITUDE_Y="${SHAKE_AMPLITUDE_Y:-0.040}"
SHAKE_AMPLITUDE_Z="${SHAKE_AMPLITUDE_Z:-0.055}"
SHAKE_CYCLES="${SHAKE_CYCLES:-4}"
SHAKE_TWIST_RX_DEG="${SHAKE_TWIST_RX_DEG:-6.0}"
SHAKE_TWIST_RZ_DEG="${SHAKE_TWIST_RZ_DEG:-22.0}"
APPROACH_LINE_TIME="${APPROACH_LINE_TIME:-3.5}"
SHAKE_LINE_TIME="${SHAKE_LINE_TIME:-0.40}"
MIN_SHAKE_Z="${MIN_SHAKE_Z:-0.55}"
DISPENSER_KEEPOUT_RADIUS="${DISPENSER_KEEPOUT_RADIUS:-0.20}"
# 교안식 검증: 실제/가상 Doosan controller service에 명령을 넣고,
# driver가 내보내는 /joint_states로 RViz가 움직이게 한다.
EXECUTE_CONTROLLER_MOTION="${EXECUTE_CONTROLLER_MOTION:-true}"
# Default to the verified virtual Doosan namespace. Use SERVICE_PREFIX=dsr01
# explicitly only when the real robot/session is intentionally armed.
SERVICE_PREFIX="${SERVICE_PREFIX:-azasvirt}"
HARDWARE_CONFIRM="${HARDWARE_CONFIRM:-ENABLE_REAL_ROBOT_MOTION}"
DISABLE_GRIPPER_COMMANDS="${DISABLE_GRIPPER_COMMANDS:-true}"
MOTION_RESPONSE_TIMEOUT_SEC="${MOTION_RESPONSE_TIMEOUT_SEC:-60.0}"
SHAKE_CONTROL_MODE="${SHAKE_CONTROL_MODE:-joint}"
VERIFY_JOINT_TARGETS="${VERIFY_JOINT_TARGETS:-false}"
SHAKE_JOINT_VELOCITY="${SHAKE_JOINT_VELOCITY:-90.0}"
SHAKE_JOINT_ACCELERATION="${SHAKE_JOINT_ACCELERATION:-140.0}"

mkdir -p "${LOG_DIR}"

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT
PIDS=()

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

if [[ "${EXECUTE_CONTROLLER_MOTION}" == "true" ]]; then
  missing=0
  service_list="$(ros2 service list --no-daemon || true)"
  for service in "/${SERVICE_PREFIX}/motion/move_line" "/${SERVICE_PREFIX}/motion/move_joint"; do
    if ! grep -qx "${service}" <<<"${service_list}"; then
      echo "[Azas] Missing controller service: ${service}" >&2
      missing=1
    fi
  done
  if [[ "${missing}" != "0" ]]; then
    echo "[Azas] Refusing controller-motion mirror. Start virtual Doosan first:" >&2
    echo "  ROBOT_NAME=${SERVICE_PREFIX} bash tools/run/run_doosan_virtual_m0609.sh" >&2
    exit 1
  fi
fi

if [[ "${START_ROBOT_DESCRIPTION}" == "true" ]]; then
  DSR_DESCRIPTION_PREFIX="$(ros2 pkg prefix dsr_description2)"
  DSR_XACRO="${DSR_DESCRIPTION_PREFIX}/share/dsr_description2/xacro/m0609.urdf.xacro"
  ROBOT_URDF="${LOG_DIR}/real_mirror_m0609.urdf"
  xacro "${DSR_XACRO}" color:=white simple:=true >"${ROBOT_URDF}"
  ros2 run robot_state_publisher robot_state_publisher "${ROBOT_URDF}" \
    >"${LOG_DIR}/real_mirror_robot_state_publisher.log" 2>&1 &
  PIDS+=("$!")
fi

if [[ "${START_RVIZ}" == "true" ]]; then
  rviz2 -d "${RVIZ_CONFIG}" >"${LOG_DIR}/real_mirror_rviz.log" 2>&1 &
  PIDS+=("$!")
fi

if [[ "${USE_DEMO_CUP_POSE}" == "true" ]]; then
  # Demo source only replaces the camera detection input. The motion nodes below
  # are still the same nodes used by real execution. Controller-motion mode
  # sends commands to the virtual Doosan services, not to RViz-only joints.
  ros2 launch azas_bringup hardware_free_demo.launch.py \
    use_rviz:=false \
    use_robot_urdf:=false \
    enable_ik_preview:=false \
    run_live_stt:=false \
    run_recipe_mapper:=false \
    use_llm:=false \
    show_sequence_markers:=false \
    show_dispenser_markers:=false \
    show_animated_cup:=false \
    show_demo_arm:=false \
    selected_dispenser_id:="${SELECTED_DISPENSER_ID}" \
    grasp_x:="${GRASP_X}" \
    grasp_y:="${GRASP_Y}" \
    grasp_z:="${GRASP_Z}" \
    mouth_x:="${MOUTH_X}" \
    mouth_y:="${MOUTH_Y}" \
    mouth_z:="${MOUTH_Z}" \
    >"${LOG_DIR}/real_mirror_demo_pose.log" 2>&1 &
  PIDS+=("$!")
  TUMBLER_POSE_TOPIC="/azas/demo/tumbler_pose"
else
  TUMBLER_POSE_TOPIC="/jarvis/tumbler_dispenser/tumbler_pose"
fi

if [[ "${EXECUTE_CONTROLLER_MOTION}" == "true" ]]; then
  FLOOR_ENABLE_HARDWARE=true
  FLOOR_ALLOW_SERVICE=true
else
  FLOOR_ENABLE_HARDWARE=false
  FLOOR_ALLOW_SERVICE=false
fi

ros2 launch azas_bringup tumbler_floor_place.launch.py \
  selected_dispenser_id:="${SELECTED_DISPENSER_ID}" \
  delivery_mode:=hold_under_outlet \
  execution_stage:=full \
  use_tumbler_pose_topic:=true \
  tumbler_pose_topic:="${TUMBLER_POSE_TOPIC}" \
  enable_hardware:="${FLOOR_ENABLE_HARDWARE}" \
  hardware_confirm:="${HARDWARE_CONFIRM}" \
  allow_service_control_without_moveit:="${FLOOR_ALLOW_SERVICE}" \
  service_prefix:="${SERVICE_PREFIX}" \
  disable_gripper_commands:="${DISABLE_GRIPPER_COMMANDS}" \
  motion_response_timeout_sec:="${MOTION_RESPONSE_TIMEOUT_SEC}" \
  allow_demo_tumbler_position_fallback:=false \
  >"${LOG_DIR}/real_mirror_floor_place.log" 2>&1 &
FLOOR_PID="$!"
PIDS+=("${FLOOR_PID}")

# 교안 원칙: one controller trajectory at a time. Wait for the dispenser
# transfer stage to finish before sending the shake sequence.
if [[ "${EXECUTE_CONTROLLER_MOTION}" == "true" ]]; then
  floor_deadline=$((SECONDS + 120))
  while (( SECONDS < floor_deadline )); do
    if grep -q "DONE" "${LOG_DIR}/real_mirror_floor_place.log" 2>/dev/null; then
      break
    fi
    if grep -q "FAILED\|REJECTED\|STALE" "${LOG_DIR}/real_mirror_floor_place.log" 2>/dev/null; then
      echo "[Azas] Floor/dispenser transfer failed; not starting shake." >&2
      exit 1
    fi
    sleep 0.5
  done
  if ! grep -q "DONE" "${LOG_DIR}/real_mirror_floor_place.log" 2>/dev/null; then
    echo "[Azas] Floor/dispenser transfer did not finish before timeout; not starting shake." >&2
    exit 1
  fi
else
  sleep 4
fi

if [[ "${EXECUTE_CONTROLLER_MOTION}" == "true" ]]; then
  SHAKE_ENABLE_HARDWARE=true
  SHAKE_ALLOW_SERVICE=true
else
  SHAKE_ENABLE_HARDWARE=false
  SHAKE_ALLOW_SERVICE=false
fi

ros2 launch azas_bringup tumbler_shake_sequence.launch.py \
  enable_hardware:="${SHAKE_ENABLE_HARDWARE}" \
  hardware_confirm:="${HARDWARE_CONFIRM}" \
  allow_service_control_without_moveit:="${SHAKE_ALLOW_SERVICE}" \
  service_prefix:="${SERVICE_PREFIX}" \
  shake_control_mode:="${SHAKE_CONTROL_MODE}" \
  verify_joint_targets:="${VERIFY_JOINT_TARGETS}" \
  motion_response_timeout_sec:="${MOTION_RESPONSE_TIMEOUT_SEC}" \
  shake_joint_velocity:="${SHAKE_JOINT_VELOCITY}" \
  shake_joint_acceleration:="${SHAKE_JOINT_ACCELERATION}" \
  use_visualizer:=false \
  shake_center_x:="${SHAKE_CENTER_X}" \
  shake_center_y:="${SHAKE_CENTER_Y}" \
  shake_center_z:="${SHAKE_CENTER_Z}" \
  shake_amplitude_x:="${SHAKE_AMPLITUDE_X}" \
  shake_amplitude_y:="${SHAKE_AMPLITUDE_Y}" \
  shake_amplitude_z:="${SHAKE_AMPLITUDE_Z}" \
  shake_cycles:="${SHAKE_CYCLES}" \
  shake_twist_rx_deg:="${SHAKE_TWIST_RX_DEG}" \
  shake_twist_rz_deg:="${SHAKE_TWIST_RZ_DEG}" \
  approach_line_time:="${APPROACH_LINE_TIME}" \
  shake_line_time:="${SHAKE_LINE_TIME}" \
  min_shake_z:="${MIN_SHAKE_Z}" \
  dispenser_keepout_radius:="${DISPENSER_KEEPOUT_RADIUS}" \
  >"${LOG_DIR}/real_mirror_shake.log" 2>&1 &
SHAKE_PID="$!"
PIDS+=("${SHAKE_PID}")

echo "[Azas] real-mirror RViz is running."
echo "[Azas] No fake joint animation is active. Robot movement in RViz must come from controller/driver /joint_states."
echo "[Azas] EXECUTE_CONTROLLER_MOTION=${EXECUTE_CONTROLLER_MOTION} SERVICE_PREFIX=${SERVICE_PREFIX} SHAKE_CONTROL_MODE=${SHAKE_CONTROL_MODE}"
echo "[Azas] Plans: /jarvis/tumbler_floor_place/plan and /jarvis/tumbler_shake_sequence/plan"
echo "[Azas] Logs: ${LOG_DIR}/real_mirror_*.log"
if [[ "${START_RVIZ}" == "true" ]]; then
  wait
else
  shake_deadline=$((SECONDS + 120))
  while (( SECONDS < shake_deadline )); do
    if grep -q "DONE" "${LOG_DIR}/real_mirror_shake.log" 2>/dev/null; then
      exit 0
    fi
    if grep -q "FAILED\|REJECTED\|STALE" "${LOG_DIR}/real_mirror_shake.log" 2>/dev/null; then
      echo "[Azas] Shake sequence failed." >&2
      exit 1
    fi
    sleep 0.5
  done
  echo "[Azas] Shake sequence did not finish before timeout." >&2
  exit 1
fi
