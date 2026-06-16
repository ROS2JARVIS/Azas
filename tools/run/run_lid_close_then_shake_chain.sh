#!/usr/bin/env bash
set -euo pipefail

# 디스펜서 레시피 완료 후 체인: 뚜껑 닫기(ArUco 성공 감시) -> 컵홀더 재픽업 -> 쉐이킹 -> 카메라 포즈 복귀.
# robot_pipeline_control_server.py chain_shake_after_lid_command()가 생성하는 패널 체인과 동일한 흐름이다.

ROOT="${ROOT:-/home/ssu/Azas}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-9}"
export ROS_LOCALHOST_ONLY="${CHAIN_ROS_LOCALHOST_ONLY:-${ROS_LOCALHOST_ONLY:-1}}"
export LID_ROS_LOCALHOST_ONLY="${LID_ROS_LOCALHOST_ONLY:-${ROS_LOCALHOST_ONLY}}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export LID_TCP_GRASP_OFFSET_Z_M="${LID_TCP_GRASP_OFFSET_Z_M:--0.032}"
export SERVICE_PREFIX="${SERVICE_PREFIX:-dsr01}"
export LID_GRIP_STATUS_TIMEOUT_SEC="${LID_GRIP_STATUS_TIMEOUT_SEC:-240}"

lid_pid=""
wait_pid=""

cleanup_lid_processes() {
  if [[ -n "${wait_pid}" ]] && kill -0 "${wait_pid}" 2>/dev/null; then
    kill -TERM "${wait_pid}" 2>/dev/null || true
    wait "${wait_pid}" 2>/dev/null || true
  fi
  if [[ -n "${lid_pid}" ]] && kill -0 "${lid_pid}" 2>/dev/null; then
    kill -TERM "${lid_pid}" 2>/dev/null || true
    wait "${lid_pid}" 2>/dev/null || true
  fi
}

source_ros() {
  cd "${ROOT}"
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f /home/ssu/ws_moveit/install/setup.bash ]]; then
    source /home/ssu/ws_moveit/install/setup.bash
  fi
  if [[ -f /home/ssu/ros2_ws/install/setup.bash ]]; then
    source /home/ssu/ros2_ws/install/setup.bash
  fi
  if [[ -f "${ROOT}/install/setup.bash" ]]; then
    source "${ROOT}/install/setup.bash"
  else
    source "${ROOT}/install/local_setup.bash"
  fi
  set -u

  mkdir -p /tmp/azas_ros_logs
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
  export PYTHONPATH="${ROOT}/tools/run/python_compat:${PYTHONPATH:-}"
}

trap cleanup_lid_processes EXIT

(
  cd "${ROOT}"
  SERVICE_PREFIX="${SERVICE_PREFIX}" \
  DISPLAY="${DISPLAY:-:0}" \
  XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}" \
  MOVE_TO_LID_VIEW_POSE=true \
  bash "${ROOT}/tools/run/run_kang_lid_grip_close_direct.sh"
) &
lid_pid=$!

(
  source_ros
  python3 "${ROOT}/tools/run/wait_for_lid_grip_status.py" \
    --timeout-sec "${LID_GRIP_STATUS_TIMEOUT_SEC}" \
    --success-status motion_sequence_requested
) &
wait_pid=$!

while true; do
  if ! kill -0 "${wait_pid}" 2>/dev/null; then
    if wait "${wait_pid}"; then
      wait_rc=0
    else
      wait_rc=$?
    fi
    wait_pid=""
    break
  fi

  if ! kill -0 "${lid_pid}" 2>/dev/null; then
    if wait "${lid_pid}"; then
      lid_rc=0
    else
      lid_rc=$?
    fi
    lid_pid=""
    sleep 1

    if [[ -n "${wait_pid}" ]] && ! kill -0 "${wait_pid}" 2>/dev/null; then
      if wait "${wait_pid}"; then
        wait_rc=0
      else
        wait_rc=$?
      fi
      wait_pid=""
      break
    fi

    echo "[Azas] lid_grip_close launch exited before ArUco success status; shake chain blocked."
    if [[ -n "${wait_pid}" ]]; then
      kill -TERM "${wait_pid}" 2>/dev/null || true
      wait "${wait_pid}" 2>/dev/null || true
      wait_pid=""
    fi
    if [[ "${lid_rc}" -eq 0 ]]; then
      exit 1
    fi
    exit "${lid_rc}"
  fi

  sleep 1
done

if [[ -n "${lid_pid}" ]]; then
  kill -TERM "${lid_pid}" 2>/dev/null || true
  wait "${lid_pid}" 2>/dev/null || true
  lid_pid=""
fi
trap - EXIT

if [[ "${wait_rc}" -ne 0 ]]; then
  echo "[Azas] ArUco lid_grip_close 실패/타임아웃 -> 컵홀더 재픽업/쉐이킹을 건너뜁니다."
  exit "${wait_rc}"
fi

echo "[Azas] ArUco lid_grip_close 성공 status 확인 -> 컵홀더 컵 다시 잡기 후 쉐이킹으로 바로 넘어갑니다."
echo "[Azas] auto_holder_pick_then_shake=true"

source_ros

echo "[Azas] SHAKE START: 컵홀더에 놓인 닫힌 컵을 측정 pose로 다시 side-grip 픽업한 뒤 흔듭니다."
echo "[Azas] 순서: RG2 open -> 컵홀더 retreat 접근 -> holder final pose에서 soft grasp -> holder lift -> 관절 쉐이킹."
echo "[Azas] 주의: 컵 좌표를 새로 만들지 않고 calibration.yaml cup_holder.side_grip_place 측정값만 사용합니다."

python3 tools/run/pick_from_cup_holder_side_grip.py \
  --service-prefix "${SERVICE_PREFIX}" \
  --config "${ROOT}/install/azas_bringup/share/azas_bringup/config/calibration.yaml" \
  --approach-velocity 40.0 --approach-acceleration 40.0 \
  --descend-velocity 40.0 --descend-acceleration 40.0 \
  --lift-velocity 40.0 --lift-acceleration 40.0 \
  --place-final-z-offset-m -0.020 \
  --timeout-sec 90.0 --target-tolerance-mm 12.0 --verify-timeout-sec 45.0 \
  --ikin-timeout-sec 20.0 --ikin-retries 2 \
  --gripper-grasp-width-m 0.068 --gripper-force-n 25.0 \
  --post-grasp-settle-sec 0.8 \
  --z-max 0.28 \
  --execute --confirm ENABLE_CUP_HOLDER_PICK

timeout 5s python3 -m azas_motion.tumbler_collision_scene_node \
  --ros-args -p action:=remove_world -p object_id:=tumbler_in_holder -p dispenser_id:=1 -p publish_once:=true
timeout 5s python3 -m azas_motion.tumbler_collision_scene_node \
  --ros-args -p action:=attach -p object_id:=carried_tumbler -p dispenser_id:=1 -p publish_once:=true

SERVICE_PREFIX="${SERVICE_PREFIX}" \
GRASPED_CUP_TEST_MODE=true \
SKIP_CUP_HOLDER_PICK=true \
REQUIRE_ROBOT_STANDBY=true \
SHAKE_CONTROL_MODE=joint \
SHAKE_CYCLES=3 \
JOINT_SHAKE_BASE_J1_DEG=0.0 \
JOINT_SHAKE_BASE_J2_DEG=-35.0 \
JOINT_SHAKE_BASE_J3_DEG=50.0 \
JOINT_SHAKE_BASE_J4_DEG=0.0 \
JOINT_SHAKE_BASE_J5_DEG=70.0 \
JOINT_SHAKE_BASE_J6_DEG=0.0 \
JOINT_SHAKE_J3_AMPLITUDE_DEG=0.0 \
JOINT_SHAKE_J4_AMPLITUDE_DEG=18.0 \
JOINT_SHAKE_J5_AMPLITUDE_DEG=20.0 \
JOINT_SHAKE_J6_AMPLITUDE_DEG=24.0 \
JOINT_SHAKE_J1_MIN_DEG=-20.0 \
JOINT_SHAKE_J1_MAX_DEG=5.0 \
JOINT_SHAKE_J2_MIN_DEG=-80.0 \
JOINT_SHAKE_J2_MAX_DEG=5.0 \
JOINT_SHAKE_J3_MIN_DEG=0.0 \
JOINT_SHAKE_J3_MAX_DEG=135.0 \
JOINT_SHAKE_MAX_SINGLE_DELTA_DEG=75.0 \
ENFORCE_WRIST_JOINT_LIMITS=false \
WRIST_MIN_DEG=-135.0 \
WRIST_MAX_DEG=135.0 \
JOINT5_MIN_DEG=40.0 \
JOINT5_MAX_DEG=100.0 \
APPROACH_JOINT_VELOCITY=18.0 \
APPROACH_JOINT_ACCELERATION=22.0 \
APPROACH_JOINT_TIME=2.6 \
SHAKE_JOINT_VELOCITY=120.0 \
SHAKE_JOINT_ACCELERATION=160.0 \
SHAKE_JOINT_TIME=0.0 \
JOINT_SHAKE_PEAK_VELOCITY_LIMIT_DEG_S=160.0 \
VERIFY_JOINT_TARGETS=true \
JOINT_TARGET_TOLERANCE_DEG=8.0 \
JOINT_TARGET_WAIT_EXTRA_SEC=3.0 \
JOINT_TARGET_POLL_SEC=0.05 \
REQUIRE_STATE_VALIDITY_FOR_JOINT_SHAKE=false \
REAL_ROBOT_MOTION_CONFIRM=ENABLE_REAL_ROBOT_MOTION \
bash tools/run/run_rule_based_shake_real.sh

echo "[Azas] SHAKE DONE: 손 검출/핸드오버를 위해 카메라 포즈로 복귀합니다 (컵 파지 유지)."
python3 tools/run/direct_movej_joints.py \
  --service-prefix "${SERVICE_PREFIX}" \
  --j1 3.0 --j2 -12.7 --j3 44.0 --j4 -9.0 --j5 133.0 --j6 90.0 \
  --velocity 15 --acceleration 15 \
  --j5-min-deg -150 --j5-max-deg 150 \
  --timeout-sec 60 --motion-timeout-sec 120 \
  --execute --confirm ENABLE_DIRECT_MOVEJ
