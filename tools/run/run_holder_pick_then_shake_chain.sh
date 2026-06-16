#!/usr/bin/env bash
set -euo pipefail

# Resume helper for the post-lid phase. This intentionally reuses the measured
# cup-holder pickup and rule-based shake path instead of introducing new poses.

ROOT="${ROOT:-/home/ssu/Azas}"
SERVICE_PREFIX="${SERVICE_PREFIX:-dsr01}"
SKIP_CUP_HOLDER_PICK="${SKIP_CUP_HOLDER_PICK:-false}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-9}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

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

export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS SERVICE_PREFIX
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
export PYTHONPATH="${ROOT}/tools/run/python_compat:${PYTHONPATH:-}"
mkdir -p "${ROS_LOG_DIR}"

echo "[Azas] HOLDER_PICK_THEN_SHAKE START: skip_holder_pick=${SKIP_CUP_HOLDER_PICK}"
echo "[Azas] source=measured cup_holder.side_grip_place and existing shake sequence; no generated cup coordinates"
echo "[Azas] direct holder-to-shake path: skipping MoveIt state-validity prewait; joint bounds and target verification remain enabled"

SERVICE_PREFIX="${SERVICE_PREFIX}" \
GRASPED_CUP_TEST_MODE=true \
SKIP_CUP_HOLDER_PICK="${SKIP_CUP_HOLDER_PICK}" \
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
SHAKE_JOINT_VELOCITY=90.0 \
SHAKE_JOINT_ACCELERATION=120.0 \
SHAKE_JOINT_TIME=0.0 \
JOINT_SHAKE_PEAK_VELOCITY_LIMIT_DEG_S=130.0 \
VERIFY_JOINT_TARGETS=true \
JOINT_TARGET_TOLERANCE_DEG=8.0 \
JOINT_TARGET_WAIT_EXTRA_SEC=3.0 \
JOINT_TARGET_POLL_SEC=0.05 \
REQUIRE_STATE_VALIDITY_FOR_JOINT_SHAKE=false \
REAL_ROBOT_MOTION_CONFIRM=ENABLE_REAL_ROBOT_MOTION \
bash tools/run/run_rule_based_shake_real.sh

echo "[Azas] SHAKE DONE: returning to camera pose with cup grasped."
python3 tools/run/direct_movej_joints.py \
  --service-prefix "${SERVICE_PREFIX}" \
  --j1 3.0 --j2 -12.7 --j3 44.0 --j4 -9.0 --j5 133.0 --j6 90.0 \
  --velocity 15 --acceleration 15 \
  --j5-min-deg -150 --j5-max-deg 150 \
  --timeout-sec 60 --motion-timeout-sec 120 \
  --execute --confirm ENABLE_DIRECT_MOVEJ
