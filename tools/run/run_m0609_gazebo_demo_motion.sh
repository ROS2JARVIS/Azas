#!/usr/bin/env bash
set -euo pipefail

# Publish a visible joint-space demo motion to the M0609 Gazebo position
# controller started by run_m0609_gazebo_ros2_control.sh.

ROBOT_NAME="${ROBOT_NAME:-dsr01}"
CYCLES="${CYCLES:-0}"
PERIOD_SEC="${PERIOD_SEC:-3.0}"
HOLD_SEC="${HOLD_SEC:-0.4}"
RATE_HZ="${RATE_HZ:-50.0}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
source /home/ssu/Azas/install/setup.bash 2>/dev/null || true
set -u

echo "[Azas] Driving Gazebo M0609 demo motion through ros2_control"
echo "[Azas] topic=/${ROBOT_NAME}/gz/dsr_position_controller/commands cycles=${CYCLES} period=${PERIOD_SEC}s hold=${HOLD_SEC}s rate=${RATE_HZ}Hz"

exec /home/ssu/Azas/tools/run/send_m0609_gazebo_demo_motion.py \
  --robot-name "${ROBOT_NAME}" \
  --cycles "${CYCLES}" \
  --period-sec "${PERIOD_SEC}" \
  --hold-sec "${HOLD_SEC}" \
  --rate-hz "${RATE_HZ}"
