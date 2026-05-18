#!/usr/bin/env bash
set -euo pipefail

# RViz-only simulation of the chained rule-based workflow:
# dispenser pre-place transfer while holding the cup, then lifted high-shake.

SELECTED_DISPENSER_ID="${SELECTED_DISPENSER_ID:-2}"
USE_RVIZ="${USE_RVIZ:-true}"
USE_ROBOT_URDF="${USE_ROBOT_URDF:-true}"
ANIMATE_ROBOT_JOINTS="${ANIMATE_ROBOT_JOINTS:-true}"
ENABLE_IK_PREVIEW="${ENABLE_IK_PREVIEW:-true}"
SHAKE_DELAY_SEC="${SHAKE_DELAY_SEC:-10.0}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
set -u

exec ros2 launch azas_bringup tumbler_dispenser_then_shake_demo.launch.py \
  selected_dispenser_id:="${SELECTED_DISPENSER_ID}" \
  use_rviz:="${USE_RVIZ}" \
  use_robot_urdf:="${USE_ROBOT_URDF}" \
  enable_ik_preview:="${ENABLE_IK_PREVIEW}" \
  shake_delay_sec:="${SHAKE_DELAY_SEC}"
