#!/usr/bin/env bash
# 색상 스캔 단계: color_scan_pose(joints 0,10,32,0,100,90)로 이동한 뒤 디스펜서 색상을 스캔한다.
# dispenser_color_scan_ros.sh가 outputs/dispenser_color_map.json을 새로 만들어야
# run_color_recipe_sequence.py가 진행되므로, 이 단계는 레시피 전에 반드시 성공해야 한다.
set -eo pipefail

cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
mkdir -p /tmp/azas_ros_logs
export ROS_LOG_DIR=/tmp/azas_ros_logs
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-9}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}
if [ -f /home/ssu/ws_moveit/install/setup.bash ]; then
  source /home/ssu/ws_moveit/install/setup.bash
fi
if [ -f /home/ssu/ros2_ws/install/setup.bash ]; then
  source /home/ssu/ros2_ws/install/setup.bash
fi
if [ -f /home/ssu/Azas/install/setup.bash ]; then
  source /home/ssu/Azas/install/setup.bash
else
  source /home/ssu/Azas/install/local_setup.bash
fi
export PYTHONPATH=/home/ssu/Azas/tools/run/python_compat:${PYTHONPATH:-}

COLOR_TOPIC="${COLOR_TOPIC:-/camera/camera/color/image_raw}"
CAMERA_READY_TIMEOUT_SEC="${CAMERA_READY_TIMEOUT_SEC:-8}"
if ! timeout "${CAMERA_READY_TIMEOUT_SEC}s" ros2 topic echo --no-daemon --once --qos-reliability best_effort "${COLOR_TOPIC}" >/tmp/azas_color_scan_camera_check.txt 2>&1; then
  echo "[Azas][FAIL] color_scan camera preflight failed: no frame from ${COLOR_TOPIC} within ${CAMERA_READY_TIMEOUT_SEC}s" >&2
  echo "[Azas][FAIL] Ensure RealSense publishes ${COLOR_TOPIC} with ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}, then retry." >&2
  timeout 3s ros2 topic info --no-daemon -v "${COLOR_TOPIC}" 2>&1 | sed 's/^/[Azas][camera_info] /' >&2 || true
  sed 's/^/[Azas][camera_check] /' /tmp/azas_color_scan_camera_check.txt >&2 || true
  exit 1
fi

SERVICE_PREFIX="${SERVICE_PREFIX:-auto}"
if [[ "${SERVICE_PREFIX}" == "auto" ]]; then
  SERVICE_PREFIX=""
  if timeout 3s ros2 service list --no-daemon >/tmp/azas_color_scan_services.txt 2>/tmp/azas_color_scan_services.err; then
    if grep -qx "/motion/move_joint" /tmp/azas_color_scan_services.txt; then
      SERVICE_PREFIX=""
    elif grep -qx "/dsr01/motion/move_joint" /tmp/azas_color_scan_services.txt; then
      SERVICE_PREFIX="dsr01"
    fi
  fi
fi
if [[ -n "${SERVICE_PREFIX}" ]]; then
  echo "[Azas] color_scan motion service_prefix=${SERVICE_PREFIX}"
else
  echo "[Azas] color_scan motion service_prefix=<root>"
fi

python3 tools/run/direct_movej_joints.py \
  --service-prefix "${SERVICE_PREFIX}" \
  --j1 0 --j2 10 --j3 32 --j4 0 --j5 100 --j6 90 \
  --velocity 30 --acceleration 30 \
  --timeout-sec 60 --motion-timeout-sec 120 \
  --execute --confirm ENABLE_DIRECT_MOVEJ
tools/run/dispenser_color_scan_ros.sh
