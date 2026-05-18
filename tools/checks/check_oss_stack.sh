#!/usr/bin/env bash
set -euo pipefail

# Non-hardware readiness check for the Azas open-source robot-control stack.
# This script does not start cameras, RG2, or real robot motion.

STRICT_OPTIONAL="${STRICT_OPTIONAL:-false}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

failures=0
warnings=0

check() {
  local label="$1"
  shift
  if "$@" >/tmp/azas_check_oss_stack.out 2>/tmp/azas_check_oss_stack.err; then
    echo "[OK] ${label}"
  else
    echo "[FAIL] ${label}"
    sed -n '1,20p' /tmp/azas_check_oss_stack.err
    failures=$((failures + 1))
  fi
}

warn_check() {
  local label="$1"
  shift
  if "$@" >/tmp/azas_check_oss_stack.out 2>/tmp/azas_check_oss_stack.err; then
    echo "[OK] ${label}"
  else
    if [[ "${STRICT_OPTIONAL}" == "true" ]]; then
      echo "[FAIL] ${label}"
      failures=$((failures + 1))
    else
      echo "[WARN] ${label}"
      warnings=$((warnings + 1))
    fi
    sed -n '1,12p' /tmp/azas_check_oss_stack.err
  fi
}

set +u
source /opt/ros/humble/setup.bash
if [[ -f /home/ssu/Azas/install/setup.bash ]]; then
  source /home/ssu/Azas/install/setup.bash
fi
if [[ -f /home/ssu/ros2_ws/install/setup.bash ]]; then
  source /home/ssu/ros2_ws/install/setup.bash
fi
set -u

echo "[Azas] Checking ROS package availability"
check "azas_bringup package" ros2 pkg prefix azas_bringup
check "azas_perception package" ros2 pkg prefix azas_perception
check "azas_interfaces package" ros2 pkg prefix azas_interfaces
check "jarvis package for floor-place/RG2 bridge" ros2 pkg prefix jarvis
check "Doosan bringup package" ros2 pkg prefix dsr_bringup2
check "Doosan M0609 MoveIt config package" ros2 pkg prefix dsr_moveit_config_m0609
check "MoveItPy package" ros2 pkg prefix moveit_py
warn_check "realsense2_camera package" ros2 pkg prefix realsense2_camera

echo "[Azas] Checking launch descriptions without starting hardware"
check "Doosan virtual MoveIt launch arguments" \
  ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py --show-args
check "robot_connection_control launch arguments" \
  ros2 launch azas_bringup robot_connection_control.launch.py --show-args
check "yolo_to_floor_place launch arguments" \
  ros2 launch azas_bringup yolo_to_floor_place.launch.py --show-args
check "cocktail_dryrun launch arguments" \
  ros2 launch azas_bringup cocktail_dryrun.launch.py --show-args

echo "[Azas] Checking Python runtime imports"
check "core Python imports" python3 -c "import numpy, cv2, rclpy"
warn_check "Ultralytics YOLO import" python3 -c "from ultralytics import YOLO"
warn_check "LangSAM import" python3 -c "from lang_sam import LangSAM"
warn_check "SpeechRecognition import" python3 -c "import speech_recognition"
warn_check "Vosk import" python3 -c "import vosk"
warn_check "Whisper import" python3 -c "import whisper"

echo "[Azas] Checking local model path"
if [[ -f /home/ssu/Downloads/best.pt ]]; then
  echo "[OK] YOLO model exists: /home/ssu/Downloads/best.pt"
else
  echo "[WARN] YOLO model missing: /home/ssu/Downloads/best.pt"
  warnings=$((warnings + 1))
fi

echo "[Azas] Result: failures=${failures} warnings=${warnings} strict_optional=${STRICT_OPTIONAL}"
if [[ "${failures}" -ne 0 ]]; then
  exit 1
fi
