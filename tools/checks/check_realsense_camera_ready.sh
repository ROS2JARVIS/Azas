#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
INSTALL_SETUP="${INSTALL_SETUP:-$ROOT/install/local_setup.bash}"
TIMEOUT_SEC="${TIMEOUT_SEC:-20}"

TOPICS=(
  "/camera/camera/color/image_raw"
  "/camera/camera/aligned_depth_to_color/image_raw"
  "/camera/camera/color/camera_info"
)

log() {
  echo "[Azas camera] $*"
}

check_topic_once() {
  local topic="$1"
  timeout 3s ros2 topic echo --once "$topic" >/tmp/azas_camera_topic_sample.txt 2>&1
}

wait_for_topics() {
  local deadline=$((SECONDS + TIMEOUT_SEC))
  local missing=()

  while (( SECONDS < deadline )); do
    missing=()
    for topic in "${TOPICS[@]}"; do
      if ! check_topic_once "$topic"; then
        missing+=("$topic")
      fi
    done

    if ((${#missing[@]} == 0)); then
      return 0
    fi
    sleep 1
  done

  log "missing topics after ${TIMEOUT_SEC}s:"
  printf '  - %s\n' "${missing[@]}"
  log "last ros2 output:"
  sed 's/^/  /' /tmp/azas_camera_topic_sample.txt 2>/dev/null || true
  return 1
}

[[ -f "$ROS_SETUP" ]] || {
  log "missing ROS setup: $ROS_SETUP"
  exit 1
}

# shellcheck source=/opt/ros/humble/setup.bash
source "$ROS_SETUP"
if [[ -f "$INSTALL_SETUP" ]]; then
  # shellcheck source=/dev/null
  source "$INSTALL_SETUP"
fi

if ! ros2 pkg prefix realsense2_camera >/dev/null 2>&1; then
  log "missing ROS package: realsense2_camera"
  log "install example: sudo apt install -y ros-humble-realsense2-camera"
  exit 1
fi
log "realsense2_camera package: $(ros2 pkg prefix realsense2_camera)"

if command -v lsusb >/dev/null 2>&1; then
  if lsusb | grep -Eiq "Intel|RealSense"; then
    log "USB device candidate:"
    lsusb | grep -Ei "Intel|RealSense" | sed 's/^/  /'
  else
    log "no Intel/RealSense USB device found by lsusb"
    log "check USB3 cable/port, power, and whether another PC owns the camera"
  fi
else
  log "lsusb not found; skipping USB device listing"
fi

video_count="$(find /dev -maxdepth 1 -name 'video*' 2>/dev/null | wc -l)"
log "/dev/video* count: $video_count"
if id -nG | tr ' ' '\n' | grep -qx video; then
  log "current user is in video group"
else
  log "current user is not in video group; camera access may fail until relogin after: sudo usermod -aG video $USER"
fi

if wait_for_topics; then
  log "camera topics already publishing"
  exit 0
fi

log "starting temporary RealSense launch for readiness check"
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true >/tmp/azas_realsense_launch.log 2>&1 &
camera_pid=$!
trap 'kill "$camera_pid" >/dev/null 2>&1 || true' EXIT

if wait_for_topics; then
  log "PASS: color, aligned depth, and camera_info topics are publishing"
  exit 0
fi

log "RealSense launch log tail:"
tail -80 /tmp/azas_realsense_launch.log | sed 's/^/  /'
log "FAIL: camera is not ready for panel side_grip"
exit 1
