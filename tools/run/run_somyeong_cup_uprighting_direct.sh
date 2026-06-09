#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/ssu/Azas}"
SERVICE_PREFIX="${SERVICE_PREFIX:-dsr01}"
DISPLAY="${DISPLAY:-:0}"
XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
MODEL_PATH="${MODEL_PATH:-${ROOT}/src/azas_perception/config/yolo_cup_uprighting_best.pt}"
AUTO_PICK="${AUTO_PICK:-false}"
SKIP_INITIAL_HOME_MOVE="${SKIP_INITIAL_HOME_MOVE:-true}"
PUBLISH_HAND_EYE_TF="${PUBLISH_HAND_EYE_TF:-true}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-9}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

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

export DISPLAY XAUTHORITY ROS_DOMAIN_ID ROS_LOCALHOST_ONLY
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
export PYTHONPATH="${ROOT}/tools/run/python_compat:${PYTHONPATH:-}"
export AZAS_CUP_UPRIGHTING_MODEL_PATH="${MODEL_PATH}"
mkdir -p "${ROS_LOG_DIR}" "${ROOT}/log/tmux_logic"

echo "[Azas] START Somyeong cup_uprighting direct command"
echo "[Azas] OpenCV window: confirm fallen cup, then press p. Quit with q/Esc."
echo "[Azas] service_prefix=${SERVICE_PREFIX} DISPLAY=${DISPLAY} XAUTHORITY=${XAUTHORITY}"
echo "[Azas] ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "[Azas] model_path=${MODEL_PATH}"
echo "[Azas] auto_pick=${AUTO_PICK} skip_initial_home_move=${SKIP_INITIAL_HOME_MOVE} publish_hand_eye_tf=${PUBLISH_HAND_EYE_TF}"

if [[ ! -f "${MODEL_PATH}" ]]; then
  echo "[Azas][FAIL] YOLO model missing: ${MODEL_PATH}" >&2
  exit 2
fi

if [[ ! -x "${ROOT}/install/azas_cup_uprighting/lib/azas_cup_uprighting/yolo_cup_uprighting" ]]; then
  echo "[Azas][FAIL] yolo_cup_uprighting executable missing. Build azas_cup_uprighting first." >&2
  exit 3
fi

ros2 launch "${ROOT}/src/azas_cup_uprighting/launch/yolo_cup_uprighting.launch.py" \
  model_path:="${MODEL_PATH}" \
  auto_pick:="${AUTO_PICK}" \
  skip_initial_home_move:="${SKIP_INITIAL_HOME_MOVE}" \
  publish_hand_eye_tf:="${PUBLISH_HAND_EYE_TF}"
