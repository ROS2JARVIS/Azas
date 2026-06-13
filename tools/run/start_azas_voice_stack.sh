#!/usr/bin/env bash
# Azas 음성 칵테일 데모 원커맨드 기동:
#   bash tools/run/start_azas_voice_stack.sh
# tmux 세션 하나에 로봇/그리퍼/카메라/음성스택을 순서대로 띄우고 브라우저를 연다.
# 이후 사용자는 화면에서 말만 하면 된다 ("달달한 거 한잔 줘" -> "응").
#
# 2026-06-13 검증 구성 고정:
#  - joint_state_relay는 띄우지 않는다 (bringup의 broadcaster가 이미 /joint_states를
#    퍼블리시하므로, relay까지 켜면 이중 퍼블리시로 MoveIt 실행 검증이 깨져 pick이 실패한다).
#  - 모든 창에 동일한 DDS env (ROS_DOMAIN_ID=9, ROS_LOCALHOST_ONLY=1, UDPv4)를 강제한다.
set -euo pipefail

ROOT="${ROOT:-/home/ssu/Azas}"
SESSION="${SESSION:-azas-voice}"
ROBOT_HOST="${ROBOT_HOST:-192.168.1.100}"
ROBOT_NAME="${ROBOT_NAME:-dsr01}"
RT_HOST="${RT_HOST:-0.0.0.0}"
RG2_IP="${RG2_IP:-192.168.1.1}"
RG2_PORT="${RG2_PORT:-502}"
VOICE_PORT="${VOICE_PORT:-8090}"
# 기본은 실제 로봇 제조까지 켠다. 리허설만 하려면 HW_EXEC=false 로 실행.
HW_EXEC="${HW_EXEC:-true}"
USE_LIVE_STT="${USE_LIVE_STT:-true}"
STT_DEVICE_INDEX="${STT_DEVICE_INDEX:--1}"
STT_LANGUAGE="${STT_LANGUAGE:-ko-KR}"
USE_LLM="${USE_LLM:-false}"
OPEN_BROWSER="${OPEN_BROWSER:-true}"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-9}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

cd "${ROOT}"
mkdir -p "${ROOT}/log/tmux_logic" /tmp/azas_ros_logs

if tmux has-session -t "${SESSION}" >/dev/null 2>&1; then
  echo "[Azas] existing ${SESSION} session found; stopping it first."
  tmux list-panes -s -t "${SESSION}" -F '#{pane_id}' | while read -r pane; do
    [[ -n "${pane}" ]] && tmux send-keys -t "${pane}" C-c >/dev/null 2>&1 || true
  done
  sleep 3
  tmux kill-session -t "${SESSION}" >/dev/null 2>&1 || true
fi

common_env="export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}; export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS}; export ROS_LOG_DIR=/tmp/azas_ros_logs"
stamp='$(date +%Y%m%d-%H%M%S)'

robot_cmd="cd ${ROOT}; ${common_env}; export ROBOT_HOST=${ROBOT_HOST}; export ROBOT_NAME=${ROBOT_NAME}; export RT_HOST=${RT_HOST}; export DOOSAN_REAL_MOTION_CONFIRM=ENABLE_DOOSAN_REAL_MOTION_BRINGUP; bash tools/run/run_doosan_real_m0609.sh 2>&1 | tee ${ROOT}/log/tmux_logic/robot-${stamp}.log"
gripper_cmd="cd ${ROOT}; ${common_env}; source /opt/ros/humble/setup.bash; source ${ROOT}/install/setup.bash; ros2 launch ${ROOT}/install/azas_gripper/share/azas_gripper/launch/rg2_trigger.launch.py ip:=${RG2_IP} port:=${RG2_PORT} connect:=true open_width:=1100 close_width:=0 force:=300 settle_seconds:=0.6 2>&1 | tee ${ROOT}/log/tmux_logic/gripper-${stamp}.log"
camera_cmd="cd ${ROOT}; ${common_env}; source /opt/ros/humble/setup.bash; source ${ROOT}/install/setup.bash; ros2 launch realsense2_camera rs_launch.py camera_name:=camera initial_reset:=true reconnect_timeout:=5.0 enable_color:=true enable_depth:=true align_depth.enable:=true rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30 2>&1 | tee ${ROOT}/log/tmux_logic/camera-${stamp}.log"
voice_cmd="cd ${ROOT}; ${common_env}; source /opt/ros/humble/setup.bash; source ${ROOT}/install/setup.bash; ros2 launch azas_voice azas_voice.launch.py use_live_stt:=${USE_LIVE_STT} stt_device_index:=${STT_DEVICE_INDEX} stt_language:=${STT_LANGUAGE} use_pipeline_executor:=true enable_pipeline_hardware_execution:=${HW_EXEC} pipeline_service_prefix:=${ROBOT_NAME} use_llm:=${USE_LLM} enable_llm:=${USE_LLM} use_tts:=true voice_screen_port:=${VOICE_PORT} 2>&1 | tee ${ROOT}/log/tmux_logic/voice-${stamp}.log"

echo "[Azas] starting robot bringup..."
tmux new-session -d -s "${SESSION}" -n robot "${robot_cmd}"
sleep 10
echo "[Azas] starting gripper..."
tmux new-window -t "${SESSION}" -n gripper "${gripper_cmd}"
sleep 3
echo "[Azas] starting camera..."
tmux new-window -t "${SESSION}" -n camera "${camera_cmd}"
sleep 8
echo "[Azas] starting voice stack (port ${VOICE_PORT}, hardware=${HW_EXEC})..."
tmux new-window -t "${SESSION}" -n voice "${voice_cmd}"
sleep 4

echo ""
echo "[Azas] voice cocktail stack is up: tmux session '${SESSION}' (robot/gripper/camera/voice)"
echo "[Azas] live STT: ${USE_LIVE_STT} device_index=${STT_DEVICE_INDEX} language=${STT_LANGUAGE}"
echo "[Azas] panel: http://localhost:${VOICE_PORT}  — 말로 주문하고 '응'으로 확정하면 제조가 시작됩니다."
echo "[Azas] logs:  tmux attach -t ${SESSION}   /  stop: bash tools/run/stop_azas_voice_stack.sh"
tmux list-windows -t "${SESSION}"

if [[ "${OPEN_BROWSER}" == "true" && -n "${DISPLAY:-}" ]] && command -v xdg-open >/dev/null 2>&1; then
  xdg-open "http://localhost:${VOICE_PORT}" >/dev/null 2>&1 || true
fi
