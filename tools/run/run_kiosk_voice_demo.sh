#!/usr/bin/env bash
set -euo pipefail

# Start the Azas voice stack and kiosk UI together for a no-hardware ordering demo.
# This does not send robot motion, gripper, dispenser, coordinate, or calibration commands.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="${LOG_DIR:-/tmp/azas_kiosk_voice_demo}"
VOICE_PORT="${VOICE_PORT:-8090}"
KIOSK_PORT="${KIOSK_PORT:-8080}"
HOST="${HOST:-0.0.0.0}"
USE_LIVE_STT="${USE_LIVE_STT:-false}"
USE_TTS="${USE_TTS:-true}"
ENABLE_TTS_AUDIO="${ENABLE_TTS_AUDIO:-true}"
USE_LLM="${USE_LLM:-false}"
ENABLE_LLM="${ENABLE_LLM:-false}"

mkdir -p "${LOG_DIR}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/azas_ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

voice_pid=""
kiosk_pid=""

terminate_tree() {
  local pid="$1"
  if [[ -z "${pid}" ]]; then
    return
  fi
  pkill -TERM -P "${pid}" 2>/dev/null || true
  if kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
  fi
  sleep 1
  pkill -KILL -P "${pid}" 2>/dev/null || true
  if kill -0 "${pid}" 2>/dev/null; then
    kill -KILL "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  terminate_tree "${kiosk_pid}"
  terminate_tree "${voice_pid}"
  wait "${kiosk_pid}" "${voice_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[Azas] Starting voice stack"
ros2 launch azas_voice azas_voice.launch.py \
  use_live_stt:="${USE_LIVE_STT}" \
  use_tts:="${USE_TTS}" \
  enable_tts_audio:="${ENABLE_TTS_AUDIO}" \
  use_llm:="${USE_LLM}" \
  enable_llm:="${ENABLE_LLM}" \
  run_voice_screen:=true \
  voice_screen_host:="${HOST}" \
  voice_screen_port:="${VOICE_PORT}" \
  >"${LOG_DIR}/voice.log" 2>&1 &
voice_pid="$!"

echo "[Azas] Starting kiosk"
ros2 launch azas_kiosk azas_kiosk.launch.py \
  host:="${HOST}" \
  port:="${KIOSK_PORT}" \
  >"${LOG_DIR}/kiosk.log" 2>&1 &
kiosk_pid="$!"

sleep 3

if ! kill -0 "${voice_pid}" 2>/dev/null; then
  echo "[FAIL] azas_voice launch exited early. Last log lines:"
  tail -n 120 "${LOG_DIR}/voice.log" || true
  exit 1
fi

if ! kill -0 "${kiosk_pid}" 2>/dev/null; then
  echo "[FAIL] azas_kiosk launch exited early. Last log lines:"
  tail -n 120 "${LOG_DIR}/kiosk.log" || true
  exit 1
fi

cat <<EOF
[Azas] Kiosk + voice demo is running.

Open:
  Kiosk:       http://localhost:${KIOSK_PORT}
  Voice UI:    http://localhost:${VOICE_PORT}

Logs:
  Voice:       ${LOG_DIR}/voice.log
  Kiosk:       ${LOG_DIR}/kiosk.log

Press Ctrl+C to stop both launches.
EOF

wait
