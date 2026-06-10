#!/usr/bin/env bash
set -euo pipefail

# One-command virtual sim for the voice -> dispenser chain on the Doosan M0609.
#
# Brings up, in one terminal:
#   1) virtual Doosan M0609 (MoveIt + RViz)         -> provides /motion/* services
#   2) Azas collision scene (safety zone + dispenser box)
#   3) azas_voice stack with the dispenser executor (hardware execution enabled)
#      incl. the voice screen web UI on VOICE_PORT (default 8090)
#   4) the kiosk web UI on KIOSK_PORT (default 8080) unless WITH_KIOSK=false
#
# Order from http://localhost:8080 (kiosk: pick menu -> 시작) or
#            http://localhost:8090 (voice: say an order -> "응").
#
# Then publish a confirmed recipe decision to drive the arm to the dispensers, e.g.:
#   ros2 topic pub --once /azas/voice/confirmed_recipe_decision std_msgs/msg/String \
#     '{data: "{\"intent\":\"make_cocktail\",\"confirmed\":true,\"recipe_id\":\"sim\",\"dispenser_ids\":[\"red\",\"blue\"],\"dispenser_amounts\":{\"red\":2,\"blue\":1}}"}'
#
# Override behaviour with env vars (see defaults below).

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

SERVICE_PREFIX="${SERVICE_PREFIX:-/}"            # virtual stack exposes /motion/* with no namespace
REQUIRE_TCP="${REQUIRE_TCP:-false}"             # sim has no named TCP -> keep false
USE_TTS="${USE_TTS:-false}"                     # silence audio for a quiet sim by default
SERVICE_WAIT_SEC="${SERVICE_WAIT_SEC:-60}"      # how long to wait for the virtual robot
AUTO_ORDER="${AUTO_ORDER:-false}"               # set to true to auto-fire a red/blue test order
WITH_KIOSK="${WITH_KIOSK:-true}"                # also launch the kiosk web UI on KIOSK_PORT
KIOSK_PORT="${KIOSK_PORT:-8080}"
VOICE_PORT="${VOICE_PORT:-8090}"                # voice screen web UI port
# Point the kiosk "제어 상태" display at the dispenser executor's status so it
# reflects the real run (queued/starting/completed) instead of staying "대기".
KIOSK_STATUS_TOPIC="${KIOSK_STATUS_TOPIC:-/azas/voice/dispenser_execution_status}"

set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
if [[ -f /home/ssu/ws_moveit/install/setup.bash ]]; then
  source /home/ssu/ws_moveit/install/setup.bash
fi
source "${ROOT_DIR}/install/setup.bash"
set -u

robot_pid=""
scene_pid=""
voice_pid=""
kiosk_pid=""

terminate_tree() {
  local pid="$1"
  [[ -z "${pid}" ]] && return
  pkill -TERM -P "${pid}" 2>/dev/null || true
  kill "${pid}" 2>/dev/null || true
  sleep 1
  pkill -KILL -P "${pid}" 2>/dev/null || true
  kill -KILL "${pid}" 2>/dev/null || true
}

cleanup() {
  echo "[Azas] Shutting down voice-dispenser sim..."
  terminate_tree "${kiosk_pid}"
  terminate_tree "${voice_pid}"
  terminate_tree "${scene_pid}"
  terminate_tree "${robot_pid}"
  wait "${kiosk_pid}" "${voice_pid}" "${scene_pid}" "${robot_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[Azas] (1/3) Starting virtual Doosan M0609 (MoveIt + RViz)..."
bash "${ROOT_DIR}/tools/run/run_doosan_virtual_m0609.sh" &
robot_pid=$!

echo "[Azas] Waiting for the virtual robot motion services (up to ${SERVICE_WAIT_SEC}s)..."
deadline=$((SECONDS + SERVICE_WAIT_SEC))
until ros2 service list 2>/dev/null | grep -q "/motion/move_joint"; do
  if (( SECONDS >= deadline )); then
    echo "[Azas] ERROR: /motion/move_joint never appeared. Aborting." >&2
    exit 1
  fi
  if ! kill -0 "${robot_pid}" 2>/dev/null; then
    echo "[Azas] ERROR: virtual robot process exited early. Aborting." >&2
    exit 1
  fi
  sleep 1
done
echo "[Azas] Virtual robot is up (/motion/move_joint found)."

echo "[Azas] (2/3) Starting Azas collision scene (safety zone + dispenser box)..."
ros2 launch azas_bringup workspace_collision_scene.launch.py &
scene_pid=$!
sleep 2

echo "[Azas] (3/3) Starting azas_voice stack with dispenser executor..."
ros2 launch azas_voice azas_voice.launch.py \
  use_dispenser_executor:=true \
  enable_dispenser_hardware_execution:=true \
  dispenser_service_prefix:="${SERVICE_PREFIX}" \
  dispenser_require_tcp_for_taught_posx:="${REQUIRE_TCP}" \
  run_voice_screen:=true \
  voice_screen_port:="${VOICE_PORT}" \
  use_tts:="${USE_TTS}" &
voice_pid=$!
sleep 3

if [[ "${WITH_KIOSK}" == "true" ]]; then
  echo "[Azas] (+) Starting kiosk web UI on port ${KIOSK_PORT}..."
  ros2 launch azas_kiosk azas_kiosk.launch.py \
    host:=0.0.0.0 port:="${KIOSK_PORT}" \
    cocktail_status_topic:="${KIOSK_STATUS_TOPIC}" &
  kiosk_pid=$!
  sleep 2
fi

echo ""
echo "[Azas] ============================================================"
echo "[Azas] Voice-dispenser sim is up. Order through the web UIs:"
echo "[Azas]"
if [[ "${WITH_KIOSK}" == "true" ]]; then
  echo "[Azas]   Kiosk : http://localhost:${KIOSK_PORT}   (click a menu, then click 시작/Start)"
fi
echo "[Azas]   Voice : http://localhost:${VOICE_PORT}   (say/type an order, then \"응\")"
echo "[Azas]"
echo "[Azas] A click/utterance alone only stages the order; the CONFIRM step"
echo "[Azas] (시작 button / \"응\") is what triggers the robot."
echo "[Azas]"
echo "[Azas] Or fire a confirmed order directly:"
echo "[Azas]   ros2 topic pub --once /azas/voice/confirmed_recipe_decision std_msgs/msg/String \\"
echo "[Azas]     '{data: \"{\\\"intent\\\":\\\"make_cocktail\\\",\\\"confirmed\\\":true,\\\"recipe_id\\\":\\\"sim\\\",\\\"dispenser_ids\\\":[\\\"red\\\",\\\"blue\\\"],\\\"dispenser_amounts\\\":{\\\"red\\\":2,\\\"blue\\\":1}}\"}'"
echo "[Azas]"
echo "[Azas] Watch status:  ros2 topic echo /azas/voice/dispenser_execution_status"
echo "[Azas] Ctrl+C here stops the whole sim."
echo "[Azas] ============================================================"

if [[ "${AUTO_ORDER}" == "true" ]]; then
  echo "[Azas] AUTO_ORDER=true -> firing a red(x2)+blue(x1) test order in 3s..."
  sleep 3
  ros2 topic pub --once /azas/voice/confirmed_recipe_decision std_msgs/msg/String \
    '{data: "{\"intent\":\"make_cocktail\",\"confirmed\":true,\"recipe_id\":\"sim\",\"dispenser_ids\":[\"red\",\"blue\"],\"dispenser_amounts\":{\"red\":2,\"blue\":1}}"}' || true
fi

# Keep the sim alive until any component exits or the user hits Ctrl+C.
wait -n "${robot_pid}" "${scene_pid}" "${voice_pid}" ${kiosk_pid:+"${kiosk_pid}"}
