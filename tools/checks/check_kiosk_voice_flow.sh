#!/usr/bin/env bash
set -euo pipefail

# End-to-end no-hardware check:
#   kiosk HTTP order button -> /stt_result -> recipe mapper -> conversation manager
#   kiosk HTTP confirm button -> /stt_result -> confirmed recipe decision
#
# No robot motion, gripper command, dispenser command, coordinates, or calibration
# values are generated or used.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="${LOG_DIR:-/tmp/azas_kiosk_voice_flow_check}"
KIOSK_PORT="${KIOSK_PORT:-18080}"
VOICE_SCREEN_PORT="${VOICE_SCREEN_PORT:-18090}"
KIOSK_URL="${KIOSK_URL:-http://127.0.0.1:${KIOSK_PORT}}"
ORDER_RECIPE_ID="${ORDER_RECIPE_ID:-recipe_01}"
TIMEOUT_SEC="${TIMEOUT_SEC:-15.0}"
START_STACK="${START_STACK:-true}"

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
  if [[ "${START_STACK}" != "true" ]]; then
    return
  fi
  terminate_tree "${kiosk_pid}"
  terminate_tree "${voice_pid}"
  wait "${kiosk_pid}" "${voice_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ "${START_STACK}" == "true" ]]; then
  rm -f "${LOG_DIR}/voice.log" "${LOG_DIR}/kiosk.log"

  echo "[Azas] Starting temporary voice stack for flow check"
  ros2 launch azas_voice azas_voice.launch.py \
    use_live_stt:=false \
    use_tts:=false \
    enable_tts_audio:=false \
    use_llm:=false \
    run_voice_screen:=true \
    voice_screen_host:=127.0.0.1 \
    voice_screen_port:="${VOICE_SCREEN_PORT}" \
    >"${LOG_DIR}/voice.log" 2>&1 &
  voice_pid="$!"

  echo "[Azas] Starting temporary kiosk on ${KIOSK_URL}"
  ros2 launch azas_kiosk azas_kiosk.launch.py \
    host:=127.0.0.1 \
    port:="${KIOSK_PORT}" \
    >"${LOG_DIR}/kiosk.log" 2>&1 &
  kiosk_pid="$!"
else
  echo "[Azas] START_STACK=false; using existing kiosk at ${KIOSK_URL}"
fi

python3 - "${KIOSK_URL}" "${ORDER_RECIPE_ID}" "${TIMEOUT_SEC}" <<'PY'
import json
import sys
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KioskVoiceFlowCheck(Node):
    def __init__(self):
        super().__init__("kiosk_voice_flow_check")
        self.stt_messages = []
        self.decisions = []
        self.confirmed = []
        self.confirmations = []
        self.create_subscription(String, "/stt_result", self._on_stt, 10)
        self.create_subscription(String, "/azas/voice/recipe_decision", self._on_decision, 10)
        self.create_subscription(
            String,
            "/azas/voice/confirmed_recipe_decision",
            self._on_confirmed,
            10,
        )
        self.create_subscription(String, "/azas/voice/confirmation", self._on_confirmation, 10)

    def _on_stt(self, msg):
        self.stt_messages.append(msg.data)

    def _append_json(self, target, msg):
        try:
            target.append(json.loads(msg.data))
        except json.JSONDecodeError:
            target.append({"invalid_json": msg.data})

    def _on_decision(self, msg):
        self._append_json(self.decisions, msg)

    def _on_confirmed(self, msg):
        self._append_json(self.confirmed, msg)

    def _on_confirmation(self, msg):
        self.confirmations.append(msg.data)


def http_json(method, url, payload=None):
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(request, timeout=2.0) as response:
        return json.loads(response.read().decode("utf-8"))


def wait_for_http(url, deadline):
    last_error = None
    while time.monotonic() < deadline:
        try:
            http_json("GET", url + "/api/state")
            return True
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            last_error = exc
            time.sleep(0.2)
    print(f"[FAIL] kiosk HTTP endpoint did not become ready: {last_error}")
    return False


def wait_until(node, deadline, predicate, label):
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    print(f"[FAIL] timed out waiting for {label}")
    return False


def main():
    kiosk_url = sys.argv[1].rstrip("/")
    recipe_id = sys.argv[2]
    timeout_sec = float(sys.argv[3])
    deadline = time.monotonic() + timeout_sec

    if not wait_for_http(kiosk_url, deadline):
        return 1

    rclpy.init()
    node = KioskVoiceFlowCheck()

    if not wait_until(
        node,
        deadline,
        lambda: node.count_subscribers("/stt_result") > 0
        and node.count_publishers("/azas/voice/recipe_decision") > 0,
        "voice subscriptions/publishers",
    ):
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(f"[Azas] POST /api/order recipe_id={recipe_id}")
    order_result = http_json("POST", kiosk_url + "/api/order", {"recipe_id": recipe_id})
    print(json.dumps(order_result, ensure_ascii=False))

    if not wait_until(
        node,
        deadline,
        lambda: any(item.get("recipe_id") == recipe_id for item in node.decisions),
        "/azas/voice/recipe_decision",
    ):
        print("[DEBUG] stt_messages=", node.stt_messages)
        print("[DEBUG] decisions=", json.dumps(node.decisions, ensure_ascii=False))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print("[Azas] POST /api/confirm")
    confirm_result = http_json("POST", kiosk_url + "/api/confirm", {})
    print(json.dumps(confirm_result, ensure_ascii=False))

    if not wait_until(
        node,
        deadline,
        lambda: any(item.get("confirmed") and item.get("recipe_id") == recipe_id for item in node.confirmed),
        "/azas/voice/confirmed_recipe_decision",
    ):
        print("[DEBUG] stt_messages=", node.stt_messages)
        print("[DEBUG] confirmations=", node.confirmations)
        print("[DEBUG] confirmed=", json.dumps(node.confirmed, ensure_ascii=False))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print("[PASS] kiosk HTTP order and confirm reached azas_voice confirmed decision")
    print("[INFO] stt_messages=", node.stt_messages)
    print("[INFO] latest_decision=", json.dumps(node.decisions[-1], ensure_ascii=False))
    print("[INFO] latest_confirmed=", json.dumps(node.confirmed[-1], ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


raise SystemExit(main())
PY

echo "[Azas] Flow check logs:"
echo "  ${LOG_DIR}/voice.log"
echo "  ${LOG_DIR}/kiosk.log"
