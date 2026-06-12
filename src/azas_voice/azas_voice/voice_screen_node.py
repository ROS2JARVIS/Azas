from __future__ import annotations

from collections import deque
import json
import mimetypes
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from typing import Any

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - allows helper tests without sourced ROS
    get_package_share_directory = None

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - allows helper tests without sourced ROS
    rclpy = None
    Node = object
    String = None

from azas_voice.recipe_catalog import build_public_catalog


def build_initial_state() -> dict[str, Any]:
    return {
        "started_at": time.time(),
        "catalog": build_public_catalog(),
        "last_stt": "",
        "last_confirmation": "",
        "ui_state": {"state": "idle", "emotion": "neutral", "text": ""},
        "decision": {},
        "confirmed_decision": {},
        "pipeline_status": {},
        "events": [],
    }


class VoiceScreenNode(Node):
    """Serve a local voice screen and aggregate Azas voice topic state."""

    def __init__(self):
        if rclpy is None or String is None or get_package_share_directory is None:
            raise RuntimeError("ROS 2 Python packages are not available. Source the ROS environment first.")
        super().__init__("azas_voice_screen_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8090)
        self.declare_parameter("stt_topic", "/stt_result")
        self.declare_parameter("decision_topic", "/azas/voice/recipe_decision")
        self.declare_parameter("confirmation_topic", "/azas/voice/confirmation")
        self.declare_parameter("ui_state_topic", "/azas/voice/ui_state")
        self.declare_parameter("confirmed_decision_topic", "/azas/voice/confirmed_recipe_decision")
        self.declare_parameter("pipeline_status_topic", "/azas/voice/pipeline_status")

        self._lock = threading.Lock()
        self._events: deque[dict[str, Any]] = deque(maxlen=12)
        self._state = build_initial_state()

        self._stt_pub = self.create_publisher(
            String,
            str(self.get_parameter("stt_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("stt_topic").value),
            self._on_stt,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("decision_topic").value),
            self._on_decision,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("confirmation_topic").value),
            self._on_confirmation,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("ui_state_topic").value),
            self._on_ui_state,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("confirmed_decision_topic").value),
            self._on_confirmed_decision,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("pipeline_status_topic").value),
            self._on_pipeline_status,
            10,
        )

        self._web_root = Path(get_package_share_directory("azas_voice")) / "web"
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self._server = ThreadingHTTPServer((host, port), self._build_handler())
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        self.get_logger().info(f"Azas voice screen ready at http://{host}:{port}")

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            payload = dict(self._state)
            payload["events"] = list(self._events)
        return payload

    def publish_test_utterance(self, text: str) -> None:
        utterance = text.strip()
        if not utterance:
            raise ValueError("empty utterance")
        msg = String()
        msg.data = utterance
        self._stt_pub.publish(msg)
        with self._lock:
            self._state["last_stt"] = utterance
            self._state["last_stt_at"] = time.time()

    def _on_stt(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        with self._lock:
            self._state["last_stt"] = text
            self._state["last_stt_at"] = time.time()
        self._remember("user", text)

    def _on_decision(self, msg: String) -> None:
        decision = _json_or_text(msg.data)
        with self._lock:
            self._state["decision"] = decision
            self._state["decision_at"] = time.time()

    def _on_confirmation(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        with self._lock:
            self._state["last_confirmation"] = text
            self._state["last_confirmation_at"] = time.time()
        self._remember("azas", text)

    def _on_ui_state(self, msg: String) -> None:
        with self._lock:
            self._state["ui_state"] = _json_or_text(msg.data)
            self._state["ui_state_at"] = time.time()

    def _on_confirmed_decision(self, msg: String) -> None:
        confirmed = _json_or_text(msg.data)
        with self._lock:
            self._state["confirmed_decision"] = confirmed
            self._state["confirmed_decision_at"] = time.time()

    def _on_pipeline_status(self, msg: String) -> None:
        with self._lock:
            self._state["pipeline_status"] = _json_or_text(msg.data)
            self._state["pipeline_status_at"] = time.time()

    def _remember(self, speaker: str, text: str) -> None:
        with self._lock:
            self._events.appendleft(
                {
                    "speaker": speaker,
                    "text": text,
                    "at": time.time(),
                }
            )

    def _build_handler(self):
        node = self

        class VoiceScreenRequestHandler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path in {"/", "/voice.html"}:
                    self._send_file(node._web_root / "voice.html")
                    return
                if self.path == "/voice.css":
                    self._send_file(node._web_root / "voice.css")
                    return
                if self.path == "/voice.js":
                    self._send_file(node._web_root / "voice.js")
                    return
                if self.path == "/api/state":
                    self._send_json(node.snapshot())
                    return
                self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self) -> None:
                try:
                    payload = self._read_json()
                    if self.path != "/api/utterance":
                        raise ValueError(f"unknown endpoint: {self.path}")
                    text = str(payload.get("text", ""))
                    node.publish_test_utterance(text)
                except ValueError as exc:
                    self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
                    return
                self._send_json({"ok": True, "text": text})

            def log_message(self, format: str, *args: object) -> None:
                node.get_logger().debug(format % args)

            def _read_json(self) -> dict[str, Any]:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0:
                    return {}
                body = self.rfile.read(length).decode("utf-8")
                try:
                    payload = json.loads(body)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"invalid json: {exc}") from exc
                if not isinstance(payload, dict):
                    raise ValueError("json body must be an object")
                return payload

            def _send_json(
                self,
                payload: dict[str, Any],
                status: HTTPStatus = HTTPStatus.OK,
            ) -> None:
                data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def _send_file(self, path: Path) -> None:
                if not path.is_file():
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                data = path.read_bytes()
                content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", f"{content_type}; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

        return VoiceScreenRequestHandler

    def destroy_node(self):
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=1.0)
        super().destroy_node()


def _json_or_text(text: str) -> Any:
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return {"text": text}


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are not available. Source the ROS environment first.")
    rclpy.init(args=args)
    node = VoiceScreenNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
