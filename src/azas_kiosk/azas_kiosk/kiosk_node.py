from __future__ import annotations

import json
import mimetypes
import random
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

from azas_kiosk.menu_catalog import MENU_ITEMS, build_menu_payload


class KioskBridgeNode(Node):
    """Expose a local kiosk UI without creating robot coordinates or motion plans."""

    def __init__(self):
        if rclpy is None or String is None or get_package_share_directory is None:
            raise RuntimeError("ROS 2 Python packages are not available. Source the ROS environment first.")
        super().__init__("azas_kiosk_node")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("stt_topic", "/stt_result")
        self.declare_parameter("decision_topic", "/azas/voice/recipe_decision")
        self.declare_parameter("confirmation_topic", "/azas/voice/confirmation")
        self.declare_parameter("ui_state_topic", "/azas/voice/ui_state")
        self.declare_parameter("cocktail_status_topic", "/azas/cocktail/status")

        self._lock = threading.Lock()
        self._state: dict[str, Any] = {
            "started_at": time.time(),
            "last_command": "",
            "last_confirmation": "",
            "selected_recipe_id": "",
            "ui_state": {"state": "unknown", "emotion": "neutral", "text": ""},
            "cocktail_status": {},
        }

        self._stt_pub = self.create_publisher(
            String,
            str(self.get_parameter("stt_topic").value),
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
            str(self.get_parameter("cocktail_status_topic").value),
            self._on_cocktail_status,
            10,
        )

        self._web_root = Path(get_package_share_directory("azas_kiosk")) / "web"
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        handler = self._build_handler()
        self._server = ThreadingHTTPServer((host, port), handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"Azas kiosk ready at http://{host}:{port} publishing to "
            f"{self.get_parameter('stt_topic').value}"
        )

    def publish_command(
        self,
        text: str,
        *,
        selected_recipe_id: str = "",
        local_confirmation: str = "",
        clear_selection: bool = False,
    ) -> None:
        command = text.strip()
        if not command:
            raise ValueError("empty command")
        msg = String()
        msg.data = command
        self._stt_pub.publish(msg)
        with self._lock:
            self._state["last_command"] = command
            self._state["last_command_at"] = time.time()
            if clear_selection:
                self._state["selected_recipe_id"] = ""
            if selected_recipe_id:
                self._state["selected_recipe_id"] = selected_recipe_id
            if local_confirmation:
                self._state["last_confirmation"] = local_confirmation
                self._state["last_confirmation_at"] = time.time()
        self.get_logger().info(f"kiosk command -> {command}")

    def recommend_menu(self) -> dict[str, str]:
        item = random.choice(MENU_ITEMS)
        self.publish_command(
            item.order_text,
            selected_recipe_id=item.recipe_id,
            local_confirmation=f"{item.name}을 추천드릴게요. 시작을 누르면 진행합니다.",
        )
        return {"recipe_id": item.recipe_id, "name": item.name, "command": item.order_text}

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            payload = dict(self._state)
        payload["menus"] = build_menu_payload()
        return payload

    def _on_confirmation(self, msg: String) -> None:
        with self._lock:
            self._state["last_confirmation"] = msg.data
            self._state["last_confirmation_at"] = time.time()

    def _on_decision(self, msg: String) -> None:
        decision = _json_or_text(msg.data)
        if not isinstance(decision, dict):
            return
        recipe_id = str(decision.get("recipe_id") or "")
        confirmation = str(decision.get("confirmation") or "")
        with self._lock:
            if recipe_id:
                self._state["selected_recipe_id"] = recipe_id
            if confirmation and not self._state.get("last_confirmation"):
                self._state["last_confirmation"] = confirmation
                self._state["last_confirmation_at"] = time.time()

    def _on_ui_state(self, msg: String) -> None:
        with self._lock:
            self._state["ui_state"] = _json_or_text(msg.data)
            self._state["ui_state_at"] = time.time()

    def _on_cocktail_status(self, msg: String) -> None:
        with self._lock:
            self._state["cocktail_status"] = _json_or_text(msg.data)
            self._state["cocktail_status_at"] = time.time()

    def _build_handler(self):
        node = self

        class KioskRequestHandler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path in {"/", "/index.html"}:
                    self._send_file(node._web_root / "index.html")
                    return
                if self.path == "/styles.css":
                    self._send_file(node._web_root / "styles.css")
                    return
                if self.path == "/app.js":
                    self._send_file(node._web_root / "app.js")
                    return
                if self.path == "/api/state":
                    self._send_json(node.snapshot())
                    return
                self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self) -> None:
                try:
                    payload = self._read_json()
                    if self.path == "/api/recommend":
                        result = node.recommend_menu()
                        self._send_json({"ok": True, **result})
                        return
                    result = _command_from_request(self.path, payload)
                    node.publish_command(**result)
                except ValueError as exc:
                    self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
                    return
                self._send_json({"ok": True, "command": result["text"]})

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

        return KioskRequestHandler

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


def _command_from_request(path: str, payload: dict[str, Any]) -> dict[str, Any]:
    if path == "/api/order":
        recipe_id = str(payload.get("recipe_id", ""))
        for item in MENU_ITEMS:
            if item.recipe_id == recipe_id:
                return {
                    "text": item.order_text,
                    "selected_recipe_id": item.recipe_id,
                    "local_confirmation": (
                        f"{item.name}을 선택했습니다. 시작을 누르면 진행합니다."
                    ),
                }
        raise ValueError(f"unknown recipe_id: {recipe_id}")
    if path == "/api/confirm":
        return {"text": "응", "local_confirmation": "제조 시작 요청을 보냈습니다."}
    if path == "/api/cancel":
        return {
            "text": "취소",
            "local_confirmation": "주문을 취소했습니다.",
            "clear_selection": True,
        }
    if path == "/api/command":
        return {"text": str(payload.get("text", "")).strip()}
    raise ValueError(f"unknown endpoint: {path}")


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are not available. Source the ROS environment first.")
    rclpy.init(args=args)
    node = KioskBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
