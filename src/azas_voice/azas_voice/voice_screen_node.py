from __future__ import annotations

from collections import deque
import json
import mimetypes
import re
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from typing import Any
from urllib.parse import urlparse

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - allows helper tests without sourced ROS
    get_package_share_directory = None

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - allows helper tests without sourced ROS
    rclpy = None
    Node = object
    DurabilityPolicy = None
    HistoryPolicy = None
    QoSProfile = None
    ReliabilityPolicy = None
    String = None

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - camera support is optional for pure tests
    cv2 = None
    np = None

try:
    from azas_interfaces.msg import CupDetection
    from sensor_msgs.msg import Image
except ImportError:  # pragma: no cover - ROS message types are unavailable in pure tests
    CupDetection = None
    Image = None

from azas_voice.recipe_catalog import build_public_catalog


if QoSProfile is not None:
    LOW_LATENCY_IMAGE_QOS = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
else:  # pragma: no cover - only used outside ROS test environments
    LOW_LATENCY_IMAGE_QOS = 10

_CAMERA_STREAMS = {"realsense", "cup", "lid", "hand"}
_CENTER_PATTERNS = {
    "cup": (r"\bcenter=\((\d+),(\d+)\)",),
    "lid": (
        r"\blid_center=\((\d+),(\d+)\)",
        r"\baruco_center=\((\d+),(\d+)\)",
        r"\bcenter=\((\d+),(\d+)\)",
    ),
}


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
        "camera_status": {},
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
        self.declare_parameter("camera_color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("cup_detection_topic", "/azas/cup_detection")
        self.declare_parameter("lid_detection_topic", "/azas/lid_detection")
        self.declare_parameter("hand_overlay_topic", "/azas/human_hand_detection/overlay")
        self.declare_parameter("camera_stream_width_px", 720)
        self.declare_parameter("camera_jpeg_quality", 78)

        self._lock = threading.Lock()
        self._camera_lock = threading.Lock()
        self._events: deque[dict[str, Any]] = deque(maxlen=12)
        self._state = build_initial_state()
        self._camera_frames: dict[str, dict[str, Any]] = {}
        self._detection_status: dict[str, dict[str, Any]] = {
            "cup": {"status": "", "at": 0.0},
            "lid": {"status": "", "at": 0.0},
        }
        self._camera_errors: dict[str, float] = {}
        self._camera_stream_width_px = max(240, int(self.get_parameter("camera_stream_width_px").value))
        self._camera_jpeg_quality = max(35, min(95, int(self.get_parameter("camera_jpeg_quality").value)))

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
        self._start_camera_subscriptions()

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
        payload["camera_status"] = self._camera_status_snapshot()
        return payload

    def camera_jpeg(self, stream: str) -> bytes:
        if stream not in _CAMERA_STREAMS:
            raise ValueError(f"unknown camera stream: {stream}")
        if cv2 is None or np is None:
            raise ValueError("opencv/numpy camera support is not available")

        frame = self._frame_for_stream(stream)
        if frame is None:
            raise ValueError(f"no frame available for camera stream: {stream}")

        if stream == "cup":
            status = self._latest_detection("cup")
            _draw_detection_overlay(frame, status["status"], kind="cup", status_time=status["at"])
        elif stream == "lid":
            status = self._latest_detection("lid")
            _draw_detection_overlay(frame, status["status"], kind="lid", status_time=status["at"])
        elif stream == "hand" and not self._has_recent_frame("hand", max_age_sec=2.0):
            _draw_stream_label(frame, "HAND DETECTION WAITING", "waiting for /azas/human_hand_detection/overlay")
        else:
            _draw_stream_label(frame, _stream_label(stream), "")

        frame = _resize_for_stream(frame, max_width=self._camera_stream_width_px)
        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self._camera_jpeg_quality],
        )
        if not ok:
            raise ValueError("failed to encode camera frame")
        return encoded.tobytes()

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

    def _start_camera_subscriptions(self) -> None:
        if Image is None or cv2 is None or np is None:
            self.get_logger().warn("Camera UI disabled: sensor_msgs, OpenCV, or numpy is unavailable")
            return

        self.create_subscription(
            Image,
            str(self.get_parameter("camera_color_topic").value),
            lambda msg: self._on_camera_image("realsense", msg),
            LOW_LATENCY_IMAGE_QOS,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("hand_overlay_topic").value),
            lambda msg: self._on_camera_image("hand", msg),
            LOW_LATENCY_IMAGE_QOS,
        )
        if CupDetection is None:
            self.get_logger().warn("Camera overlays disabled: azas_interfaces/CupDetection is unavailable")
            return
        self.create_subscription(
            CupDetection,
            str(self.get_parameter("cup_detection_topic").value),
            lambda msg: self._on_detection("cup", msg),
            10,
        )
        self.create_subscription(
            CupDetection,
            str(self.get_parameter("lid_detection_topic").value),
            lambda msg: self._on_detection("lid", msg),
            10,
        )
        self.get_logger().info(
            "Camera UI streams ready: "
            f"color={self.get_parameter('camera_color_topic').value}, "
            f"cup={self.get_parameter('cup_detection_topic').value}, "
            f"lid={self.get_parameter('lid_detection_topic').value}, "
            f"hand={self.get_parameter('hand_overlay_topic').value}"
        )

    def _on_camera_image(self, stream: str, msg: Any) -> None:
        try:
            frame = _image_msg_to_bgr(msg)
        except ValueError as exc:
            now = time.monotonic()
            last = self._camera_errors.get(stream, 0.0)
            if now - last > 2.0:
                self._camera_errors[stream] = now
                self.get_logger().warn(f"{stream} camera frame ignored: {exc}")
            return
        with self._camera_lock:
            self._camera_frames[stream] = {
                "frame": frame,
                "at": time.monotonic(),
                "encoding": str(getattr(msg, "encoding", "")),
                "width": int(getattr(msg, "width", 0) or frame.shape[1]),
                "height": int(getattr(msg, "height", 0) or frame.shape[0]),
            }

    def _on_detection(self, kind: str, msg: Any) -> None:
        with self._camera_lock:
            self._detection_status[kind] = {
                "status": str(getattr(msg, "status", "")),
                "at": time.monotonic(),
            }

    def _frame_for_stream(self, stream: str) -> Any | None:
        source = "hand" if stream == "hand" and self._has_recent_frame("hand", max_age_sec=5.0) else "realsense"
        with self._camera_lock:
            item = self._camera_frames.get(source)
            if item is None:
                return None
            return item["frame"].copy()

    def _latest_detection(self, kind: str) -> dict[str, Any]:
        with self._camera_lock:
            return dict(self._detection_status.get(kind, {"status": "", "at": 0.0}))

    def _has_recent_frame(self, stream: str, *, max_age_sec: float) -> bool:
        with self._camera_lock:
            item = self._camera_frames.get(stream)
            return item is not None and time.monotonic() - float(item["at"]) <= max_age_sec

    def _camera_status_snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._camera_lock:
            frames = {
                name: {
                    "available": True,
                    "age_sec": round(now - float(item["at"]), 3),
                    "width": item.get("width"),
                    "height": item.get("height"),
                    "encoding": item.get("encoding"),
                }
                for name, item in self._camera_frames.items()
            }
            detections = {
                name: {
                    "status": item.get("status", ""),
                    "age_sec": round(now - float(item.get("at", 0.0)), 3)
                    if float(item.get("at", 0.0)) > 0.0
                    else None,
                }
                for name, item in self._detection_status.items()
            }
        return {
            "enabled": Image is not None and cv2 is not None and np is not None,
            "frames": frames,
            "detections": detections,
        }

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
                route = urlparse(self.path).path
                if route in {"/", "/voice.html"}:
                    self._send_file(node._web_root / "voice.html")
                    return
                if route == "/voice.css":
                    self._send_file(node._web_root / "voice.css")
                    return
                if route == "/voice.js":
                    self._send_file(node._web_root / "voice.js")
                    return
                if route == "/api/state":
                    self._send_json(node.snapshot())
                    return
                if route.startswith("/api/camera/") and route.endswith(".jpg"):
                    stream = route.removeprefix("/api/camera/").removesuffix(".jpg")
                    try:
                        data = node.camera_jpeg(stream)
                    except ValueError as exc:
                        self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, str(exc))
                        return
                    self._send_bytes(data, "image/jpeg")
                    return
                self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self) -> None:
                try:
                    route = urlparse(self.path).path
                    payload = self._read_json()
                    if route != "/api/utterance":
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

            def _send_bytes(self, data: bytes, content_type: str) -> None:
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store, max-age=0")
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


def _image_msg_to_bgr(msg: Any) -> Any:
    if cv2 is None or np is None:
        raise ValueError("opencv/numpy are required")
    height = int(getattr(msg, "height", 0))
    width = int(getattr(msg, "width", 0))
    if height <= 0 or width <= 0:
        raise ValueError(f"invalid image dimensions: {width}x{height}")

    encoding = str(getattr(msg, "encoding", "")).lower()
    if encoding in {"bgr8", "rgb8"}:
        rows = _image_rows(msg, bytes_per_pixel=3)
        image = rows.reshape((height, width, 3))
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image.copy()

    if encoding in {"bgra8", "rgba8"}:
        rows = _image_rows(msg, bytes_per_pixel=4)
        image = rows.reshape((height, width, 4))
        code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
        return cv2.cvtColor(image, code)

    if encoding in {"mono8", "8uc1"}:
        rows = _image_rows(msg, bytes_per_pixel=1)
        gray = rows.reshape((height, width))
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    if encoding == "16uc1":
        rows = _image_rows(msg, bytes_per_pixel=2)
        depth = np.ascontiguousarray(rows).view(np.uint16).reshape((height, width))
        return _depth_to_bgr(depth)

    if encoding == "32fc1":
        rows = _image_rows(msg, bytes_per_pixel=4)
        depth = np.ascontiguousarray(rows).view(np.float32).reshape((height, width))
        return _depth_to_bgr(depth)

    raise ValueError(f"unsupported image encoding: {getattr(msg, 'encoding', '')}")


def _image_rows(msg: Any, *, bytes_per_pixel: int) -> Any:
    if np is None:
        raise ValueError("numpy is required")
    height = int(getattr(msg, "height", 0))
    width = int(getattr(msg, "width", 0))
    step = int(getattr(msg, "step", 0)) or width * bytes_per_pixel
    expected = height * step
    raw = np.frombuffer(getattr(msg, "data", b""), dtype=np.uint8)
    if raw.size < expected:
        raise ValueError(f"image buffer too small: {raw.size} < {expected}")
    return np.ascontiguousarray(raw[:expected].reshape((height, step))[:, : width * bytes_per_pixel])


def _depth_to_bgr(depth: Any) -> Any:
    if cv2 is None or np is None:
        raise ValueError("opencv/numpy are required")
    finite = np.asarray(depth, dtype=np.float32)
    finite = np.nan_to_num(finite, nan=0.0, posinf=0.0, neginf=0.0)
    if float(np.max(finite)) <= float(np.min(finite)):
        normalized = np.zeros(finite.shape, dtype=np.uint8)
    else:
        normalized = cv2.normalize(finite, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    return cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)


def _parse_detection_overlay(status: str, *, kind: str) -> dict[str, Any]:
    text = str(status or "")
    center = None
    for pattern in _CENTER_PATTERNS.get(kind, (r"\bcenter=\((\d+),(\d+)\)",)):
        match = re.search(pattern, text)
        if match:
            center = (int(match.group(1)), int(match.group(2)))
            break

    bbox = None
    bbox_match = re.search(r"\bbbox=(\d+)x(\d+)", text)
    if bbox_match:
        bbox = (int(bbox_match.group(1)), int(bbox_match.group(2)))

    orientation = _parse_orientation(text)
    detected = text.startswith("detected:")
    return {
        "text": text,
        "center": center,
        "bbox": bbox,
        "orientation": orientation,
        "detected": detected,
    }


def _parse_orientation(status: str) -> str:
    normalized = str(status or "").lower()
    match = re.search(r"\borientation=([a-z_]+)", normalized)
    if match:
        return match.group(1)
    if normalized.startswith("detected:upright"):
        return "upright"
    if normalized.startswith("rejected:lying"):
        return "lying"
    return ""


def _draw_detection_overlay(frame: Any, status: str, *, kind: str, status_time: float) -> None:
    if cv2 is None:
        return
    parsed = _parse_detection_overlay(status, kind=kind)
    age = time.monotonic() - status_time if status_time > 0.0 else float("inf")
    stale = age > 1.2

    if kind == "cup":
        if parsed["orientation"] == "upright":
            label = "CUP UPRIGHT"
            color = (44, 220, 125)
        elif parsed["orientation"] == "lying":
            label = "CUP LYING"
            color = (0, 176, 255)
        else:
            label = "CUP DETECTION WAITING"
            color = (74, 74, 255)
    elif parsed["detected"]:
        label = "LID DETECTED"
        color = (64, 220, 230)
    else:
        label = "LID DETECTION WAITING"
        color = (74, 74, 255)

    if stale:
        label = f"{label} STALE"
        color = (160, 160, 160)

    center = parsed["center"]
    bbox = parsed["bbox"]
    if center is not None and bbox is not None:
        cx, cy = center
        bw, bh = bbox
        x1 = max(int(cx - bw / 2), 0)
        y1 = max(int(cy - bh / 2), 0)
        x2 = min(int(cx + bw / 2), frame.shape[1] - 1)
        y2 = min(int(cy + bh / 2), frame.shape[0] - 1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        cv2.circle(frame, (cx, cy), 5, color, -1)

    detail = parsed["text"][:96] if parsed["text"] else ""
    _draw_stream_label(frame, label, detail, color=color)


def _draw_stream_label(frame: Any, label: str, detail: str = "", *, color: tuple[int, int, int] = (95, 216, 173)) -> None:
    if cv2 is None:
        return
    height, width = frame.shape[:2]
    box_width = min(width - 24, 700)
    box_height = 78 if detail else 56
    overlay = frame.copy()
    cv2.rectangle(overlay, (12, 12), (12 + box_width, 12 + box_height), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.54, frame, 0.46, 0, frame)
    cv2.putText(frame, label, (26, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.88, color, 2, cv2.LINE_AA)
    if detail:
        cv2.putText(frame, detail, (26, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (235, 235, 235), 1, cv2.LINE_AA)


def _resize_for_stream(frame: Any, *, max_width: int) -> Any:
    if cv2 is None:
        return frame
    height, width = frame.shape[:2]
    if width <= max_width:
        return frame
    scale = max_width / float(width)
    return cv2.resize(frame, (max_width, int(round(height * scale))), interpolation=cv2.INTER_AREA)


def _stream_label(stream: str) -> str:
    return {
        "realsense": "REALSENSE LIVE",
        "cup": "CUP DETECTION",
        "lid": "LID DETECTION",
        "hand": "HAND DETECTION",
    }.get(stream, "CAMERA")


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
