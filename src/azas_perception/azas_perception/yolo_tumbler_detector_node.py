from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from azas_interfaces.msg import CupDetection
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from azas_perception.depth_projection import CameraIntrinsics, pixel_depth_to_camera_point

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - depends on deployment environment
    YOLO = None


@dataclass(frozen=True)
class Detection2D:
    x_min: int
    y_min: int
    x_max: int
    y_max: int
    center_u: int
    center_v: int
    width: int
    height: int
    area: int
    confidence: float
    class_name: str


@dataclass(frozen=True)
class DepthSample:
    raw: float
    meters: float
    scale: float
    encoding: str
    valid_count: int
    total_count: int
    window_size: int


class YoloTumblerDetectorNode(Node):
    """YOLO + aligned depth detector for the Azas tumbler.

    The node publishes a camera-frame CupDetection only. Downstream motion must
    still transform, validate workspace, check collision, and verify gripper fit.
    """

    def __init__(self):
        super().__init__("yolo_tumbler_detector_node")
        self.declare_parameter("model_path", "/home/ssu/Downloads/best.pt")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("target_class", "")
        self.declare_parameter("target_class_names", "cup,tumbler,bottle")
        self.declare_parameter("selection_policy", "largest_bbox")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("source_frame", "camera_color_optical_frame")
        self.declare_parameter("depth_window_size", 7)
        self.declare_parameter("depth_patch_radius_px", 3)
        self.declare_parameter("depth_scale_mode", "auto")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 2.0)
        self.declare_parameter("cup_height_m", 0.17)
        self.declare_parameter("source", "yolo_tumbler_detector")

        self._latest_depth: Optional[np.ndarray] = None
        self._latest_depth_encoding = ""
        self._latest_depth_error = ""
        self._latest_info: Optional[CameraInfo] = None
        self._last_depth_scale_log = ""
        self._model = self._load_model()

        self._pub = self.create_publisher(CupDetection, "/azas/cup_detection", 10)
        self.create_subscription(Image, self.get_parameter("color_topic").value, self._on_color, 10)
        self.create_subscription(Image, self.get_parameter("depth_topic").value, self._on_depth, 10)
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._on_camera_info,
            10,
        )
        self.get_logger().info("YOLO tumbler detector ready")

    def _load_model(self):
        model_path = str(self.get_parameter("model_path").value)
        if YOLO is None:
            self.get_logger().error(
                "ultralytics is not installed; install it before live YOLO detection"
            )
            return None
        try:
            model = YOLO(model_path)
        except Exception as exc:
            self.get_logger().error(f"failed to load YOLO model {model_path}: {exc}")
            return None
        self.get_logger().info(f"loaded YOLO model: {model_path}")
        return model

    def _on_depth(self, msg: Image) -> None:
        encoding = msg.encoding.lower()
        if encoding not in self._auto_depth_scales():
            self._latest_depth = None
            self._latest_depth_encoding = encoding
            self._latest_depth_error = f"unsupported_depth_encoding:{msg.encoding}"
            self.get_logger().error(
                "Rejecting depth image with unsupported encoding "
                f"{msg.encoding}; expected 16UC1, mono16, or 32FC1"
            )
            return
        try:
            self._latest_depth = self._image_to_array(msg)
            self._latest_depth_encoding = encoding
            self._latest_depth_error = ""
        except Exception as exc:
            self._latest_depth = None
            self._latest_depth_encoding = encoding
            self._latest_depth_error = "depth_conversion_failed"
            self.get_logger().error(f"depth conversion failed: {exc}")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._latest_info = msg

    def _on_color(self, msg: Image) -> None:
        if self._model is None:
            self._publish_invalid(msg, "model_not_loaded")
            return
        if self._latest_depth_error:
            self._publish_invalid(msg, self._latest_depth_error)
            return
        if self._latest_depth is None or self._latest_info is None:
            self._publish_invalid(msg, "waiting_for_depth_and_camera_info")
            return

        try:
            image = self._image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f"color conversion failed: {exc}")
            self._publish_invalid(msg, "color_conversion_failed")
            return

        try:
            detection = self._detect_best(image)
        except Exception as exc:
            self.get_logger().error(f"YOLO prediction failed: {exc}")
            self._publish_invalid(msg, "prediction_failed")
            return
        if detection is None:
            self._publish_invalid(msg, "no_tumbler_detection")
            return

        orientation_state = self._classify_cup_orientation(detection.width, detection.height)
        if orientation_state != "upright":
            self.get_logger().warn(
                "Rejecting tumbler detection for side grasp: "
                f"orientation_state={orientation_state} "
                f"bbox={detection.width}x{detection.height} "
                f"aspect_ratio_h_over_w={self._bbox_aspect_ratio(detection.width, detection.height):.3f}; "
                "bbox aspect ratio is a heuristic and does not prove cup pose"
            )
            self._publish_rejected_orientation(msg, detection, orientation_state)
            return

        depth = self._median_depth(detection.center_u, detection.center_v)
        if depth is None:
            self._publish_invalid(msg, "invalid_depth_at_detection")
            return

        info = self._latest_info
        intrinsics = CameraIntrinsics(fx=info.k[0], fy=info.k[4], cx=info.k[2], cy=info.k[5])
        if not self._valid_intrinsics(intrinsics):
            self.get_logger().error(
                "Invalid CameraInfo intrinsics; refusing projection: "
                f"fx={intrinsics.fx} fy={intrinsics.fy} cx={intrinsics.cx} cy={intrinsics.cy}"
            )
            self._publish_invalid(msg, "invalid_camera_info")
            return
        try:
            x, y, z = pixel_depth_to_camera_point(
                detection.center_u,
                detection.center_v,
                float(depth.raw),
                intrinsics,
                depth_scale=depth.scale,
            )
        except ValueError as exc:
            self.get_logger().error(f"depth projection failed: {exc}")
            self._publish_invalid(msg, "invalid_projected_depth")
            return

        self.get_logger().info(
            "Selected target bbox: "
            f"class={detection.class_name} conf={detection.confidence:.3f} "
            f"bbox=({detection.x_min},{detection.y_min})-({detection.x_max},{detection.y_max}) "
            f"center=({detection.center_u},{detection.center_v}) area={detection.area} "
            f"depth_raw_median={depth.raw:.3f} depth_m={depth.meters:.3f} "
            f"depth_encoding={depth.encoding} depth_scale={depth.scale:.6g} "
            f"valid_depth={depth.valid_count}/{depth.total_count} window={depth.window_size}"
        )
        self.get_logger().info(
            "Projected target camera point: "
            f"frame={self._source_frame(info, msg)} "
            f"x={x:.4f} y={y:.4f} z={z:.4f}"
        )

        output = CupDetection()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self._source_frame(info, msg)
        output.grasp_pose = self._pose_at(x, y, z)
        output.cup_mouth_center = self._pose_at(x, y, z + float(self.get_parameter("cup_height_m").value))
        output.confidence = float(detection.confidence)
        output.status = (
            f"detected:upright class={detection.class_name} "
            f"bbox={detection.width}x{detection.height} "
            f"orientation={orientation_state} "
            f"aspect_ratio_h_over_w={self._bbox_aspect_ratio(detection.width, detection.height):.3f} "
            f"center=({detection.center_u},{detection.center_v}) "
            f"area={detection.area} "
            f"depth_raw={depth.raw:.1f} depth_m={depth.meters:.3f} "
            f"depth_encoding={depth.encoding} depth_scale={depth.scale:.6g} "
            f"window={depth.window_size} valid_depth={depth.valid_count}/{depth.total_count}"
        )
        output.source = str(self.get_parameter("source").value)
        self._pub.publish(output)

    def _detect_best(self, image: np.ndarray) -> Optional[Detection2D]:
        threshold = float(self.get_parameter("confidence_threshold").value)
        target_names = self._target_class_names()
        selection_policy = str(self.get_parameter("selection_policy").value).strip().lower()
        device = str(self.get_parameter("device").value).strip() or "cpu"
        results = self._model.predict(image, verbose=False, device=device)
        if not results:
            return None

        names = getattr(results[0], "names", {}) or {}
        best = None
        for box in results[0].boxes:
            confidence = float(box.conf[0])
            if confidence < threshold:
                continue
            class_id = int(box.cls[0])
            class_name = str(names.get(class_id, class_id)).lower()
            if target_names and not any(target in class_name for target in target_names):
                continue
            x1, y1, x2, y2 = [int(round(v)) for v in box.xyxy[0].tolist()]
            width = max(x2 - x1, 0)
            height = max(y2 - y1, 0)
            if width <= 0 or height <= 0:
                continue
            candidate = Detection2D(
                x_min=x1,
                y_min=y1,
                x_max=x2,
                y_max=y2,
                center_u=int(round((x1 + x2) / 2.0)),
                center_v=int(round((y1 + y2) / 2.0)),
                width=width,
                height=height,
                area=width * height,
                confidence=confidence,
                class_name=class_name,
            )
            if self._is_better_detection(candidate, best, selection_policy):
                best = candidate
        return best

    def _median_depth(self, u: int, v: int) -> Optional[DepthSample]:
        depth = self._latest_depth
        if depth is None or depth.size == 0:
            return None
        window_size = self._depth_window_size()
        radius = window_size // 2
        height, width = depth.shape[:2]
        x1, x2 = max(u - radius, 0), min(u + radius + 1, width)
        y1, y2 = max(v - radius, 0), min(v + radius + 1, height)
        patch = np.asarray(depth[y1:y2, x1:x2], dtype=np.float32)
        total_count = int(patch.size)
        finite_positive = patch[np.isfinite(patch) & (patch > 0)]
        if finite_positive.size == 0:
            self.get_logger().warn(
                f"Rejecting detection depth: no finite positive depth in {window_size}x{window_size} "
                f"window around center=({u},{v})"
            )
            return None
        depth_scale = self._depth_scale()
        if depth_scale is None:
            return None
        min_depth_m = float(self.get_parameter("min_depth_m").value)
        max_depth_m = float(self.get_parameter("max_depth_m").value)
        self._log_depth_scale(depth_scale)
        depth_m = finite_positive * depth_scale
        valid = finite_positive[
            np.isfinite(depth_m)
            & (depth_m >= min_depth_m)
            & (depth_m <= max_depth_m)
        ]
        if valid.size == 0:
            observed_min = float(np.min(depth_m))
            observed_max = float(np.max(depth_m))
            self.get_logger().warn(
                "Rejecting detection depth: "
                f"no values in range [{min_depth_m:.3f}, {max_depth_m:.3f}] m "
                f"around center=({u},{v}); observed_m={observed_min:.3f}-{observed_max:.3f}"
            )
            return None
        median_raw = float(np.median(valid))
        return DepthSample(
            raw=median_raw,
            meters=median_raw * depth_scale,
            scale=depth_scale,
            encoding=self._latest_depth_encoding,
            valid_count=int(valid.size),
            total_count=total_count,
            window_size=window_size,
        )

    def _target_class_names(self) -> list[str]:
        legacy_target_class = str(self.get_parameter("target_class").value).strip().lower()
        if legacy_target_class:
            return [legacy_target_class]

        raw = self.get_parameter("target_class_names").value
        if isinstance(raw, str):
            values = raw.replace(";", ",").split(",")
        else:
            values = list(raw)
        return [str(value).strip().lower() for value in values if str(value).strip()]

    @staticmethod
    def _is_better_detection(
        candidate: Detection2D,
        best: Optional[Detection2D],
        selection_policy: str,
    ) -> bool:
        if best is None:
            return True
        if selection_policy == "largest_bbox":
            if candidate.area != best.area:
                return candidate.area > best.area
            return candidate.confidence > best.confidence
        if selection_policy == "highest_confidence":
            if abs(candidate.confidence - best.confidence) > 1e-9:
                return candidate.confidence > best.confidence
            return candidate.area > best.area
        return candidate.area > best.area

    def _depth_window_size(self) -> int:
        configured = int(self.get_parameter("depth_window_size").value)
        if configured <= 0:
            radius = max(int(self.get_parameter("depth_patch_radius_px").value), 0)
            configured = radius * 2 + 1
        configured = max(configured, 1)
        if configured % 2 == 0:
            configured += 1
        return configured

    def _source_frame(self, info: CameraInfo, msg: Image) -> str:
        configured = str(self.get_parameter("source_frame").value).strip()
        return configured or info.header.frame_id or msg.header.frame_id

    @staticmethod
    def _auto_depth_scales() -> dict[str, float]:
        return {
            "16uc1": 0.001,
            "mono16": 0.001,
            "32fc1": 1.0,
        }

    def _depth_scale(self) -> Optional[float]:
        mode = str(self.get_parameter("depth_scale_mode").value).strip().lower()
        encoding = self._latest_depth_encoding
        if mode == "manual":
            depth_scale = float(self.get_parameter("depth_scale").value)
            if depth_scale <= 0:
                self.get_logger().error(
                    f"Rejecting depth projection: manual depth_scale must be positive, got {depth_scale}"
                )
                return None
            return depth_scale
        if mode != "auto":
            self.get_logger().error(
                f"Rejecting depth projection: unsupported depth_scale_mode={mode!r}; use 'auto' or 'manual'"
            )
            return None
        scale = self._auto_depth_scales().get(encoding)
        if scale is None:
            self.get_logger().error(
                f"Rejecting depth projection: unsupported depth encoding {encoding!r} in auto mode"
            )
            return None
        return scale

    def _log_depth_scale(self, depth_scale: float) -> None:
        mode = str(self.get_parameter("depth_scale_mode").value).strip().lower()
        key = f"{self._latest_depth_encoding}:{mode}:{depth_scale:.9g}"
        if key == self._last_depth_scale_log:
            return
        self._last_depth_scale_log = key
        self.get_logger().info(
            "Depth scale selected: "
            f"encoding={self._latest_depth_encoding} mode={mode} scale={depth_scale:.6g}"
        )

    @staticmethod
    def _valid_intrinsics(intrinsics: CameraIntrinsics) -> bool:
        values = (intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy)
        return all(math.isfinite(value) for value in values) and intrinsics.fx > 0 and intrinsics.fy > 0

    @staticmethod
    def _bbox_aspect_ratio(width: int, height: int) -> float:
        if width <= 0:
            return math.inf
        return float(height) / float(width)

    @classmethod
    def _classify_cup_orientation(cls, width: int, height: int) -> str:
        ratio = cls._bbox_aspect_ratio(width, height)
        if ratio >= 1.2:
            return "upright"
        if ratio < 0.8:
            return "lying"
        return "unknown"

    def _publish_rejected_orientation(
        self,
        msg: Image,
        detection: Detection2D,
        orientation_state: str,
    ) -> None:
        output = CupDetection()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self._source_frame(self._latest_info, msg) if self._latest_info else msg.header.frame_id
        output.grasp_pose = Pose()
        output.cup_mouth_center = Pose()
        output.confidence = float(detection.confidence)
        reason = (
            "lying_or_unknown"
            if orientation_state == "lying"
            else "unknown_orientation"
        )
        output.status = (
            f"rejected:{reason} class={detection.class_name} "
            f"orientation={orientation_state} "
            f"bbox={detection.width}x{detection.height} "
            f"aspect_ratio_h_over_w={self._bbox_aspect_ratio(detection.width, detection.height):.3f} "
            "heuristic=bbox_height_over_width"
        )
        output.source = str(self.get_parameter("source").value)
        self._pub.publish(output)

    def _publish_invalid(self, msg: Image, status: str) -> None:
        output = CupDetection()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = msg.header.frame_id
        output.grasp_pose = Pose()
        output.cup_mouth_center = Pose()
        output.confidence = 0.0
        output.status = status
        output.source = str(self.get_parameter("source").value)
        self._pub.publish(output)

    @staticmethod
    def _image_to_array(msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        dtype_by_encoding = {
            "8uc1": np.uint8,
            "mono8": np.uint8,
            "8uc3": np.uint8,
            "rgb8": np.uint8,
            "bgr8": np.uint8,
            "16uc1": np.uint16,
            "mono16": np.uint16,
            "32fc1": np.float32,
        }
        channels_by_encoding = {
            "8uc1": 1,
            "mono8": 1,
            "8uc3": 3,
            "rgb8": 3,
            "bgr8": 3,
            "16uc1": 1,
            "mono16": 1,
            "32fc1": 1,
        }
        if encoding not in dtype_by_encoding:
            raise ValueError(f"unsupported image encoding: {msg.encoding}")

        dtype = dtype_by_encoding[encoding]
        channels = channels_by_encoding[encoding]
        itemsize = np.dtype(dtype).itemsize
        row_values = msg.step // itemsize
        data = np.frombuffer(msg.data, dtype=dtype)
        if msg.is_bigendian != (data.dtype.byteorder == ">"):
            data = data.byteswap().newbyteorder()
        if channels == 1:
            image = data.reshape((msg.height, row_values))[:, : msg.width]
        else:
            image = data.reshape((msg.height, row_values // channels, channels))[:, : msg.width, :]
        return np.ascontiguousarray(image)

    @classmethod
    def _image_to_bgr(cls, msg: Image) -> np.ndarray:
        image = cls._image_to_array(msg)
        encoding = msg.encoding.lower()
        if encoding == "bgr8":
            return image
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding in {"mono8", "8uc1"}:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        raise ValueError(f"unsupported color encoding: {msg.encoding}")

    @staticmethod
    def _pose_at(x: float, y: float, z: float) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = YoloTumblerDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
