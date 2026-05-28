from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from azas_interfaces.msg import CupDetection
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from azas_perception.cup_orientation_classifier import (
    classifier_available,
    crop_detection_bgr,
    load_classifier_checkpoint,
    predict_crop_orientation,
)
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


@dataclass(frozen=True)
class BboxHeightStats:
    median_m: float
    p90_m: float
    max_m: float
    valid_ratio: float
    valid_count: int
    total_count: int
    center_median_m: float = math.nan
    edge_median_m: float = math.nan
    center_valid_ratio: float = 0.0
    edge_valid_ratio: float = 0.0


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
        self.declare_parameter("capture_empty_table_baseline", False)
        self.declare_parameter("baseline_frame_count", 30)
        self.declare_parameter("empty_table_baseline_path", "")
        self.declare_parameter("cup_standing_height_threshold_m", 0.0)
        self.declare_parameter("cup_side_lie_height_threshold_m", 0.0)
        self.declare_parameter("cup_inverted_center_ratio_threshold", 0.0)
        self.declare_parameter("cup_inverted_min_center_height_m", 0.0)
        self.declare_parameter("enable_top_view_upright", False)
        self.declare_parameter("top_view_aspect_min", 0.85)
        self.declare_parameter("top_view_aspect_max", 1.15)
        self.declare_parameter("top_view_guard_aspect_min", 0.70)
        self.declare_parameter("top_view_guard_aspect_max", 1.35)
        self.declare_parameter("height_stat_for_orientation", "p90")
        self.declare_parameter("min_height_valid_ratio", 0.10)
        self.declare_parameter("orientation_classifier_path", "")
        self.declare_parameter("orientation_classifier_arch", "cnn")
        self.declare_parameter("orientation_classifier_min_confidence", 0.70)
        self.declare_parameter("orientation_classifier_device", "cpu")
        self.declare_parameter("orientation_classifier_pad", 0.25)
        self.declare_parameter("orientation_classifier_tall_lie_aspect", 1.35)
        self.declare_parameter("orientation_classifier_tall_lie_height_threshold_m", 0.09)
        self.declare_parameter("source", "yolo_tumbler_detector")

        self._latest_depth: Optional[np.ndarray] = None
        self._latest_depth_encoding = ""
        self._latest_depth_error = ""
        self._latest_info: Optional[CameraInfo] = None
        self._last_depth_scale_log = ""
        self._baseline_frames: list[np.ndarray] = []
        self._table_depth_map: Optional[np.ndarray] = None
        self._baseline_ready_logged = False
        self._load_empty_table_baseline()
        self._model = self._load_model()
        self._orientation_classifier_model = None
        self._orientation_classifier_class_names: list[str] = []
        self._orientation_classifier_image_size = 0
        self._load_orientation_classifier()

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

    def _load_orientation_classifier(self) -> None:
        model_path = str(self.get_parameter("orientation_classifier_path").value).strip()
        if not classifier_available(model_path):
            if model_path:
                self.get_logger().warn(f"orientation classifier path does not exist: {model_path}")
            return
        device = str(self.get_parameter("orientation_classifier_device").value).strip() or "cpu"
        arch = str(self.get_parameter("orientation_classifier_arch").value).strip() or "cnn"
        try:
            model, class_names, image_size = load_classifier_checkpoint(
                model_path,
                device=device,
                arch=arch,
            )
        except Exception as exc:
            self.get_logger().error(f"failed to load orientation classifier {model_path}: {exc}")
            return
        self._orientation_classifier_model = model
        self._orientation_classifier_class_names = class_names
        self._orientation_classifier_image_size = image_size
        self.get_logger().info(
            "loaded orientation classifier: "
            f"{model_path} arch={arch} classes={class_names} image_size={image_size} device={device}"
        )

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
            self._capture_empty_table_baseline()
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

        height_stats = self._bbox_height_stats(detection)
        orientation_state, orientation_detail = self._classify_detection_orientation(
            image,
            detection,
            detection.width,
            detection.height,
            height_stats,
        )
        if orientation_state != "upright":
            self.get_logger().warn(
                "Rejecting tumbler detection for side grasp: "
                f"orientation_state={orientation_state} "
                f"bbox={detection.width}x{detection.height} "
                f"aspect_ratio_h_over_w={self._bbox_aspect_ratio(detection.width, detection.height):.3f} "
                f"{orientation_detail} "
                f"{self._height_stats_status(height_stats)}; "
                "orientation is a perception heuristic and does not prove cup pose"
            )
            self._publish_rejected_orientation(
                msg,
                detection,
                orientation_state,
                height_stats,
                orientation_detail,
            )
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
            f"{orientation_detail} "
            f"{self._height_stats_status(height_stats)} "
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
            f"{orientation_detail} "
            f"{self._height_stats_status(height_stats)} "
            f"depth_raw={depth.raw:.1f} depth_m={depth.meters:.3f} "
            f"depth_encoding={depth.encoding} depth_scale={depth.scale:.6g} "
            f"window={depth.window_size} valid_depth={depth.valid_count}/{depth.total_count}"
        )
        output.source = str(self.get_parameter("source").value)
        self._pub.publish(output)

    def _capture_empty_table_baseline(self) -> None:
        if not bool(self.get_parameter("capture_empty_table_baseline").value):
            return
        if self._table_depth_map is not None or self._latest_depth is None:
            return

        target_count = max(int(self.get_parameter("baseline_frame_count").value), 1)
        depth_scale = self._depth_scale()
        if depth_scale is None:
            return
        frame_m = np.asarray(self._latest_depth, dtype=np.float32) * depth_scale
        finite_positive = np.isfinite(frame_m) & (frame_m > 0)
        if not np.any(finite_positive):
            return
        self._baseline_frames.append(np.where(finite_positive, frame_m, np.nan))
        self.get_logger().info(
            f"Capturing empty-table depth baseline: {len(self._baseline_frames)}/{target_count}"
        )
        if len(self._baseline_frames) < target_count:
            return

        with np.errstate(all="ignore"):
            self._table_depth_map = np.nanmedian(np.stack(self._baseline_frames, axis=0), axis=0)
        self._baseline_frames = []
        baseline_path = str(self.get_parameter("empty_table_baseline_path").value).strip()
        if baseline_path:
            try:
                np.save(baseline_path, self._table_depth_map)
                self.get_logger().info(f"Saved empty-table depth baseline: {baseline_path}")
            except Exception as exc:
                self.get_logger().error(f"Failed to save empty-table depth baseline {baseline_path}: {exc}")
        if not self._baseline_ready_logged:
            self._baseline_ready_logged = True
            self.get_logger().info("Empty-table depth baseline ready")

    def _load_empty_table_baseline(self) -> None:
        if bool(self.get_parameter("capture_empty_table_baseline").value):
            return
        baseline_path = str(self.get_parameter("empty_table_baseline_path").value).strip()
        if not baseline_path:
            return
        path = Path(baseline_path)
        if not path.exists():
            self.get_logger().warn(f"Empty-table depth baseline path does not exist: {baseline_path}")
            return
        try:
            table_depth_map = np.load(path)
        except Exception as exc:
            self.get_logger().error(f"Failed to load empty-table depth baseline {baseline_path}: {exc}")
            return
        if table_depth_map.ndim < 2:
            self.get_logger().error(f"Invalid empty-table depth baseline shape: {table_depth_map.shape}")
            return
        self._table_depth_map = np.asarray(table_depth_map, dtype=np.float32)
        self._baseline_ready_logged = True
        self.get_logger().info(f"Empty-table depth baseline ready: loaded {baseline_path}")

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

    def _bbox_height_stats(self, detection: Detection2D) -> Optional[BboxHeightStats]:
        depth = self._latest_depth
        table_depth_map = self._table_depth_map
        if depth is None or table_depth_map is None:
            return None
        depth_scale = self._depth_scale()
        if depth_scale is None:
            return None
        return self._height_stats_from_depth_maps(
            table_depth_map,
            np.asarray(depth, dtype=np.float32) * depth_scale,
            detection,
            min_height_m=0.0,
        )

    @staticmethod
    def _height_stats_from_depth_maps(
        table_depth_map_m: np.ndarray,
        object_depth_map_m: np.ndarray,
        detection: Detection2D,
        min_height_m: float = 0.0,
    ) -> Optional[BboxHeightStats]:
        if table_depth_map_m.shape[:2] != object_depth_map_m.shape[:2]:
            return None
        height, width = object_depth_map_m.shape[:2]
        x1 = max(min(detection.x_min, width), 0)
        x2 = max(min(detection.x_max, width), 0)
        y1 = max(min(detection.y_min, height), 0)
        y2 = max(min(detection.y_max, height), 0)
        total_count = max((x2 - x1) * (y2 - y1), 0)
        if total_count <= 0:
            return None

        table_patch = np.asarray(table_depth_map_m[y1:y2, x1:x2], dtype=np.float32)
        object_patch = np.asarray(object_depth_map_m[y1:y2, x1:x2], dtype=np.float32)
        height_patch = table_patch - object_patch
        valid = height_patch[
            np.isfinite(table_patch)
            & np.isfinite(object_patch)
            & np.isfinite(height_patch)
            & (table_patch > 0)
            & (object_patch > 0)
            & (height_patch >= min_height_m)
        ]
        if valid.size == 0:
            return BboxHeightStats(
                median_m=math.nan,
                p90_m=math.nan,
                max_m=math.nan,
                valid_ratio=0.0,
                valid_count=0,
                total_count=total_count,
            )
        center_valid, center_total, edge_valid, edge_total = (
            YoloTumblerDetectorNode._center_edge_height_values(height_patch, table_patch, object_patch)
        )
        return BboxHeightStats(
            median_m=float(np.median(valid)),
            p90_m=float(np.percentile(valid, 90)),
            max_m=float(np.max(valid)),
            valid_ratio=float(valid.size) / float(total_count),
            valid_count=int(valid.size),
            total_count=total_count,
            center_median_m=float(np.median(center_valid)) if center_valid.size else math.nan,
            edge_median_m=float(np.median(edge_valid)) if edge_valid.size else math.nan,
            center_valid_ratio=float(center_valid.size) / float(center_total) if center_total > 0 else 0.0,
            edge_valid_ratio=float(edge_valid.size) / float(edge_total) if edge_total > 0 else 0.0,
        )

    @staticmethod
    def _center_edge_height_values(
        height_patch: np.ndarray,
        table_patch: np.ndarray,
        object_patch: np.ndarray,
    ) -> tuple[np.ndarray, int, np.ndarray, int]:
        patch_height, patch_width = height_patch.shape[:2]
        if patch_height <= 0 or patch_width <= 0:
            empty = np.asarray([], dtype=np.float32)
            return empty, 0, empty, 0
        x_margin = max(int(round(patch_width * 0.25)), 1)
        y_margin = max(int(round(patch_height * 0.25)), 1)
        center_mask = np.zeros(height_patch.shape[:2], dtype=bool)
        center_mask[
            y_margin : max(patch_height - y_margin, y_margin),
            x_margin : max(patch_width - x_margin, x_margin),
        ] = True
        if not np.any(center_mask):
            center_mask[patch_height // 2, patch_width // 2] = True
        edge_mask = ~center_mask
        valid_mask = (
            np.isfinite(table_patch)
            & np.isfinite(object_patch)
            & np.isfinite(height_patch)
            & (table_patch > 0)
            & (object_patch > 0)
            & (height_patch >= 0.0)
        )
        center_values = height_patch[center_mask & valid_mask]
        edge_values = height_patch[edge_mask & valid_mask]
        return (
            np.asarray(center_values, dtype=np.float32),
            int(np.count_nonzero(center_mask)),
            np.asarray(edge_values, dtype=np.float32),
            int(np.count_nonzero(edge_mask)),
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

    def _classify_detection_orientation(
        self,
        image_bgr: np.ndarray,
        detection: Detection2D,
        width: int,
        height: int,
        height_stats: Optional[BboxHeightStats],
    ) -> tuple[str, str]:
        bbox_orientation = self._classify_cup_orientation(width, height)
        height_orientation = self._classify_height_orientation(
            height_stats,
            standing_threshold_m=float(self.get_parameter("cup_standing_height_threshold_m").value),
            lying_threshold_m=float(self.get_parameter("cup_side_lie_height_threshold_m").value),
            inverted_center_ratio_threshold=float(
                self.get_parameter("cup_inverted_center_ratio_threshold").value
            ),
            inverted_min_center_height_m=float(
                self.get_parameter("cup_inverted_min_center_height_m").value
            ),
            stat_name=str(self.get_parameter("height_stat_for_orientation").value),
            min_valid_ratio=float(self.get_parameter("min_height_valid_ratio").value),
        )
        aspect_ratio = self._bbox_aspect_ratio(width, height)
        classifier_orientation = self._classify_crop_orientation(
            image_bgr,
            detection,
            aspect_ratio=aspect_ratio,
            height_orientation=height_orientation,
            height_stats=height_stats,
        )
        if classifier_orientation is not None:
            return classifier_orientation
        top_view_aspect_min = float(self.get_parameter("top_view_aspect_min").value)
        top_view_aspect_max = float(self.get_parameter("top_view_aspect_max").value)
        top_view_upright = (
            bool(self.get_parameter("enable_top_view_upright").value)
            and self._is_top_view_upright_candidate(
                aspect_ratio,
                height_orientation,
                aspect_min=top_view_aspect_min,
                aspect_max=top_view_aspect_max,
                guard_aspect_min=float(self.get_parameter("top_view_guard_aspect_min").value),
                guard_aspect_max=float(self.get_parameter("top_view_guard_aspect_max").value),
            )
        )
        low_height_lie_candidate = (
            aspect_ratio < top_view_aspect_min or aspect_ratio > top_view_aspect_max
        )
        orientation = self._combine_bbox_and_height_orientation(
            bbox_orientation,
            height_orientation,
            top_view_upright=top_view_upright,
            low_height_lie_candidate=low_height_lie_candidate,
        )
        return (
            orientation,
            f"orientation_classifier=disabled "
            f"bbox_orientation={bbox_orientation} height_orientation={height_orientation or 'unavailable'}",
        )

    def _classify_crop_orientation(
        self,
        image_bgr: np.ndarray,
        detection: Detection2D,
        aspect_ratio: float,
        height_orientation: Optional[str],
        height_stats: Optional[BboxHeightStats],
    ) -> Optional[tuple[str, str]]:
        if self._orientation_classifier_model is None:
            return None
        device = str(self.get_parameter("orientation_classifier_device").value).strip() or "cpu"
        min_confidence = float(self.get_parameter("orientation_classifier_min_confidence").value)
        pad = float(self.get_parameter("orientation_classifier_pad").value)
        try:
            crop = crop_detection_bgr(
                image_bgr,
                center_u=detection.center_u,
                center_v=detection.center_v,
                width=detection.width,
                height=detection.height,
                pad=pad,
            )
            label, confidence = predict_crop_orientation(
                self._orientation_classifier_model,
                self._orientation_classifier_class_names,
                crop,
                image_size=self._orientation_classifier_image_size,
                device=device,
            )
        except Exception as exc:
            self.get_logger().error(f"orientation classifier inference failed: {exc}")
            return "unknown", "orientation_classifier=error"
        detail = f"orientation_classifier={label} orientation_classifier_confidence={confidence:.3f}"
        if confidence < min_confidence:
            return "unknown", f"{detail} orientation_classifier_result=below_threshold"
        if label == "upright":
            tall_lie_aspect = float(self.get_parameter("orientation_classifier_tall_lie_aspect").value)
            tall_lie_height_threshold_m = float(
                self.get_parameter("orientation_classifier_tall_lie_height_threshold_m").value
            )
            height_p90_m = height_stats.p90_m if height_stats is not None else math.nan
            tall_lie_height = (
                math.isfinite(height_p90_m)
                and tall_lie_height_threshold_m > 0.0
                and height_p90_m <= tall_lie_height_threshold_m
            )
            if aspect_ratio >= tall_lie_aspect and (
                height_orientation == "lying" or tall_lie_height
            ):
                return (
                    "lying",
                    f"{detail} orientation_classifier_result=height_aspect_safety_override "
                    f"height_orientation={height_orientation or 'unavailable'} "
                    f"height_p90={height_p90_m:.3f} "
                    f"tall_lie_aspect={tall_lie_aspect:.3f} "
                    f"tall_lie_height_threshold_m={tall_lie_height_threshold_m:.3f}",
                )
            return "upright", f"{detail} orientation_classifier_result=accepted"
        if label == "lying":
            return "lying", f"{detail} orientation_classifier_result=accepted"
        return "unknown", f"{detail} orientation_classifier_result=unknown_label"

    @staticmethod
    def _is_top_view_upright_candidate(
        aspect_ratio: float,
        height_orientation: Optional[str],
        aspect_min: float,
        aspect_max: float,
        guard_aspect_min: float,
        guard_aspect_max: float,
    ) -> bool:
        if not math.isfinite(aspect_ratio):
            return False
        if aspect_min <= aspect_ratio <= aspect_max:
            return True
        return (
            height_orientation == "upright"
            and guard_aspect_min <= aspect_ratio <= guard_aspect_max
        )

    @staticmethod
    def _combine_bbox_and_height_orientation(
        bbox_orientation: str,
        height_orientation: Optional[str],
        top_view_upright: bool = False,
        low_height_lie_candidate: bool = False,
    ) -> str:
        if bbox_orientation == "lying":
            return "lying"
        if height_orientation == "inverted":
            return "inverted"
        if height_orientation == "lying":
            if bbox_orientation == "upright":
                return "lying"
            if top_view_upright:
                return "upright"
            return "lying" if low_height_lie_candidate else "unknown"
        if bbox_orientation == "upright":
            if height_orientation == "unknown":
                return "unknown"
            return "upright"
        if top_view_upright:
            return "upright"
        if bbox_orientation == "unknown" and height_orientation == "upright":
            return "upright"
        return "unknown"

    @staticmethod
    def _classify_height_orientation(
        height_stats: Optional[BboxHeightStats],
        standing_threshold_m: float,
        lying_threshold_m: float,
        inverted_center_ratio_threshold: float,
        inverted_min_center_height_m: float,
        stat_name: str,
        min_valid_ratio: float,
    ) -> Optional[str]:
        if (
            height_stats is None
            or standing_threshold_m <= 0.0
            or lying_threshold_m <= 0.0
            or height_stats.valid_ratio < min_valid_ratio
        ):
            return None
        value = YoloTumblerDetectorNode._height_stat_value(height_stats, stat_name)
        if not math.isfinite(value):
            return None
        if value >= standing_threshold_m:
            if YoloTumblerDetectorNode._is_inverted_height_pattern(
                height_stats,
                inverted_center_ratio_threshold,
                inverted_min_center_height_m,
            ):
                return "inverted"
            return "upright"
        if value <= lying_threshold_m:
            return "lying"
        return "unknown"

    @staticmethod
    def _is_inverted_height_pattern(
        height_stats: BboxHeightStats,
        center_ratio_threshold: float,
        min_center_height_m: float,
    ) -> bool:
        if center_ratio_threshold <= 0.0 or min_center_height_m <= 0.0:
            return False
        if not math.isfinite(height_stats.center_median_m) or not math.isfinite(height_stats.p90_m):
            return False
        if height_stats.p90_m <= 0.0:
            return False
        center_ratio = height_stats.center_median_m / height_stats.p90_m
        return (
            height_stats.center_median_m >= min_center_height_m
            and center_ratio >= center_ratio_threshold
        )

    @staticmethod
    def _height_stat_value(height_stats: BboxHeightStats, stat_name: str) -> float:
        normalized = stat_name.strip().lower()
        if normalized in {"median", "height_median"}:
            return height_stats.median_m
        if normalized in {"max", "height_max"}:
            return height_stats.max_m
        return height_stats.p90_m

    @staticmethod
    def _height_stats_status(height_stats: Optional[BboxHeightStats]) -> str:
        if height_stats is None:
            return (
                "table_height_m=unavailable height_median=nan height_p90=nan height_max=nan "
                "height_valid_ratio=0.000 center_height_median=nan edge_height_median=nan "
                "center_edge_height_ratio=nan"
            )
        center_edge_ratio = math.nan
        if math.isfinite(height_stats.center_median_m) and math.isfinite(height_stats.edge_median_m):
            center_edge_ratio = height_stats.center_median_m / max(height_stats.edge_median_m, 1e-9)
        return (
            f"table_height_m={height_stats.p90_m:.3f} "
            f"height_median={height_stats.median_m:.3f} "
            f"height_p90={height_stats.p90_m:.3f} "
            f"height_max={height_stats.max_m:.3f} "
            f"height_valid_ratio={height_stats.valid_ratio:.3f} "
            f"center_height_median={height_stats.center_median_m:.3f} "
            f"edge_height_median={height_stats.edge_median_m:.3f} "
            f"center_height_valid_ratio={height_stats.center_valid_ratio:.3f} "
            f"edge_height_valid_ratio={height_stats.edge_valid_ratio:.3f} "
            f"center_edge_height_ratio={center_edge_ratio:.3f} "
            f"height_valid={height_stats.valid_count}/{height_stats.total_count}"
        )

    def _publish_rejected_orientation(
        self,
        msg: Image,
        detection: Detection2D,
        orientation_state: str,
        height_stats: Optional[BboxHeightStats],
        orientation_detail: str,
    ) -> None:
        output = CupDetection()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self._source_frame(self._latest_info, msg) if self._latest_info else msg.header.frame_id
        output.grasp_pose = Pose()
        output.cup_mouth_center = Pose()
        output.confidence = float(detection.confidence)
        reason = "lying" if orientation_state == "lying" else "unknown_orientation"
        output.status = (
            f"rejected:{reason} class={detection.class_name} "
            f"orientation={orientation_state} "
            f"bbox={detection.width}x{detection.height} "
            f"aspect_ratio_h_over_w={self._bbox_aspect_ratio(detection.width, detection.height):.3f} "
            f"center=({detection.center_u},{detection.center_v}) "
            f"area={detection.area} "
            f"{orientation_detail} "
            f"{self._height_stats_status(height_stats)} "
            "heuristic=height_stats_with_bbox_fallback"
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
