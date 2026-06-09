from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from azas_interfaces.msg import CupDetection
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from azas_perception.depth_projection import CameraIntrinsics, pixel_depth_to_camera_point
from azas_perception.lid_marker import (
    ArucoMarker,
    ImageRoi,
    RedCircle,
    RedCircleConfig,
    detect_aruco_marker,
    detect_red_circle_marker,
    padded_roi,
    quaternion_from_lid_normal,
)
from azas_perception.yolo_tumbler_detector_node import DepthSample, YoloTumblerDetectorNode

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - depends on deployment environment
    YOLO = None


@dataclass(frozen=True)
class LidDetection2D:
    roi: ImageRoi
    center_u: int
    center_v: int
    area: int
    confidence: float
    class_name: str


@dataclass(frozen=True)
class PlaneEstimate:
    normal: np.ndarray
    point_count: int
    rmse_m: float


def _default_model_path() -> str:
    env_path = os.environ.get("AZAS_YOLO_MODEL_PATH") or os.environ.get("MODEL_PATH")
    candidates = [
        env_path,
        "/home/ssu/Downloads/best.pt",
        str(
            Path(__file__).resolve().parents[3]
            / "src"
            / "cocktail_robot_system"
            / "models"
            / "best.pt"
        ),
        "/home/ssu/ros2_ws/src/cocktail_robot_system/src/cocktail_robot_system/models/best.pt",
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return str(candidate)
    return "/home/ssu/Downloads/best.pt"


class LidStickerDetectorNode(Node):
    """Detect a trained lid and refine its grip pose from a center marker.

    This node publishes a camera-frame detection only. It never commands the
    robot or gripper. An ArUco or red marker supplies the lid center; a local
    depth plane around that center supplies the grip approach angle.
    """

    def __init__(self):
        super().__init__("lid_sticker_detector_node")
        self.declare_parameter("model_path", _default_model_path())
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("output_topic", "/azas/lid_detection")
        self.declare_parameter("grip_request_topic", "/jarvis/lid_gripper/grip_request")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("target_class_names", "lid")
        self.declare_parameter("selection_policy", "highest_confidence")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("source_frame", "camera_color_optical_frame")
        self.declare_parameter("depth_window_size", 7)
        self.declare_parameter("depth_scale_mode", "auto")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 2.0)
        self.declare_parameter("source", "lid_aruco_detector")
        self.declare_parameter("marker_type", "aruco")
        self.declare_parameter("require_lid_detection", True)
        self.declare_parameter("allow_aruco_only_after_grip_request", True)
        self.declare_parameter("aruco_only_after_grip_request_sec", 20.0)
        self.declare_parameter("roi_padding_ratio", 0.12)
        self.declare_parameter("require_red_marker", True)
        self.declare_parameter("red_min_area_px", 80.0)
        self.declare_parameter("red_min_radius_px", 4.0)
        self.declare_parameter("red_min_circularity", 0.65)
        self.declare_parameter("red_min_saturation", 80)
        self.declare_parameter("red_min_value", 40)
        self.declare_parameter("red_morph_kernel_px", 3)
        self.declare_parameter("aruco_dictionary", "DICT_6X6_250")
        self.declare_parameter("aruco_marker_id", 0)
        self.declare_parameter("aruco_fallback_markers", "DICT_4X4_50:14")
        self.declare_parameter("aruco_marker_length_m", 0.03)
        self.declare_parameter("use_aruco_axis_for_orientation", True)
        self.declare_parameter("aruco_finger_axis_quarter_turns", 1)
        self.declare_parameter("require_plane_normal", True)
        self.declare_parameter("plane_patch_radius_px", 18)
        self.declare_parameter("plane_sample_stride_px", 2)
        self.declare_parameter("min_plane_points", 20)
        self.declare_parameter("max_plane_rmse_m", 0.015)
        self.declare_parameter("log_detections", False)
        self.declare_parameter("show_preview", True)
        self.declare_parameter("preview_window_name", "Azas Lid Detection - p grip, ESC quit")
        self.declare_parameter("preview_wait_ms", 1)

        self._latest_depth: Optional[np.ndarray] = None
        self._latest_depth_encoding = ""
        self._latest_depth_error = ""
        self._latest_info: Optional[CameraInfo] = None
        self._last_depth_scale_log = ""
        self._latest_valid_status = ""
        self._latest_valid_stamp = None
        self._last_accepted_grip_request_time: float | None = None
        self._preview_window_created = False
        self._model = self._load_model()

        self._pub = self.create_publisher(
            CupDetection,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self._grip_request_pub = self.create_publisher(
            String,
            str(self.get_parameter("grip_request_topic").value),
            10,
        )
        self.create_subscription(Image, self.get_parameter("color_topic").value, self._on_color, 10)
        self.create_subscription(Image, self.get_parameter("depth_topic").value, self._on_depth, 10)
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._on_camera_info,
            10,
        )
        self.get_logger().info("Lid sticker detector ready")

    def _load_model(self):
        if not bool(self.get_parameter("require_lid_detection").value):
            return None
        model_path = str(self.get_parameter("model_path").value)
        if YOLO is None:
            self.get_logger().error(
                "ultralytics is not installed; install it before live lid detection"
            )
            return None
        try:
            model = YOLO(model_path)
        except Exception as exc:
            self.get_logger().error(f"failed to load YOLO model {model_path}: {exc}")
            return None
        self.get_logger().info(f"loaded YOLO lid model: {model_path}")
        return model

    def _on_depth(self, msg: Image) -> None:
        encoding = msg.encoding.lower()
        if encoding not in YoloTumblerDetectorNode._auto_depth_scales():
            self._latest_depth = None
            self._latest_depth_encoding = encoding
            self._latest_depth_error = f"unsupported_depth_encoding:{msg.encoding}"
            self.get_logger().error(
                "Rejecting depth image with unsupported encoding "
                f"{msg.encoding}; expected 16UC1, mono16, or 32FC1"
            )
            return
        try:
            self._latest_depth = YoloTumblerDetectorNode._image_to_array(msg)
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
        try:
            image = YoloTumblerDetectorNode._image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f"color conversion failed: {exc}")
            self._publish_invalid(msg, "color_conversion_failed")
            return

        if self._latest_depth_error:
            self._publish_invalid(msg, self._latest_depth_error)
            self._show_preview(image, self._latest_depth_error)
            return
        if self._latest_depth is None or self._latest_info is None:
            self._publish_invalid(msg, "waiting_for_depth_and_camera_info")
            self._show_preview(image, "waiting_for_depth_and_camera_info")
            return

        lid = self._detect_lid_if_required(image, msg)
        if lid is False:
            return

        marker_roi = self._marker_roi(image, lid)
        marker = self._detect_marker(image, marker_roi)
        if marker is None:
            status = "no_aruco_marker" if self._marker_type() == "aruco" else "no_red_sticker_marker"
            self._publish_invalid(msg, status)
            self._show_preview(image, status, lid=lid if isinstance(lid, LidDetection2D) else None)
            return

        depth = self._median_depth(marker.center_u, marker.center_v)
        if depth is None:
            self._publish_invalid(msg, "invalid_depth_at_marker")
            self._show_preview(
                image,
                "invalid_depth_at_marker",
                lid=lid if isinstance(lid, LidDetection2D) else None,
                marker=marker,
            )
            return

        info = self._latest_info
        intrinsics = CameraIntrinsics(fx=info.k[0], fy=info.k[4], cx=info.k[2], cy=info.k[5])
        if not YoloTumblerDetectorNode._valid_intrinsics(intrinsics):
            self.get_logger().error(
                "Invalid CameraInfo intrinsics; refusing lid projection: "
                f"fx={intrinsics.fx} fy={intrinsics.fy} cx={intrinsics.cx} cy={intrinsics.cy}"
            )
            self._publish_invalid(msg, "invalid_camera_info")
            self._show_preview(
                image,
                "invalid_camera_info",
                lid=lid if isinstance(lid, LidDetection2D) else None,
                marker=marker,
            )
            return

        try:
            x, y, z = pixel_depth_to_camera_point(
                marker.center_u,
                marker.center_v,
                float(depth.raw),
                intrinsics,
                depth_scale=depth.scale,
            )
        except ValueError as exc:
            self.get_logger().error(f"lid marker depth projection failed: {exc}")
            self._publish_invalid(msg, "invalid_projected_depth")
            self._show_preview(
                image,
                "invalid_projected_depth",
                lid=lid if isinstance(lid, LidDetection2D) else None,
                marker=marker,
            )
            return

        plane = self._estimate_plane(marker.center_u, marker.center_v, intrinsics, depth.scale)
        if plane is None:
            if bool(self.get_parameter("require_plane_normal").value):
                self._publish_invalid(msg, "invalid_lid_plane")
                self._show_preview(
                    image,
                    "invalid_lid_plane",
                    lid=lid if isinstance(lid, LidDetection2D) else None,
                    marker=marker,
                    depth_m=depth.meters,
                )
                return
            normal = np.array([0.0, 0.0, -1.0], dtype=float)
            plane_points = 0
            plane_rmse = math.nan
        else:
            normal = plane.normal
            plane_points = plane.point_count
            plane_rmse = plane.rmse_m

        finger_axis_hint = self._finger_axis_hint(marker, intrinsics)
        try:
            qx, qy, qz, qw = quaternion_from_lid_normal(normal, finger_axis_hint)
        except ValueError as exc:
            self.get_logger().error(f"lid orientation construction failed: {exc}")
            self._publish_invalid(msg, "invalid_lid_orientation")
            self._show_preview(
                image,
                "invalid_lid_orientation",
                lid=lid if isinstance(lid, LidDetection2D) else None,
                marker=marker,
                depth_m=depth.meters,
            )
            return

        lid_confidence = float(lid.confidence) if isinstance(lid, LidDetection2D) else 1.0
        output = CupDetection()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self._source_frame(info, msg)
        output.grasp_pose = self._pose_at(x, y, z, qx, qy, qz, qw)
        output.cup_mouth_center = output.grasp_pose
        output.confidence = lid_confidence
        output.status = self._detected_status(
            lid=lid if isinstance(lid, LidDetection2D) else None,
            marker=marker,
            depth=depth,
            normal=normal,
            plane_points=plane_points,
            plane_rmse=plane_rmse,
            finger_axis_hint=finger_axis_hint,
        )
        output.source = str(self.get_parameter("source").value)
        self._pub.publish(output)
        self._latest_valid_status = output.status
        self._latest_valid_stamp = output.header.stamp
        if bool(self.get_parameter("log_detections").value):
            self.get_logger().info(
                "Published lid marker detection: "
                f"frame={output.header.frame_id} x={x:.4f} y={y:.4f} z={z:.4f} "
                f"normal=({normal[0]:.3f},{normal[1]:.3f},{normal[2]:.3f}) "
                f"conf={lid_confidence:.3f}"
            )
        self._show_preview(
            image,
            output.status,
            lid=lid if isinstance(lid, LidDetection2D) else None,
            marker=marker,
            depth_m=depth.meters,
            valid=True,
        )

    def _detect_lid_if_required(self, image: np.ndarray, msg: Image) -> LidDetection2D | bool | None:
        require_lid = bool(self.get_parameter("require_lid_detection").value)
        if self._model is None:
            if require_lid:
                self._publish_invalid(msg, "model_not_loaded")
                self._show_preview(image, "model_not_loaded")
                return False
            return None
        try:
            lid = self._detect_best_lid(image)
        except Exception as exc:
            self.get_logger().error(f"YOLO lid prediction failed: {exc}")
            if require_lid:
                self._publish_invalid(msg, "prediction_failed")
                self._show_preview(image, "prediction_failed")
                return False
            return None
        if lid is None and require_lid:
            if self._aruco_only_after_grip_request_active():
                return None
            self._publish_invalid(msg, "no_lid_detection")
            self._show_preview(image, "no_lid_detection")
            return False
        return lid

    def _marker_roi(self, image: np.ndarray, lid: LidDetection2D | bool | None) -> ImageRoi:
        if isinstance(lid, LidDetection2D):
            return padded_roi(
                lid.roi,
                image_width=image.shape[1],
                image_height=image.shape[0],
                padding_ratio=float(self.get_parameter("roi_padding_ratio").value),
            )
        return ImageRoi(0, 0, image.shape[1], image.shape[0])

    def _detect_marker(self, image: np.ndarray, marker_roi: ImageRoi) -> ArucoMarker | RedCircle | None:
        marker_type = self._marker_type()
        if marker_type == "aruco":
            for dictionary_name, marker_id in self._aruco_marker_candidates():
                marker = detect_aruco_marker(
                    image,
                    marker_roi,
                    dictionary_name=dictionary_name,
                    marker_id=marker_id,
                )
                if marker is not None:
                    if (
                        dictionary_name != str(self.get_parameter("aruco_dictionary").value)
                        or marker_id != int(self.get_parameter("aruco_marker_id").value)
                    ):
                        self.get_logger().info(
                            f"Detected fallback ArUco marker dictionary={dictionary_name} id={marker_id}"
                        )
                    return marker
            return None
        if marker_type == "red":
            marker = detect_red_circle_marker(image, marker_roi, self._red_circle_config())
            if marker is not None:
                return marker
            if not bool(self.get_parameter("require_red_marker").value):
                return RedCircle(
                    center_u=int(round((marker_roi.x_min + marker_roi.x_max) / 2.0)),
                    center_v=int(round((marker_roi.y_min + marker_roi.y_max) / 2.0)),
                    radius_px=0.0,
                    area_px=0.0,
                    circularity=0.0,
                )
        return None

    def _aruco_marker_candidates(self) -> list[tuple[str, int]]:
        """Primary configured ArUco marker followed by explicit fallbacks.

        The lab has used both IsaacSim-style DICT_6X6_250/id0 and the earlier
        lid-closing setup DICT_4X4_50/id14.  Try only configured pairs so the
        detector remains strict and does not accept arbitrary table markers.
        """
        primary = (
            str(self.get_parameter("aruco_dictionary").value).strip() or "DICT_6X6_250",
            int(self.get_parameter("aruco_marker_id").value),
        )
        candidates: list[tuple[str, int]] = [primary]
        raw = str(self.get_parameter("aruco_fallback_markers").value or "").strip()
        for item in raw.replace(";", ",").split(","):
            token = item.strip()
            if not token:
                continue
            if ":" in token:
                dictionary_name, marker_id_text = token.rsplit(":", 1)
            elif "=" in token:
                dictionary_name, marker_id_text = token.rsplit("=", 1)
            else:
                continue
            dictionary_name = dictionary_name.strip()
            try:
                marker_id = int(marker_id_text.strip())
            except ValueError:
                self.get_logger().warn(f"Ignoring invalid aruco_fallback_markers entry: {token!r}")
                continue
            candidate = (dictionary_name, marker_id)
            if candidate not in candidates:
                candidates.append(candidate)
        return candidates

    def _finger_axis_hint(
        self,
        marker: ArucoMarker | RedCircle,
        intrinsics: CameraIntrinsics,
    ) -> np.ndarray | None:
        if not isinstance(marker, ArucoMarker):
            return None
        if not bool(self.get_parameter("use_aruco_axis_for_orientation").value):
            return None
        corners = np.asarray(marker.corners, dtype=float)
        if corners.shape != (4, 2):
            return None
        quarter_turns = int(self.get_parameter("aruco_finger_axis_quarter_turns").value) % 4
        start = corners[quarter_turns]
        end = corners[(quarter_turns + 1) % 4]
        delta = end - start
        if float(np.linalg.norm(delta)) <= 1e-9:
            return None
        return np.array(
            [
                float(delta[0]) / float(intrinsics.fx),
                float(delta[1]) / float(intrinsics.fy),
                0.0,
            ],
            dtype=float,
        )

    def _detected_status(
        self,
        lid: LidDetection2D | None,
        marker: ArucoMarker | RedCircle,
        depth: DepthSample,
        normal: np.ndarray,
        plane_points: int,
        plane_rmse: float,
        finger_axis_hint: np.ndarray | None = None,
    ) -> str:
        parts = ["detected:lid"]
        if lid is not None:
            parts.extend(
                [
                    f"class={lid.class_name}",
                    f"bbox={lid.roi.width}x{lid.roi.height}",
                    f"lid_center=({lid.center_u},{lid.center_v})",
                ]
            )
        else:
            parts.extend(["class=aruco_marker", "bbox=none"])
        if isinstance(marker, ArucoMarker):
            parts.extend(
                [
                    "marker_type=aruco",
                    f"aruco_id={marker.marker_id}",
                    f"aruco_center=({marker.center_u},{marker.center_v})",
                    f"aruco_side_px={marker.side_px:.1f}",
                    f"aruco_marker_length_m={float(self.get_parameter('aruco_marker_length_m').value):.3f}",
                ]
            )
        else:
            parts.extend(
                [
                    "marker_type=red",
                    f"red_center=({marker.center_u},{marker.center_v})",
                    f"red_radius_px={marker.radius_px:.1f}",
                    f"red_circularity={marker.circularity:.3f}",
                ]
            )
        parts.extend(
            [
                f"depth_raw={depth.raw:.1f}",
                f"depth_m={depth.meters:.3f}",
                f"depth_encoding={depth.encoding}",
                f"depth_scale={depth.scale:.6g}",
                f"normal=({normal[0]:.4f},{normal[1]:.4f},{normal[2]:.4f})",
                f"plane_points={plane_points}",
                f"plane_rmse_m={plane_rmse:.4f}",
            ]
        )
        if isinstance(marker, ArucoMarker) and finger_axis_hint is not None:
            parts.extend(
                [
                    f"aruco_axis_quarter_turns={int(self.get_parameter('aruco_finger_axis_quarter_turns').value)}",
                    (
                        "aruco_axis_hint="
                        f"({finger_axis_hint[0]:.4f},{finger_axis_hint[1]:.4f},{finger_axis_hint[2]:.4f})"
                    ),
                ]
            )
        return " ".join(parts)

    def _detect_best_lid(self, image: np.ndarray) -> LidDetection2D | None:
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
            x1 = max(min(x1, image.shape[1]), 0)
            x2 = max(min(x2, image.shape[1]), 0)
            y1 = max(min(y1, image.shape[0]), 0)
            y2 = max(min(y2, image.shape[0]), 0)
            width = max(x2 - x1, 0)
            height = max(y2 - y1, 0)
            if width <= 0 or height <= 0:
                continue
            candidate = LidDetection2D(
                roi=ImageRoi(x1, y1, x2, y2),
                center_u=int(round((x1 + x2) / 2.0)),
                center_v=int(round((y1 + y2) / 2.0)),
                area=width * height,
                confidence=confidence,
                class_name=class_name,
            )
            if self._is_better_detection(candidate, best, selection_policy):
                best = candidate
        return best

    def _median_depth(self, u: int, v: int) -> DepthSample | None:
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

    def _estimate_plane(
        self,
        center_u: int,
        center_v: int,
        intrinsics: CameraIntrinsics,
        depth_scale: float,
    ) -> PlaneEstimate | None:
        depth = self._latest_depth
        if depth is None or depth.size == 0:
            return None
        radius = max(int(self.get_parameter("plane_patch_radius_px").value), 1)
        stride = max(int(self.get_parameter("plane_sample_stride_px").value), 1)
        min_depth_m = float(self.get_parameter("min_depth_m").value)
        max_depth_m = float(self.get_parameter("max_depth_m").value)
        height, width = depth.shape[:2]
        points = []
        for v in range(max(center_v - radius, 0), min(center_v + radius + 1, height), stride):
            for u in range(max(center_u - radius, 0), min(center_u + radius + 1, width), stride):
                if (u - center_u) * (u - center_u) + (v - center_v) * (v - center_v) > radius * radius:
                    continue
                raw = float(depth[v, u])
                if not math.isfinite(raw) or raw <= 0.0:
                    continue
                depth_m = raw * depth_scale
                if depth_m < min_depth_m or depth_m > max_depth_m:
                    continue
                try:
                    points.append(
                        pixel_depth_to_camera_point(u, v, raw, intrinsics, depth_scale=depth_scale)
                    )
                except ValueError:
                    continue
        min_points = int(self.get_parameter("min_plane_points").value)
        if len(points) < min_points:
            self.get_logger().warn(
                f"Not enough valid lid plane points: {len(points)} < {min_points}"
            )
            return None
        point_array = np.asarray(points, dtype=float)
        centroid = np.mean(point_array, axis=0)
        centered = point_array - centroid
        try:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError as exc:
            self.get_logger().warn(f"lid plane SVD failed: {exc}")
            return None
        normal = vh[-1]
        norm = float(np.linalg.norm(normal))
        if norm <= 1e-12:
            return None
        normal = normal / norm
        if normal[2] > 0.0:
            normal = -normal
        residuals = centered @ normal
        rmse = float(math.sqrt(float(np.mean(residuals * residuals))))
        max_rmse = float(self.get_parameter("max_plane_rmse_m").value)
        if max_rmse > 0.0 and rmse > max_rmse:
            self.get_logger().warn(
                f"Lid plane RMSE too high: {rmse:.4f}m > {max_rmse:.4f}m"
            )
            return None
        return PlaneEstimate(normal=normal, point_count=len(points), rmse_m=rmse)

    def _red_circle_config(self) -> RedCircleConfig:
        return RedCircleConfig(
            min_area_px=float(self.get_parameter("red_min_area_px").value),
            min_radius_px=float(self.get_parameter("red_min_radius_px").value),
            min_circularity=float(self.get_parameter("red_min_circularity").value),
            min_saturation=int(self.get_parameter("red_min_saturation").value),
            min_value=int(self.get_parameter("red_min_value").value),
            morph_kernel_px=int(self.get_parameter("red_morph_kernel_px").value),
        )

    def _target_class_names(self) -> list[str]:
        raw = self.get_parameter("target_class_names").value
        if isinstance(raw, str):
            values = raw.replace(";", ",").split(",")
        else:
            values = list(raw)
        return [str(value).strip().lower() for value in values if str(value).strip()]

    def _marker_type(self) -> str:
        return str(self.get_parameter("marker_type").value).strip().lower()

    @staticmethod
    def _is_better_detection(
        candidate: LidDetection2D,
        best: LidDetection2D | None,
        selection_policy: str,
    ) -> bool:
        if best is None:
            return True
        if selection_policy == "largest_bbox":
            if candidate.area != best.area:
                return candidate.area > best.area
            return candidate.confidence > best.confidence
        if abs(candidate.confidence - best.confidence) > 1e-9:
            return candidate.confidence > best.confidence
        return candidate.area > best.area

    def _depth_window_size(self) -> int:
        configured = max(int(self.get_parameter("depth_window_size").value), 1)
        if configured % 2 == 0:
            configured += 1
        return configured

    def _source_frame(self, info: CameraInfo, msg: Image) -> str:
        configured = str(self.get_parameter("source_frame").value).strip()
        return configured or info.header.frame_id or msg.header.frame_id

    def _depth_scale(self) -> float | None:
        mode = str(self.get_parameter("depth_scale_mode").value).strip().lower()
        encoding = self._latest_depth_encoding
        if mode == "manual":
            depth_scale = float(self.get_parameter("depth_scale").value)
            if depth_scale <= 0.0:
                self.get_logger().error(
                    f"Rejecting lid projection: manual depth_scale must be positive, got {depth_scale}"
                )
                return None
            return depth_scale
        if mode != "auto":
            self.get_logger().error(
                f"Rejecting lid projection: unsupported depth_scale_mode={mode!r}; use 'auto' or 'manual'"
            )
            return None
        scale = YoloTumblerDetectorNode._auto_depth_scales().get(encoding)
        if scale is None:
            self.get_logger().error(
                f"Rejecting lid projection: unsupported depth encoding {encoding!r} in auto mode"
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
            "Lid depth scale selected: "
            f"encoding={self._latest_depth_encoding} mode={mode} scale={depth_scale:.6g}"
        )

    def _publish_invalid(self, msg: Image, status: str) -> None:
        self._latest_valid_status = ""
        self._latest_valid_stamp = None
        output = CupDetection()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = msg.header.frame_id
        output.grasp_pose = Pose()
        output.cup_mouth_center = Pose()
        output.confidence = 0.0
        output.status = status
        output.source = str(self.get_parameter("source").value)
        self._pub.publish(output)

    def _show_preview(
        self,
        image: np.ndarray,
        status: str,
        lid: LidDetection2D | None = None,
        marker: ArucoMarker | RedCircle | None = None,
        depth_m: float | None = None,
        valid: bool = False,
    ) -> None:
        if not bool(self.get_parameter("show_preview").value):
            return

        frame = image.copy()
        color = (0, 220, 0) if valid else (0, 180, 255)
        if lid is not None:
            cv2.rectangle(
                frame,
                (lid.roi.x_min, lid.roi.y_min),
                (lid.roi.x_max, lid.roi.y_max),
                color,
                2,
            )
            cv2.putText(
                frame,
                f"{lid.class_name} {lid.confidence:.2f}",
                (lid.roi.x_min, max(24, lid.roi.y_min - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                color,
                2,
                cv2.LINE_AA,
            )
        if isinstance(marker, ArucoMarker):
            corners = marker.corners.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(frame, [corners], True, (255, 0, 255), 2, cv2.LINE_AA)
            cv2.circle(frame, (marker.center_u, marker.center_v), 5, (255, 0, 255), -1)
            cv2.putText(
                frame,
                f"aruco {marker.marker_id}",
                (marker.center_u + 8, max(24, marker.center_v - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
        elif isinstance(marker, RedCircle):
            cv2.circle(frame, (marker.center_u, marker.center_v), 5, (0, 0, 255), -1)
            cv2.circle(
                frame,
                (marker.center_u, marker.center_v),
                max(int(round(marker.radius_px)), 6),
                (0, 0, 255),
                2,
            )

        short_status = status if len(status) <= 150 else status[:147] + "..."
        lines = [
            "p: grip request  ESC/q: quit",
            f"status: {short_status}",
        ]
        if depth_m is not None:
            lines.append(f"depth: {depth_m:.3f} m")
        for index, text in enumerate(lines):
            y = 26 + index * 24
            cv2.putText(
                frame,
                text,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (230, 230, 230),
                2,
                cv2.LINE_AA,
            )

        window = str(self.get_parameter("preview_window_name").value)
        if not self._preview_window_created:
            cv2.namedWindow(window)
            self._preview_window_created = True
        cv2.imshow(window, frame)
        key = cv2.waitKey(max(int(self.get_parameter("preview_wait_ms").value), 1)) & 0xFF
        if key in (ord("p"), ord("P")):
            self._publish_grip_request()
        elif key in (27, ord("q"), ord("Q")):
            cv2.destroyWindow(window)
            if rclpy.ok():
                rclpy.shutdown()

    def _publish_grip_request(self) -> None:
        accepted = bool(self._latest_valid_status.startswith("detected:lid"))
        if bool(self.get_parameter("require_lid_detection").value):
            accepted = accepted and not self._status_is_aruco_only(self._latest_valid_status)
        stamp = self._latest_valid_stamp
        payload = {
            "command": "grip_lid",
            "accepted": accepted,
            "source": "lid_sticker_detector_node",
            "status": self._latest_valid_status if accepted else "no_valid_lid_detection",
        }
        if stamp is not None:
            payload["stamp"] = f"{stamp.sec}.{stamp.nanosec:09d}"
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._grip_request_pub.publish(msg)
        if accepted:
            self._last_accepted_grip_request_time = time.monotonic()
            self.get_logger().warn(
                "Published supervised lid grip request from p key; downstream motion remains gated"
            )
        else:
            self.get_logger().warn("Ignored p key because there is no valid detected:lid frame")

    def _aruco_only_after_grip_request_active(self) -> bool:
        if self._marker_type() != "aruco":
            return False
        if not bool(self.get_parameter("allow_aruco_only_after_grip_request").value):
            return False
        if self._last_accepted_grip_request_time is None:
            return False
        window_sec = max(
            float(self.get_parameter("aruco_only_after_grip_request_sec").value),
            0.0,
        )
        return time.monotonic() - self._last_accepted_grip_request_time <= window_sec

    @staticmethod
    def _status_is_aruco_only(status: str) -> bool:
        return "class=aruco_marker" in status and "bbox=none" in status

    @staticmethod
    def _pose_at(
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = LidStickerDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        if node._preview_window_created:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
