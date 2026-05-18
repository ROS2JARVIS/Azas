# Role: Fuse YOLO 2D detections with aligned depth and camera intrinsics.

from __future__ import annotations

import json
import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class Detection3DNode(Node):
    """Compute object center points in camera coordinates and robot base coordinates."""

    def __init__(self) -> None:
        super().__init__("detection_3d_node")

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("detections_topic", "/cocktail/vision/detections")
        self.declare_parameter(
            "detections_3d_topic", "/cocktail/detection_3d/detections"
        )
        self.declare_parameter("pose_topic", "/cocktail/detection_3d/target_pose")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 2.00)
        self.declare_parameter("roi_shrink_ratio", 0.50)
        self.declare_parameter("minimum_valid_depth_pixels", 20)
        self.declare_parameter("use_hand_eye_matrix", False)
        self.declare_parameter(
            "hand_eye_transform_matrix",
            [
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
        )
        self.declare_parameter("hand_eye_translation_xyz", [0.35, 0.00, 0.45])
        self.declare_parameter("hand_eye_rotation_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("object_pose_orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("log_3d_detections", True)

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.detections_3d_topic = str(
            self.get_parameter("detections_3d_topic").value
        )
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.roi_shrink_ratio = float(self.get_parameter("roi_shrink_ratio").value)
        self.minimum_valid_depth_pixels = int(
            self.get_parameter("minimum_valid_depth_pixels").value
        )
        self.object_pose_orientation_xyzw = [
            float(v)
            for v in self.get_parameter("object_pose_orientation_xyzw").value
        ]
        self.log_3d_detections = bool(self.get_parameter("log_3d_detections").value)

        hand_eye_translation = [
            float(v) for v in self.get_parameter("hand_eye_translation_xyz").value
        ]
        hand_eye_rpy_deg = [
            float(v) for v in self.get_parameter("hand_eye_rotation_rpy_deg").value
        ]
        self.use_hand_eye_matrix = bool(
            self.get_parameter("use_hand_eye_matrix").value
        )
        hand_eye_matrix = [
            float(v) for v in self.get_parameter("hand_eye_transform_matrix").value
        ]
        if self.use_hand_eye_matrix:
            self.camera_to_base = self._make_transform_from_matrix(hand_eye_matrix)
            self.get_logger().info(
                "Using hand_eye_transform_matrix as T_base_camera."
            )
        else:
            self.camera_to_base = self._make_transform(
                hand_eye_translation, hand_eye_rpy_deg
            )
            self.get_logger().info(
                "Using hand_eye_translation_xyz and hand_eye_rotation_rpy_deg "
                "as T_base_camera."
            )

        self.bridge = CvBridge()
        self.latest_color_msg: Optional[Image] = None
        self.latest_depth_image: Optional[np.ndarray] = None
        self.latest_depth_encoding: str = ""
        self.latest_camera_info: Optional[CameraInfo] = None

        self.detections_3d_pub = self.create_publisher(
            String, self.detections_3d_topic, 10
        )
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)

        self.color_sub = self.create_subscription(
            Image, self.color_topic, self._color_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._depth_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback, 10
        )
        self.detections_sub = self.create_subscription(
            String, self.detections_topic, self._detections_callback, 10
        )

        self.get_logger().info(
            "Detection3DNode ready. "
            f"depth_topic={self.depth_topic}, camera_info_topic={self.camera_info_topic}"
        )

    def _color_callback(self, msg: Image) -> None:
        self.latest_color_msg = msg

    def _depth_callback(self, msg: Image) -> None:
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            self.latest_depth_encoding = msg.encoding
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert depth image: {exc}")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def _detections_callback(self, msg: String) -> None:
        if self.latest_depth_image is None:
            self.get_logger().warn("No depth image received yet; cannot compute 3D.")
            return
        if self.latest_camera_info is None:
            self.get_logger().warn("No camera_info received yet; cannot compute 3D.")
            return

        try:
            detection_payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid detection JSON: {exc}")
            return

        detections = detection_payload.get("detections", [])
        if not detections:
            self._publish_empty_result(detection_payload)
            return

        detections_3d: List[Dict[str, Any]] = []
        for det in detections:
            try:
                result = self._compute_detection_3d(det)
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to compute 3D for {det.get('class_name', 'unknown')}: {exc}"
                )
                continue

            if result is not None:
                detections_3d.append(result)

        payload = {
            "stamp": detection_payload.get("stamp"),
            "source_frame_id": detection_payload.get("frame_id"),
            "target_frame_id": self.target_frame,
            "detections": detections_3d,
        }

        out_msg = String()
        out_msg.data = json.dumps(payload)
        self.detections_3d_pub.publish(out_msg)

        best = self._select_best_detection(detections_3d)
        if best is not None:
            pose_msg = self._make_pose_stamped(best)
            self.pose_pub.publish(pose_msg)

        if self.log_3d_detections and detections_3d:
            summary = ", ".join(
                f"{det['class_name']}:{det['confidence']:.2f} "
                f"base=({det['robot_xyz'][0]:.3f},"
                f"{det['robot_xyz'][1]:.3f},{det['robot_xyz'][2]:.3f})"
                for det in detections_3d
            )
            self.get_logger().info(f"3D detections: {summary}")

    def _publish_empty_result(self, detection_payload: Dict[str, Any]) -> None:
        payload = {
            "stamp": detection_payload.get("stamp"),
            "source_frame_id": detection_payload.get("frame_id"),
            "target_frame_id": self.target_frame,
            "detections": [],
        }
        out_msg = String()
        out_msg.data = json.dumps(payload)
        self.detections_3d_pub.publish(out_msg)

    def _compute_detection_3d(self, det: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        bbox = [float(v) for v in det["bbox_xyxy"]]
        center_px = [float(v) for v in det["center_px"]]

        depth_m = self._median_depth_from_bbox(bbox)
        if depth_m is None:
            self.get_logger().warn(
                f"No valid depth for {det.get('class_name', 'unknown')} bbox={bbox}"
            )
            return None

        camera_xyz = self._deproject_pixel_to_point(center_px[0], center_px[1], depth_m)
        robot_xyz = self._transform_camera_to_base(camera_xyz)

        return {
            "class_id": int(det.get("class_id", -1)),
            "class_name": str(det.get("class_name", "unknown")),
            "confidence": float(det.get("confidence", 0.0)),
            "bbox_xyxy": bbox,
            "center_px": center_px,
            "depth_m": float(depth_m),
            "camera_xyz": [float(v) for v in camera_xyz],
            "robot_xyz": [float(v) for v in robot_xyz],
        }

    def _median_depth_from_bbox(self, bbox: List[float]) -> Optional[float]:
        if self.latest_depth_image is None:
            return None

        height, width = self.latest_depth_image.shape[:2]
        x1, y1, x2, y2 = bbox

        cx = (x1 + x2) * 0.5
        cy = (y1 + y2) * 0.5
        half_w = max(1.0, (x2 - x1) * self.roi_shrink_ratio * 0.5)
        half_h = max(1.0, (y2 - y1) * self.roi_shrink_ratio * 0.5)

        rx1 = int(max(0, math.floor(cx - half_w)))
        ry1 = int(max(0, math.floor(cy - half_h)))
        rx2 = int(min(width, math.ceil(cx + half_w)))
        ry2 = int(min(height, math.ceil(cy + half_h)))

        if rx2 <= rx1 or ry2 <= ry1:
            return None

        roi = self.latest_depth_image[ry1:ry2, rx1:rx2]
        depth_m = self._depth_to_meters(roi)
        valid = depth_m[
            np.isfinite(depth_m)
            & (depth_m >= self.min_depth_m)
            & (depth_m <= self.max_depth_m)
        ]

        if valid.size < self.minimum_valid_depth_pixels:
            return None

        return float(np.median(valid))

    def _depth_to_meters(self, depth: np.ndarray) -> np.ndarray:
        if np.issubdtype(depth.dtype, np.integer):
            return depth.astype(np.float32) * self.depth_scale
        return depth.astype(np.float32)

    def _deproject_pixel_to_point(
        self, u: float, v: float, depth_m: float
    ) -> Tuple[float, float, float]:
        if self.latest_camera_info is None:
            raise RuntimeError("camera_info is not available")

        k = self.latest_camera_info.k
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])

        if abs(fx) < 1e-6 or abs(fy) < 1e-6:
            raise RuntimeError("Invalid camera intrinsics fx/fy")

        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m
        return (float(x), float(y), float(z))

    def _transform_camera_to_base(
        self, camera_xyz: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        point = np.array([camera_xyz[0], camera_xyz[1], camera_xyz[2], 1.0])
        transformed = self.camera_to_base @ point
        return (float(transformed[0]), float(transformed[1]), float(transformed[2]))

    def _select_best_detection(
        self, detections_3d: List[Dict[str, Any]]
    ) -> Optional[Dict[str, Any]]:
        if not detections_3d:
            return None

        cups = [det for det in detections_3d if det["class_name"] == "cup"]
        candidates = cups if cups else detections_3d
        return max(candidates, key=lambda item: float(item.get("confidence", 0.0)))

    def _make_pose_stamped(self, det: Dict[str, Any]) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.target_frame

        pose_msg.pose.position.x = float(det["robot_xyz"][0])
        pose_msg.pose.position.y = float(det["robot_xyz"][1])
        pose_msg.pose.position.z = float(det["robot_xyz"][2])

        qx, qy, qz, qw = self.object_pose_orientation_xyzw
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        return pose_msg

    def _make_transform(
        self, translation_xyz: List[float], rpy_deg: List[float]
    ) -> np.ndarray:
        roll, pitch, yaw = [math.radians(v) for v in rpy_deg]

        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        rot_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        rot_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rot_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        rotation = rot_z @ rot_y @ rot_x

        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = np.array(translation_xyz, dtype=float)
        return transform

    def _make_transform_from_matrix(self, values: List[float]) -> np.ndarray:
        if len(values) != 16:
            raise ValueError(
                "hand_eye_transform_matrix must have 16 values in row-major order."
            )
        transform = np.array(values, dtype=float).reshape((4, 4))
        if abs(transform[3, 3]) < 1e-9:
            raise ValueError("Invalid hand-eye matrix: bottom-right value is zero.")
        return transform


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = Detection3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
