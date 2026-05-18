# Role: Run Ultralytics YOLO on RealSense color images and publish 2D detections.

from __future__ import annotations

import json
import os
from typing import Any, Dict, List, Optional

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cocktail_robot_system.image_utils import (
    cv_image_to_image_msg,
    image_msg_to_cv_image,
)


class VisionNode(Node):
    """ROS2 node that detects cups and lids in the RealSense color stream."""

    def __init__(self) -> None:
        super().__init__("vision_node")

        self.declare_parameter("model_path", "models/best.pt")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/cocktail/vision/detections")
        self.declare_parameter("debug_image_topic", "/cocktail/vision/debug_image")
        self.declare_parameter("class_names", ["cup", "lid"])
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("debug", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("yolo_device", "")
        self.declare_parameter("log_detections", True)

        self.model_path = self._resolve_model_path(
            str(self.get_parameter("model_path").value)
        )
        self.color_topic = str(self.get_parameter("color_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.class_names = set(self.get_parameter("class_names").value)
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.debug = bool(self.get_parameter("debug").value)
        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )
        self.yolo_device = str(self.get_parameter("yolo_device").value)
        self.log_detections = bool(self.get_parameter("log_detections").value)

        self.model: Optional[Any] = None
        self._model_error_logged = False

        self.detections_pub = self.create_publisher(
            String, self.detections_topic, 10
        )
        self.debug_image_pub = self.create_publisher(
            Image, self.debug_image_topic, 10
        )

        self.image_sub = self.create_subscription(
            Image, self.color_topic, self._image_callback, 10
        )

        self._load_model()
        self.get_logger().info(
            f"VisionNode ready. color_topic={self.color_topic}, "
            f"detections_topic={self.detections_topic}, model={self.model_path}"
        )

    def _resolve_model_path(self, model_path: str) -> str:
        if model_path.startswith("package://"):
            relative_path = model_path.replace(
                "package://cocktail_robot_system/", "", 1
            )
            package_share = get_package_share_directory("cocktail_robot_system")
            return os.path.join(package_share, relative_path)

        if os.path.isabs(model_path):
            return model_path

        package_share = get_package_share_directory("cocktail_robot_system")
        return os.path.join(package_share, model_path)

    def _load_model(self) -> None:
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"YOLO model file not found: {self.model_path}")
            return

        try:
            from ultralytics import YOLO

            self.model = YOLO(self.model_path)
            self.get_logger().info("YOLO model loaded successfully.")
        except Exception as exc:
            self.model = None
            self.get_logger().error(
                "Failed to load Ultralytics YOLO. Install it with "
                "`pip install ultralytics` in the ROS2 environment. "
                f"error={exc}"
            )

    def _image_callback(self, msg: Image) -> None:
        if self.model is None:
            if not self._model_error_logged:
                self.get_logger().warn("Skipping inference because YOLO is not ready.")
                self._model_error_logged = True
            return

        try:
            cv_image = image_msg_to_cv_image(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert color image: {exc}")
            return

        try:
            yolo_kwargs: Dict[str, Any] = {
                "conf": self.confidence_threshold,
                "verbose": False,
            }
            if self.yolo_device:
                yolo_kwargs["device"] = self.yolo_device

            results = self.model(cv_image, **yolo_kwargs)
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            return

        detections = self._parse_yolo_results(results, msg, cv_image)
        payload = {
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "frame_id": msg.header.frame_id,
            "image_width": int(msg.width),
            "image_height": int(msg.height),
            "detections": detections,
        }

        out_msg = String()
        out_msg.data = json.dumps(payload)
        self.detections_pub.publish(out_msg)

        if self.log_detections and detections:
            summary = ", ".join(
                f"{det['class_name']}:{det['confidence']:.2f}@"
                f"({det['center_px'][0]:.0f},{det['center_px'][1]:.0f})"
                for det in detections
            )
            self.get_logger().info(f"YOLO detections: {summary}")

        if self.debug and self.publish_debug_image:
            self._publish_debug_image(msg, cv_image, detections)

    def _parse_yolo_results(
        self, results: Any, msg: Image, cv_image: Any
    ) -> List[Dict[str, Any]]:
        detections: List[Dict[str, Any]] = []
        if not results:
            return detections

        result = results[0]
        names = getattr(result, "names", None) or getattr(self.model, "names", {})
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return detections

        for box in boxes:
            try:
                cls_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())
                class_name = self._class_name_from_id(names, cls_id)

                if self.class_names and class_name not in self.class_names:
                    continue

                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                x1 = max(0.0, min(x1, float(msg.width - 1)))
                y1 = max(0.0, min(y1, float(msg.height - 1)))
                x2 = max(0.0, min(x2, float(msg.width - 1)))
                y2 = max(0.0, min(y2, float(msg.height - 1)))

                if x2 <= x1 or y2 <= y1:
                    continue

                detections.append(
                    {
                        "class_id": cls_id,
                        "class_name": class_name,
                        "confidence": confidence,
                        "bbox_xyxy": [x1, y1, x2, y2],
                        "center_px": [(x1 + x2) * 0.5, (y1 + y2) * 0.5],
                    }
                )
            except Exception as exc:
                self.get_logger().warn(f"Failed to parse one YOLO box: {exc}")

        return detections

    def _class_name_from_id(self, names: Any, cls_id: int) -> str:
        if isinstance(names, dict):
            return str(names.get(cls_id, cls_id))
        if isinstance(names, list) and 0 <= cls_id < len(names):
            return str(names[cls_id])
        return str(cls_id)

    def _publish_debug_image(
        self, source_msg: Image, cv_image: Any, detections: List[Dict[str, Any]]
    ) -> None:
        debug_image = cv_image.copy()

        for det in detections:
            x1, y1, x2, y2 = [int(round(v)) for v in det["bbox_xyxy"]]
            label = f"{det['class_name']} {det['confidence']:.2f}"
            color = (0, 220, 0) if det["class_name"] == "cup" else (0, 165, 255)
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), color, 2)
            cv2.circle(
                debug_image,
                (int(det["center_px"][0]), int(det["center_px"][1])),
                4,
                color,
                -1,
            )
            cv2.putText(
                debug_image,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

        try:
            debug_msg = cv_image_to_image_msg(
                debug_image, encoding="bgr8", header=source_msg.header
            )
            self.debug_image_pub.publish(debug_msg)
        except Exception as exc:
            self.get_logger().error(f"Failed to publish debug image: {exc}")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
