#!/usr/bin/env python3
"""Show YOLO best.pt detections on the ROS camera image.

This viewer is intentionally detector-only: no depth, no orientation classifier,
no robot pose, and no motion-facing `/azas/cup_detection` contract. Use it to
check whether the YOLO model finds `cup` and `lid`, especially near image edges.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--model", default="/home/ssu/Downloads/best.pt")
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--window-name", default="YOLO best.pt cup/lid check")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    return parser.parse_args()


def make_viewer_class(node_base, image_type):
    class YoloBestViewer(node_base):
        def __init__(self, args: argparse.Namespace):
            super().__init__("azas_yolo_best_viewer")
            self.args = args
            self.model = YOLO(args.model)
            self.create_subscription(image_type, args.image_topic, self._on_image, 10)
            cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(args.window_name, args.width, args.height)
            self.get_logger().info(
                f"Viewing {args.image_topic} with YOLO model {args.model}. Press q or ESC to quit."
            )

        def _on_image(self, msg) -> None:
            try:
                frame = image_msg_to_bgr(msg)
            except ValueError as exc:
                self.get_logger().warn(str(exc))
                return
            annotated = frame.copy()
            try:
                results = self.model.predict(
                    frame,
                    conf=float(self.args.conf),
                    device=str(self.args.device),
                    verbose=False,
                )
            except Exception as exc:
                self.get_logger().error(f"YOLO inference failed: {exc}")
                return
            draw_yolo_results(annotated, results)
            cv2.imshow(self.args.window_name, annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                raise KeyboardInterrupt

    return YoloBestViewer


def image_msg_to_bgr(msg) -> np.ndarray:
    height = int(msg.height)
    width = int(msg.width)
    encoding = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if encoding in {"bgr8", "rgb8"}:
        channels = 3
        row_step = int(msg.step) if int(msg.step) > 0 else width * channels
        expected = height * row_step
        if data.size < expected:
            raise ValueError(f"image buffer too small for {msg.encoding}: {data.size} < {expected}")
        image = data[:expected].reshape((height, row_step))[:, : width * channels]
        image = image.reshape((height, width, channels))
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image.copy()
    raise ValueError(f"unsupported image encoding: {msg.encoding}")


def draw_yolo_results(frame: np.ndarray, results) -> None:
    detections = []
    if results:
        result = results[0]
        names = getattr(result, "names", {}) or {}
        if result.boxes is not None:
            for box in result.boxes:
                x1, y1, x2, y2 = [int(round(v)) for v in box.xyxy[0].tolist()]
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = str(names.get(class_id, class_id)).lower()
                detections.append((class_name, confidence, x1, y1, x2, y2))

    for class_name, confidence, x1, y1, x2, y2 in detections:
        color = (0, 255, 0) if "cup" in class_name else (255, 0, 255)
        if "lid" in class_name:
            color = (0, 165, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        label = f"{class_name} {confidence:.2f}"
        text_y = max(y1 - 10, 28)
        cv2.putText(frame, label, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.85, color, 2, cv2.LINE_AA)

    banner = f"YOLO best.pt only | detections={len(detections)}"
    cv2.rectangle(frame, (12, 12), (560, 58), (0, 0, 0), -1)
    cv2.putText(frame, banner, (24, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)


def main() -> int:
    args = parse_args()
    model_path = Path(args.model)
    if not model_path.exists():
        raise SystemExit(f"YOLO model does not exist: {model_path}")

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    rclpy.init()
    YoloBestViewer = make_viewer_class(Node, Image)
    node = YoloBestViewer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
