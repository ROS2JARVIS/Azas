#!/usr/bin/env python3
"""Show the ROS camera image in a large OpenCV window with cup state overlay."""

from __future__ import annotations

import argparse
import re
import time

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--cup-detection-topic", default="/azas/cup_detection")
    parser.add_argument("--window-name", default="Azas cup perception")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    return parser.parse_args()


def make_viewer_class(node_base, image_type, cup_detection_type):
    class CupDetectionViewer(node_base):
        def __init__(self, args: argparse.Namespace):
            super().__init__("azas_cup_detection_viewer")
            self.args = args
            self.latest_status = ""
            self.latest_status_time = 0.0
            self.create_subscription(image_type, args.image_topic, self._on_image, 10)
            self.create_subscription(cup_detection_type, args.cup_detection_topic, self._on_detection, 10)
            cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(args.window_name, args.width, args.height)
            self.get_logger().info(
                f"Viewing {args.image_topic}; overlaying {args.cup_detection_topic}. Press q or ESC to quit."
            )

        def _on_detection(self, msg) -> None:
            self.latest_status = msg.status
            self.latest_status_time = time.monotonic()

        def _on_image(self, msg) -> None:
            try:
                frame = image_msg_to_bgr(msg)
            except ValueError as exc:
                self.get_logger().warn(str(exc))
                return
            draw_detection_overlay(frame, self.latest_status, self.latest_status_time)
            cv2.imshow(self.args.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                raise KeyboardInterrupt

    return CupDetectionViewer


def image_msg_to_bgr(msg: Image) -> np.ndarray:
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
    if encoding in {"mono8", "8uc1"}:
        row_step = int(msg.step) if int(msg.step) > 0 else width
        expected = height * row_step
        if data.size < expected:
            raise ValueError(f"image buffer too small for {msg.encoding}: {data.size} < {expected}")
        gray = data[:expected].reshape((height, row_step))[:, :width]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported image encoding for viewer: {msg.encoding}")


def draw_detection_overlay(frame: np.ndarray, status: str, status_time: float) -> None:
    age = time.monotonic() - status_time if status_time > 0.0 else float("inf")
    stale = age > 1.0
    orientation = parse_orientation(status)
    color = (0, 255, 0) if orientation == "upright" else (0, 165, 255)
    if orientation not in {"upright", "lying"}:
        color = (0, 0, 255)
    if stale:
        color = (160, 160, 160)

    label = orientation.upper() if orientation else "NO DETECTION"
    if stale:
        label = f"{label} STALE"

    center, bbox = parse_center_and_bbox(status)
    if center is not None and bbox is not None:
        cx, cy = center
        bw, bh = bbox
        x1 = max(int(cx - bw / 2), 0)
        y1 = max(int(cy - bh / 2), 0)
        x2 = min(int(cx + bw / 2), frame.shape[1] - 1)
        y2 = min(int(cy + bh / 2), frame.shape[0] - 1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        cv2.circle(frame, (cx, cy), 5, color, -1)

    cv2.rectangle(frame, (12, 12), (520, 78), (0, 0, 0), -1)
    cv2.putText(frame, label, (28, 58), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3, cv2.LINE_AA)


def parse_orientation(status: str) -> str:
    normalized = status.lower()
    match = re.search(r"\borientation=([a-z_]+)", normalized)
    if match:
        return match.group(1)
    if normalized.startswith("detected:upright"):
        return "upright"
    if normalized.startswith("rejected:lying"):
        return "lying"
    return ""


def parse_center_and_bbox(status: str) -> tuple[tuple[int, int] | None, tuple[int, int] | None]:
    center_match = re.search(r"\bcenter=\((\d+),(\d+)\)", status)
    bbox_match = re.search(r"\bbbox=(\d+)x(\d+)", status)
    if not center_match or not bbox_match:
        return None, None
    center = (int(center_match.group(1)), int(center_match.group(2)))
    bbox = (int(bbox_match.group(1)), int(bbox_match.group(2)))
    return center, bbox


def main() -> int:
    args = parse_args()
    import rclpy
    from azas_interfaces.msg import CupDetection
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    rclpy.init()
    CupDetectionViewer = make_viewer_class(Node, Image, CupDetection)
    node = CupDetectionViewer(args)
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
