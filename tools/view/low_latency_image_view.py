#!/usr/bin/env python3
"""Low-latency ROS Image viewer.

rqt_image_view is convenient, but in field testing it can appear to lag when
old image messages queue up. This viewer subscribes with BEST_EFFORT/depth=1
and only displays the newest frame.
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


LOW_LATENCY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    if msg.encoding == "bgr8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
    if msg.encoding == "rgb8":
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if msg.encoding == "mono8":
        mono = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
    if msg.encoding == "16UC1":
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        return cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported image encoding: {msg.encoding}")


class LowLatencyImageView(Node):
    def __init__(self, topic: str, window_name: str) -> None:
        super().__init__("azas_low_latency_image_view")
        self.topic = topic
        self.window_name = window_name
        self.latest: Image | None = None
        self.last_fps_report = time.monotonic()
        self.frames = 0
        self.create_subscription(Image, topic, self.on_image, LOW_LATENCY_QOS)
        self.get_logger().info(f"viewing {topic} with BEST_EFFORT depth=1")

    def on_image(self, msg: Image) -> None:
        self.latest = msg

    def show_once(self) -> bool:
        if self.latest is None:
            return True
        msg = self.latest
        self.latest = None
        try:
            frame = image_msg_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return False
        self.frames += 1
        now = time.monotonic()
        if now - self.last_fps_report >= 1.0:
            fps = self.frames / (now - self.last_fps_report)
            self.frames = 0
            self.last_fps_report = now
            cv2.setWindowTitle(self.window_name, f"{self.topic}  {fps:.1f} fps")
        cv2.imshow(self.window_name, frame)
        return cv2.waitKey(1) not in (27, ord("q"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic", nargs="?", default="/azas/human_hand_detection/overlay")
    parser.add_argument("--window-name", default="Azas Low Latency Image View")
    args = parser.parse_args()

    rclpy.init()
    node = LowLatencyImageView(args.topic, args.window_name)
    cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            if not node.show_once():
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
