#!/usr/bin/env python3
"""Sample aligned depth + CameraInfo and project one pixel to camera-frame XYZ.

This is a live camera gate. It does not command robot motion.
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from azas_perception.depth_projection import CameraIntrinsics, pixel_depth_to_camera_point
from azas_perception.yolo_tumbler_detector_node import YoloTumblerDetectorNode


class DepthProjectionSampler(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("azas_depth_projection_sampler")
        self.args = args
        self.info: Optional[CameraInfo] = None
        self.depth: Optional[Image] = None
        self.create_subscription(CameraInfo, args.camera_info_topic, self.on_info, 10)
        self.create_subscription(Image, args.depth_topic, self.on_depth, 10)

    def on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def on_depth(self, msg: Image) -> None:
        self.depth = msg

    def ready(self) -> bool:
        return self.info is not None and self.depth is not None

    def sample(self) -> tuple[int, int, float, tuple[float, float, float]]:
        if self.info is None or self.depth is None:
            raise RuntimeError("missing CameraInfo or depth image")
        depth_array = YoloTumblerDetectorNode._image_to_array(self.depth)
        height, width = depth_array.shape[:2]
        u = self.args.u if self.args.u is not None else width // 2
        v = self.args.v if self.args.v is not None else height // 2
        radius = max(int(self.args.patch_radius), 0)
        x1, x2 = max(u - radius, 0), min(u + radius + 1, width)
        y1, y2 = max(v - radius, 0), min(v + radius + 1, height)
        patch = np.asarray(depth_array[y1:y2, x1:x2], dtype=np.float32)
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            raise RuntimeError(f"no valid depth around pixel ({u}, {v})")

        depth_raw = float(np.median(valid))
        intrinsics = CameraIntrinsics(
            fx=float(self.info.k[0]),
            fy=float(self.info.k[4]),
            cx=float(self.info.k[2]),
            cy=float(self.info.k[5]),
        )
        point = pixel_depth_to_camera_point(u, v, depth_raw, intrinsics)
        if not all(math.isfinite(value) for value in point):
            raise RuntimeError(f"projected point is not finite: {point}")
        return u, v, depth_raw, point


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--patch-radius", type=int, default=3)
    parser.add_argument("--u", type=int, default=None)
    parser.add_argument("--v", type=int, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = DepthProjectionSampler(args)
    deadline = time.monotonic() + float(args.timeout)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ready():
                break
        if not node.ready():
            print(
                "[FAIL] timed out waiting for depth and CameraInfo: "
                f"{args.depth_topic}, {args.camera_info_topic}"
            )
            return 1
        try:
            u, v, depth_raw, point = node.sample()
        except Exception as exc:
            print(f"[FAIL] depth projection failed: {exc}")
            return 1
        frame_id = node.info.header.frame_id if node.info is not None else ""
        print(
            "[PASS] depth projection sample "
            f"frame={frame_id!r} pixel=({u},{v}) depth_raw={depth_raw:.3f} "
            f"point_camera_m=({point[0]:.4f},{point[1]:.4f},{point[2]:.4f})"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

