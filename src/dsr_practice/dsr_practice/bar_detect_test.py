#!/usr/bin/env python3
"""
bar_detect_test.py  –  마우스 호버 depth 표시 (시각화 전용)

RealSense 영상 위에서 마우스 커서 위치의 depth(mm)를 실시간으로 표시.
ESC 로 종료.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class BarDetectTest(Node):
    def __init__(self):
        super().__init__("bar_detect_test")
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.mouse_xy = None   # (x, y) 커서 좌표

        self.create_subscription(
            Image, "/camera/camera/color/image_raw",
            self._color_cb, 10)
        self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw",
            self._depth_cb, 10)

    def _color_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # ── 마우스 콜백 ──
    def _mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_xy = (x, y)

    # ── depth 샘플링 (5×5 median, mm) ──
    def sample_depth_mm(self, px, py):
        if self.depth_image is None:
            return None
        h, w = self.depth_image.shape[:2]
        if not (0 <= px < w and 0 <= py < h):
            return None
        x0, x1 = max(0, px - 2), min(w, px + 3)
        y0, y1 = max(0, py - 2), min(h, py + 3)
        patch = self.depth_image[y0:y1, x0:x1]
        valid = patch[patch > 0]
        if valid.size == 0:
            return None
        z = float(np.median(valid))
        if self.depth_image.dtype != np.uint16:
            z *= 1000.0   # m → mm
        return z

    # ── 메인 ──
    def run(self):
        window = "DepthHover"
        cv2.namedWindow(window)
        cv2.setMouseCallback(window, self._mouse_cb)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.color_image is None:
                continue

            vis = self.color_image.copy()

            if self.mouse_xy is not None:
                mx, my = self.mouse_xy
                # 십자선
                cv2.drawMarker(vis, (mx, my), (0, 255, 255),
                               markerType=cv2.MARKER_CROSS,
                               markerSize=20, thickness=1)
                # depth
                d = self.sample_depth_mm(mx, my)
                if d is None:
                    text = f"({mx},{my}) d=?"
                else:
                    text = f"({mx},{my}) d={d:.0f}mm"
                # 좌상단 상태바
                cv2.putText(vis, text, (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255, 255, 255), 2)
                # 커서 옆
                cv2.putText(vis,
                            "?" if d is None else f"{d:.0f}",
                            (mx + 10, my - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 255), 2)

            cv2.imshow(window, vis)
            if (cv2.waitKey(1) & 0xFF) == 27:
                break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = BarDetectTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
