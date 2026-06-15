#!/usr/bin/env python3
"""Perception-only human hand detection for the post-shake handover plan.

Publishes a stable open-hand 3D target on /azas/human_hand_detection as a
geometry_msgs/PointStamped in the color camera optical frame. This node sends
NO robot motion command of any kind; it only reads camera topics, following
docs/post_shake_human_handover_plan.md phase VERIFY_HUMAN_HAND_TRACKING
(gate: no_motion_hri_perception_only).

Pipeline:
  RealSense color + aligned depth -> MediaPipe HandLandmarker (tasks API)
  -> open-palm heuristic over 21 landmarks -> palm-center pixel
  -> median depth window -> intrinsics deprojection -> stability window
  -> publish only while the hand stays open and spatially stable.

Usage:
  python3 tools/perception/human_hand_detection_node.py
  python3 tools/perception/human_hand_detection_node.py --show-overlay false
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

DEFAULT_MODEL_PATH = "/home/ssu/Azas/models/mediapipe/hand_landmarker.task"
COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
OUTPUT_TOPIC = "/azas/human_hand_detection"
STATUS_TOPIC = "/azas/human_hand_detection/status"
OVERLAY_TOPIC = "/azas/human_hand_detection/overlay"

WRIST = 0
PALM_LANDMARKS = (0, 5, 9, 13, 17)
FINGER_TIPS = (8, 12, 16, 20)
FINGER_PIPS = (6, 10, 14, 18)


# cv_bridge is avoided on purpose: the ROS humble build is ABI-incompatible
# with the pip-installed numpy 2.x that mediapipe requires.
def image_msg_to_array(msg: Image) -> np.ndarray:
    if msg.encoding in ("bgr8", "rgb8"):
        array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(array, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else array.copy()
    if msg.encoding == "16UC1":
        dtype = np.dtype(np.uint16).newbyteorder(">" if msg.is_bigendian else "<")
        return np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
    if msg.encoding == "32FC1":
        dtype = np.dtype(np.float32).newbyteorder(">" if msg.is_bigendian else "<")
        return np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
    raise ValueError(f"unsupported image encoding: {msg.encoding}")


def bgr_array_to_image_msg(array: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = array.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(array).tobytes()
    return msg


def resize_to_width(array: np.ndarray, width_px: int) -> np.ndarray:
    if width_px <= 0 or array.shape[1] == width_px:
        return array
    scale = float(width_px) / float(array.shape[1])
    height_px = max(int(round(array.shape[0] * scale)), 1)
    return cv2.resize(array, (width_px, height_px), interpolation=cv2.INTER_AREA)


class HumanHandDetectionNode(Node):
    """Perception-only node: no motion service client is created here."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("azas_human_hand_detection")
        self.args = args
        self.camera_info: CameraInfo | None = None
        self.latest_depth: np.ndarray | None = None
        self.latest_depth_encoding = ""
        self.last_process_monotonic = 0.0
        self.last_timestamp_ms = 0
        # Recent accepted (monotonic_time, xyz_m) detections for the stability gate.
        self.recent: collections.deque[tuple[float, tuple[float, float, float]]] = collections.deque(maxlen=64)

        options = mp_vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=args.model_path),
            running_mode=mp_vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=args.min_detection_confidence,
            min_tracking_confidence=args.min_tracking_confidence,
        )
        self.landmarker = mp_vision.HandLandmarker.create_from_options(options)

        self.point_pub = self.create_publisher(PointStamped, OUTPUT_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.overlay_pub = self.create_publisher(Image, OVERLAY_TOPIC, 2) if args.show_overlay else None

        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.on_camera_info, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, 5)
        self.create_subscription(Image, COLOR_TOPIC, self.on_color, 5)

        self.get_logger().info(
            "human hand detection ready (perception-only, no motion commands). "
            f"publishing stable open-hand target on {OUTPUT_TOPIC}; "
            f"stability: {args.stable_min_samples} samples within {args.stable_radius_m:.3f}m "
            f"over >= {args.stable_min_seconds:.2f}s"
        )

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def on_depth(self, msg: Image) -> None:
        self.latest_depth = image_msg_to_array(msg)
        self.latest_depth_encoding = msg.encoding

    def on_color(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_process_monotonic < 1.0 / max(self.args.max_rate_hz, 0.5):
            return
        self.last_process_monotonic = now
        if self.camera_info is None or self.latest_depth is None:
            self.publish_status({"detected": False, "reason": "waiting for camera_info/depth"})
            return

        color = image_msg_to_array(msg)
        process_color = resize_to_width(color, int(self.args.process_width_px))
        rgb = cv2.cvtColor(process_color, cv2.COLOR_BGR2RGB)
        timestamp_ms = max(int(now * 1000.0), self.last_timestamp_ms + 1)
        self.last_timestamp_ms = timestamp_ms
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self.landmarker.detect_for_video(mp_image, timestamp_ms)

        overlay = color.copy() if self.overlay_pub is not None else None
        status: dict[str, object] = {"detected": False}
        try:
            if not result.hand_landmarks:
                self.recent.clear()
                status["reason"] = "no hand"
                return
            landmarks = result.hand_landmarks[0]
            height, width = color.shape[:2]
            process_height, process_width = process_color.shape[:2]
            scale_x = float(width) / float(process_width)
            scale_y = float(height) / float(process_height)
            pixels = [(lm.x * process_width * scale_x, lm.y * process_height * scale_y) for lm in landmarks]
            open_fingers = self.count_extended_fingers(pixels)
            hand_open = open_fingers >= self.args.min_extended_fingers
            palm_px = (
                int(np.clip(np.mean([pixels[i][0] for i in PALM_LANDMARKS]), 0, width - 1)),
                int(np.clip(np.mean([pixels[i][1] for i in PALM_LANDMARKS]), 0, height - 1)),
            )
            depth_m = self.median_depth_m(palm_px)
            status.update(
                {
                    "detected": True,
                    "open_fingers": open_fingers,
                    "hand_open": hand_open,
                    "palm_px": list(palm_px),
                    "depth_m": None if depth_m is None else round(depth_m, 4),
                }
            )
            if overlay is not None:
                for px, py in pixels:
                    cv2.circle(overlay, (int(px), int(py)), 3, (0, 255, 0) if hand_open else (0, 165, 255), -1)
                cv2.circle(overlay, palm_px, 8, (255, 0, 0), 2)

            if not hand_open:
                self.recent.clear()
                status["reason"] = f"hand not open ({open_fingers} extended fingers)"
                return
            if depth_m is None:
                self.recent.clear()
                status["reason"] = "no valid depth at palm"
                return

            xyz = self.deproject(palm_px, depth_m)
            status["camera_xyz_m"] = [round(v, 4) for v in xyz]
            self.recent.append((now, xyz))
            stable = self.is_stable(now, xyz)
            status["stable"] = stable
            status["stability_samples"] = len(self.recent)
            if overlay is not None:
                label = f"hand {'STABLE' if stable else 'tracking'} z={depth_m:.2f}m"
                cv2.putText(overlay, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0) if stable else (0, 165, 255), 2)
            if not stable:
                return

            point = PointStamped()
            point.header.stamp = msg.header.stamp
            point.header.frame_id = msg.header.frame_id or "camera_color_optical_frame"
            point.point.x, point.point.y, point.point.z = xyz
            self.point_pub.publish(point)
        finally:
            self.publish_status(status)
            if overlay is not None and self.overlay_pub is not None:
                overlay = resize_to_width(overlay, int(self.args.overlay_width_px))
                self.overlay_pub.publish(bgr_array_to_image_msg(overlay, msg.header))

    def count_extended_fingers(self, pixels: list[tuple[float, float]]) -> int:
        """A finger counts as extended when its tip is farther from the wrist than its PIP joint."""
        wrist = pixels[WRIST]
        count = 0
        for tip, pip in zip(FINGER_TIPS, FINGER_PIPS):
            tip_dist = math.dist(pixels[tip], wrist)
            pip_dist = math.dist(pixels[pip], wrist)
            if tip_dist > pip_dist * 1.05:
                count += 1
        return count

    def median_depth_m(self, palm_px: tuple[int, int]) -> float | None:
        depth = self.latest_depth
        if depth is None:
            return None
        half = max(int(self.args.depth_window_px) // 2, 1)
        y0 = max(palm_px[1] - half, 0)
        y1 = min(palm_px[1] + half + 1, depth.shape[0])
        x0 = max(palm_px[0] - half, 0)
        x1 = min(palm_px[0] + half + 1, depth.shape[1])
        window = depth[y0:y1, x0:x1].astype(np.float64)
        scale = 0.001 if self.latest_depth_encoding == "16UC1" else 1.0
        values = window.flatten() * scale
        values = values[(values >= self.args.min_depth_m) & (values <= self.args.max_depth_m)]
        if values.size < 3:
            return None
        return float(np.median(values))

    def deproject(self, pixel: tuple[int, int], depth_m: float) -> tuple[float, float, float]:
        k = self.camera_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        x = (pixel[0] - cx) / fx * depth_m
        y = (pixel[1] - cy) / fy * depth_m
        return (x, y, depth_m)

    def is_stable(self, now: float, xyz: tuple[float, float, float]) -> bool:
        window = [item for item in self.recent if now - item[0] <= self.args.stable_window_seconds]
        if len(window) < self.args.stable_min_samples:
            return False
        if now - window[0][0] < self.args.stable_min_seconds:
            return False
        return all(math.dist(item[1], xyz) <= self.args.stable_radius_m for item in window)

    def publish_status(self, status: dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def parse_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model-path", default=DEFAULT_MODEL_PATH)
    parser.add_argument("--process-width-px", type=int, default=0,
                        help="resize color frames to this width before MediaPipe; 0 keeps camera width")
    parser.add_argument("--overlay-width-px", type=int, default=0,
                        help="resize published overlay images to this width; 0 keeps camera width")
    parser.add_argument("--max-rate-hz", type=float, default=15.0)
    parser.add_argument("--min-detection-confidence", type=float, default=0.6)
    parser.add_argument("--min-tracking-confidence", type=float, default=0.6)
    parser.add_argument("--min-extended-fingers", type=int, default=4,
                        help="open-palm gate: required extended fingers out of 4 (thumb excluded)")
    parser.add_argument("--depth-window-px", type=int, default=7)
    parser.add_argument("--min-depth-m", type=float, default=0.3)
    parser.add_argument("--max-depth-m", type=float, default=1.5)
    parser.add_argument("--stable-radius-m", type=float, default=0.05,
                        help="all samples in the stability window must stay inside this radius")
    parser.add_argument("--stable-min-samples", type=int, default=8)
    parser.add_argument("--stable-min-seconds", type=float, default=0.8)
    parser.add_argument("--stable-window-seconds", type=float, default=1.5)
    parser.add_argument("--show-overlay", type=parse_bool, default=True)
    args = parser.parse_args()

    rclpy.init()
    node = HumanHandDetectionNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
