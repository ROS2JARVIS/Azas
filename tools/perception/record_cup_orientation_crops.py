#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from azas_interfaces.msg import CupDetection
from rclpy.node import Node
from sensor_msgs.msg import Image


BBOX_RE = re.compile(r"\bbbox=(?P<w>\d+)x(?P<h>\d+)\b")
CENTER_RE = re.compile(r"\bcenter=\((?P<u>-?\d+),(?P<v>-?\d+)\)")


class CropRecorder(Node):
    def __init__(self, image_topic: str, detection_topic: str):
        super().__init__("azas_cup_orientation_crop_recorder")
        self.latest_image: Optional[np.ndarray] = None
        self.latest_image_stamp = None
        self.latest_detection: Optional[CupDetection] = None
        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.create_subscription(CupDetection, detection_topic, self._on_detection, 10)

    def _on_image(self, msg: Image) -> None:
        self.latest_image = image_to_bgr(msg)
        self.latest_image_stamp = msg.header.stamp

    def _on_detection(self, msg: CupDetection) -> None:
        self.latest_detection = msg


def image_to_bgr(msg: Image) -> np.ndarray:
    height = int(msg.height)
    width = int(msg.width)
    encoding = msg.encoding.lower()
    if encoding == "bgr8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3).copy()
    if encoding == "rgb8":
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if encoding in {"mono8", "8uc1"}:
        mono = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported image encoding: {msg.encoding}")


def parse_bbox_and_center(status: str) -> Optional[tuple[int, int, int, int]]:
    bbox = BBOX_RE.search(status)
    center = CENTER_RE.search(status)
    if bbox is None or center is None:
        return None
    width = int(bbox.group("w"))
    height = int(bbox.group("h"))
    center_u = int(center.group("u"))
    center_v = int(center.group("v"))
    return width, height, center_u, center_v


def crop_detection(image: np.ndarray, width: int, height: int, center_u: int, center_v: int, pad: float) -> np.ndarray:
    image_height, image_width = image.shape[:2]
    crop_width = max(int(round(width * (1.0 + pad))), 1)
    crop_height = max(int(round(height * (1.0 + pad))), 1)
    x1 = max(center_u - crop_width // 2, 0)
    y1 = max(center_v - crop_height // 2, 0)
    x2 = min(x1 + crop_width, image_width)
    y2 = min(y1 + crop_height, image_height)
    x1 = max(x2 - crop_width, 0)
    y1 = max(y2 - crop_height, 0)
    return image[y1:y2, x1:x2].copy()


def wait_for_sample(node: CropRecorder, timeout_sec: float) -> tuple[Optional[np.ndarray], Optional[CupDetection]]:
    node.latest_detection = None
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest_image is not None and node.latest_detection is not None:
            return node.latest_image, node.latest_detection
    return node.latest_image, node.latest_detection


def main() -> int:
    parser = argparse.ArgumentParser(description="Record YOLO cup bbox crops for upright/lying classifier training.")
    parser.add_argument("--output-dir", default="/tmp/azas_cup_orientation_dataset")
    parser.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--detection-topic", default="/azas/cup_detection")
    parser.add_argument("--labels", nargs="+", default=["upright", "lying"])
    parser.add_argument("--fixed-label", default="", help="Record every sample with this label.")
    parser.add_argument("--wait-sec", type=float, default=5.0)
    parser.add_argument("--pad", type=float, default=0.25)
    args = parser.parse_args()
    fixed_label = args.fixed_label.strip()
    if fixed_label and fixed_label not in args.labels:
        parser.error(f"--fixed-label must be one of: {', '.join(args.labels)}")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    for label in args.labels:
        (output_dir / label).mkdir(parents=True, exist_ok=True)
    metadata_path = output_dir / "metadata.jsonl"

    rclpy.init()
    node = CropRecorder(args.image_topic, args.detection_topic)
    print(f"Saving crops under: {output_dir}")
    print(f"Labels: {', '.join(args.labels)}")
    if fixed_label:
        print(f"Fixed label: {fixed_label}")
        print("Place the cup and press Enter to capture. Use 'q' to quit.")
    else:
        print("Enter a label, then place the cup and press Enter. Use 'q' to quit.")

    try:
        while rclpy.ok():
            if fixed_label:
                answer = input(f"\n{fixed_label}> ").strip()
                if answer.lower() in {"q", "quit", "exit"}:
                    break
                label = fixed_label
            else:
                label = input("\nlabel> ").strip()
                if label.lower() in {"q", "quit", "exit"}:
                    break
                if label not in args.labels:
                    print(f"Unknown label {label!r}; expected one of: {', '.join(args.labels)}")
                    continue
                input("Place cup, then press Enter to capture: ")
            image, detection = wait_for_sample(node, args.wait_sec)
            if image is None or detection is None:
                print("NO_SAMPLE_TIMEOUT")
                continue
            parsed = parse_bbox_and_center(detection.status)
            if parsed is None:
                print("Detection status has no bbox/center. Rebuild detector so rejected statuses include center=(u,v).")
                print(detection.status)
                continue
            width, height, center_u, center_v = parsed
            crop = crop_detection(image, width, height, center_u, center_v, args.pad)
            if crop.size == 0:
                print("Empty crop; skipping")
                continue
            stamp = f"{detection.header.stamp.sec}_{detection.header.stamp.nanosec:09d}"
            filename = f"{label}_{stamp}_{center_u}_{center_v}_{width}x{height}.jpg"
            crop_path = output_dir / label / filename
            cv2.imwrite(str(crop_path), crop)
            record = {
                "label": label,
                "crop_path": str(crop_path),
                "status": detection.status,
                "confidence": float(detection.confidence),
                "center_u": center_u,
                "center_v": center_v,
                "bbox_width": width,
                "bbox_height": height,
            }
            with metadata_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
            print(f"saved: {crop_path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
