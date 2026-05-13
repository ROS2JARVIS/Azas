#!/usr/bin/env python3
"""Export one RGB-D frame for offline grasp-detector experiments.

This tool records perception inputs only. It does not command robot motion,
MoveIt execution, RG2, or any hardware service.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from azas_interfaces.msg import CupDetection
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


DEPTH_AUTO_SCALES = {
    "16uc1": 0.001,
    "mono16": 0.001,
    "32fc1": 1.0,
}


class GraspFrameExporter(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("azas_grasp_frame_exporter")
        self.args = args
        self.rgb_msg: Image | None = None
        self.depth_msg: Image | None = None
        self.info_msg: CameraInfo | None = None
        self.detection_msg: CupDetection | None = None
        self.mask_msg: Image | None = None

        self.create_subscription(Image, args.rgb_topic, self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.camera_info_topic, self._on_info, 10)
        if args.detection_topic:
            self.create_subscription(CupDetection, args.detection_topic, self._on_detection, 10)
        if args.mask_topic:
            self.create_subscription(Image, args.mask_topic, self._on_mask, qos_profile_sensor_data)

    def _on_rgb(self, msg: Image) -> None:
        self.rgb_msg = msg

    def _on_depth(self, msg: Image) -> None:
        self.depth_msg = msg

    def _on_info(self, msg: CameraInfo) -> None:
        self.info_msg = msg

    def _on_detection(self, msg: CupDetection) -> None:
        self.detection_msg = msg

    def _on_mask(self, msg: Image) -> None:
        self.mask_msg = msg

    def ready(self) -> bool:
        required = self.rgb_msg is not None and self.depth_msg is not None and self.info_msg is not None
        if self.args.wait_for_bbox:
            required = required and self.detection_msg is not None
        if self.args.mask_topic and self.args.wait_for_mask:
            required = required and self.mask_msg is not None
        return required

    def export(self) -> dict[str, Any]:
        if self.rgb_msg is None or self.depth_msg is None or self.info_msg is None:
            raise RuntimeError("missing required RGB-D/CameraInfo messages")

        output_dir = Path(self.args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        rgb = image_to_bgr(self.rgb_msg)
        depth_m, depth_metadata = depth_to_meters(
            self.depth_msg,
            mode=self.args.depth_scale_mode,
            manual_scale=self.args.depth_scale,
        )
        cv2.imwrite(str(output_dir / "rgb.png"), rgb)
        np.save(output_dir / "depth_m.npy", depth_m.astype(np.float32))
        write_json(output_dir / "camera_info.json", camera_info_json(self.info_msg))

        bbox = None
        if self.detection_msg is not None:
            bbox = detection_to_bbox_json(self.detection_msg)
            write_json(output_dir / "bbox.json", bbox)

        if self.mask_msg is not None:
            mask = image_to_mask(self.mask_msg)
            cv2.imwrite(str(output_dir / "mask.png"), mask)

        manifest = {
            "format": "azas_grasp_frame_v1",
            "output_dir": str(output_dir),
            "rgb": "rgb.png",
            "depth_m": "depth_m.npy",
            "camera_info": "camera_info.json",
            "bbox": "bbox.json" if bbox is not None else None,
            "mask": "mask.png" if self.mask_msg is not None else None,
            "topics": {
                "rgb": self.args.rgb_topic,
                "depth": self.args.depth_topic,
                "camera_info": self.args.camera_info_topic,
                "detection": self.args.detection_topic,
                "mask": self.args.mask_topic,
            },
            "frames": {
                "rgb": self.rgb_msg.header.frame_id,
                "depth": self.depth_msg.header.frame_id,
                "camera_info": self.info_msg.header.frame_id,
            },
            "depth": depth_metadata,
            "safety": {
                "robot_motion": False,
                "moveit_execute": False,
                "rg2_command": False,
            },
        }
        write_json(output_dir / "manifest.json", manifest)
        return manifest


def image_to_array(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    dtype_by_encoding = {
        "8uc1": np.uint8,
        "mono8": np.uint8,
        "8uc3": np.uint8,
        "rgb8": np.uint8,
        "bgr8": np.uint8,
        "16uc1": np.uint16,
        "mono16": np.uint16,
        "32fc1": np.float32,
    }
    channels_by_encoding = {
        "8uc1": 1,
        "mono8": 1,
        "8uc3": 3,
        "rgb8": 3,
        "bgr8": 3,
        "16uc1": 1,
        "mono16": 1,
        "32fc1": 1,
    }
    if encoding not in dtype_by_encoding:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")
    dtype = dtype_by_encoding[encoding]
    channels = channels_by_encoding[encoding]
    itemsize = np.dtype(dtype).itemsize
    row_values = msg.step // itemsize
    data = np.frombuffer(msg.data, dtype=dtype)
    if msg.is_bigendian != (data.dtype.byteorder == ">"):
        data = data.byteswap().newbyteorder()
    if channels == 1:
        image = data.reshape((msg.height, row_values))[:, : msg.width]
    else:
        image = data.reshape((msg.height, row_values // channels, channels))[:, : msg.width, :]
    return np.ascontiguousarray(image)


def image_to_bgr(msg: Image) -> np.ndarray:
    image = image_to_array(msg)
    encoding = msg.encoding.lower()
    if encoding == "bgr8":
        return image
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding in {"mono8", "8uc1"}:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if encoding == "8uc3":
        return image
    raise ValueError(f"unsupported RGB image encoding: {msg.encoding}")


def image_to_mask(msg: Image) -> np.ndarray:
    mask = image_to_array(msg)
    if mask.ndim == 3:
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    return np.where(mask > 0, 255, 0).astype(np.uint8)


def depth_to_meters(msg: Image, mode: str, manual_scale: float) -> tuple[np.ndarray, dict[str, Any]]:
    encoding = msg.encoding.lower()
    depth = image_to_array(msg).astype(np.float32)
    if mode == "manual":
        scale = float(manual_scale)
        if not math.isfinite(scale) or scale <= 0.0:
            raise ValueError(f"manual depth_scale must be positive and finite, got {manual_scale}")
    elif mode == "auto":
        if encoding not in DEPTH_AUTO_SCALES:
            raise ValueError(
                f"unsupported depth encoding {msg.encoding!r}; expected 16UC1, mono16, or 32FC1"
            )
        scale = DEPTH_AUTO_SCALES[encoding]
    else:
        raise ValueError(f"unsupported depth_scale_mode={mode!r}; use auto or manual")

    depth_m = depth * scale
    depth_m[~np.isfinite(depth_m)] = np.nan
    depth_m[depth_m <= 0.0] = np.nan
    return depth_m, {
        "encoding": msg.encoding,
        "scale_mode": mode,
        "scale": scale,
        "unit": "meter",
        "finite_positive_count": int(np.isfinite(depth_m).sum()),
        "shape": [int(depth_m.shape[0]), int(depth_m.shape[1])],
    }


def camera_info_json(msg: CameraInfo) -> dict[str, Any]:
    fx = float(msg.k[0])
    fy = float(msg.k[4])
    cx = float(msg.k[2])
    cy = float(msg.k[5])
    return {
        "header": {
            "frame_id": msg.header.frame_id,
            "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
        },
        "height": int(msg.height),
        "width": int(msg.width),
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "distortion_model": msg.distortion_model,
        "d": [float(v) for v in msg.d],
        "k": [float(v) for v in msg.k],
        "r": [float(v) for v in msg.r],
        "p": [float(v) for v in msg.p],
        "K": [
            [fx, float(msg.k[1]), cx],
            [float(msg.k[3]), fy, cy],
            [float(msg.k[6]), float(msg.k[7]), float(msg.k[8])],
        ],
    }


def detection_to_bbox_json(msg: CupDetection) -> dict[str, Any]:
    status = msg.status or ""
    bbox_match = re.search(r"bbox=(\d+)x(\d+)", status)
    center_match = re.search(r"center=\(([-+]?\d+),([-+]?\d+)\)", status)
    class_match = re.search(r"class=([A-Za-z0-9_.-]+)", status)
    xyxy_match = re.search(
        r"xyxy=\(([-+]?\d+),([-+]?\d+),([-+]?\d+),([-+]?\d+)\)", status
    )
    return {
        "header": {
            "frame_id": msg.header.frame_id,
            "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
        },
        "status": status,
        "source": msg.source,
        "confidence": float(msg.confidence),
        "class": class_match.group(1) if class_match else None,
        "class_name": class_match.group(1) if class_match else None,
        "bbox_width": int(bbox_match.group(1)) if bbox_match else None,
        "bbox_height": int(bbox_match.group(2)) if bbox_match else None,
        "center_u": int(center_match.group(1)) if center_match else None,
        "center_v": int(center_match.group(2)) if center_match else None,
        "xyxy": [int(xyxy_match.group(i)) for i in range(1, 5)] if xyxy_match else None,
        "note": "xyxy is null unless the CupDetection status includes xyxy=(x1,y1,x2,y2)",
    }


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", "--output", dest="output_dir", default="/tmp/azas_grasp_frame")
    parser.add_argument("--rgb-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--detection-topic", default="/azas/cup_detection")
    parser.add_argument("--mask-topic", default="")
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--depth-scale-mode", choices=["auto", "manual"], default="auto")
    parser.add_argument("--depth-scale", type=float, default=0.001)
    parser.add_argument("--wait-for-bbox", action="store_true")
    parser.add_argument("--wait-for-mask", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = GraspFrameExporter(args)
    deadline = time.monotonic() + max(args.timeout_sec, 0.0)
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.ready():
            missing = []
            if node.rgb_msg is None:
                missing.append(args.rgb_topic)
            if node.depth_msg is None:
                missing.append(args.depth_topic)
            if node.info_msg is None:
                missing.append(args.camera_info_topic)
            if args.wait_for_bbox and node.detection_msg is None:
                missing.append(args.detection_topic)
            if args.wait_for_mask and node.mask_msg is None:
                missing.append(args.mask_topic)
            raise TimeoutError(f"timed out waiting for required topics: {', '.join(missing)}")
        manifest = node.export()
        print(f"[OK] exported Azas grasp frame: {manifest['output_dir']}")
        print("[OK] wrote rgb.png, depth_m.npy, camera_info.json, manifest.json")
        if manifest.get("bbox"):
            print("[OK] wrote bbox.json")
        if manifest.get("mask"):
            print("[OK] wrote mask.png")
        print("[Azas] no robot motion, MoveIt execute, or RG2 command was used")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
