#!/usr/bin/env python3
"""Detect dispenser bottle colors from a RealSense color image and map them to physical slots.

This helper commands no robot motion. It observes the camera image, segments the
known bottle colors, orders detected color blobs left-to-right, and writes a
runtime color->physical-slot map such as ``red=4,green=2,yellow=1,blue=3``.
Physical slot geometry remains measured separately in YAML; this script only
resolves which color bottle currently sits in which numbered slot.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image

DEFAULT_ENV_OUT = Path("/tmp/azas_dispenser_color_map.env")

# Conservative HSV ranges for saturated dispenser liquid/bottle labels.
# Red wraps around hue=0, so it has two intervals.
HSV_RANGES = {
    "red": [((0, 70, 40), (12, 255, 255)), ((170, 70, 40), (179, 255, 255))],
    "yellow": [((18, 60, 50), (42, 255, 255))],
    "green": [((42, 45, 35), (92, 255, 255))],
    "blue": [((92, 45, 35), (135, 255, 255))],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect current dispenser color-to-slot mapping from camera image.")
    parser.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument("--sample-frames", type=int, default=6)
    parser.add_argument("--min-area-ratio", type=float, default=0.00015)
    parser.add_argument(
        "--slot-order-left-to-right",
        default="1,2,3,4",
        help="physical slot IDs in image left-to-right order; use 4,3,2,1 if camera view is mirrored",
    )
    parser.add_argument("--write-env", type=Path, default=DEFAULT_ENV_OUT)
    parser.add_argument("--write-json", type=Path, default=None)
    parser.add_argument("--debug-image", type=Path, default=None, help="write annotated detection image for HTML/log review")
    parser.add_argument("--sample-image", type=Path, default=None, help="write raw sampled/median image")
    parser.add_argument(
        "--roi",
        default="",
        help="optional x1,y1,x2,y2 crop in image pixels; use when only the dispenser area should be scanned",
    )
    parser.add_argument("--allow-partial", action="store_true")
    return parser.parse_args()


def image_msg_to_bgr8(msg: Image) -> np.ndarray:
    """Convert common ROS Image encodings to BGR without cv_bridge.

    cv_bridge can be unavailable when the local NumPy ABI differs from the ROS
    binary build.  The RealSense color stream used here is normally bgr8 or
    rgb8, so a small direct converter is more robust for this panel helper.
    """

    encoding = str(msg.encoding).lower()
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if encoding in {"bgr8", "rgb8"}:
        row = raw.reshape((height, step))[:, : width * 3]
        image = row.reshape((height, width, 3))
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image.copy()
    if encoding in {"bgra8", "rgba8"}:
        row = raw.reshape((height, step))[:, : width * 4]
        image = row.reshape((height, width, 4))
        code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
        return cv2.cvtColor(image, code)
    if encoding in {"mono8", "8uc1"}:
        row = raw.reshape((height, step))[:, :width]
        return cv2.cvtColor(row.reshape((height, width)), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported image encoding for color detection: {msg.encoding}")


class ImageSampler:
    def __init__(self, topic: str):
        self.node = rclpy.create_node("azas_dispenser_color_map_detector")
        self.images: list[np.ndarray] = []
        self.node.create_subscription(Image, topic, self._on_image, 10)

    def _on_image(self, msg: Image) -> None:
        try:
            image = image_msg_to_bgr8(msg)
        except Exception as exc:  # pragma: no cover - live ROS image path
            self.node.get_logger().error(f"image conversion failed: {exc}")
            return
        self.images.append(image)
        if len(self.images) > 20:
            self.images = self.images[-20:]

    def wait(self, sample_frames: int, timeout_sec: float) -> np.ndarray:
        deadline = time.monotonic() + max(timeout_sec, 0.1)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if len(self.images) >= sample_frames:
                break
        if not self.images:
            raise RuntimeError("no color image received; start RealSense camera first")
        stack = np.stack(self.images[-max(1, min(sample_frames, len(self.images))):], axis=0)
        return np.median(stack, axis=0).astype(np.uint8)

    def destroy(self) -> None:
        self.node.destroy_node()


def realsense_usb_status() -> str:
    try:
        output = subprocess.check_output(["lsusb"], text=True, stderr=subprocess.DEVNULL, timeout=2.0)
    except Exception:
        return "unknown"
    return "present" if any(token in output.lower() for token in ("realsense", "intel corp.")) else "missing"


def parse_roi(raw: str, image_shape: tuple[int, int, int]) -> tuple[int, int, int, int] | None:
    if not raw.strip():
        return None
    try:
        values = [int(float(part.strip())) for part in raw.split(",")]
    except ValueError as exc:
        raise ValueError("--roi must be x1,y1,x2,y2") from exc
    if len(values) != 4:
        raise ValueError("--roi must be x1,y1,x2,y2")
    height, width = image_shape[:2]
    x1, y1, x2, y2 = values
    x1 = max(0, min(width - 1, x1))
    x2 = max(1, min(width, x2))
    y1 = max(0, min(height - 1, y1))
    y2 = max(1, min(height, y2))
    if x2 <= x1 or y2 <= y1:
        raise ValueError(f"invalid --roi after clamping: {x1},{y1},{x2},{y2}")
    return x1, y1, x2, y2


def detect_color_blobs(image: np.ndarray, min_area_ratio: float, roi: tuple[int, int, int, int] | None = None) -> dict[str, dict[str, float]]:
    x_offset = 0
    y_offset = 0
    scan_image = image
    if roi is not None:
        x1, y1, x2, y2 = roi
        scan_image = image[y1:y2, x1:x2]
        x_offset = x1
        y_offset = y1
    scan_height, scan_width = scan_image.shape[:2]
    min_area = max(20.0, scan_width * scan_height * min_area_ratio)
    hsv = cv2.cvtColor(scan_image, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    detections: dict[str, dict[str, float]] = {}
    for color, ranges in HSV_RANGES.items():
        mask = np.zeros((scan_height, scan_width), dtype=np.uint8)
        for low, high in ranges:
            mask |= cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < min_area:
            continue
        moments = cv2.moments(contour)
        if abs(moments["m00"]) < 1e-9:
            continue
        cx = float(moments["m10"] / moments["m00"])
        cy = float(moments["m01"] / moments["m00"])
        x, y, w, h = cv2.boundingRect(contour)
        detections[color] = {
            "x_px": cx + x_offset,
            "y_px": cy + y_offset,
            "area_px": area,
            "area_ratio": area / float(scan_width * scan_height),
            "bbox_xywh": [float(x + x_offset), float(y + y_offset), float(w), float(h)],
        }
    return detections


def draw_debug_image(
    image: np.ndarray,
    detections: dict[str, dict[str, float]],
    mapping: dict[str, str],
    roi: tuple[int, int, int, int] | None = None,
) -> np.ndarray:
    out = image.copy()
    palette = {
        "red": (0, 0, 255),
        "yellow": (0, 255, 255),
        "green": (0, 255, 0),
        "blue": (255, 0, 0),
    }
    if roi is not None:
        x1, y1, x2, y2 = roi
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.putText(out, "ROI", (x1 + 5, max(20, y1 + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    for color, info in detections.items():
        x, y, w, h = [int(round(v)) for v in info.get("bbox_xywh", [info["x_px"], info["y_px"], 1, 1])]
        cx, cy = int(round(info["x_px"])), int(round(info["y_px"]))
        bgr = palette.get(color, (255, 255, 255))
        cv2.rectangle(out, (x, y), (x + w, y + h), bgr, 2)
        cv2.circle(out, (cx, cy), 5, bgr, -1)
        label = f"{color}->#{mapping.get(color, '?')}"
        cv2.putText(out, label, (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, bgr, 2)
    return out


def build_mapping(detections: dict[str, dict[str, float]], slot_order: list[str]) -> dict[str, str]:
    ordered = sorted(detections.items(), key=lambda item: item[1]["x_px"])
    if len(ordered) > len(slot_order):
        ordered = ordered[: len(slot_order)]
    return {color: slot_order[index] for index, (color, _info) in enumerate(ordered)}


def main() -> int:
    args = parse_args()
    slot_order = [item.strip() for item in args.slot_order_left_to_right.split(",") if item.strip()]
    if sorted(slot_order) != ["1", "2", "3", "4"]:
        print("[FAIL] --slot-order-left-to-right must contain exactly 1,2,3,4")
        return 2

    rclpy.init(args=None)
    sampler = ImageSampler(args.image_topic)
    try:
        image = sampler.wait(args.sample_frames, args.timeout_sec)
    except Exception as exc:
        print(f"[FAIL] {exc}")
        usb_status = realsense_usb_status()
        if usb_status == "missing":
            print("[Azas] RealSense USB 장치가 lsusb에 보이지 않습니다. 케이블/전원/USB 포트를 확인한 뒤 '카메라 시작'을 다시 누르세요.")
        elif usb_status == "present":
            print("[Azas] RealSense USB는 보이지만 color image가 안 들어옵니다. 패널의 '프로세스 정리' 후 '카메라 시작'을 다시 실행하세요.")
        sampler.destroy()
        if rclpy.ok():
            rclpy.shutdown()
        return 2
    finally:
        pass
    sampler.destroy()
    if rclpy.ok():
        rclpy.shutdown()

    if args.sample_image:
        args.sample_image.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.sample_image), image)
    try:
        roi = parse_roi(args.roi, image.shape)
    except ValueError as exc:
        print(f"[FAIL] {exc}")
        return 2
    detections = detect_color_blobs(image, args.min_area_ratio, roi)
    missing = sorted(set(HSV_RANGES) - set(detections))
    if missing and not args.allow_partial:
        print(f"[FAIL] missing color detection(s): {', '.join(missing)}")
        print("[Azas] detections=" + json.dumps(detections, ensure_ascii=False, sort_keys=True))
        return 1

    mapping = build_mapping(detections, slot_order)
    map_text = ",".join(f"{color}={slot}" for color, slot in sorted(mapping.items()))
    result = {
        "dispenser_color_map": map_text,
        "slot_order_left_to_right": slot_order,
        "detections": detections,
        "missing_colors": missing,
        "roi_xyxy": list(roi) if roi is not None else None,
        "debug_image": str(args.debug_image) if args.debug_image else None,
        "sample_image": str(args.sample_image) if args.sample_image else None,
    }
    args.write_env.parent.mkdir(parents=True, exist_ok=True)
    args.write_env.write_text(f"DISPENSER_COLOR_MAP={map_text}\n", encoding="utf-8")
    if args.debug_image:
        args.debug_image.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.debug_image), draw_debug_image(image, detections, mapping, roi))
    if args.write_json:
        args.write_json.write_text(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
    print("[PASS] dispenser color map detected")
    print(f"DISPENSER_COLOR_MAP={map_text}")
    for color, slot in sorted(mapping.items()):
        info = detections[color]
        print(
            f"[Azas] {color} -> dispenser_{slot} "
            f"x={info['x_px']:.1f}px y={info['y_px']:.1f}px area_ratio={info['area_ratio']:.5f}"
        )
    if args.debug_image:
        print(f"[Azas] debug_image={args.debug_image}")
    if args.sample_image:
        print(f"[Azas] sample_image={args.sample_image}")
    if missing:
        print("[WARN] partial detection; missing=" + ",".join(missing))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
