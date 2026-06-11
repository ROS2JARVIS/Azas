#!/usr/bin/env python3
"""Sample a ROS color image and report visible ArUco marker dictionaries/IDs."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


DEFAULT_DICTIONARIES = [
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_4X4_250",
    "DICT_5X5_50",
    "DICT_5X5_100",
    "DICT_5X5_250",
    "DICT_6X6_50",
    "DICT_6X6_100",
    "DICT_6X6_250",
    "DICT_7X7_50",
    "DICT_7X7_100",
    "DICT_7X7_250",
]


def image_to_bgr(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    channels = 3 if encoding in {"rgb8", "bgr8"} else 1
    array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
    if encoding == "rgb8":
        return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    if encoding == "bgr8":
        return array.copy()
    if encoding in {"mono8", "8uc1"}:
        return cv2.cvtColor(array.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported image encoding: {msg.encoding}")


def aruco_dictionary(name: str):
    dictionary_id = getattr(cv2.aruco, name, None)
    if dictionary_id is None:
        return None
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dictionary_id)
    return cv2.aruco.Dictionary_get(dictionary_id)


def aruco_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()
    tuned_values = {
        "adaptiveThreshWinSizeMax": 53,
        "perspectiveRemovePixelPerCell": 8,
    }
    for name, value in tuned_values.items():
        if hasattr(parameters, name):
            setattr(parameters, name, value)
    return parameters


def detect_markers(gray: np.ndarray, dictionary, parameters):
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


class ImageSampler(Node):
    def __init__(self, topic: str):
        super().__init__("azas_aruco_marker_diagnostic")
        self.msg: Image | None = None
        self.create_subscription(Image, topic, self._on_image, 10)

    def _on_image(self, msg: Image) -> None:
        if self.msg is None:
            self.msg = msg


def sample_image(topic: str, timeout_sec: float) -> Image:
    node = ImageSampler(topic)
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    try:
        while rclpy.ok() and node.msg is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.msg is None:
            raise RuntimeError(f"timed out waiting for image on {topic}")
        return node.msg
    finally:
        node.destroy_node()


def marker_summary(corners: np.ndarray) -> dict[str, Any]:
    points = np.asarray(corners, dtype=float).reshape(4, 2)
    center = points.mean(axis=0)
    side_lengths = [
        float(np.linalg.norm(points[(index + 1) % 4] - points[index]))
        for index in range(4)
    ]
    return {
        "center_u": round(float(center[0]), 2),
        "center_v": round(float(center[1]), 2),
        "side_px": round(float(np.mean(side_lengths)), 2),
        "corners": [[round(float(x), 2), round(float(y), 2)] for x, y in points],
    }


def diagnose(image_bgr: np.ndarray, dictionaries: list[str], expected_id: int) -> tuple[list[dict[str, Any]], np.ndarray]:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    overlay = image_bgr.copy()
    results: list[dict[str, Any]] = []
    for name in dictionaries:
        dictionary = aruco_dictionary(name)
        if dictionary is None:
            results.append({"dictionary": name, "available": False, "markers": [], "rejected": 0})
            continue
        corners_list, ids, rejected = detect_markers(gray, dictionary, aruco_parameters())
        markers = []
        if ids is not None:
            for corners, marker_id_array in zip(corners_list, ids):
                marker_id = int(marker_id_array[0])
                summary = marker_summary(corners)
                summary["id"] = marker_id
                summary["matches_expected_id"] = expected_id < 0 or marker_id == expected_id
                markers.append(summary)
            if markers:
                cv2.aruco.drawDetectedMarkers(overlay, corners_list, ids)
        results.append(
            {
                "dictionary": name,
                "available": True,
                "markers": markers,
                "rejected": len(rejected) if rejected is not None else 0,
            }
        )
    return results, overlay


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--expected-dictionary", default="DICT_4X4_50")
    parser.add_argument("--expected-id", type=int, default=14)
    parser.add_argument("--all-dictionaries", action="store_true")
    parser.add_argument("--debug-image", default="outputs/aruco_marker_diagnostic.jpg")
    parser.add_argument("--json-output", default="outputs/aruco_marker_diagnostic.json")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    try:
        msg = sample_image(args.topic, args.timeout_sec)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    image_bgr = image_to_bgr(msg)
    dictionaries = DEFAULT_DICTIONARIES if args.all_dictionaries else [args.expected_dictionary]
    results, overlay = diagnose(image_bgr, dictionaries, args.expected_id)
    payload = {
        "topic": args.topic,
        "encoding": msg.encoding,
        "width": msg.width,
        "height": msg.height,
        "expected_dictionary": args.expected_dictionary,
        "expected_id": args.expected_id,
        "opencv_version": cv2.__version__,
        "results": results,
    }

    debug_path = Path(args.debug_image)
    debug_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(debug_path), overlay)
    json_path = Path(args.json_output)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    expected_hits = [
        marker
        for result in results
        if result.get("dictionary") == args.expected_dictionary
        for marker in result.get("markers", [])
        if marker.get("matches_expected_id")
    ]
    any_hits = [
        (result.get("dictionary"), marker)
        for result in results
        for marker in result.get("markers", [])
    ]
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    print(f"[Azas] debug_image={debug_path}")
    print(f"[Azas] json_output={json_path}")
    if expected_hits:
        print(f"[PASS] expected marker visible: {args.expected_dictionary} id={args.expected_id}")
    elif any_hits:
        print("[WARN] ArUco marker(s) visible, but expected dictionary/id did not match")
    else:
        print("[FAIL] no ArUco marker detected in sampled color frame")


if __name__ == "__main__":
    main()
