#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import time
from collections import Counter

import rclpy
from azas_interfaces.msg import CupDetection
from rclpy.node import Node


CLASS_RE = re.compile(r"^detected:([^ ]+)")
DEPTH_RE = re.compile(r"depth_raw=([0-9.]+)")


class DetectionSampler(Node):
    def __init__(self, topic: str):
        super().__init__("azas_detection_stability_check")
        self.samples: list[CupDetection] = []
        self.create_subscription(CupDetection, topic, self._on_detection, 20)

    def _on_detection(self, msg: CupDetection) -> None:
        self.samples.append(msg)


def detected_class(status: str) -> str | None:
    match = CLASS_RE.search(status)
    return match.group(1) if match else None


def depth_raw(status: str) -> float | None:
    match = DEPTH_RE.search(status)
    return float(match.group(1)) if match else None


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Sample /azas/cup_detection and summarize cup/lid stability."
    )
    parser.add_argument("--topic", default="/azas/cup_detection")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--min-detected-ratio", type=float, default=0.7)
    parser.add_argument("--expect-class", default="", choices=["", "cup", "lid"])
    args = parser.parse_args()

    rclpy.init()
    node = DetectionSampler(args.topic)
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        samples = list(node.samples)
        node.destroy_node()
        rclpy.shutdown()

    total = len(samples)
    status_counts: Counter[str] = Counter(msg.status for msg in samples)
    class_counts: Counter[str] = Counter()
    detected_samples = []
    for msg in samples:
        cls = detected_class(msg.status)
        if cls is None:
            continue
        class_counts[cls] += 1
        detected_samples.append(msg)

    print(f"[Azas] Detection stability topic={args.topic} duration={args.duration:.1f}s")
    print(f"[Azas] samples={total} detected={len(detected_samples)}")

    if status_counts:
        print("[Azas] status counts:")
        for status, count in status_counts.most_common():
            print(f"  {count:3d} {status}")

    if class_counts:
        print("[Azas] detected class counts:")
        for cls, count in class_counts.most_common():
            confidences = [msg.confidence for msg in detected_samples if detected_class(msg.status) == cls]
            depths = [
                value
                for msg in detected_samples
                if detected_class(msg.status) == cls
                for value in [depth_raw(msg.status)]
                if value is not None
            ]
            print(
                f"  {count:3d} {cls} "
                f"confidence={min(confidences):.3f}-{max(confidences):.3f} "
                f"depth_raw={min(depths):.1f}-{max(depths):.1f}"
            )

    if samples:
        latest = samples[-1]
        print(
            "[Azas] latest "
            f"status='{latest.status}' confidence={latest.confidence:.3f} "
            f"frame={latest.header.frame_id} "
            f"grasp=({latest.grasp_pose.position.x:.4f},"
            f"{latest.grasp_pose.position.y:.4f},"
            f"{latest.grasp_pose.position.z:.4f})"
        )

    failures = 0
    if total < args.min_samples:
        print(f"[FAIL] not enough samples: {total} < {args.min_samples}")
        failures += 1

    detected_ratio = (len(detected_samples) / total) if total else 0.0
    if detected_ratio < args.min_detected_ratio:
        print(
            f"[FAIL] detected ratio too low: {detected_ratio:.2f} < {args.min_detected_ratio:.2f}"
        )
        failures += 1
    else:
        print(f"[PASS] detected ratio: {detected_ratio:.2f}")

    if args.expect_class:
        expected_ratio = (class_counts[args.expect_class] / total) if total else 0.0
        if expected_ratio < args.min_detected_ratio:
            print(
                f"[FAIL] expected class '{args.expect_class}' ratio too low: "
                f"{expected_ratio:.2f} < {args.min_detected_ratio:.2f}"
            )
            failures += 1
        else:
            print(f"[PASS] expected class '{args.expect_class}' ratio: {expected_ratio:.2f}")

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
