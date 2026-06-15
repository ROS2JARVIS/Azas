#!/usr/bin/env python3
"""Measure ROS Image topic rate and header age.

Useful for separating camera delay from perception/viewer delay:
  - raw camera topic has low age: camera/DDS is fine
  - overlay topic has high age: perception or viewer path is lagging
"""
from __future__ import annotations

import argparse
import statistics
import time

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


class ImageLatencyCheck(Node):
    def __init__(self, topic: str, samples: int) -> None:
        super().__init__("azas_image_latency_check")
        self.topic = topic
        self.samples = samples
        self.ages_ms: list[float] = []
        self.arrival_times: list[float] = []
        self.create_subscription(Image, topic, self.on_image, LOW_LATENCY_QOS)

    def on_image(self, msg: Image) -> None:
        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        age_ms = (now - stamp).nanoseconds / 1_000_000.0
        self.ages_ms.append(age_ms)
        self.arrival_times.append(time.monotonic())

    @property
    def done(self) -> bool:
        return len(self.ages_ms) >= self.samples


def percentile(values: list[float], pct: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((pct / 100.0) * (len(ordered) - 1))))
    return ordered[index]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic")
    parser.add_argument("--samples", type=int, default=120)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = ImageLatencyCheck(args.topic, args.samples)
    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.ages_ms:
            print(f"[FAIL] no image samples received from {args.topic}")
            return 2
        duration = max(node.arrival_times[-1] - node.arrival_times[0], 1e-6)
        rate = (len(node.arrival_times) - 1) / duration if len(node.arrival_times) > 1 else 0.0
        print(f"topic: {args.topic}")
        print(f"samples: {len(node.ages_ms)}")
        print(f"rate_hz: {rate:.2f}")
        print(f"age_ms_avg: {statistics.mean(node.ages_ms):.1f}")
        print(f"age_ms_p50: {statistics.median(node.ages_ms):.1f}")
        print(f"age_ms_p95: {percentile(node.ages_ms, 95):.1f}")
        print(f"age_ms_max: {max(node.ages_ms):.1f}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
