#!/usr/bin/env python3
"""Publish visible M0609 joint targets to the Gazebo ros2_control position controller.

The target is the Doosan Gazebo namespace created by
tools/run/run_m0609_gazebo_ros2_control.sh. This sends only ROS simulation
controller commands, not Doosan hardware service calls.
"""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Drive M0609 Gazebo ros2_control preview motion.")
    parser.add_argument("--robot-name", default="dsr01")
    parser.add_argument("--controller-namespace", default=None)
    parser.add_argument("--period-sec", type=float, default=2.0)
    parser.add_argument("--hold-sec", type=float, default=0.4)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--cycles", type=int, default=0, help="0 means loop forever")
    parser.add_argument("--once", action="store_true", help="smoothly move to one target and exit")
    parser.add_argument(
        "--target",
        type=float,
        nargs=6,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="single joint target in radians; use with --once or as the first target",
    )
    return parser.parse_args()


def controller_topic(args: argparse.Namespace) -> str:
    namespace = args.controller_namespace
    if namespace is None:
        namespace = f"/{args.robot_name.strip('/')}/gz"
    namespace = "/" + namespace.strip("/")
    return f"{namespace}/dsr_position_controller/commands"


def default_sequence() -> list[list[float]]:
    return [
        [0.0, 0.0, 1.20, 0.0, 1.30, 0.0],
        [0.18, -0.28, 1.28, 0.10, 1.20, 0.18],
        [-0.12, -0.24, 1.16, -0.10, 1.32, -0.18],
        [0.10, -0.38, 1.36, 0.16, 1.14, 0.28],
        [0.0, 0.0, 1.20, 0.0, 1.30, 0.0],
    ]


def smoothstep(t: float) -> float:
    return t * t * (3.0 - 2.0 * t)


def interpolate(start: list[float], goal: list[float], fraction: float) -> list[float]:
    blend = smoothstep(max(0.0, min(1.0, fraction)))
    return [a + (b - a) * blend for a, b in zip(start, goal)]


class GazeboMotionPublisher(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("azas_m0609_gazebo_demo_motion")
        self.publisher = self.create_publisher(Float64MultiArray, topic, 10)
        self.topic = topic

    def publish_target(self, target: list[float], *, log: bool = False) -> None:
        msg = Float64MultiArray()
        msg.data = [float(value) for value in target]
        self.publisher.publish(msg)
        if log:
            self.get_logger().info(
                "published Gazebo target "
                + ", ".join(f"{name}={value:.3f}" for name, value in zip(JOINT_NAMES, msg.data))
            )

    def move_smoothly(self, start: list[float], goal: list[float], duration_sec: float, rate_hz: float) -> None:
        steps = max(2, int(duration_sec * rate_hz))
        step_period = 1.0 / rate_hz
        for step in range(steps + 1):
            if not rclpy.ok():
                return
            self.publish_target(interpolate(start, goal, step / steps), log=(step == steps))
            deadline = time.monotonic() + step_period
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.005)

    def hold(self, target: list[float], duration_sec: float, rate_hz: float) -> None:
        end_time = time.monotonic() + max(0.0, duration_sec)
        step_period = 1.0 / rate_hz
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_target(target)
            deadline = time.monotonic() + step_period
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.005)


def main() -> int:
    args = parse_args()
    if args.period_sec <= 0.0:
        raise SystemExit("--period-sec must be > 0")
    if args.hold_sec < 0.0:
        raise SystemExit("--hold-sec must be >= 0")
    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be > 0")

    rclpy.init(args=None)
    node = GazeboMotionPublisher(controller_topic(args))
    try:
        # Give discovery a moment so the first message is not lost when the user
        # starts this immediately after launching Gazebo.
        end_discovery = time.monotonic() + 1.0
        while time.monotonic() < end_discovery:
            rclpy.spin_once(node, timeout_sec=0.05)

        sequence = default_sequence()
        if args.target is not None:
            sequence = [list(args.target)] if args.once else [list(args.target), *sequence]
        current = sequence[-1] if not args.once else default_sequence()[0]
        if args.once:
            node.move_smoothly(current, sequence[0], args.period_sec, args.rate_hz)
            node.hold(sequence[0], args.hold_sec, args.rate_hz)
            return 0

        completed_cycles = 0
        while rclpy.ok():
            for target in sequence:
                node.move_smoothly(current, target, args.period_sec, args.rate_hz)
                node.hold(target, args.hold_sec, args.rate_hz)
                current = target
            completed_cycles += 1
            if args.cycles > 0 and completed_cycles >= args.cycles:
                return 0
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
