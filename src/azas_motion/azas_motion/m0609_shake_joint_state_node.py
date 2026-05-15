#!/usr/bin/env python3
"""Visual-only M0609 joint states for the high-shake RViz dry-run."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class M0609ShakeJointStateNode(Node):
    def __init__(self) -> None:
        super().__init__("m0609_shake_joint_state_node")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("shake_cycles_per_second", 2.4)
        self.declare_parameter("home_joints_rad", [0.0, -0.62, 1.38, 0.0, 1.22, 0.0])

        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.start_time = self.get_clock().now()
        rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self.publish_joint_state)
        self.get_logger().info(
            "Publishing RViz-only M0609 joint states for high lifted shake visualization."
        )

    def publish_joint_state(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        freq = max(float(self.get_parameter("shake_cycles_per_second").value), 0.1)
        home = [float(value) for value in self.get_parameter("home_joints_rad").value]
        while len(home) < 6:
            home.append(0.0)

        phase = elapsed * math.tau * freq
        sway = math.sin(phase)
        counter = math.sin(phase + math.pi * 0.5)
        lift_pulse = math.sin(phase * 0.5)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        msg.position = [
            home[0] + 0.30 * sway,
            home[1] + 0.16 * lift_pulse,
            home[2] - 0.20 * lift_pulse,
            home[3] + 0.42 * counter,
            home[4] + 0.28 * sway,
            home[5] + 0.85 * counter,
        ]
        self.publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = M0609ShakeJointStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
