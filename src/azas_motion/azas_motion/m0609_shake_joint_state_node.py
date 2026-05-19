#!/usr/bin/env python3
"""Visual-only M0609 joint states for RViz dry-run previews."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class M0609ShakeJointStateNode(Node):
    def __init__(self) -> None:
        super().__init__("m0609_shake_joint_state_node")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("shake_cycles_per_second", 4.0)
        self.declare_parameter("preview_mode", "shake")
        self.declare_parameter(
            "home_joints_rad",
            [0.0, math.radians(-35.0), math.radians(-55.0), 0.0, math.radians(70.0), 0.0],
        )

        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.start_time = self.get_clock().now()
        rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self.publish_joint_state)
        self.get_logger().info(
            "Publishing RViz-only M0609 joint states for side-grasp / shake visualization."
        )

    def publish_joint_state(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        home = [float(value) for value in self.get_parameter("home_joints_rad").value]
        while len(home) < 6:
            home.append(0.0)

        mode = str(self.get_parameter("preview_mode").value).strip().lower()
        if mode in {"cup_target_move", "side_grasp_target_move", "target_move"}:
            positions = self.cup_target_move_joints(elapsed, home)
        elif mode in {"side_grasp_move_then_shake", "side_grasp_then_shake", "move_then_shake"}:
            positions = self.side_grasp_move_then_shake_joints(elapsed, home)
        else:
            positions = self.high_shake_joints(elapsed, home)

        positions[4] = max(min(positions[4], math.radians(100.0)), math.radians(40.0))

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        msg.position = positions
        self.publisher.publish(msg)

    def high_shake_joints(self, elapsed: float, home: list[float]) -> list[float]:
        freq = max(float(self.get_parameter("shake_cycles_per_second").value), 0.1)
        phase = elapsed * math.tau * freq
        j5_swing = math.sin(phase)
        wrist_counter = math.sin(phase + math.pi * 0.5)
        elbow_pulse = math.sin(phase * 0.5)
        wrist_snap = math.sin(phase * 1.7 + math.pi * 0.25)

        return [
            home[0],
            home[1],
            home[2],
            home[3] + math.radians(24.0) * wrist_counter,
            home[4] + math.radians(30.0) * j5_swing,
            home[5] + math.radians(36.0) * wrist_counter + math.radians(8.0) * wrist_snap,
        ]

    def side_grasp_move_then_shake_joints(self, elapsed: float, home: list[float]) -> list[float]:
        keyframes = [
            (0.0, home),
            (2.0, [0.18, -0.76, 1.48, -0.22, 1.18, 0.20]),  # side pre-grasp
            (4.0, [0.25, -0.82, 1.56, -0.10, 1.12, 0.12]),  # side grasp
            (7.5, [0.30, -0.55, 1.22, 0.05, 1.05, 0.10]),  # slower lift with cup
            (9.5, [-0.12, -0.48, 1.10, 0.18, 1.05, -0.10]),  # carry to dispenser
            (11.0, [-0.20, -0.60, 1.30, 0.10, 1.18, 0.00]),  # outlet front
            (13.0, [-0.05, -0.42, 1.05, 0.00, 1.02, 0.00]),  # retreat before shake
        ]
        cycle_seconds = 19.0
        t = elapsed % cycle_seconds
        if t >= keyframes[-1][0]:
            shake_elapsed = t - keyframes[-1][0]
            shake_home = keyframes[-1][1]
            return self.high_shake_joints(shake_elapsed, shake_home)

        for index in range(len(keyframes) - 1):
            start_t, start_joints = keyframes[index]
            end_t, end_joints = keyframes[index + 1]
            if start_t <= t < end_t:
                ratio = (t - start_t) / max(end_t - start_t, 1e-6)
                smooth = 0.5 - 0.5 * math.cos(math.pi * ratio)
                return [
                    start + (end - start) * smooth
                    for start, end in zip(start_joints, end_joints)
                ]
        return home

    def cup_target_move_joints(self, elapsed: float, home: list[float]) -> list[float]:
        safe_j5 = math.radians(120.0)
        keyframes = [
            (0.0, home),
            (2.0, [-0.55, 0.90, 1.35, 0.0, safe_j5, 1.57]),  # low side pre-grasp
            (4.0, [-0.50, 0.95, 1.35, 0.0, safe_j5, 1.57]),  # low side grasp
            (6.0, [-0.15, 0.95, 1.35, 0.0, safe_j5, 1.57]),  # move while staying low
            (8.0, [0.20, 0.95, 1.35, 0.0, safe_j5, 1.57]),  # low target hold
            (10.0, [0.20, 0.95, 1.35, 0.0, safe_j5, 1.57]),  # hold target
        ]
        cycle_seconds = 12.0
        t = elapsed % cycle_seconds
        if t >= keyframes[-1][0]:
            return keyframes[-1][1]

        for index in range(len(keyframes) - 1):
            start_t, start_joints = keyframes[index]
            end_t, end_joints = keyframes[index + 1]
            if start_t <= t < end_t:
                ratio = (t - start_t) / max(end_t - start_t, 1e-6)
                smooth = 0.5 - 0.5 * math.cos(math.pi * ratio)
                return [
                    start + (end - start) * smooth
                    for start, end in zip(start_joints, end_joints)
                ]
        return home


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
