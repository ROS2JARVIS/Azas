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
        self.declare_parameter("publish_rate", 60.0)
        self.declare_parameter("shake_cycles_per_second", 0.55)
        self.declare_parameter("preview_mode", "side_grasp_move_then_shake")
        self.declare_parameter("loop_motion", True)
        self.declare_parameter(
            "home_joints_rad",
            [0.0, math.radians(-35.0), math.radians(50.0), 0.0, math.radians(70.0), 0.0],
        )

        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.start_time = self.get_clock().now()
        rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self.publish_joint_state)
        self.get_logger().info(
            "Publishing smooth RViz M0609 robot motion from /joint_states; no path display."
        )

    def publish_joint_state(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        home = [float(value) for value in self.get_parameter("home_joints_rad").value]
        while len(home) < 6:
            home.append(0.0)

        mode = str(self.get_parameter("preview_mode").value).strip().lower()
        if mode in {"color_scan_pose_move", "color_scan_move", "camera_view_move"}:
            positions = self.color_scan_pose_move_joints(elapsed, home)
        elif mode in {"static_pose", "color_scan_pose", "color_scan", "camera_view_pose"}:
            positions = home[:6]
        elif mode in {"cup_target_move", "side_grasp_target_move", "target_move"}:
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
        # Deliberately slow/small: this is for readable RViz robot motion, not
        # a high-frequency fake shake. The cup stays generally upright while
        # the wrist shows a gentle mixing motion.
        wrist_roll = math.sin(phase)
        wrist_pitch = math.sin(phase + math.pi * 0.5)
        wrist_yaw = math.sin(phase * 0.5)

        return [
            home[0],
            home[1],
            home[2],
            home[3] + math.radians(7.0) * wrist_roll,
            home[4] + math.radians(10.0) * wrist_pitch,
            home[5] + math.radians(12.0) * wrist_yaw,
        ]

    def color_scan_pose_move_joints(self, elapsed: float, target: list[float]) -> list[float]:
        start = [
            0.0,
            0.0,
            math.radians(90.0),
            0.0,
            math.radians(90.0),
            0.0,
        ]
        cycle_seconds = 10.0
        t = elapsed % cycle_seconds
        if t < 4.0:
            ratio = self.minimum_jerk(t / 4.0)
            return [a + (b - a) * ratio for a, b in zip(start, target[:6])]
        if t < 7.0:
            return target[:6]
        ratio = self.minimum_jerk((t - 7.0) / 3.0)
        return [a + (b - a) * ratio for a, b in zip(target[:6], start)]

    def side_grasp_move_then_shake_joints(self, elapsed: float, home: list[float]) -> list[float]:
        # Joint-space storyboard for RViz visibility only.  It follows the
        # dispenser task shape without publishing Path/markers as the primary
        # visual: approach -> side grasp -> lift -> dispenser hold -> retreat
        # -> gentle wrist shake.  Every segment uses minimum-jerk interpolation
        # so the robot model moves smoothly instead of snapping.
        keyframes = [
            (0.0, home),
            (3.0, [0.16, -0.70, 1.36, -0.16, 1.17, 0.12]),  # side pre-grasp
            (5.8, [0.24, -0.76, 1.46, -0.08, 1.13, 0.08]),  # side grasp
            (8.8, [0.28, -0.58, 1.25, 0.02, 1.08, 0.07]),  # lift with cup
            (12.8, [-0.08, -0.52, 1.18, 0.10, 1.07, -0.05]),  # carry to dispenser
            (15.8, [-0.18, -0.62, 1.32, 0.06, 1.18, 0.00]),  # outlet front hold
            (18.8, [-0.06, -0.48, 1.12, 0.00, 1.08, 0.00]),  # retreat before shake
        ]
        cycle_seconds = 30.0
        if bool(self.get_parameter("loop_motion").value):
            t = elapsed % cycle_seconds
        else:
            t = min(elapsed, cycle_seconds)
        if t >= keyframes[-1][0]:
            shake_elapsed = t - keyframes[-1][0]
            shake_home = keyframes[-1][1]
            return self.high_shake_joints(shake_elapsed, shake_home)

        for index in range(len(keyframes) - 1):
            start_t, start_joints = keyframes[index]
            end_t, end_joints = keyframes[index + 1]
            if start_t <= t < end_t:
                ratio = (t - start_t) / max(end_t - start_t, 1e-6)
                smooth = self.minimum_jerk(ratio)
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
                smooth = self.minimum_jerk(ratio)
                return [
                    start + (end - start) * smooth
                    for start, end in zip(start_joints, end_joints)
                ]
        return home

    @staticmethod
    def minimum_jerk(ratio: float) -> float:
        ratio = max(0.0, min(1.0, ratio))
        return ratio * ratio * ratio * (10.0 - 15.0 * ratio + 6.0 * ratio * ratio)


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
