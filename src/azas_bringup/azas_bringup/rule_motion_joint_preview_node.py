#!/usr/bin/env python3
"""RViz-only joint-state preview driven by the rule-motion Path."""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RuleMotionJointPreviewNode(Node):
    def __init__(self) -> None:
        super().__init__("rule_motion_joint_preview_node")
        self.declare_parameter("path_topic", "/azas/dispenser_sequence/plan")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("frames_per_pose", 12)
        self.declare_parameter("loop_preview", True)

        self.path: Path | None = None
        self.segment_index = 0
        self.frame_index = 0
        self.publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            10,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("path_topic").value),
            self._on_path,
            10,
        )
        period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.create_timer(period, self._publish)
        self.get_logger().info(
            "RViz-only rule motion joint preview publishing /joint_states from dispenser Path"
        )

    def _on_path(self, msg: Path) -> None:
        if len(msg.poses) < 2:
            return
        if self.path is None or len(self.path.poses) != len(msg.poses):
            self.segment_index = 0
            self.frame_index = 0
            self.get_logger().info(f"Received rule motion Path with {len(msg.poses)} poses")
        self.path = msg

    def _publish(self) -> None:
        if self.path is None or len(self.path.poses) < 2:
            return
        frames_per_pose = max(int(self.get_parameter("frames_per_pose").value), 1)
        pose_count = len(self.path.poses)
        start = self.path.poses[self.segment_index].pose.position
        end = self.path.poses[min(self.segment_index + 1, pose_count - 1)].pose.position
        ratio = self.frame_index / frames_per_pose
        ratio = 0.5 - 0.5 * math.cos(math.pi * ratio)
        x = start.x + (end.x - start.x) * ratio
        y = start.y + (end.y - start.y) * ratio
        z = start.z + (end.z - start.z) * ratio
        self._publish_joint_state(x, y, z)

        self.frame_index += 1
        if self.frame_index > frames_per_pose:
            self.frame_index = 0
            self.segment_index += 1
            if self.segment_index >= pose_count - 1:
                if bool(self.get_parameter("loop_preview").value):
                    self.segment_index = 0
                else:
                    self.segment_index = pose_count - 2

    def _publish_joint_state(self, x: float, y: float, z: float) -> None:
        radius = max(math.hypot(x, y), 0.1)
        base = math.atan2(y, x)
        shoulder = -0.65 + (0.42 - min(max(z, 0.08), 0.70)) * 1.15
        elbow = 1.35 - min(max(radius - 0.20, 0.0), 0.70) * 1.30
        wrist1 = -0.65 + min(max(z - 0.18, 0.0), 0.50) * 1.25
        wrist2 = 1.10 + 0.25 * math.sin(self.segment_index * 0.7)
        wrist3 = -base * 0.45 + 0.2 * math.sin(self.frame_index * 0.25)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        msg.position = [base, shoulder, elbow, wrist1, wrist2, wrist3]
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RuleMotionJointPreviewNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
