#!/usr/bin/env python3
"""RViz-only helpers for the tumbler shake dry-run scene.

This node does not command MoveIt, Doosan, or RG2 hardware. It only publishes
visualization markers so the dry-run scene shows a gripper and visible shake
motion while the real motion gates remain closed.
"""

from __future__ import annotations

import math
from typing import Iterable

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class ShakeVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("shake_visualizer_node")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("shake_center_x", 0.28)
        self.declare_parameter("shake_center_y", -0.30)
        self.declare_parameter("shake_center_z", 0.62)
        self.declare_parameter("shake_amplitude_x", 0.050)
        self.declare_parameter("shake_amplitude_y", 0.025)
        self.declare_parameter("shake_amplitude_z", 0.035)
        self.declare_parameter("segment_seconds", 0.16)
        self.declare_parameter("publish_demo_arm", False)

        self.robot_arm_pub = self.create_publisher(
            MarkerArray, "/jarvis/robot_arm/markers", 10
        )
        self.robot_gripper_pub = self.create_publisher(
            MarkerArray, "/jarvis/robot_gripper/markers", 10
        )
        self.shake_pub = self.create_publisher(
            MarkerArray, "/jarvis/shake_animation/markers", 10
        )
        self.start_time = self.get_clock().now()
        self.path_points = self._make_path_points()
        self.timer = self.create_timer(0.05, self.publish_markers)
        self.get_logger().info(
            "Publishing RViz-only RG2 and shake animation markers; no hardware commands are sent."
        )

    def _param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _param_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _make_path_points(self) -> list[tuple[float, float, float]]:
        cx = self._param_float("shake_center_x")
        cy = self._param_float("shake_center_y")
        cz = self._param_float("shake_center_z")
        ax = self._param_float("shake_amplitude_x")
        ay = self._param_float("shake_amplitude_y")
        az = self._param_float("shake_amplitude_z")
        return [
            (cx, cy, cz),
            (cx + ax, cy, cz),
            (cx, cy, cz),
            (cx - ax, cy, cz),
            (cx, cy, cz),
            (cx + ax * 0.65, cy + ay * 0.65, cz),
            (cx, cy, cz),
            (cx - ax * 0.65, cy - ay * 0.65, cz),
            (cx, cy, cz),
            (cx, cy, cz + az),
            (cx, cy, cz),
        ]

    def publish_markers(self) -> None:
        now = self.get_clock().now()
        pose = self._animated_pose(now)
        if bool(self.get_parameter("publish_demo_arm").value):
            self.robot_arm_pub.publish(self._robot_arm_markers(now, pose))
        self.robot_gripper_pub.publish(self._robot_gripper_markers(now, pose))
        self.shake_pub.publish(self._shake_markers(now, pose))

    def _animated_pose(self, now) -> tuple[float, float, float, float]:
        segment_seconds = max(0.05, self._param_float("segment_seconds"))
        elapsed = (now - self.start_time).nanoseconds / 1e9
        segment_index = int(elapsed / segment_seconds) % len(self.path_points)
        next_index = (segment_index + 1) % len(self.path_points)
        t = (elapsed % segment_seconds) / segment_seconds
        smooth_t = 0.5 - 0.5 * math.cos(math.pi * t)
        start = self.path_points[segment_index]
        end = self.path_points[next_index]
        x = start[0] + (end[0] - start[0]) * smooth_t
        y = start[1] + (end[1] - start[1]) * smooth_t
        z = start[2] + (end[2] - start[2]) * smooth_t
        yaw = math.sin(elapsed * math.tau * 2.8) * 0.14
        return x, y, z, yaw

    def _robot_arm_markers(
        self, now, pose: tuple[float, float, float, float]
    ) -> MarkerArray:
        x, y, z, yaw = pose
        frame = self._param_str("base_frame")
        wrist = (x - 0.105, y, z - 0.020)
        joints = [
            (0.0, 0.0, 0.055),
            (0.0, 0.0, 0.220),
            (0.120, y * 0.28, 0.470),
            (wrist[0] - 0.085, y * 0.82, z - 0.005),
            wrist,
        ]
        markers = [
            self._line_strip(
                now,
                frame,
                100,
                "m0609_demo_arm_links",
                joints,
                self._color(0.12, 0.36, 0.95, 0.92),
                width=0.050,
                close=False,
            )
        ]
        for index, joint in enumerate(joints):
            radius = 0.070 if index == 0 else 0.045
            markers.append(
                self._sphere(
                    now,
                    frame,
                    110 + index,
                    "m0609_demo_arm_joints",
                    joint,
                    (radius, radius, radius),
                    self._color(0.06, 0.18, 0.42, 0.95),
                )
            )
        markers.append(
            self._cube(
                now,
                frame,
                120,
                "m0609_demo_wrist",
                wrist,
                (0.070, 0.055, 0.055),
                self._color(0.10, 0.12, 0.16, 0.95),
                yaw=yaw,
            )
        )
        return MarkerArray(markers=markers)

    def _robot_gripper_markers(
        self, now, pose: tuple[float, float, float, float]
    ) -> MarkerArray:
        x, y, z, yaw = pose
        frame = self._param_str("base_frame")
        markers = [
            self._cube(
                now,
                frame,
                0,
                "rg2_attached_palm",
                (x - 0.056, y, z - 0.035),
                (0.070, 0.045, 0.025),
                self._color(0.18, 0.19, 0.20, 0.85),
                yaw=yaw,
            ),
            self._cube(
                now,
                frame,
                1,
                "rg2_attached_finger",
                (x - 0.006, y + 0.041, z - 0.052),
                (0.075, 0.010, 0.060),
                self._color(0.08, 0.08, 0.09, 0.85),
                yaw=yaw,
            ),
            self._cube(
                now,
                frame,
                2,
                "rg2_attached_finger",
                (x - 0.006, y - 0.041, z - 0.052),
                (0.075, 0.010, 0.060),
                self._color(0.08, 0.08, 0.09, 0.85),
                yaw=yaw,
            ),
        ]
        return MarkerArray(markers=markers)

    def _shake_markers(
        self, now, pose: tuple[float, float, float, float]
    ) -> MarkerArray:
        x, y, z, yaw = pose
        frame = self._param_str("base_frame")
        markers = [
            self._cylinder(
                now,
                frame,
                10,
                "shake_tumbler_body",
                (x, y, z - 0.085),
                (0.072, 0.072, 0.170),
                self._color(0.88, 0.89, 0.86, 0.88),
                yaw=yaw,
            ),
            self._cylinder(
                now,
                frame,
                11,
                "shake_tumbler_lid",
                (x, y, z + 0.010),
                (0.080, 0.080, 0.020),
                self._color(0.16, 0.19, 0.22, 0.95),
                yaw=yaw,
            ),
            self._line_strip(
                now,
                frame,
                15,
                "shake_motion_loop",
                self.path_points,
                self._color(1.0, 0.56, 0.12, 0.95),
                width=0.018,
            ),
        ]
        return MarkerArray(markers=markers)

    def _marker(
        self,
        now,
        frame: str,
        marker_id: int,
        namespace: str,
        marker_type: int,
        color: ColorRGBA,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = now.to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.color = color
        marker.pose.orientation.w = 1.0
        return marker

    def _cube(
        self,
        now,
        frame: str,
        marker_id: int,
        namespace: str,
        position: tuple[float, float, float],
        scale: tuple[float, float, float],
        color: ColorRGBA,
        *,
        yaw: float = 0.0,
        frame_locked: bool = False,
    ) -> Marker:
        marker = self._marker(now, frame, marker_id, namespace, Marker.CUBE, color)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.frame_locked = frame_locked
        return marker

    def _cylinder(
        self,
        now,
        frame: str,
        marker_id: int,
        namespace: str,
        position: tuple[float, float, float],
        scale: tuple[float, float, float],
        color: ColorRGBA,
        *,
        yaw: float = 0.0,
    ) -> Marker:
        marker = self._marker(now, frame, marker_id, namespace, Marker.CYLINDER, color)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        return marker

    def _sphere(
        self,
        now,
        frame: str,
        marker_id: int,
        namespace: str,
        position: tuple[float, float, float],
        scale: tuple[float, float, float],
        color: ColorRGBA,
    ) -> Marker:
        marker = self._marker(now, frame, marker_id, namespace, Marker.SPHERE, color)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        return marker

    def _line_strip(
        self,
        now,
        frame: str,
        marker_id: int,
        namespace: str,
        points: Iterable[tuple[float, float, float]],
        color: ColorRGBA,
        *,
        width: float = 0.018,
        close: bool = True,
    ) -> Marker:
        marker = self._marker(now, frame, marker_id, namespace, Marker.LINE_STRIP, color)
        marker.scale.x = width
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
        if close and marker.points:
            marker.points.append(marker.points[0])
        return marker

    @staticmethod
    def _color(r: float, g: float, b: float, a: float) -> ColorRGBA:
        return ColorRGBA(r=r, g=g, b=b, a=a)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ShakeVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
