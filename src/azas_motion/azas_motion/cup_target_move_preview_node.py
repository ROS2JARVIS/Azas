#!/usr/bin/env python3
"""RViz preview: side-grasp a cup and move it to a target coordinate.

This node is intentionally narrow. It does not know about dispensers,
shaking, Gazebo, MoveIt execution, Doosan services, or RG2 services. It only
publishes a low side-grasp transfer path and visible RViz markers.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


XYZ = tuple[float, float, float]
RGBA = tuple[float, float, float, float]


@dataclass(frozen=True)
class Step:
    label: str
    xyz: XYZ


def point(xyz: XYZ) -> Point:
    return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


def pose(xyz: XYZ) -> Pose:
    msg = Pose()
    msg.position = point(xyz)
    msg.orientation.w = 1.0
    return msg


def side_grasp_pose(xyz: XYZ, tool_z_xy: tuple[float, float]) -> Pose:
    msg = pose(xyz)
    qx, qy, qz, qw = side_grasp_quaternion(tool_z_xy)
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw
    return msg


def side_grasp_quaternion(tool_z_xy: tuple[float, float]) -> tuple[float, float, float, float]:
    zx, zy = normalize_xy(tool_z_xy[0], tool_z_xy[1])
    tool_z = (zx, zy, 0.0)
    tool_x = (-zy, zx, 0.0)
    tool_y = (0.0, 0.0, 1.0)
    matrix = (
        (tool_x[0], tool_y[0], tool_z[0]),
        (tool_x[1], tool_y[1], tool_z[1]),
        (tool_x[2], tool_y[2], tool_z[2]),
    )
    return quaternion_from_matrix(matrix)


def quaternion_from_matrix(matrix: tuple[tuple[float, float, float], ...]) -> tuple[float, float, float, float]:
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (m21 - m12) / scale
        qy = (m02 - m20) / scale
        qz = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / scale
        qx = 0.25 * scale
        qy = (m01 + m10) / scale
        qz = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / scale
        qx = (m01 + m10) / scale
        qy = 0.25 * scale
        qz = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / scale
        qx = (m02 + m20) / scale
        qy = (m12 + m21) / scale
        qz = 0.25 * scale
    return qx, qy, qz, qw


def normalize_xy(x: float, y: float) -> tuple[float, float]:
    length = math.hypot(x, y)
    if length <= 1e-9:
        return 1.0, 0.0
    return x / length, y / length


class CupTargetMovePreviewNode(Node):
    def __init__(self) -> None:
        super().__init__("cup_target_move_preview_node")
        self.declare_parameter("cup_pose_topic", "/azas/demo/tumbler_pose")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("target_x", 0.43)
        self.declare_parameter("target_y", 0.08)
        self.declare_parameter("target_z", 0.175)
        self.declare_parameter("grasp_height_m", 0.085)
        self.declare_parameter("lift_height_m", 0.040)
        self.declare_parameter("side_pre_grasp_offset_m", 0.100)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("animation_hold_ticks", 30)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.plan_pub = self.create_publisher(Path, "/azas/cup_target_move/plan", qos)
        self.status_marker_pub = self.create_publisher(
            MarkerArray, "/azas/cup_target_move/markers", 10
        )
        self.robot_arm_pub = self.create_publisher(
            MarkerArray, "/jarvis/robot_arm/markers", 10
        )
        self.gripper_pub = self.create_publisher(
            MarkerArray, "/jarvis/robot_gripper/markers", 10
        )
        self.target_pub = self.create_publisher(
            PoseStamped, "/azas/cup_target_move/target_pose", qos
        )

        self.last_cup_pose: PoseStamped | None = None
        self.step_index = 0
        self.step_tick = 0
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("cup_pose_topic").value),
            self.on_cup_pose,
            10,
        )
        period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 0.5)
        self.create_timer(period, self.publish_preview)
        self.get_logger().info(
            "cup_target_move_preview_node ready: side grasp -> target coordinate only"
        )

    def on_cup_pose(self, msg: PoseStamped) -> None:
        frame_id = str(self.get_parameter("frame_id").value)
        if msg.header.frame_id != frame_id:
            self.get_logger().error(
                f"Rejected cup pose frame={msg.header.frame_id!r}; expected {frame_id!r}"
            )
            return
        if self.last_cup_pose is None or self.pose_changed(self.last_cup_pose, msg):
            self.step_index = 0
            self.step_tick = 0
        self.last_cup_pose = msg

    @staticmethod
    def pose_changed(previous: PoseStamped, current: PoseStamped) -> bool:
        prev = previous.pose.position
        cur = current.pose.position
        delta = math.sqrt(
            (prev.x - cur.x) ** 2 + (prev.y - cur.y) ** 2 + (prev.z - cur.z) ** 2
        )
        return delta > 0.005

    def build_steps(self, cup_pose: PoseStamped) -> list[Step]:
        cup = cup_pose.pose.position
        cup_base = (float(cup.x), float(cup.y), float(cup.z))
        grasp_height = float(self.get_parameter("grasp_height_m").value)
        lift_height = float(self.get_parameter("lift_height_m").value)
        offset = float(self.get_parameter("side_pre_grasp_offset_m").value)
        target = (
            float(self.get_parameter("target_x").value),
            float(self.get_parameter("target_y").value),
            float(self.get_parameter("target_z").value),
        )

        grasp = (cup_base[0], cup_base[1], cup_base[2] + grasp_height)
        dx, dy = normalize_xy(grasp[0], grasp[1])
        side_pre_grasp = (grasp[0] - dx * offset, grasp[1] - dy * offset, grasp[2])
        lift = (grasp[0], grasp[1], grasp[2] + lift_height)
        target_hold = target
        self.tool_z_xy = (grasp[0] - side_pre_grasp[0], grasp[1] - side_pre_grasp[1])
        return [
            Step("side_pre_grasp", side_pre_grasp),
            Step("side_grasp_tumbler", grasp),
            Step("lift_tumbler", lift),
            Step("target_hold", target_hold),
        ]

    def publish_preview(self) -> None:
        if self.last_cup_pose is None:
            return
        steps = self.build_steps(self.last_cup_pose)
        frame_id = str(self.get_parameter("frame_id").value)
        stamp = self.get_clock().now().to_msg()

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = frame_id
        for step in steps:
            stamped = PoseStamped()
            stamped.header = path.header
            stamped.pose = side_grasp_pose(step.xyz, self.tool_z_xy)
            path.poses.append(stamped)
        self.plan_pub.publish(path)
        self.target_pub.publish(path.poses[-1])

        active = self.active_step(steps)
        scene_markers = self.make_scene_markers(steps, active, stamp, frame_id)
        robot_markers = self.make_robot_markers(active, stamp, frame_id)
        gripper_markers = self.make_gripper_markers(active, stamp, frame_id)
        self.status_marker_pub.publish(MarkerArray(markers=scene_markers + robot_markers + gripper_markers))
        self.robot_arm_pub.publish(MarkerArray(markers=robot_markers))
        self.gripper_pub.publish(MarkerArray(markers=gripper_markers))
        self.advance(len(steps))

    def active_step(self, steps: Sequence[Step]) -> Step:
        if not steps:
            return Step("idle", (0.0, 0.0, 0.0))
        start = steps[self.step_index % len(steps)]
        end = steps[(self.step_index + 1) % len(steps)]
        hold_ticks = max(int(self.get_parameter("animation_hold_ticks").value), 1)
        ratio = min(max(self.step_tick / float(hold_ticks), 0.0), 1.0)
        xyz = (
            start.xyz[0] + (end.xyz[0] - start.xyz[0]) * ratio,
            start.xyz[1] + (end.xyz[1] - start.xyz[1]) * ratio,
            start.xyz[2] + (end.xyz[2] - start.xyz[2]) * ratio,
        )
        return Step(end.label, xyz)

    def advance(self, step_count: int) -> None:
        hold_ticks = max(int(self.get_parameter("animation_hold_ticks").value), 1)
        self.step_tick += 1
        if self.step_tick >= hold_ticks:
            self.step_tick = 0
            self.step_index = (self.step_index + 1) % max(step_count, 1)

    def marker(
        self,
        stamp,
        frame_id: str,
        marker_id: int,
        marker_type: int,
        ns: str,
        xyz: XYZ,
        scale: XYZ,
        color: RGBA,
    ) -> Marker:
        msg = Marker()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.ns = ns
        msg.id = marker_id
        msg.type = marker_type
        msg.action = Marker.ADD
        msg.pose = pose(xyz)
        msg.scale = Vector3(x=scale[0], y=scale[1], z=scale[2])
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = color
        return msg

    def make_scene_markers(
        self, steps: Sequence[Step], active: Step, stamp, frame_id: str
    ) -> list[Marker]:
        markers: list[Marker] = []
        line = self.marker(
            stamp, frame_id, 1, Marker.LINE_STRIP, "cup_target_path",
            (0.0, 0.0, 0.0), (0.030, 0.0, 0.0), (1.0, 0.05, 0.9, 1.0)
        )
        line.points = [point(step.xyz) for step in steps]
        markers.append(line)
        for index, step in enumerate(steps):
            markers.append(
                self.marker(
                    stamp, frame_id, 10 + index, Marker.SPHERE, "cup_target_waypoint",
                    step.xyz, (0.050, 0.050, 0.050), (1.0, 0.05, 0.9, 1.0)
                )
            )
            label = self.marker(
                stamp, frame_id, 30 + index, Marker.TEXT_VIEW_FACING, "cup_target_label",
                (step.xyz[0], step.xyz[1], step.xyz[2] + 0.045),
                (0.0, 0.0, 0.032), (1.0, 1.0, 1.0, 1.0)
            )
            label.text = step.label
            markers.append(label)

        cup_base = (active.xyz[0], active.xyz[1], active.xyz[2] - 0.085)
        markers.append(
            self.marker(
                stamp, frame_id, 80, Marker.CYLINDER, "held_cup_body",
                (cup_base[0], cup_base[1], cup_base[2] + 0.085),
                (0.090, 0.090, 0.190), (0.1, 0.95, 1.0, 0.92)
            )
        )
        return markers

    def make_robot_markers(self, active: Step, stamp, frame_id: str) -> list[Marker]:
        x, y, z = active.xyz
        wrist = (x - 0.090, y, max(z, 0.10))
        joints = [
            (0.0, 0.0, 0.060),
            (0.0, 0.0, 0.260),
            (max(0.10, wrist[0] * 0.42), wrist[1] * 0.25, max(0.34, wrist[2] + 0.23)),
            (max(0.14, wrist[0] * 0.74), wrist[1] * 0.68, max(0.24, wrist[2] + 0.10)),
            wrist,
        ]
        base = self.marker(
            stamp, frame_id, 99, Marker.CYLINDER, "simple_target_move_robot_base",
            (0.0, 0.0, 0.030), (0.180, 0.180, 0.060), (0.18, 0.20, 0.24, 1.0)
        )
        arm = self.marker(
            stamp, frame_id, 100, Marker.LINE_STRIP, "simple_target_move_robot",
            (0.0, 0.0, 0.0), (0.060, 0.0, 0.0), (1.0, 0.42, 0.05, 1.0)
        )
        arm.points = [point(joint) for joint in joints]
        markers = [base, arm]
        for index, joint in enumerate(joints):
            markers.append(
                self.marker(
                    stamp, frame_id, 110 + index, Marker.SPHERE, "simple_target_move_joints",
                    joint, (0.075, 0.075, 0.075), (0.95, 0.22, 0.05, 1.0)
                )
            )
        return markers

    def make_gripper_markers(self, active: Step, stamp, frame_id: str) -> list[Marker]:
        x, y, z = active.xyz
        return [
            self.marker(
                stamp, frame_id, 200, Marker.CUBE, "simple_target_move_rg2",
                (x - 0.045, y, z - 0.012), (0.100, 0.060, 0.035), (0.05, 0.05, 0.06, 1.0)
            ),
            self.marker(
                stamp, frame_id, 201, Marker.CUBE, "simple_target_move_rg2",
                (x + 0.018, y + 0.050, z - 0.030), (0.095, 0.014, 0.070), (1.0, 0.9, 0.05, 1.0)
            ),
            self.marker(
                stamp, frame_id, 202, Marker.CUBE, "simple_target_move_rg2",
                (x + 0.018, y - 0.050, z - 0.030), (0.095, 0.014, 0.070), (1.0, 0.9, 0.05, 1.0)
            ),
        ]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CupTargetMovePreviewNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
