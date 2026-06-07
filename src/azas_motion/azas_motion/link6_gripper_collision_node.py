#!/usr/bin/env python3
"""Publish a conservative RG2-style gripper envelope attached to link_6.

The Doosan model in this project often exposes only the robot flange, so
planning previews can behave as if no gripper occupies space.  This node adds a
fixed, link_6-relative collision envelope matching the supplemental
``rg2_link6_tcp.urdf.xacro`` preview.  It does not create robot poses or
calibration values; all geometry is local to link_6.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def transient_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def make_pose(xyz: tuple[float, float, float]) -> Pose:
    pose = Pose()
    pose.position.x = xyz[0]
    pose.position.y = xyz[1]
    pose.position.z = xyz[2]
    pose.orientation.w = 1.0
    return pose


def box(size_xyz: tuple[float, float, float]) -> SolidPrimitive:
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(value) for value in size_xyz]
    return primitive


def cylinder_z(height_m: float, radius_m: float) -> SolidPrimitive:
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.CYLINDER
    primitive.dimensions = [float(height_m), float(radius_m)]
    return primitive


class Link6GripperCollisionNode(Node):
    def __init__(self) -> None:
        super().__init__("link6_gripper_collision_node")
        self.declare_parameter("object_id", "azas_rg2_gripper_on_link6")
        self.declare_parameter("attached_link_name", "link_6")
        self.declare_parameter(
            "touch_links",
            [
                "link_6",
                "GripperDA_v1_jarvis",
                "rg2_left_finger_visual",
                "rg2_right_finger_visual",
                "rg2_open_tcp",
                "rg2_closed_tcp",
                "dispenser_press_tcp",
            ],
        )
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("publish_once", False)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_topic", "/azas/link6_gripper/markers")

        self.publisher = self.create_publisher(
            AttachedCollisionObject,
            "/attached_collision_object",
            transient_qos(),
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            transient_qos(),
        )
        self._logged = False
        self._publish()

        period = float(self.get_parameter("publish_period_sec").value)
        if not bool(self.get_parameter("publish_once").value):
            self.timer = self.create_timer(max(period, 0.2), self._publish)

    def _attached_object(self) -> AttachedCollisionObject:
        link_name = str(self.get_parameter("attached_link_name").value)
        attached = AttachedCollisionObject()
        attached.link_name = link_name
        attached.touch_links = [str(item) for item in self.get_parameter("touch_links").value]

        obj = CollisionObject()
        obj.id = str(self.get_parameter("object_id").value)
        obj.header = Header()
        obj.header.frame_id = link_name
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.operation = CollisionObject.ADD

        # Same envelope as rg2_link6_tcp.urdf.xacro:
        # flange/mount cylinder, palm, two long fingers, and inward blue pads.
        obj.primitives.extend(
            [
                cylinder_z(0.050, 0.040),
                box((0.090, 0.140, 0.050)),
                box((0.035, 0.018, 0.160)),
                box((0.035, 0.018, 0.160)),
                box((0.025, 0.012, 0.035)),
                box((0.025, 0.012, 0.035)),
            ]
        )
        obj.primitive_poses.extend(
            [
                make_pose((0.0, 0.0, 0.025)),
                make_pose((0.0, 0.0, 0.075)),
                make_pose((0.0, 0.055, 0.155)),
                make_pose((0.0, -0.055, 0.155)),
                make_pose((0.0, 0.040, 0.245)),
                make_pose((0.0, -0.040, 0.245)),
            ]
        )
        attached.object = obj
        return attached

    def _marker_array(self) -> MarkerArray:
        link_name = str(self.get_parameter("attached_link_name").value)
        stamp = self.get_clock().now().to_msg()
        specs = [
            ("mount", Marker.CYLINDER, (0.0, 0.0, 0.025), (0.080, 0.080, 0.050), (0.42, 0.43, 0.45, 0.95)),
            ("palm", Marker.CUBE, (0.0, 0.0, 0.075), (0.090, 0.140, 0.050), (0.08, 0.08, 0.09, 0.95)),
            ("left_finger", Marker.CUBE, (0.0, 0.055, 0.155), (0.035, 0.018, 0.160), (0.08, 0.08, 0.09, 0.95)),
            ("right_finger", Marker.CUBE, (0.0, -0.055, 0.155), (0.035, 0.018, 0.160), (0.08, 0.08, 0.09, 0.95)),
            ("left_pad", Marker.CUBE, (0.0, 0.040, 0.245), (0.025, 0.012, 0.035), (0.05, 0.35, 0.95, 0.95)),
            ("right_pad", Marker.CUBE, (0.0, -0.040, 0.245), (0.025, 0.012, 0.035), (0.05, 0.35, 0.95, 0.95)),
        ]
        markers: list[Marker] = []
        for index, (name, marker_type, xyz, scale, rgba) in enumerate(specs):
            marker = Marker()
            marker.header.frame_id = link_name
            marker.header.stamp = stamp
            marker.ns = "azas_link6_rg2_gripper"
            marker.id = index
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.pose = make_pose(xyz)
            marker.scale.x = scale[0]
            marker.scale.y = scale[1]
            marker.scale.z = scale[2]
            marker.color.r = rgba[0]
            marker.color.g = rgba[1]
            marker.color.b = rgba[2]
            marker.color.a = rgba[3]
            marker.text = name
            markers.append(marker)
        return MarkerArray(markers=markers)

    def _publish(self) -> None:
        self.publisher.publish(self._attached_object())
        if bool(self.get_parameter("publish_markers").value):
            self.marker_publisher.publish(self._marker_array())
        if not self._logged:
            self.get_logger().info(
                "Publishing RG2-style attached collision envelope and markers on link_6"
            )
            self._logged = True


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Link6GripperCollisionNode()
    try:
        if bool(node.get_parameter("publish_once").value):
            import time

            time.sleep(0.35)
            return
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
