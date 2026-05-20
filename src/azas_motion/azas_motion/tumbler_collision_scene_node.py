#!/usr/bin/env python3
"""Publish measured tumbler occupancy to the MoveIt planning scene.

This node does not invent cup coordinates.  It uses only existing project
contracts:

* live detected cup pose: /jarvis/tumbler_dispenser/tumbler_pose
* measured dispenser front-hold references: measured_dispenser_collision.yaml
* measured cup-holder center: calibration.yaml
* tumbler dimensions from docs/tumbler_dispenser_models.md

The node can run continuously for the live detected tumbler pose, or one-shot to
add/remove a world object at a known station or attach/detach the tumbler to the
current gripper link.  The current Doosan direct move services do not consume
MoveIt collision objects; this is the scene-state foundation for planned motion
and fail-close checks.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

ROOT = Path("/home/ssu/Azas")
DEFAULT_DISPENSER_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "measured_dispenser_collision.yaml"
DEFAULT_CALIBRATION_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"

# Source: docs/tumbler_dispenser_models.md
TUMBLER_DIAMETER_M = 0.075
TUMBLER_RADIUS_M = TUMBLER_DIAMETER_M / 2.0
TUMBLER_LIDDED_HEIGHT_M = 0.170
TUMBLER_LIDLESS_HEIGHT_M = 0.140


def transient_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"YAML config not found: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"YAML config is not a map: {path}")
    return data


def pose_from_xyz(xyz: list[float], *, z_center_offset_m: float = 0.0) -> Pose:
    pose = Pose()
    pose.position.x = float(xyz[0])
    pose.position.y = float(xyz[1])
    pose.position.z = float(xyz[2]) + z_center_offset_m
    pose.orientation.w = 1.0
    return pose


class TumblerCollisionSceneNode(Node):
    def __init__(self) -> None:
        super().__init__("tumbler_collision_scene_node")

        self.declare_parameter("action", "publish_detected")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("object_id", "azas_tumbler")
        self.declare_parameter("tumbler_pose_topic", "/jarvis/tumbler_dispenser/tumbler_pose")
        self.declare_parameter("dispenser_config_path", str(DEFAULT_DISPENSER_CONFIG))
        self.declare_parameter("calibration_config_path", str(DEFAULT_CALIBRATION_CONFIG))
        self.declare_parameter("dispenser_id", 1)
        self.declare_parameter("attached_link_name", "GripperDA_v1_jarvis")
        self.declare_parameter("touch_links", ["GripperDA_v1_jarvis", "link_6"])
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("publish_once", False)
        self.declare_parameter("use_lidded_height", True)
        self.declare_parameter("radius_margin_m", 0.006)
        self.declare_parameter("height_margin_m", 0.010)
        # Conservative station estimate: front_hold pose is a measured link_6
        # placement reference, not an arbitrary LLM coordinate.  It is used only
        # as an occupancy seed until a live detected/base_link pose or attached
        # object replaces it.
        self.declare_parameter("station_z_is_bottom", True)

        self.action = str(self.get_parameter("action").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.object_id = str(self.get_parameter("object_id").value)
        self.dispenser_config = load_yaml(Path(str(self.get_parameter("dispenser_config_path").value)))
        self.calibration_config = load_yaml(Path(str(self.get_parameter("calibration_config_path").value)))
        self.last_detected_pose: PoseStamped | None = None

        self.collision_pub = self.create_publisher(CollisionObject, "/collision_object", transient_qos())
        self.attached_pub = self.create_publisher(
            AttachedCollisionObject, "/attached_collision_object", transient_qos()
        )

        if self.action == "publish_detected":
            topic = str(self.get_parameter("tumbler_pose_topic").value)
            self.create_subscription(PoseStamped, topic, self._on_detected_pose, 10)
            self.get_logger().info(f"Publishing detected tumbler collision from {topic}")
        elif self.action in {
            "add_dispenser",
            "add_holder",
            "remove_world",
            "attach",
            "detach",
        }:
            self._publish_action_once()
        else:
            raise ValueError(
                "action must be one of publish_detected, add_dispenser, add_holder, "
                "remove_world, attach, detach"
            )

        period = float(self.get_parameter("publish_period_sec").value)
        if self.action == "publish_detected":
            self.timer = self.create_timer(max(period, 0.2), self._publish_detected_if_available)

    def tumbler_height(self) -> float:
        height = TUMBLER_LIDDED_HEIGHT_M if bool(self.get_parameter("use_lidded_height").value) else TUMBLER_LIDLESS_HEIGHT_M
        return height + float(self.get_parameter("height_margin_m").value)

    def tumbler_radius(self) -> float:
        return TUMBLER_RADIUS_M + float(self.get_parameter("radius_margin_m").value)

    def _primitive(self) -> SolidPrimitive:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [self.tumbler_height(), self.tumbler_radius()]
        return primitive

    def _collision_object(self, object_id: str, pose: Pose, operation: int) -> CollisionObject:
        msg = CollisionObject()
        msg.id = object_id
        msg.header = Header()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.operation = operation
        if operation != CollisionObject.REMOVE:
            msg.primitives.append(self._primitive())
            msg.primitive_poses.append(pose)
        return msg

    def _on_detected_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != self.frame_id:
            self.get_logger().warning(
                f"Ignoring tumbler pose frame_id={msg.header.frame_id}; expected {self.frame_id}"
            )
            return
        self.last_detected_pose = msg

    def _publish_detected_if_available(self) -> None:
        if self.last_detected_pose is None:
            return
        pose = Pose()
        pose.position = self.last_detected_pose.pose.position
        pose.position.z += self.tumbler_height() * 0.5
        pose.orientation.w = 1.0
        self.collision_pub.publish(self._collision_object(self.object_id, pose, CollisionObject.ADD))

    def dispenser_id_text(self) -> str:
        value = self.get_parameter("dispenser_id").value
        return str(int(value)) if isinstance(value, int) else str(value)

    def _dispenser_station_pose(self) -> Pose:
        dispenser_id = self.dispenser_id_text()
        key = f"dispenser_{dispenser_id}"
        front_holds = self.dispenser_config.get("front_hold_poses", {})
        if key not in front_holds:
            raise ValueError(f"front_hold_poses.{key} not found")
        xyz = front_holds[key].get("position_xyz_m")
        if not isinstance(xyz, list) or len(xyz) < 3:
            raise ValueError(f"front_hold_poses.{key}.position_xyz_m is invalid")
        z_offset = self.tumbler_height() * 0.5 if bool(self.get_parameter("station_z_is_bottom").value) else 0.0
        return pose_from_xyz([float(v) for v in xyz[:3]], z_center_offset_m=z_offset)

    def _holder_station_pose(self) -> Pose:
        holder = self.calibration_config.get("cup_holder", {})
        xyz = holder.get("top_center_estimated_xyz_m") or holder.get("bottom_insert_center_pose_xyz_m")
        if not isinstance(xyz, list) or len(xyz) < 3:
            raise ValueError("cup_holder top/bottom center is missing in calibration.yaml")
        # The holder top center is near the cup bottom/insertion plane; publish a
        # vertical cylinder centered above it using documented tumbler height.
        return pose_from_xyz([float(v) for v in xyz[:3]], z_center_offset_m=self.tumbler_height() * 0.5)

    def _attached_object(self, operation: int) -> AttachedCollisionObject:
        attached = AttachedCollisionObject()
        attached.link_name = str(self.get_parameter("attached_link_name").value)
        attached.touch_links = [str(item) for item in self.get_parameter("touch_links").value]
        pose = Pose()
        pose.orientation.w = 1.0
        # Conservative placeholder centered on the attached link.  Precise
        # TCP-to-cup offsets can replace this after measured evidence is added.
        attached.object = self._collision_object(self.object_id, pose, CollisionObject.ADD)
        attached.object.header.frame_id = attached.link_name
        attached.object.operation = operation
        return attached

    def _publish_action_once(self) -> None:
        if self.action == "add_dispenser":
            object_id = f"tumbler_at_dispenser_{self.dispenser_id_text()}"
            self.collision_pub.publish(
                self._collision_object(object_id, self._dispenser_station_pose(), CollisionObject.ADD)
            )
            self.get_logger().info(f"Added world collision object {object_id}")
        elif self.action == "add_holder":
            self.collision_pub.publish(
                self._collision_object("tumbler_in_holder", self._holder_station_pose(), CollisionObject.ADD)
            )
            self.get_logger().info("Added world collision object tumbler_in_holder")
        elif self.action == "remove_world":
            self.collision_pub.publish(self._collision_object(self.object_id, Pose(), CollisionObject.REMOVE))
            self.get_logger().info(f"Removed world collision object {self.object_id}")
        elif self.action == "attach":
            self.attached_pub.publish(self._attached_object(CollisionObject.ADD))
            self.get_logger().info(f"Attached collision object {self.object_id}")
        elif self.action == "detach":
            self.attached_pub.publish(self._attached_object(CollisionObject.REMOVE))
            self.get_logger().info(f"Detached collision object {self.object_id}")
        # `main()` exits one-shot actions after a short DDS publish settle.


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TumblerCollisionSceneNode()
    try:
        if node.action != "publish_detected" and bool(node.get_parameter("publish_once").value):
            time_to_settle_sec = 0.35
            import time

            time.sleep(time_to_settle_sec)
            return
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
