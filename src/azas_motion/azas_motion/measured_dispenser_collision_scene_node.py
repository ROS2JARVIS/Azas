#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_CONFIG_PATH = (
    "/home/ssu/Azas/src/azas_bringup/config/measured_dispenser_collision.yaml"
)

LEGACY_DISPENSER_COLLISION_OBJECT_IDS = (
    "dispenser_body_box",
    "dispenser_1_body_box_v2",
    "dispenser_2_body_box_v2",
    "dispenser_3_body_box_v2",
    "dispenser_4_body_box_v2",
    "dispenser_head_box",
    "dispenser_head_nozzle_merged_vertical_box",
    "dispenser_head_nozzle_merged_horizontal_spout_box",
    "dispenser_1_head_nozzle_box",
    "dispenser_2_head_nozzle_box",
    "dispenser_3_head_nozzle_box",
    "dispenser_4_head_nozzle_box",
)

COURSE_WORKSPACE_COLLISION_OBJECT_IDS = (
    "side_grip_workspace_x_min_wall",
    "side_grip_workspace_x_max_wall",
    "side_grip_workspace_y_min_wall",
    "side_grip_workspace_y_max_wall",
)


def transient_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _xyz(values: list[float]) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"expected xyz list with 3 values, got {values!r}")
    return float(values[0]), float(values[1]), float(values[2])


def _xyzw(values: list[float]) -> tuple[float, float, float, float]:
    if len(values) != 4:
        raise ValueError(f"expected xyzw list with 4 values, got {values!r}")
    return float(values[0]), float(values[1]), float(values[2]), float(values[3])


def _pose(center_xyz: list[float], orientation_xyzw: list[float]) -> Pose:
    x, y, z = _xyz(center_xyz)
    qx, qy, qz, qw = _xyzw(orientation_xyzw)

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def _point_inside_bounds(point: list[float], bounds: dict[str, list[float]]) -> bool:
    px, py, pz = _xyz(point)
    min_x, min_y, min_z = _xyz(bounds["min"])
    max_x, max_y, max_z = _xyz(bounds["max"])
    return min_x <= px <= max_x and min_y <= py <= max_y and min_z <= pz <= max_z


def _rotate_vector_by_inverse_quaternion(
    vector: tuple[float, float, float],
    quaternion_xyzw: list[float],
) -> tuple[float, float, float]:
    qx, qy, qz, qw = _xyzw(quaternion_xyzw)
    vx, vy, vz = vector

    # q_conjugate * v * q; enough here to validate oriented box containment.
    ix = qw * vx - qy * vz + qz * vy
    iy = qw * vy - qz * vx + qx * vz
    iz = qw * vz - qx * vy + qy * vx
    iw = qx * vx + qy * vy + qz * vz

    return (
        ix * qw + iw * qx + iy * qz - iz * qy,
        iy * qw + iw * qy + iz * qx - ix * qz,
        iz * qw + iw * qz + ix * qy - iy * qx,
    )


def _point_inside_box_object(point: list[float], object_config: dict[str, Any]) -> bool:
    if "bounds_xyz_m" in object_config:
        return _point_inside_bounds(point, object_config["bounds_xyz_m"])

    px, py, pz = _xyz(point)
    cx, cy, cz = _xyz(object_config["center_xyz_m"])
    sx, sy, sz = _xyz(object_config["size_xyz_m"])
    local_x, local_y, local_z = _rotate_vector_by_inverse_quaternion(
        (px - cx, py - cy, pz - cz),
        object_config.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0]),
    )
    epsilon = 1e-9
    return (
        abs(local_x) <= sx / 2.0 + epsilon
        and abs(local_y) <= sy / 2.0 + epsilon
        and abs(local_z) <= sz / 2.0 + epsilon
    )


class MeasuredDispenserCollisionSceneNode(Node):
    def __init__(self) -> None:
        super().__init__("measured_dispenser_collision_scene_node")

        self.declare_parameter("config_path", DEFAULT_CONFIG_PATH)
        self.declare_parameter("publish_period_sec", 2.0)
        self.declare_parameter("publish_collision_objects", True)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("publish_rviz_visual_tools_compat", True)
        self.declare_parameter("publish_debug_labels", True)
        self.declare_parameter("remove_legacy_collision_objects", True)
        self.declare_parameter("remove_course_workspace_collision_objects", False)
        self.declare_parameter("clear_markers_before_publish", True)
        self.declare_parameter("collision_object_exclude_ids", "")

        config_path = Path(
            self.get_parameter("config_path").get_parameter_value().string_value
        )
        self.config = self._load_config(config_path)
        self.frame_id = self.config.get("metadata", {}).get("frame_id", "base_link")

        self.collision_pub = self.create_publisher(
            CollisionObject, "/collision_object", transient_qos(10)
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/azas/measured_dispenser_collision/markers", transient_qos(10)
        )
        self.rviz_visual_tools_pub = self.create_publisher(
            MarkerArray, "/rviz_visual_tools", transient_qos(10)
        )

        self.publish_collision_objects = (
            self.get_parameter("publish_collision_objects")
            .get_parameter_value()
            .bool_value
        )
        self.publish_markers = (
            self.get_parameter("publish_markers").get_parameter_value().bool_value
        )
        self.publish_rviz_visual_tools_compat = (
            self.get_parameter("publish_rviz_visual_tools_compat")
            .get_parameter_value()
            .bool_value
        )
        self.publish_debug_labels = (
            self.get_parameter("publish_debug_labels").get_parameter_value().bool_value
        )
        self.remove_legacy_collision_objects = (
            self.get_parameter("remove_legacy_collision_objects")
            .get_parameter_value()
            .bool_value
        )
        self.remove_course_workspace_collision_objects = (
            self.get_parameter("remove_course_workspace_collision_objects")
            .get_parameter_value()
            .bool_value
        )
        self.clear_markers_before_publish = (
            self.get_parameter("clear_markers_before_publish")
            .get_parameter_value()
            .bool_value
        )
        exclude_raw = (
            self.get_parameter("collision_object_exclude_ids")
            .get_parameter_value()
            .string_value
        )
        self.collision_object_exclude_ids = {
            item.strip()
            for item in exclude_raw.replace(";", ",").split(",")
            if item.strip()
        }

        self._warn_about_draft_status()
        self._warn_about_front_hold_overlaps()
        self._legacy_collision_objects_removed = False
        self._published_ids_logged = False
        self._publish_scene()

        period = (
            self.get_parameter("publish_period_sec").get_parameter_value().double_value
        )
        self.timer = self.create_timer(max(0.5, period), self._publish_scene)

    def _load_config(self, config_path: Path) -> dict[str, Any]:
        if not config_path.exists():
            raise FileNotFoundError(f"collision config does not exist: {config_path}")
        with config_path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        if not isinstance(config, dict):
            raise ValueError(f"collision config is not a YAML map: {config_path}")
        self.get_logger().info(f"Loaded measured dispenser collision config: {config_path}")
        return config

    def _warn_about_draft_status(self) -> None:
        metadata = self.config.get("metadata", {})
        status = str(metadata.get("status", "unknown"))
        if "draft" in status or "not_enabled" in status:
            self.get_logger().warning(
                "Collision config status is draft/not-enabled. Use for RViz review "
                "first, not direct real-motion approval."
            )

        for object_id, object_config in self._collision_objects().items():
            if not object_config.get("publish_to_planning_scene", True):
                continue
            if not object_config.get("enabled_for_real_motion", False):
                self.get_logger().warning(
                    f"{object_id} enabled_for_real_motion=false; publishing is for "
                    "PlanningScene/RViz validation only."
                )

    def _warn_about_front_hold_overlaps(self) -> None:
        front_holds = self.config.get("front_hold_poses", {})
        for hold_name, hold_config in front_holds.items():
            point = hold_config.get("position_xyz_m")
            if point is None:
                continue
            for object_id, object_config in self._collision_objects().items():
                if not object_config.get("publish_to_planning_scene", True):
                    continue
                if _point_inside_box_object(point, object_config):
                    self.get_logger().warning(
                        f"{hold_name} front-hold point {point} is inside {object_id}. "
                        "This can over-block planning; verify in RViz before enabling."
                    )

    def _collision_objects(self) -> dict[str, dict[str, Any]]:
        objects = self.config.get("estimated_collision_objects", {})
        if not isinstance(objects, dict):
            raise ValueError("estimated_collision_objects must be a YAML map")
        return objects

    def _publish_scene(self) -> None:
        collision_objects = self._collision_objects()
        if (
            self.remove_legacy_collision_objects
            and not self._legacy_collision_objects_removed
        ):
            remove_ids = list(LEGACY_DISPENSER_COLLISION_OBJECT_IDS)
            if self.remove_course_workspace_collision_objects:
                remove_ids.extend(COURSE_WORKSPACE_COLLISION_OBJECT_IDS)
            for object_id in remove_ids:
                self.collision_pub.publish(self._make_remove_collision_object(object_id))
            self._legacy_collision_objects_removed = True
            if self.remove_course_workspace_collision_objects:
                self.get_logger().info(
                    "Requested removal of course workspace wall collision objects: "
                    + ", ".join(COURSE_WORKSPACE_COLLISION_OBJECT_IDS)
                )
        if self.publish_collision_objects:
            published_ids = []
            for object_id, object_config in collision_objects.items():
                if object_id in self.collision_object_exclude_ids:
                    if not self._published_ids_logged:
                        self.get_logger().info(
                            f"Skipping PlanningScene collision object {object_id}; "
                            "it remains visible as an RViz marker only."
                        )
                    continue
                if not object_config.get("publish_to_planning_scene", True):
                    continue
                self.collision_pub.publish(
                    self._make_collision_object(object_id, object_config)
                )
                published_ids.append(object_id)
            if published_ids and not self._published_ids_logged:
                self.get_logger().info(
                    "Publishing measured dispenser collision objects: " + ", ".join(published_ids)
                )
                self._published_ids_logged = True

        if self.publish_markers:
            markers = self._make_markers(collision_objects)
            self.marker_pub.publish(markers)
            if self.publish_rviz_visual_tools_compat:
                rviz_markers = MarkerArray(
                    markers=[m for m in markers.markers if m.action != Marker.DELETEALL]
                )
                self.rviz_visual_tools_pub.publish(rviz_markers)

    def _make_collision_object(
        self, object_id: str, object_config: dict[str, Any]
    ) -> CollisionObject:
        if object_config.get("type") != "box":
            raise ValueError(f"{object_id}: only type=box is supported")

        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = object_config.get("frame_id", self.frame_id)

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(_xyz(object_config["size_xyz_m"]))

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(
            _pose(
                object_config["center_xyz_m"],
                object_config.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0]),
            )
        )
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def _make_remove_collision_object(self, object_id: str) -> CollisionObject:
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = self.frame_id
        collision_object.operation = CollisionObject.REMOVE
        return collision_object

    def _make_markers(
        self, collision_objects: dict[str, dict[str, Any]]
    ) -> MarkerArray:
        markers: list[Marker] = []
        stamp = self.get_clock().now().to_msg()

        if self.clear_markers_before_publish:
            clear_marker = Marker()
            clear_marker.header.frame_id = self.frame_id
            clear_marker.header.stamp = stamp
            clear_marker.action = Marker.DELETEALL
            markers.append(clear_marker)

        for index, (object_id, object_config) in enumerate(collision_objects.items()):
            marker = Marker()
            marker.header.frame_id = object_config.get("frame_id", self.frame_id)
            marker.header.stamp = stamp
            marker.ns = "measured_dispenser_collision_boxes"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = _pose(
                object_config["center_xyz_m"],
                object_config.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0]),
            )
            sx, sy, sz = _xyz(object_config["size_xyz_m"])
            marker.scale.x = sx
            marker.scale.y = sy
            marker.scale.z = sz
            if not object_config.get("publish_to_planning_scene", True):
                marker.color.r = 0.55
                marker.color.g = 0.55
                marker.color.b = 0.55
                marker.color.a = 0.12
            elif "head" in object_id:
                marker.color.r = 0.20
                marker.color.g = 0.35
                marker.color.b = 1.00
                marker.color.a = 0.65
            else:
                marker.color.r = 1.00
                marker.color.g = 0.40
                marker.color.b = 0.10
                marker.color.a = 0.70
            markers.append(marker)

            if self.publish_debug_labels and object_config.get(
                "publish_to_planning_scene", True
            ):
                label = Marker()
                label.header.frame_id = object_config.get("frame_id", self.frame_id)
                label.header.stamp = stamp
                label.ns = "measured_dispenser_collision_box_labels"
                label.id = 1000 + index
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose = _pose(
                    object_config["center_xyz_m"], [0.0, 0.0, 0.0, 1.0]
                )
                label.pose.position.z += sz / 2.0 + 0.08
                label.scale.z = 0.045
                label.color.r = 1.0
                label.color.g = 1.0
                label.color.b = 1.0
                label.color.a = 1.0
                label.text = object_id
                markers.append(label)

        marker_id = 100
        for hold_name, hold_config in self.config.get("front_hold_poses", {}).items():
            point = hold_config.get("position_xyz_m")
            if point is None:
                continue
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "measured_dispenser_front_hold_points"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = _pose(point, hold_config.get("quaternion_xyzw", [0, 0, 0, 1]))
            marker.scale.x = 0.035
            marker.scale.y = 0.035
            marker.scale.z = 0.035
            marker.color.r = 0.05
            marker.color.g = 0.90
            marker.color.b = 0.30
            marker.color.a = 0.85
            markers.append(marker)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "measured_dispenser_front_hold_labels"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = _pose(point, [0.0, 0.0, 0.0, 1.0])
            label.pose.position.z += 0.05
            label.scale.z = 0.035
            label.color.r = 0.05
            label.color.g = 0.90
            label.color.b = 0.30
            label.color.a = 0.95
            label.text = hold_name
            markers.append(label)

        return MarkerArray(markers=markers)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MeasuredDispenserCollisionSceneNode()
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
