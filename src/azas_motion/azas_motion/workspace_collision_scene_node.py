#!/usr/bin/env python3
from __future__ import annotations

import time
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


DEFAULT_SAFETY_CONFIG_PATH = "/home/ssu/Azas/src/azas_bringup/config/safety.yaml"


def transient_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def parse_workspace_bounds(value: Any) -> dict[str, float] | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError("workspace_bounds_m must be a YAML map")

    required = ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")
    missing = [key for key in required if key not in value]
    if missing:
        raise ValueError(f"workspace_bounds_m is missing keys: {missing}")

    bounds = {key: float(value[key]) for key in required}
    if bounds["x_min"] > bounds["x_max"]:
        raise ValueError("workspace_bounds_m x_min must be <= x_max")
    if bounds["y_min"] > bounds["y_max"]:
        raise ValueError("workspace_bounds_m y_min must be <= y_max")
    if bounds["z_min"] > bounds["z_max"]:
        raise ValueError("workspace_bounds_m z_min must be <= z_max")
    return bounds


class WorkspaceCollisionSceneNode(Node):
    def __init__(self) -> None:
        super().__init__("workspace_collision_scene_node")

        self.declare_parameter("safety_config_path", DEFAULT_SAFETY_CONFIG_PATH)
        self.declare_parameter("publish_period_sec", 2.0)
        self.declare_parameter("publish_collision_objects", True)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("table_collision_enabled", True)
        self.declare_parameter("table_collision_id", "side_grip_table")
        self.declare_parameter("table_surface_z", 0.0)
        self.declare_parameter("table_thickness", 0.04)
        self.declare_parameter("table_size_x", 1.20)
        self.declare_parameter("table_size_y", 1.00)
        self.declare_parameter("table_center_x", 0.45)
        self.declare_parameter("table_center_y", 0.0)
        self.declare_parameter("table_collision_expand_to_workspace_walls", True)
        self.declare_parameter("publish_repeats", 3)
        self.declare_parameter("workspace_boundary_collision_enabled", True)
        self.declare_parameter("workspace_boundary_collision_prefix", "side_grip_workspace")
        self.declare_parameter("workspace_boundary_wall_thickness", 0.04)
        self.declare_parameter("workspace_boundary_wall_clearance", 0.02)

        safety_config_path = Path(
            self.get_parameter("safety_config_path").get_parameter_value().string_value
        ).expanduser()
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.publish_collision_objects = (
            self.get_parameter("publish_collision_objects")
            .get_parameter_value()
            .bool_value
        )
        self.table_collision_enabled = (
            self.get_parameter("table_collision_enabled").get_parameter_value().bool_value
        )
        self.table_collision_id = (
            self.get_parameter("table_collision_id").get_parameter_value().string_value
        )
        self.table_surface_z = (
            self.get_parameter("table_surface_z").get_parameter_value().double_value
        )
        self.table_thickness = max(
            0.001,
            self.get_parameter("table_thickness").get_parameter_value().double_value,
        )
        self.table_size_x = (
            self.get_parameter("table_size_x").get_parameter_value().double_value
        )
        self.table_size_y = (
            self.get_parameter("table_size_y").get_parameter_value().double_value
        )
        self.table_center_x = (
            self.get_parameter("table_center_x").get_parameter_value().double_value
        )
        self.table_center_y = (
            self.get_parameter("table_center_y").get_parameter_value().double_value
        )
        self.table_collision_expand_to_workspace_walls = (
            self.get_parameter("table_collision_expand_to_workspace_walls")
            .get_parameter_value()
            .bool_value
        )
        self.publish_repeats = max(
            1, self.get_parameter("publish_repeats").get_parameter_value().integer_value
        )
        self.workspace_boundary_collision_enabled = (
            self.get_parameter("workspace_boundary_collision_enabled")
            .get_parameter_value()
            .bool_value
        )
        self.workspace_boundary_collision_prefix = (
            self.get_parameter("workspace_boundary_collision_prefix")
            .get_parameter_value()
            .string_value
        )
        self.workspace_boundary_wall_thickness = max(
            0.001,
            self.get_parameter("workspace_boundary_wall_thickness")
            .get_parameter_value()
            .double_value,
        )
        self.workspace_boundary_wall_clearance = max(
            0.0,
            self.get_parameter("workspace_boundary_wall_clearance")
            .get_parameter_value()
            .double_value,
        )

        self.workspace_bounds_m = self._load_safety_workspace_bounds(safety_config_path)
        self.collision_pub = self.create_publisher(
            CollisionObject, "/collision_object", transient_qos(10)
        )

        self._publish_scene()
        period = self.get_parameter("publish_period_sec").get_parameter_value().double_value
        self.timer = self.create_timer(max(0.5, period), self._publish_scene)

    def _load_safety_workspace_bounds(self, safety_config_path: Path) -> dict[str, float]:
        if not safety_config_path.exists():
            raise FileNotFoundError(f"safety config does not exist: {safety_config_path}")
        with safety_config_path.open("r", encoding="utf-8") as stream:
            safety_config = yaml.safe_load(stream) or {}

        motion_config = safety_config.get("motion", {})
        if not isinstance(motion_config, dict):
            raise ValueError(f"motion section is not a YAML map: {safety_config_path}")

        bounds = parse_workspace_bounds(motion_config.get("workspace_bounds_m"))
        if bounds is None:
            raise ValueError(
                f"motion.workspace_bounds_m must contain x/y/z min/max values: "
                f"{safety_config_path}"
            )

        min_z = motion_config.get("min_z_m")
        if min_z is not None and float(min_z) > bounds["z_min"]:
            self.get_logger().warning(
                f"safety min_z_m={float(min_z):.3f} is higher than workspace z_min="
                f"{bounds['z_min']:.3f}; using min_z_m for boundary floor height"
            )
            bounds["z_min"] = float(min_z)

        self.get_logger().info(
            f"Loaded workspace safety bounds from {safety_config_path}: "
            f"x=[{bounds['x_min']:.3f}, {bounds['x_max']:.3f}], "
            f"y=[{bounds['y_min']:.3f}, {bounds['y_max']:.3f}], "
            f"z=[{bounds['z_min']:.3f}, {bounds['z_max']:.3f}]"
        )
        return bounds

    def _make_box_collision_object(
        self,
        object_id: str,
        center_xyz: list[float],
        size_xyz: list[float],
    ) -> CollisionObject:
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = self.frame_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in size_xyz]

        pose = Pose()
        pose.position.x = float(center_xyz[0])
        pose.position.y = float(center_xyz[1])
        pose.position.z = float(center_xyz[2])
        pose.orientation.w = 1.0

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def _table_collision_object(self) -> CollisionObject | None:
        if not self.table_collision_enabled:
            return None

        table_center_x = self.table_center_x
        table_center_y = self.table_center_y
        table_size_x = self.table_size_x
        table_size_y = self.table_size_y
        if self.table_collision_expand_to_workspace_walls:
            clearance = (
                self.workspace_boundary_wall_clearance
                if self.workspace_boundary_collision_enabled
                else 0.0
            )
            x_min = self.workspace_bounds_m["x_min"] - clearance
            x_max = self.workspace_bounds_m["x_max"] + clearance
            y_min = self.workspace_bounds_m["y_min"] - clearance
            y_max = self.workspace_bounds_m["y_max"] + clearance
            table_center_x = (x_min + x_max) * 0.5
            table_center_y = (y_min + y_max) * 0.5
            table_size_x = x_max - x_min
            table_size_y = y_max - y_min

        center_z = self.table_surface_z - self.table_thickness * 0.5
        return self._make_box_collision_object(
            self.table_collision_id or "side_grip_table",
            [table_center_x, table_center_y, center_z],
            [table_size_x, table_size_y, self.table_thickness],
        )

    def _workspace_wall_collision_objects(self) -> list[CollisionObject]:
        if not self.workspace_boundary_collision_enabled:
            return []

        bounds = self.workspace_bounds_m
        thickness = self.workspace_boundary_wall_thickness
        clearance = self.workspace_boundary_wall_clearance
        prefix = self.workspace_boundary_collision_prefix or "side_grip_workspace"

        x_min = bounds["x_min"] - clearance
        x_max = bounds["x_max"] + clearance
        y_min = bounds["y_min"] - clearance
        y_max = bounds["y_max"] + clearance
        z_min = bounds["z_min"]
        z_max = bounds["z_max"]
        x_span = x_max - x_min
        y_span = y_max - y_min
        z_span = z_max - z_min
        x_mid = (x_min + x_max) * 0.5
        y_mid = (y_min + y_max) * 0.5
        z_mid = (z_min + z_max) * 0.5

        return [
            self._make_box_collision_object(
                f"{prefix}_x_min_wall",
                [x_min - thickness * 0.5, y_mid, z_mid],
                [thickness, y_span + 2.0 * thickness, z_span],
            ),
            self._make_box_collision_object(
                f"{prefix}_x_max_wall",
                [x_max + thickness * 0.5, y_mid, z_mid],
                [thickness, y_span + 2.0 * thickness, z_span],
            ),
            self._make_box_collision_object(
                f"{prefix}_y_min_wall",
                [x_mid, y_min - thickness * 0.5, z_mid],
                [x_span + 2.0 * thickness, thickness, z_span],
            ),
            self._make_box_collision_object(
                f"{prefix}_y_max_wall",
                [x_mid, y_max + thickness * 0.5, z_mid],
                [x_span + 2.0 * thickness, thickness, z_span],
            ),
        ]

    def _publish_scene(self) -> None:
        if not self.publish_collision_objects:
            return

        objects = []
        table = self._table_collision_object()
        if table is not None:
            objects.append(table)
        objects.extend(self._workspace_wall_collision_objects())

        for _ in range(self.publish_repeats):
            for collision_object in objects:
                self.collision_pub.publish(collision_object)
            time.sleep(0.05)

        table_state = "on" if table is not None else "off"
        wall_state = "on" if self.workspace_boundary_collision_enabled else "off"
        self.get_logger().info(
            "Published workspace collision scene "
            f"frame={self.frame_id}, table={table_state}, walls={wall_state}, "
            f"objects={len(objects)}"
        )


def main() -> None:
    rclpy.init()
    node = WorkspaceCollisionSceneNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
