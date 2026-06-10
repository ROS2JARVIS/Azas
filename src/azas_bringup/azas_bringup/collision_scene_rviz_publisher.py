#!/usr/bin/env python3
"""Publish a high-visibility collision scene for RViz review."""

from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


WALL_THICKNESS = 0.04


def transient_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _read_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text())
    return data if isinstance(data, dict) else {}


def _box_marker(
    marker_id: int,
    namespace: str,
    frame_id: str,
    stamp,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    color: tuple[float, float, float, float],
    label: str = "",
) -> list[Marker]:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = Pose()
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = center
    marker.pose.orientation.w = 1.0
    marker.scale.x, marker.scale.y, marker.scale.z = size
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    markers = [marker]

    if label:
        text = Marker()
        text.header.frame_id = frame_id
        text.header.stamp = stamp
        text.ns = f"{namespace}_labels"
        text.id = marker_id + 10000
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = Pose()
        text.pose.position.x = center[0]
        text.pose.position.y = center[1]
        text.pose.position.z = center[2] + size[2] / 2.0 + 0.05
        text.pose.orientation.w = 1.0
        text.scale.z = 0.045
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = label
        markers.append(text)
    return markers


def _box_edge_marker(
    marker_id: int,
    namespace: str,
    frame_id: str,
    stamp,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    color: tuple[float, float, float, float],
    line_width: float = 0.018,
) -> Marker:
    cx, cy, cz = center
    sx, sy, sz = (value / 2.0 for value in size)
    corners = [
        (cx - sx, cy - sy, cz - sz),
        (cx + sx, cy - sy, cz - sz),
        (cx + sx, cy + sy, cz - sz),
        (cx - sx, cy + sy, cz - sz),
        (cx - sx, cy - sy, cz + sz),
        (cx + sx, cy - sy, cz + sz),
        (cx + sx, cy + sy, cz + sz),
        (cx - sx, cy + sy, cz + sz),
    ]
    edge_indices = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ]

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = line_width
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    for start, end in edge_indices:
        for corner_index in (start, end):
            point = Point()
            point.x, point.y, point.z = corners[corner_index]
            marker.points.append(point)
    return marker


def _box_with_edges(
    marker_id: int,
    namespace: str,
    frame_id: str,
    stamp,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    fill_color: tuple[float, float, float, float],
    edge_color: tuple[float, float, float, float],
    label: str = "",
) -> list[Marker]:
    markers = _box_marker(marker_id, namespace, frame_id, stamp, center, size, fill_color, label)
    markers.append(
        _box_edge_marker(
            marker_id + 20000,
            f"{namespace}_edges",
            frame_id,
            stamp,
            center,
            size,
            edge_color,
        )
    )
    return markers


class CollisionSceneRvizPublisher(Node):
    def __init__(self) -> None:
        super().__init__("collision_scene_rviz_publisher")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter(
            "safety_config_path",
            str(Path(get_package_share_directory("azas_bringup")) / "config" / "safety.yaml"),
        )
        self.declare_parameter(
            "dispenser_collision_config_path",
            str(
                Path(get_package_share_directory("azas_bringup"))
                / "config"
                / "measured_dispenser_collision.yaml"
            ),
        )
        self.declare_parameter(
            "calibration_path",
            str(Path(get_package_share_directory("azas_bringup")) / "config" / "calibration.yaml"),
        )
        self.declare_parameter("publish_workspace_ceiling", False)

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.publish_workspace_ceiling = (
            self.get_parameter("publish_workspace_ceiling").get_parameter_value().bool_value
        )
        self.safety_path = Path(
            self.get_parameter("safety_config_path").get_parameter_value().string_value
        )
        self.dispenser_path = Path(
            self.get_parameter("dispenser_collision_config_path").get_parameter_value().string_value
        )
        self.calibration_path = Path(
            self.get_parameter("calibration_path").get_parameter_value().string_value
        )
        self.publisher = self.create_publisher(
            MarkerArray, "/azas/collision_scene/markers", transient_qos(10)
        )
        self.default_marker_publisher = self.create_publisher(
            MarkerArray, "/visualization_marker_array", transient_qos(10)
        )
        self.timer = self.create_timer(1.0, self._publish)
        self._publish()
        self.get_logger().info(
            "Publishing high-visibility collision scene to "
            "/azas/collision_scene/markers and /visualization_marker_array"
        )

    def _workspace_markers(self, stamp, start_id: int) -> tuple[list[Marker], int]:
        safety = _read_yaml(self.safety_path)
        bounds = safety.get("motion", {}).get("workspace_bounds_m", {})
        x_min = float(bounds.get("x_min", -0.25))
        x_max = float(bounds.get("x_max", 1.15))
        y_min = float(bounds.get("y_min", -0.60))
        y_max = float(bounds.get("y_max", 0.60))
        z_min = float(bounds.get("z_min", 0.07))
        z_max = float(bounds.get("z_max", 0.80))

        height = z_max - z_min
        cx = (x_min + x_max) / 2.0
        cy = (y_min + y_max) / 2.0
        cz = z_min + height / 2.0
        dx = x_max - x_min
        dy = y_max - y_min
        t = WALL_THICKNESS
        walls = [
            ("+Y safety", (cx, y_max + t / 2.0, cz), (dx + 2 * t, t, height)),
            ("-Y safety", (cx, y_min - t / 2.0, cz), (dx + 2 * t, t, height)),
            ("+X safety", (x_max + t / 2.0, cy, cz), (t, dy, height)),
            ("-X safety", (x_min - t / 2.0, cy, cz), (t, dy, height)),
            ("floor", (cx, cy, z_min - t / 2.0), (dx, dy, t)),
        ]
        if self.publish_workspace_ceiling:
            walls.append(("ceiling", (cx, cy, z_max + t / 2.0), (dx, dy, t)))

        markers: list[Marker] = []
        marker_id = start_id
        for label, center, size in walls:
            markers.extend(
                _box_with_edges(
                    marker_id,
                    "safety_workspace_green",
                    self.frame_id,
                    stamp,
                    center,
                    size,
                    (0.0, 0.95, 0.25, 0.18),
                    (0.0, 1.0, 0.15, 1.0),
                    label,
                )
            )
            marker_id += 1
        return markers, marker_id

    def _table_markers(self, stamp, start_id: int) -> tuple[list[Marker], int]:
        calibration = _read_yaml(self.calibration_path)
        table = calibration.get("table", {})
        if not table:
            return [], start_id
        center_xy = table.get("center_xy_m", [0.45, 0.0])
        size_xy = table.get("size_xy_m", [1.2, 1.0])
        thickness = float(table.get("thickness_m", 0.04))
        surface_z = float(table.get("surface_z_m", 0.0))
        markers = _box_with_edges(
            start_id,
            "table_collision",
            self.frame_id,
            stamp,
            (float(center_xy[0]), float(center_xy[1]), surface_z - thickness / 2.0),
            (float(size_xy[0]), float(size_xy[1]), thickness),
            (0.48, 0.32, 0.16, 0.42),
            (0.85, 0.55, 0.20, 1.0),
            "table",
        )
        return markers, start_id + 1

    def _dispenser_markers(self, stamp, start_id: int) -> tuple[list[Marker], int]:
        dispenser = _read_yaml(self.dispenser_path)
        markers: list[Marker] = []
        marker_id = start_id
        for object_id, obj in (dispenser.get("estimated_collision_objects") or {}).items():
            if obj.get("type") != "box":
                continue
            center = tuple(float(value) for value in obj["center_xyz_m"])
            size = tuple(float(value) for value in obj["size_xyz_m"])
            markers.extend(
                _box_with_edges(
                    marker_id,
                    "dispenser_collision_orange",
                    self.frame_id,
                    stamp,
                    center,
                    size,
                    (1.0, 0.33, 0.0, 0.42),
                    (1.0, 0.55, 0.0, 1.0),
                    object_id,
                )
            )
            marker_id += 1

        for hold_name, hold in (dispenser.get("front_hold_poses") or {}).items():
            xyz = hold.get("position_xyz_m")
            if not xyz:
                continue
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "dispenser_front_hold_green"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = Pose()
            marker.pose.position.x = float(xyz[0])
            marker.pose.position.y = float(xyz[1])
            marker.pose.position.z = float(xyz[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.035
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.25
            marker.color.a = 1.0
            markers.append(marker)
            marker_id += 1

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "dispenser_front_hold_labels"
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = Pose()
            label.pose.position.x = float(xyz[0])
            label.pose.position.y = float(xyz[1])
            label.pose.position.z = float(xyz[2]) + 0.06
            label.pose.orientation.w = 1.0
            label.scale.z = 0.04
            label.color.r = 0.0
            label.color.g = 1.0
            label.color.b = 0.25
            label.color.a = 1.0
            label.text = hold_name
            markers.append(label)
            marker_id += 1
        return markers, marker_id

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL

        markers: list[Marker] = [clear]
        marker_id = 0
        for builder in (self._workspace_markers, self._table_markers, self._dispenser_markers):
            built, marker_id = builder(stamp, marker_id)
            markers.extend(built)
        marker_array = MarkerArray(markers=markers)
        self.publisher.publish(marker_array)
        self.default_marker_publisher.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CollisionSceneRvizPublisher()
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
