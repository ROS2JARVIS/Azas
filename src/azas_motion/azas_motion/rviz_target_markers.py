"""RViz marker publishing for motion validation targets."""

from __future__ import annotations

from geometry_msgs.msg import Point
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

from .dispenser_targets import Position

DISPENSER_MESH = "package://azas_motion/models/azas_dispenser_single.obj"

# Local model coordinates from tools/generate_tumbler_dispenser_models.py.
# The pump outlet cylinder center is used as the visual alignment point.
DISPENSER_OUTLET_LOCAL_X = 0.175
DISPENSER_OUTLET_LOCAL_Z = 0.392


class RvizTargetMarkers:
    def __init__(self, node, frame_id: str) -> None:
        self._node = node
        self._frame_id = frame_id
        self._floor_publisher = None
        self._dispenser_publisher = None

    def publish_floor_target(
        self, floor_x: float, floor_y: float, floor_z: float, approach_z: float
    ) -> None:
        if self._floor_publisher is None:
            self._floor_publisher = self._node.create_publisher(
                Marker, "/azas/floor_target_marker", transient_qos(3)
            )

        self._floor_publisher.publish(
            self.marker(
                marker_id=1,
                marker_type=Marker.SPHERE,
                x=floor_x,
                y=floor_y,
                z=floor_z,
                scale=(0.055, 0.055, 0.055),
                color=(0.95, 0.10, 0.10, 0.90),
            )
        )
        self._floor_publisher.publish(
            self.marker(
                marker_id=2,
                marker_type=Marker.SPHERE,
                x=floor_x,
                y=floor_y,
                z=approach_z,
                scale=(0.045, 0.045, 0.045),
                color=(0.10, 0.35, 0.95, 0.80),
            )
        )
        line = self.marker(
            marker_id=3,
            marker_type=Marker.LINE_STRIP,
            x=floor_x,
            y=floor_y,
            z=floor_z,
            scale=(0.012, 0.0, 0.0),
            color=(0.95, 0.75, 0.05, 0.90),
        )
        line.points = [
            Point(x=floor_x, y=floor_y, z=approach_z),
            Point(x=floor_x, y=floor_y, z=floor_z),
        ]
        self._floor_publisher.publish(line)

    def publish_dispenser_target(
        self,
        outlets: list[Position],
        hold: Position,
        selected_dispenser_id: int,
    ) -> None:
        if self._dispenser_publisher is None:
            self._dispenser_publisher = self._node.create_publisher(
                Marker, "/azas/dispenser_target_marker", transient_qos(8)
            )

        selected_outlet = outlets[min(max(selected_dispenser_id, 1), len(outlets)) - 1]
        for index, outlet in enumerate(outlets, start=1):
            self._dispenser_publisher.publish(
                self.dispenser_mesh_marker(marker_id=10 + index, outlet=outlet)
            )
            self._dispenser_publisher.publish(
                self.marker(
                    marker_id=20 + index,
                    marker_type=Marker.SPHERE,
                    x=outlet.x,
                    y=outlet.y,
                    z=outlet.z,
                    scale=(0.035, 0.035, 0.035),
                    color=(0.15, 0.70, 0.95, 0.85),
                )
            )

        self._dispenser_publisher.publish(
            self.marker(
                marker_id=40,
                marker_type=Marker.SPHERE,
                x=hold.x,
                y=hold.y,
                z=hold.z,
                scale=(0.055, 0.055, 0.055),
                color=(0.95, 0.18, 0.10, 0.95),
            )
        )

        guide = self.marker(
            marker_id=41,
            marker_type=Marker.LINE_STRIP,
            x=hold.x,
            y=hold.y,
            z=hold.z,
            scale=(0.012, 0.0, 0.0),
            color=(0.95, 0.78, 0.08, 0.95),
        )
        guide.points = [
            Point(x=hold.x, y=hold.y, z=hold.z),
            Point(x=selected_outlet.x, y=hold.y, z=hold.z),
        ]
        self._dispenser_publisher.publish(guide)

    def publish_dispenser_sequence_path(self, poses) -> None:
        if self._dispenser_publisher is None:
            self._dispenser_publisher = self._node.create_publisher(
                Marker, "/azas/dispenser_target_marker", transient_qos(8)
            )

        path = self.marker(
            marker_id=70,
            marker_type=Marker.LINE_STRIP,
            x=0.0,
            y=0.0,
            z=0.0,
            scale=(0.014, 0.0, 0.0),
            color=(0.95, 0.78, 0.08, 0.95),
        )
        path.points = [
            Point(x=pose.pose.position.x, y=pose.pose.position.y, z=pose.pose.position.z)
            for label, pose in poses
            if "outlet_front_hold" in label
        ]
        self._dispenser_publisher.publish(path)

    def marker(
        self,
        marker_id: int,
        marker_type: int,
        x: float,
        y: float,
        z: float,
        scale: tuple[float, float, float],
        color: tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.ns = "azas_motion_targets"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        return marker

    def dispenser_mesh_marker(self, marker_id: int, outlet: Position) -> Marker:
        marker = self.marker(
            marker_id=marker_id,
            marker_type=Marker.MESH_RESOURCE,
            x=outlet.x + DISPENSER_OUTLET_LOCAL_X,
            y=outlet.y,
            z=outlet.z - DISPENSER_OUTLET_LOCAL_Z,
            scale=(1.0, 1.0, 1.0),
            color=(1.0, 1.0, 1.0, 1.0),
        )
        marker.mesh_resource = DISPENSER_MESH
        marker.mesh_use_embedded_materials = True
        marker.pose.orientation.z = 1.0
        marker.pose.orientation.w = 0.0
        return marker


def transient_qos(depth: int) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
