#!/usr/bin/env python3
"""Standalone collision scene visualizer for RViz.

Publishes ALL collision boxes (workspace walls + table + dispenser body)
as MarkerArray on /azas/collision_scene/markers.

Usage:
  source /home/ssu/Azas/install/local_setup.bash
  python3 tools/run/publish_collision_scene_rviz.py

RViz: Add > MarkerArray > topic: /azas/collision_scene/markers
      Fixed Frame: base_link
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
import yaml

SAFETY_YAML   = ROOT / "src" / "azas_bringup" / "config" / "safety.yaml"
DISPENSER_YAML = ROOT / "src" / "azas_bringup" / "config" / "measured_dispenser_collision.yaml"
CALIBRATION_YAML = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"
WALL_THICKNESS = 0.04
FRAME_ID = "base_link"


def transient_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def make_box_marker(
    marker_id: int, ns: str,
    cx: float, cy: float, cz: float,
    sx: float, sy: float, sz: float,
    r: float, g: float, b: float, a: float,
    stamp, label: str = "",
) -> list[Marker]:
    markers = []
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose = Pose()
    m.pose.position.x = cx
    m.pose.position.y = cy
    m.pose.position.z = cz
    m.pose.orientation.w = 1.0
    m.scale.x = sx
    m.scale.y = sy
    m.scale.z = sz
    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = a
    markers.append(m)

    if label:
        t = Marker()
        t.header.frame_id = FRAME_ID
        t.header.stamp = stamp
        t.ns = ns + "_labels"
        t.id = marker_id + 10000
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose = Pose()
        t.pose.position.x = cx
        t.pose.position.y = cy
        t.pose.position.z = cz + sz / 2.0 + 0.05
        t.pose.orientation.w = 1.0
        t.scale.z = 0.04
        t.color.r = 1.0
        t.color.g = 1.0
        t.color.b = 1.0
        t.color.a = 1.0
        t.text = label
        markers.append(t)
    return markers


def build_markers(stamp) -> list[Marker]:
    markers: list[Marker] = []
    mid = 0

    # ── 1. 워크스페이스 경계 벽 (safety.yaml) ──────────────────────────────
    safety = yaml.safe_load(SAFETY_YAML.read_text())
    wb = safety["motion"]["workspace_bounds_m"]
    x_min, x_max = wb["x_min"], wb["x_max"]
    y_min, y_max = wb["y_min"], wb["y_max"]
    z_min, z_max = wb["z_min"], wb["z_max"]
    t = WALL_THICKNESS
    height = z_max - z_min
    cx = (x_min + x_max) / 2
    cy = (y_min + y_max) / 2
    cz = z_min + height / 2
    dx = x_max - x_min
    dy = y_max - y_min

    walls = [
        # (label, cx, cy, cz, sx, sy, sz)
        ("+Y wall", cx, y_max + t/2, cz, dx + 2*t, t, height),
        ("-Y wall", cx, y_min - t/2, cz, dx + 2*t, t, height),
        ("+X wall", x_max + t/2, cy, cz, t, dy, height),
        ("-X wall", x_min - t/2, cy, cz, t, dy, height),
        ("floor",   cx, cy, z_min - t/2, dx, dy, t),
        ("ceiling", cx, cy, z_max + t/2, dx, dy, t),
    ]
    for label, wcx, wcy, wcz, wsx, wsy, wsz in walls:
        markers += make_box_marker(mid, "workspace_walls",
            wcx, wcy, wcz, wsx, wsy, wsz,
            0.2, 0.6, 1.0, 0.18, stamp, label)
        mid += 1

    # ── 2. 테이블 (calibration.yaml) ──────────────────────────────────────
    calib = yaml.safe_load(CALIBRATION_YAML.read_text())
    tbl = calib.get("table", {})
    if tbl:
        tcx = tbl.get("center_xy_m", [0.45, 0.0])[0]
        tcy = tbl.get("center_xy_m", [0.45, 0.0])[1]
        tsx, tsy = tbl.get("size_xy_m", [1.2, 1.0])
        thick = tbl.get("thickness_m", 0.04)
        surf_z = tbl.get("surface_z_m", 0.0)
        markers += make_box_marker(mid, "table",
            tcx, tcy, surf_z - thick/2, tsx, tsy, thick,
            0.6, 0.4, 0.2, 0.55, stamp, "table")
        mid += 1

    # ── 3. 디스펜서 합산 박스 (measured_dispenser_collision.yaml) ──────────
    disp = yaml.safe_load(DISPENSER_YAML.read_text())
    for obj_id, obj in (disp.get("estimated_collision_objects") or {}).items():
        if obj.get("type") != "box":
            continue
        dcx, dcy, dcz = obj["center_xyz_m"]
        dsx, dsy, dsz = obj["size_xyz_m"]
        markers += make_box_marker(mid, "dispenser_collision",
            dcx, dcy, dcz, dsx, dsy, dsz,
            1.0, 0.35, 0.05, 0.70, stamp, obj_id)
        mid += 1

    # ── 4. 디스펜서 front-hold 위치 (녹색 구) ──────────────────────────────
    for hold_name, hold in (disp.get("front_hold_poses") or {}).items():
        xyz = hold.get("position_xyz_m")
        if not xyz:
            continue
        s = Marker()
        s.header.frame_id = FRAME_ID
        s.header.stamp = stamp
        s.ns = "dispenser_front_hold"
        s.id = mid
        s.type = Marker.SPHERE
        s.action = Marker.ADD
        s.pose = Pose()
        s.pose.position.x, s.pose.position.y, s.pose.position.z = xyz
        s.pose.orientation.w = 1.0
        s.scale.x = s.scale.y = s.scale.z = 0.03
        s.color.r = 0.0
        s.color.g = 0.95
        s.color.b = 0.3
        s.color.a = 0.9
        markers.append(s)
        mid += 1

        lbl = Marker()
        lbl.header.frame_id = FRAME_ID
        lbl.header.stamp = stamp
        lbl.ns = "dispenser_front_hold_labels"
        lbl.id = mid
        lbl.type = Marker.TEXT_VIEW_FACING
        lbl.action = Marker.ADD
        lbl.pose = Pose()
        lbl.pose.position.x, lbl.pose.position.y = xyz[0], xyz[1]
        lbl.pose.position.z = xyz[2] + 0.06
        lbl.pose.orientation.w = 1.0
        lbl.scale.z = 0.035
        lbl.color.r = 0.0
        lbl.color.g = 0.95
        lbl.color.b = 0.3
        lbl.color.a = 1.0
        lbl.text = hold_name
        markers.append(lbl)
        mid += 1

    return markers


class CollisionScenePublisher(Node):
    def __init__(self):
        super().__init__("collision_scene_rviz_publisher")
        self.pub = self.create_publisher(
            MarkerArray, "/azas/collision_scene/markers", transient_qos(10)
        )
        self.timer = self.create_timer(2.0, self._publish)
        self._publish()
        self.get_logger().info(
            "Publishing collision scene to /azas/collision_scene/markers\n"
            "RViz: Add > MarkerArray > /azas/collision_scene/markers  (Fixed Frame: base_link)"
        )

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.header.frame_id = FRAME_ID
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        markers = [clear] + build_markers(stamp)
        self.pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = CollisionScenePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
