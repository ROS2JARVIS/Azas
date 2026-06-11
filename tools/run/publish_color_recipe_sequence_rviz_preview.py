#!/usr/bin/env python3
"""Publish an RViz-only mirror of run_color_recipe_sequence.py.

This script reads the same color map and recipe inputs used by
run_color_recipe_sequence.py, derives the same physical dispenser order, then
publishes a Path and MarkerArray. It never calls Doosan motion, gripper, camera,
or execution services.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Path as RosPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

ROOT = Path(__file__).resolve().parents[2]
RUN_DIR = ROOT / "tools" / "run"
sys.path.insert(0, str(RUN_DIR))

from run_color_recipe_sequence import (  # noqa: E402
    RECIPE_PATH,
    color_to_dispenser_id,
    load_color_map,
    parse_colors_arg,
    parse_direct_dispenser_sequence,
)

DEFAULT_DISPENSER_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "measured_dispenser_collision.yaml"
DEFAULT_CALIBRATION = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"


XYZ = tuple[float, float, float]
RGBA = tuple[float, float, float, float]


@dataclass(frozen=True)
class Step:
    label: str
    xyz: XYZ
    color: RGBA
    scale: float = 0.028


def point(xyz: XYZ) -> Point:
    return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


def pose(xyz: XYZ) -> Pose:
    msg = Pose()
    msg.position = point(xyz)
    msg.orientation = Quaternion(w=1.0)
    return msg


def add(a: XYZ, b: XYZ) -> XYZ:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def read_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"YAML is not a map: {path}")
    return data


def read_front_holds(path: Path) -> dict[str, XYZ]:
    data = read_yaml(path)
    poses = data.get("front_hold_poses") or {}
    if not isinstance(poses, dict):
        raise ValueError(f"front_hold_poses is missing in {path}")
    result: dict[str, XYZ] = {}
    for dispenser_id in ("1", "2", "3", "4"):
        block = poses.get(f"dispenser_{dispenser_id}") or {}
        xyz = block.get("position_xyz_m")
        if not isinstance(xyz, list) or len(xyz) != 3:
            raise ValueError(f"front_hold_poses.dispenser_{dispenser_id}.position_xyz_m is invalid")
        result[dispenser_id] = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
    return result


def read_press_poses(path: Path) -> dict[str, XYZ]:
    data = read_yaml(path)
    outlets = data.get("dispenser_outlets") or {}
    if not isinstance(outlets, dict):
        raise ValueError(f"dispenser_outlets is missing in {path}")
    result: dict[str, XYZ] = {}
    for dispenser_id in ("1", "2", "3", "4"):
        block = outlets.get(dispenser_id) or {}
        xyz = block.get("press_pose_xyz_m")
        if not isinstance(xyz, list) or len(xyz) != 3:
            raise ValueError(f"dispenser_outlets.{dispenser_id}.press_pose_xyz_m is invalid")
        result[dispenser_id] = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
    return result


def group_consecutive(ids: Iterable[str]) -> list[tuple[str, int]]:
    grouped: list[tuple[str, int]] = []
    for dispenser_id in ids:
        if grouped and grouped[-1][0] == dispenser_id:
            grouped[-1] = (dispenser_id, grouped[-1][1] + 1)
        else:
            grouped.append((dispenser_id, 1))
    return grouped


def derive_sequence(args: argparse.Namespace) -> tuple[list[str], dict[str, str], list[tuple[str, int]]]:
    if args.dispenser_ids.strip():
        sequence = parse_direct_dispenser_sequence(args.dispenser_ids)
        return sequence, {}, []

    color_map = load_color_map(override_json=args.color_map_json)
    if not color_map:
        if not args.allow_missing_color_map_fallback:
            raise RuntimeError("outputs/dispenser_color_map.json is missing/invalid")
        return ["1", "2", "3", "4"], {}, []

    if args.colors.strip():
        color_pumps = parse_colors_arg(args.colors)
    else:
        if not RECIPE_PATH.exists():
            raise RuntimeError(f"recipe is missing: {RECIPE_PATH}")
        recipe = json.loads(RECIPE_PATH.read_text(encoding="utf-8"))
        colors = recipe.get("colors", [])
        pumps = recipe.get("pumps", {})
        color_pumps = [(str(color).lower(), int(pumps.get(color, 1))) for color in colors]

    sequence: list[str] = []
    for color, count in color_pumps:
        dispenser_id = color_to_dispenser_id(color, color_map)
        if dispenser_id is None:
            raise RuntimeError(f"color {color!r} is not present in color map {color_map}")
        sequence.extend([dispenser_id] * count)
    if not sequence:
        raise RuntimeError("derived dispenser sequence is empty")
    return sequence, color_map, color_pumps


def build_steps(args: argparse.Namespace, sequence: list[str]) -> list[Step]:
    front_holds = read_front_holds(args.dispenser_config)
    press_poses = read_press_poses(args.calibration)

    steps: list[Step] = []
    grouped = group_consecutive(sequence)
    for group_index, (dispenser_id, press_count) in enumerate(grouped, start=1):
        hold = front_holds[dispenser_id]
        press = press_poses[dispenser_id]
        release = add(
            hold,
            (
                args.move_release_offset_x_m,
                args.move_release_offset_y_m,
                args.move_release_offset_z_m,
            ),
        )
        prehold = add(hold, (args.move_prehold_offset_x_m, args.move_prehold_offset_y_m, args.move_prehold_offset_z_m))
        above = add(hold, (0.0, 0.0, args.move_prehold_offset_z_m))
        press_retreat = add(release, (args.press_pre_lift_retreat_x_m, args.press_pre_lift_retreat_y_m, 0.0))
        empty_lift = (
            press_retreat[0],
            press_retreat[1],
            max(args.press_min_transit_z_m, release[2] + args.press_transit_height_m),
        )
        press_ready = (press[0], press[1], press[2] + args.press_pre_lift_m)
        press_down = (press[0], press[1], press[2] - args.press_depth_m)
        post_press_lift = (
            press[0],
            press[1],
            max(args.regrasp_min_transit_z_m, press[2] + args.press_pre_lift_m),
        )
        post_press_retreat = add(
            post_press_lift,
            (args.regrasp_retreat_x_m, args.regrasp_retreat_y_m, 0.0),
        )
        rear_high = (
            release[0] + args.regrasp_rear_entry_offset_x_m,
            release[1] + args.regrasp_rear_entry_offset_y_m,
            min(max(args.regrasp_min_transit_z_m, release[2] + args.regrasp_approach_offset_z_m), args.regrasp_max_transit_z_m),
        )
        rear_low = (
            release[0] + args.regrasp_rear_entry_offset_x_m,
            release[1] + args.regrasp_rear_entry_offset_y_m,
            release[2],
        )
        regrasp_lift = (release[0], release[1], release[2] + args.pick_lift_m)

        prefix = f"G{group_index} D{dispenser_id}x{press_count}"
        steps.extend(
            [
                Step(f"{prefix} prehold", prehold, (0.1, 0.55, 1.0, 1.0)),
                Step(f"{prefix} above_front_hold", above, (0.1, 0.75, 1.0, 1.0)),
                Step(f"{prefix} RELEASE front_hold exact", release, (0.0, 1.0, 0.25, 1.0), 0.04),
                Step(f"{prefix} X- retreat before press lift", press_retreat, (1.0, 0.9, 0.1, 1.0)),
                Step(f"{prefix} safe lift", empty_lift, (1.0, 0.9, 0.1, 1.0)),
                Step(f"{prefix} press ready", press_ready, (1.0, 0.4, 0.1, 1.0)),
                Step(f"{prefix} press down", press_down, (1.0, 0.0, 0.0, 1.0), 0.035),
                Step(f"{prefix} lift after press", post_press_lift, (1.0, 0.4, 0.1, 1.0)),
                Step(f"{prefix} X- retreat then RG2 open", post_press_retreat, (1.0, 0.8, 0.0, 1.0), 0.04),
                Step(f"{prefix} rear high", rear_high, (0.7, 0.2, 1.0, 1.0)),
                Step(f"{prefix} rear low", rear_low, (0.7, 0.2, 1.0, 1.0)),
                Step(f"{prefix} forward regrasp front_hold", release, (0.0, 1.0, 0.9, 1.0), 0.04),
                Step(f"{prefix} regrasp lift", regrasp_lift, (0.0, 0.8, 1.0, 1.0)),
            ]
        )
    return steps


class ColorRecipePreviewPublisher(Node):
    def __init__(self, args: argparse.Namespace, steps: list[Step], sequence: list[str], color_map: dict[str, str], color_pumps: list[tuple[str, int]]) -> None:
        super().__init__("azas_color_recipe_sequence_rviz_preview")
        self.args = args
        self.steps = steps
        self.sequence = sequence
        self.color_map = color_map
        self.color_pumps = color_pumps
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(RosPath, "/azas/dispenser_sequence/plan", qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/azas/dispenser_sequence/markers", qos)
        self.create_timer(1.0 / max(args.publish_rate_hz, 0.2), self.publish_preview)
        self.get_logger().info(
            "RViz mirror of run_color_recipe_sequence.py ready; no robot/gripper/camera commands are sent"
        )
        self.get_logger().info(f"sequence={','.join(sequence)} grouped={group_consecutive(sequence)}")

    def publish_preview(self) -> None:
        stamp = self.get_clock().now().to_msg()
        path = RosPath()
        path.header.frame_id = self.args.frame_id
        path.header.stamp = stamp
        for step in self.steps:
            msg = PoseStamped()
            msg.header = path.header
            msg.pose = pose(step.xyz)
            path.poses.append(msg)
        self.path_pub.publish(path)
        self.marker_pub.publish(self.make_markers(stamp))

    def make_markers(self, stamp) -> MarkerArray:
        markers: list[Marker] = []
        clear = Marker()
        clear.header.frame_id = self.args.frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        markers.append(clear)

        line = Marker()
        line.header.frame_id = self.args.frame_id
        line.header.stamp = stamp
        line.ns = "actual_color_recipe_path"
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.018
        line.color.r = 1.0
        line.color.g = 0.9
        line.color.b = 0.0
        line.color.a = 1.0
        line.points = [point(step.xyz) for step in self.steps]
        markers.append(line)

        for idx, step in enumerate(self.steps, start=10):
            markers.append(self.sphere_marker(idx, step, stamp))
            markers.append(self.text_marker(idx + 10000, step, stamp))
        markers.append(self.command_text_marker(90000, stamp))
        return MarkerArray(markers=markers)

    def sphere_marker(self, marker_id: int, step: Step, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.args.frame_id
        marker.header.stamp = stamp
        marker.ns = "actual_color_recipe_steps"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose(step.xyz)
        marker.scale = Vector3(x=step.scale, y=step.scale, z=step.scale)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = step.color
        return marker

    def text_marker(self, marker_id: int, step: Step, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.args.frame_id
        marker.header.stamp = stamp
        marker.ns = "actual_color_recipe_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = pose((step.xyz[0], step.xyz[1], step.xyz[2] + 0.045))
        marker.scale.z = 0.032
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = step.label
        return marker

    def command_text_marker(self, marker_id: int, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.args.frame_id
        marker.header.stamp = stamp
        marker.ns = "actual_color_recipe_command"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = pose((0.43, 0.20, 0.62))
        marker.scale.z = 0.04
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.3
        marker.color.a = 1.0
        marker.text = (
            "RViz mirror: run_color_recipe_sequence.py --execute --confirm | "
            f"sequence={','.join(self.sequence)}"
        )
        return marker


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--colors", default="")
    parser.add_argument("--dispenser-ids", default="")
    parser.add_argument("--color-map-json", default="")
    parser.add_argument("--allow-missing-color-map-fallback", action="store_true")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--dispenser-config", type=Path, default=DEFAULT_DISPENSER_CONFIG)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--publish-rate-hz", type=float, default=5.0)
    parser.add_argument("--move-prehold-offset-x-m", type=float, default=-0.030)
    parser.add_argument("--move-prehold-offset-y-m", type=float, default=0.0)
    parser.add_argument("--move-prehold-offset-z-m", type=float, default=0.180)
    parser.add_argument("--move-release-offset-x-m", type=float, default=0.0)
    parser.add_argument("--move-release-offset-y-m", type=float, default=0.0)
    parser.add_argument("--move-release-offset-z-m", type=float, default=0.0)
    parser.add_argument("--press-pre-lift-retreat-x-m", type=float, default=-0.050)
    parser.add_argument("--press-pre-lift-retreat-y-m", type=float, default=0.0)
    parser.add_argument("--press-min-transit-z-m", type=float, default=0.500)
    parser.add_argument("--press-transit-height-m", type=float, default=0.080)
    parser.add_argument("--press-pre-lift-m", type=float, default=0.080)
    parser.add_argument("--press-depth-m", type=float, default=0.060)
    parser.add_argument("--regrasp-min-transit-z-m", type=float, default=0.500)
    parser.add_argument("--regrasp-approach-offset-z-m", type=float, default=0.250)
    parser.add_argument("--regrasp-max-transit-z-m", type=float, default=0.560)
    parser.add_argument("--regrasp-retreat-x-m", type=float, default=-0.080)
    parser.add_argument("--regrasp-retreat-y-m", type=float, default=0.0)
    parser.add_argument("--regrasp-rear-entry-offset-x-m", type=float, default=-0.080)
    parser.add_argument("--regrasp-rear-entry-offset-y-m", type=float, default=0.0)
    parser.add_argument("--pick-lift-m", type=float, default=0.100)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        sequence, color_map, color_pumps = derive_sequence(args)
        steps = build_steps(args, sequence)
    except Exception as exc:
        print(f"[FAIL] cannot build RViz preview: {exc}", file=sys.stderr)
        return 1

    print("[Azas] RViz mirror for exact color recipe command")
    print("[Azas] no robot/gripper/camera services will be called")
    print(f"[Azas] color_map={color_map if color_map else 'direct dispenser input'}")
    if color_pumps:
        print(f"[Azas] recipe colors+pumps={color_pumps}")
    print(f"[Azas] dispenser_ids={','.join(sequence)}")
    print("[Azas] publishing /azas/dispenser_sequence/plan and /azas/dispenser_sequence/markers")

    rclpy.init()
    node = ColorRecipePreviewPublisher(args, steps, sequence, color_map, color_pumps)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
