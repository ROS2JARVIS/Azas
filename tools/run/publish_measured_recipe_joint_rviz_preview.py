#!/usr/bin/env python3
"""RViz-only measured joint preview for the cocktail dispenser recipe.

This publishes /joint_states from calibration.yaml DISP/PRESS joint pairs.  It
does not call Doosan motion, gripper, camera, or execution services.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
    parse_recipe_data,
)

INVALID_PRESS_CONTACT_STATUSES = {
    "invalid",
    "invalid_reteach_required",
    "needs_reteach",
    "reteach_required",
    "확인 필요",
}

CALIBRATION = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


@dataclass(frozen=True)
class Keyframe:
    label: str
    joints_deg: list[float]
    hold_frames: int = 8


def read_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML is not a map: {path}")
    return data


def numeric_list(raw: object, label: str, size: int) -> list[float]:
    if not isinstance(raw, list) or len(raw) != size:
        raise ValueError(f"{label} must be a {size}-item list")
    return [float(value) for value in raw]


def group_consecutive(ids: list[str]) -> list[tuple[str, int]]:
    grouped: list[tuple[str, int]] = []
    for dispenser_id in ids:
        if grouped and grouped[-1][0] == dispenser_id:
            grouped[-1] = (dispenser_id, grouped[-1][1] + 1)
        else:
            grouped.append((dispenser_id, 1))
    return grouped


def derive_sequence(args: argparse.Namespace) -> tuple[list[str], dict[str, str], list[tuple[str, int]]]:
    if args.dispenser_ids.strip():
        return parse_direct_dispenser_sequence(args.dispenser_ids), {}, []

    color_map = load_color_map(override_json=args.color_map_json)
    if not color_map:
        raise RuntimeError("color map is missing/invalid; use --dispenser-ids for physical-number preview")

    if args.colors.strip():
        color_pumps = parse_colors_arg(args.colors)
    else:
        if not RECIPE_PATH.exists():
            raise RuntimeError(f"recipe is missing: {RECIPE_PATH}")
        color_pumps = parse_recipe_data(json.loads(RECIPE_PATH.read_text(encoding="utf-8")))

    sequence: list[str] = []
    for color, count in color_pumps:
        dispenser_id = color_to_dispenser_id(color, color_map)
        if dispenser_id is None:
            raise RuntimeError(f"color {color!r} is not present in color map {color_map}")
        sequence.extend([dispenser_id] * count)
    if not sequence:
        raise RuntimeError("derived dispenser sequence is empty")
    return sequence, color_map, color_pumps


def load_outlet_joints(calibration: Path, dispenser_id: str) -> dict[str, list[float]]:
    data = read_yaml(calibration)
    outlet = (data.get("dispenser_outlets") or {}).get(dispenser_id)
    if not isinstance(outlet, dict):
        raise ValueError(f"dispenser_outlets.{dispenser_id} is missing in {calibration}")
    status = str(outlet.get("press_contact_status", "")).strip()
    if status.lower() in INVALID_PRESS_CONTACT_STATUSES:
        raise ValueError(
            f"dispenser_outlets.{dispenser_id}.press_contact_joints_deg is marked "
            f"{status!r}; refusing to preview stale PRESS{dispenser_id}_CONTACT"
        )
    return {
        "cup_pre": numeric_list(outlet.get("cup_pre_place_joints_deg"), f"D{dispenser_id} cup_pre_place_joints_deg", 6),
        "cup_place": numeric_list(outlet.get("cup_place_joints_deg"), f"D{dispenser_id} cup_place_joints_deg", 6),
        "press_contact": numeric_list(outlet.get("press_contact_joints_deg"), f"D{dispenser_id} press_contact_joints_deg", 6),
    }


def build_keyframes(args: argparse.Namespace, sequence: list[str]) -> list[Keyframe]:
    keyframes: list[Keyframe] = []
    if args.include_home:
        keyframes.append(Keyframe("HOME start", [0.0, 0.0, 90.0, 0.0, 90.0, 0.0], args.hold_frames))

    for group_index, (dispenser_id, count) in enumerate(group_consecutive(sequence), start=1):
        joints = load_outlet_joints(args.calibration, dispenser_id)
        prefix = f"G{group_index} D{dispenser_id}x{count}"
        if args.preview_mode == "press-only":
            keyframes.append(
                Keyframe(
                    f"{prefix} PRESS_CONTACT measured; real PRE=CONTACT+Z, PRESS=CONTACT-Z",
                    joints["press_contact"],
                    args.hold_frames,
                )
            )
            for press_index in range(1, count + 1):
                suffix = f"{press_index}/{count}" if count > 1 else "1/1"
                keyframes.append(
                    Keyframe(f"{prefix} PRESS_CONTACT touch {suffix}", joints["press_contact"], args.press_hold_frames)
                )
                keyframes.append(
                    Keyframe(
                        f"{prefix} PRESS_EXTRA_Z visual note {suffix}",
                        joints["press_contact"],
                        args.press_hold_frames,
                    )
                )
                keyframes.append(
                    Keyframe(
                        f"{prefix} GENERATED_PRESS_PRE visual note {suffix}",
                        joints["press_contact"],
                        args.hold_frames,
                    )
                )
            continue

        keyframes.extend(
            [
                Keyframe(f"{prefix} DISP_PRE cup approach", joints["cup_pre"], args.hold_frames),
                Keyframe(f"{prefix} DISP_PLACE cup release", joints["cup_place"], args.release_hold_frames),
                Keyframe(f"{prefix} SAFE after release: RG2 open then close empty", joints["cup_pre"], args.hold_frames),
                Keyframe(
                    f"{prefix} PRESS_CONTACT measured; real PRE=CONTACT+Z",
                    joints["press_contact"],
                    args.hold_frames,
                ),
            ]
        )
        for press_index in range(1, count + 1):
            suffix = f"{press_index}/{count}" if count > 1 else "1/1"
            keyframes.append(Keyframe(f"{prefix} PRESS_CONTACT touch {suffix}", joints["press_contact"], args.press_hold_frames))
            keyframes.append(Keyframe(f"{prefix} PRESS_EXTRA_Z visual note {suffix}", joints["press_contact"], args.press_hold_frames))
            keyframes.append(Keyframe(f"{prefix} GENERATED_PRESS_PRE visual note {suffix}", joints["press_contact"], args.hold_frames))
        keyframes.extend(
            [
                Keyframe(f"{prefix} SAFE robot-side retreat: RG2 opens here", joints["press_contact"], args.release_hold_frames),
                Keyframe(f"{prefix} DISP_PRE re-grasp approach with RG2 already open", joints["cup_pre"], args.hold_frames),
                Keyframe(f"{prefix} DISP_PLACE side grasp", joints["cup_place"], args.release_hold_frames),
                Keyframe(f"{prefix} DISP_PRE lift after grasp", joints["cup_pre"], args.hold_frames),
            ]
        )
    return keyframes


def interpolate_deg(start: list[float], end: list[float], ratio: float) -> list[float]:
    smooth = 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(ratio, 1.0)))
    return [a + (b - a) * smooth for a, b in zip(start, end)]


class MeasuredJointPreview(Node):
    def __init__(self, args: argparse.Namespace, keyframes: list[Keyframe], sequence: list[str]) -> None:
        super().__init__("azas_measured_recipe_joint_rviz_preview")
        self.args = args
        self.keyframes = keyframes
        self.sequence = sequence
        self.segment = 0
        self.frame = 0
        self.js_pub = self.create_publisher(JointState, args.joint_state_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, args.marker_topic, 10)
        self.create_timer(1.0 / max(args.publish_rate_hz, 1.0), self.publish)
        self.get_logger().info("RViz-only measured joint preview publishing; no robot services are called")
        self.get_logger().info(f"sequence={','.join(sequence)} grouped={group_consecutive(sequence)}")

    def publish(self) -> None:
        if not self.keyframes:
            return
        start = self.keyframes[self.segment]
        end = self.keyframes[min(self.segment + 1, len(self.keyframes) - 1)]
        frames = max(self.args.frames_per_segment, 1)
        ratio = self.frame / frames
        joints = interpolate_deg(start.joints_deg, end.joints_deg, ratio)

        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = JOINT_NAMES
        msg.position = [math.radians(value) for value in joints]
        self.js_pub.publish(msg)
        self.marker_pub.publish(self.make_label_marker(stamp, end.label, joints))

        self.frame += 1
        if self.frame > frames + max(end.hold_frames, 0):
            self.frame = 0
            self.segment += 1
            if self.segment >= len(self.keyframes) - 1:
                self.segment = 0 if self.args.loop else len(self.keyframes) - 2

    def make_label_marker(self, stamp, label: str, joints: list[float]) -> MarkerArray:
        clear = Marker()
        clear.header.frame_id = self.args.frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL

        text = Marker()
        text.header.frame_id = self.args.frame_id
        text.header.stamp = stamp
        text.ns = "measured_joint_recipe_preview"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 0.48
        text.pose.position.y = -0.36
        text.pose.position.z = 0.72
        text.pose.orientation.w = 1.0
        text.scale.z = 0.035
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f"{label}\nsequence={','.join(self.sequence)}\njoints_deg={[round(v, 1) for v in joints]}"
        return MarkerArray(markers=[clear, text])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Preview measured dispenser recipe joints in RViz only.")
    parser.add_argument("--colors", default="")
    parser.add_argument("--dispenser-ids", default="")
    parser.add_argument("--color-map-json", default="")
    parser.add_argument("--calibration", type=Path, default=CALIBRATION)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--marker-topic", default="/azas/measured_joint_preview/markers")
    parser.add_argument("--publish-rate-hz", type=float, default=30.0)
    parser.add_argument("--frames-per-segment", type=int, default=45)
    parser.add_argument("--hold-frames", type=int, default=12)
    parser.add_argument("--release-hold-frames", type=int, default=30)
    parser.add_argument("--press-hold-frames", type=int, default=18)
    parser.add_argument("--loop", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--include-home", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--preview-mode", choices=["full", "press-only"], default="full")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sequence, color_map, color_pumps = derive_sequence(args)
    print(f"[Azas] RViz measured joint preview sequence={','.join(sequence)} grouped={group_consecutive(sequence)}")
    if color_map:
        print(f"[Azas] color_map={color_map}")
        print(f"[Azas] color_pumps={color_pumps}")
    keyframes = build_keyframes(args, sequence)
    print(f"[Azas] keyframes={len(keyframes)}")
    rclpy.init()
    node = MeasuredJointPreview(args, keyframes, sequence)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
