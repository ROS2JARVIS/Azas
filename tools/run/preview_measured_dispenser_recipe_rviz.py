#!/usr/bin/env python3
"""RViz-only preview of measured cocktail dispenser recipe joints.

This script reads joint teaching values from calibration.yaml and publishes only
sensor_msgs/JointState so RViz can animate the M0609 RobotModel in sequence.
It never calls real robot services, MoveJoint, MoveLine, gripper services, or
MoveIt execution.  SAFE_LIFT and PRESS_Z_OVERDRIVE_40MM are logged as real-code
TCP linear motions, but this preview holds the measured joint pose for safety.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

DEFAULT_CALIBRATION = Path("/home/ssu/Azas/src/azas_bringup/config/calibration.yaml")
DEFAULT_JOINT_NAMES = "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6"
ALLOWED_DISPENSER_IDS = {"1", "2", "3", "4"}
JOINT_COUNT = 6
DEFAULT_JOINT_VELOCITY_DEG_S = 40.0
DEFAULT_PRESS_DEPTH_M = 0.040

STAGES = (
    "DISP_PRE",
    "DISP_PLACE",
    "RELEASE",
    "SAFE_LIFT",
    "PRESS_PRE",
    "PRESS_CONTACT",
    "PRESS_Z_OVERDRIVE",
    "PRESS_CONTACT_RETURN",
    "PRESS_PRE_RETURN",
    "DISP_PRE_REGRASP",
    "DISP_PLACE_REGRASP",
)


def log(message: str) -> None:
    print(message, flush=True)


@dataclass(frozen=True)
class Waypoint:
    order: int
    dispenser_id: str
    stage: str
    joints_deg: list[float]
    note: str = ""
    taught_joints_deg: list[float] | None = None

    @property
    def source_joints_deg(self) -> list[float]:
        return self.taught_joints_deg if self.taught_joints_deg is not None else self.joints_deg


def parse_dispenser_ids(raw: str) -> list[str]:
    values: list[str] = []
    for part in raw.replace(";", ",").split(","):
        item = part.strip().lower()
        if not item:
            continue
        if "x" in item:
            dispenser_id, count_raw = item.split("x", 1)
        elif ":" in item:
            dispenser_id, count_raw = item.split(":", 1)
        else:
            dispenser_id, count_raw = item, "1"
        dispenser_id = dispenser_id.strip()
        try:
            count = int(count_raw.strip())
        except ValueError as exc:
            raise ValueError(f"invalid count for dispenser {dispenser_id}: {count_raw!r}") from exc
        if count < 1:
            raise ValueError(f"count must be >= 1 for dispenser {dispenser_id}")
        values.extend([dispenser_id] * count)

    if not values:
        raise ValueError("at least one dispenser id is required")

    invalid = [value for value in values if value not in ALLOWED_DISPENSER_IDS]
    if invalid:
        allowed = ",".join(sorted(ALLOWED_DISPENSER_IDS))
        raise ValueError(f"unsupported dispenser id(s): {', '.join(invalid)}; allowed: {allowed}")
    return values


def parse_joint_names(raw: str) -> list[str]:
    names = [item.strip() for item in raw.split(",") if item.strip()]
    if len(names) != JOINT_COUNT:
        raise ValueError(f"--joint-names must provide exactly {JOINT_COUNT} names")
    return names


def parse_joint_index_set(raw: str) -> set[int]:
    indexes: set[int] = set()
    for part in raw.replace(";", ",").split(","):
        item = part.strip().lower()
        if not item:
            continue
        if item.startswith("joint_"):
            item = item[6:]
        elif item.startswith("j"):
            item = item[1:]
        try:
            index = int(item)
        except ValueError as exc:
            raise ValueError(f"--press-lock-contact-joints contains a non-joint value: {part!r}") from exc
        if not 1 <= index <= JOINT_COUNT:
            raise ValueError(f"--press-lock-contact-joints joint index must be 1..{JOINT_COUNT}, got {part!r}")
        indexes.add(index - 1)
    return indexes


def read_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a map: {path}")
    return data


def numeric_joints(raw: object, label: str) -> list[float]:
    if not isinstance(raw, list) or len(raw) != JOINT_COUNT:
        raise ValueError(f"{label} must be a {JOINT_COUNT}-item list")
    try:
        return [float(value) for value in raw]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain numeric degree values") from exc


def load_outlet_joints(calibration: dict, dispenser_id: str) -> dict[str, list[float]]:
    outlets = calibration.get("dispenser_outlets")
    if not isinstance(outlets, dict):
        raise ValueError("calibration.yaml is missing dispenser_outlets")

    outlet = outlets.get(dispenser_id)
    if not isinstance(outlet, dict):
        raise ValueError(f"dispenser_outlets.{dispenser_id} is missing")

    required_fields = (
        "cup_pre_place_joints_deg",
        "cup_place_joints_deg",
        "press_pre_joints_deg",
        "press_contact_joints_deg",
    )
    joints: dict[str, list[float]] = {}
    for field in required_fields:
        joints[field] = numeric_joints(
            outlet.get(field),
            f"dispenser_outlets.{dispenser_id}.{field}",
        )
    return joints


def lock_contact_joints_to_pre(
    press_contact: list[float],
    press_pre: list[float],
    locked_joint_indexes: set[int],
) -> tuple[list[float], str]:
    if not locked_joint_indexes:
        return list(press_contact), ""
    locked = list(press_contact)
    labels: list[str] = []
    for index in sorted(locked_joint_indexes):
        original = locked[index]
        locked[index] = press_pre[index]
        labels.append(f"joint_{index + 1} {original:.2f}->{locked[index]:.2f}")
    return locked, "press-lock-contact-joints: " + ", ".join(labels)


def build_waypoints(
    calibration: dict,
    dispenser_ids: list[str],
    locked_press_contact_joints: set[int],
    press_overdrive_mm: float,
) -> list[Waypoint]:
    waypoints: list[Waypoint] = []
    for order, dispenser_id in enumerate(dispenser_ids, start=1):
        joints = load_outlet_joints(calibration, dispenser_id)
        cup_pre = joints["cup_pre_place_joints_deg"]
        cup_place = joints["cup_place_joints_deg"]
        press_pre = joints["press_pre_joints_deg"]
        press_contact, press_contact_note = lock_contact_joints_to_pre(
            joints["press_contact_joints_deg"],
            press_pre,
            locked_press_contact_joints,
        )

        waypoints.extend(
            [
                Waypoint(order, dispenser_id, "DISP_PRE", cup_pre),
                Waypoint(order, dispenser_id, "DISP_PLACE", cup_place),
                Waypoint(order, dispenser_id, "RELEASE", cup_place, "preview holds DISP_PLACE"),
                Waypoint(
                    order,
                    dispenser_id,
                    "SAFE_LIFT",
                    cup_place,
                    "real code has safe_lift_current TCP MoveLine; preview holds DISP_PLACE",
                ),
                Waypoint(order, dispenser_id, "PRESS_PRE", press_pre),
                Waypoint(order, dispenser_id, "PRESS_CONTACT", press_contact, press_contact_note),
                # TODO: replace this hold with a preview-only FK/IK joint state for
                # the 40 mm TCP-Z overdrive once a non-hardware kinematics path is
                # available.
                Waypoint(
                    order,
                    dispenser_id,
                    f"PRESS_Z_OVERDRIVE_{press_overdrive_mm:.0f}MM",
                    press_contact,
                    (
                        f"real code uses TCP Z MoveLine {press_overdrive_mm:.1f}mm; "
                        "preview holds PRESS_CONTACT"
                    ),
                ),
                Waypoint(order, dispenser_id, "PRESS_CONTACT_RETURN", press_contact),
                Waypoint(order, dispenser_id, "PRESS_PRE_RETURN", press_pre),
                Waypoint(order, dispenser_id, "DISP_PRE_REGRASP", cup_pre),
                Waypoint(order, dispenser_id, "DISP_PLACE_REGRASP", cup_place),
            ]
        )
    return waypoints


def nearest_equivalent_deg(target: float, current: float) -> float:
    candidates = [target + 360.0 * offset for offset in range(-2, 3)]
    bounded = [candidate for candidate in candidates if abs(candidate) <= 360.0]
    if bounded:
        candidates = bounded
    return min(candidates, key=lambda candidate: abs(candidate - current))


def unwrap_waypoints(waypoints: list[Waypoint]) -> list[Waypoint]:
    if not waypoints:
        return []

    unwrapped: list[Waypoint] = []
    previous = waypoints[0].joints_deg
    unwrapped.append(waypoints[0])
    for waypoint in waypoints[1:]:
        adjusted = [
            nearest_equivalent_deg(target, current)
            for target, current in zip(waypoint.joints_deg, previous)
        ]
        previous = adjusted
        unwrapped.append(
            Waypoint(
                order=waypoint.order,
                dispenser_id=waypoint.dispenser_id,
                stage=waypoint.stage,
                joints_deg=adjusted,
                note=waypoint.note,
                taught_joints_deg=waypoint.joints_deg,
            )
        )
    return unwrapped


def interpolate_deg(start: list[float], end: list[float], ratio: float) -> list[float]:
    ratio = max(0.0, min(1.0, ratio))
    return [a + (b - a) * ratio for a, b in zip(start, end)]


def format_joints(values: list[float]) -> str:
    return "[" + ", ".join(f"{value:.2f}" for value in values) + "]"


class JointStatePreviewNode(Node):
    def __init__(
        self,
        waypoints: list[Waypoint],
        joint_names: list[str],
        joint_topic: str,
        marker_topic: str,
        rate_hz: float,
        segment_seconds: float,
        joint_velocity_deg_s: float,
        hold_seconds: float,
        loop: bool,
    ) -> None:
        super().__init__("preview_measured_dispenser_recipe_rviz")
        self._publisher = self.create_publisher(JointState, joint_topic, 10)
        self._marker_publisher = self.create_publisher(MarkerArray, marker_topic, 10) if marker_topic else None
        self._waypoints = waypoints
        self._joint_names = joint_names
        self._marker_topic = marker_topic
        self._loop = loop
        self._rate_hz = rate_hz
        self._segment_seconds = segment_seconds
        self._joint_velocity_deg_s = joint_velocity_deg_s
        self._segment_frames = self._segment_frame_count(0, loop_to_start=False)
        self._hold_frames = max(1, int(round(hold_seconds * rate_hz)))
        self._index = 0
        self._phase = "hold"
        self._phase_frame = 0
        self._looping_to_start = False
        self._announced_current = False
        self._done = False
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        log("[Azas] RViz-only measured dispenser recipe preview")
        log("[Azas] publishing JointState only; service_calls=none")
        log("[Azas] this process does not open RViz; use tools/run/show_measured_recipe_joint_preview_rviz.sh to open RViz too")
        log(f"[Azas] joint_topic={joint_topic}")
        if marker_topic:
            log(f"[Azas] marker_topic={marker_topic}")
        log(f"[Azas] joint_names={','.join(joint_names)}")
        if joint_velocity_deg_s > 0.0:
            log(f"[Azas] preview_joint_velocity_deg_s={joint_velocity_deg_s:.2f}")
        else:
            log(f"[Azas] preview_fixed_segment_seconds={segment_seconds:.2f}")
        log(
            "[Azas] sequence="
            + " -> ".join(STAGES)
        )

    @property
    def done(self) -> bool:
        return self._done

    def _tick(self) -> None:
        if self._phase == "hold":
            waypoint = self._waypoints[self._index]
            if not self._announced_current:
                self._announce_stage(waypoint)
                self._announced_current = True
            self._publish(waypoint.joints_deg)
            self._phase_frame += 1
            if self._phase_frame >= self._hold_frames:
                self._phase_frame = 0
                if self._index >= len(self._waypoints) - 1:
                    if self._loop:
                        self._phase = "segment"
                        self._looping_to_start = True
                        self._segment_frames = self._segment_frame_count(self._index, loop_to_start=True)
                    else:
                        log("[Azas] preview_complete")
                        self._done = True
                        self._timer.cancel()
                else:
                    self._phase = "segment"
                    self._segment_frames = self._segment_frame_count(self._index, loop_to_start=False)
            return

        target_index = 0 if self._looping_to_start else self._index + 1
        start = self._waypoints[self._index].joints_deg
        end = self._waypoints[target_index].joints_deg
        ratio = (self._phase_frame + 1) / self._segment_frames
        self._publish(interpolate_deg(start, end, ratio))
        self._phase_frame += 1
        if self._phase_frame >= self._segment_frames:
            self._index = target_index
            self._phase = "hold"
            self._phase_frame = 0
            self._looping_to_start = False
            self._announced_current = False

    def _segment_frame_count(self, start_index: int, *, loop_to_start: bool) -> int:
        if self._joint_velocity_deg_s <= 0.0:
            seconds = self._segment_seconds
        else:
            target_index = 0 if loop_to_start else min(start_index + 1, len(self._waypoints) - 1)
            start = self._waypoints[start_index].joints_deg
            end = self._waypoints[target_index].joints_deg
            max_delta_deg = max(abs(a - b) for a, b in zip(start, end))
            seconds = max_delta_deg / self._joint_velocity_deg_s if max_delta_deg > 0.0 else 0.0
            seconds = max(seconds, 1.0 / self._rate_hz)
        return max(1, int(math.ceil(seconds * self._rate_hz)))

    def _announce_stage(self, waypoint: Waypoint) -> None:
        source = waypoint.source_joints_deg
        published = waypoint.joints_deg
        suffix = ""
        if any(abs(a - b) > 1e-6 for a, b in zip(source, published)):
            suffix = f" publish_joint_deg={format_joints(published)}"
        log(
            f"[Azas] dispenser={waypoint.dispenser_id} "
            f"cycle={waypoint.order} stage={waypoint.stage} "
            f"joint_deg={format_joints(source)}{suffix}"
        )
        if waypoint.note:
            log(f"[Azas] {waypoint.stage} note={waypoint.note}")
        self._publish_stage_marker(waypoint)

    def _publish(self, joints_deg: list[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = [math.radians(value) for value in joints_deg]
        self._publisher.publish(msg)

    def _publish_stage_marker(self, waypoint: Waypoint) -> None:
        if self._marker_publisher is None:
            return
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "measured_joint_preview_stage"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.42
        marker.pose.position.y = -0.42
        marker.pose.position.z = 0.72
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.055
        marker.color.r = 0.92
        marker.color.g = 0.96
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"D{waypoint.dispenser_id} cycle {waypoint.order}\\n{waypoint.stage}"
        if waypoint.note:
            marker.text += f"\\n{waypoint.note}"
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self._marker_publisher.publish(marker_array)


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be > 0")
    return parsed


def nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("must be >= 0")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Publish measured dispenser recipe joints as JointState for RViz only. "
            "No robot, gripper, MoveIt execution, MoveJoint, or MoveLine service is called."
        )
    )
    parser.add_argument(
        "--dispenser-ids",
        default="1",
        help="Physical dispenser sequence. Supports values like 1,2,3,4 or 1x2,3.",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=DEFAULT_CALIBRATION,
        help=f"Path to calibration.yaml (default: {DEFAULT_CALIBRATION})",
    )
    parser.add_argument(
        "--rate-hz",
        "--publish-rate-hz",
        dest="rate_hz",
        type=positive_float,
        default=30.0,
    )
    parser.add_argument("--segment-seconds", type=positive_float, default=2.0)
    parser.add_argument(
        "--joint-velocity-deg-s",
        type=nonnegative_float,
        default=DEFAULT_JOINT_VELOCITY_DEG_S,
        help=(
            "Preview interpolation speed in deg/s. Default 40. Set 0 to use "
            "--segment-seconds as a fixed duration for every waypoint transition."
        ),
    )
    parser.add_argument("--hold-seconds", type=nonnegative_float, default=1.0)
    parser.add_argument(
        "--press-lock-contact-joints",
        default="",
        help=(
            "Compatibility with real recipe commands: comma-separated joint numbers "
            "copied from PRESS_PRE into PRESS_CONTACT for preview, e.g. 6."
        ),
    )
    parser.add_argument("--press-depth-m", type=nonnegative_float, default=DEFAULT_PRESS_DEPTH_M)
    parser.add_argument("--press-extra-depth-m", type=nonnegative_float, default=0.0)
    parser.add_argument(
        "--press-use-recorded-pre-joints",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Compatibility no-op: preview always uses calibration.yaml press_pre_joints_deg.",
    )
    parser.add_argument(
        "--press-reset-before-press",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Compatibility no-op: preview never inserts the reset joint pose.",
    )
    parser.add_argument(
        "--safe-lift-joint-fallback",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Compatibility no-op: preview logs SAFE_LIFT but does not call IK/MoveJoint.",
    )
    parser.add_argument(
        "--unwrap-joints",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use nearest equivalent joint angles for smooth RViz interpolation across +/-180/360 boundaries.",
    )
    loop_group = parser.add_mutually_exclusive_group()
    loop_group.add_argument("--loop", dest="loop", action="store_true", help="Repeat the preview until interrupted.")
    loop_group.add_argument("--no-loop", dest="loop", action="store_false", help="Play once and exit.")
    parser.set_defaults(loop=False)
    parser.add_argument("--joint-topic", "--joint-state-topic", dest="joint_topic", default="/joint_states")
    parser.add_argument("--marker-topic", default="/azas/measured_joint_preview/markers")
    parser.add_argument("--joint-names", default=DEFAULT_JOINT_NAMES)
    args, unknown = parser.parse_known_args()
    args.ignored_real_motion_args = unknown
    return args


def main() -> int:
    args = parse_args()
    try:
        dispenser_ids = parse_dispenser_ids(args.dispenser_ids)
        joint_names = parse_joint_names(args.joint_names)
        locked_press_contact_joints = parse_joint_index_set(args.press_lock_contact_joints)
        press_overdrive_mm = (args.press_depth_m + args.press_extra_depth_m) * 1000.0
        calibration = read_yaml(args.calibration)
        waypoints = build_waypoints(
            calibration,
            dispenser_ids,
            locked_press_contact_joints,
            press_overdrive_mm,
        )
        if args.unwrap_joints:
            waypoints = unwrap_waypoints(waypoints)
    except (OSError, ValueError) as exc:
        log(f"[Azas] preview setup failed: {exc}")
        return 2

    if not waypoints:
        log("[Azas] preview setup failed: no waypoints")
        return 2

    rclpy.init()
    node = JointStatePreviewNode(
        waypoints=waypoints,
        joint_names=joint_names,
        joint_topic=args.joint_topic,
        marker_topic=args.marker_topic,
        rate_hz=args.rate_hz,
        segment_seconds=args.segment_seconds,
        joint_velocity_deg_s=args.joint_velocity_deg_s,
        hold_seconds=args.hold_seconds,
        loop=args.loop,
    )
    if args.ignored_real_motion_args:
        log("[Azas] ignored_real_motion_args=" + " ".join(args.ignored_real_motion_args))
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
