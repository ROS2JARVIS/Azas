#!/usr/bin/env python3
"""RViz-only dispenser sequence preview for Azas package structure.

This node is deliberately hardware-free. It subscribes to a base_link cup pose,
builds a readable sequential path, and publishes RViz markers plus a Path for
the optional IK preview node. It does not call MoveIt execution, Doosan
services, gripper services, or camera APIs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path as FsPath
from typing import List, Sequence, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


XYZ = Tuple[float, float, float]
RGBA = Tuple[float, float, float, float]


@dataclass(frozen=True)
class SequenceStep:
    label: str
    xyz: XYZ
    cup_base_xyz: XYZ | None = None


def point(xyz: XYZ) -> Point:
    return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


def quat_identity() -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)


def pose(xyz: XYZ, quat: Quaternion | None = None) -> Pose:
    msg = Pose()
    msg.position = point(xyz)
    msg.orientation = quat if quat is not None else quat_identity()
    return msg


def triples(values: Sequence[float]) -> List[XYZ]:
    if len(values) % 3 != 0:
        raise ValueError("flat XYZ array length must be a multiple of 3")
    return [
        (float(values[i]), float(values[i + 1]), float(values[i + 2]))
        for i in range(0, len(values), 3)
    ]


class DispenserSequencePreviewNode(Node):
    def __init__(self) -> None:
        super().__init__("dispenser_sequence_preview_node")
        self.declare_parameter("cup_pose_topic", "/azas/sim/tumbler_pose")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter(
            "calibration_path",
            "/home/ssu/Azas/src/azas_bringup/config/calibration.yaml",
        )
        self.declare_parameter(
            "dispenser_collision_config_path",
            "/home/ssu/Azas/src/azas_bringup/config/measured_dispenser_collision.yaml",
        )
        self.declare_parameter("selected_dispenser_id", 2)
        self.declare_parameter("cup_height_m", 0.17)
        self.declare_parameter("grasp_height_m", 0.085)
        self.declare_parameter("side_pre_grasp_offset_m", 0.10)
        self.declare_parameter("lift_height_m", 0.04)
        self.declare_parameter("shake_clearance_m", 0.13)
        self.declare_parameter("shake_swing_m", 0.109)
        self.declare_parameter("shake_lift_m", 0.055)
        self.declare_parameter("outlet_mouth_clearance_m", 0.0)
        self.declare_parameter("move_release_offset_x_m", 0.0)
        self.declare_parameter("move_release_offset_y_m", 0.0)
        self.declare_parameter("move_release_offset_z_m", 0.0)
        self.declare_parameter("press_pre_lift_retreat_x_m", -0.050)
        self.declare_parameter("press_pre_lift_retreat_y_m", 0.0)
        self.declare_parameter("regrasp_retreat_x_m", -0.080)
        self.declare_parameter("regrasp_retreat_y_m", 0.0)
        self.declare_parameter("regrasp_rear_entry_offset_x_m", -0.080)
        self.declare_parameter("regrasp_rear_entry_offset_y_m", 0.0)
        self.declare_parameter("regrasp_min_transit_z_m", 0.500)
        self.declare_parameter("regrasp_max_transit_z_m", 0.560)
        self.declare_parameter("publish_rate_hz", 4.0)
        self.declare_parameter("show_sequence_markers", True)
        self.declare_parameter("show_dispenser_markers", True)
        self.declare_parameter("show_animated_cup", True)
        self.declare_parameter("show_demo_arm", True)
        self.declare_parameter("animated_cup_step_hold_ticks", 3)
        self.declare_parameter(
            "dispenser_bottle_positions",
            [
                0.55,
                0.18,
                0.1375,
                0.55,
                0.08,
                0.1375,
                0.55,
                -0.02,
                0.1375,
                0.55,
                -0.12,
                0.1375,
            ],
        )
        self.declare_parameter(
            "dispenser_outlet_positions",
            [
                0.555,
                -0.100,
                0.093,
                0.549,
                -0.150,
                0.097,
                0.527,
                -0.204,
                0.107,
                0.517,
                -0.235,
                0.109,
            ],
        )
        self.calibration = self.load_yaml_config("calibration_path", "measured dispenser sequence calibration")
        self.dispenser_collision = self.load_yaml_config(
            "dispenser_collision_config_path",
            "measured dispenser front-hold collision config",
        )

        plan_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "/azas/dispenser_sequence/plan", plan_qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/azas/dispenser_sequence/markers", 10
        )
        self.robot_arm_pub = self.create_publisher(
            MarkerArray, "/jarvis/robot_arm/markers", 10
        )
        self.target_pub = self.create_publisher(
            PoseStamped, "/azas/dispenser_sequence/target_pose", plan_qos
        )
        self.last_cup_pose: PoseStamped | None = None
        self.preview_step_index = 0
        self.preview_step_tick = 0
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("cup_pose_topic").value),
            self.on_cup_pose,
            10,
        )
        period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 0.2)
        self.create_timer(period, self.publish_preview)
        self.get_logger().info(
            "Azas RViz dispenser sequence preview ready; hardware and camera execution are disabled."
        )

    def load_yaml_config(self, parameter_name: str, label: str) -> dict:
        config_path = FsPath(str(self.get_parameter(parameter_name).value)).expanduser()
        if not config_path.exists():
            self.get_logger().warning(
                f"{parameter_name} does not exist; using launch fallback where available: {config_path}"
            )
            return {}
        with config_path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        if not isinstance(config, dict):
            self.get_logger().warning(
                f"{parameter_name} is not a YAML map; using launch fallback where available: {config_path}"
            )
            return {}
        self.get_logger().info(f"Loaded {label}: {config_path}")
        return config

    def on_cup_pose(self, msg: PoseStamped) -> None:
        expected = str(self.get_parameter("frame_id").value)
        if msg.header.frame_id != expected:
            self.get_logger().error(
                f"Rejected cup pose frame={msg.header.frame_id!r}; expected {expected!r}"
            )
            return
        self.last_cup_pose = msg
        self.preview_step_index = 0
        self.preview_step_tick = 0

    def dispenser_layout(self) -> tuple[List[XYZ], List[XYZ], int]:
        bottles = triples(self.get_parameter("dispenser_bottle_positions").value)
        outlets = self.measured_points("outlet_pose_xyz_m")
        if not outlets:
            outlets = triples(self.get_parameter("dispenser_outlet_positions").value)
        if len(bottles) != len(outlets):
            raise ValueError("bottle/outlet count mismatch")
        selected = min(
            max(int(self.get_parameter("selected_dispenser_id").value), 1),
            len(outlets),
        )
        return bottles, outlets, selected - 1

    def measured_points(self, field_name: str) -> List[XYZ]:
        outlets = self.calibration.get("dispenser_outlets", {})
        if not isinstance(outlets, dict):
            return []
        points: List[XYZ] = []
        for dispenser_id in sorted(outlets.keys(), key=lambda value: int(value)):
            config = outlets.get(dispenser_id) or {}
            if not isinstance(config, dict):
                return []
            xyz = config.get(field_name)
            if not isinstance(xyz, list) or len(xyz) != 3:
                return []
            points.append((float(xyz[0]), float(xyz[1]), float(xyz[2])))
        return points

    def measured_front_holds(self) -> List[XYZ]:
        front_holds = self.dispenser_collision.get("front_hold_poses", {})
        if not isinstance(front_holds, dict):
            return []
        points: List[XYZ] = []
        for dispenser_id in sorted(
            front_holds.keys(),
            key=lambda value: int(str(value).replace("dispenser_", "")),
        ):
            config = front_holds.get(dispenser_id) or {}
            if not isinstance(config, dict):
                return []
            xyz = config.get("position_xyz_m")
            if not isinstance(xyz, list) or len(xyz) != 3:
                return []
            points.append((float(xyz[0]), float(xyz[1]), float(xyz[2])))
        return points

    def build_steps(self, cup_pose: PoseStamped) -> List[SequenceStep]:
        _, outlets, selected_index = self.dispenser_layout()
        press_targets = self.measured_points("press_pose_xyz_m")
        front_holds = self.measured_front_holds()
        cup = cup_pose.pose.position
        cup_base = (float(cup.x), float(cup.y), float(cup.z))
        outlet = outlets[selected_index]
        measured_front_hold = front_holds[selected_index] if len(front_holds) == len(outlets) else outlet
        grasp_height = float(self.get_parameter("grasp_height_m").value)
        side_offset = float(self.get_parameter("side_pre_grasp_offset_m").value)
        lift_height = float(self.get_parameter("lift_height_m").value)
        shake_clearance = float(self.get_parameter("shake_clearance_m").value)
        shake_swing = float(self.get_parameter("shake_swing_m").value)
        shake_lift = float(self.get_parameter("shake_lift_m").value)
        release_dx = float(self.get_parameter("move_release_offset_x_m").value)
        release_dy = float(self.get_parameter("move_release_offset_y_m").value)
        release_dz = float(self.get_parameter("move_release_offset_z_m").value)
        press_retreat_x = float(self.get_parameter("press_pre_lift_retreat_x_m").value)
        press_retreat_y = float(self.get_parameter("press_pre_lift_retreat_y_m").value)
        regrasp_retreat_x = float(self.get_parameter("regrasp_retreat_x_m").value)
        regrasp_retreat_y = float(self.get_parameter("regrasp_retreat_y_m").value)
        rear_entry_x = float(self.get_parameter("regrasp_rear_entry_offset_x_m").value)
        rear_entry_y = float(self.get_parameter("regrasp_rear_entry_offset_y_m").value)
        regrasp_min_z = float(self.get_parameter("regrasp_min_transit_z_m").value)
        regrasp_max_z = float(self.get_parameter("regrasp_max_transit_z_m").value)

        grasp = (cup_base[0], cup_base[1], cup_base[2] + grasp_height)
        side_pre_grasp = (grasp[0], grasp[1] - side_offset, grasp[2])
        low_transfer_z = grasp[2] + lift_height
        lift = (grasp[0], grasp[1], low_transfer_z)
        front_lane = (measured_front_hold[0] - 0.12, grasp[1], low_transfer_z)
        outlet_front_hold = (
            measured_front_hold[0] + release_dx,
            measured_front_hold[1] + release_dy,
            measured_front_hold[2] + release_dz,
        )
        cup_front_base = (
            outlet_front_hold[0],
            outlet_front_hold[1],
            outlet_front_hold[2] - grasp_height,
        )
        press_retreat = (
            outlet_front_hold[0] + press_retreat_x,
            outlet_front_hold[1] + press_retreat_y,
            outlet_front_hold[2],
        )
        empty_lift = (
            press_retreat[0],
            press_retreat[1],
            max(regrasp_min_z, outlet_front_hold[2] + 0.20),
        )
        if len(press_targets) == len(outlets):
            press_down = press_targets[selected_index]
        else:
            press_down = (
                outlet[0] - 0.03,
                outlet[1],
                max(outlet[2] + 0.035, low_transfer_z + 0.12),
            )
        press_ready = (
            press_down[0],
            press_down[1],
            min(max(press_down[2] + 0.12, 0.50), 0.56),
        )
        regrasp_high_z = min(max(regrasp_min_z, outlet_front_hold[2] + 0.25), regrasp_max_z)
        post_press_lift = (
            press_down[0],
            press_down[1],
            regrasp_high_z,
        )
        regrasp_high = (
            outlet_front_hold[0] + regrasp_retreat_x,
            outlet_front_hold[1] + regrasp_retreat_y,
            regrasp_high_z,
        )
        regrasp_rear_low = (
            outlet_front_hold[0] + rear_entry_x,
            outlet_front_hold[1] + rear_entry_y,
            outlet_front_hold[2],
        )
        regrasp = outlet_front_hold
        regrasp_lift = (outlet_front_hold[0], outlet_front_hold[1], low_transfer_z + 0.12)
        shake_z = max(low_transfer_z + shake_clearance, 0.55)
        shake_center = (outlet_front_hold[0] - 0.15, outlet_front_hold[1] - 0.38, shake_z + shake_lift)
        shake_left = (
            shake_center[0],
            shake_center[1] + shake_swing,
            shake_center[2] + shake_lift,
        )
        shake_right = (
            shake_center[0],
            shake_center[1] - shake_swing,
            shake_center[2] - shake_lift * 0.45,
        )
        shake_forward = (
            shake_center[0] + 0.045,
            shake_center[1],
            shake_center[2] + shake_lift * 0.75,
        )
        shake_back = (
            shake_center[0] - 0.035,
            shake_center[1],
            shake_center[2] - shake_lift * 0.35,
        )

        return [
            SequenceStep("1 side_pre_grasp", side_pre_grasp),
            SequenceStep("2 side_grasp", grasp),
            SequenceStep("3 lift_cup", lift),
            SequenceStep("4 carry_to_front_lane", front_lane),
            SequenceStep("5 place_cup_front", outlet_front_hold),
            SequenceStep("6 release_cup_front", outlet_front_hold, cup_front_base),
            SequenceStep("7 retreat_back_before_press_lift", press_retreat, cup_front_base),
            SequenceStep("8 lift_empty_gripper", empty_lift, cup_front_base),
            SequenceStep("9 press_ready", press_ready, cup_front_base),
            SequenceStep("10 press_dispenser", press_down, cup_front_base),
            SequenceStep("11 lift_after_press_open_gripper", post_press_lift, cup_front_base),
            SequenceStep("12 retreat_back_high_after_press", regrasp_high, cup_front_base),
            SequenceStep("13 lower_at_rear_entry", regrasp_rear_low, cup_front_base),
            SequenceStep("14 forward_regrasp_cup", regrasp, cup_front_base),
            SequenceStep("15 regrasp_lift", regrasp_lift),
            SequenceStep("16 shake_center", shake_center),
            SequenceStep("17 shake_left", shake_left),
            SequenceStep("18 shake_right", shake_right),
            SequenceStep("19 shake_forward", shake_forward),
            SequenceStep("20 shake_back", shake_back),
            SequenceStep("21 shake_recenter", shake_center),
        ]

    def publish_preview(self) -> None:
        if self.last_cup_pose is None:
            return
        steps = self.build_steps(self.last_cup_pose)
        now = self.get_clock().now().to_msg()
        frame_id = str(self.get_parameter("frame_id").value)

        path = Path()
        path.header.stamp = now
        path.header.frame_id = frame_id
        for step in steps:
            stamped = PoseStamped()
            stamped.header = path.header
            stamped.pose = pose(step.xyz)
            path.poses.append(stamped)
        self.path_pub.publish(path)
        self.target_pub.publish(path.poses[-2])
        marker_array, robot_arm_array = self.make_markers(steps, now, frame_id)
        self.marker_pub.publish(marker_array)
        self.robot_arm_pub.publish(robot_arm_array)
        self.advance_animation(len(steps))

    def advance_animation(self, step_count: int) -> None:
        if step_count <= 0:
            self.preview_step_index = 0
            self.preview_step_tick = 0
            return
        hold_ticks = max(int(self.get_parameter("animated_cup_step_hold_ticks").value), 1)
        self.preview_step_tick += 1
        if self.preview_step_tick >= hold_ticks:
            self.preview_step_tick = 0
            self.preview_step_index = (self.preview_step_index + 1) % step_count

    def marker(
        self,
        marker_id: int,
        marker_type: int,
        ns: str,
        xyz: XYZ,
        scale: Vector3,
        color: RGBA,
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = str(self.get_parameter("frame_id").value)
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose = pose(xyz)
        marker.scale = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def make_markers(
        self, steps: Sequence[SequenceStep], stamp, frame_id: str
    ) -> tuple[MarkerArray, MarkerArray]:
        markers: List[Marker] = []
        robot_arm_markers: List[Marker] = []
        bottles, outlets, selected_index = self.dispenser_layout()
        active_step = self.active_step(steps)
        if bool(self.get_parameter("show_demo_arm").value) and active_step is not None:
            robot_arm_markers = self.make_demo_arm_markers(active_step)
        if bool(self.get_parameter("show_animated_cup").value):
            markers.extend(self.make_cup_markers(steps))
        if bool(self.get_parameter("show_dispenser_markers").value):
            for index, bottle in enumerate(bottles, start=1):
                selected = index - 1 == selected_index
                markers.append(
                    self.marker(
                        100 + index,
                        Marker.CUBE,
                        "dispenser_bottle",
                        bottle,
                        Vector3(x=0.058, y=0.058, z=0.275),
                        (0.82, 0.96, 1.0, 0.38 if selected else 0.18),
                    )
                )
                outlet = outlets[index - 1]
                markers.append(
                    self.marker(
                        120 + index,
                        Marker.SPHERE,
                        "dispenser_outlet",
                        outlet,
                        Vector3(x=0.024, y=0.024, z=0.024),
                        (1.0, 0.85, 0.0, 1.0 if selected else 0.55),
                    )
                )
                arrow = self.marker(
                    140 + index,
                    Marker.ARROW,
                    "dispenser_faces_robot",
                    (0.0, 0.0, 0.0),
                    Vector3(x=0.012, y=0.020, z=0.020),
                    (1.0, 0.85, 0.0, 1.0 if selected else 0.45),
                )
                arrow.points = [point(outlet), point((outlet[0] - 0.08, outlet[1], outlet[2]))]
                markers.append(arrow)

        if bool(self.get_parameter("show_sequence_markers").value):
            line = self.marker(
                1,
                Marker.LINE_STRIP,
                "sequence_path",
                (0.0, 0.0, 0.0),
                Vector3(x=0.012, y=0.0, z=0.0),
                (0.8, 0.1, 1.0, 1.0),
            )
            line.points = [point(step.xyz) for step in steps]
            markers.append(line)

            for index, step in enumerate(steps, start=1):
                markers.append(
                    self.marker(
                        10 + index,
                        Marker.SPHERE,
                        "sequence_waypoints",
                        step.xyz,
                        Vector3(x=0.026, y=0.026, z=0.026),
                        (0.8, 0.1, 1.0, 1.0),
                    )
                )
                label = self.marker(
                    30 + index,
                    Marker.TEXT_VIEW_FACING,
                    "sequence_labels",
                    (step.xyz[0], step.xyz[1], step.xyz[2] + 0.04),
                    Vector3(x=0.0, y=0.0, z=-0.150),
                    (1.0, 1.0, 1.0, 1.0),
                )
                label.text = step.label
                markers.append(label)

            status = self.marker(
                2,
                Marker.TEXT_VIEW_FACING,
                "preview_status",
                (0.18, 0.32, 0.46),
                Vector3(x=0.0, y=0.0, z=0.033),
                (0.2, 1.0, 0.35, 1.0),
            )
            status.text = "Azas RViz preview: pick -> place front -> press -> re-grasp -> shake"
            markers.append(status)

        for marker in markers:
            marker.header.stamp = stamp
            marker.header.frame_id = frame_id
        for marker in robot_arm_markers:
            marker.header.stamp = stamp
            marker.header.frame_id = frame_id
        return MarkerArray(markers=markers), MarkerArray(markers=robot_arm_markers)

    def active_step(self, steps: Sequence[SequenceStep]) -> SequenceStep | None:
        if not steps:
            return None
        active_index = min(self.preview_step_index, len(steps) - 1)
        return steps[active_index]

    def make_demo_arm_markers(self, active_step: SequenceStep) -> List[Marker]:
        x, y, z = active_step.xyz
        wrist = (x - 0.109, y, max(z, 0.10))
        shoulder = (0.0, 0.0, 0.225)
        elbow = (
            max(0.10, wrist[0] * 0.48),
            wrist[1] * 0.32,
            max(0.28, wrist[2] + 0.18),
        )
        forearm = (
            max(0.14, wrist[0] * 0.76),
            wrist[1] * 0.70,
            max(0.20, wrist[2] + 0.07),
        )
        joints = [
            (0.0, 0.0, 0.055),
            shoulder,
            elbow,
            forearm,
            wrist,
        ]

        arm = self.marker(
            700,
            Marker.LINE_STRIP,
            "low_side_grasp_demo_arm",
            (0.0, 0.0, 0.0),
            Vector3(x=0.030, y=0.0, z=0.0),
            (0.10, 0.42, 1.0, 0.95),
        )
        arm.points = [point(joint) for joint in joints]
        markers = [arm]
        for index, joint in enumerate(joints):
            markers.append(
                self.marker(
                    710 + index,
                    Marker.SPHERE,
                    "low_side_grasp_demo_joints",
                    joint,
                    Vector3(x=0.045, y=0.045, z=0.045),
                    (0.08, 0.18, 0.45, 0.96),
                )
            )

        palm = self.marker(
            730,
            Marker.CUBE,
            "low_side_grasp_rg2",
            (x - 0.035, y, z - 0.012),
            Vector3(x=-0.100, y=0.045, z=0.025),
            (0.16, 0.17, 0.18, 0.92),
        )
        finger_a = self.marker(
            731,
            Marker.CUBE,
            "low_side_grasp_rg2",
            (x + 0.010, y + 0.038, z - 0.030),
            Vector3(x=0.109, y=0.010, z=0.052),
            (0.06, 0.06, 0.07, 0.92),
        )
        finger_b = self.marker(
            732,
            Marker.CUBE,
            "low_side_grasp_rg2",
            (x + 0.010, y - 0.038, z - 0.030),
            Vector3(x=0.109, y=0.010, z=0.052),
            (0.06, 0.06, 0.07, 0.92),
        )
        markers.extend([palm, finger_a, finger_b])
        return markers

    def make_cup_markers(self, steps: Sequence[SequenceStep]) -> List[Marker]:
        if self.last_cup_pose is None or not steps:
            return []
        cup_height = float(self.get_parameter("cup_height_m").value)
        grasp_height = float(self.get_parameter("grasp_height_m").value)
        original = self.last_cup_pose.pose.position
        original_base = (float(original.x), float(original.y), float(original.z))
        active_index = min(self.preview_step_index, len(steps) - 1)
        active_step = steps[active_index]

        if active_step.cup_base_xyz is not None:
            base = active_step.cup_base_xyz
        elif active_index == 0:
            base = original_base
        else:
            base = (
                active_step.xyz[0],
                active_step.xyz[1],
                active_step.xyz[2] - grasp_height,
            )
        body_center = (base[0], base[1], base[2] + cup_height * 0.5)
        mouth_center = (base[0], base[1], base[2] + cup_height)

        body = self.marker(
            500,
            Marker.CYLINDER,
            "animated_cup_body",
            body_center,
            Vector3(x=0.109, y=0.109, z=cup_height),
            (0.1, 0.85, 1.0, 0.72),
        )
        mouth = self.marker(
            501,
            Marker.CYLINDER,
            "animated_cup_mouth",
            mouth_center,
            Vector3(x=0.092, y=0.092, z=0.008),
            (1.0, 1.0, 1.0, 0.95),
        )
        label = self.marker(
            502,
            Marker.TEXT_VIEW_FACING,
            "animated_cup_label",
            (mouth_center[0], mouth_center[1], mouth_center[2] + 0.055),
            Vector3(x=0.0, y=0.0, z=0.034),
            (0.1, 0.85, 1.0, 1.0),
        )
        label.text = f"cup moving: {active_step.label}"
        return [body, mouth, label]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DispenserSequencePreviewNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
