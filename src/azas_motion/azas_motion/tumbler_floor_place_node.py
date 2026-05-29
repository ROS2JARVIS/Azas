#!/usr/bin/env python3
"""Dry-run first tumbler side-grasp transfer controller.

Default behavior is planning/logging only. Doosan motion clients are created
only when all explicit hardware gates are enabled. The legacy default still
plans floor placement; set delivery_mode:=hold_under_outlet to keep the cup
grasped at low side-grasp transfer height in front of the dispenser outlet
instead of placing it on the floor.
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import rclpy
from azas_interfaces.srv import SetGripper
from dsr_msgs2.srv import MoveJoint, MoveLine
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


XYZ = Tuple[float, float, float]

DR_BASE = 0
MOVE_MODE_ABSOLUTE = 0
SYNC = 0
BLENDING_SPEED_TYPE_DUPLICATE = 0
HARDWARE_CONFIRM_PHRASE = "ENABLE_REAL_ROBOT_MOTION"


@dataclass(frozen=True)
class MotionStep:
    label: str
    xyz: XYZ
    gripper: str = "none"
    width_m: float = 0.0
    force_n: float = 0.0


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def tapered_diameter_at_height(
    grip_height: float,
    tumbler_height: float,
    bottom_diameter: float,
    top_diameter: float,
) -> float:
    ratio = clamp(grip_height / max(tumbler_height, 1e-6), 0.0, 1.0)
    return bottom_diameter + (top_diameter - bottom_diameter) * ratio


def service_name(prefix: str, name: str) -> str:
    clean_prefix = prefix.strip("/")
    clean_name = name.strip("/")
    if not clean_prefix:
        return f"/{clean_name}"
    return f"/{clean_prefix}/{clean_name}"


def xyz_list_from_flat(values: Sequence[float], expected_count: int) -> List[XYZ]:
    triples: List[XYZ] = []
    for index in range(expected_count):
        offset = index * 3
        triples.append(
            (
                float(values[offset]),
                float(values[offset + 1]),
                float(values[offset + 2]),
            )
        )
    return triples


def normalize_xy(x: float, y: float) -> Tuple[float, float]:
    length = math.hypot(x, y)
    if length <= 1e-9:
        return (1.0, 0.0)
    return (x / length, y / length)


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def unique_directions(directions: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    unique: List[Tuple[float, float]] = []
    for x, y in directions:
        nx, ny = normalize_xy(x, y)
        if any(abs(nx - ux) < 1e-3 and abs(ny - uy) < 1e-3 for ux, uy in unique):
            continue
        unique.append((nx, ny))
    return unique


def axis_direction(axis: str) -> Optional[Tuple[float, float]]:
    """Return the pre-grasp offset direction for a side-approach axis label."""
    axis = axis.strip()
    if axis == "-x":
        return (-1.0, 0.0)
    if axis == "+x":
        return (1.0, 0.0)
    if axis == "-y":
        return (0.0, -1.0)
    if axis == "+y":
        return (0.0, 1.0)
    return None


def side_grasp_candidates(
    grasp: XYZ,
    offset: float,
    sample_count: int,
    detected_yaw: Optional[float] = None,
    preferred_axes: str = "",
) -> List[XYZ]:
    radial = normalize_xy(grasp[0], grasp[1])
    directions: List[Tuple[float, float]] = []
    for axis in preferred_axes.split(","):
        direction = axis_direction(axis)
        if direction is not None:
            directions.append(direction)
    directions.append(radial)
    if detected_yaw is not None:
        yaw_x = math.cos(detected_yaw)
        yaw_y = math.sin(detected_yaw)
        directions.extend(
            [
                (yaw_x, yaw_y),
                (-yaw_x, -yaw_y),
                (-yaw_y, yaw_x),
                (yaw_y, -yaw_x),
            ]
        )
    for index in range(max(sample_count, 0)):
        theta = 2.0 * math.pi * index / max(sample_count, 1)
        directions.append((math.cos(theta), math.sin(theta)))

    return [
        (grasp[0] - dx * offset, grasp[1] - dy * offset, grasp[2])
        for dx, dy in unique_directions(directions)
    ]


def xy_distance_to_segment(
    point_xy: Tuple[float, float],
    a_xy: Tuple[float, float],
    b_xy: Tuple[float, float],
) -> float:
    px, py = point_xy
    ax, ay = a_xy
    bx, by = b_xy
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = clamp(((px - ax) * dx + (py - ay) * dy) / length_sq, 0.0, 1.0)
    nearest_x = ax + t * dx
    nearest_y = ay + t * dy
    return math.hypot(px - nearest_x, py - nearest_y)


def path_intersects_bottle(a: XYZ, b: XYZ, bottle_xy: Tuple[float, float], safety_radius: float) -> bool:
    return xy_distance_to_segment(bottle_xy, (a[0], a[1]), (b[0], b[1])) < safety_radius


def compute_detour(a: XYZ, b: XYZ, bottle_xy: Tuple[float, float], safety_radius: float, z: float) -> XYZ:
    ax, ay = a[0], a[1]
    bx, by = b[0], b[1]
    dx = bx - ax
    dy = by - ay
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return (a[0], a[1] + safety_radius, z)

    nx = -dy / length
    ny = dx / length
    candidate_a = (bottle_xy[0] + nx * safety_radius, bottle_xy[1] + ny * safety_radius, z)
    candidate_b = (bottle_xy[0] - nx * safety_radius, bottle_xy[1] - ny * safety_radius, z)
    base_y = (ay + by) * 0.5
    return candidate_a if abs(candidate_a[1] - base_y) > abs(candidate_b[1] - base_y) else candidate_b


class TumblerFloorPlaceNode(Node):
    def __init__(self) -> None:
        super().__init__("tumbler_floor_place_node")

        self.declare_parameter("auto_start", True)
        self.declare_parameter("enable_hardware", False)
        self.declare_parameter("hardware_confirm", "")
        self.declare_parameter("allow_service_control_without_moveit", False)
        self.declare_parameter("service_prefix", "")
        self.declare_parameter("execution_stage", "full")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("use_tumbler_pose_topic", True)
        self.declare_parameter("tumbler_pose_topic", "/jarvis/tumbler_dispenser/tumbler_pose")
        self.declare_parameter("tumbler_pose_wait_timeout", 3.0)
        self.declare_parameter("tumbler_pose_max_age_sec", 1.0)
        self.declare_parameter("allow_demo_tumbler_position_fallback", True)
        self.declare_parameter("selected_dispenser_id", 1)
        self.declare_parameter("dispenser_count", 4)
        self.declare_parameter("tumbler_position", [0.32, -0.22, 0.05])
        self.declare_parameter("tumbler_position_x", 0.32)
        self.declare_parameter("tumbler_position_y", -0.22)
        self.declare_parameter("tumbler_position_z", 0.05)
        self.declare_parameter("tumbler_height", 0.17)
        self.declare_parameter("tumbler_radius", 0.0375)
        self.declare_parameter("tumbler_bottom_diameter", 0.065)
        self.declare_parameter("tumbler_top_diameter", 0.075)
        self.declare_parameter("grasp_height", 0.085)
        self.declare_parameter("side_grasp_approach_offset", 0.10)
        self.declare_parameter("side_grasp_candidate_count", 16)
        self.declare_parameter("side_grasp_preferred_axes", "")
        self.declare_parameter("use_detected_grasp_yaw", True)
        self.declare_parameter("lift_height", 0.04)
        self.declare_parameter("delivery_mode", "floor_place")
        self.declare_parameter("place_approach_height", 0.06)
        self.declare_parameter("placement_floor_z", 0.0)
        self.declare_parameter("place_mouth_under_outlet", False)
        self.declare_parameter("outlet_mouth_clearance", 0.0)
        self.declare_parameter("clearance", 0.05)

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
                0.609,
                0.070,
                0.087,
                0.617,
                0.028,
                0.082,
                0.616,
                -0.026,
                0.079,
                0.607,
                -0.083,
                0.075,
            ],
        )

        self.declare_parameter("home_joints_deg", [0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
        self.declare_parameter("move_home_first", False)
        self.declare_parameter("return_home", False)
        self.declare_parameter("rx", 180.0)
        self.declare_parameter("ry", 0.0)
        self.declare_parameter("rz", 180.0)
        self.declare_parameter("joint_velocity", 20.0)
        self.declare_parameter("joint_acceleration", 20.0)
        self.declare_parameter("line_velocity", 30.0)
        self.declare_parameter("line_acceleration", 50.0)
        self.declare_parameter("hold_seconds_after_grasp", 0.2)
        self.declare_parameter("hold_seconds_after_place", 0.2)

        self.declare_parameter("workspace_x_min", 0.0)
        self.declare_parameter("workspace_x_max", 0.80)
        self.declare_parameter("workspace_y_min", -0.35)
        self.declare_parameter("workspace_y_max", 0.35)
        self.declare_parameter("workspace_z_min", 0.0)
        self.declare_parameter("workspace_z_max", 0.80)

        self.declare_parameter("gripper_open_service", "")
        self.declare_parameter("gripper_close_service", "")
        self.declare_parameter("gripper_set_service", "/jarvis/rg2/set_width")
        self.declare_parameter("gripper_preopen_clearance", 0.025)
        self.declare_parameter("gripper_grasp_compression", 0.006)
        self.declare_parameter("gripper_grasp_force_n", 12.0)
        self.declare_parameter("gripper_preopen_force_n", 8.0)
        self.declare_parameter("gripper_max_width_m", 0.110)
        self.declare_parameter("gripper_min_width_m", 0.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.service_prefix = str(self.get_parameter("service_prefix").value)
        self.enable_hardware = bool(self.get_parameter("enable_hardware").value)
        self.hardware_confirm = str(self.get_parameter("hardware_confirm").value)
        self.allow_service_control_without_moveit = bool(
            self.get_parameter("allow_service_control_without_moveit").value
        )
        self.hardware_armed = all(
            (
                self.enable_hardware,
                self.hardware_confirm == HARDWARE_CONFIRM_PHRASE,
                self.allow_service_control_without_moveit,
            )
        )

        self.move_joint = None
        self.move_line = None
        self.gripper_open = None
        self.gripper_close = None
        self.gripper_set = None
        if self.hardware_armed:
            self.move_joint = self.create_client(
                MoveJoint,
                service_name(self.service_prefix, "motion/move_joint"),
            )
            self.move_line = self.create_client(
                MoveLine,
                service_name(self.service_prefix, "motion/move_line"),
            )
            open_service = str(self.get_parameter("gripper_open_service").value)
            close_service = str(self.get_parameter("gripper_close_service").value)
            set_service = str(self.get_parameter("gripper_set_service").value)
            if open_service:
                self.gripper_open = self.create_client(Trigger, open_service)
            if close_service:
                self.gripper_close = self.create_client(Trigger, close_service)
            if set_service:
                self.gripper_set = self.create_client(SetGripper, set_service)

        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "/jarvis/tumbler_floor_place/plan", path_qos)
        self.target_pub = self.create_publisher(PoseStamped, "/jarvis/tumbler_floor_place/target_pose", 10)
        self.status_pub = self.create_publisher(String, "/jarvis/tumbler_floor_place/status", 10)
        self.last_tumbler_pose = None
        self.last_tumbler_pose_received_time = None
        self.start_time = self.get_clock().now()

        if bool(self.get_parameter("use_tumbler_pose_topic").value):
            topic = str(self.get_parameter("tumbler_pose_topic").value)
            self.create_subscription(PoseStamped, topic, self.on_tumbler_pose, 10)
            self.get_logger().info(f"Waiting for detected tumbler pose on {topic}")

        self.started = False
        self.timer = self.create_timer(0.5, self.on_timer)

        self.get_logger().info(
            "tumbler_floor_place_node ready. "
            f"delivery_mode={self.delivery_mode()}; "
            f"hardware_armed={self.hardware_armed}; default is dry-run."
        )

    def on_tumbler_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != self.frame_id:
            self.last_tumbler_pose = None
            self.last_tumbler_pose_received_time = None
            self.get_logger().error(
                "Rejected tumbler pose with unexpected frame_id "
                f"'{msg.header.frame_id}'. Expected '{self.frame_id}'."
            )
            self.publish_status("REJECTED_TUMBLER_POSE_FRAME")
            return
        self.last_tumbler_pose = msg
        self.last_tumbler_pose_received_time = self.get_clock().now()

    def on_timer(self) -> None:
        if self.started or not bool(self.get_parameter("auto_start").value):
            return
        if not self.tumbler_pose_ready_for_start():
            return
        self.started = True
        threading.Thread(target=self._run_once_and_publish, daemon=True).start()

    def _run_once_and_publish(self) -> None:
        ok = self.run_once()
        status = "DONE" if ok else "FAILED"
        self.publish_status(status)

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def tumbler_pose_ready_for_start(self) -> bool:
        if not bool(self.get_parameter("use_tumbler_pose_topic").value):
            return True
        if self.valid_tumbler_pose_available():
            return True

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        timeout = float(self.get_parameter("tumbler_pose_wait_timeout").value)
        if elapsed < timeout:
            self.publish_status("WAITING_FOR_TUMBLER_POSE")
            return False

        if self.enable_hardware:
            self.get_logger().error(
                "No detected tumbler pose received. Refusing hardware-capable run."
            )
            return True

        if bool(self.get_parameter("allow_demo_tumbler_position_fallback").value):
            self.get_logger().warning(
                "No detected tumbler pose received before timeout; using demo tumbler_position fallback."
            )
            return True

        self.get_logger().error("No detected tumbler pose received and fallback is disabled.")
        return True

    def valid_tumbler_pose_available(self) -> bool:
        if self.last_tumbler_pose is None or self.last_tumbler_pose_received_time is None:
            return False

        age = (self.get_clock().now() - self.last_tumbler_pose_received_time).nanoseconds / 1e9
        max_age = float(self.get_parameter("tumbler_pose_max_age_sec").value)
        if age > max_age:
            self.get_logger().warning(
                f"Detected tumbler pose is stale: age={age:.2f}s max={max_age:.2f}s"
            )
            self.last_tumbler_pose = None
            self.last_tumbler_pose_received_time = None
            self.publish_status("STALE_TUMBLER_POSE")
            return False
        return True

    def selected_layout(self) -> Tuple[List[XYZ], List[XYZ], int]:
        count = max(int(self.get_parameter("dispenser_count").value), 1)
        selected_id = min(max(int(self.get_parameter("selected_dispenser_id").value), 1), count)
        bottle_values = self.get_parameter("dispenser_bottle_positions").value
        outlet_values = self.get_parameter("dispenser_outlet_positions").value
        if len(bottle_values) != count * 3 or len(outlet_values) != count * 3:
            raise ValueError("dispenser_bottle_positions and dispenser_outlet_positions must be flat XYZ arrays")
        return xyz_list_from_flat(bottle_values, count), xyz_list_from_flat(outlet_values, count), selected_id - 1

    def delivery_mode(self) -> str:
        mode = str(self.get_parameter("delivery_mode").value).strip().lower()
        if mode in {"", "floor", "floor_place", "place_on_floor"}:
            return "floor_place"
        if mode in {
            "hold",
            "outlet_hold",
            "hold_under_outlet",
            "outlet_front_hold",
            "move_to_outlet",
        }:
            return "hold_under_outlet"
        raise RuntimeError(
            "unsupported delivery_mode. Use one of: floor_place, hold_under_outlet"
        )

    def build_steps(self) -> List[MotionStep]:
        bottle_positions, outlet_positions, selected_index = self.selected_layout()
        tumbler_base = self.tumbler_base()
        grasp_height = float(self.get_parameter("grasp_height").value)
        side_approach_offset = float(self.get_parameter("side_grasp_approach_offset").value)
        lift_height = float(self.get_parameter("lift_height").value)
        approach_height = float(self.get_parameter("place_approach_height").value)
        floor_z = float(self.get_parameter("placement_floor_z").value)
        target_outlet = outlet_positions[selected_index]
        delivery_mode = self.delivery_mode()
        grasp_width, preopen_width = self.gripper_width_targets(grasp_height)
        grasp_force = float(self.get_parameter("gripper_grasp_force_n").value)
        preopen_force = float(self.get_parameter("gripper_preopen_force_n").value)

        target_base_z = floor_z
        mouth_under_outlet = bool(self.get_parameter("place_mouth_under_outlet").value)
        if mouth_under_outlet:
            target_base_z = (
                target_outlet[2]
                - float(self.get_parameter("tumbler_height").value)
                - float(self.get_parameter("outlet_mouth_clearance").value)
            )
        grasp = (tumbler_base[0], tumbler_base[1], tumbler_base[2] + grasp_height)
        target = (target_outlet[0], target_outlet[1], target_base_z + grasp_height)
        pre_target = (target[0], target[1], target[2] + approach_height)
        lift_z = grasp[2] + lift_height
        if mouth_under_outlet:
            lift_z = max(lift_z, pre_target[2])
        lift = (grasp[0], grasp[1], lift_z)
        if delivery_mode == "hold_under_outlet" and not mouth_under_outlet:
            target = (target_outlet[0], target_outlet[1], lift_z)
            pre_target = target
        safety_radius = 0.041 + float(self.get_parameter("clearance").value)
        safety_radius += float(self.get_parameter("tumbler_radius").value)
        side_pre_grasp = self.select_side_pre_grasp(
            grasp,
            side_approach_offset,
            bottle_positions,
            safety_radius,
        )

        steps = [
            MotionStep("side_pre_grasp", side_pre_grasp, "preopen", preopen_width, preopen_force),
            MotionStep("side_grasp_tumbler", grasp, "close", grasp_width, grasp_force),
            MotionStep("lift_tumbler", lift),
        ]

        current = lift
        for index, bottle in enumerate(bottle_positions, start=1):
            bottle_xy = (bottle[0], bottle[1])
            if path_intersects_bottle(current, pre_target, bottle_xy, safety_radius):
                detour = compute_detour(current, pre_target, bottle_xy, safety_radius, current[2])
                steps.append(MotionStep(f"detour_around_dispenser_{index}", detour))
                current = detour

        if delivery_mode == "hold_under_outlet":
            steps.extend(
                [
                    MotionStep("pre_outlet_front_hold", pre_target),
                    MotionStep("outlet_front_hold", target),
                ]
            )
        else:
            steps.extend(
                [
                    MotionStep("pre_floor_place", pre_target),
                    MotionStep("floor_place", target, "open"),
                    MotionStep("retreat_after_place", pre_target),
                ]
            )
        return self.limit_steps_for_stage(steps)

    def limit_steps_for_stage(self, steps: Sequence[MotionStep]) -> List[MotionStep]:
        stage = str(self.get_parameter("execution_stage").value).strip().lower()
        if stage in {"", "all", "full"}:
            return list(steps)

        terminal_by_stage = {
            "approach": "side_pre_grasp",
            "grasp": "side_grasp_tumbler",
            "lift": "lift_tumbler",
            "pre_place": "pre_floor_place",
            "pre_floor_place": "pre_floor_place",
            "place": "floor_place",
            "pre_outlet": "pre_outlet_front_hold",
            "pre_outlet_hold": "pre_outlet_front_hold",
            "outlet": "outlet_front_hold",
            "outlet_hold": "outlet_front_hold",
            "hold": "outlet_front_hold",
        }
        terminal = terminal_by_stage.get(stage)
        if terminal is None:
            raise RuntimeError(
                "unsupported execution_stage. Use one of: full, approach, grasp, lift, "
                "pre_place, place, pre_outlet, outlet_hold"
            )

        limited: List[MotionStep] = []
        for step in steps:
            limited.append(step)
            if step.label == terminal:
                self.get_logger().warning(
                    f"execution_stage={stage}: stopping plan at {terminal}"
                )
                return limited
        raise RuntimeError(f"execution_stage={stage} terminal step {terminal} was not generated")

    def gripper_width_targets(self, grasp_height: float) -> Tuple[float, float]:
        diameter = tapered_diameter_at_height(
            grasp_height,
            float(self.get_parameter("tumbler_height").value),
            float(self.get_parameter("tumbler_bottom_diameter").value),
            float(self.get_parameter("tumbler_top_diameter").value),
        )
        min_width = float(self.get_parameter("gripper_min_width_m").value)
        max_width = float(self.get_parameter("gripper_max_width_m").value)
        grasp_width = clamp(
            diameter - float(self.get_parameter("gripper_grasp_compression").value),
            min_width,
            max_width,
        )
        preopen_width = clamp(
            diameter + float(self.get_parameter("gripper_preopen_clearance").value),
            min_width,
            max_width,
        )
        self.get_logger().info(
            "Gripper taper targets: "
            f"diameter_at_grasp={diameter:.3f} preopen_width={preopen_width:.3f} "
            f"grasp_width={grasp_width:.3f}"
        )
        return grasp_width, preopen_width

    def select_side_pre_grasp(
        self,
        grasp: XYZ,
        offset: float,
        bottle_positions: Sequence[XYZ],
        safety_radius: float,
    ) -> XYZ:
        sample_count = int(self.get_parameter("side_grasp_candidate_count").value)
        preferred_axes = str(self.get_parameter("side_grasp_preferred_axes").value)
        detected_yaw = self.detected_grasp_yaw()
        candidates = side_grasp_candidates(grasp, offset, sample_count, detected_yaw, preferred_axes)
        for index, candidate in enumerate(candidates, start=1):
            if not self.point_in_workspace(candidate):
                continue
            if any(
                xy_distance_to_segment((bottle[0], bottle[1]), (candidate[0], candidate[1]), (grasp[0], grasp[1]))
                < safety_radius
                for bottle in bottle_positions
            ):
                continue
            self.get_logger().info(
                "Selected side grasp candidate "
                f"{index}/{len(candidates)}: x={candidate[0]:.3f} y={candidate[1]:.3f}"
            )
            return candidate

        self.get_logger().error(
            "No side grasp candidate passed workspace/keep-out checks; refusing to plan cup grasp."
        )
        raise RuntimeError("no valid side grasp candidate")

    def detected_grasp_yaw(self) -> Optional[float]:
        if not bool(self.get_parameter("use_detected_grasp_yaw").value):
            return None
        if not self.valid_tumbler_pose_available():
            return None
        orientation = self.last_tumbler_pose.pose.orientation
        return quaternion_yaw(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )

    def tumbler_base(self) -> XYZ:
        if self.valid_tumbler_pose_available():
            pose = self.last_tumbler_pose.pose.position
            self.get_logger().info(
                "Using detected tumbler pose: "
                f"x={pose.x:.3f} y={pose.y:.3f} z={pose.z:.3f}"
            )
            return (float(pose.x), float(pose.y), float(pose.z))

        using_pose_topic = bool(self.get_parameter("use_tumbler_pose_topic").value)
        allow_fallback = bool(self.get_parameter("allow_demo_tumbler_position_fallback").value)
        if self.enable_hardware and using_pose_topic and not allow_fallback:
            raise RuntimeError("hardware run requires detected tumbler pose")

        if not allow_fallback:
            raise RuntimeError("detected tumbler pose is required")

        fallback = (
            float(self.get_parameter("tumbler_position_x").value),
            float(self.get_parameter("tumbler_position_y").value),
            float(self.get_parameter("tumbler_position_z").value),
        )
        self.get_logger().warning(
            "Using demo tumbler_position fallback: "
            f"x={fallback[0]:.3f} y={fallback[1]:.3f} z={fallback[2]:.3f}"
        )
        return fallback

    def validate_workspace(self, steps: Sequence[MotionStep]) -> bool:
        ok = True
        for step in steps:
            if not self.point_in_workspace(step.xyz):
                x, y, z = step.xyz
                self.get_logger().error(
                    f"{step.label} outside workspace: ({x:.3f}, {y:.3f}, {z:.3f})"
                )
                ok = False
        return ok

    def point_in_workspace(self, xyz: XYZ) -> bool:
        x_min = float(self.get_parameter("workspace_x_min").value)
        x_max = float(self.get_parameter("workspace_x_max").value)
        y_min = float(self.get_parameter("workspace_y_min").value)
        y_max = float(self.get_parameter("workspace_y_max").value)
        z_min = float(self.get_parameter("workspace_z_min").value)
        z_max = float(self.get_parameter("workspace_z_max").value)
        x, y, z = xyz
        return x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max

    def publish_plan(self, steps: Sequence[MotionStep]) -> None:
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = self.frame_id
        for step in steps:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = step.xyz[0]
            pose.pose.position.y = step.xyz[1]
            pose.pose.position.z = step.xyz[2]
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        if path.poses:
            target_index = -2 if steps[-1].label.startswith("retreat_after_") and len(path.poses) >= 2 else -1
            self.target_pub.publish(path.poses[target_index])

    def wait_for_hardware_services(self) -> bool:
        clients = [(self.move_joint, "motion/move_joint"), (self.move_line, "motion/move_line")]
        if self.gripper_open is not None:
            clients.append((self.gripper_open, "gripper_open_service"))
        if self.gripper_close is not None:
            clients.append((self.gripper_close, "gripper_close_service"))
        if self.gripper_set is not None:
            clients.append((self.gripper_set, "gripper_set_service"))
        for client, label in clients:
            while rclpy.ok() and client is not None and not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {label}")
        return True

    def call_trigger(self, client, label: str) -> bool:
        if client is None:
            self.get_logger().warning(f"{label}: no gripper service configured; logging only")
            return True
        future = client.call_async(Trigger.Request())
        result = self.wait_for_future(future, label)
        if result is None or not result.success:
            self.get_logger().error(f"{label} failed")
            return False
        return True

    def call_set_gripper(self, step: MotionStep, label: str) -> bool:
        if self.gripper_set is None:
            self.get_logger().warning(f"{label}: no gripper width service configured; falling back")
            return False
        req = SetGripper.Request()
        req.command = "preopen" if step.gripper == "preopen" else "grasp"
        req.width_m = float(step.width_m)
        req.force_n = float(step.force_n)
        self.get_logger().info(
            f"{label}: command={req.command} width_m={req.width_m:.3f} force_n={req.force_n:.1f}"
        )
        future = self.gripper_set.call_async(req)
        result = self.wait_for_future(future, label)
        if result is None or not result.success:
            self.get_logger().error(f"{label} failed")
            return False
        return True

    def command_gripper(self, step: MotionStep) -> bool:
        if step.gripper == "preopen":
            return self.call_set_gripper(step, "preopen_gripper") or self.call_trigger(
                self.gripper_open, "open_gripper"
            )
        if step.gripper == "close":
            return self.call_set_gripper(step, "close_gripper") or self.call_trigger(
                self.gripper_close, "close_gripper"
            )
        if step.gripper == "open":
            return self.call_trigger(self.gripper_open, "open_gripper")
        return True

    def call_movej(self, label: str) -> bool:
        req = MoveJoint.Request()
        req.pos = [float(v) for v in self.get_parameter("home_joints_deg").value]
        req.vel = float(self.get_parameter("joint_velocity").value)
        req.acc = float(self.get_parameter("joint_acceleration").value)
        req.time = 0.0
        req.radius = 0.0
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        return self.call_motion(self.move_joint, req, label)

    def call_movel(self, step: MotionStep) -> bool:
        req = MoveLine.Request()
        req.pos = [
            step.xyz[0] * 1000.0,
            step.xyz[1] * 1000.0,
            step.xyz[2] * 1000.0,
            float(self.get_parameter("rx").value),
            float(self.get_parameter("ry").value),
            float(self.get_parameter("rz").value),
        ]
        req.vel = [float(self.get_parameter("line_velocity").value)] * 2
        req.acc = [float(self.get_parameter("line_acceleration").value)] * 2
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        return self.call_motion(self.move_line, req, step.label)

    def call_motion(self, client, request, label: str) -> bool:
        self.get_logger().info(f"{label}: calling hardware service")
        future = client.call_async(request)
        result = self.wait_for_future(future, label)
        if result is None:
            self.get_logger().error(f"{label} failed: {future.exception()}")
            return False
        if not result.success:
            self.get_logger().error(f"{label} returned success=false")
            return False
        return True

    def wait_for_future(self, future, label: str, timeout_sec: float = 10.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(f"{label} timed out waiting for service response")
                return None
            time.sleep(0.01)
        if not future.done():
            return None
        return future.result()

    def execute_hardware(self, steps: Sequence[MotionStep]) -> bool:
        if not self.hardware_armed:
            self.get_logger().warning(
                "Hardware not armed. Dry-run only. To arm, set enable_hardware:=true, "
                f"hardware_confirm:={HARDWARE_CONFIRM_PHRASE}, and "
                "allow_service_control_without_moveit:=true."
            )
            return True

        self.wait_for_hardware_services()
        if bool(self.get_parameter("move_home_first").value) and not self.call_movej("move_home_first"):
            return False

        for step in steps:
            if step.gripper == "preopen" and not self.command_gripper(step):
                return False
            if not self.call_movel(step):
                return False
            if step.gripper == "close":
                if not self.command_gripper(step):
                    return False
                time.sleep(float(self.get_parameter("hold_seconds_after_grasp").value))
            elif step.gripper == "open":
                if not self.command_gripper(step):
                    return False
                time.sleep(float(self.get_parameter("hold_seconds_after_place").value))

        if bool(self.get_parameter("return_home").value):
            return self.call_movej("return_home")
        return True

    def run_once(self) -> bool:
        if self.enable_hardware and not self.hardware_armed:
            self.get_logger().error(
                "enable_hardware was requested but hardware gates are incomplete. Refusing motion."
            )
            return False

        try:
            steps = self.build_steps()
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return False
        self.publish_plan(steps)
        for step in steps:
            self.get_logger().info(
                f"plan {step.label}: x={step.xyz[0]:.3f} y={step.xyz[1]:.3f} "
                f"z={step.xyz[2]:.3f} gripper={step.gripper} "
                f"width_m={step.width_m:.3f} force_n={step.force_n:.1f}"
            )
        if not self.validate_workspace(steps):
            return False
        return self.execute_hardware(steps)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TumblerFloorPlaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
