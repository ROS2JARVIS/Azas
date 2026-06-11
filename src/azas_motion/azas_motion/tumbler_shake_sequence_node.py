#!/usr/bin/env python3
"""Dry-run first tumbler shake sequence in a validated safe space."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import rclpy
from dsr_msgs2.srv import GetCurrentPosj, Ikin, MoveJoint, MoveLine, MoveWait
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetStateValidity
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


XYZ = Tuple[float, float, float]

DR_BASE = 0
MOVE_MODE_ABSOLUTE = 0
SYNC = 0
BLENDING_SPEED_TYPE_DUPLICATE = 0
HARDWARE_CONFIRM_PHRASE = "ENABLE_REAL_ROBOT_MOTION"
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


@dataclass(frozen=True)
class SequenceStep:
    label: str
    xyz: XYZ
    hold_seconds: float = 0.0
    rx_offset_deg: float = 0.0
    ry_offset_deg: float = 0.0
    rz_offset_deg: float = 0.0
    phase: str = "shake"


@dataclass(frozen=True)
class JointSequenceStep:
    label: str
    joints_deg: tuple[float, float, float, float, float, float]
    hold_seconds: float = 0.0
    phase: str = "shake"


ShakeStep = SequenceStep | JointSequenceStep


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


class TumblerShakeSequenceNode(Node):
    def __init__(self) -> None:
        super().__init__("tumbler_shake_sequence_node")

        self.declare_parameter("auto_start", True)
        self.declare_parameter("shutdown_after_run", True)
        self.declare_parameter("enable_hardware", False)
        self.declare_parameter("hardware_confirm", "")
        self.declare_parameter("allow_service_control_without_moveit", False)
        self.declare_parameter("service_prefix", "")
        self.declare_parameter("execution_stage", "full")
        self.declare_parameter("shake_control_mode", "cartesian")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("dispenser_count", 4)
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

        self.declare_parameter("shake_center_x", 0.28)
        self.declare_parameter("shake_center_y", -0.30)
        self.declare_parameter("shake_center_z", 0.62)
        self.declare_parameter("shake_approach_height", 0.10)
        self.declare_parameter("shake_amplitude_x", 0.100)
        self.declare_parameter("shake_amplitude_y", 0.040)
        self.declare_parameter("shake_amplitude_z", 0.055)
        self.declare_parameter("shake_cycles", 4)
        self.declare_parameter("shake_twist_rx_deg", 6.0)
        self.declare_parameter("shake_twist_ry_deg", 3.0)
        self.declare_parameter("shake_twist_rz_deg", 22.0)
        self.declare_parameter("shake_hold_seconds", 0.0)

        self.declare_parameter("workspace_min_x", 0.0)
        self.declare_parameter("workspace_max_x", 0.80)
        self.declare_parameter("workspace_min_y", -0.35)
        self.declare_parameter("workspace_max_y", 0.35)
        self.declare_parameter("workspace_min_z", 0.0)
        self.declare_parameter("workspace_max_z", 0.80)
        self.declare_parameter("min_shake_z", 0.55)
        self.declare_parameter("dispenser_keepout_radius", 0.20)

        self.declare_parameter("rx", 180.0)
        self.declare_parameter("ry", 0.0)
        self.declare_parameter("rz", 180.0)
        self.declare_parameter("line_velocity", 45.0)
        self.declare_parameter("line_acceleration", 80.0)
        self.declare_parameter("line_time", 0.0)
        self.declare_parameter("approach_line_velocity", 20.0)
        self.declare_parameter("approach_line_acceleration", 25.0)
        self.declare_parameter("approach_line_time", 3.5)
        self.declare_parameter("shake_line_velocity", 85.0)
        self.declare_parameter("shake_line_acceleration", 130.0)
        self.declare_parameter("shake_line_time", 0.40)
        self.declare_parameter("service_wait_timeout_sec", 5.0)
        self.declare_parameter("motion_response_timeout_sec", 10.0)
        self.declare_parameter("precheck_ikin_joint5", True)
        self.declare_parameter("enforce_wrist_joint_limits", False)
        self.declare_parameter("ikin_sol_space", 2)
        self.declare_parameter("joint5_min_deg", 40.0)
        self.declare_parameter("joint5_max_deg", 100.0)
        self.declare_parameter("wrist_min_deg", -135.0)
        self.declare_parameter("wrist_max_deg", 135.0)
        self.declare_parameter("joint_shake_base_j1_deg", 0.0)
        self.declare_parameter("joint_shake_base_j2_deg", -35.0)
        self.declare_parameter("joint_shake_base_j3_deg", 50.0)
        self.declare_parameter("joint_shake_base_j4_deg", 0.0)
        self.declare_parameter("joint_shake_base_j5_deg", 70.0)
        self.declare_parameter("joint_shake_base_j6_deg", 0.0)
        self.declare_parameter("joint_shake_j3_amplitude_deg", 0.0)
        self.declare_parameter("joint_shake_j4_amplitude_deg", 25.0)
        self.declare_parameter("joint_shake_j5_amplitude_deg", 30.0)
        self.declare_parameter("joint_shake_j6_amplitude_deg", 37.0)
        self.declare_parameter("joint_shake_j1_min_deg", -20.0)
        self.declare_parameter("joint_shake_j1_max_deg", 5.0)
        self.declare_parameter("joint_shake_j2_min_deg", -80.0)
        self.declare_parameter("joint_shake_j2_max_deg", 5.0)
        self.declare_parameter("joint_shake_j3_min_deg", 0.0)
        self.declare_parameter("joint_shake_j3_max_deg", 135.0)
        self.declare_parameter("joint_shake_max_single_delta_deg", 75.0)
        self.declare_parameter("approach_joint_velocity", 18.0)
        self.declare_parameter("approach_joint_acceleration", 22.0)
        self.declare_parameter("approach_joint_time", 2.6)
        self.declare_parameter("shake_joint_velocity", 180.0)
        self.declare_parameter("shake_joint_acceleration", 260.0)
        self.declare_parameter("shake_joint_time", 0.0)
        self.declare_parameter("joint_shake_peak_velocity_limit_deg_s", 225.0)
        self.declare_parameter("verify_joint_targets", True)
        self.declare_parameter("joint_target_tolerance_deg", 8.0)
        self.declare_parameter("joint_target_wait_extra_sec", 3.0)
        self.declare_parameter("joint_target_poll_sec", 0.05)
        self.declare_parameter("require_state_validity_for_joint_shake", False)
        self.declare_parameter("state_validity_service", "/check_state_validity")
        self.declare_parameter("planning_group", "manipulator")

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

        self.move_line = None
        self.move_joint = None
        self.move_wait = None
        self.current_posj = None
        self.ikin = None
        self.state_validity = None
        if self.hardware_armed:
            if self.using_joint_control():
                self.move_joint = self.create_client(
                    MoveJoint,
                    service_name(self.service_prefix, "motion/move_joint"),
                )
                self.move_wait = self.create_client(
                    MoveWait,
                    service_name(self.service_prefix, "motion/move_wait"),
                )
                self.current_posj = self.create_client(
                    GetCurrentPosj,
                    service_name(self.service_prefix, "aux_control/get_current_posj"),
                )
                if bool(self.get_parameter("require_state_validity_for_joint_shake").value):
                    self.state_validity = self.create_client(
                        GetStateValidity,
                        str(self.get_parameter("state_validity_service").value),
                    )
            else:
                self.move_line = self.create_client(
                    MoveLine,
                    service_name(self.service_prefix, "motion/move_line"),
                )
                self.ikin = self.create_client(
                    Ikin,
                    service_name(self.service_prefix, "motion/ikin"),
                )

        self.path_pub = self.create_publisher(Path, "/jarvis/tumbler_shake_sequence/plan", 10)
        self.status_pub = self.create_publisher(String, "/jarvis/tumbler_shake_sequence/status", 10)
        self.started = False
        self.done = False
        self.timer = self.create_timer(0.5, self.on_timer)
        self.get_logger().info(
            "tumbler_shake_sequence_node ready. "
            f"hardware_armed={self.hardware_armed}; "
            f"shake_control_mode={self.shake_control_mode()}; default is dry-run."
        )

    def shake_control_mode(self) -> str:
        return str(self.get_parameter("shake_control_mode").value).strip().lower()

    def using_joint_control(self) -> bool:
        return self.shake_control_mode() in {"joint", "joint_space", "move_joint"}

    def on_timer(self) -> None:
        if self.started or not bool(self.get_parameter("auto_start").value):
            return
        self.started = True
        threading.Thread(target=self._run_once_and_publish, daemon=True).start()

    def _run_once_and_publish(self) -> None:
        ok = self.run_once()
        self.publish_status("DONE" if ok else "FAILED")
        if bool(self.get_parameter("shutdown_after_run").value):
            self.get_logger().info("shutdown_after_run=true; exiting tumbler shake node.")
            self.done = True

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def dispenser_positions(self) -> List[XYZ]:
        count = max(int(self.get_parameter("dispenser_count").value), 1)
        values = self.get_parameter("dispenser_bottle_positions").value
        if len(values) != count * 3:
            raise ValueError("dispenser_bottle_positions must be a flat XYZ array")
        return xyz_list_from_flat(values, count)

    def build_steps(self) -> List[SequenceStep]:
        center_x = float(self.get_parameter("shake_center_x").value)
        center_y = float(self.get_parameter("shake_center_y").value)
        center_z = float(self.get_parameter("shake_center_z").value)
        approach_height = float(self.get_parameter("shake_approach_height").value)
        amp_x = abs(float(self.get_parameter("shake_amplitude_x").value))
        amp_y = abs(float(self.get_parameter("shake_amplitude_y").value))
        amp_z = abs(float(self.get_parameter("shake_amplitude_z").value))
        cycles = max(int(self.get_parameter("shake_cycles").value), 1)
        twist_rx = self._clamped_abs_parameter("shake_twist_rx_deg", 20.0)
        twist_ry = self._clamped_abs_parameter("shake_twist_ry_deg", 10.0)
        twist_rz = self._clamped_abs_parameter("shake_twist_rz_deg", 45.0)
        hold = max(float(self.get_parameter("shake_hold_seconds").value), 0.0)

        steps = [
            SequenceStep(
                "shake_safe_approach",
                (center_x, center_y, center_z + approach_height),
                phase="approach",
            ),
            SequenceStep("shake_center_start", (center_x, center_y, center_z), phase="approach"),
        ]
        for cycle in range(1, cycles + 1):
            steps.extend(
                [
                    SequenceStep(
                        f"shake_cycle_{cycle}_x_plus",
                        (center_x + amp_x, center_y, center_z),
                        hold,
                        ry_offset_deg=twist_ry,
                        rz_offset_deg=twist_rz,
                    ),
                    SequenceStep(
                        f"shake_cycle_{cycle}_x_minus",
                        (center_x - amp_x, center_y, center_z),
                        hold,
                        ry_offset_deg=-twist_ry,
                        rz_offset_deg=-twist_rz,
                    ),
                    SequenceStep(
                        f"shake_cycle_{cycle}_y_plus",
                        (center_x, center_y + amp_y, center_z),
                        hold,
                        rx_offset_deg=twist_rx,
                        ry_offset_deg=-0.5 * twist_ry,
                        rz_offset_deg=-0.6 * twist_rz,
                    ),
                    SequenceStep(
                        f"shake_cycle_{cycle}_y_minus",
                        (center_x, center_y - amp_y, center_z),
                        hold,
                        rx_offset_deg=-twist_rx,
                        ry_offset_deg=0.5 * twist_ry,
                        rz_offset_deg=0.6 * twist_rz,
                    ),
                    SequenceStep(
                        f"shake_cycle_{cycle}_z_plus",
                        (center_x, center_y, center_z + amp_z),
                        hold,
                        rx_offset_deg=twist_rx,
                        ry_offset_deg=twist_ry,
                    ),
                    SequenceStep(
                        f"shake_cycle_{cycle}_z_minus",
                        (center_x, center_y, center_z - amp_z),
                        hold,
                        rx_offset_deg=-twist_rx,
                        ry_offset_deg=-twist_ry,
                    ),
                    SequenceStep(f"shake_cycle_{cycle}_center", (center_x, center_y, center_z), hold),
                ]
            )
        steps.extend(
            [
                SequenceStep("shake_center_end", (center_x, center_y, center_z)),
                SequenceStep(
                    "shake_safe_retreat",
                    (center_x, center_y, center_z + approach_height),
                    phase="approach",
                ),
            ]
        )
        return self.limit_steps_for_stage(steps)

    def build_joint_steps(self) -> List[JointSequenceStep]:
        base = tuple(
            float(self.get_parameter(f"joint_shake_base_j{index}_deg").value)
            for index in range(1, 7)
        )
        j4_amp = self._clamped_abs_parameter("joint_shake_j4_amplitude_deg", 25.0)
        j5_amp = self._clamped_abs_parameter("joint_shake_j5_amplitude_deg", 30.0)
        j6_amp = self._clamped_abs_parameter("joint_shake_j6_amplitude_deg", 37.0)
        j4_cross_amp = min(j4_amp * 0.75, 18.0)
        j5_mid_amp = min(j5_amp * 0.55, 16.0)
        j5_recoil_amp = min(j5_amp * 0.35, 10.0)
        j6_cross_amp = min(j6_amp * 0.75, 27.0)
        cycles = max(int(self.get_parameter("shake_cycles").value), 1)
        hold = max(float(self.get_parameter("shake_hold_seconds").value), 0.0)

        def step(
            label: str,
            deltas: tuple[float, float, float, float, float, float],
            *,
            phase: str = "shake",
        ) -> JointSequenceStep:
            return JointSequenceStep(
                label=label,
                joints_deg=tuple(float(value + delta) for value, delta in zip(base, deltas)),
                hold_seconds=hold if phase == "shake" else 0.0,
                phase=phase,
            )

        steps: list[JointSequenceStep] = [
            JointSequenceStep("joint_shake_safe_ready", base, phase="approach"),
            JointSequenceStep("joint_shake_center_start", base, phase="approach"),
        ]
        for cycle in range(1, cycles + 1):
            # Keep shoulder/elbow fixed and make the closed-cup shake read as wrist twist.
            steps.extend(
                [
                    step(
                        f"joint_shake_cycle_{cycle}_twist_left_high",
                        (0.0, 0.0, 0.0, -j4_amp, j5_amp, j6_amp),
                    ),
                    step(
                        f"joint_shake_cycle_{cycle}_counter_twist_left",
                        (0.0, 0.0, 0.0, 0.0, j5_mid_amp, -j6_cross_amp),
                    ),
                    step(
                        f"joint_shake_cycle_{cycle}_twist_right_low",
                        (0.0, 0.0, 0.0, j4_amp, -j5_amp, -j6_amp),
                    ),
                    step(
                        f"joint_shake_cycle_{cycle}_counter_twist_right",
                        (0.0, 0.0, 0.0, 0.0, -j5_mid_amp, j6_cross_amp),
                    ),
                    step(
                        f"joint_shake_cycle_{cycle}_diagonal_snap_left",
                        (0.0, 0.0, 0.0, -j4_cross_amp, j5_recoil_amp, -j6_cross_amp),
                    ),
                    step(
                        f"joint_shake_cycle_{cycle}_diagonal_snap_right",
                        (0.0, 0.0, 0.0, j4_cross_amp, -j5_recoil_amp, j6_cross_amp),
                    ),
                    step(
                        f"joint_shake_cycle_{cycle}_j5_pop_high",
                        (0.0, 0.0, 0.0, 0.0, j5_amp, 0.0),
                    ),
                    step(
                        f"joint_shake_cycle_{cycle}_j5_pop_low",
                        (0.0, 0.0, 0.0, 0.0, -j5_amp, 0.0),
                    ),
                    step(f"joint_shake_cycle_{cycle}_center", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
                ]
            )
        steps.extend(
            [
                JointSequenceStep("joint_shake_center_end", base),
                JointSequenceStep("joint_shake_safe_retreat", base, phase="approach"),
            ]
        )
        return self.limit_steps_for_stage(steps)

    def _clamped_abs_parameter(self, name: str, max_abs: float) -> float:
        value = abs(float(self.get_parameter(name).value))
        if value > max_abs:
            self.get_logger().warning(
                f"{name}={value:.1f} is too large; clamping to {max_abs:.1f}"
            )
            return max_abs
        return value

    def limit_steps_for_stage(self, steps: Sequence[ShakeStep]) -> List[ShakeStep]:
        stage = str(self.get_parameter("execution_stage").value).strip().lower()
        if stage in {"", "all", "full"}:
            return list(steps)
        if stage == "approach":
            limited = [steps[0]]
        elif stage == "shake":
            limited = [step for step in steps if step.label != "shake_safe_retreat"]
        else:
            raise RuntimeError("unsupported execution_stage. Use one of: full, approach, shake")

        self.get_logger().warning(
            f"execution_stage={stage}: generated {len(limited)} of {len(steps)} sequence steps"
        )
        return limited

    def assert_workspace_safe(self, steps: Sequence[SequenceStep]) -> None:
        min_x = float(self.get_parameter("workspace_min_x").value)
        max_x = float(self.get_parameter("workspace_max_x").value)
        min_y = float(self.get_parameter("workspace_min_y").value)
        max_y = float(self.get_parameter("workspace_max_y").value)
        min_z = float(self.get_parameter("workspace_min_z").value)
        max_z = float(self.get_parameter("workspace_max_z").value)
        min_shake_z = float(self.get_parameter("min_shake_z").value)
        keepout_radius = float(self.get_parameter("dispenser_keepout_radius").value)
        dispensers = self.dispenser_positions()

        for step in steps:
            x, y, z = step.xyz
            if not (min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z):
                raise RuntimeError(f"{step.label} is outside workspace bounds: {step.xyz}")
            if z < min_shake_z:
                raise RuntimeError(
                    f"{step.label} z={z:.3f} is below min_shake_z={min_shake_z:.3f}"
                )
            for index, dispenser in enumerate(dispensers, start=1):
                distance = math.hypot(x - dispenser[0], y - dispenser[1])
                if distance < keepout_radius:
                    raise RuntimeError(
                        f"{step.label} is too close to dispenser {index}: "
                        f"xy_distance={distance:.3f} keepout={keepout_radius:.3f}"
                    )

        self.get_logger().info(
            f"Shake safety validated: min_z={min_shake_z:.3f} "
            f"keepout_radius={keepout_radius:.3f} workspace=ok"
        )

    def assert_joint_sequence_safe(self, steps: Sequence[JointSequenceStep]) -> None:
        joint_lower = -180.0
        joint_upper = 180.0
        joint5_lower = float(self.get_parameter("joint5_min_deg").value)
        joint5_upper = float(self.get_parameter("joint5_max_deg").value)
        j1_lower = float(self.get_parameter("joint_shake_j1_min_deg").value)
        j1_upper = float(self.get_parameter("joint_shake_j1_max_deg").value)
        j2_lower = float(self.get_parameter("joint_shake_j2_min_deg").value)
        j2_upper = float(self.get_parameter("joint_shake_j2_max_deg").value)
        j3_lower = float(self.get_parameter("joint_shake_j3_min_deg").value)
        j3_upper = float(self.get_parameter("joint_shake_j3_max_deg").value)
        max_delta = float(self.get_parameter("joint_shake_max_single_delta_deg").value)
        peak_velocity_limit = float(
            self.get_parameter("joint_shake_peak_velocity_limit_deg_s").value
        )
        shake_joint_velocity = float(self.get_parameter("shake_joint_velocity").value)
        if peak_velocity_limit > 0.0 and shake_joint_velocity > peak_velocity_limit:
            raise RuntimeError(
                "shake_joint_velocity="
                f"{shake_joint_velocity:.1f} deg/s is above "
                "joint_shake_peak_velocity_limit_deg_s="
                f"{peak_velocity_limit:.1f} deg/s"
            )

        previous: JointSequenceStep | None = None
        for step in steps:
            if len(step.joints_deg) != 6:
                raise RuntimeError(f"{step.label}: expected 6 joints, got {len(step.joints_deg)}")
            for index, value in enumerate(step.joints_deg, start=1):
                if not joint_lower <= value <= joint_upper:
                    raise RuntimeError(
                        f"{step.label}: joint_{index}={value:.3f} deg is outside "
                        f"[{joint_lower:.1f}, {joint_upper:.1f}] deg"
                    )
            if not j1_lower <= step.joints_deg[0] <= j1_upper:
                raise RuntimeError(
                    f"{step.label}: joint_1={step.joints_deg[0]:.3f} deg is outside "
                    f"safe shake range [{j1_lower:.1f}, {j1_upper:.1f}] deg"
                )
            if not j2_lower <= step.joints_deg[1] <= j2_upper:
                raise RuntimeError(
                    f"{step.label}: joint_2={step.joints_deg[1]:.3f} deg is outside "
                    f"safe shake range [{j2_lower:.1f}, {j2_upper:.1f}] deg"
                )
            if not j3_lower <= step.joints_deg[2] <= j3_upper:
                raise RuntimeError(
                    f"{step.label}: joint_3={step.joints_deg[2]:.3f} deg is outside "
                    f"safe shake range [{j3_lower:.1f}, {j3_upper:.1f}] deg"
                )
            if not joint5_lower <= step.joints_deg[4] <= joint5_upper:
                raise RuntimeError(
                    f"{step.label}: refusing MoveJoint because joint_5="
                    f"{step.joints_deg[4]:.3f} deg is outside "
                    f"[{joint5_lower:.3f}, {joint5_upper:.3f}] deg"
                )
            if previous is not None:
                deltas = [
                    abs(current - last)
                    for current, last in zip(step.joints_deg, previous.joints_deg)
                ]
                largest = max(deltas)
                if largest > max_delta:
                    raise RuntimeError(
                        f"{step.label}: largest joint step is {largest:.1f} deg, "
                        f"above joint_shake_max_single_delta_deg={max_delta:.1f}"
                    )
                step_time = self._joint_time_for_step(step)
                if step_time > 0.0 and peak_velocity_limit > 0.0:
                    estimated_peak_velocity = (2.0 * largest) / step_time
                    if estimated_peak_velocity > peak_velocity_limit:
                        raise RuntimeError(
                            f"{step.label}: final-time joint step would require peak velocity "
                            f"{estimated_peak_velocity:.1f} deg/s, above "
                            f"joint_shake_peak_velocity_limit_deg_s="
                            f"{peak_velocity_limit:.1f} deg/s. Increase shake_joint_time "
                            "or set shake_joint_time=0.0 to use velocity/acceleration mode."
                        )
            previous = step

        self.get_logger().info(
            "Joint shake safety validated: "
            f"joint_1 range=[{j1_lower:.1f}, {j1_upper:.1f}], "
            f"joint_2 range=[{j2_lower:.1f}, {j2_upper:.1f}], "
            f"joint_3 range=[{j3_lower:.1f}, {j3_upper:.1f}], "
            f"joint_5 range=[{joint5_lower:.1f}, {joint5_upper:.1f}], "
            f"max_single_delta={max_delta:.1f}, "
            f"peak_velocity_limit={peak_velocity_limit:.1f}"
        )

    def publish_joint_plan(self) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        self.path_pub.publish(path)

    def check_joint_state_validity(self, step: JointSequenceStep) -> bool:
        if not bool(self.get_parameter("require_state_validity_for_joint_shake").value):
            return True
        if self.state_validity is None:
            self.get_logger().error(
                "State-validity client is not initialized; refusing joint-space shake."
            )
            return False

        req = GetStateValidity.Request()
        req.group_name = str(self.get_parameter("planning_group").value)
        req.robot_state.joint_state = JointState()
        req.robot_state.joint_state.name = JOINT_NAMES
        req.robot_state.joint_state.position = [
            math.radians(value) for value in step.joints_deg
        ]

        future = self.state_validity.call_async(req)
        timeout_sec = max(float(self.get_parameter("motion_response_timeout_sec").value), 0.1)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"{step.label}: /check_state_validity timed out after {timeout_sec:.1f}s"
                )
                return False
            time.sleep(0.01)
        if future.exception() is not None:
            self.get_logger().error(
                f"{step.label}: /check_state_validity raised: {future.exception()}"
            )
            return False
        result = future.result()
        if result is None or not result.valid:
            contacts = getattr(result, "contacts", []) if result is not None else []
            contact_text = ""
            if contacts:
                first = contacts[0]
                contact_text = (
                    f" first_contact={getattr(first, 'contact_body_1', '?')}"
                    f"<->{getattr(first, 'contact_body_2', '?')}"
                )
            self.get_logger().error(
                f"{step.label}: MoveIt state validity is invalid; refusing MoveJoint."
                f"{contact_text}"
            )
            return False
        self.get_logger().info(f"{step.label}: MoveIt state validity OK")
        return True

    def publish_plan(self, steps: Sequence[SequenceStep]) -> None:
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

    def pos_mm_deg_for_step(self, step: SequenceStep) -> list[float]:
        base_rx = float(self.get_parameter("rx").value)
        base_ry = float(self.get_parameter("ry").value)
        base_rz = float(self.get_parameter("rz").value)
        return [
            step.xyz[0] * 1000.0,
            step.xyz[1] * 1000.0,
            step.xyz[2] * 1000.0,
            base_rx + step.rx_offset_deg,
            base_ry + step.ry_offset_deg,
            base_rz + step.rz_offset_deg,
        ]

    def precheck_ikin_joint5(self, step: SequenceStep) -> bool:
        if not bool(self.get_parameter("precheck_ikin_joint5").value):
            return True
        if self.ikin is None:
            self.get_logger().error("Ikin client is not initialized; refusing hardware shake.")
            return False

        joint5_lower = float(self.get_parameter("joint5_min_deg").value)
        joint5_upper = float(self.get_parameter("joint5_max_deg").value)
        wrist_lower = float(self.get_parameter("wrist_min_deg").value)
        wrist_upper = float(self.get_parameter("wrist_max_deg").value)
        enforce_wrist = bool(self.get_parameter("enforce_wrist_joint_limits").value)
        req = Ikin.Request()
        req.pos = self.pos_mm_deg_for_step(step)
        req.sol_space = int(self.get_parameter("ikin_sol_space").value)
        req.ref = DR_BASE

        future = self.ikin.call_async(req)
        timeout_sec = max(float(self.get_parameter("motion_response_timeout_sec").value), 0.1)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"{step.label} Ikin timed out waiting {timeout_sec:.1f}s"
                )
                return False
            time.sleep(0.01)
        if future.exception() is not None:
            self.get_logger().error(f"{step.label} Ikin raised: {future.exception()}")
            return False

        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(
                f"{step.label} Ikin returned success=false for "
                f"pos_mm_deg={[round(value, 1) for value in req.pos]}"
            )
            return False

        joints_deg = [float(value) for value in result.conv_posj]
        if len(joints_deg) < 5:
            self.get_logger().error(f"{step.label} Ikin returned too few joints: {joints_deg}")
            return False
        joint5 = joints_deg[4]
        self.get_logger().info(
            f"{step.label}: Ikin precheck sol_space={req.sol_space} "
            f"joints_deg={[round(value, 1) for value in joints_deg]}"
        )
        if not joint5_lower <= joint5 <= joint5_upper:
            self.get_logger().error(
                f"{step.label}: refusing MoveLine because predicted joint_5={joint5:.3f} deg "
                f"is outside [{joint5_lower:.3f}, {joint5_upper:.3f}] deg."
            )
            return False
        if enforce_wrist:
            for joint_index, value in ((4, joints_deg[3]), (5, joints_deg[4]), (6, joints_deg[5])):
                if not wrist_lower <= value <= wrist_upper:
                    self.get_logger().error(
                        f"{step.label}: refusing MoveLine because predicted joint_{joint_index}="
                        f"{value:.3f} deg is outside wrist limit "
                        f"[{wrist_lower:.3f}, {wrist_upper:.3f}] deg."
                    )
                    return False
        return True

    def call_movel(self, step: SequenceStep) -> bool:
        req = MoveLine.Request()
        req.pos = self.pos_mm_deg_for_step(step)
        line_time = self._line_time_for_step(step)
        velocity, acceleration = self._velocity_acceleration_for_step(step)
        req.vel = [float(velocity)] * 2
        req.acc = [float(acceleration)] * 2
        if line_time > 0.0:
            req.time = float(line_time)
        else:
            req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC

        self.get_logger().info(
            f"{step.label}: calling hardware service "
            f"pos_mm=[{req.pos[0]:.1f}, {req.pos[1]:.1f}, {req.pos[2]:.1f}, "
            f"{req.pos[3]:.1f}, {req.pos[4]:.1f}, {req.pos[5]:.1f}] "
            f"time={req.time:.2f} vel={list(req.vel)} acc={list(req.acc)}"
        )
        future = self.move_line.call_async(req)
        timeout_sec = max(float(self.get_parameter("motion_response_timeout_sec").value), 0.1)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"{step.label} timed out waiting {timeout_sec:.1f}s for service response"
                )
                return False
            time.sleep(0.01)
        if future.exception() is not None:
            self.get_logger().error(f"{step.label} service call raised: {future.exception()}")
            return False
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(
                f"{step.label} returned success=false for "
                f"pos_mm=[{req.pos[0]:.1f}, {req.pos[1]:.1f}, {req.pos[2]:.1f}, "
                f"{req.pos[3]:.1f}, {req.pos[4]:.1f}, {req.pos[5]:.1f}]. "
                "Check the Doosan controller log for the exact reject reason."
            )
            return False
        return True

    def call_movej(self, step: JointSequenceStep) -> bool:
        if self.move_joint is None:
            self.get_logger().error("MoveJoint client is not initialized; refusing hardware shake.")
            return False
        req = MoveJoint.Request()
        req.pos = [float(value) for value in step.joints_deg]
        req.vel, req.acc = self._joint_velocity_acceleration_for_step(step)
        req.time = self._joint_time_for_step(step)
        req.radius = 0.0
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC

        self.get_logger().info(
            f"{step.label}: calling hardware service "
            f"joints_deg={[round(float(value), 1) for value in req.pos]} "
            f"time={req.time:.2f} vel={req.vel:.1f} acc={req.acc:.1f}"
        )
        future = self.move_joint.call_async(req)
        timeout_sec = max(float(self.get_parameter("motion_response_timeout_sec").value), 0.1)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"{step.label} timed out waiting {timeout_sec:.1f}s for service response"
                )
                return False
            time.sleep(0.01)
        if future.exception() is not None:
            self.get_logger().error(f"{step.label} service call raised: {future.exception()}")
            return False
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(
                f"{step.label} returned success=false for "
                f"joints_deg={[round(float(value), 1) for value in req.pos]}. "
                "Check the Doosan controller log for the exact reject reason."
            )
            return False
        return True

    def wait_for_movej_done(self, step: JointSequenceStep) -> bool:
        if self.move_wait is None:
            self.get_logger().error("MoveWait client is not initialized; refusing hardware shake.")
            return False

        req = MoveWait.Request()
        timeout_sec = max(
            float(self.get_parameter("motion_response_timeout_sec").value),
            self._joint_time_for_step(step) + 3.0,
            0.1,
        )
        self.get_logger().info(f"{step.label}: waiting for MoveJoint completion")
        future = self.move_wait.call_async(req)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"{step.label} move_wait timed out after {timeout_sec:.1f}s"
                )
                return False
            time.sleep(0.01)
        if future.exception() is not None:
            self.get_logger().error(f"{step.label} move_wait raised: {future.exception()}")
            return False
        result = future.result()
        if result is not None and hasattr(result, "success") and not result.success:
            self.get_logger().error(f"{step.label} move_wait returned success=false")
            return False
        return True

    def read_current_posj(self, label: str) -> tuple[float, float, float, float, float, float] | None:
        if self.current_posj is None:
            self.get_logger().error(
                "GetCurrentPosj client is not initialized; cannot verify joint target."
            )
            return None

        future = self.current_posj.call_async(GetCurrentPosj.Request())
        timeout_sec = max(float(self.get_parameter("service_wait_timeout_sec").value), 0.1)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"{label}: get_current_posj timed out after {timeout_sec:.1f}s"
                )
                return None
            time.sleep(0.01)
        if future.exception() is not None:
            self.get_logger().error(f"{label}: get_current_posj raised: {future.exception()}")
            return None
        result = future.result()
        if result is None or not getattr(result, "success", False):
            self.get_logger().error(f"{label}: get_current_posj returned success=false")
            return None
        current = tuple(float(value) for value in result.pos[:6])
        if len(current) != 6:
            self.get_logger().error(
                f"{label}: get_current_posj returned {len(current)} joints, expected 6"
            )
            return None
        return current

    def wait_for_joint_target_reached(self, step: JointSequenceStep) -> bool:
        if not bool(self.get_parameter("verify_joint_targets").value):
            return True

        tolerance_deg = max(float(self.get_parameter("joint_target_tolerance_deg").value), 0.0)
        wait_extra_sec = max(float(self.get_parameter("joint_target_wait_extra_sec").value), 0.0)
        poll_sec = max(float(self.get_parameter("joint_target_poll_sec").value), 0.01)
        timeout_sec = max(self._joint_time_for_step(step) + wait_extra_sec, poll_sec)
        deadline = time.monotonic() + timeout_sec
        last_current: tuple[float, float, float, float, float, float] | None = None
        last_error = math.inf

        while rclpy.ok() and time.monotonic() <= deadline:
            current = self.read_current_posj(step.label)
            if current is None:
                return False
            last_current = current
            last_error = max(
                abs(float(target) - float(actual))
                for target, actual in zip(step.joints_deg, current)
            )
            self.get_logger().info(
                f"{step.label}: target_error_deg={last_error:.3f} "
                f"current_joints_deg={[round(value, 2) for value in current]}"
            )
            if last_error <= tolerance_deg:
                return True
            time.sleep(poll_sec)

        self.get_logger().error(
            f"{step.label}: MoveJoint was accepted, but current joints did not reach target "
            f"within {timeout_sec:.2f}s. max_error_deg={last_error:.3f}, "
            f"tolerance_deg={tolerance_deg:.3f}, "
            f"target_joints_deg={[round(value, 2) for value in step.joints_deg]}, "
            f"last_current_joints_deg="
            f"{[round(value, 2) for value in last_current] if last_current is not None else '<unreadable>'}. "
            "Check robot STATE_STANDBY/servo state, safe-stop/recovery, pendant hold state, "
            "and that the running Doosan bringup is connected to the real controller."
        )
        return False

    def _line_time_for_step(self, step: SequenceStep) -> float:
        global_time = max(float(self.get_parameter("line_time").value), 0.0)
        if global_time > 0.0:
            return global_time
        if step.phase == "approach":
            return max(float(self.get_parameter("approach_line_time").value), 0.0)
        return max(float(self.get_parameter("shake_line_time").value), 0.0)

    def _velocity_acceleration_for_step(self, step: SequenceStep) -> tuple[float, float]:
        if step.phase == "approach":
            return (
                float(self.get_parameter("approach_line_velocity").value),
                float(self.get_parameter("approach_line_acceleration").value),
            )
        if self.has_parameter("shake_line_velocity"):
            return (
                float(self.get_parameter("shake_line_velocity").value),
                float(self.get_parameter("shake_line_acceleration").value),
            )
        return (
            float(self.get_parameter("line_velocity").value),
            float(self.get_parameter("line_acceleration").value),
        )

    def _joint_time_for_step(self, step: JointSequenceStep) -> float:
        if step.phase == "approach":
            return max(float(self.get_parameter("approach_joint_time").value), 0.0)
        return max(float(self.get_parameter("shake_joint_time").value), 0.0)

    def _joint_velocity_acceleration_for_step(self, step: JointSequenceStep) -> tuple[float, float]:
        if step.phase == "approach":
            return (
                float(self.get_parameter("approach_joint_velocity").value),
                float(self.get_parameter("approach_joint_acceleration").value),
            )
        return (
            float(self.get_parameter("shake_joint_velocity").value),
            float(self.get_parameter("shake_joint_acceleration").value),
        )

    def execute_hardware_joint(self, steps: Sequence[JointSequenceStep]) -> bool:
        if not self.hardware_armed:
            self.get_logger().warning(
                "Hardware not armed. Dry-run only. To arm, set enable_hardware:=true, "
                f"hardware_confirm:={HARDWARE_CONFIRM_PHRASE}, and "
                "allow_service_control_without_moveit:=true."
            )
            return True

        service_timeout_sec = max(float(self.get_parameter("service_wait_timeout_sec").value), 0.1)
        service_deadline = time.monotonic() + service_timeout_sec
        while (
            rclpy.ok()
            and self.move_joint is not None
            and not self.move_joint.wait_for_service(timeout_sec=1.0)
        ):
            if time.monotonic() > service_deadline:
                self.get_logger().error(
                    f"motion/move_joint service was not available within {service_timeout_sec:.1f}s"
                )
                return False
            self.get_logger().info("Waiting for motion/move_joint")

        move_wait_deadline = time.monotonic() + service_timeout_sec
        while (
            rclpy.ok()
            and self.move_wait is not None
            and not self.move_wait.wait_for_service(timeout_sec=1.0)
        ):
            if time.monotonic() > move_wait_deadline:
                self.get_logger().error(
                    f"motion/move_wait service was not available within {service_timeout_sec:.1f}s"
                )
                return False
            self.get_logger().info("Waiting for motion/move_wait")

        if bool(self.get_parameter("verify_joint_targets").value):
            current_posj_deadline = time.monotonic() + service_timeout_sec
            while (
                rclpy.ok()
                and self.current_posj is not None
                and not self.current_posj.wait_for_service(timeout_sec=1.0)
            ):
                if time.monotonic() > current_posj_deadline:
                    self.get_logger().error(
                        "aux_control/get_current_posj service was not available within "
                        f"{service_timeout_sec:.1f}s"
                    )
                    return False
                self.get_logger().info("Waiting for aux_control/get_current_posj")

        if bool(self.get_parameter("require_state_validity_for_joint_shake").value):
            validity_deadline = time.monotonic() + service_timeout_sec
            while (
                rclpy.ok()
                and self.state_validity is not None
                and not self.state_validity.wait_for_service(timeout_sec=1.0)
            ):
                if time.monotonic() > validity_deadline:
                    self.get_logger().error(
                        f"{self.get_parameter('state_validity_service').value} service was not "
                        f"available within {service_timeout_sec:.1f}s"
                    )
                    return False
                self.get_logger().info("Waiting for MoveIt state validity service")
            self.get_logger().info(
                "Running MoveIt state-validity checks for all joint shake waypoints before MoveJoint."
            )
            for step in steps:
                if not self.check_joint_state_validity(step):
                    return False

        for step in steps:
            if not self.call_movej(step):
                return False
            if not self.wait_for_movej_done(step):
                return False
            if not self.wait_for_joint_target_reached(step):
                return False
            if step.hold_seconds > 0.0:
                time.sleep(step.hold_seconds)
        return True

    def execute_hardware(self, steps: Sequence[SequenceStep]) -> bool:
        if not self.hardware_armed:
            self.get_logger().warning(
                "Hardware not armed. Dry-run only. To arm, set enable_hardware:=true, "
                f"hardware_confirm:={HARDWARE_CONFIRM_PHRASE}, and "
                "allow_service_control_without_moveit:=true."
            )
            return True

        service_timeout_sec = max(float(self.get_parameter("service_wait_timeout_sec").value), 0.1)
        service_deadline = time.monotonic() + service_timeout_sec
        while rclpy.ok() and self.move_line is not None and not self.move_line.wait_for_service(timeout_sec=1.0):
            if time.monotonic() > service_deadline:
                self.get_logger().error(
                    f"motion/move_line service was not available within {service_timeout_sec:.1f}s"
                )
                return False
            self.get_logger().info("Waiting for motion/move_line")
        if bool(self.get_parameter("precheck_ikin_joint5").value):
            ikin_deadline = time.monotonic() + service_timeout_sec
            while rclpy.ok() and self.ikin is not None and not self.ikin.wait_for_service(timeout_sec=1.0):
                if time.monotonic() > ikin_deadline:
                    self.get_logger().error(
                        f"motion/ikin service was not available within {service_timeout_sec:.1f}s"
                    )
                    return False
                self.get_logger().info("Waiting for motion/ikin")
            self.get_logger().info(
                "Running Ikin joint_5 precheck for all shake waypoints before MoveLine. Optional joint_4/6 wrist limit is disabled unless requested."
            )
            for step in steps:
                if not self.precheck_ikin_joint5(step):
                    return False
        for step in steps:
            if not self.call_movel(step):
                return False
            if step.hold_seconds > 0.0:
                time.sleep(step.hold_seconds)
        return True

    def run_once(self) -> bool:
        if self.enable_hardware and not self.hardware_armed:
            self.get_logger().error(
                "enable_hardware was requested but hardware gates are incomplete. Refusing motion."
            )
            return False
        if self.shake_control_mode() not in {"cartesian", "pose", "movel", "joint", "joint_space", "move_joint"}:
            self.get_logger().error(
                "unsupported shake_control_mode. Use cartesian or joint."
            )
            return False
        if self.using_joint_control():
            try:
                joint_steps = self.build_joint_steps()
                self.assert_joint_sequence_safe(joint_steps)
            except (RuntimeError, ValueError) as exc:
                self.get_logger().error(str(exc))
                return False
            self.publish_joint_plan()
            for step in joint_steps:
                self.get_logger().info(
                    f"plan {step.label}: joints_deg="
                    f"{[round(float(value), 1) for value in step.joints_deg]} "
                    f"phase={step.phase} time={self._joint_time_for_step(step):.2f} "
                    f"hold={step.hold_seconds:.2f}"
                )
            return self.execute_hardware_joint(joint_steps)
        try:
            steps = self.build_steps()
            self.assert_workspace_safe(steps)
        except (RuntimeError, ValueError) as exc:
            self.get_logger().error(str(exc))
            return False
        self.publish_plan(steps)
        for step in steps:
            self.get_logger().info(
                f"plan {step.label}: x={step.xyz[0]:.3f} y={step.xyz[1]:.3f} "
                f"z={step.xyz[2]:.3f} "
                f"rpy_offset=({step.rx_offset_deg:.1f}, {step.ry_offset_deg:.1f}, {step.rz_offset_deg:.1f}) "
                f"phase={step.phase} time={self._line_time_for_step(step):.2f} "
                f"hold={step.hold_seconds:.2f}"
            )
        return self.execute_hardware(steps)


def main() -> None:
    rclpy.init()
    node = TumblerShakeSequenceNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
