#!/usr/bin/env python3
"""Run an ordered measured-dispenser recipe loop.

For each dispenser ID this composes existing field primitives:
  move/release cup at measured front-hold -> press dispenser -> re-grasp/lift cup.

All cup/dispenser positions come from measured front_hold_poses and taught
press poses used by existing nodes.  This runner does not ask for or generate
new robot coordinates.
"""

from __future__ import annotations

import argparse
import math
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import yaml
import rclpy
import tf2_ros
from azas_interfaces.srv import SetGripper
from dsr_msgs2.srv import Fkin, GetCurrentPosj, GetCurrentPosx, Ikin, MoveJoint, MoveLine, MoveWait


ROOT = Path("/home/ssu/Azas")
DEFAULT_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "measured_dispenser_collision.yaml"
CALIBRATION_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"
MOVE_FRONT_HOLD = ROOT / "tools" / "run" / "move_to_measured_dispenser_front_hold.py"
PICK_FRONT_HOLD = ROOT / "tools" / "run" / "pick_from_measured_dispenser_front_hold.py"
RG2_OPEN = ROOT / "tools" / "run" / "rg2_full_open_verify.sh"
TUMBLER_SCENE = "ros2 run azas_motion tumbler_collision_scene_node"
CONFIRM_PHRASE = "ENABLE_MEASURED_DISPENSER_RECIPE_SEQUENCE"
FRONT_HOLD_CONFIRM_PHRASE = "ENABLE_MEASURED_DISPENSER_FRONT_HOLD"
PICK_CONFIRM_PHRASE = "ENABLE_PICK_FROM_MEASURED_DISPENSER_FRONT_HOLD"
DISPENSER_TARGETS = {
    "1": "red",
    "2": "green",
    "3": "yellow",
    "4": "blue",
}

DR_BASE = 0
MOVE_MODE_ABSOLUTE = 0
SYNC = 0
BLENDING_SPEED_TYPE_DUPLICATE = 0
Pose = tuple[list[float], list[list[float]]]


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
    invalid = [value for value in values if value not in DISPENSER_TARGETS]
    if invalid:
        raise ValueError(f"unsupported dispenser id(s): {', '.join(invalid)}; allowed: 1,2,3,4")
    return values


def parse_float_list(raw: str, *, expected_count: int, label: str) -> list[float]:
    values = [part.strip() for part in raw.replace(";", ",").split(",") if part.strip()]
    if len(values) != expected_count:
        raise ValueError(f"{label} must contain {expected_count} comma-separated values")
    try:
        return [float(value) for value in values]
    except ValueError as exc:
        raise ValueError(f"{label} contains a non-numeric value: {raw!r}") from exc


def service_name(prefix: str, suffix: str) -> str:
    clean_prefix = prefix.strip("/")
    clean_suffix = suffix.strip("/")
    return f"/{clean_prefix}/{clean_suffix}" if clean_prefix else f"/{clean_suffix}"


def numeric_list(value: Any, label: str, count: int) -> list[float]:
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{label} must be a {count}-number list")
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain only numbers") from exc


def quaternion_to_matrix_xyzw(quaternion: list[float]) -> list[list[float]]:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("quaternion norm must be non-zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def matrix_to_doosan_zyz_deg(matrix: list[list[float]]) -> list[float]:
    beta = math.acos(max(-1.0, min(1.0, matrix[2][2])))
    sin_beta = math.sin(beta)
    if abs(sin_beta) > 1e-8:
        alpha = math.atan2(matrix[1][2], matrix[0][2])
        gamma = math.atan2(matrix[2][1], -matrix[2][0])
    else:
        alpha = 0.0
        gamma = math.atan2(-matrix[0][1], matrix[0][0])
    return [math.degrees(value) for value in (alpha, beta, gamma)]


def doosan_zyz_deg_to_matrix(values: list[float]) -> list[list[float]]:
    alpha, beta, gamma = [math.radians(float(value)) for value in values]
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    cg, sg = math.cos(gamma), math.sin(gamma)
    rz_alpha = [[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]]
    ry_beta = [[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]]
    rz_gamma = [[cg, -sg, 0.0], [sg, cg, 0.0], [0.0, 0.0, 1.0]]
    return matmul3(matmul3(rz_alpha, ry_beta), rz_gamma)


def matmul3(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)] for row in range(3)]


def matvec3(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3)]


def transpose3(matrix: list[list[float]]) -> list[list[float]]:
    return [[matrix[col][row] for col in range(3)] for row in range(3)]


def pose_inverse(pose: Pose) -> Pose:
    position, rotation = pose
    rotation_t = transpose3(rotation)
    return [-value for value in matvec3(rotation_t, position)], rotation_t


def pose_multiply(a: Pose, b: Pose) -> Pose:
    a_position, a_rotation = a
    b_position, b_rotation = b
    rotated_b = matvec3(a_rotation, b_position)
    return [a_position[index] + rotated_b[index] for index in range(3)], matmul3(a_rotation, b_rotation)


def transform_to_pose(transform: Any) -> Pose:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (
        [float(translation.x), float(translation.y), float(translation.z)],
        quaternion_to_matrix_xyzw([float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)]),
    )


def load_front_hold_pose(config_path: Path, dispenser_id: str) -> tuple[list[float], list[float], list[float]]:
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    poses = data.get("front_hold_poses") or {}
    key = f"dispenser_{dispenser_id}"
    block = poses.get(key)
    if not isinstance(block, dict):
        raise ValueError(f"front_hold_poses.{key} is missing in {config_path}")
    position = numeric_list(block.get("position_xyz_m"), f"front_hold_poses.{key}.position_xyz_m", 3)
    quaternion = numeric_list(block.get("quaternion_xyzw"), f"front_hold_poses.{key}.quaternion_xyzw", 4)
    return position, quaternion, matrix_to_doosan_zyz_deg(quaternion_to_matrix_xyzw(quaternion))


def load_press_pose(dispenser_id: str) -> tuple[list[float], list[float]]:
    data = yaml.safe_load(CALIBRATION_CONFIG.read_text(encoding="utf-8")) or {}
    outlets = data.get("dispenser_outlets") or {}
    block = outlets.get(str(dispenser_id))
    if not isinstance(block, dict):
        raise ValueError(f"dispenser_outlets.{dispenser_id} is missing in {CALIBRATION_CONFIG}")
    position = numeric_list(
        block.get("press_pose_xyz_m"),
        f"dispenser_outlets.{dispenser_id}.press_pose_xyz_m",
        3,
    )
    rpy_deg = numeric_list(
        block.get("press_pose_rpy_deg"),
        f"dispenser_outlets.{dispenser_id}.press_pose_rpy_deg",
        3,
    )
    return position, rpy_deg


def load_press_ready_joints_deg(dispenser_id: str) -> list[float] | None:
    data = yaml.safe_load(CALIBRATION_CONFIG.read_text(encoding="utf-8")) or {}
    outlets = data.get("dispenser_outlets") or {}
    block = outlets.get(str(dispenser_id))
    if not isinstance(block, dict):
        raise ValueError(f"dispenser_outlets.{dispenser_id} is missing in {CALIBRATION_CONFIG}")
    raw_joints = block.get("press_contact_joints_deg", block.get("press_ready_joints_deg"))
    if raw_joints is None:
        return None
    return numeric_list(
        raw_joints,
        f"dispenser_outlets.{dispenser_id}.press_contact_joints_deg",
        6,
    )


def group_consecutive_dispenser_ids(dispenser_ids: list[str]) -> list[tuple[str, int]]:
    groups: list[tuple[str, int]] = []
    for dispenser_id in dispenser_ids:
        if groups and groups[-1][0] == dispenser_id:
            previous_id, count = groups[-1]
            groups[-1] = (previous_id, count + 1)
        else:
            groups.append((dispenser_id, 1))
    return groups


class IntegratedRecipeMotion:
    """Keep ROS service clients alive across release/re-grasp loops.

    This avoids spawning the move/release and re-grasp helper scripts for every
    dispenser.  Targets still come only from measured front_hold_poses and the
    live current TCP/TF state; no cup coordinates are requested or generated.
    """

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        rclpy.init(args=None)
        self.node = rclpy.create_node("azas_integrated_dispenser_recipe_sequence")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.move_line = self.node.create_client(MoveLine, service_name(args.service_prefix, "motion/move_line"))
        self.move_joint = self.node.create_client(MoveJoint, service_name(args.service_prefix, "motion/move_joint"))
        self.move_wait = self.node.create_client(MoveWait, service_name(args.service_prefix, "motion/move_wait"))
        self.fkin = self.node.create_client(Fkin, service_name(args.service_prefix, "motion/fkin"))
        self.ikin = self.node.create_client(Ikin, service_name(args.service_prefix, "motion/ikin"))
        self.get_posj = self.node.create_client(GetCurrentPosj, service_name(args.service_prefix, "aux_control/get_current_posj"))
        self.get_posx = self.node.create_client(GetCurrentPosx, service_name(args.service_prefix, "aux_control/get_current_posx"))
        self.gripper = self.node.create_client(SetGripper, args.gripper_service)

    def close(self) -> None:
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def preflight(self) -> None:
        required = [
            (self.move_line, "MoveLine"),
            (self.move_joint, "MoveJoint"),
            (self.move_wait, "MoveWait"),
            (self.fkin, "Fkin"),
            (self.ikin, "Ikin"),
            (self.get_posj, "GetCurrentPosj"),
            (self.get_posx, "GetCurrentPosx"),
            (self.gripper, "RG2 set_width"),
        ]
        missing = [
            f"{label} ({getattr(client, 'srv_name', '<unknown service>')})"
            for client, label in required
            if not client.wait_for_service(timeout_sec=max(self.args.wait_service_sec, 0.1))
        ]
        if missing:
            raise RuntimeError("required service(s) unavailable before motion: " + ", ".join(missing))

    def _call(self, client: Any, request: Any, *, timeout_sec: float, label: str) -> Any:
        if not client.wait_for_service(timeout_sec=max(self.args.wait_service_sec, 0.1)):
            raise RuntimeError(f"{label} service not available: {getattr(client, 'srv_name', '<unknown service>')}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=max(timeout_sec, 0.1))
        if not future.done():
            raise RuntimeError(f"{label} response timeout after {timeout_sec:.1f}s")
        if future.exception() is not None:
            raise RuntimeError(f"{label} exception: {future.exception()}")
        response = future.result()
        if response is None:
            raise RuntimeError(f"{label} returned no response")
        return response

    def wait_motion_done(self, label: str, *, timeout_sec: float) -> None:
        response = self._call(
            self.move_wait,
            MoveWait.Request(),
            timeout_sec=timeout_sec,
            label=f"MoveWait {label}",
        )
        if not response.success:
            raise RuntimeError(f"MoveWait returned success=false for {label}")

    def current_posx(self, timeout_sec: float | None = None) -> list[float]:
        timeout = timeout_sec or self.args.wait_service_sec
        last_error = ""
        for attempt in range(1, max(int(self.args.pose_read_retries), 1) + 1):
            try:
                req = GetCurrentPosx.Request()
                req.ref = DR_BASE
                response = self._call(
                    self.get_posx,
                    req,
                    timeout_sec=timeout,
                    label="GetCurrentPosx",
                )
                if not response.success or not response.task_pos_info:
                    raise RuntimeError("GetCurrentPosx returned success=false or empty task_pos_info")
                values = list(response.task_pos_info[0].data)
                if len(values) < 6:
                    raise RuntimeError(f"GetCurrentPosx returned too few values: {values}")
                return [float(value) for value in values[:6]]
            except RuntimeError as exc:
                last_error = str(exc)
                if attempt >= max(int(self.args.pose_read_retries), 1):
                    break
                print(
                    f"[Azas] GetCurrentPosx retry {attempt}/{int(self.args.pose_read_retries)}: {last_error}",
                    file=sys.stderr,
                )
                time.sleep(max(float(self.args.pose_read_retry_sleep_sec), 0.0))
        raise RuntimeError(last_error or "GetCurrentPosx failed")

    def current_posj(self, timeout_sec: float | None = None) -> list[float]:
        timeout = timeout_sec or self.args.wait_service_sec
        last_error = ""
        for attempt in range(1, max(int(self.args.pose_read_retries), 1) + 1):
            try:
                response = self._call(
                    self.get_posj,
                    GetCurrentPosj.Request(),
                    timeout_sec=timeout,
                    label="GetCurrentPosj",
                )
                values = list(response.pos)
                if not response.success or len(values) < 6:
                    raise RuntimeError("GetCurrentPosj returned success=false or too few joint values")
                return [float(value) for value in values[:6]]
            except RuntimeError as exc:
                last_error = str(exc)
                if attempt >= max(int(self.args.pose_read_retries), 1):
                    break
                print(
                    f"[Azas] GetCurrentPosj retry {attempt}/{int(self.args.pose_read_retries)}: {last_error}",
                    file=sys.stderr,
                )
                time.sleep(max(float(self.args.pose_read_retry_sleep_sec), 0.0))
        raise RuntimeError(last_error or "GetCurrentPosj failed")

    def current_tcp_pose(self) -> Pose:
        values = self.current_posx()
        return [values[index] / 1000.0 for index in range(3)], doosan_zyz_deg_to_matrix(values[3:6])

    def lookup_pose(self, target_frame: str, source_frame: str, timeout_sec: float) -> Pose:
        deadline = time.monotonic() + max(timeout_sec, 0.1)
        last_error = ""
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            try:
                return transform_to_pose(self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time()))
            except Exception as exc:  # tf2 exception types vary by install.
                last_error = str(exc)
                time.sleep(0.05)
        raise RuntimeError(f"TF lookup {target_frame}->{source_frame} timed out: {last_error}")

    def compensate_current_tcp(self, desired_link6_position: list[float], quaternion: list[float]) -> Pose:
        desired_link6_pose = (desired_link6_position, quaternion_to_matrix_xyzw(quaternion))
        live_link6_pose = self.lookup_pose("base_link", "link_6", max(self.args.wait_service_sec, 0.1))
        link6_to_tcp = pose_multiply(pose_inverse(live_link6_pose), self.current_tcp_pose())
        return pose_multiply(desired_link6_pose, link6_to_tcp)

    def move_front_hold(
        self,
        dispenser_id: str,
        *,
        label: str,
        offset_x_m: float,
        offset_y_m: float,
        offset_z_m: float,
        velocity: float,
        acceleration: float,
        prefer_joint: bool = False,
    ) -> None:
        position, quaternion, raw_zyz = load_front_hold_pose(self.args.config, dispenser_id)
        link6_position = [
            position[0] + offset_x_m,
            position[1] + offset_y_m,
            position[2] + offset_z_m,
        ]
        move_position, move_rotation = self.compensate_current_tcp(link6_position, quaternion)
        move_zyz = matrix_to_doosan_zyz_deg(move_rotation)
        pos = [move_position[0] * 1000.0, move_position[1] * 1000.0, move_position[2] * 1000.0, *move_zyz]
        print(
            f"[Azas] {label}: dispenser={dispenser_id} "
            f"link6_target_m=[{link6_position[0]:.4f}, {link6_position[1]:.4f}, {link6_position[2]:.4f}] "
            f"raw_zyz_deg=[{raw_zyz[0]:.2f}, {raw_zyz[1]:.2f}, {raw_zyz[2]:.2f}]"
        )
        if self.args.precheck_ikin:
            req = Ikin.Request()
            req.pos = pos
            req.sol_space = int(self.args.ikin_sol_space)
            req.ref = DR_BASE
            response = self._call(self.ikin, req, timeout_sec=self.args.wait_service_sec, label="Ikin")
            if not response.success:
                raise RuntimeError(f"Ikin failed for {label}")
        if prefer_joint:
            print(
                f"[Azas] {label}: using IK MoveJoint for transit, not Cartesian MoveLine, "
                "to avoid a straight TCP path through dispenser/bottle geometry"
            )
            self.move_front_hold_joint_fallback(pos, label=label)
            return
        req = MoveLine.Request()
        req.pos = pos
        req.vel = [velocity, velocity]
        req.acc = [acceleration, acceleration]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        response = self._call(self.move_line, req, timeout_sec=self.args.move_timeout_sec, label=f"MoveLine {label}")
        if not response.success:
            if not self.args.front_hold_joint_fallback:
                raise RuntimeError(f"MoveLine returned success=false for {label}")
            print(
                f"[WARN] MoveLine returned success=false for {label}; "
                "retrying same measured target with IK MoveJoint fallback"
            )
            self.move_front_hold_joint_fallback(pos, label=label)
            return
        self.wait_motion_done(label, timeout_sec=self.args.move_timeout_sec)
        try:
            self.wait_for_target(pos, label=label)
        except RuntimeError as exc:
            if not self.args.front_hold_joint_fallback:
                raise
            print(
                f"[WARN] MoveLine verification failed for {label}: {exc}; "
                "retrying same measured target with IK MoveJoint fallback"
            )
            self.move_front_hold_joint_fallback(pos, label=label)

    def move_posx_joint_fallback(
        self,
        posx_mm_deg: list[float],
        *,
        label: str,
        velocity: float,
        acceleration: float,
    ) -> None:
        joints_deg = self.ikin_posj(posx_mm_deg, label=f"{label} IK joint fallback")
        self.validate_ik_fallback_joints(joints_deg, label=label)
        self.movej(
            joints_deg,
            label=f"{label} IK MoveJoint fallback",
            velocity=velocity,
            acceleration=acceleration,
        )
        self.wait_for_target(posx_mm_deg, label=f"{label} IK MoveJoint fallback posx")

    def move_front_hold_joint_fallback(self, posx_mm_deg: list[float], *, label: str) -> None:
        joints_deg = self.ikin_posj(posx_mm_deg, label=f"{label} IK joint fallback")
        self.validate_ik_fallback_joints(joints_deg, label=label)
        self.movej(
            joints_deg,
            label=f"{label} IK MoveJoint fallback",
            velocity=self.args.front_hold_joint_fallback_velocity,
            acceleration=self.args.front_hold_joint_fallback_acceleration,
        )
        self.wait_for_target(posx_mm_deg, label=f"{label} IK MoveJoint fallback posx")

    def safe_lift_current(
        self,
        *,
        label: str,
        min_z_m: float,
        velocity: float,
        acceleration: float,
        timeout_sec: float,
    ) -> None:
        pose = self.current_posx()
        target_z_mm = max(pose[2], max(min_z_m, 0.0) * 1000.0)
        if target_z_mm <= pose[2] + 1.0:
            print(
                f"[Azas] {label}: already above safe transit z "
                f"current_z={pose[2] / 1000.0:.3f}m min_z={min_z_m:.3f}m"
            )
            return
        target = [pose[0], pose[1], target_z_mm, pose[3], pose[4], pose[5]]
        try:
            self.move_posx(
                target,
                label=label,
                velocity=velocity,
                acceleration=acceleration,
                timeout_sec=timeout_sec,
            )
        except RuntimeError as exc:
            if not self.args.safe_lift_joint_fallback:
                raise
            print(
                f"[WARN] MoveLine safe lift failed for {label}: {exc}; "
                "retrying the same high-Z target with IK MoveJoint fallback"
            )
            self.move_posx_joint_fallback(
                target,
                label=label,
                velocity=self.args.safe_lift_joint_fallback_velocity,
                acceleration=self.args.safe_lift_joint_fallback_acceleration,
            )

    def move_posx(
        self,
        pos: list[float],
        *,
        label: str,
        velocity: float,
        acceleration: float,
        timeout_sec: float,
    ) -> None:
        print(
            f"[Azas] {label}: posx=[{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}, "
            f"{pos[3]:.1f}, {pos[4]:.1f}, {pos[5]:.1f}]"
        )
        req = MoveLine.Request()
        req.pos = pos
        req.vel = [velocity, velocity]
        req.acc = [acceleration, acceleration]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        response = self._call(self.move_line, req, timeout_sec=timeout_sec, label=f"MoveLine {label}")
        if not response.success:
            raise RuntimeError(f"MoveLine returned success=false for {label}")
        self.wait_motion_done(label, timeout_sec=timeout_sec)
        self.wait_for_target(pos, label=label)

    def move_posx_no_verify(
        self,
        pos: list[float],
        *,
        label: str,
        velocity: float,
        acceleration: float,
        timeout_sec: float,
    ) -> None:
        print(
            f"[Azas] {label}: posx=[{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}, "
            f"{pos[3]:.1f}, {pos[4]:.1f}, {pos[5]:.1f}]"
        )
        req = MoveLine.Request()
        req.pos = pos
        req.vel = [velocity, velocity]
        req.acc = [acceleration, acceleration]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        response = self._call(self.move_line, req, timeout_sec=timeout_sec, label=f"MoveLine {label}")
        if not response.success:
            raise RuntimeError(f"MoveLine returned success=false for {label}")
        self.wait_motion_done(label, timeout_sec=timeout_sec)

    def movej_no_verify(self, joints_deg: list[float], *, label: str, velocity: float, acceleration: float) -> None:
        print(
            "[Azas] "
            + label
            + ": movej_deg=["
            + ", ".join(f"{value:.1f}" for value in joints_deg)
            + "]"
        )
        req = MoveJoint.Request()
        req.pos = [float(value) for value in joints_deg]
        req.vel = float(velocity)
        req.acc = float(acceleration)
        req.time = 0.0
        req.radius = 0.0
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        response = self._call(self.move_joint, req, timeout_sec=self.args.press_timeout_sec, label=f"MoveJoint {label}")
        if not response.success:
            raise RuntimeError(f"MoveJoint returned success=false for {label}")
        self.wait_motion_done(label, timeout_sec=self.args.press_timeout_sec)

    def movej(self, joints_deg: list[float], *, label: str, velocity: float, acceleration: float) -> None:
        print(
            "[Azas] "
            + label
            + ": movej_deg=["
            + ", ".join(f"{value:.1f}" for value in joints_deg)
            + "]"
        )
        req = MoveJoint.Request()
        req.pos = [float(value) for value in joints_deg]
        req.vel = float(velocity)
        req.acc = float(acceleration)
        req.time = 0.0
        req.radius = 0.0
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        response = self._call(self.move_joint, req, timeout_sec=self.args.press_timeout_sec, label=f"MoveJoint {label}")
        if not response.success:
            raise RuntimeError(f"MoveJoint returned success=false for {label}")
        self.wait_motion_done(label, timeout_sec=self.args.press_timeout_sec)
        self.wait_for_joint_target(joints_deg, label=label)

    def fkin_posx(self, joints_deg: list[float], *, label: str) -> list[float]:
        req = Fkin.Request()
        req.pos = [float(value) for value in joints_deg]
        req.ref = DR_BASE
        response = self._call(self.fkin, req, timeout_sec=self.args.wait_service_sec, label=f"Fkin {label}")
        if not response.success:
            raise RuntimeError(f"Fkin returned success=false for {label}")
        values = [float(value) for value in response.conv_posx[:6]]
        if len(values) < 6:
            raise RuntimeError(f"Fkin returned too few posx values for {label}: {values}")
        print(
            f"[Azas] {label}: fkin_posx=[{values[0]:.1f}, {values[1]:.1f}, {values[2]:.1f}, "
            f"{values[3]:.1f}, {values[4]:.1f}, {values[5]:.1f}]"
        )
        return values

    def ikin_posj(self, posx_mm_deg: list[float], *, label: str) -> list[float]:
        req = Ikin.Request()
        req.pos = [float(value) for value in posx_mm_deg]
        req.sol_space = int(self.args.ikin_sol_space)
        req.ref = DR_BASE
        response = self._call(self.ikin, req, timeout_sec=self.args.wait_service_sec, label=f"Ikin {label}")
        if not response.success:
            raise RuntimeError(f"Ikin returned success=false for {label}")
        values = [float(value) for value in response.conv_posj[:6]]
        if len(values) < 6:
            raise RuntimeError(f"Ikin returned too few posj values for {label}: {values}")
        print(
            f"[Azas] {label}: ikin_posj=["
            + ", ".join(f"{value:.1f}" for value in values)
            + "]"
        )
        return values

    def validate_ik_fallback_joints(self, joints_deg: list[float], *, label: str) -> None:
        max_abs = max(float(self.args.ik_fallback_max_abs_joint_deg), 0.0)
        if max_abs > 0.0:
            for index, value in enumerate(joints_deg, start=1):
                if abs(value) > max_abs:
                    raise RuntimeError(
                        f"IK fallback rejected for {label}: joint_{index}={value:.1f}deg "
                        f"exceeds limit {max_abs:.1f}deg"
                    )
        max_delta = max(float(self.args.ik_fallback_max_joint_delta_deg), 0.0)
        if max_delta <= 0.0:
            return
        current = self.current_posj(timeout_sec=5.0)
        deltas = [abs(joints_deg[index] - current[index]) for index in range(6)]
        worst_delta = max(deltas)
        if worst_delta > max_delta:
            joint_index = deltas.index(worst_delta) + 1
            raise RuntimeError(
                f"IK fallback rejected for {label}: joint_{joint_index} delta "
                f"{worst_delta:.1f}deg exceeds limit {max_delta:.1f}deg"
            )

    def wait_for_joint_target(self, target_joints_deg: list[float], *, label: str) -> None:
        deadline = time.monotonic() + max(self.args.verify_timeout_sec, 0.1)
        last_error = 999999.0
        best_error = last_error
        last_progress_time = time.monotonic()
        while time.monotonic() < deadline:
            actual = self.current_posj(timeout_sec=5.0)
            errors = [abs(actual[index] - target_joints_deg[index]) for index in range(6)]
            last_error = max(errors)
            print(
                f"[Azas] verify {label}: max_joint_error={last_error:.2f}deg "
                f"j6={actual[5]:.2f}deg tolerance={self.args.joint_target_tolerance_deg:.2f}deg"
            )
            if last_error <= max(self.args.joint_target_tolerance_deg, 0.1):
                return
            if best_error - last_error >= max(self.args.target_stall_delta_mm, 0.1):
                best_error = last_error
                last_progress_time = time.monotonic()
            elif (
                self.args.target_stall_timeout_sec > 0.0
                and last_error >= max(self.args.joint_target_tolerance_deg, 0.1)
                and time.monotonic() - last_progress_time >= max(self.args.target_stall_timeout_sec, 0.0)
            ):
                raise RuntimeError(
                    f"joint target verification stalled for {label}; "
                    f"max_error={last_error:.2f}deg best={best_error:.2f}deg "
                    f"no_progress_for={time.monotonic() - last_progress_time:.1f}s"
                )
            time.sleep(max(self.args.verify_poll_seconds, 0.05))
        raise RuntimeError(f"joint target verification timeout for {label}; max_error={last_error:.2f}deg")

    def wait_for_target(self, target_pos_mm_deg: list[float], *, label: str) -> None:
        deadline = time.monotonic() + max(self.args.verify_timeout_sec, 0.1)
        last_distance = 999999.0
        best_distance = last_distance
        last_progress_time = time.monotonic()
        while time.monotonic() < deadline:
            actual = self.current_posx(timeout_sec=5.0)
            last_distance = sum((actual[index] - target_pos_mm_deg[index]) ** 2 for index in range(3)) ** 0.5
            print(f"[Azas] verify {label}: distance={last_distance:.1f}mm tolerance={self.args.target_tolerance_mm:.1f}mm")
            if last_distance <= max(self.args.target_tolerance_mm, 0.1):
                return
            if best_distance - last_distance >= max(self.args.target_stall_delta_mm, 0.1):
                best_distance = last_distance
                last_progress_time = time.monotonic()
            elif (
                self.args.target_stall_timeout_sec > 0.0
                and last_distance >= max(self.args.target_stall_min_distance_mm, self.args.target_tolerance_mm)
                and time.monotonic() - last_progress_time >= max(self.args.target_stall_timeout_sec, 0.0)
            ):
                raise RuntimeError(
                    f"target verification stalled for {label}; "
                    f"distance={last_distance:.1f}mm best={best_distance:.1f}mm "
                    f"no_progress_for={time.monotonic() - last_progress_time:.1f}s"
                )
            time.sleep(max(self.args.verify_poll_seconds, 0.05))
        raise RuntimeError(f"target verification timeout for {label}; distance={last_distance:.1f}mm")

    def gripper_command(self, command: str, *, width_m: float, force_n: float, label: str) -> None:
        req = SetGripper.Request()
        req.command = command
        req.width_m = float(width_m)
        req.force_n = float(force_n)
        response = self._call(self.gripper, req, timeout_sec=self.args.gripper_timeout_sec, label=label)
        if not response.success:
            raise RuntimeError(f"{label} returned success=false: {response.message}")
        print(f"[Azas] {label}: {response.message}")
        settle_sec = max(
            self.args.gripper_open_settle_seconds if command == "open" else self.args.gripper_settle_seconds,
            0.0,
        )
        if settle_sec > 0.0:
            print(f"[Azas] {label}: waiting {settle_sec:.2f}s for physical RG2 motion to settle")
            time.sleep(settle_sec)

    def move_and_release(self, dispenser_id: str) -> None:
        stages = [
            (
                "pre-hold",
                self.args.move_prehold_offset_x_m,
                self.args.move_prehold_offset_y_m,
                self.args.move_prehold_offset_z_m,
                self.args.move_prehold_velocity,
                self.args.move_prehold_acceleration,
            ),
            (
                "above-hold",
                self.args.move_prehold_offset_x_m,
                self.args.move_prehold_offset_y_m,
                self.args.move_prehold_offset_z_m,
                self.args.move_prehold_velocity,
                self.args.move_prehold_acceleration,
            ),
            (
                "front-hold",
                self.args.move_release_offset_x_m,
                self.args.move_release_offset_y_m,
                self.args.move_release_offset_z_m,
                self.args.move_velocity,
                self.args.move_acceleration,
            ),
        ]
        seen: set[tuple[float, float, float, float, float]] = set()
        for stage_label, offset_x, offset_y, offset_z, velocity, acceleration in stages:
            key = (offset_x, offset_y, offset_z, velocity, acceleration)
            if key in seen:
                continue
            seen.add(key)
            self.move_front_hold(
                dispenser_id,
                label=stage_label,
                offset_x_m=offset_x,
                offset_y_m=offset_y,
                offset_z_m=offset_z,
                velocity=velocity,
                acceleration=acceleration,
            )
        self.gripper_command(
            "open",
            width_m=self.args.gripper_open_width_m,
            force_n=self.args.gripper_open_force_n,
            label="RG2 full-open release",
        )
        print("[Azas] RG2 full-open release complete; continuing only after open settle wait")

    def regrasp_and_lift(self, dispenser_id: str) -> None:
        self.safe_lift_current(
            label="safe vertical lift after press before re-grasp transit",
            min_z_m=self.args.regrasp_min_transit_z_m,
            velocity=self.args.regrasp_approach_velocity,
            acceleration=self.args.regrasp_approach_acceleration,
            timeout_sec=self.args.move_timeout_sec,
        )
        if abs(self.args.regrasp_retreat_y_m) > 1e-6 or abs(self.args.regrasp_retreat_x_m) > 1e-6:
            pose = self.current_posx()
            retreat = [
                pose[0] + self.args.regrasp_retreat_x_m * 1000.0,
                pose[1] + self.args.regrasp_retreat_y_m * 1000.0,
                pose[2],
                pose[3],
                pose[4],
                pose[5],
            ]
            self.move_posx(
                retreat,
                label="safe lateral retreat away from dispenser before re-grasp transit",
                velocity=self.args.regrasp_approach_velocity,
                acceleration=self.args.regrasp_approach_acceleration,
                timeout_sec=self.args.move_timeout_sec,
            )
        front_hold_position, _, _ = load_front_hold_pose(self.args.config, dispenser_id)
        released_hold_z_m = front_hold_position[2] + self.args.move_release_offset_z_m
        desired_approach_z_m = max(
            released_hold_z_m + max(self.args.regrasp_approach_offset_z_m, 0.0),
            max(self.args.regrasp_min_transit_z_m, 0.0),
        )
        capped_approach_z_m = min(desired_approach_z_m, max(self.args.regrasp_max_transit_z_m, 0.0))
        if capped_approach_z_m < desired_approach_z_m:
            print(
                f"[WARN] capping re-grasp high approach z from "
                f"{desired_approach_z_m:.3f}m to {capped_approach_z_m:.3f}m"
            )
        approach_offset_z_m = max(capped_approach_z_m - front_hold_position[2], 0.0)
        self.move_front_hold(
            dispenser_id,
            label="final re-grasp high transit above front-hold",
            offset_x_m=self.args.move_release_offset_x_m,
            offset_y_m=self.args.move_release_offset_y_m,
            offset_z_m=approach_offset_z_m,
            velocity=self.args.regrasp_approach_velocity,
            acceleration=self.args.regrasp_approach_acceleration,
            prefer_joint=self.args.regrasp_high_transit_joint,
        )
        self.gripper_command(
            "open",
            width_m=self.args.gripper_open_width_m,
            force_n=self.args.gripper_open_force_n,
            label="RG2 open at safe high re-grasp approach",
        )
        self.move_front_hold(
            dispenser_id,
            label="final re-grasp front-hold",
            offset_x_m=self.args.move_release_offset_x_m,
            offset_y_m=self.args.move_release_offset_y_m,
            offset_z_m=self.args.move_release_offset_z_m,
            velocity=self.args.pick_approach_velocity,
            acceleration=self.args.pick_approach_acceleration,
        )
        self.gripper_command(
            "set_width",
            width_m=self.args.gripper_grasp_width_m,
            force_n=self.args.gripper_force_n,
            label="RG2 soft side-grasp",
        )
        pose = self.current_posx()
        target = [pose[0], pose[1], pose[2] + max(self.args.pick_lift_m, 0.0) * 1000.0, pose[3], pose[4], pose[5]]
        req = MoveLine.Request()
        req.pos = target
        req.vel = [self.args.pick_lift_velocity, self.args.pick_lift_velocity]
        req.acc = [self.args.pick_lift_acceleration, self.args.pick_lift_acceleration]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        response = self._call(self.move_line, req, timeout_sec=self.args.pick_timeout_sec, label="post-grasp lift")
        if not response.success:
            raise RuntimeError("post-grasp lift returned success=false")
        self.wait_motion_done("post-grasp lift", timeout_sec=self.args.pick_timeout_sec)
        self.wait_for_target(target, label="post-grasp lift")

    def press_dispenser(self, dispenser_id: str, press_count: int) -> None:
        press_xyz_m, press_rpy_deg = load_press_pose(dispenser_id)
        current_pose = self.current_posx()
        contact_joints = load_press_ready_joints_deg(dispenser_id)
        joint_space_press = contact_joints is not None
        if contact_joints is None:
            x_mm = press_xyz_m[0] * 1000.0
            y_mm = press_xyz_m[1] * 1000.0
            contact_z = press_xyz_m[2] * 1000.0
            rx, ry, rz = press_rpy_deg
            print(
                f"[Azas] dispenser {dispenser_id}: no press contact joints in calibration; "
                "falling back to press_pose_xyz_m/rpy_deg"
            )
        else:
            contact_joints = list(contact_joints)
            if self.args.press_force_joint6_zero:
                before_j6 = contact_joints[5]
                contact_joints[5] = 0.0
                print(
                    f"[Azas] dispenser {dispenser_id}: forcing press contact joint_6/link_6 "
                    f"from {before_j6:.2f}deg to 0.00deg before FK"
                )
            else:
                print(
                    f"[Azas] dispenser {dispenser_id}: using measured press contact joints exactly "
                    f"(joint_6/link_6={contact_joints[5]:.2f}deg)"
                )
            x_mm = press_xyz_m[0] * 1000.0
            y_mm = press_xyz_m[1] * 1000.0
            contact_z = press_xyz_m[2] * 1000.0
            rx, ry, rz = press_rpy_deg
        if joint_space_press:
            # Do not depend on /motion/fkin here.  The Doosan FK service can
            # block on this setup, and the measured joint position itself is
            # the authoritative press contact pose.  MoveJoint to the measured
            # pose first, then read the live TCP as the contact reference.
            pre_z = contact_z + max(self.args.press_pre_lift_m, 0.0) * 1000.0
            transit_z = max(
                current_pose[2] + max(self.args.press_transit_height_m, self.args.press_pre_lift_m, 0.0) * 1000.0,
                min(pre_z, max(self.args.press_min_transit_z_m, 0.0) * 1000.0),
            )
            pressed_z = contact_z - max(self.args.press_depth_m, 0.0) * 1000.0
            print(
                "[Azas] integrated press: "
                f"dispenser={dispenser_id} count={press_count} "
                f"configured_contact=({x_mm:.1f}, {y_mm:.1f}, {contact_z:.1f}) "
                f"configured_pre_z={pre_z:.1f} configured_pressed_z={pressed_z:.1f} "
                f"z_descent={contact_z - pressed_z:.1f}mm transit_z={transit_z:.1f} "
                "source=calibration pre before measured MoveJoint"
            )
        else:
            pre_z = contact_z + max(self.args.press_pre_lift_m, 0.0) * 1000.0
            pressed_z = contact_z - max(self.args.press_depth_m, 0.0) * 1000.0
            transit_z = max(current_pose[2], pre_z) + max(self.args.press_transit_height_m, 0.0) * 1000.0
            print(
                "[Azas] integrated press: "
                f"dispenser={dispenser_id} count={press_count} "
                f"contact=({x_mm:.1f}, {y_mm:.1f}, {contact_z:.1f}) "
                f"pre_z={pre_z:.1f} pressed_z={pressed_z:.1f} transit_z={transit_z:.1f}"
            )
        if abs(self.args.press_pre_lift_retreat_x_m) > 1e-6 or abs(self.args.press_pre_lift_retreat_y_m) > 1e-6:
            retreat = [
                current_pose[0] + self.args.press_pre_lift_retreat_x_m * 1000.0,
                current_pose[1] + self.args.press_pre_lift_retreat_y_m * 1000.0,
                current_pose[2],
                current_pose[3],
                current_pose[4],
                current_pose[5],
            ]
            self.move_posx(
                retreat,
                label="safe lateral retreat away from dispenser before press lift",
                velocity=self.args.press_travel_velocity,
                acceleration=self.args.press_travel_acceleration,
                timeout_sec=self.args.press_timeout_sec,
            )
            current_pose = self.current_posx(timeout_sec=self.args.wait_service_sec)
        safe_lift = [
            current_pose[0],
            current_pose[1],
            transit_z,
            current_pose[3],
            current_pose[4],
            current_pose[5],
        ]
        self.move_posx(
            safe_lift,
            label="safe lift away from released cup before press",
            velocity=self.args.press_travel_velocity,
            acceleration=self.args.press_travel_acceleration,
            timeout_sec=self.args.press_timeout_sec,
        )
        self.gripper_command(
            "set_width",
            width_m=self.args.press_gripper_close_width_m,
            force_n=self.args.press_gripper_force_n,
            label="RG2 close empty gripper for dispenser press",
        )
        if self.args.press_reset_before_press:
            # The operator requirement is explicit: after releasing the cup and
            # closing the empty RG2, rotate the gripper/link_6 back to 0 deg
            # from a safe high pose.  Do not use a Cartesian orientation move
            # for this reset, because Doosan IK may swing wrist joint 4/5.
            safe_joints = self.current_posj()
            reset_joints = list(safe_joints)
            reset_joints[5] = 0.0
            self.movej(
                reset_joints,
                label="reset link_6/joint_6 to 0 at safe height",
                velocity=self.args.press_reset_joint_velocity,
                acceleration=self.args.press_reset_joint_acceleration,
            )

        steps: list[tuple[list[float], str, float, float]] = []
        if joint_space_press:
            if self.args.press_move_configured_prepose_before_joint:
                self.move_posx(
                    [x_mm, y_mm, pre_z, rx, ry, rz],
                    label="high pre pose before measured press joint",
                    velocity=self.args.press_travel_velocity,
                    acceleration=self.args.press_travel_acceleration,
                    timeout_sec=self.args.press_timeout_sec,
                )
            else:
                print(
                    "[Azas] joint-space press: skipping configured Cartesian pre pose; "
                    "measured press contact joints are authoritative"
                )
            self.movej(
                contact_joints,
                label="move to measured press contact joints exactly",
                velocity=self.args.press_contact_joint_velocity,
                acceleration=self.args.press_contact_joint_acceleration,
            )
            contact_posx = self.current_posx(timeout_sec=self.args.wait_service_sec)
            x_mm, y_mm, contact_z, rx, ry, rz = contact_posx
            pre_z = contact_z + max(self.args.press_pre_lift_m, 0.0) * 1000.0
            pressed_z = contact_z - max(self.args.press_depth_m, 0.0) * 1000.0
            print(
                "[Azas] integrated press: "
                f"dispenser={dispenser_id} count={press_count} "
                f"contact=({x_mm:.1f}, {y_mm:.1f}, {contact_z:.1f}) "
                f"pre_z={pre_z:.1f} pressed_z={pressed_z:.1f} "
                f"z_descent={contact_z - pressed_z:.1f}mm transit_z={transit_z:.1f} "
                "source=live TCP after measured MoveJoint"
            )
            if self.args.press_joint_space_use_high_prepose:
                steps.append(
                    (
                        [x_mm, y_mm, pre_z, rx, ry, rz],
                        "pre pose above dispenser head",
                        self.args.press_line_velocity,
                        self.args.press_line_acceleration,
                    )
                )
            else:
                print(
                    "[Azas] joint-space press: skipping high Cartesian pre pose after measured contact; "
                    "pump will run contact -> pressed_z -> contact to avoid the 300mm pre-pose stall"
                )
                pre_z = contact_z
        else:
            steps.extend(
                [
                    (
                        [x_mm, y_mm, transit_z, rx, ry, rz],
                        "align above measured press contact",
                        self.args.press_travel_velocity,
                        self.args.press_travel_acceleration,
                    ),
                    (
                        [x_mm, y_mm, pre_z, rx, ry, rz],
                        "pre pose above dispenser head",
                        self.args.press_travel_velocity,
                        self.args.press_travel_acceleration,
                    ),
                ]
            )
        for press_index in range(1, max(int(press_count), 1) + 1):
            suffix = f" {press_index}/{press_count}" if press_count > 1 else ""
            if max(self.args.press_depth_m, 0.0) == 0.0:
                steps.append(
                    (
                        [x_mm, y_mm, contact_z, rx, ry, rz],
                        f"press dispenser pump{suffix}",
                        self.args.press_line_velocity,
                        self.args.press_line_acceleration,
                    )
                )
            else:
                steps.extend(
                    [
                        (
                            [x_mm, y_mm, contact_z, rx, ry, rz],
                            f"move to measured contact pose{suffix}",
                            self.args.press_line_velocity,
                            self.args.press_line_acceleration,
                        ),
                        (
                            [x_mm, y_mm, pressed_z, rx, ry, rz],
                            f"press dispenser pump{suffix}",
                            self.args.press_line_velocity,
                            self.args.press_line_acceleration,
                        ),
                    ]
                )
            steps.append(
                (
                    [x_mm, y_mm, pre_z, rx, ry, rz],
                    f"retreat above dispenser{suffix}",
                    self.args.press_line_velocity,
                    self.args.press_line_acceleration,
                )
            )
        if self.args.press_post_retreat_after_sequence and joint_space_press:
            print(
                "[Azas] joint-space press: skipping Cartesian post-retreat away from dispenser; "
                "measured press joints are already authoritative and the lateral retreat can stall "
                "real hardware verification before the re-grasp step"
            )
        elif self.args.press_post_retreat_after_sequence:
            steps.append(
                (
                    [
                        x_mm + self.args.press_post_retreat_dx_m * 1000.0,
                        y_mm + self.args.press_post_retreat_dy_m * 1000.0,
                        (transit_z if joint_space_press and not self.args.press_joint_space_use_high_prepose else pre_z),
                        rx,
                        ry,
                        rz,
                    ],
                    "retreat away from dispenser",
                    self.args.press_travel_velocity,
                    self.args.press_travel_acceleration,
                )
            )
        for pos, label, velocity, acceleration in steps:
            self.move_posx(
                pos,
                label=label,
                velocity=velocity,
                acceleration=acceleration,
                timeout_sec=self.args.press_timeout_sec,
            )
            if label.startswith("press dispenser pump") and self.args.press_hold_seconds > 0.0:
                time.sleep(self.args.press_hold_seconds)
        if self.args.press_post_retreat_wait_seconds > 0.0:
            time.sleep(self.args.press_post_retreat_wait_seconds)


def run_command(label: str, cmd: list[str] | str) -> int:
    print(f"[Azas] === {label} ===")
    if isinstance(cmd, list):
        print("[Azas] command=" + " ".join(shlex.quote(part) for part in cmd))
    else:
        print(f"[Azas] command={cmd}")
    sys.stdout.flush()
    result = subprocess.run(cmd, cwd=str(ROOT), shell=isinstance(cmd, str), check=False)
    if result.returncode != 0:
        print(f"[FAIL] {label} failed with returncode={result.returncode}")
    return result.returncode


def tumbler_scene_cmd(action: str, *, object_id: str, dispenser_id: str = "1") -> str:
    return (
        f"timeout 5s {TUMBLER_SCENE} --ros-args "
        f"-p action:={shlex.quote(action)} "
        f"-p object_id:={shlex.quote(object_id)} "
        f"-p dispenser_id:={shlex.quote(dispenser_id)} "
        "-p publish_once:=true"
    )


def move_front_hold_cmd(
    args: argparse.Namespace,
    dispenser_id: str,
    *,
    offset_x_m: float,
    offset_y_m: float,
    offset_z_m: float,
    velocity: float,
    acceleration: float,
) -> list[str]:
    return [
        sys.executable,
        str(MOVE_FRONT_HOLD),
        "--service-prefix",
        args.service_prefix,
        "--dispenser-id",
        dispenser_id,
        "--velocity",
        f"{velocity:.6f}",
        "--acceleration",
        f"{acceleration:.6f}",
        "--timeout-sec",
        f"{args.move_timeout_sec:.6f}",
        "--wait-service-sec",
        f"{args.wait_service_sec:.6f}",
        "--verify-target",
        "--verify-timeout-sec",
        f"{args.verify_timeout_sec:.6f}",
        "--target-tolerance-mm",
        f"{args.target_tolerance_mm:.6f}",
        "--target-offset-x-m",
        f"{offset_x_m:.6f}",
        "--target-offset-y-m",
        f"{offset_y_m:.6f}",
        "--target-offset-z-m",
        f"{offset_z_m:.6f}",
        "--compensate-current-tcp",
        "--verify-link6-target",
        "--no-moveit-planning-guard",
        "--execute",
        "--confirm",
        FRONT_HOLD_CONFIRM_PHRASE,
    ]


def move_and_release_cmd(args: argparse.Namespace, dispenser_id: str) -> str:
    # Newly taught side-grip front-hold poses are already close to the cup.
    # Keep staging vertical by default; skip duplicate above-pose commands when the
    # configurable prehold offset is the same as the vertical above offset.
    stages = [
        (
            args.move_prehold_offset_x_m,
            args.move_prehold_offset_y_m,
            args.move_prehold_offset_z_m,
            args.move_prehold_velocity,
            args.move_prehold_acceleration,
        ),
        (0.0, 0.0, args.move_prehold_offset_z_m, args.move_prehold_velocity, args.move_prehold_acceleration),
        (0.0, 0.0, 0.0, args.move_velocity, args.move_acceleration),
    ]
    commands: list[list[str]] = []
    seen: set[tuple[float, float, float, float, float]] = set()
    for offset_x_m, offset_y_m, offset_z_m, velocity, acceleration in stages:
        key = (offset_x_m, offset_y_m, offset_z_m, velocity, acceleration)
        if key in seen:
            continue
        seen.add(key)
        commands.append(
            move_front_hold_cmd(
                args,
                dispenser_id,
                offset_x_m=offset_x_m,
                offset_y_m=offset_y_m,
                offset_z_m=offset_z_m,
                velocity=velocity,
                acceleration=acceleration,
            )
        )
    return " && ".join(shlex.join(command) for command in commands)


def press_cmd(args: argparse.Namespace, dispenser_id: str, press_count: int) -> str:
    press_xyz_m, press_rpy_deg = load_press_pose(dispenser_id)
    service_prefix = shlex.quote(args.service_prefix)
    tcp_name = shlex.quote(args.dispenser_tcp_name)
    return (
        "echo "
        + shlex.quote(
            "[Azas] measured recipe press pose dispenser_"
            f"{dispenser_id}: xyz_m={press_xyz_m} rpy_deg={press_rpy_deg} "
            f"press_count={press_count} source=calibration.yaml"
        )
        + " && "
        "ros2 run azas_dispenser dispenser_press_node --ros-args "
        f"-p service_prefix:={service_prefix} "
        "-p use_taught_posx:=false "
        "-p use_home_as_reference:=false "
        "-p keep_home_orientation:=false "
        f"-p dispenser_x:={press_xyz_m[0]:.6f} "
        f"-p dispenser_y:={press_xyz_m[1]:.6f} "
        "-p dispenser_y_offset:=0.0 "
        f"-p dispenser_top_z:={press_xyz_m[2]:.6f} "
        f"-p rx:={press_rpy_deg[0]:.6f} "
        f"-p ry:={press_rpy_deg[1]:.6f} "
        f"-p rz:={press_rpy_deg[2]:.6f} "
        f"-p press_count:={int(press_count)} "
        # calibration.yaml press_pose_xyz_m is the taught final press pose.
        # Do not subtract an extra legacy pump depth here.
        "-p press_depth:=0.0 "
        f"-p tcp_name:={tcp_name} "
        "-p require_tcp_for_taught_posx:=false "
        "-p allow_tcp_set_failure:=false "
        "-p move_home_first:=false "
        "-p pre_home_retreat_before_home:=false "
        "-p pre_home_retreat_dx_mm:=-180.0 "
        "-p pre_home_retreat_dy_mm:=0.0 "
        "-p pre_home_retreat_min_z_mm:=520.0 -p pre_home_retreat_lift_first:=true "
        "-p pre_home_retreat_min_current_x_mm:=450.0 "
        "-p pre_home_retreat_velocity:=20.0 "
        "-p pre_home_retreat_acceleration:=25.0 "
        "-p joint1_clearance_before_home:=false "
        "-p joint1_clearance_return_home:=false "
        "-p joint1_clearance_offset_deg:=12.0 "
        "-p return_home:=false "
        "-p close_gripper_at_home:=false "
        "-p post_press_retreat_after_sequence:=true "
        "-p post_press_retreat_dx_mm:=-120.0 "
        "-p post_press_retreat_dy_mm:=0.0 "
        "-p post_press_retreat_wait_seconds:=1.0 "
        "-p gripper_service:=/jarvis/rg2/set_width "
        "-p gripper_close_width:=0.0 "
        "-p gripper_close_force:=30.0 "
        "-p gripper_wait_timeout:=12.0 "
        "-p strict_pose_verification:=false "
        "-p service_wait_timeout_sec:=10.0 "
        "-p pose_position_tolerance_mm:=8.0 "
        "-p pose_orientation_tolerance_deg:=6.0 "
        "-p line_velocity:=20.0 "
        "-p line_acceleration:=30.0 "
        "-p travel_line_velocity:=45.0 "
        "-p travel_line_acceleration:=70.0 "
        "-p joint_velocity:=40.0 "
        "-p joint_acceleration:=50.0"
    )


def pick_cmd(args: argparse.Namespace, dispenser_id: str) -> list[str]:
    return [
        sys.executable,
        str(PICK_FRONT_HOLD),
        "--service-prefix",
        args.service_prefix,
        "--dispenser-id",
        dispenser_id,
        "--approach-velocity",
        f"{args.pick_approach_velocity:.6f}",
        "--approach-acceleration",
        f"{args.pick_approach_acceleration:.6f}",
        "--no-pregrasp-staging",
        "--pregrasp-offset-x-m",
        f"{args.pick_pregrasp_offset_x_m:.6f}",
        "--pregrasp-offset-y-m",
        f"{args.pick_pregrasp_offset_y_m:.6f}",
        "--pregrasp-offset-z-m",
        f"{args.pick_pregrasp_offset_z_m:.6f}",
        "--pregrasp-staging-velocity",
        f"{args.pick_pregrasp_staging_velocity:.6f}",
        "--pregrasp-staging-acceleration",
        f"{args.pick_pregrasp_staging_acceleration:.6f}",
        "--joint1-clearance-deg",
        "0.000000",
        "--lift-m",
        f"{args.pick_lift_m:.6f}",
        "--lift-velocity",
        f"{args.pick_lift_velocity:.6f}",
        "--lift-acceleration",
        f"{args.pick_lift_acceleration:.6f}",
        "--timeout-sec",
        f"{args.pick_timeout_sec:.6f}",
        "--wait-service-sec",
        f"{args.wait_service_sec:.6f}",
        "--verify-timeout-sec",
        f"{args.verify_timeout_sec:.6f}",
        "--target-tolerance-mm",
        f"{args.target_tolerance_mm:.6f}",
        "--gripper-grasp-width-m",
        f"{args.gripper_grasp_width_m:.6f}",
        "--gripper-force-n",
        f"{args.gripper_force_n:.6f}",
        "--execute",
        "--confirm",
        PICK_CONFIRM_PHRASE,
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run move/release -> press -> re-grasp for ordered dispenser IDs."
    )
    parser.add_argument("--dispenser-ids", default="1,2,3,4", help="comma-separated IDs, e.g. 1,3,2")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--dispenser-tcp-name", default="GripperDA_v1_jarvis")
    parser.add_argument("--move-velocity", type=float, default=70.0)
    parser.add_argument("--move-acceleration", type=float, default=90.0)
    parser.add_argument("--move-prehold-offset-x-m", type=float, default=0.0)
    parser.add_argument(
        "--move-prehold-offset-y-m",
        type=float,
        default=-0.080,
        help=(
            "Y retreat from measured front_hold for the pre-hold/above-hold approach. "
            "Default -0.080 m keeps the cup farther from dispenser bottles before final placement."
        ),
    )
    parser.add_argument(
        "--move-prehold-offset-z-m",
        type=float,
        default=0.180,
        help=(
            "Vertical approach offset for initial cup placement at dispenser front-hold. "
            "Default 0.180 m avoids the previously too-high approach near the glass bottles."
        ),
    )
    parser.add_argument(
        "--move-release-offset-x-m",
        type=float,
        default=0.0,
        help="Final cup release X offset from measured front_hold.",
    )
    parser.add_argument(
        "--move-release-offset-y-m",
        type=float,
        default=-0.060,
        help="Final cup release Y retreat from measured front_hold to avoid placing too close to the dispenser bottle.",
    )
    parser.add_argument(
        "--move-release-offset-z-m",
        type=float,
        default=-0.010,
        help="Final cup release Z offset from measured front_hold; negative lowers the cup slightly.",
    )
    parser.add_argument("--move-prehold-velocity", type=float, default=50.0)
    parser.add_argument("--move-prehold-acceleration", type=float, default=70.0)
    parser.add_argument("--move-timeout-sec", type=float, default=180.0)
    parser.add_argument("--pick-approach-velocity", type=float, default=35.0)
    parser.add_argument("--pick-approach-acceleration", type=float, default=50.0)
    parser.add_argument("--pick-pregrasp-offset-x-m", type=float, default=0.0)
    parser.add_argument("--pick-pregrasp-offset-y-m", type=float, default=0.0)
    parser.add_argument("--pick-pregrasp-offset-z-m", type=float, default=0.0)
    parser.add_argument("--pick-pregrasp-staging-velocity", type=float, default=12.0)
    parser.add_argument("--pick-pregrasp-staging-acceleration", type=float, default=16.0)
    parser.add_argument("--pick-lift-m", type=float, default=0.100)
    parser.add_argument("--pick-lift-velocity", type=float, default=35.0)
    parser.add_argument("--pick-lift-acceleration", type=float, default=50.0)
    parser.add_argument("--pick-timeout-sec", type=float, default=120.0)
    parser.add_argument(
        "--regrasp-min-transit-z-m",
        type=float,
        default=0.500,
        help="Minimum absolute TCP Z for the vertical lift immediately after pressing, before returning to the cup.",
    )
    parser.add_argument(
        "--regrasp-approach-offset-z-m",
        type=float,
        default=0.250,
        help=(
            "High front-hold Z offset used before opening the gripper for the post-press re-grasp. "
            "The gripper opens only after this high approach is reached."
        ),
    )
    parser.add_argument(
        "--regrasp-max-transit-z-m",
        type=float,
        default=0.560,
        help="Maximum absolute TCP/front-hold high approach Z used for post-press re-grasp transit.",
    )
    parser.add_argument("--regrasp-approach-velocity", type=float, default=45.0)
    parser.add_argument("--regrasp-approach-acceleration", type=float, default=60.0)
    parser.add_argument(
        "--regrasp-retreat-x-m",
        type=float,
        default=0.0,
        help="Optional high-Z X retreat immediately after press before returning to cup.",
    )
    parser.add_argument(
        "--regrasp-retreat-y-m",
        type=float,
        default=0.0,
        help="Optional high-Z Y retreat immediately after press before returning to cup.",
    )
    parser.add_argument(
        "--regrasp-high-transit-joint",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Use IK MoveJoint, not Cartesian MoveLine, for the high post-press return-to-cup transit. "
            "Default true because a straight TCP line can sweep through dispenser/bottle geometry."
        ),
    )
    parser.add_argument(
        "--press-depth-m",
        type=float,
        default=0.060,
        help="Z descent below the measured dispenser-head contact pose. Default is 0.060 m (6 cm).",
    )
    parser.add_argument(
        "--press-pre-lift-m",
        type=float,
        default=0.080,
        help="Z lift above the measured dispenser-head contact pose before descending to press.",
    )
    parser.add_argument("--press-approach-height-m", type=float, default=0.100)
    parser.add_argument("--press-transit-height-m", type=float, default=0.080)
    parser.add_argument(
        "--press-pre-lift-retreat-x-m",
        type=float,
        default=0.0,
        help="X retreat after cup release and before the vertical press lift.",
    )
    parser.add_argument(
        "--press-pre-lift-retreat-y-m",
        type=float,
        default=-0.050,
        help="Y retreat after cup release and before the vertical press lift; negative backs away from the dispenser.",
    )
    parser.add_argument(
        "--press-min-transit-z-m",
        type=float,
        default=0.500,
        help="Minimum absolute TCP Z before moving from cup release toward dispenser press joints.",
    )
    parser.add_argument("--press-line-velocity", type=float, default=18.0)
    parser.add_argument("--press-line-acceleration", type=float, default=25.0)
    parser.add_argument("--press-travel-velocity", type=float, default=45.0)
    parser.add_argument("--press-travel-acceleration", type=float, default=60.0)
    parser.add_argument("--press-timeout-sec", type=float, default=120.0)
    parser.add_argument("--press-hold-seconds", type=float, default=0.25)
    parser.add_argument("--press-gripper-close-width-m", type=float, default=0.0)
    parser.add_argument("--press-gripper-force-n", type=float, default=30.0)
    parser.add_argument("--press-reset-before-press", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--press-reset-joints-deg",
        default="0,0,90,0,90,0",
        help="Joint reset pose used after safe lift and RG2 close, before moving above the press pose.",
    )
    parser.add_argument("--press-reset-joint-velocity", type=float, default=40.0)
    parser.add_argument("--press-reset-joint-acceleration", type=float, default=50.0)
    parser.add_argument("--press-contact-joint-velocity", type=float, default=22.0)
    parser.add_argument("--press-contact-joint-acceleration", type=float, default=30.0)
    parser.add_argument(
        "--press-joint-space-use-high-prepose",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="For measured press_contact_joints_deg, also climb to Cartesian pre_z after contact. Default false avoids the observed pre-pose stall.",
    )
    parser.add_argument(
        "--press-move-configured-prepose-before-joint",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "When measured press_contact_joints_deg exists, optionally move to "
            "calibration press_pose_xyz_m + pre_lift before MoveJoint. Default false: "
            "use the measured joints as the authoritative press target."
        ),
    )
    parser.add_argument(
        "--press-post-retreat-after-sequence",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "After a Cartesian-only press, move laterally away from the dispenser. "
            "Default false; joint-space measured press skips this because it caused "
            "real-hardware target verification stalls before cup re-grasp."
        ),
    )
    parser.add_argument("--press-post-retreat-dx-m", type=float, default=-0.120)
    parser.add_argument("--press-post-retreat-dy-m", type=float, default=0.0)
    parser.add_argument("--press-post-retreat-wait-seconds", type=float, default=0.10)
    parser.add_argument("--wait-service-sec", type=float, default=15.0)
    parser.add_argument(
        "--pose-read-retries",
        type=int,
        default=3,
        help="Retry count for non-motion pose read services such as GetCurrentPosx/GetCurrentPosj.",
    )
    parser.add_argument(
        "--pose-read-retry-sleep-sec",
        type=float,
        default=0.5,
        help="Delay between pose read retries.",
    )
    parser.add_argument("--verify-timeout-sec", type=float, default=70.0)
    parser.add_argument("--verify-poll-seconds", type=float, default=0.15)
    parser.add_argument("--target-tolerance-mm", type=float, default=15.0)
    parser.add_argument(
        "--target-stall-timeout-sec",
        type=float,
        default=8.0,
        help="Fail target verification early when the TCP is far from target and position is not improving.",
    )
    parser.add_argument("--target-stall-min-distance-mm", type=float, default=80.0)
    parser.add_argument("--target-stall-delta-mm", type=float, default=2.0)
    parser.add_argument("--joint-target-tolerance-deg", type=float, default=2.0)
    parser.add_argument(
        "--ik-fallback-max-abs-joint-deg",
        type=float,
        default=360.0,
        help="Reject IK fallback joint solutions with absolute joint values beyond this limit before commanding MoveJoint.",
    )
    parser.add_argument(
        "--ik-fallback-max-joint-delta-deg",
        type=float,
        default=170.0,
        help="Reject IK fallback joint solutions that jump too far from the current joint state before commanding MoveJoint.",
    )
    parser.add_argument(
        "--front-hold-joint-fallback",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "For measured front-hold/pre-hold targets, retry with IK MoveJoint when "
            "MoveLine enters a singularity or stalls target verification."
        ),
    )
    parser.add_argument("--front-hold-joint-fallback-velocity", type=float, default=30.0)
    parser.add_argument("--front-hold-joint-fallback-acceleration", type=float, default=40.0)
    parser.add_argument(
        "--safe-lift-joint-fallback",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "When the post-press vertical MoveLine to safe transit Z stalls in a singularity, "
            "retry the same live-TCP-derived high-Z target with IK MoveJoint before failing."
        ),
    )
    parser.add_argument("--safe-lift-joint-fallback-velocity", type=float, default=30.0)
    parser.add_argument("--safe-lift-joint-fallback-acceleration", type=float, default=40.0)
    parser.add_argument("--gripper-service", default="/jarvis/rg2/set_width")
    parser.add_argument("--gripper-open-width-m", type=float, default=0.110)
    parser.add_argument("--gripper-open-force-n", type=float, default=12.0)
    parser.add_argument("--gripper-grasp-width-m", type=float, default=0.075)
    parser.add_argument("--gripper-force-n", type=float, default=25.0)
    parser.add_argument("--gripper-timeout-sec", type=float, default=12.0)
    parser.add_argument(
        "--gripper-settle-seconds",
        type=float,
        default=0.8,
        help="Physical wait after every non-open RG2 command before the next robot motion.",
    )
    parser.add_argument(
        "--gripper-open-settle-seconds",
        type=float,
        default=1.5,
        help="Physical wait after every RG2 open command before the next robot motion.",
    )
    parser.add_argument(
        "--press-force-joint6-zero",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Force measured press contact joint_6/link_6 to 0 deg before FK-derived pre/press poses. Default is false: use measured joints exactly.",
    )
    parser.add_argument("--precheck-ikin", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--ikin-sol-space", type=int, default=2)
    parser.add_argument("--legacy-subprocess-primitives", action="store_true", help="use the old helper-script-per-step implementation for fallback/debugging")
    parser.add_argument(
        "--integrated-regrasp-fallback-subprocess",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "If the persistent integrated re-grasp/lift stalls verification, retry once "
            "with the legacy pick_from_measured_dispenser_front_hold helper. Default false "
            "because that helper uses Cartesian front-hold entry and can reproduce the "
            "post-press singularity/low direct approach."
        ),
    )
    parser.add_argument(
        "--skip-initial-move-release",
        action="store_true",
        help=(
            "Recovery mode: assume the cup is already resting at the current dispenser front-hold "
            "and start from press -> re-grasp/lift without repeating the move/release placement."
        ),
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} when --execute is used")
    args = parser.parse_args()
    args.press_reset_joints_deg = parse_float_list(
        args.press_reset_joints_deg,
        expected_count=6,
        label="--press-reset-joints-deg",
    )
    return args


def main() -> int:
    args = parse_args()
    try:
        dispenser_ids = parse_dispenser_ids(args.dispenser_ids)
    except ValueError as exc:
        print(f"[FAIL] {exc}")
        return 2
    if args.execute and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
        return 2
    if not args.config.is_file():
        print(f"[FAIL] measured dispenser config not found: {args.config}")
        return 2
    if args.pick_lift_m <= 0.0:
        print("[BLOCKED] --pick-lift-m must be positive for safe retreat after re-grasp")
        return 2
    if not args.execute:
        print("[DRY-RUN] --execute not set; sequence plan only, no robot command sent.")

    print("[Azas] Measured dispenser recipe sequence")
    print(f"[Azas] dispenser_ids={','.join(dispenser_ids)}")
    grouped_dispenser_ids = group_consecutive_dispenser_ids(dispenser_ids)
    print(
        "[Azas] grouped_press_counts="
        + ",".join(f"{dispenser_id}x{count}" for dispenser_id, count in grouped_dispenser_ids)
    )
    print(f"[Azas] service_prefix={args.service_prefix}")
    print(f"[Azas] dispenser_tcp_name={args.dispenser_tcp_name}")
    print("[Azas] source=existing measured front_hold poses and calibration.yaml press poses")

    motion: IntegratedRecipeMotion | None = None
    if args.execute and not args.legacy_subprocess_primitives:
        print("[Azas] integrated_motion=true (persistent ROS clients for move/release/re-grasp)")
        try:
            motion = IntegratedRecipeMotion(args)
            motion.preflight()
        except RuntimeError as exc:
            print(f"[FAIL] integrated preflight failed: {exc}")
            return 1
    elif args.execute:
        print("[Azas] integrated_motion=false (legacy subprocess primitives requested)")

    try:
        total_groups = len(grouped_dispenser_ids)
        for index, (dispenser_id, press_count) in enumerate(grouped_dispenser_ids, start=1):
            label_prefix = f"recipe group {index}/{total_groups} dispenser {dispenser_id} x{press_count}"
            if not args.execute:
                move_release_step = "skip initial move/release" if args.skip_initial_move_release else "integrated move/release"
                print(
                    f"[PLAN] {label_prefix}: {move_release_step} -> "
                    f"integrated press {press_count} time(s) -> integrated re-grasp/lift"
                )
                continue

            if args.skip_initial_move_release:
                print(
                    f"[Azas] {label_prefix}: skipping initial move/release; "
                    "cup is assumed already released at dispenser front-hold"
                )
            elif motion is None:
                rc = run_command(f"{label_prefix}: move cup to front-hold and release", move_and_release_cmd(args, dispenser_id))
                if rc != 0:
                    return rc
                rc = run_command(f"{label_prefix}: RG2 full-open release verify", [str(RG2_OPEN)])
                if rc != 0:
                    return rc
            else:
                try:
                    motion.move_and_release(dispenser_id)
                except RuntimeError as exc:
                    print(f"[FAIL] {label_prefix}: integrated move/release failed: {exc}")
                    return 1

            if motion is None:
                rc = run_command(
                    f"{label_prefix}: mark tumbler world object at dispenser",
                    tumbler_scene_cmd(
                        "add_dispenser",
                        object_id=f"tumbler_at_dispenser_{dispenser_id}",
                        dispenser_id=dispenser_id,
                    ),
                )
                if rc != 0:
                    return rc
            if motion is None:
                rc = run_command(
                    f"{label_prefix}: press dispenser {press_count} time(s)",
                    press_cmd(args, dispenser_id, press_count),
                )
                if rc != 0:
                    return rc
            else:
                try:
                    motion.press_dispenser(dispenser_id, press_count)
                except RuntimeError as exc:
                    print(f"[FAIL] {label_prefix}: integrated press failed: {exc}")
                    return 1

            if motion is None:
                rc = run_command(f"{label_prefix}: re-grasp cup from front-hold", pick_cmd(args, dispenser_id))
                if rc != 0:
                    return rc
            else:
                try:
                    motion.regrasp_and_lift(dispenser_id)
                except RuntimeError as exc:
                    if not args.integrated_regrasp_fallback_subprocess:
                        print(f"[FAIL] {label_prefix}: integrated re-grasp/lift failed: {exc}")
                        return 1
                    print(
                        f"[WARN] {label_prefix}: integrated re-grasp/lift failed: {exc}; "
                        "retrying once with legacy front-hold pick helper"
                    )
                    rc = run_command(f"{label_prefix}: fallback re-grasp cup from front-hold", pick_cmd(args, dispenser_id))
                    if rc != 0:
                        print(f"[FAIL] {label_prefix}: fallback re-grasp/lift failed after integrated timeout")
                        return rc

            if motion is None:
                rc = run_command(
                    f"{label_prefix}: remove dispenser world object",
                    tumbler_scene_cmd(
                        "remove_world",
                        object_id=f"tumbler_at_dispenser_{dispenser_id}",
                        dispenser_id=dispenser_id,
                    ),
                )
                if rc != 0:
                    return rc
                rc = run_command(
                    f"{label_prefix}: attach carried tumbler object",
                    tumbler_scene_cmd("attach", object_id="carried_tumbler", dispenser_id=dispenser_id),
                )
                if rc != 0:
                    return rc
    finally:
        if motion is not None:
            motion.close()

    print("[PASS] measured dispenser recipe sequence completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
