#!/usr/bin/env python3
"""Minimal measured dispenser cycle without MoveIt planning guards.

Sequence per dispenser group:
  measured cup pre-place -> measured cup place/open -> measured cup pre-place ->
  measured press pre -> measured press/contact -> lift/back/open ->
  measured cup pre-place -> measured cup place/grasp -> lift.

All target values are loaded from calibration.yaml. This script does not ask
for, invent, or persist new robot coordinates.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from azas_interfaces.srv import SetGripper
from dsr_msgs2.srv import GetCurrentPosx, MoveJoint, MoveLine, MoveWait


ROOT = Path("/home/ssu/Azas")
DEFAULT_CALIBRATION = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"
CONFIRM_PHRASE = "ENABLE_MINIMAL_DISPENSER_CYCLE"

DR_BASE = 0
MOVE_MODE_ABSOLUTE = 0
SYNC = 0
BLENDING_SPEED_TYPE_DUPLICATE = 0


@dataclass(frozen=True)
class DispenserCalibration:
    dispenser_id: str
    cup_pre_place_joints_deg: list[float] | None
    cup_place_joints_deg: list[float] | None
    outlet_xyz_m: list[float]
    outlet_zyz_deg: list[float]
    press_xyz_m: list[float]
    press_zyz_deg: list[float]
    press_pre_joints_deg: list[float] | None
    press_contact_joints_deg: list[float]


def numeric_list(value: Any, label: str, count: int) -> list[float]:
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{label} must be a {count}-number list")
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain only numbers") from exc


def optional_numeric_list(value: Any, label: str, count: int) -> list[float] | None:
    if value is None:
        return None
    return numeric_list(value, label, count)


def require_cup_place_joints(cfg: DispenserCalibration) -> tuple[list[float], list[float]]:
    if cfg.cup_pre_place_joints_deg is None or cfg.cup_place_joints_deg is None:
        raise ValueError(
            f"dispenser_outlets.{cfg.dispenser_id} must define "
            "cup_pre_place_joints_deg and cup_place_joints_deg for joint cup-place mode"
        )
    return cfg.cup_pre_place_joints_deg, cfg.cup_place_joints_deg


def require_press_joints(cfg: DispenserCalibration) -> tuple[list[float], list[float]]:
    if cfg.press_pre_joints_deg is None:
        raise ValueError(f"dispenser_outlets.{cfg.dispenser_id} must define press_pre_joints_deg for joint press mode")
    return cfg.press_pre_joints_deg, cfg.press_contact_joints_deg


def unwrap_joint_target_near(reference: list[float], target: list[float]) -> list[float]:
    adjusted: list[float] = []
    for ref_value, target_value in zip(reference, target):
        candidates = [target_value - 360.0, target_value, target_value + 360.0]
        adjusted.append(min(candidates, key=lambda candidate: abs(candidate - ref_value)))
    return adjusted


def parse_joint_index_set(raw: str) -> set[int]:
    result: set[int] = set()
    for item in raw.replace(";", ",").split(","):
        value = item.strip().lower()
        if not value:
            continue
        if value.startswith("j"):
            value = value[1:]
        index = int(value)
        if not 1 <= index <= 6:
            raise ValueError(f"joint index out of range: {item!r}")
        result.add(index - 1)
    return result


def lock_joints_to_reference(target: list[float], reference: list[float], joint_indexes: set[int]) -> list[float]:
    adjusted = list(target)
    for index in joint_indexes:
        adjusted[index] = reference[index]
    return adjusted


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


def quaternion_to_doosan_zyz_deg(quaternion: list[float]) -> list[float]:
    return matrix_to_doosan_zyz_deg(quaternion_to_matrix_xyzw(quaternion))


def parse_dispenser_ids(raw: str) -> list[str]:
    result: list[str] = []
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
        if dispenser_id not in {"1", "2", "3", "4"}:
            raise ValueError(f"unsupported dispenser id: {dispenser_id!r}")
        count = int(count_raw.strip())
        if count < 1:
            raise ValueError(f"count must be >= 1 for dispenser {dispenser_id}")
        result.extend([dispenser_id] * count)
    if not result:
        raise ValueError("at least one dispenser id is required")
    return result


def parse_dispenser_id_set(raw: str) -> set[str]:
    result: set[str] = set()
    for item in raw.replace(";", ",").split(","):
        value = item.strip()
        if not value:
            continue
        if value not in {"1", "2", "3", "4"}:
            raise ValueError(f"unsupported waypoint dispenser id: {value!r}")
        result.add(value)
    return result


def group_consecutive(values: list[str]) -> list[tuple[str, int]]:
    groups: list[tuple[str, int]] = []
    for value in values:
        if groups and groups[-1][0] == value:
            groups[-1] = (value, groups[-1][1] + 1)
        else:
            groups.append((value, 1))
    return groups


def load_dispenser(calibration_path: Path, dispenser_id: str) -> DispenserCalibration:
    data = yaml.safe_load(calibration_path.read_text(encoding="utf-8")) or {}
    block = (data.get("dispenser_outlets") or {}).get(dispenser_id)
    if not isinstance(block, dict):
        raise ValueError(f"dispenser_outlets.{dispenser_id} missing in {calibration_path}")
    outlet_xyz_m = numeric_list(block.get("outlet_pose_xyz_m"), f"outlet {dispenser_id} outlet_pose_xyz_m", 3)
    outlet_q = numeric_list(block.get("outlet_pose_quaternion_xyzw"), f"outlet {dispenser_id} outlet_pose_quaternion_xyzw", 4)
    press_xyz_m = numeric_list(block.get("press_pose_xyz_m"), f"outlet {dispenser_id} press_pose_xyz_m", 3)
    press_q = numeric_list(block.get("press_pose_quaternion_xyzw"), f"outlet {dispenser_id} press_pose_quaternion_xyzw", 4)
    press_pre_joints = optional_numeric_list(block.get("press_pre_joints_deg"), f"outlet {dispenser_id} press_pre_joints_deg", 6)
    press_joints = numeric_list(block.get("press_contact_joints_deg"), f"outlet {dispenser_id} press_contact_joints_deg", 6)
    cup_pre_place_joints = optional_numeric_list(
        block.get("cup_pre_place_joints_deg"),
        f"outlet {dispenser_id} cup_pre_place_joints_deg",
        6,
    )
    cup_place_joints = optional_numeric_list(
        block.get("cup_place_joints_deg"),
        f"outlet {dispenser_id} cup_place_joints_deg",
        6,
    )
    return DispenserCalibration(
        dispenser_id=dispenser_id,
        cup_pre_place_joints_deg=cup_pre_place_joints,
        cup_place_joints_deg=cup_place_joints,
        outlet_xyz_m=outlet_xyz_m,
        outlet_zyz_deg=quaternion_to_doosan_zyz_deg(outlet_q),
        press_xyz_m=press_xyz_m,
        press_zyz_deg=quaternion_to_doosan_zyz_deg(press_q),
        press_pre_joints_deg=press_pre_joints,
        press_contact_joints_deg=press_joints,
    )


def load_press_transfer_waypoint(calibration_path: Path, waypoint_name: str) -> list[float] | None:
    if waypoint_name == "none":
        return None
    data = yaml.safe_load(calibration_path.read_text(encoding="utf-8")) or {}
    if waypoint_name == "color_scan":
        return numeric_list(data.get("color_scan_pose", {}).get("joints_deg"), "color_scan_pose.joints_deg", 6)
    raise ValueError(f"unsupported press transfer waypoint: {waypoint_name}")


def service_name(prefix: str, suffix: str) -> str:
    clean = prefix.strip("/")
    return f"/{clean}/{suffix}" if clean else f"/{suffix}"


class MinimalCycle:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        rclpy.init(args=None)
        self.node = rclpy.create_node("azas_minimal_dispenser_cycle")
        self.move_line = self.node.create_client(MoveLine, service_name(args.service_prefix, "motion/move_line"))
        self.move_joint = self.node.create_client(MoveJoint, service_name(args.service_prefix, "motion/move_joint"))
        self.move_wait = self.node.create_client(MoveWait, service_name(args.service_prefix, "motion/move_wait"))
        self.get_posx = self.node.create_client(GetCurrentPosx, service_name(args.service_prefix, "aux_control/get_current_posx"))
        self.gripper = self.node.create_client(SetGripper, args.gripper_service)
        self.press_transfer_waypoint_joints = load_press_transfer_waypoint(args.calibration, args.press_transfer_waypoint)
        self.press_waypoint_dispenser_ids = parse_dispenser_id_set(args.press_waypoint_dispenser_ids)
        self.press_lock_joint_indexes = parse_joint_index_set(args.press_lock_joints)
        self.last_joint_target_deg: list[float] | None = None

    def close(self) -> None:
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def preflight(self) -> None:
        required = [
            (self.move_line, "MoveLine"),
            (self.move_joint, "MoveJoint"),
            (self.move_wait, "MoveWait"),
            (self.get_posx, "GetCurrentPosx"),
            (self.gripper, "RG2 set_width"),
        ]
        missing = [
            f"{label} ({getattr(client, 'srv_name', '<unknown>')})"
            for client, label in required
            if not client.wait_for_service(timeout_sec=max(self.args.wait_service_sec, 0.1))
        ]
        if missing:
            raise RuntimeError("required service(s) unavailable: " + ", ".join(missing))

    def _call(self, client: Any, request: Any, *, timeout_sec: float, label: str) -> Any:
        if not client.wait_for_service(timeout_sec=max(self.args.wait_service_sec, 0.1)):
            raise RuntimeError(f"{label} service not available: {getattr(client, 'srv_name', '<unknown>')}")
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

    def wait_motion(self, label: str, timeout_sec: float) -> None:
        response = self._call(self.move_wait, MoveWait.Request(), timeout_sec=timeout_sec, label=f"MoveWait {label}")
        if not bool(getattr(response, "success", True)):
            raise RuntimeError(f"MoveWait returned success=false for {label}")

    def current_posx(self) -> list[float]:
        req = GetCurrentPosx.Request()
        req.ref = DR_BASE
        response = self._call(self.get_posx, req, timeout_sec=self.args.wait_service_sec, label="GetCurrentPosx")
        if not response.success or not response.task_pos_info:
            raise RuntimeError("GetCurrentPosx returned success=false or empty task_pos_info")
        values = list(response.task_pos_info[0].data)
        if len(values) < 6:
            raise RuntimeError(f"GetCurrentPosx returned too few values: {values}")
        return [float(value) for value in values[:6]]

    def validate_xyz(self, pos: list[float], label: str) -> None:
        x, y, z = pos[:3]
        failures = []
        if not self.args.x_min <= x / 1000.0 <= self.args.x_max:
            failures.append(f"x={x / 1000.0:.3f} outside [{self.args.x_min:.3f}, {self.args.x_max:.3f}]")
        if not self.args.y_min <= y / 1000.0 <= self.args.y_max:
            failures.append(f"y={y / 1000.0:.3f} outside [{self.args.y_min:.3f}, {self.args.y_max:.3f}]")
        if not self.args.z_min <= z / 1000.0 <= self.args.z_max:
            failures.append(f"z={z / 1000.0:.3f} outside [{self.args.z_min:.3f}, {self.args.z_max:.3f}]")
        if failures:
            raise RuntimeError(f"{label} outside direct MoveLine bounds: " + "; ".join(failures))

    def validate_joints(self, joints_deg: list[float], label: str) -> None:
        failures = []
        for index, value in enumerate(joints_deg, start=1):
            if not self.args.joint_min_deg <= value <= self.args.joint_max_deg:
                failures.append(
                    f"j{index}={value:.2f} outside "
                    f"[{self.args.joint_min_deg:.0f}, {self.args.joint_max_deg:.0f}]"
                )
        if not self.args.j5_min_deg <= joints_deg[4] <= self.args.j5_max_deg:
            failures.append(
                f"j5={joints_deg[4]:.2f} outside "
                f"[{self.args.j5_min_deg:.0f}, {self.args.j5_max_deg:.0f}]"
            )
        if failures:
            raise RuntimeError(f"{label} outside direct MoveJoint bounds: " + "; ".join(failures))

    def movel(self, pos: list[float], label: str, velocity: float, acceleration: float) -> None:
        self.validate_xyz(pos, label)
        print(
            f"[Azas] {label}: movel posx=[{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}, "
            f"{pos[3]:.1f}, {pos[4]:.1f}, {pos[5]:.1f}] vel={velocity:.1f} acc={acceleration:.1f}"
        )
        req = MoveLine.Request()
        req.pos = [float(value) for value in pos]
        req.vel = [float(velocity), float(velocity)]
        req.acc = [float(acceleration), float(acceleration)]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        response = self._call(self.move_line, req, timeout_sec=self.args.motion_timeout_sec, label=f"MoveLine {label}")
        if not response.success:
            raise RuntimeError(f"MoveLine returned success=false for {label}")
        self.wait_motion(label, self.args.motion_timeout_sec)

    def unwrap_near_last_joint_target(self, joints_deg: list[float], label: str) -> list[float]:
        if self.last_joint_target_deg is None or self.args.disable_joint_branch_unwrap:
            return list(joints_deg)
        adjusted = unwrap_joint_target_near(self.last_joint_target_deg, joints_deg)
        if any(abs(raw - command) > 1e-6 for raw, command in zip(joints_deg, adjusted)):
            print(
                "[Azas] "
                + label
                + ": unwrap joint branch raw=["
                + ", ".join(f"{value:.1f}" for value in joints_deg)
                + "] command=["
                + ", ".join(f"{value:.1f}" for value in adjusted)
                + "]"
            )
        return adjusted

    def movej(
        self,
        joints_deg: list[float],
        label: str,
        velocity: float,
        acceleration: float,
    ) -> None:
        command_joints = self.unwrap_near_last_joint_target(joints_deg, label)
        self.validate_joints(command_joints, label)
        print(
            "[Azas] "
            + label
            + ": movej_deg=["
            + ", ".join(f"{value:.1f}" for value in command_joints)
            + f"] vel={velocity:.1f} acc={acceleration:.1f}"
        )
        req = MoveJoint.Request()
        req.pos = [float(value) for value in command_joints]
        req.vel = float(velocity)
        req.acc = float(acceleration)
        req.time = 0.0
        req.radius = 0.0
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        response = self._call(self.move_joint, req, timeout_sec=self.args.motion_timeout_sec, label=f"MoveJoint {label}")
        if not response.success:
            raise RuntimeError(f"MoveJoint returned success=false for {label}")
        self.wait_motion(label, self.args.motion_timeout_sec)
        self.last_joint_target_deg = command_joints

    def should_use_press_waypoint(self, cfg: DispenserCalibration) -> bool:
        return self.press_transfer_waypoint_joints is not None and cfg.dispenser_id in self.press_waypoint_dispenser_ids

    def move_press_transfer_waypoint(self, cfg: DispenserCalibration, label: str) -> bool:
        if not self.should_use_press_waypoint(cfg):
            return False
        assert self.press_transfer_waypoint_joints is not None
        self.movej(
            self.press_transfer_waypoint_joints,
            label,
            self.args.press_waypoint_velocity,
            self.args.press_waypoint_acceleration,
        )
        return True

    def gripper_command(self, command: str, width_m: float, force_n: float, label: str) -> None:
        print(f"[Azas] {label}: gripper command={command} width_m={width_m:.3f} force_n={force_n:.1f}")
        req = SetGripper.Request()
        req.command = command
        req.width_m = float(width_m)
        req.force_n = float(force_n)
        response = self._call(self.gripper, req, timeout_sec=self.args.gripper_timeout_sec, label=label)
        if not response.success:
            raise RuntimeError(f"{label} returned success=false: {response.message}")
        print(f"[Azas] {label}: {response.message}")
        settle = self.args.gripper_open_settle_sec if command == "open" else self.args.gripper_settle_sec
        if settle > 0.0:
            time.sleep(settle)

    def lift_current_to(self, min_z_m: float, label: str, velocity: float, acceleration: float) -> None:
        pose = self.current_posx()
        target_z_mm = max(float(pose[2]), max(min_z_m, 0.0) * 1000.0)
        if target_z_mm <= pose[2] + 1.0:
            print(f"[Azas] {label}: already high enough z={pose[2] / 1000.0:.3f}m")
            return
        target = [pose[0], pose[1], target_z_mm, pose[3], pose[4], pose[5]]
        self.movel(target, label, velocity, acceleration)

    def shift_current_x(self, dx_m: float, label: str, velocity: float, acceleration: float) -> None:
        if abs(dx_m) < 1e-6:
            print(f"[Azas] {label}: skipped dx=0")
            return
        pose = self.current_posx()
        target = [pose[0] + dx_m * 1000.0, pose[1], pose[2], pose[3], pose[4], pose[5]]
        self.movel(target, label, velocity, acceleration)

    @staticmethod
    def posx(xyz_m: list[float], zyz_deg: list[float]) -> list[float]:
        return [xyz_m[0] * 1000.0, xyz_m[1] * 1000.0, xyz_m[2] * 1000.0, *zyz_deg]

    def run_group(self, cfg: DispenserCalibration, press_count: int) -> None:
        place = self.posx(cfg.outlet_xyz_m, cfg.outlet_zyz_deg)
        pre_place_xyz = [
            cfg.outlet_xyz_m[0] + self.args.cup_pre_x_m,
            cfg.outlet_xyz_m[1] + self.args.cup_pre_y_m,
            cfg.outlet_xyz_m[2] + self.args.cup_pre_z_m,
        ]
        pre_place = self.posx(pre_place_xyz, cfg.outlet_zyz_deg)
        press_pre_xyz = [
            cfg.press_xyz_m[0],
            cfg.press_xyz_m[1],
            cfg.press_xyz_m[2] + self.args.press_pre_z_m,
        ]
        press_pre = self.posx(press_pre_xyz, cfg.press_zyz_deg)

        print(f"[Azas] === dispenser {cfg.dispenser_id} x{press_count} minimal cycle ===")
        if self.args.cup_place_mode == "joint":
            cup_pre_place_joints, cup_place_joints = require_cup_place_joints(cfg)
            self.movej(
                cup_pre_place_joints,
                "cup pre-place measured joints",
                self.args.travel_velocity,
                self.args.travel_acceleration,
            )
            self.movej(
                cup_place_joints,
                "cup place measured joints",
                self.args.approach_velocity,
                self.args.approach_acceleration,
            )
        else:
            self.movel(pre_place, "cup pre-place robot-side/up", self.args.travel_velocity, self.args.travel_acceleration)
            self.movel(place, "cup place at measured outlet", self.args.approach_velocity, self.args.approach_acceleration)
        self.gripper_command("open", self.args.gripper_open_width_m, self.args.gripper_open_force_n, "release cup")

        if self.args.cup_place_mode == "joint":
            cup_pre_place_joints, _ = require_cup_place_joints(cfg)
            self.movej(
                cup_pre_place_joints,
                "exit cup place to measured pre-place",
                self.args.travel_velocity,
                self.args.travel_acceleration,
            )
        else:
            self.lift_current_to(self.args.after_release_min_z_m, "lift after cup release", self.args.travel_velocity, self.args.travel_acceleration)
            self.shift_current_x(self.args.after_release_retreat_x_m, "robot-side retreat after release", self.args.travel_velocity, self.args.travel_acceleration)
        self.gripper_command("set_width", self.args.press_gripper_width_m, self.args.press_gripper_force_n, "close empty gripper before press")

        if self.args.press_mode == "joint":
            press_pre_joints, press_contact_joints = require_press_joints(cfg)
            press_contact_command_joints = lock_joints_to_reference(
                press_contact_joints,
                press_pre_joints,
                self.press_lock_joint_indexes,
            )
            if not self.args.disable_press_entry_waypoint:
                self.move_press_transfer_waypoint(cfg, f"press entry transfer waypoint before dispenser {cfg.dispenser_id}")
            self.movej(
                press_pre_joints,
                "press pre measured joints",
                self.args.travel_velocity,
                self.args.travel_acceleration,
            )
            for index in range(1, press_count + 1):
                suffix = f" {index}/{press_count}" if press_count > 1 else ""
                self.movej(
                    press_contact_command_joints,
                    f"press contact measured joints{suffix}",
                    self.args.press_joint_velocity,
                    self.args.press_joint_acceleration,
                )
                if self.args.press_hold_sec > 0.0:
                    time.sleep(self.args.press_hold_sec)
                self.movej(
                    press_pre_joints,
                    f"press release to measured pre{suffix}",
                    self.args.press_joint_velocity,
                    self.args.press_joint_acceleration,
                )
        else:
            self.movel(press_pre, "press pre-pose above dispenser", self.args.travel_velocity, self.args.travel_acceleration)
            self.movej(cfg.press_contact_joints_deg, "press contact measured joints", self.args.press_joint_velocity, self.args.press_joint_acceleration)

            contact = self.current_posx()
            up = list(contact)
            down = list(contact)
            down[2] = contact[2] - max(self.args.press_depth_m, 0.0) * 1000.0
            for index in range(1, press_count + 1):
                suffix = f" {index}/{press_count}" if press_count > 1 else ""
                if self.args.press_depth_m > 0.0:
                    self.movel(down, f"press pump down{suffix}", self.args.press_line_velocity, self.args.press_line_acceleration)
                self.movel(up, f"press pump release/up{suffix}", self.args.press_line_velocity, self.args.press_line_acceleration)
                if self.args.press_hold_sec > 0.0:
                    time.sleep(self.args.press_hold_sec)

        exited_via_waypoint = False
        if self.args.press_mode == "joint" and not self.args.disable_press_exit_waypoint:
            exited_via_waypoint = self.move_press_transfer_waypoint(
                cfg,
                f"press exit transfer waypoint after dispenser {cfg.dispenser_id}",
            )
        if not exited_via_waypoint:
            self.lift_current_to(self.args.post_press_min_z_m, "lift after press before re-grasp", self.args.travel_velocity, self.args.travel_acceleration)
            self.shift_current_x(self.args.post_press_retreat_x_m, "robot-side retreat after press", self.args.travel_velocity, self.args.travel_acceleration)
        if self.args.cup_place_mode == "joint":
            cup_pre_place_joints, cup_place_joints = require_cup_place_joints(cfg)
            self.movej(
                cup_pre_place_joints,
                "re-grasp pre-place measured joints",
                self.args.travel_velocity,
                self.args.travel_acceleration,
            )
            self.gripper_command(
                "open",
                self.args.gripper_open_width_m,
                self.args.gripper_open_force_n,
                "open gripper at cup pre-place before re-grasp",
            )
            self.movej(
                cup_place_joints,
                "re-grasp cup place measured joints",
                self.args.approach_velocity,
                self.args.approach_acceleration,
            )
        else:
            self.gripper_command(
                "open",
                self.args.gripper_open_width_m,
                self.args.gripper_open_force_n,
                "open gripper away from dispenser before re-grasp",
            )
            rear_high_xyz = [
                cfg.outlet_xyz_m[0] + self.args.regrasp_rear_x_m,
                cfg.outlet_xyz_m[1] + self.args.regrasp_rear_y_m,
                cfg.outlet_xyz_m[2] + self.args.regrasp_pre_z_m,
            ]
            rear_low_xyz = [
                cfg.outlet_xyz_m[0] + self.args.regrasp_rear_x_m,
                cfg.outlet_xyz_m[1] + self.args.regrasp_rear_y_m,
                cfg.outlet_xyz_m[2],
            ]
            self.movel(
                self.posx(rear_high_xyz, cfg.outlet_zyz_deg),
                "re-grasp rear high pre-pose",
                self.args.travel_velocity,
                self.args.travel_acceleration,
            )
            self.movel(
                self.posx(rear_low_xyz, cfg.outlet_zyz_deg),
                "re-grasp rear lowered pose",
                self.args.approach_velocity,
                self.args.approach_acceleration,
            )
            self.movel(place, "re-grasp forward to cup", self.args.approach_velocity, self.args.approach_acceleration)
        self.gripper_command("set_width", self.args.gripper_grasp_width_m, self.args.gripper_grasp_force_n, "soft side-grasp cup")

        pose = self.current_posx()
        final_lift = [pose[0], pose[1], pose[2] + self.args.final_lift_m * 1000.0, pose[3], pose[4], pose[5]]
        self.movel(final_lift, "lift cup after re-grasp", self.args.travel_velocity, self.args.travel_acceleration)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a minimal dispenser place/press/re-grasp cycle.")
    parser.add_argument("--dispenser-ids", default="1x1", help="comma-separated IDs, e.g. 1x1,2x2")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--wait-service-sec", type=float, default=8.0)
    parser.add_argument("--motion-timeout-sec", type=float, default=90.0)
    parser.add_argument("--x-min", type=float, default=0.10)
    parser.add_argument("--x-max", type=float, default=0.90)
    parser.add_argument("--y-min", type=float, default=-0.45)
    parser.add_argument("--y-max", type=float, default=0.45)
    parser.add_argument("--z-min", type=float, default=0.05)
    parser.add_argument("--z-max", type=float, default=0.85)
    parser.add_argument("--joint-min-deg", type=float, default=-360.0)
    parser.add_argument("--joint-max-deg", type=float, default=360.0)
    parser.add_argument("--j5-min-deg", type=float, default=-135.0)
    parser.add_argument("--j5-max-deg", type=float, default=135.0)
    parser.add_argument(
        "--cup-place-mode",
        choices=("joint", "cartesian"),
        default="joint",
        help="joint uses cup_pre/place_joints_deg; cartesian uses outlet_pose offsets",
    )
    parser.add_argument(
        "--press-mode",
        choices=("joint", "cartesian"),
        default="joint",
        help="joint uses press_pre/contact_joints_deg; cartesian uses press_pose plus line pump",
    )
    parser.add_argument(
        "--press-transfer-waypoint",
        choices=("none", "color_scan"),
        default="color_scan",
        help="measured joint waypoint used before/after selected dispenser press moves",
    )
    parser.add_argument(
        "--press-waypoint-dispenser-ids",
        default="3,4",
        help="comma-separated dispenser IDs that use the press transfer waypoint",
    )
    parser.add_argument("--disable-press-entry-waypoint", action="store_true")
    parser.add_argument("--disable-press-exit-waypoint", action="store_true")
    parser.add_argument("--disable-joint-branch-unwrap", action="store_true")
    parser.add_argument(
        "--press-lock-joints",
        default="4,6",
        help="comma-separated joints copied from PRESS_PRE into PRESS contact command; empty disables",
    )

    parser.add_argument("--cup-pre-x-m", type=float, default=-0.080)
    parser.add_argument("--cup-pre-y-m", type=float, default=0.0)
    parser.add_argument("--cup-pre-z-m", type=float, default=0.120)
    parser.add_argument("--after-release-min-z-m", type=float, default=0.500)
    parser.add_argument("--after-release-retreat-x-m", type=float, default=-0.050)

    parser.add_argument("--press-pre-z-m", type=float, default=0.080)
    parser.add_argument("--press-depth-m", type=float, default=0.040)
    parser.add_argument("--post-press-min-z-m", type=float, default=0.500)
    parser.add_argument("--post-press-retreat-x-m", type=float, default=-0.080)

    parser.add_argument("--regrasp-rear-x-m", type=float, default=-0.080)
    parser.add_argument("--regrasp-rear-y-m", type=float, default=0.0)
    parser.add_argument("--regrasp-pre-z-m", type=float, default=0.120)
    parser.add_argument("--final-lift-m", type=float, default=0.100)

    parser.add_argument("--travel-velocity", type=float, default=35.0)
    parser.add_argument("--travel-acceleration", type=float, default=50.0)
    parser.add_argument("--approach-velocity", type=float, default=20.0)
    parser.add_argument("--approach-acceleration", type=float, default=30.0)
    parser.add_argument("--press-joint-velocity", type=float, default=20.0)
    parser.add_argument("--press-joint-acceleration", type=float, default=30.0)
    parser.add_argument("--press-waypoint-velocity", type=float, default=25.0)
    parser.add_argument("--press-waypoint-acceleration", type=float, default=35.0)
    parser.add_argument("--press-line-velocity", type=float, default=12.0)
    parser.add_argument("--press-line-acceleration", type=float, default=18.0)
    parser.add_argument("--press-hold-sec", type=float, default=0.15)

    parser.add_argument("--gripper-service", default="/jarvis/rg2/set_width")
    parser.add_argument("--gripper-open-width-m", type=float, default=0.110)
    parser.add_argument("--gripper-open-force-n", type=float, default=12.0)
    parser.add_argument("--press-gripper-width-m", type=float, default=0.0)
    parser.add_argument("--press-gripper-force-n", type=float, default=30.0)
    parser.add_argument("--gripper-grasp-width-m", type=float, default=0.075)
    parser.add_argument("--gripper-grasp-force-n", type=float, default=25.0)
    parser.add_argument("--gripper-timeout-sec", type=float, default=12.0)
    parser.add_argument("--gripper-settle-sec", type=float, default=0.8)
    parser.add_argument("--gripper-open-settle-sec", type=float, default=1.2)

    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} when --execute is used")
    return parser.parse_args()


def print_plan(groups: list[tuple[str, int]], args: argparse.Namespace) -> None:
    print("[Azas] Minimal dispenser cycle")
    print("[Azas] no MoveIt, no /collision_object, no link6/TCP target verification")
    print(f"[Azas] dispenser_groups={','.join(f'{dispenser_id}x{count}' for dispenser_id, count in groups)}")
    print(f"[Azas] cup_place_mode={args.cup_place_mode}")
    print(f"[Azas] press_mode={args.press_mode}")
    print(
        f"[Azas] press_transfer_waypoint={args.press_transfer_waypoint} "
        f"ids={args.press_waypoint_dispenser_ids or '-'} "
        f"entry={'off' if args.disable_press_entry_waypoint else 'on'} "
        f"exit={'off' if args.disable_press_exit_waypoint else 'on'} "
        f"branch_unwrap={'off' if args.disable_joint_branch_unwrap else 'on'} "
        f"press_lock_joints={args.press_lock_joints or '-'}"
    )
    print(
        "[Azas] sequence=cup pre-place -> cup place/open -> cup pre-place/close -> "
        "press pre -> press/contact/pre repeat -> press exit -> cup pre-place/open -> cup place/grasp -> lift"
    )
    if args.cup_place_mode == "cartesian":
        print(
            f"[Azas] offsets: cup_pre=({args.cup_pre_x_m:.3f},{args.cup_pre_y_m:.3f},{args.cup_pre_z_m:.3f}) "
            f"after_release_retreat_x={args.after_release_retreat_x_m:.3f} "
            f"post_press_retreat_x={args.post_press_retreat_x_m:.3f} "
            f"regrasp_rear=({args.regrasp_rear_x_m:.3f},{args.regrasp_rear_y_m:.3f},{args.regrasp_pre_z_m:.3f})"
        )
    else:
        print(
            f"[Azas] post_press_escape: min_z={args.post_press_min_z_m:.3f} "
            f"retreat_x={args.post_press_retreat_x_m:.3f}; "
            f"joint_bounds=[{args.joint_min_deg:.0f},{args.joint_max_deg:.0f}], "
            f"j5_bounds=[{args.j5_min_deg:.0f},{args.j5_max_deg:.0f}]"
        )


def main() -> int:
    args = parse_args()
    try:
        dispenser_ids = parse_dispenser_ids(args.dispenser_ids)
        waypoint_dispenser_ids = parse_dispenser_id_set(args.press_waypoint_dispenser_ids)
        press_lock_joint_indexes = parse_joint_index_set(args.press_lock_joints)
    except ValueError as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 2
    groups = group_consecutive(dispenser_ids)
    if args.execute and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}", file=sys.stderr)
        return 2
    if not args.calibration.is_file():
        print(f"[FAIL] calibration file not found: {args.calibration}", file=sys.stderr)
        return 2

    print_plan(groups, args)
    try:
        calibrations = [load_dispenser(args.calibration, dispenser_id) for dispenser_id, _ in groups]
        press_transfer_waypoint = load_press_transfer_waypoint(args.calibration, args.press_transfer_waypoint)
        if args.cup_place_mode == "joint":
            for cfg in calibrations:
                require_cup_place_joints(cfg)
        if args.press_mode == "joint":
            for cfg in calibrations:
                require_press_joints(cfg)
    except ValueError as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 2
    for cfg, (_, count) in zip(calibrations, groups):
        pre_place = [
            cfg.outlet_xyz_m[0] + args.cup_pre_x_m,
            cfg.outlet_xyz_m[1] + args.cup_pre_y_m,
            cfg.outlet_xyz_m[2] + args.cup_pre_z_m,
        ]
        press_pre = [cfg.press_xyz_m[0], cfg.press_xyz_m[1], cfg.press_xyz_m[2] + args.press_pre_z_m]
        if args.cup_place_mode == "joint":
            cup_pre_place_joints, cup_place_joints = require_cup_place_joints(cfg)
            cup_part = (
                f"cup_pre_joints={[round(v, 2) for v in cup_pre_place_joints]} "
                f"cup_place_joints={[round(v, 2) for v in cup_place_joints]}"
            )
        else:
            cup_part = f"cup_place={cfg.outlet_xyz_m} cup_pre={[round(v, 3) for v in pre_place]}"
        if args.press_mode == "joint":
            press_pre_joints, press_contact_joints = require_press_joints(cfg)
            press_locked_joints = lock_joints_to_reference(
                press_contact_joints,
                press_pre_joints,
                press_lock_joint_indexes,
            )
            press_command_joints = (
                press_locked_joints
                if args.disable_joint_branch_unwrap
                else unwrap_joint_target_near(press_pre_joints, press_locked_joints)
            )
            waypoint_part = ""
            if press_transfer_waypoint is not None and cfg.dispenser_id in waypoint_dispenser_ids:
                waypoint_part = f" waypoint={args.press_transfer_waypoint}:{[round(v, 2) for v in press_transfer_waypoint]}"
            command_part = ""
            if any(abs(raw - command) > 1e-6 for raw, command in zip(press_contact_joints, press_command_joints)):
                command_part = f" press_command={[round(v, 2) for v in press_command_joints]}"
            press_part = (
                f"press_pre_joints={[round(v, 2) for v in press_pre_joints]} "
                f"press_joints={[round(v, 2) for v in press_contact_joints]}"
                f"{command_part}"
                f"{waypoint_part}"
            )
        else:
            press_part = (
                f"press_pre={[round(v, 3) for v in press_pre]} "
                f"press_joints={[round(v, 2) for v in cfg.press_contact_joints_deg]}"
            )
        print(
            f"[PLAN] dispenser {cfg.dispenser_id} x{count}: "
            f"{cup_part} "
            f"{press_part}"
        )

    if not args.execute:
        print("[DRY-RUN] --execute not set; no robot or gripper command sent.")
        return 0

    cycle = MinimalCycle(args)
    try:
        cycle.preflight()
        for cfg, (_, count) in zip(calibrations, groups):
            cycle.run_group(cfg, count)
    except RuntimeError as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1
    finally:
        cycle.close()
    print("[PASS] minimal dispenser cycle completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
