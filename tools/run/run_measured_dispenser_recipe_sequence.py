#!/usr/bin/env python3
"""Run an ordered measured-dispenser recipe loop.

For each dispenser ID this composes measured rule-based primitives:
  move/release cup at DISP_PRE/DISP_PLACE -> measured PRESS_CONTACT
  -> generated CONTACT_ENTRY_LIFT -> Z-only pump overdrive
  -> re-grasp/lift cup at DISP_PLACE.

All cup/dispenser positions come from measured calibration data.  This runner
does not ask for or generate new robot coordinates.
"""

from __future__ import annotations

import argparse
import json
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
from dsr_msgs2.srv import (
    Fkin,
    GetCurrentPosj,
    GetCurrentPosx,
    GetCurrentTcp,
    Ikin,
    MoveJoint,
    MoveLine,
    MoveWait,
    SetCurrentTcp,
)


ROOT = Path("/home/ssu/Azas")
DEFAULT_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "measured_dispenser_collision.yaml"
CALIBRATION_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"
DEFAULT_RESUME_STATE = ROOT / "outputs" / "measured_dispenser_recipe_resume.json"
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
INVALID_PRESS_CONTACT_STATUSES = {
    "invalid",
    "invalid_reteach_required",
    "needs_reteach",
    "reteach_required",
    "확인 필요",
}
INVALID_CUP_PLACE_STATUSES = INVALID_PRESS_CONTACT_STATUSES

DR_BASE = 0
MOVE_MODE_ABSOLUTE = 0
SYNC = 0
BLENDING_SPEED_TYPE_DUPLICATE = 0
Pose = tuple[list[float], list[list[float]]]


def angular_delta_deg(target: float, current: float) -> float:
    """Smallest absolute angular delta for wrapped revolute joints."""
    return abs((float(target) - float(current) + 180.0) % 360.0 - 180.0)


def equivalent_angle_near_current_deg(target: float, current: float, *, max_abs: float) -> float:
    """Choose the equivalent joint angle closest to the current controller reading."""
    target_value = float(target)
    current_value = float(current)
    center_step = round((current_value - target_value) / 360.0)
    candidates = [target_value + 360.0 * (center_step + step) for step in range(-2, 3)]
    modulo = ((target_value + 180.0) % 360.0) - 180.0
    candidates.extend(
        [
            modulo,
            modulo + 360.0,
            modulo - 360.0,
        ]
    )
    if max_abs > 0.0:
        bounded = [candidate for candidate in candidates if abs(candidate) <= max_abs]
        if bounded:
            candidates = bounded
    return min(candidates, key=lambda candidate: abs(candidate - current_value))


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


def parse_joint_index_set(raw: str, *, label: str) -> set[int]:
    result: set[int] = set()
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
            raise ValueError(f"{label} contains a non-joint value: {part!r}") from exc
        if not 1 <= index <= 6:
            raise ValueError(f"{label} joint index must be 1..6, got {part!r}")
        result.add(index - 1)
    return result


def lock_joints_to_reference(target: list[float], reference: list[float], joint_indexes: set[int]) -> list[float]:
    adjusted = list(target)
    for index in joint_indexes:
        adjusted[index] = reference[index]
    return adjusted


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


def press_contact_status(block: dict[str, Any]) -> str:
    return str(block.get("press_contact_status", "")).strip()


def ensure_press_contact_enabled(dispenser_id: str, block: dict[str, Any]) -> None:
    status = press_contact_status(block)
    if status.lower() in INVALID_PRESS_CONTACT_STATUSES:
        raise ValueError(
            f"dispenser_outlets.{dispenser_id}.press_contact_joints_deg is marked "
            f"{status!r}; refusing real press motion until PRESS{dispenser_id}_CONTACT is re-taught"
        )


def cup_place_status(block: dict[str, Any]) -> str:
    return str(block.get("cup_place_status", "")).strip()


def ensure_cup_place_enabled(dispenser_id: str, block: dict[str, Any]) -> None:
    status = cup_place_status(block)
    if status.lower() in INVALID_CUP_PLACE_STATUSES:
        raise ValueError(
            f"dispenser_outlets.{dispenser_id}.cup_place_joints_deg is marked "
            f"{status!r}; refusing real cup place/re-grasp motion until DISP{dispenser_id} cup place is re-taught"
        )


def require_dispenser_press_contact_enabled(dispenser_id: str) -> None:
    data = yaml.safe_load(CALIBRATION_CONFIG.read_text(encoding="utf-8")) or {}
    outlets = data.get("dispenser_outlets") or {}
    block = outlets.get(str(dispenser_id))
    if not isinstance(block, dict):
        raise ValueError(f"dispenser_outlets.{dispenser_id} is missing in {CALIBRATION_CONFIG}")
    ensure_press_contact_enabled(dispenser_id, block)


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
    ensure_press_contact_enabled(dispenser_id, block)
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
    ensure_press_contact_enabled(dispenser_id, block)
    raw_joints = block.get("press_contact_joints_deg", block.get("press_ready_joints_deg"))
    if raw_joints is None:
        return None
    return numeric_list(
        raw_joints,
        f"dispenser_outlets.{dispenser_id}.press_contact_joints_deg",
        6,
    )


def load_press_pre_joints_deg(dispenser_id: str) -> list[float] | None:
    data = yaml.safe_load(CALIBRATION_CONFIG.read_text(encoding="utf-8")) or {}
    outlets = data.get("dispenser_outlets") or {}
    block = outlets.get(str(dispenser_id))
    if not isinstance(block, dict):
        raise ValueError(f"dispenser_outlets.{dispenser_id} is missing in {CALIBRATION_CONFIG}")
    ensure_press_contact_enabled(dispenser_id, block)
    raw_joints = block.get("press_pre_joints_deg")
    if raw_joints is None:
        return None
    return numeric_list(
        raw_joints,
        f"dispenser_outlets.{dispenser_id}.press_pre_joints_deg",
        6,
    )


def load_press_common_pre_joints_deg(dispenser_id: str) -> list[float] | None:
    data = yaml.safe_load(CALIBRATION_CONFIG.read_text(encoding="utf-8")) or {}
    outlets = data.get("dispenser_outlets") or {}
    block = outlets.get(str(dispenser_id))
    if not isinstance(block, dict):
        raise ValueError(f"dispenser_outlets.{dispenser_id} is missing in {CALIBRATION_CONFIG}")
    raw_joints = block.get("press_common_pre_joints_deg")
    source_id = str(dispenser_id)
    if raw_joints is None:
        for candidate_id in sorted(
            outlets,
            key=lambda value: (0, int(value)) if str(value).isdigit() else (1, str(value)),
        ):
            candidate_block = outlets.get(str(candidate_id))
            if isinstance(candidate_block, dict) and candidate_block.get("press_common_pre_joints_deg") is not None:
                raw_joints = candidate_block.get("press_common_pre_joints_deg")
                source_id = str(candidate_id)
                break
    if raw_joints is None:
        return None
    return numeric_list(
        raw_joints,
        f"dispenser_outlets.{source_id}.press_common_pre_joints_deg",
        6,
    )


def load_cup_common_pre_joints_deg(dispenser_id: str) -> list[float] | None:
    data = yaml.safe_load(CALIBRATION_CONFIG.read_text(encoding="utf-8")) or {}
    outlets = data.get("dispenser_outlets") or {}
    block = outlets.get(str(dispenser_id))
    if not isinstance(block, dict):
        raise ValueError(f"dispenser_outlets.{dispenser_id} is missing in {CALIBRATION_CONFIG}")
    raw_joints = block.get("cup_common_pre_joints_deg")
    source_id = str(dispenser_id)
    if raw_joints is None:
        for candidate_id in sorted(
            outlets,
            key=lambda value: (0, int(value)) if str(value).isdigit() else (1, str(value)),
        ):
            candidate_block = outlets.get(str(candidate_id))
            if isinstance(candidate_block, dict) and candidate_block.get("cup_common_pre_joints_deg") is not None:
                raw_joints = candidate_block.get("cup_common_pre_joints_deg")
                source_id = str(candidate_id)
                break
    if raw_joints is None:
        return None
    return numeric_list(
        raw_joints,
        f"dispenser_outlets.{source_id}.cup_common_pre_joints_deg",
        6,
    )


def load_cup_pre_place_joints_deg(dispenser_id: str) -> list[float] | None:
    data = yaml.safe_load(CALIBRATION_CONFIG.read_text(encoding="utf-8")) or {}
    outlets = data.get("dispenser_outlets") or {}
    block = outlets.get(str(dispenser_id))
    if not isinstance(block, dict):
        raise ValueError(f"dispenser_outlets.{dispenser_id} is missing in {CALIBRATION_CONFIG}")
    ensure_cup_place_enabled(dispenser_id, block)
    raw_joints = block.get("cup_pre_place_joints_deg")
    if raw_joints is None:
        return None
    return numeric_list(
        raw_joints,
        f"dispenser_outlets.{dispenser_id}.cup_pre_place_joints_deg",
        6,
    )


def load_cup_place_joints_deg(dispenser_id: str) -> list[float] | None:
    data = yaml.safe_load(CALIBRATION_CONFIG.read_text(encoding="utf-8")) or {}
    outlets = data.get("dispenser_outlets") or {}
    block = outlets.get(str(dispenser_id))
    if not isinstance(block, dict):
        raise ValueError(f"dispenser_outlets.{dispenser_id} is missing in {CALIBRATION_CONFIG}")
    ensure_cup_place_enabled(dispenser_id, block)
    raw_joints = block.get("cup_place_joints_deg")
    if raw_joints is None:
        return None
    return numeric_list(
        raw_joints,
        f"dispenser_outlets.{dispenser_id}.cup_place_joints_deg",
        6,
    )


def load_cup_holder_target_posx(name: str) -> list[float]:
    data = yaml.safe_load(CALIBRATION_CONFIG.read_text(encoding="utf-8")) or {}
    holder = data.get("cup_holder")
    if not isinstance(holder, dict):
        raise ValueError(f"cup_holder section is missing in {CALIBRATION_CONFIG}")
    block = holder.get("side_grip_place")
    if not isinstance(block, dict):
        raise ValueError(f"cup_holder.side_grip_place section is missing in {CALIBRATION_CONFIG}")
    xyz_m = numeric_list(
        block.get(f"{name}_pose_xyz_m"),
        f"cup_holder.side_grip_place.{name}_pose_xyz_m",
        3,
    )
    rpy_rad = numeric_list(
        block.get(f"{name}_pose_rpy_rad"),
        f"cup_holder.side_grip_place.{name}_pose_rpy_rad",
        3,
    )
    return [
        xyz_m[0] * 1000.0,
        xyz_m[1] * 1000.0,
        xyz_m[2] * 1000.0,
        math.degrees(rpy_rad[0]),
        math.degrees(rpy_rad[1]),
        math.degrees(rpy_rad[2]),
    ]


def format_joints_deg(joints: list[float]) -> str:
    return "[" + ", ".join(f"{value:.2f}" for value in joints) + "]"


def print_dry_run_group_detail(args: argparse.Namespace, dispenser_id: str, press_count: int) -> None:
    cup_place = load_cup_place_joints_deg(dispenser_id)
    cup_common_pre = load_cup_common_pre_joints_deg(dispenser_id) if args.use_cup_common_pre else None
    if args.skip_release_pre:
        if cup_place is not None:
            print(f"[Azas] DISP_PLACE={format_joints_deg(cup_place)}")
        staging_step = "RELEASE_STAGING -> " if args.use_release_staging else ""
        print(
            f"[PLAN] dispenser {dispenser_id}: safe release (skip_release_pre) "
            f"START_SAFE_LIFT(Z-only, >= {max(args.release_min_transit_z_m, 0.0):.3f}m) -> {staging_step}"
            f"RELEASE_ABOVE(final+{max(args.release_approach_lift_m, 0.0) * 1000.0:.0f}mm) -> "
            f"vertical RELEASE_FINAL -> OPEN -> "
            f"POST_RELEASE_LIFT(+{max(args.post_release_safe_lift_m, 0.0) * 1000.0:.0f}mm)"
        )
    elif cup_place is not None:
        if cup_common_pre is not None:
            print(f"[Azas] CUP_COMMON_PRE={format_joints_deg(cup_common_pre)}")
        print(f"[Azas] DISP_PLACE={format_joints_deg(cup_place)}")
        if cup_common_pre is not None:
            print(f"[PLAN] dispenser {dispenser_id}: cup CUP_COMMON_PRE -> DISP_PLACE -> RELEASE")
        else:
            extra_x_m = args.dispenser_3_cup_pre_extra_x_offset_m if str(dispenser_id) == "3" else 0.0
            total_x_offset_m = args.cup_pre_from_place_x_offset_m + extra_x_m
            print(
                f"[PLAN] dispenser {dispenser_id}: cup generated DISP_PRE "
                f"(DISP_PLACE X{total_x_offset_m * 1000.0:+.0f}mm "
                f"Z{args.cup_pre_from_place_z_offset_m * 1000.0:+.0f}mm) "
                "-> DISP_PLACE -> RELEASE"
            )
    else:
        print(f"[PLAN] dispenser {dispenser_id}: cup placement fallback=front_hold_poses")

    press_contact = load_press_ready_joints_deg(dispenser_id)
    press_pre = (
        load_press_pre_joints_deg(dispenser_id)
        if press_contact is not None and not args.skip_measured_press_pre
        else None
    )
    press_common_pre = load_press_common_pre_joints_deg(dispenser_id) if press_contact is not None else None
    z_overdrive_mm = (max(args.press_depth_m, 0.0) + max(args.press_extra_depth_m, 0.0)) * 1000.0
    if press_contact is not None and press_pre is not None and not args.skip_measured_press_pre:
        print(f"[Azas] PRESS_PRE={format_joints_deg(press_pre)}")
        print(f"[Azas] PRESS_CONTACT={format_joints_deg(press_contact)}")
        print(
            f"[PLAN] dispenser {dispenser_id}: press PRESS_PRE "
            f"-> CONTACT_ENTRY_LIFT +{max(args.press_contact_entry_lift_m, 0.0) * 1000.0:.1f}mm -> PRESS_CONTACT "
            f"-> PRESS_Z_OVERDRIVE {z_overdrive_mm:.1f}mm -> PRESS_CONTACT -> PRESS_PRE x{press_count}"
        )
    elif press_contact is not None:
        if args.press_reset_before_press and press_common_pre is not None:
            print(f"[Azas] PRESS_COMMON_PRE={format_joints_deg(press_common_pre)}")
        print(f"[Azas] PRESS_CONTACT={format_joints_deg(press_contact)}")
        if args.press_reset_before_press:
            pre_step = "PRESS_COMMON_PRE -> " if press_common_pre is not None else "PRESS_RESET_HOME -> "
        else:
            pre_step = ""
        print(
            f"[PLAN] dispenser {dispenser_id}: press {pre_step}CONTACT_ENTRY_LIFT "
            f"(CONTACT+Z{max(args.press_contact_entry_lift_m, 0.0) * 1000.0:.1f}mm) "
            f"-> PRESS_CONTACT -> "
            f"(PRESS_Z_OVERDRIVE {z_overdrive_mm:.1f}mm -> PRESS_CONTACT) x{press_count} "
            "-> POST_PRESS_SAFE_LIFT"
        )
    else:
        press_xyz_m, press_rpy_deg = load_press_pose(dispenser_id)
        print(
            f"[PLAN] dispenser {dispenser_id}: press Cartesian fallback xyz_m={press_xyz_m} "
            f"rpy_deg={press_rpy_deg} -> Z overdrive={z_overdrive_mm:.1f}mm x{press_count}"
        )
    press_y_offset_m = dispenser_press_y_offset_m(args, dispenser_id)
    if abs(press_y_offset_m) > 1e-9:
        print(
            f"[PLAN] dispenser {dispenser_id}: press target Y offset "
            f"{press_y_offset_m * 1000.0:+.1f}mm applied at runtime; calibration.yaml unchanged"
        )


def dispenser_press_y_offset_m(args: argparse.Namespace, dispenser_id: str) -> float:
    if str(dispenser_id) == "1":
        return float(args.dispenser_1_press_y_offset_m)
    return 0.0


def apply_dispenser_press_y_offset(
    args: argparse.Namespace,
    dispenser_id: str,
    posx: list[float],
    *,
    label: str,
) -> list[float]:
    adjusted = list(posx)
    offset_m = dispenser_press_y_offset_m(args, dispenser_id)
    if abs(offset_m) <= 1e-9:
        return adjusted
    before_y = adjusted[1]
    adjusted[1] += offset_m * 1000.0
    print(
        "[Azas] press target runtime offset: "
        f"dispenser={dispenser_id} label={label} y_mm={before_y:.1f}->{adjusted[1]:.1f} "
        f"offset={offset_m * 1000.0:+.1f}mm"
    )
    return adjusted


def group_consecutive_dispenser_ids(dispenser_ids: list[str]) -> list[tuple[str, int]]:
    groups: list[tuple[str, int]] = []
    for dispenser_id in dispenser_ids:
        if groups and groups[-1][0] == dispenser_id:
            previous_id, count = groups[-1]
            groups[-1] = (previous_id, count + 1)
        else:
            groups.append((dispenser_id, 1))
    return groups


RESUME_STAGES = ("move_release", "press", "regrasp")
RESUME_STAGE_LABELS = {
    "move_release": "move/release",
    "press": "press",
    "regrasp": "re-grasp/lift",
    "cup_holder": "cup-holder place",
}


def grouped_resume_payload(groups: list[tuple[str, int]]) -> list[dict[str, object]]:
    return [
        {"dispenser_id": dispenser_id, "press_count": int(press_count)}
        for dispenser_id, press_count in groups
    ]


def resume_recipe_token(dispenser_ids: list[str]) -> str:
    return ",".join(dispenser_ids)


def resume_stage_index(stage: str) -> int:
    try:
        return RESUME_STAGES.index(stage)
    except ValueError as exc:
        raise ValueError(f"unknown resume stage: {stage}") from exc


class RecipeResumeTracker:
    """Durably records the next robot step after each successful stage.

    The checkpoint stores symbolic dispenser IDs and stage names only.  It does
    not persist or synthesize robot coordinates; all poses still come from the
    measured calibration/vision-derived path used by the runner.
    """

    def __init__(
        self,
        args: argparse.Namespace,
        dispenser_ids: list[str],
        grouped_dispenser_ids: list[tuple[str, int]],
    ) -> None:
        self.enabled = bool(args.execute)
        self.resume_enabled = bool(args.resume)
        self.path = Path(args.resume_state_file)
        self.recipe_token = resume_recipe_token(dispenser_ids)
        self.groups = grouped_resume_payload(grouped_dispenser_ids)
        self.total_groups = len(grouped_dispenser_ids)
        self.next_group_index = 1
        self.next_stage = "move_release"
        self.loaded = False

        if not self.enabled:
            return
        if args.clear_resume_state:
            self.clear()
        if self.resume_enabled:
            self._load_if_present()
        self._write(status="running")

    def clear(self) -> None:
        try:
            self.path.unlink()
            print(f"[Azas] resume_state cleared: {self.path}")
        except FileNotFoundError:
            pass

    def _base_payload(self) -> dict[str, object]:
        return {
            "version": 1,
            "runner": Path(__file__).name,
            "recipe_token": self.recipe_token,
            "groups": self.groups,
            "total_groups": self.total_groups,
            "next_group_index": self.next_group_index,
            "next_stage": self.next_stage,
            "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        }

    def _write(
        self,
        *,
        status: str,
        current_group_index: int | None = None,
        current_stage: str | None = None,
        dispenser_id: str | None = None,
        press_count: int | None = None,
    ) -> None:
        if not self.enabled:
            return
        payload = self._base_payload()
        payload["status"] = status
        if current_group_index is not None:
            payload["current_group_index"] = current_group_index
        if current_stage is not None:
            payload["current_stage"] = current_stage
        if dispenser_id is not None:
            payload["current_dispenser_id"] = dispenser_id
        if press_count is not None:
            payload["current_press_count"] = press_count
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def _load_if_present(self) -> None:
        if not self.path.is_file():
            return
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(
                f"resume state is unreadable: {self.path} ({exc}); "
                "pass --clear-resume-state only after confirming the robot/cup state is safe"
            ) from exc
        if not isinstance(payload, dict):
            raise ValueError(f"resume state is invalid: {self.path}")

        status = str(payload.get("status") or "")
        stored_recipe = str(payload.get("recipe_token") or "")
        stored_groups = payload.get("groups")
        if stored_recipe != self.recipe_token or stored_groups != self.groups:
            if status == "completed":
                print(f"[Azas] resume_state completed for a different recipe; starting fresh: {self.path}")
                return
            raise ValueError(
                "resume state belongs to a different unfinished recipe. "
                f"state={self.path} stored_recipe={stored_recipe!r} requested_recipe={self.recipe_token!r}; "
                "use the same --dispenser-ids to resume or pass --clear-resume-state after manual safety review"
            )
        if status == "completed":
            print(f"[Azas] resume_state already completed for this recipe; starting fresh: {self.path}")
            return

        try:
            next_group_index = int(payload.get("next_group_index", 1))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"resume state has invalid next_group_index: {self.path}") from exc
        next_stage = str(payload.get("next_stage") or "move_release")
        if not 1 <= next_group_index <= self.total_groups + 1:
            raise ValueError(f"resume state next_group_index is out of range: {next_group_index}")
        if next_stage not in (*RESUME_STAGES, "cup_holder"):
            raise ValueError(f"resume state next_stage is invalid: {next_stage!r}")
        if next_group_index <= self.total_groups and next_stage == "cup_holder":
            raise ValueError("resume state cannot enter cup_holder before all dispenser groups complete")

        self.next_group_index = next_group_index
        self.next_stage = next_stage
        self.loaded = True
        print(
            f"[Azas] resume_state loaded: {self.path} "
            f"next_group={self.next_group_index}/{self.total_groups} "
            f"next_stage={RESUME_STAGE_LABELS.get(self.next_stage, self.next_stage)}"
        )

    def should_run_stage(self, group_index: int, stage: str) -> bool:
        if not self.enabled:
            return True
        if group_index < self.next_group_index:
            return False
        if group_index > self.next_group_index:
            return True
        if self.next_stage == "cup_holder":
            return False
        return resume_stage_index(stage) >= resume_stage_index(self.next_stage)

    def start_stage(self, group_index: int, stage: str, dispenser_id: str, press_count: int) -> None:
        if not self.enabled:
            return
        self.next_group_index = group_index
        self.next_stage = stage
        self._write(
            status="running",
            current_group_index=group_index,
            current_stage=stage,
            dispenser_id=dispenser_id,
            press_count=press_count,
        )
        print(
            f"[Azas] resume_state step_start: group={group_index}/{self.total_groups} "
            f"stage={RESUME_STAGE_LABELS[stage]} state={self.path}"
        )

    def complete_stage(self, group_index: int, stage: str) -> None:
        if not self.enabled:
            return
        if stage == "move_release":
            self.next_group_index = group_index
            self.next_stage = "press"
        elif stage == "press":
            self.next_group_index = group_index
            self.next_stage = "regrasp"
        elif stage == "regrasp":
            self.next_group_index = group_index + 1
            self.next_stage = "move_release"
        else:
            raise ValueError(f"cannot complete unknown stage: {stage}")
        self._write(status="running")
        print(
            f"[Azas] resume_state step_done: group={group_index}/{self.total_groups} "
            f"stage={RESUME_STAGE_LABELS[stage]} next_group={self.next_group_index} "
            f"next_stage={RESUME_STAGE_LABELS.get(self.next_stage, self.next_stage)}"
        )

    def should_run_cup_holder(self) -> bool:
        if not self.enabled:
            return True
        return self.next_group_index >= self.total_groups + 1

    def start_cup_holder(self) -> None:
        if not self.enabled:
            return
        self.next_group_index = self.total_groups + 1
        self.next_stage = "cup_holder"
        self._write(status="running", current_group_index=self.next_group_index, current_stage="cup_holder")
        print(f"[Azas] resume_state step_start: final stage=cup-holder place state={self.path}")

    def complete_all(self) -> None:
        if not self.enabled:
            return
        self.next_group_index = self.total_groups + 1
        self.next_stage = "cup_holder"
        self._write(status="completed")
        print(f"[Azas] resume_state completed: {self.path}")


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
        self.set_current_tcp = self.node.create_client(SetCurrentTcp, service_name(args.service_prefix, "tcp/set_current_tcp"))
        self.get_current_tcp = self.node.create_client(GetCurrentTcp, service_name(args.service_prefix, "tcp/get_current_tcp"))
        self.gripper = self.node.create_client(SetGripper, args.gripper_service)
        self.previous_tcp_name: str | None = None

    def close(self) -> None:
        self.restore_tcp_if_needed()
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
        if self.args.dispenser_tcp_name:
            required.extend(
                [
                    (self.set_current_tcp, "SetCurrentTcp"),
                    (self.get_current_tcp, "GetCurrentTcp"),
                ]
            )
        missing = [
            f"{label} ({getattr(client, 'srv_name', '<unknown service>')})"
            for client, label in required
            if not client.wait_for_service(timeout_sec=max(self.args.wait_service_sec, 0.1))
        ]
        if missing:
            raise RuntimeError("required service(s) unavailable before motion: " + ", ".join(missing))
        self.configure_tcp()

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

    def current_tcp_name(self) -> str:
        response = self._call(
            self.get_current_tcp,
            GetCurrentTcp.Request(),
            timeout_sec=self.args.wait_service_sec,
            label="GetCurrentTcp",
        )
        if not response.success:
            raise RuntimeError("GetCurrentTcp returned success=false")
        return str(response.info).strip()

    def set_tcp_name(self, name: str, *, label: str) -> None:
        request = SetCurrentTcp.Request()
        request.name = str(name).strip()
        print(f"[Azas] {label}: setting Doosan current TCP to {request.name or '<empty/default>'}")
        response = self._call(
            self.set_current_tcp,
            request,
            timeout_sec=self.args.wait_service_sec,
            label=label,
        )
        if not response.success:
            raise RuntimeError(f"{label} returned success=false")

    def configure_tcp(self) -> None:
        requested = str(self.args.dispenser_tcp_name).strip()
        if not requested:
            print("[WARN] dispenser_tcp_name is empty; keeping current Doosan TCP")
            return
        self.previous_tcp_name = self.current_tcp_name()
        print(
            "[Azas] Doosan TCP before measured recipe: "
            f"{self.previous_tcp_name if self.previous_tcp_name else '<empty/default>'}"
        )
        if self.previous_tcp_name != requested:
            try:
                self.set_tcp_name(requested, label="SetCurrentTcp before measured recipe")
            except RuntimeError as exc:
                if not self.args.allow_tcp_set_failure:
                    raise RuntimeError(
                        f"failed to set Doosan TCP '{requested}': {exc}. "
                        "The measured press poses require this TCP; refusing real press motion."
                    ) from exc
                print(f"[WARN] failed to set TCP '{requested}': {exc}; continuing because allow_tcp_set_failure=true")
                return
        current = self.current_tcp_name()
        print(f"[Azas] Doosan TCP active for measured recipe: {current if current else '<empty/default>'}")
        if current != requested and not self.args.allow_tcp_set_failure:
            raise RuntimeError(
                f"requested TCP '{requested}' but controller reports '{current}'. "
                "Refusing real press motion because FK/press poses would use the wrong TCP."
            )

    def restore_tcp_if_needed(self) -> None:
        requested = str(self.args.dispenser_tcp_name).strip()
        if not self.args.restore_tcp_after_run:
            return
        if self.previous_tcp_name is None or self.previous_tcp_name == requested:
            return
        try:
            self.set_tcp_name(self.previous_tcp_name, label="SetCurrentTcp restore after measured recipe")
            current = self.current_tcp_name()
            print(f"[Azas] restored Doosan TCP after measured recipe: {current if current else '<empty/default>'}")
        except RuntimeError as exc:
            print(f"[WARN] failed to restore previous Doosan TCP after measured recipe: {exc}", file=sys.stderr)

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
        verify_tolerance_mm: float | None = None,
        max_joint_delta_deg: float | None = None,
    ) -> None:
        self.validate_cartesian_target_z(posx_mm_deg, label=label)
        joints_deg = self.ikin_posj(posx_mm_deg, label=f"{label} IK joint fallback")
        joints_deg = self.normalize_ik_joints_near_current(joints_deg, label=label)
        self.validate_ik_fallback_joints(joints_deg, label=label, max_joint_delta_deg=max_joint_delta_deg)
        self.movej(
            joints_deg,
            label=f"{label} IK MoveJoint fallback",
            velocity=velocity,
            acceleration=acceleration,
        )
        self.wait_for_target(
            posx_mm_deg,
            label=f"{label} IK MoveJoint fallback posx",
            tolerance_mm=verify_tolerance_mm,
        )

    def measured_cup_place_posx_with_release_offset(self, joints_deg: list[float], *, label: str) -> list[float]:
        posx = self.fkin_posx(joints_deg, label=f"{label} measured FK")
        adjusted = list(posx)
        adjusted[0] += self.args.move_release_offset_x_m * 1000.0
        adjusted[1] += self.args.move_release_offset_y_m * 1000.0
        adjusted[2] += self.args.move_release_offset_z_m * 1000.0
        print(
            "[Azas] "
            f"{label}: applying measured cup-place offset "
            f"dx={self.args.move_release_offset_x_m * 1000.0:.1f}mm "
            f"dy={self.args.move_release_offset_y_m * 1000.0:.1f}mm "
            f"dz={self.args.move_release_offset_z_m * 1000.0:.1f}mm "
            f"target_posx=[{adjusted[0]:.1f}, {adjusted[1]:.1f}, {adjusted[2]:.1f}, "
            f"{adjusted[3]:.1f}, {adjusted[4]:.1f}, {adjusted[5]:.1f}]"
        )
        return adjusted

    def move_measured_cup_place_with_offset(
        self,
        joints_deg: list[float],
        *,
        label: str,
        velocity: float,
        acceleration: float,
    ) -> None:
        if (
            abs(self.args.move_release_offset_x_m) <= 1e-6
            and abs(self.args.move_release_offset_y_m) <= 1e-6
            and abs(self.args.move_release_offset_z_m) <= 1e-6
        ):
            joints_deg = self.normalize_joints_near_current(joints_deg, label=label)
            self.movej(joints_deg, label=label, velocity=velocity, acceleration=acceleration)
            return
        target = self.measured_cup_place_posx_with_release_offset(joints_deg, label=label)
        self.move_posx_joint_fallback(
            target,
            label=f"{label} with release offset",
            velocity=velocity,
            acceleration=acceleration,
        )

    def move_measured_cup_place_via_high_entry(
        self,
        joints_deg: list[float],
        *,
        dispenser_id: str,
    ) -> None:
        final_target = self.measured_cup_place_posx_with_release_offset(
            joints_deg,
            label=f"move to measured DISP{dispenser_id}_PLACE cup-place joints",
        )
        high_target = list(final_target)
        desired_high_z = max(
            final_target[2] + max(self.args.move_prehold_offset_z_m, 0.0) * 1000.0,
            max(self.args.regrasp_min_transit_z_m, 0.0) * 1000.0,
        )
        max_high_z = max(self.args.regrasp_max_transit_z_m, 0.0) * 1000.0
        if max_high_z > 0.0:
            desired_high_z = min(desired_high_z, max_high_z)
        high_target[2] = max(final_target[2], desired_high_z)
        print(
            "[Azas] "
            f"measured cup placement high entry: dispenser={dispenser_id} "
            f"target_posx=[{high_target[0]:.1f}, {high_target[1]:.1f}, {high_target[2]:.1f}, "
            f"{high_target[3]:.1f}, {high_target[4]:.1f}, {high_target[5]:.1f}]"
        )
        self.move_posx_joint_fallback(
            high_target,
            label=f"measured DISP{dispenser_id} high entry before cup place",
            velocity=self.args.move_prehold_velocity,
            acceleration=self.args.move_prehold_acceleration,
        )
        self.move_posx(
            final_target,
            label=f"final measured DISP{dispenser_id} cup-place descent",
            velocity=self.args.move_velocity,
            acceleration=self.args.move_acceleration,
            timeout_sec=self.args.move_timeout_sec,
        )

    def move_measured_cup_regrasp_with_rear_entry(
        self,
        joints_deg: list[float],
        *,
        dispenser_id: str,
        final_regrasp: bool = False,
    ) -> None:
        final_target = self.measured_cup_place_posx_with_release_offset(
            joints_deg,
            label=f"return to measured DISP{dispenser_id}_PLACE cup re-grasp joints",
        )
        if final_regrasp:
            dx_mm = self.args.final_regrasp_extra_x_offset_m * 1000.0
            dy_mm = self.args.final_regrasp_extra_y_offset_m * 1000.0
            dz_mm = self.args.final_regrasp_extra_z_offset_m * 1000.0
            if abs(dx_mm) > 1e-6 or abs(dy_mm) > 1e-6 or abs(dz_mm) > 1e-6:
                final_target = list(final_target)
                final_target[0] += dx_mm
                final_target[1] += dy_mm
                final_target[2] += dz_mm
                print(
                    "[Azas] "
                    f"final cup re-grasp tighter target before holder: dispenser={dispenser_id} "
                    f"dx={dx_mm:.1f}mm dy={dy_mm:.1f}mm dz={dz_mm:.1f}mm "
                    f"target_posx=[{final_target[0]:.1f}, {final_target[1]:.1f}, {final_target[2]:.1f}, "
                    f"{final_target[3]:.1f}, {final_target[4]:.1f}, {final_target[5]:.1f}]"
                )
        rear_target = list(final_target)
        rear_target[0] += self.args.regrasp_rear_entry_offset_x_m * 1000.0
        rear_target[1] += self.args.regrasp_rear_entry_offset_y_m * 1000.0
        high_rear_target = list(rear_target)
        desired_high_z = max(
            final_target[2] + max(self.args.regrasp_approach_offset_z_m, 0.0) * 1000.0,
            max(self.args.regrasp_min_transit_z_m, 0.0) * 1000.0,
        )
        max_high_z = max(self.args.regrasp_max_transit_z_m, 0.0) * 1000.0
        if max_high_z > 0.0:
            desired_high_z = min(desired_high_z, max_high_z)
        high_rear_target[2] = max(rear_target[2], desired_high_z)
        if (
            abs(self.args.regrasp_rear_entry_offset_x_m) > 1e-6
            or abs(self.args.regrasp_rear_entry_offset_y_m) > 1e-6
        ):
            print(
                "[Azas] "
                f"measured cup re-grasp high rear entry: dispenser={dispenser_id} "
                f"target_posx=[{high_rear_target[0]:.1f}, {high_rear_target[1]:.1f}, "
                f"{high_rear_target[2]:.1f}, {high_rear_target[3]:.1f}, "
                f"{high_rear_target[4]:.1f}, {high_rear_target[5]:.1f}]"
            )
            self.move_posx_joint_fallback(
                high_rear_target,
                label=f"measured DISP{dispenser_id} high rear entry before cup re-grasp",
                velocity=self.args.regrasp_approach_velocity,
                acceleration=self.args.regrasp_approach_acceleration,
            )
            print(
                "[Azas] "
                f"measured cup re-grasp lowered rear entry: dispenser={dispenser_id} "
                f"dx={self.args.regrasp_rear_entry_offset_x_m * 1000.0:.1f}mm "
                f"dy={self.args.regrasp_rear_entry_offset_y_m * 1000.0:.1f}mm "
                f"target_posx=[{rear_target[0]:.1f}, {rear_target[1]:.1f}, {rear_target[2]:.1f}, "
                f"{rear_target[3]:.1f}, {rear_target[4]:.1f}, {rear_target[5]:.1f}]"
            )
            self.move_posx_joint_fallback(
                rear_target,
                label=f"measured DISP{dispenser_id} lowered rear entry before cup re-grasp",
                velocity=self.args.pick_approach_velocity,
                acceleration=self.args.pick_approach_acceleration,
            )
        self.move_posx(
            final_target,
            label=f"final measured DISP{dispenser_id} re-grasp forward approach to cup",
            velocity=self.args.pick_approach_velocity,
            acceleration=self.args.pick_approach_acceleration,
            timeout_sec=self.args.pick_timeout_sec,
        )

    def move_front_hold_joint_fallback(self, posx_mm_deg: list[float], *, label: str) -> None:
        joints_deg = self.ikin_posj(posx_mm_deg, label=f"{label} IK joint fallback")
        joints_deg = self.normalize_ik_joints_near_current(joints_deg, label=label)
        self.validate_ik_fallback_joints(joints_deg, label=label)
        self.movej(
            joints_deg,
            label=f"{label} IK MoveJoint fallback",
            velocity=self.args.front_hold_joint_fallback_velocity,
            acceleration=self.args.front_hold_joint_fallback_acceleration,
        )
        self.wait_for_target(posx_mm_deg, label=f"{label} IK MoveJoint fallback posx")

    def validate_cartesian_target_z(self, pos_mm_deg: list[float], *, label: str) -> None:
        min_allowed_mm = max(self.args.min_allowed_tcp_z_m, 0.0) * 1000.0
        if pos_mm_deg[2] < min_allowed_mm - 1e-6:
            raise RuntimeError(
                f"unsafe cartesian target for {label}: "
                f"target_z={pos_mm_deg[2] / 1000.0:.3f}m < min_allowed_tcp_z={min_allowed_mm / 1000.0:.3f}m; "
                "no robot command sent"
            )

    def ensure_start_safe_pose(self) -> None:
        """Read the live TCP pose and lift straight up before the first XY waypoint."""
        try:
            pose = self.current_posx(timeout_sec=self.args.wait_service_sec)
        except RuntimeError as exc:
            raise RuntimeError(
                f"start TCP pose read failed; aborting before any motion: {exc}"
            ) from exc
        current_z_m = pose[2] / 1000.0
        min_allowed_z_m = max(self.args.min_allowed_tcp_z_m, 0.0)
        start_lift_z_m = max(self.args.start_safe_lift_z_m, min_allowed_z_m, 0.0)
        print(
            "[Azas] start TCP pose: "
            f"posx=[{pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}, "
            f"{pose[3]:.1f}, {pose[4]:.1f}, {pose[5]:.1f}] "
            f"current_z={current_z_m:.3f}m start_safe_lift_z={start_lift_z_m:.3f}m "
            f"min_allowed_tcp_z={min_allowed_z_m:.3f}m "
            f"force_start_safe_lift={self.args.force_start_safe_lift}"
        )
        if current_z_m < min_allowed_z_m:
            print(
                f"[WARN] live TCP Z {current_z_m:.3f}m is below min_allowed_tcp_z "
                f"{min_allowed_z_m:.3f}m; XY motion stays blocked until the Z-only "
                "start safe lift completes"
            )
        if not self.args.force_start_safe_lift and current_z_m >= start_lift_z_m:
            print(
                "[Azas] start safe lift skipped: live TCP already at or above "
                f"start_safe_lift_z={start_lift_z_m:.3f}m and force_start_safe_lift=false"
            )
            return
        self.safe_lift_current(
            label="START_SAFE_LIFT Z-only lift from live TCP pose before first waypoint",
            min_z_m=start_lift_z_m,
            velocity=self.args.move_prehold_velocity,
            acceleration=self.args.move_prehold_acceleration,
            timeout_sec=self.args.move_timeout_sec,
        )

    def safe_lift_current(
        self,
        *,
        label: str,
        min_z_m: float,
        velocity: float,
        acceleration: float,
        timeout_sec: float,
        verify_tolerance_mm: float | None = None,
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
        safe_lift_tolerance_mm = (
            self.args.safe_lift_target_tolerance_mm
            if verify_tolerance_mm is None
            else verify_tolerance_mm
        )
        try:
            self.move_posx(
                target,
                label=label,
                velocity=velocity,
                acceleration=acceleration,
                timeout_sec=timeout_sec,
                verify_tolerance_mm=safe_lift_tolerance_mm,
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
                verify_tolerance_mm=safe_lift_tolerance_mm,
            )

    def move_posx(
        self,
        pos: list[float],
        *,
        label: str,
        velocity: float,
        acceleration: float,
        timeout_sec: float,
        verify_tolerance_mm: float | None = None,
    ) -> None:
        self.validate_cartesian_target_z(pos, label=label)
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
        self.wait_for_target(pos, label=label, tolerance_mm=verify_tolerance_mm)

    def move_generated_press_pre(self, pos: list[float], *, label: str) -> None:
        if self.args.press_generated_pre_use_joint:
            print(
                f"[Azas] {label}: using IK MoveJoint for generated press entry approach; "
                "CONTACT/PRESS stroke remains Z-only MoveLine"
            )
            self.move_posx_joint_fallback(
                pos,
                label=label,
                velocity=self.args.press_contact_joint_velocity,
                acceleration=self.args.press_contact_joint_acceleration,
            )
            return
        try:
            self.move_posx(
                pos,
                label=label,
                velocity=self.args.press_travel_velocity,
                acceleration=self.args.press_travel_acceleration,
                timeout_sec=self.args.press_timeout_sec,
            )
        except RuntimeError as exc:
            if not self.args.press_generated_pre_joint_fallback:
                raise
            print(
                f"[WARN] MoveLine/verification failed for {label}: {exc}; "
                "retrying the same generated press entry target with IK MoveJoint fallback"
            )
            self.move_posx_joint_fallback(
                pos,
                label=label,
                velocity=self.args.press_contact_joint_velocity,
                acceleration=self.args.press_contact_joint_acceleration,
            )

    def move_posx_no_verify(
        self,
        pos: list[float],
        *,
        label: str,
        velocity: float,
        acceleration: float,
        timeout_sec: float,
    ) -> None:
        self.validate_cartesian_target_z(pos, label=label)
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

    def normalize_ik_joints_near_current(self, joints_deg: list[float], *, label: str) -> list[float]:
        """Prevent wrist wrap by commanding the nearest equivalent joint branch."""
        current = self.current_posj(timeout_sec=5.0)
        max_abs = max(float(self.args.ik_fallback_max_abs_joint_deg), 0.0)
        normalized = [
            equivalent_angle_near_current_deg(target, current_value, max_abs=max_abs)
            for target, current_value in zip(joints_deg, current, strict=True)
        ]
        changed = [
            f"joint_{index + 1} {before:.1f}->{after:.1f}deg current={current[index]:.1f}"
            for index, (before, after) in enumerate(zip(joints_deg, normalized, strict=True))
            if abs(before - after) > 1.0
        ]
        if changed:
            print(
                "[Azas] "
                f"{label}: normalized joints near current to avoid wrist wrap: "
                + "; ".join(changed)
            )
        return normalized

    def normalize_joints_near_current(self, joints_deg: list[float], *, label: str) -> list[float]:
        if self.args.normalize_measured_joints_near_current:
            return self.normalize_ik_joints_near_current(joints_deg, label=label)
        print(
            "[Azas] "
            f"{label}: using measured joint values exactly; no 360deg wrap normalization"
        )
        return list(joints_deg)

    def validate_ik_fallback_joints(
        self,
        joints_deg: list[float],
        *,
        label: str,
        max_joint_delta_deg: float | None = None,
    ) -> None:
        max_abs = max(float(self.args.ik_fallback_max_abs_joint_deg), 0.0)
        if max_abs > 0.0:
            for index, value in enumerate(joints_deg, start=1):
                if abs(value) > max_abs:
                    raise RuntimeError(
                        f"IK fallback rejected for {label}: joint_{index}={value:.1f}deg "
                        f"exceeds limit {max_abs:.1f}deg"
                    )
        delta_limit = self.args.ik_fallback_max_joint_delta_deg if max_joint_delta_deg is None else max_joint_delta_deg
        max_delta = max(float(delta_limit), 0.0)
        if max_delta <= 0.0:
            return
        current = self.current_posj(timeout_sec=5.0)
        deltas = [angular_delta_deg(joints_deg[index], current[index]) for index in range(6)]
        worst_delta = max(deltas)
        if worst_delta > max_delta:
            joint_index = deltas.index(worst_delta) + 1
            raise RuntimeError(
                f"IK fallback rejected for {label}: joint_{joint_index} delta "
                f"{worst_delta:.1f}deg exceeds limit {max_delta:.1f}deg"
            )

    def validate_press_contact_joints(
        self,
        dispenser_id: str,
        joints_deg: list[float],
        configured_xyz_mm: list[float],
    ) -> None:
        tolerance_mm = max(float(self.args.press_contact_joint_pose_tolerance_mm), 0.0)
        if tolerance_mm <= 0.0:
            return
        try:
            fk_posx = self.fkin_posx(joints_deg, label=f"press contact joint sanity dispenser {dispenser_id}")
        except RuntimeError as exc:
            if self.args.strict_press_contact_joint_pose_match:
                raise
            print(
                f"[WARN] press contact joint sanity skipped for dispenser {dispenser_id}: {exc}; "
                "continuing because measured press_contact_joints_deg are authoritative"
            )
            return
        distance_mm = math.dist(fk_posx[:3], configured_xyz_mm)
        print(
            "[Azas] press contact joint sanity: "
            f"dispenser={dispenser_id} configured_xyz_mm=[{configured_xyz_mm[0]:.1f}, "
            f"{configured_xyz_mm[1]:.1f}, {configured_xyz_mm[2]:.1f}] "
            f"fk_xyz_mm=[{fk_posx[0]:.1f}, {fk_posx[1]:.1f}, {fk_posx[2]:.1f}] "
            f"distance={distance_mm:.1f}mm tolerance={tolerance_mm:.1f}mm"
        )
        if distance_mm <= tolerance_mm:
            return
        if self.args.strict_press_contact_joint_pose_match and not self.args.allow_press_contact_joint_pose_mismatch:
            raise RuntimeError(
                f"press_contact_joints_deg for dispenser {dispenser_id} do not match "
                f"press_pose_xyz_m: FK distance={distance_mm:.1f}mm exceeds "
                f"{tolerance_mm:.1f}mm. Refusing real press motion; re-teach "
                "press_contact_joints_deg or run a non-motion preview."
            )
        print(
            f"[WARN] press_contact_joints_deg/FK mismatch for dispenser {dispenser_id}: "
            f"distance={distance_mm:.1f}mm exceeds {tolerance_mm:.1f}mm; continuing because "
            "measured press_contact_joints_deg are authoritative on this controller/TCP setup"
        )

    def wait_for_joint_target(self, target_joints_deg: list[float], *, label: str) -> None:
        deadline = time.monotonic() + max(self.args.verify_timeout_sec, 0.1)
        last_error = 999999.0
        best_error = last_error
        last_progress_time = time.monotonic()
        while time.monotonic() < deadline:
            actual = self.current_posj(timeout_sec=5.0)
            errors = [abs(angular_delta_deg(target_joints_deg[index], actual[index])) for index in range(6)]
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

    def wait_for_target(
        self,
        target_pos_mm_deg: list[float],
        *,
        label: str,
        tolerance_mm: float | None = None,
    ) -> None:
        deadline = time.monotonic() + max(self.args.verify_timeout_sec, 0.1)
        last_distance = 999999.0
        best_distance = last_distance
        last_progress_time = time.monotonic()
        tolerance = max(float(self.args.target_tolerance_mm if tolerance_mm is None else tolerance_mm), 0.1)
        while time.monotonic() < deadline:
            actual = self.current_posx(timeout_sec=5.0)
            last_distance = sum((actual[index] - target_pos_mm_deg[index]) ** 2 for index in range(3)) ** 0.5
            print(f"[Azas] verify {label}: distance={last_distance:.1f}mm tolerance={tolerance:.1f}mm")
            if last_distance <= tolerance:
                return
            if best_distance - last_distance >= max(self.args.target_stall_delta_mm, 0.1):
                best_distance = last_distance
                last_progress_time = time.monotonic()
            elif (
                self.args.target_stall_timeout_sec > 0.0
                and last_distance >= max(self.args.target_stall_min_distance_mm, tolerance)
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

    def validate_cup_holder_target(self, posx_mm_deg: list[float], *, label: str) -> None:
        self.validate_cartesian_target_z(posx_mm_deg, label=label)
        x_m = posx_mm_deg[0] / 1000.0
        y_m = posx_mm_deg[1] / 1000.0
        z_m = posx_mm_deg[2] / 1000.0
        if not (self.args.cup_holder_x_min_m <= x_m <= self.args.cup_holder_x_max_m):
            raise RuntimeError(
                f"cup-holder target {label} rejected: x={x_m:.3f}m outside "
                f"[{self.args.cup_holder_x_min_m:.3f}, {self.args.cup_holder_x_max_m:.3f}]"
            )
        if not (self.args.cup_holder_y_min_m <= y_m <= self.args.cup_holder_y_max_m):
            raise RuntimeError(
                f"cup-holder target {label} rejected: y={y_m:.3f}m outside "
                f"[{self.args.cup_holder_y_min_m:.3f}, {self.args.cup_holder_y_max_m:.3f}]"
            )
        if not (self.args.cup_holder_z_min_m <= z_m <= self.args.cup_holder_z_max_m):
            raise RuntimeError(
                f"cup-holder target {label} rejected: z={z_m:.3f}m outside "
                f"[{self.args.cup_holder_z_min_m:.3f}, {self.args.cup_holder_z_max_m:.3f}]"
            )

    def place_cup_in_holder(self) -> None:
        pre_place = load_cup_holder_target_posx("pre_place")
        place_final = load_cup_holder_target_posx("place_final")
        retreat = load_cup_holder_target_posx("retreat")
        rz_offset_deg = self.args.cup_holder_rz_offset_deg
        if abs(rz_offset_deg) > 1e-9:
            for posx in (pre_place, place_final, retreat):
                posx[5] += rz_offset_deg
        place_final[0] += self.args.cup_holder_place_final_x_offset_m * 1000.0
        place_final[1] += self.args.cup_holder_place_final_y_offset_m * 1000.0
        place_final[2] += self.args.cup_holder_place_final_z_offset_m * 1000.0
        for label, posx in (
            ("CUP_HOLDER_PRE_PLACE", pre_place),
            ("CUP_HOLDER_PLACE_FINAL", place_final),
            ("CUP_HOLDER_RETREAT", retreat),
        ):
            self.validate_cup_holder_target(posx, label=label)
            print(
                f"[Azas] {label}: posx=[{posx[0]:.1f}, {posx[1]:.1f}, {posx[2]:.1f}, "
                f"{posx[3]:.1f}, {posx[4]:.1f}, {posx[5]:.1f}]"
            )
        print(
            "[Azas] cup holder place path: "
            "CUP_HOLDER_PRE_PLACE -> CUP_HOLDER_PLACE_FINAL -> RG2_OPEN -> CUP_HOLDER_RETREAT"
        )
        self.move_posx(
            pre_place,
            label="CUP_HOLDER_PRE_PLACE measured side-grip approach",
            velocity=self.args.cup_holder_approach_velocity,
            acceleration=self.args.cup_holder_approach_acceleration,
            timeout_sec=self.args.cup_holder_timeout_sec,
            verify_tolerance_mm=self.args.cup_holder_target_tolerance_mm,
        )
        self.move_posx(
            place_final,
            label="CUP_HOLDER_PLACE_FINAL measured side-grip place",
            velocity=self.args.cup_holder_place_velocity,
            acceleration=self.args.cup_holder_place_acceleration,
            timeout_sec=self.args.cup_holder_timeout_sec,
            verify_tolerance_mm=self.args.cup_holder_target_tolerance_mm,
        )
        self.gripper_command(
            "open",
            width_m=self.args.gripper_open_width_m,
            force_n=self.args.gripper_open_force_n,
            label="RG2 full-open at cup holder",
        )
        self.move_posx(
            retreat,
            label="CUP_HOLDER_RETREAT measured side-grip retreat",
            velocity=self.args.cup_holder_retreat_velocity,
            acceleration=self.args.cup_holder_retreat_acceleration,
            timeout_sec=self.args.cup_holder_timeout_sec,
            verify_tolerance_mm=self.args.cup_holder_target_tolerance_mm,
        )

    def _log_release_pose(self, name: str, pos: list[float], *, min_required_z_mm: float) -> None:
        z_ok = pos[2] >= min_required_z_mm - 1e-6
        print(
            f"[Azas] release pose {name}: "
            f"posx=[{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}, {pos[3]:.1f}, {pos[4]:.1f}, {pos[5]:.1f}] "
            f"z={pos[2] / 1000.0:.3f}m min_required_z={min_required_z_mm / 1000.0:.3f}m z_ok={z_ok}"
        )
        if not z_ok:
            raise RuntimeError(
                f"release pose {name} failed z validation: "
                f"z={pos[2] / 1000.0:.3f}m < min_required_z={min_required_z_mm / 1000.0:.3f}m; "
                "no release command sent"
            )

    def release_final_posx(self, dispenser_id: str) -> list[float]:
        """Release final pose (unchanged from the legacy release target)."""
        cup_place_joints = load_cup_place_joints_deg(dispenser_id)
        if cup_place_joints is not None:
            print(f"[Azas] DISP_PLACE={format_joints_deg(cup_place_joints)}")
            return self.measured_cup_place_posx_with_release_offset(
                cup_place_joints,
                label=f"safe release measured DISP{dispenser_id}_PLACE",
            )
        position, quaternion, raw_zyz = load_front_hold_pose(self.args.config, dispenser_id)
        link6_position = [
            position[0] + self.args.move_release_offset_x_m,
            position[1] + self.args.move_release_offset_y_m,
            position[2] + self.args.move_release_offset_z_m,
        ]
        move_position, move_rotation = self.compensate_current_tcp(link6_position, quaternion)
        move_zyz = matrix_to_doosan_zyz_deg(move_rotation)
        print(
            f"[Azas] safe release final from front_hold fallback: dispenser={dispenser_id} "
            f"link6_target_m=[{link6_position[0]:.4f}, {link6_position[1]:.4f}, {link6_position[2]:.4f}] "
            f"raw_zyz_deg=[{raw_zyz[0]:.2f}, {raw_zyz[1]:.2f}, {raw_zyz[2]:.2f}]"
        )
        return [move_position[0] * 1000.0, move_position[1] * 1000.0, move_position[2] * 1000.0, *move_zyz]

    def _release_transit_move(self, pos: list[float], *, label: str) -> None:
        try:
            self.move_posx(
                pos,
                label=label,
                velocity=self.args.move_prehold_velocity,
                acceleration=self.args.move_prehold_acceleration,
                timeout_sec=self.args.move_timeout_sec,
                verify_tolerance_mm=self.args.safe_lift_target_tolerance_mm,
            )
        except RuntimeError as exc:
            if not self.args.safe_lift_joint_fallback:
                raise
            print(
                f"[WARN] MoveLine failed for {label}: {exc}; "
                "retrying the same target with IK MoveJoint fallback"
            )
            self.move_posx_joint_fallback(
                pos,
                label=label,
                velocity=self.args.safe_lift_joint_fallback_velocity,
                acceleration=self.args.safe_lift_joint_fallback_acceleration,
                verify_tolerance_mm=self.args.safe_lift_target_tolerance_mm,
            )

    def safe_release_cup(self, dispenser_id: str) -> None:
        """Posture-independent cup release.

        Converges from any joint state / TCP position: Z-only lift from the live
        pose, transit at a safe height, then a vertical-only descent onto the
        unchanged release final pose. Never moves directly from the current pose
        to the release final.
        """
        print(
            f"[Azas] safe cup release: dispenser={dispenser_id} "
            f"skip_release_pre={self.args.skip_release_pre} "
            f"use_release_staging={self.args.use_release_staging}"
        )
        try:
            current = self.current_posx(timeout_sec=self.args.wait_service_sec)
        except RuntimeError as exc:
            raise RuntimeError(
                f"release start TCP pose read failed; no release command sent: {exc}"
            ) from exc
        print(
            "[Azas] release current TCP pose: "
            f"posx=[{current[0]:.1f}, {current[1]:.1f}, {current[2]:.1f}, "
            f"{current[3]:.1f}, {current[4]:.1f}, {current[5]:.1f}]"
        )
        min_allowed_mm = max(self.args.min_allowed_tcp_z_m, 0.0) * 1000.0
        min_transit_mm = max(self.args.release_min_transit_z_m, 0.0) * 1000.0

        final_posx = self.release_final_posx(dispenser_id)
        above_posx = list(final_posx)
        above_posx[2] = final_posx[2] + max(self.args.release_approach_lift_m, 0.0) * 1000.0
        above_high_posx = list(above_posx)
        above_high_posx[2] = max(above_posx[2], min_transit_mm)
        staging_posx: list[float] | None = None
        if self.args.use_release_staging:
            staging_posx = list(above_high_posx)
            if self.args.release_staging_x_m is not None:
                staging_posx[0] = self.args.release_staging_x_m * 1000.0
            if self.args.release_staging_y_m is not None:
                staging_posx[1] = self.args.release_staging_y_m * 1000.0
            if self.args.release_staging_z_m is not None:
                staging_posx[2] = self.args.release_staging_z_m * 1000.0
        start_lift_posx = list(current)
        start_lift_posx[2] = max(
            current[2] + max(self.args.release_start_safe_lift_m, 0.0) * 1000.0,
            min_transit_mm,
        )
        post_release_posx = list(final_posx)
        post_release_posx[2] = final_posx[2] + max(self.args.post_release_safe_lift_m, 0.0) * 1000.0

        # Validate every pose before sending any robot command. XY-transit poses
        # must stay at or above the release transit height; the vertical descent
        # chain (above -> final) and the post-release lift only need the global
        # TCP Z floor.
        self._log_release_pose("release_start_safe_lift", start_lift_posx, min_required_z_mm=min_transit_mm)
        if staging_posx is not None:
            self._log_release_pose("release_staging", staging_posx, min_required_z_mm=min_transit_mm)
        else:
            print("[Azas] release staging pose disabled (--no-use-release-staging)")
        self._log_release_pose("release_above_high_entry", above_high_posx, min_required_z_mm=min_transit_mm)
        self._log_release_pose("release_above", above_posx, min_required_z_mm=min_allowed_mm)
        self._log_release_pose("release_final", final_posx, min_required_z_mm=min_allowed_mm)
        self._log_release_pose("post_release_lift", post_release_posx, min_required_z_mm=min_allowed_mm)
        if above_posx[2] < final_posx[2] - 1e-6:
            raise RuntimeError(
                "release_above pose ended up below release_final; check --release-approach-lift-m; "
                "no release command sent"
            )

        self._release_transit_move(
            start_lift_posx,
            label="release start vertical safe lift (Z-only from live pose)",
        )
        if staging_posx is not None and any(
            abs(staging_posx[index] - above_high_posx[index]) > 1.0 for index in range(3)
        ):
            self._release_transit_move(staging_posx, label="release staging pose")
        self._release_transit_move(
            above_high_posx,
            label="release high entry directly above release final",
        )
        if above_high_posx[2] - above_posx[2] > 1.0:
            self.move_posx(
                above_posx,
                label="vertical descent to release_above",
                velocity=self.args.move_velocity,
                acceleration=self.args.move_acceleration,
                timeout_sec=self.args.move_timeout_sec,
            )

        # Final descent is allowed only as a vertical drop from release_above:
        # the live pose must already match release_above in X/Y/orientation.
        live = self.current_posx(timeout_sec=self.args.wait_service_sec)
        xy_tolerance_mm = max(self.args.target_tolerance_mm, 20.0)
        orientation_tolerance_deg = 5.0
        xy_error_mm = max(abs(live[0] - above_posx[0]), abs(live[1] - above_posx[1]))
        orientation_error_deg = max(
            abs(angular_delta_deg(above_posx[index], live[index])) for index in (3, 4, 5)
        )
        if xy_error_mm > xy_tolerance_mm or orientation_error_deg > orientation_tolerance_deg:
            raise RuntimeError(
                "release final descent blocked: live pose is not vertically above release final "
                f"(xy_error={xy_error_mm:.1f}mm tol={xy_tolerance_mm:.1f}mm, "
                f"orientation_error={orientation_error_deg:.2f}deg tol={orientation_tolerance_deg:.2f}deg); "
                "no descent command sent"
            )
        self.move_posx(
            final_posx,
            label="vertical descent to release_final",
            velocity=self.args.move_velocity,
            acceleration=self.args.move_acceleration,
            timeout_sec=self.args.move_timeout_sec,
        )
        self.gripper_command(
            "open",
            width_m=self.args.gripper_open_width_m,
            force_n=self.args.gripper_open_force_n,
            label="RG2 full-open release",
        )
        print("[Azas] RG2 full-open release complete; continuing only after open settle wait")
        self.move_posx(
            post_release_posx,
            label="post-release vertical lift above released cup",
            velocity=self.args.pick_lift_velocity,
            acceleration=self.args.pick_lift_acceleration,
            timeout_sec=self.args.move_timeout_sec,
        )

    def move_and_release(self, dispenser_id: str) -> None:
        if self.args.skip_release_pre:
            self.safe_release_cup(dispenser_id)
            return
        cup_common_pre_joints = load_cup_common_pre_joints_deg(dispenser_id) if self.args.use_cup_common_pre else None
        cup_place_joints = load_cup_place_joints_deg(dispenser_id)
        if cup_place_joints is not None:
            if cup_common_pre_joints is not None:
                print(f"[Azas] CUP_COMMON_PRE={format_joints_deg(cup_common_pre_joints)}")
            print(f"[Azas] DISP_PLACE={format_joints_deg(cup_place_joints)}")
            final_target = self.measured_cup_place_posx_with_release_offset(
                cup_place_joints,
                label=f"DISP{dispenser_id}_PLACE source measured DISP_PLACE",
            )
            if cup_common_pre_joints is not None:
                print(
                    f"[Azas] cup placement: dispenser={dispenser_id} using "
                    "CUP_COMMON_PRE -> DISP_PLACE; saved cup_pre_place_joints_deg ignored"
                )
                self.movej(
                    cup_common_pre_joints,
                    label="CUP_COMMON_PRE common cup-place pre joints",
                    velocity=self.args.move_prehold_velocity,
                    acceleration=self.args.move_prehold_acceleration,
                )
            else:
                print(
                    f"[Azas] cup placement: dispenser={dispenser_id} using measured "
                    "DISP_PLACE with generated DISP_PRE from X offset; saved cup_pre_place_joints_deg ignored"
                )
                extra_x_m = (
                    self.args.dispenser_3_cup_pre_extra_x_offset_m
                    if str(dispenser_id) == "3"
                    else 0.0
                )
                total_x_offset_m = self.args.cup_pre_from_place_x_offset_m + extra_x_m
                pre_target = list(final_target)
                pre_target[0] += total_x_offset_m * 1000.0
                pre_target[2] += self.args.cup_pre_from_place_z_offset_m * 1000.0
                print(
                    "[Azas] generated cup pre: "
                    f"dispenser={dispenser_id} "
                    f"pre_x_offset={total_x_offset_m * 1000.0:.1f}mm "
                    f"base_pre_x_offset={self.args.cup_pre_from_place_x_offset_m * 1000.0:.1f}mm "
                    f"dispenser_3_extra_x_offset={extra_x_m * 1000.0:.1f}mm "
                    f"pre_z_offset={self.args.cup_pre_from_place_z_offset_m * 1000.0:.1f}mm "
                    f"target_posx=[{pre_target[0]:.1f}, {pre_target[1]:.1f}, {pre_target[2]:.1f}, "
                    f"{pre_target[3]:.1f}, {pre_target[4]:.1f}, {pre_target[5]:.1f}]"
                )
                pre_label = f"generated DISP{dispenser_id}_PRE from DISP_PLACE X/Z offset"
                if self.args.generated_cup_pre_use_joint_fallback:
                    print(
                        f"[Azas] {pre_label}: using IK MoveJoint fallback because "
                        "generated_cup_pre_use_joint_fallback=true"
                    )
                    self.move_posx_joint_fallback(
                        pre_target,
                        label=pre_label,
                        velocity=self.args.move_prehold_velocity,
                        acceleration=self.args.move_prehold_acceleration,
                        max_joint_delta_deg=self.args.generated_cup_pre_max_joint_delta_deg,
                    )
                else:
                    self.move_posx(
                        pre_target,
                        label=pre_label,
                        velocity=self.args.move_prehold_velocity,
                        acceleration=self.args.move_prehold_acceleration,
                        timeout_sec=self.args.move_timeout_sec,
                    )
            self.move_posx(
                final_target,
                label=f"DISP_PLACE measured DISP{dispenser_id} cup release",
                velocity=self.args.move_velocity,
                acceleration=self.args.move_acceleration,
                timeout_sec=self.args.move_timeout_sec,
            )
            self.gripper_command(
                "open",
                width_m=self.args.gripper_open_width_m,
                force_n=self.args.gripper_open_force_n,
                label="RG2 full-open release",
            )
            print("[Azas] RG2 full-open release complete; continuing only after open settle wait")
            return

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

    def regrasp_and_lift(self, dispenser_id: str, *, final_regrasp: bool = False) -> None:
        grasp_width_m = (
            self.args.final_regrasp_grasp_width_m
            if final_regrasp
            else self.args.gripper_grasp_width_m
        )
        grasp_force_n = self.args.final_regrasp_force_n if final_regrasp else self.args.gripper_force_n
        grasp_label = "RG2 tight final side-grasp before cup holder" if final_regrasp else "RG2 soft side-grasp"
        if self.args.regrasp_reset_before_cup:
            print(
                "[Azas] re-grasp transit: moving through HOME joint waypoint before returning to cup; "
                "skipping post-press Cartesian high-Z lift"
            )
            self.movej(
                list(self.args.regrasp_reset_joints_deg),
                label="move to re-grasp HOME joints before cup return",
                velocity=self.args.regrasp_reset_joint_velocity,
                acceleration=self.args.regrasp_reset_joint_acceleration,
            )
        else:
            self.safe_lift_current(
                label="safe vertical lift after press before re-grasp transit",
                min_z_m=self.args.post_press_safe_lift_z_m,
                velocity=self.args.regrasp_approach_velocity,
                acceleration=self.args.regrasp_approach_acceleration,
                timeout_sec=self.args.move_timeout_sec,
                verify_tolerance_mm=self.args.post_press_safe_lift_target_tolerance_mm,
            )
        if (
            not self.args.regrasp_reset_before_cup
            and (abs(self.args.regrasp_retreat_y_m) > 1e-6 or abs(self.args.regrasp_retreat_x_m) > 1e-6)
        ):
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
                label="safe robot-side X retreat away from dispenser before re-grasp transit",
                velocity=self.args.regrasp_approach_velocity,
                acceleration=self.args.regrasp_approach_acceleration,
                timeout_sec=self.args.move_timeout_sec,
                verify_tolerance_mm=self.args.safe_lift_target_tolerance_mm,
            )
        cup_place_joints = load_cup_place_joints_deg(dispenser_id)
        if cup_place_joints is not None:
            print(
                f"[Azas] cup re-grasp: dispenser={dispenser_id} using measured "
                "DISP_PLACE with generated high rear-entry; not front_hold_poses"
            )
            self.gripper_command(
                "open",
                width_m=self.args.gripper_open_width_m,
                force_n=self.args.gripper_open_force_n,
                label="RG2 open at safe robot-side retreat before measured cup re-grasp",
            )
            self.move_measured_cup_regrasp_with_rear_entry(
                cup_place_joints,
                dispenser_id=dispenser_id,
                final_regrasp=final_regrasp,
            )
            self.gripper_command(
                "set_width",
                width_m=grasp_width_m,
                force_n=grasp_force_n,
                label=grasp_label,
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
            return

        front_hold_position, _, _ = load_front_hold_pose(self.args.config, dispenser_id)
        final_offset_x_m = self.args.move_release_offset_x_m
        final_offset_y_m = self.args.move_release_offset_y_m
        final_offset_z_m = self.args.move_release_offset_z_m
        if final_regrasp:
            final_offset_x_m += self.args.final_regrasp_extra_x_offset_m
            final_offset_y_m += self.args.final_regrasp_extra_y_offset_m
            final_offset_z_m += self.args.final_regrasp_extra_z_offset_m
            print(
                "[Azas] "
                f"final front-hold re-grasp tighter offset before holder: dispenser={dispenser_id} "
                f"offset_x={final_offset_x_m:.3f}m offset_y={final_offset_y_m:.3f}m "
                f"offset_z={final_offset_z_m:.3f}m"
            )
        released_hold_z_m = front_hold_position[2] + final_offset_z_m
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
        rear_offset_x_m = final_offset_x_m + self.args.regrasp_rear_entry_offset_x_m
        rear_offset_y_m = final_offset_y_m + self.args.regrasp_rear_entry_offset_y_m
        self.move_front_hold(
            dispenser_id,
            label="re-grasp high transit above rear entry",
            offset_x_m=rear_offset_x_m,
            offset_y_m=rear_offset_y_m,
            offset_z_m=approach_offset_z_m,
            velocity=self.args.regrasp_approach_velocity,
            acceleration=self.args.regrasp_approach_acceleration,
            prefer_joint=self.args.regrasp_high_transit_joint,
        )
        self.gripper_command(
            "open",
            width_m=self.args.gripper_open_width_m,
            force_n=self.args.gripper_open_force_n,
            label="RG2 open at re-grasp high rear entry",
        )
        self.move_front_hold(
            dispenser_id,
            label="re-grasp lowered rear entry before forward approach",
            offset_x_m=rear_offset_x_m,
            offset_y_m=rear_offset_y_m,
            offset_z_m=final_offset_z_m,
            velocity=self.args.pick_approach_velocity,
            acceleration=self.args.pick_approach_acceleration,
        )
        self.move_front_hold(
            dispenser_id,
            label="final re-grasp forward approach to cup",
            offset_x_m=final_offset_x_m,
            offset_y_m=final_offset_y_m,
            offset_z_m=final_offset_z_m,
            velocity=self.args.pick_approach_velocity,
            acceleration=self.args.pick_approach_acceleration,
        )
        self.gripper_command(
            "set_width",
            width_m=grasp_width_m,
            force_n=grasp_force_n,
            label=grasp_label,
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
        current_pose = self.current_posx()
        press_drop_m = max(self.args.press_depth_m, 0.0) + max(self.args.press_extra_depth_m, 0.0)
        contact_joints = None if self.args.force_cartesian_press else load_press_ready_joints_deg(dispenser_id)
        skip_measured_press_pre = bool(self.args.skip_measured_press_pre)
        pre_joints = (
            None
            if self.args.force_cartesian_press or skip_measured_press_pre
            else load_press_pre_joints_deg(dispenser_id)
        )
        press_common_pre_joints = None if self.args.force_cartesian_press else load_press_common_pre_joints_deg(dispenser_id)

        if contact_joints is not None and (skip_measured_press_pre or pre_joints is not None):
            print(f"[Azas] skip_measured_press_pre={str(skip_measured_press_pre).lower()}")
            if not skip_measured_press_pre:
                print(f"[Azas] PRESS_PRE={format_joints_deg(pre_joints)}")
            if skip_measured_press_pre and self.args.press_reset_before_press and press_common_pre_joints is not None:
                print(f"[Azas] PRESS_COMMON_PRE={format_joints_deg(press_common_pre_joints)}")
            print(f"[Azas] PRESS_CONTACT={format_joints_deg(contact_joints)}")
            common_pre_path = ""
            if skip_measured_press_pre and self.args.press_reset_before_press:
                common_pre_path = "PRESS_COMMON_PRE -> " if press_common_pre_joints is not None else "PRESS_RESET_HOME -> "
            if skip_measured_press_pre:
                print(
                    "[Azas] measured press path: "
                    f"SAFE_LIFT -> {common_pre_path}CONTACT_ENTRY_LIFT -> PRESS_CONTACT -> "
                    f"(PRESS_Z_OVERDRIVE -> PRESS_CONTACT) x{max(int(press_count), 1)} -> "
                    "POST_PRESS_SAFE_LIFT -> REGRASP"
                )
            else:
                print(
                    "[Azas] measured press path: "
                    "SAFE_LIFT -> PRESS_PRE -> CONTACT_ENTRY_LIFT -> PRESS_CONTACT -> "
                    "PRESS_Z_OVERDRIVE -> PRESS_CONTACT -> PRESS_PRE -> REGRASP"
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
                    label="SAFE_LIFT pre-step robot-side X retreat away from dispenser before press",
                    velocity=self.args.press_travel_velocity,
                    acceleration=self.args.press_travel_acceleration,
                    timeout_sec=self.args.press_timeout_sec,
                )
                current_pose = self.current_posx(timeout_sec=self.args.wait_service_sec)
            transit_z = max(
                current_pose[2] + max(self.args.press_transit_height_m, 0.0) * 1000.0,
                max(self.args.press_min_transit_z_m, 0.0) * 1000.0,
            )
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
                label=(
                    "SAFE_LIFT away from released cup before CONTACT_ENTRY_LIFT"
                    if skip_measured_press_pre
                    else "SAFE_LIFT away from released cup before measured PRESS_PRE"
                ),
                velocity=self.args.press_travel_velocity,
                acceleration=self.args.press_travel_acceleration,
                timeout_sec=self.args.press_timeout_sec,
                verify_tolerance_mm=self.args.safe_lift_target_tolerance_mm,
            )
            self.gripper_command(
                "set_width",
                width_m=self.args.press_gripper_close_width_m,
                force_n=self.args.press_gripper_force_n,
                label="RG2 close empty gripper for dispenser press",
            )
            if skip_measured_press_pre and self.args.press_reset_before_press:
                if press_common_pre_joints is not None:
                    reset_joints = list(press_common_pre_joints)
                    reset_label = "PRESS_COMMON_PRE common contact pre joints before CONTACT_ENTRY_LIFT"
                else:
                    reset_joints = list(self.args.press_reset_joints_deg)
                    reset_label = "move to press HOME joints before CONTACT_ENTRY_LIFT"
                self.movej(
                    reset_joints,
                    label=reset_label,
                    velocity=self.args.press_reset_joint_velocity,
                    acceleration=self.args.press_reset_joint_acceleration,
                )

            if not skip_measured_press_pre:
                pre_joints = self.normalize_joints_near_current(
                    list(pre_joints),
                    label="PRESS_PRE measured joints",
                )
                self.movej(
                    pre_joints,
                    label="PRESS_PRE measured press pre-contact joints",
                    velocity=self.args.press_pre_joint_velocity,
                    acceleration=self.args.press_pre_joint_acceleration,
                )
            contact_joints = list(contact_joints)
            raw_contact_joints = list(contact_joints)
            if self.args.press_lock_contact_joint_indexes:
                if skip_measured_press_pre:
                    print(
                        "[WARN] press contact joint lock ignored because "
                        "skip_measured_press_pre=true and measured pre-contact joints are not commanded"
                    )
                else:
                    contact_joints = lock_joints_to_reference(
                        contact_joints,
                        pre_joints,
                        self.args.press_lock_contact_joint_indexes,
                    )
                    if contact_joints != raw_contact_joints:
                        print(
                            "[Azas] press contact joint lock: raw=["
                            + ", ".join(f"{value:.2f}" for value in raw_contact_joints)
                            + "] command=["
                            + ", ".join(f"{value:.2f}" for value in contact_joints)
                            + "] locked_joints="
                            + ",".join(str(index + 1) for index in sorted(self.args.press_lock_contact_joint_indexes))
                        )
            contact_joints = self.normalize_joints_near_current(
                contact_joints,
                label="PRESS_CONTACT measured joints",
            )
            # Generated entry-lift pose directly above the measured contact: every
            # contact approach must pass through it so the press always enters from
            # straight above instead of from an arbitrary PRESS_PRE direction.
            entry_lift_m = max(self.args.press_contact_entry_lift_m, 0.0)
            contact_fk_posx = self.fkin_posx(
                contact_joints,
                label=f"measured PRESS_CONTACT FK for entry lift dispenser {dispenser_id}",
            )
            contact_fk_posx = apply_dispenser_press_y_offset(
                self.args,
                dispenser_id,
                list(contact_fk_posx[:6]),
                label="PRESS_CONTACT_FK",
            )
            contact_entry_posx = list(contact_fk_posx[:6])
            contact_entry_posx[2] += entry_lift_m * 1000.0
            self.validate_cartesian_target_z(
                contact_entry_posx,
                label=f"generated PRESS_CONTACT entry lift dispenser {dispenser_id}",
            )
            print(
                "[Azas] integrated press: "
                f"dispenser={dispenser_id} count={press_count} "
                f"PRESS_CONTACT={format_joints_deg(contact_joints)} "
                f"contact_entry_z={contact_entry_posx[2] / 1000.0:.3f}m "
                f"(entry_lift={entry_lift_m * 1000.0:.1f}mm) "
                f"z_overdrive_m={press_drop_m:.3f}"
                + (
                    " source=PRESS_CONTACT_FK; measured pre-contact joints skipped; "
                    f"press_contact_use_joint_move={str(self.args.press_contact_use_joint_move).lower()}"
                    if skip_measured_press_pre
                    else f" PRESS_PRE={format_joints_deg(pre_joints)}"
                )
            )
            if skip_measured_press_pre:
                self.move_generated_press_pre(
                    contact_entry_posx,
                    label="CONTACT_ENTRY_LIFT above measured PRESS_CONTACT",
                )
                if self.args.press_contact_use_joint_move:
                    if abs(dispenser_press_y_offset_m(self.args, dispenser_id)) > 1e-9:
                        print(
                            "[WARN] press Y runtime offset is ignored for "
                            "press_contact_use_joint_move=true because measured joints are commanded directly"
                        )
                    self.movej(
                        contact_joints,
                        label="PRESS_CONTACT measured contact joints",
                        velocity=self.args.press_contact_joint_velocity,
                        acceleration=self.args.press_contact_joint_acceleration,
                    )
                else:
                    self.move_posx(
                        list(contact_fk_posx[:6]),
                        label="Z-only descend to measured PRESS_CONTACT FK",
                        velocity=self.args.press_line_velocity,
                        acceleration=self.args.press_line_acceleration,
                        timeout_sec=self.args.press_timeout_sec,
                        verify_tolerance_mm=max(self.args.target_tolerance_mm, 25.0),
                    )
                contact_posx = self.current_posx(timeout_sec=self.args.wait_service_sec)
                for press_index in range(1, max(int(press_count), 1) + 1):
                    suffix = f" {press_index}/{press_count}" if press_count > 1 else ""
                    if press_drop_m > 0.0:
                        extra_press_posx = list(contact_posx)
                        extra_press_posx[2] -= press_drop_m * 1000.0
                        self.move_posx(
                            extra_press_posx,
                            label=f"PRESS_Z_OVERDRIVE from stable measured contact{suffix}",
                            velocity=self.args.press_line_velocity,
                            acceleration=self.args.press_line_acceleration,
                            timeout_sec=self.args.press_timeout_sec,
                            verify_tolerance_mm=max(self.args.target_tolerance_mm, 25.0),
                        )
                        if self.args.press_hold_seconds > 0.0:
                            time.sleep(self.args.press_hold_seconds)
                        self.move_posx(
                            contact_posx,
                            label=f"return to stable PRESS_CONTACT after Z overdrive{suffix}",
                            velocity=self.args.press_line_velocity,
                            acceleration=self.args.press_line_acceleration,
                            timeout_sec=self.args.press_timeout_sec,
                            verify_tolerance_mm=max(self.args.target_tolerance_mm, 25.0),
                        )
                    elif self.args.press_hold_seconds > 0.0:
                        time.sleep(self.args.press_hold_seconds)
                self.move_posx(
                    contact_entry_posx,
                    label="POST_PRESS_SAFE_LIFT above measured PRESS_CONTACT",
                    velocity=self.args.press_line_velocity,
                    acceleration=self.args.press_line_acceleration,
                    timeout_sec=self.args.press_timeout_sec,
                    verify_tolerance_mm=max(self.args.target_tolerance_mm, 25.0),
                )
                if self.args.press_post_retreat_wait_seconds > 0.0:
                    time.sleep(self.args.press_post_retreat_wait_seconds)
                return

            for press_index in range(1, max(int(press_count), 1) + 1):
                suffix = f" {press_index}/{press_count}" if press_count > 1 else ""
                self.move_generated_press_pre(
                    contact_entry_posx,
                    label=f"CONTACT_ENTRY_LIFT above measured PRESS_CONTACT{suffix}",
                )
                self.movej(
                    contact_joints,
                    label=f"PRESS_CONTACT measured contact joints{suffix}",
                    velocity=self.args.press_contact_joint_velocity,
                    acceleration=self.args.press_contact_joint_acceleration,
                )
                if press_drop_m > 0.0:
                    contact_posx = self.current_posx(timeout_sec=self.args.wait_service_sec)
                    extra_press_posx = list(contact_posx)
                    extra_press_posx[2] -= press_drop_m * 1000.0
                    self.move_posx(
                        extra_press_posx,
                        label=f"PRESS_Z_OVERDRIVE from measured contact{suffix}",
                        velocity=self.args.press_line_velocity,
                        acceleration=self.args.press_line_acceleration,
                        timeout_sec=self.args.press_timeout_sec,
                        verify_tolerance_mm=max(self.args.target_tolerance_mm, 25.0),
                    )
                    if self.args.press_hold_seconds > 0.0:
                        time.sleep(self.args.press_hold_seconds)
                    self.move_posx(
                        contact_posx,
                        label=f"return to PRESS_CONTACT after Z overdrive{suffix}",
                        velocity=self.args.press_line_velocity,
                        acceleration=self.args.press_line_acceleration,
                        timeout_sec=self.args.press_timeout_sec,
                        verify_tolerance_mm=max(self.args.target_tolerance_mm, 25.0),
                    )
                elif self.args.press_hold_seconds > 0.0:
                    time.sleep(self.args.press_hold_seconds)
                if skip_measured_press_pre:
                    self.move_posx(
                        contact_entry_posx,
                        label=f"POST_PRESS_SAFE_LIFT above measured PRESS_CONTACT{suffix}",
                        velocity=self.args.press_line_velocity,
                        acceleration=self.args.press_line_acceleration,
                        timeout_sec=self.args.press_timeout_sec,
                        verify_tolerance_mm=max(self.args.target_tolerance_mm, 25.0),
                    )
                else:
                    self.movej(
                        pre_joints,
                        label=f"return to PRESS_PRE measured press pre-contact joints{suffix}",
                        velocity=self.args.press_pre_joint_velocity,
                        acceleration=self.args.press_pre_joint_acceleration,
                    )
            if self.args.press_post_retreat_wait_seconds > 0.0:
                time.sleep(self.args.press_post_retreat_wait_seconds)
            return

        press_xyz_m, press_rpy_deg = load_press_pose(dispenser_id)
        joint_space_press = contact_joints is not None
        if contact_joints is None:
            press_posx = apply_dispenser_press_y_offset(
                self.args,
                dispenser_id,
                [
                    press_xyz_m[0] * 1000.0,
                    press_xyz_m[1] * 1000.0,
                    press_xyz_m[2] * 1000.0,
                    press_rpy_deg[0],
                    press_rpy_deg[1],
                    press_rpy_deg[2],
                ],
                label="press_pose_xyz_m fallback",
            )
            x_mm, y_mm, contact_z, rx, ry, rz = press_posx[:6]
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
                    f"[Azas] dispenser {dispenser_id}: loading measured press contact joints "
                    f"(joint_6/link_6={contact_joints[5]:.2f}deg)"
                )
            contact_fk_posx = self.fkin_posx(
                contact_joints,
                label=f"measured PRESS_CONTACT FK dispenser {dispenser_id}",
            )
            contact_fk_posx = apply_dispenser_press_y_offset(
                self.args,
                dispenser_id,
                list(contact_fk_posx[:6]),
                label="fallback PRESS_CONTACT_FK",
            )
            x_mm, y_mm, contact_z, rx, ry, rz = contact_fk_posx[:6]
        if joint_space_press:
            # Fallback for older calibration where PRESS_CONTACT is the only
            # taught press pose. PRE and the pump stroke are generated by
            # changing only Z from the FK contact pose.
            generated_pre_lift_m = max(self.args.press_contact_entry_lift_m, 0.0)
            pre_z = contact_z + generated_pre_lift_m * 1000.0
            transit_z = max(
                current_pose[2] + max(self.args.press_transit_height_m, generated_pre_lift_m, 0.0) * 1000.0,
                min(pre_z, max(self.args.press_min_transit_z_m, 0.0) * 1000.0),
            )
            pressed_z = contact_z - press_drop_m * 1000.0
            print(
                "[Azas] integrated press: "
                f"dispenser={dispenser_id} count={press_count} "
                f"contact_fk=({x_mm:.1f}, {y_mm:.1f}, {contact_z:.1f}) "
                f"generated_pre_z={pre_z:.1f} pressed_z={pressed_z:.1f} "
                f"z_descent={contact_z - pressed_z:.1f}mm transit_z={transit_z:.1f} "
                "source=measured PRESS_CONTACT only; PRE/PRESS are Z-only"
            )
        else:
            pre_z = contact_z + max(self.args.press_pre_lift_m, 0.0) * 1000.0
            pressed_z = contact_z - press_drop_m * 1000.0
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
                label="safe robot-side X retreat away from dispenser before press lift",
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
            verify_tolerance_mm=self.args.safe_lift_target_tolerance_mm,
        )
        self.gripper_command(
            "set_width",
            width_m=self.args.press_gripper_close_width_m,
            force_n=self.args.press_gripper_force_n,
            label="RG2 close empty gripper for dispenser press",
        )
        if self.args.press_reset_before_press:
            reset_joints = list(self.args.press_reset_joints_deg)
            self.movej(
                reset_joints,
                label="move to press HOME joints before fallback generated PRESS_PRE",
                velocity=self.args.press_reset_joint_velocity,
                acceleration=self.args.press_reset_joint_acceleration,
            )

        if joint_space_press and pre_joints is None:
            contact_posx = [x_mm, y_mm, contact_z, rx, ry, rz]
            generated_pre_posx = list(contact_posx)
            generated_pre_posx[2] = pre_z
            print(
                "[Azas] contact-only press path: "
                f"dispenser={dispenser_id} contact_posx=[{x_mm:.1f}, {y_mm:.1f}, {contact_z:.1f}, "
                f"{rx:.1f}, {ry:.1f}, {rz:.1f}] "
                f"generated_pre_z={pre_z:.1f} press_drop_mm={press_drop_m * 1000.0:.1f}"
            )
            self.move_generated_press_pre(
                generated_pre_posx,
                label="generated PRESS_PRE above measured contact",
            )
            for press_index in range(1, max(int(press_count), 1) + 1):
                suffix = f" {press_index}/{press_count}" if press_count > 1 else ""
                self.move_posx(
                    contact_posx,
                    label=f"Z-only descend to measured PRESS_CONTACT{suffix}",
                    velocity=self.args.press_line_velocity,
                    acceleration=self.args.press_line_acceleration,
                    timeout_sec=self.args.press_timeout_sec,
                )
                if press_drop_m > 0.0:
                    pressed_posx = list(contact_posx)
                    pressed_posx[2] -= press_drop_m * 1000.0
                    self.move_posx(
                        pressed_posx,
                        label=f"Z-only press from measured contact{suffix}",
                        velocity=self.args.press_line_velocity,
                        acceleration=self.args.press_line_acceleration,
                        timeout_sec=self.args.press_timeout_sec,
                        verify_tolerance_mm=max(self.args.target_tolerance_mm, 25.0),
                    )
                    if self.args.press_hold_seconds > 0.0:
                        time.sleep(self.args.press_hold_seconds)
                    self.move_posx(
                        contact_posx,
                        label=f"return to measured PRESS_CONTACT{suffix}",
                        velocity=self.args.press_line_velocity,
                        acceleration=self.args.press_line_acceleration,
                        timeout_sec=self.args.press_timeout_sec,
                        verify_tolerance_mm=max(self.args.target_tolerance_mm, 25.0),
                    )
                elif self.args.press_hold_seconds > 0.0:
                    time.sleep(self.args.press_hold_seconds)
                self.move_posx(
                    generated_pre_posx,
                    label=f"Z-only retreat to generated PRESS_PRE{suffix}",
                    velocity=self.args.press_line_velocity,
                    acceleration=self.args.press_line_acceleration,
                    timeout_sec=self.args.press_timeout_sec,
                )
            if self.args.press_post_retreat_wait_seconds > 0.0:
                time.sleep(self.args.press_post_retreat_wait_seconds)
            return

        steps: list[tuple[list[float], str, float, float]] = []
        if joint_space_press:
            if self.args.press_move_configured_prepose_before_joint:
                self.move_posx(
                    [x_mm, y_mm, pre_z, rx, ry, rz],
                    label="fallback high pre pose before measured PRESS_CONTACT",
                    velocity=self.args.press_travel_velocity,
                    acceleration=self.args.press_travel_acceleration,
                    timeout_sec=self.args.press_timeout_sec,
                )
            else:
                print("[Azas] joint-space press fallback: no PRESS_PRE joints; using generated CONTACT+Z entry")
            if self.args.press_contact_entry_lift_m > 0.0:
                entry_z = contact_z + max(self.args.press_contact_entry_lift_m, 0.0) * 1000.0
                self.move_posx(
                    [x_mm, y_mm, entry_z, rx, ry, rz],
                    label="fallback safe high waypoint above measured PRESS_CONTACT",
                    velocity=self.args.press_travel_velocity,
                    acceleration=self.args.press_travel_acceleration,
                    timeout_sec=self.args.press_timeout_sec,
                )
            contact_joints = self.normalize_joints_near_current(
                contact_joints,
                label="fallback PRESS_CONTACT measured joints",
            )
            self.movej(
                contact_joints,
                label="fallback PRESS_CONTACT measured contact joints",
                velocity=self.args.press_contact_joint_velocity,
                acceleration=self.args.press_contact_joint_acceleration,
            )
            contact_posx = self.current_posx(timeout_sec=self.args.wait_service_sec)
            x_mm, y_mm, contact_z, rx, ry, rz = contact_posx
            pre_z = contact_z
            pressed_z = contact_z - press_drop_m * 1000.0
            print(
                "[Azas] integrated press fallback: "
                f"dispenser={dispenser_id} count={press_count} "
                f"contact=({x_mm:.1f}, {y_mm:.1f}, {contact_z:.1f}) "
                f"pressed_z={pressed_z:.1f} z_descent={contact_z - pressed_z:.1f}mm "
                "source=live TCP after CONTACT-only fallback"
            )
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
            if press_drop_m == 0.0:
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
                            f"move to Z-only press start{suffix}",
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
        # Legacy helper path only receives the taught contact pose here.  The
        # integrated path above is the normal hardware path for extra Z strokes.
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
        "-p pre_home_retreat_velocity:=80.0 "
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
        "-p line_velocity:=80.0 "
        "-p line_acceleration:=30.0 "
        "-p travel_line_velocity:=80.0 "
        "-p travel_line_acceleration:=70.0 "
        "-p joint_velocity:=80.0 "
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
    parser.add_argument(
        "--allow-tcp-set-failure",
        action="store_true",
        help=(
            "Dangerous debug option: continue even if Doosan tcp/set_current_tcp fails. "
            "Default false because measured press poses require the configured dispenser TCP."
        ),
    )
    parser.add_argument(
        "--restore-tcp-after-run",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Restore the previous Doosan TCP when the sequence exits. Default false so "
            "the measured gripper TCP stays active for follow-up recipe motions."
        ),
    )
    parser.add_argument("--move-velocity", type=float, default=80.0)
    parser.add_argument("--move-acceleration", type=float, default=25.0)
    parser.add_argument("--move-prehold-offset-x-m", type=float, default=-0.030)
    parser.add_argument(
        "--cup-pre-from-place-x-offset-m",
        type=float,
        default=-0.090,
        help=(
            "Generate DISP_PRE from the latest measured cup_place pose by changing only X. "
            "Default -0.090m."
        ),
    )
    parser.add_argument(
        "--cup-pre-from-place-z-offset-m",
        type=float,
        default=0.030,
        help=(
            "Generate DISP_PRE from the latest measured cup_place pose by changing only Z. "
            "Default +0.030m."
        ),
    )
    parser.add_argument(
        "--dispenser-3-cup-pre-extra-x-offset-m",
        type=float,
        default=-0.010,
        help=(
            "Extra X offset applied only to generated DISP3_PRE. Default -0.010m "
            "adds 10mm robot-side clearance without changing measured DISP3_PLACE."
        ),
    )
    parser.add_argument(
        "--use-cup-common-pre",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Use calibration.yaml cup_common_pre_joints_deg before DISP_PLACE. "
            "Default false: generate cup pre from cup_place with X/Z offsets."
        ),
    )
    parser.add_argument(
        "--generated-cup-pre-use-joint-fallback",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Use IK MoveJoint for generated DISP_PRE. Default false: generated DISP_PRE "
            "uses Cartesian MoveLine so X/Z-offset cup pre does not jump to a different wrist/joint branch."
        ),
    )
    parser.add_argument(
        "--generated-cup-pre-max-joint-delta-deg",
        type=float,
        default=190.0,
        help=(
            "Joint delta guard used only when --generated-cup-pre-use-joint-fallback is enabled. "
            "Default 190deg preserves the old fallback limit without affecting the default Cartesian path."
        ),
    )
    parser.add_argument(
        "--move-prehold-offset-y-m",
        type=float,
        default=0.0,
        help=(
            "Y offset from measured front_hold for the pre-hold/above-hold approach. "
            "The default is 0.0 because Y is the left/right slot axis, not the robot-side safety retreat."
        ),
    )
    parser.add_argument(
        "--move-prehold-offset-z-m",
        type=float,
        default=0.180,
        help="Vertical approach offset for initial cup placement at dispenser front-hold.",
    )
    parser.add_argument(
        "--move-release-offset-x-m",
        type=float,
        default=-0.020,
        help=(
            "Final cup release X offset from measured front_hold. Negative backs the cup "
            "away from the dispenser toward the robot; default -0.020m keeps all cup-place "
            "positions 20mm behind the taught front-hold."
        ),
    )
    parser.add_argument(
        "--move-release-offset-y-m",
        type=float,
        default=0.0,
        help="Final cup release Y offset from measured front_hold; default 0 avoids shifting dispenser 4 farther right.",
    )
    parser.add_argument(
        "--move-release-offset-z-m",
        type=float,
        default=0.0,
        help="Final cup release Z offset from measured front_hold. Default 0 uses the taught cup-place height exactly.",
    )
    parser.add_argument("--move-prehold-velocity", type=float, default=80.0)
    parser.add_argument("--move-prehold-acceleration", type=float, default=22.0)
    parser.add_argument("--move-timeout-sec", type=float, default=180.0)
    parser.add_argument("--pick-approach-velocity", type=float, default=80.0)
    parser.add_argument("--pick-approach-acceleration", type=float, default=14.0)
    parser.add_argument("--pick-pregrasp-offset-x-m", type=float, default=0.0)
    parser.add_argument("--pick-pregrasp-offset-y-m", type=float, default=0.0)
    parser.add_argument("--pick-pregrasp-offset-z-m", type=float, default=0.0)
    parser.add_argument("--pick-pregrasp-staging-velocity", type=float, default=80.0)
    parser.add_argument("--pick-pregrasp-staging-acceleration", type=float, default=16.0)
    parser.add_argument("--pick-lift-m", type=float, default=0.100)
    parser.add_argument("--pick-lift-velocity", type=float, default=80.0)
    parser.add_argument("--pick-lift-acceleration", type=float, default=25.0)
    parser.add_argument("--pick-timeout-sec", type=float, default=120.0)
    parser.add_argument(
        "--regrasp-min-transit-z-m",
        type=float,
        default=0.500,
        help="Minimum absolute TCP Z for the vertical lift immediately after pressing, before returning to the cup.",
    )
    parser.add_argument(
        "--post-press-safe-lift-z-m",
        type=float,
        default=0.470,
        help=(
            "Initial vertical TCP Z after dispenser pressing before robot-side X retreat. "
            "The following re-grasp high transit still uses --regrasp-min-transit-z-m."
        ),
    )
    parser.add_argument(
        "--start-safe-lift-z-m",
        type=float,
        default=0.15,
        help=(
            "Minimum absolute TCP Z for the start-of-sequence vertical lift. The sequence "
            "reads the live TCP pose first and raises only Z to this height before any "
            "XY waypoint motion."
        ),
    )
    parser.add_argument(
        "--min-allowed-tcp-z-m",
        type=float,
        default=0.02,
        help=(
            "Reject any Cartesian target pose whose absolute TCP Z is below this height. "
            "A failed check prints an error and aborts before any robot command is sent."
        ),
    )
    parser.add_argument(
        "--force-start-safe-lift",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Always run the start-of-sequence Z-only safe lift regardless of the current "
            "TCP height. With --no-force-start-safe-lift the lift still runs whenever the "
            "live TCP Z is below --start-safe-lift-z-m."
        ),
    )
    parser.add_argument(
        "--skip-release-pre",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Use the posture-independent safe release structure instead of the generated "
            "DISP_PRE-from-DISP_PLACE release waypoint. Default false: cup pre is generated "
            "from the latest cup_place pose by X offset."
        ),
    )
    parser.add_argument(
        "--release-approach-lift-m",
        type=float,
        default=0.100,
        help="Height of the generated release_above pose over the release final pose (same X/Y/orientation).",
    )
    parser.add_argument(
        "--release-start-safe-lift-m",
        type=float,
        default=0.120,
        help="Relative Z-only lift from the live TCP pose at the start of the safe release.",
    )
    parser.add_argument(
        "--release-min-transit-z-m",
        type=float,
        default=0.300,
        help=(
            "Minimum absolute TCP Z for every XY-transit pose of the safe release "
            "(start lift, staging, high entry). The vertical descent chain above->final "
            "is exempt but still checked against --min-allowed-tcp-z-m."
        ),
    )
    parser.add_argument(
        "--post-release-safe-lift-m",
        type=float,
        default=0.100,
        help="Vertical lift above the release final pose right after the gripper opens.",
    )
    parser.add_argument(
        "--release-staging-x-m",
        type=float,
        default=None,
        help="Optional absolute base-frame X of the release staging pose. Default: release final X.",
    )
    parser.add_argument(
        "--release-staging-y-m",
        type=float,
        default=None,
        help="Optional absolute base-frame Y of the release staging pose. Default: release final Y.",
    )
    parser.add_argument(
        "--release-staging-z-m",
        type=float,
        default=None,
        help="Optional absolute base-frame Z of the release staging pose. Default: release transit height.",
    )
    parser.add_argument(
        "--use-release-staging",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Route the safe release through the staging pose before the above-final entry.",
    )
    parser.add_argument(
        "--regrasp-approach-offset-z-m",
        type=float,
        default=0.250,
        help=(
            "High front-hold Z offset used for the post-press re-grasp transit after the gripper "
            "has opened at the lifted robot-side retreat pose."
        ),
    )
    parser.add_argument(
        "--regrasp-max-transit-z-m",
        type=float,
        default=0.560,
        help="Maximum absolute TCP/front-hold high approach Z used for post-press re-grasp transit.",
    )
    parser.add_argument("--regrasp-approach-velocity", type=float, default=80.0)
    parser.add_argument("--regrasp-approach-acceleration", type=float, default=18.0)
    parser.add_argument(
        "--regrasp-reset-before-cup",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "After pressing, move through a fixed HOME joint waypoint before returning to the cup. "
            "Default true because the Cartesian post-press high-Z lift can stall near the dispenser."
        ),
    )
    parser.add_argument(
        "--regrasp-reset-joints-deg",
        default="0,0,90,0,90,180",
        help="HOME joint pose used after dispenser press, before returning to cup re-grasp.",
    )
    parser.add_argument("--regrasp-reset-joint-velocity", type=float, default=80.0)
    parser.add_argument("--regrasp-reset-joint-acceleration", type=float, default=35.0)
    parser.add_argument(
        "--regrasp-retreat-x-m",
        type=float,
        default=-0.080,
        help=(
            "Optional high-Z X retreat immediately after press before returning to cup. "
            "In the measured dispenser setup, negative X backs away from the dispenser toward the robot."
        ),
    )
    parser.add_argument(
        "--regrasp-retreat-y-m",
        type=float,
        default=0.0,
        help="Optional high-Z Y shift immediately after press before returning to cup; not used as the default safety retreat.",
    )
    parser.add_argument(
        "--regrasp-rear-entry-offset-x-m",
        type=float,
        default=-0.090,
        help=(
            "Extra X offset from final release pose for the lowered rear entry before re-grasp. "
            "Negative X approaches from the robot side instead of sliding sideways along dispenser slots."
        ),
    )
    parser.add_argument(
        "--regrasp-rear-entry-offset-y-m",
        type=float,
        default=0.0,
        help="Extra Y offset from final release pose for the lowered rear entry before moving forward to the cup.",
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
        default=0.070,
        help=(
            "Z-only pump stroke after the measured PRESS_CONTACT joints touch the dispenser. "
            "Default is 0.070 m (7 cm)."
        ),
    )
    parser.add_argument(
        "--press-extra-depth-m",
        type=float,
        default=0.0,
        help=(
            "Additional Z-only pump stroke added after PRESS_CONTACT. "
            "Total stroke is --press-depth-m + this value. Default 0."
        ),
    )
    parser.add_argument(
        "--press-pre-lift-m",
        type=float,
        default=0.080,
        help="Cartesian fallback lift above the measured dispenser contact pose.",
    )
    parser.add_argument("--press-approach-height-m", type=float, default=0.100)
    parser.add_argument("--press-transit-height-m", type=float, default=0.080)
    parser.add_argument(
        "--press-pre-lift-retreat-x-m",
        type=float,
        default=-0.050,
        help="X retreat after cup release and before the vertical press lift; negative backs toward the robot.",
    )
    parser.add_argument(
        "--press-pre-lift-retreat-y-m",
        type=float,
        default=0.0,
        help="Y shift after cup release and before the vertical press lift; not used as the default dispenser retreat.",
    )
    parser.add_argument(
        "--press-min-transit-z-m",
        type=float,
        default=0.500,
        help="Minimum absolute TCP Z before moving from cup release toward dispenser press joints.",
    )
    parser.add_argument("--press-line-velocity", type=float, default=35.0)
    parser.add_argument("--press-line-acceleration", type=float, default=30.0)
    parser.add_argument("--press-travel-velocity", type=float, default=60.0)
    parser.add_argument("--press-travel-acceleration", type=float, default=50.0)
    parser.add_argument("--press-timeout-sec", type=float, default=120.0)
    parser.add_argument("--press-hold-seconds", type=float, default=0.25)
    parser.add_argument("--press-gripper-close-width-m", type=float, default=0.0)
    parser.add_argument("--press-gripper-force-n", type=float, default=30.0)
    parser.add_argument(
        "--press-reset-before-press",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "After cup release, safe lift, and empty-gripper close, move through "
            "calibration.yaml press_common_pre_joints_deg before CONTACT_ENTRY_LIFT. "
            "If that key is missing, falls back to --press-reset-joints-deg. "
            "Default false because the measured common pre/HOME joint waypoint can choose a large wrist branch."
        ),
    )
    parser.add_argument(
        "--press-reset-joints-deg",
        default="0,0,90,0,90,0",
        help="Fallback HOME joint pose used only when press_common_pre_joints_deg is missing.",
    )
    parser.add_argument("--press-reset-joint-velocity", type=float, default=80.0)
    parser.add_argument("--press-reset-joint-acceleration", type=float, default=25.0)
    parser.add_argument("--press-pre-joint-velocity", type=float, default=80.0)
    parser.add_argument("--press-pre-joint-acceleration", type=float, default=25.0)
    parser.add_argument("--press-contact-joint-velocity", type=float, default=50.0)
    parser.add_argument("--press-contact-joint-acceleration", type=float, default=40.0)
    parser.add_argument(
        "--press-contact-use-joint-move",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Use measured PRESS_CONTACT movej after CONTACT_ENTRY_LIFT. Default false: "
            "PRESS_CONTACT joints are used only for FK, then the robot descends Z-only by Cartesian MoveLine."
        ),
    )
    parser.add_argument(
        "--press-contact-entry-lift-m",
        type=float,
        default=0.050,
        help=(
            "Generated Cartesian CONTACT_ENTRY_LIFT height above measured PRESS_CONTACT FK. "
            "Default stays in the 50-80mm hardware-safe range."
        ),
    )
    parser.add_argument(
        "--dispenser-1-press-y-offset-m",
        type=float,
        default=0.002,
        help=(
            "Runtime Y offset applied only to dispenser 1 press Cartesian targets. "
            "Default +0.002m; calibration.yaml measured values are not modified."
        ),
    )
    parser.add_argument(
        "--skip-measured-press-pre",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Default true: ignore calibration.yaml press_pre_joints_deg and approach/retreat "
            "through PRESS_CONTACT FK with Z raised by --press-contact-entry-lift-m. "
            "Use --no-skip-measured-press-pre to restore the old measured PRESS_PRE joint path."
        ),
    )
    parser.add_argument(
        "--press-joint-space-use-high-prepose",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Fallback-only option for old contact-joint mode. Default false; measured "
            "PRESS_PRE/PRESS_CONTACT pairs do not need this Cartesian pre_z."
        ),
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
        "--force-cartesian-press",
        action="store_true",
        help=(
            "Ignore dispenser press_contact_joints_deg and press using measured "
            "press_pose_xyz_m/press_pose_rpy_deg Cartesian poses."
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
        "--safe-lift-target-tolerance-mm",
        type=float,
        default=30.0,
        help=(
            "XYZ tolerance for high safe-lift verification. The Doosan controller can stop "
            "roughly 20mm from the requested high Z while still clearing the dispenser/cup; "
            "keep front-hold/press targets on --target-tolerance-mm."
        ),
    )
    parser.add_argument(
        "--post-press-safe-lift-target-tolerance-mm",
        type=float,
        default=60.0,
        help=(
            "XYZ tolerance only for the vertical lift immediately after dispenser pressing. "
            "This avoids stalling on the lifted clearance pose while keeping other safe-lift "
            "and re-grasp checks at --safe-lift-target-tolerance-mm."
        ),
    )
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
        "--normalize-measured-joints-near-current",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Compatibility mode for old measured-joint execution. Default false: "
            "operator-measured DISP/PRESS joints are commanded exactly, without "
            "rewriting them to a nearby 360deg-equivalent branch."
        ),
    )
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
    parser.add_argument("--front-hold-joint-fallback-velocity", type=float, default=80.0)
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
    parser.add_argument("--safe-lift-joint-fallback-velocity", type=float, default=80.0)
    parser.add_argument("--safe-lift-joint-fallback-acceleration", type=float, default=40.0)
    parser.add_argument(
        "--press-generated-pre-use-joint",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Approach the generated CONTACT_ENTRY_LIFT/PRESS_PRE pose with IK MoveJoint. "
            "Default false: press entry uses Cartesian MoveLine to avoid large wrist/joint branch changes."
        ),
    )
    parser.add_argument(
        "--press-generated-pre-joint-fallback",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "When MoveLine to generated PRESS_PRE returns complete but target verification stalls, "
            "retry that high approach pose with IK MoveJoint. Default false because press entry should not "
            "fall back to a large joint branch unless explicitly requested."
        ),
    )
    parser.add_argument("--gripper-service", default="/jarvis/rg2/set_width")
    parser.add_argument("--gripper-open-width-m", type=float, default=0.110)
    parser.add_argument("--gripper-open-force-n", type=float, default=12.0)
    parser.add_argument("--gripper-grasp-width-m", type=float, default=0.075)
    parser.add_argument("--gripper-force-n", type=float, default=25.0)
    parser.add_argument("--gripper-timeout-sec", type=float, default=12.0)
    parser.add_argument(
        "--final-regrasp-extra-x-offset-m",
        type=float,
        default=0.020,
        help=(
            "Only for the final re-grasp before cup-holder placement: add this X offset "
            "to the cup re-grasp target. Positive X moves closer toward the dispenser/cup."
        ),
    )
    parser.add_argument("--final-regrasp-extra-y-offset-m", type=float, default=0.0)
    parser.add_argument("--final-regrasp-extra-z-offset-m", type=float, default=0.0)
    parser.add_argument("--final-regrasp-grasp-width-m", type=float, default=0.068)
    parser.add_argument("--final-regrasp-force-n", type=float, default=35.0)
    parser.add_argument(
        "--place-cup-holder-after-sequence",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="After the final dispenser re-grasp, place the held cup into calibration.yaml cup_holder.side_grip_place.",
    )
    parser.add_argument("--cup-holder-place-final-z-offset-m", type=float, default=-0.040)
    parser.add_argument("--cup-holder-place-final-x-offset-m", type=float, default=0.015)
    parser.add_argument(
        "--cup-holder-place-final-dispenser-1-x-extra-offset-m",
        type=float,
        default=-0.010,
        help=(
            "When the final re-grasp is from physical dispenser 1, add this extra X "
            "offset to CUP_HOLDER_PLACE_FINAL without editing calibration.yaml."
        ),
    )
    parser.add_argument(
        "--cup-holder-place-final-dispenser-1-y-extra-offset-m",
        type=float,
        default=0.010,
        help=(
            "When the final re-grasp is from physical dispenser 1, add this extra Y "
            "offset to CUP_HOLDER_PLACE_FINAL without editing calibration.yaml."
        ),
    )
    parser.add_argument(
        "--cup-holder-place-final-dispenser-2-x-extra-offset-m",
        type=float,
        default=-0.003,
        help=(
            "When the final re-grasp is from physical dispenser 2, add this extra X "
            "offset to CUP_HOLDER_PLACE_FINAL without editing calibration.yaml."
        ),
    )
    parser.add_argument(
        "--cup-holder-place-final-dispenser-2-y-extra-offset-m",
        type=float,
        default=0.010,
        help=(
            "When the final re-grasp is from physical dispenser 2, add this extra Y "
            "offset to CUP_HOLDER_PLACE_FINAL without editing calibration.yaml."
        ),
    )
    parser.add_argument(
        "--cup-holder-place-final-dispenser-3-x-extra-offset-m",
        type=float,
        default=-0.010,
        help=(
            "When the final re-grasp is from physical dispenser 3, add this extra X "
            "offset to CUP_HOLDER_PLACE_FINAL without editing calibration.yaml."
        ),
    )
    parser.add_argument(
        "--cup-holder-place-final-dispenser-3-y-extra-offset-m",
        type=float,
        default=0.010,
        help=(
            "When the final re-grasp is from physical dispenser 3, add this extra Y "
            "offset to CUP_HOLDER_PLACE_FINAL without editing calibration.yaml."
        ),
    )
    parser.add_argument(
        "--cup-holder-place-final-dispenser-4-x-extra-offset-m",
        type=float,
        default=-0.010,
        help=(
            "When the final re-grasp is from physical dispenser 4, add this extra X "
            "offset to CUP_HOLDER_PLACE_FINAL without editing calibration.yaml."
        ),
    )
    parser.add_argument(
        "--cup-holder-place-final-dispenser-4-y-extra-offset-m",
        type=float,
        default=0.010,
        help=(
            "When the final re-grasp is from physical dispenser 4, add this extra Y "
            "offset to CUP_HOLDER_PLACE_FINAL without editing calibration.yaml."
        ),
    )
    parser.add_argument("--cup-holder-place-final-y-offset-m", type=float, default=-0.010)
    parser.add_argument(
        "--cup-holder-rz-offset-deg",
        type=float,
        default=0.0,
        help="Add this RZ offset to all measured cup-holder side-grip poses without editing calibration.yaml.",
    )
    parser.add_argument("--cup-holder-approach-velocity", type=float, default=80.0)
    parser.add_argument("--cup-holder-approach-acceleration", type=float, default=20.0)
    parser.add_argument("--cup-holder-place-velocity", type=float, default=80.0)
    parser.add_argument("--cup-holder-place-acceleration", type=float, default=10.0)
    parser.add_argument("--cup-holder-retreat-velocity", type=float, default=80.0)
    parser.add_argument("--cup-holder-retreat-acceleration", type=float, default=16.0)
    parser.add_argument("--cup-holder-timeout-sec", type=float, default=90.0)
    parser.add_argument("--cup-holder-target-tolerance-mm", type=float, default=12.0)
    parser.add_argument("--cup-holder-x-min-m", type=float, default=0.35)
    parser.add_argument("--cup-holder-x-max-m", type=float, default=0.50)
    parser.add_argument("--cup-holder-y-min-m", type=float, default=0.15)
    parser.add_argument("--cup-holder-y-max-m", type=float, default=0.30)
    parser.add_argument("--cup-holder-z-min-m", type=float, default=0.06)
    parser.add_argument("--cup-holder-z-max-m", type=float, default=0.28)
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
    parser.add_argument(
        "--press-lock-contact-joints",
        default="",
        help=(
            "Comma-separated joint numbers copied from press_pre_joints_deg into "
            "press_contact_joints_deg before MoveJoint. Default empty: command measured PRESS_CONTACT exactly."
        ),
    )
    parser.add_argument(
        "--press-use-recorded-pre-joints",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Deprecated compatibility flag accepted for old commands. Ignored by the integrated path: "
            "use --no-skip-measured-press-pre when the old measured PRESS_PRE joint path is explicitly needed."
        ),
    )
    parser.add_argument(
        "--press-contact-joint-pose-tolerance-mm",
        type=float,
        default=35.0,
        help=(
            "Compare measured press_contact_joints_deg FK against press_pose_xyz_m and warn "
            "when the distance exceeds this tolerance. Set <=0 to disable the check."
        ),
    )
    parser.add_argument(
        "--strict-press-contact-joint-pose-match",
        action="store_true",
        help=(
            "Fail if press_contact_joints_deg FK and press_pose_xyz_m differ beyond "
            "--press-contact-joint-pose-tolerance-mm. Default false because this controller's "
            "Fkin can report a different TCP basis than the taught press pose."
        ),
    )
    parser.add_argument(
        "--allow-press-contact-joint-pose-mismatch",
        action="store_true",
        help="Deprecated compatibility flag for overriding strict FK/press_pose mismatch failures.",
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
    parser.add_argument(
        "--resume",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Resume from --resume-state-file when it contains the same unfinished recipe. "
            "Default false so a new order cannot silently skip move/release from stale state."
        ),
    )
    parser.add_argument(
        "--resume-state-file",
        type=Path,
        default=DEFAULT_RESUME_STATE,
        help="Durable JSON checkpoint used to remember the next recipe stage after unexpected stops.",
    )
    parser.add_argument(
        "--clear-resume-state",
        action="store_true",
        help="Delete the existing resume checkpoint before starting this run.",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} when --execute is used")
    args = parser.parse_args()
    args.press_reset_joints_deg = parse_float_list(
        args.press_reset_joints_deg,
        expected_count=6,
        label="--press-reset-joints-deg",
    )
    args.regrasp_reset_joints_deg = parse_float_list(
        args.regrasp_reset_joints_deg,
        expected_count=6,
        label="--regrasp-reset-joints-deg",
    )
    args.press_lock_contact_joint_indexes = parse_joint_index_set(
        args.press_lock_contact_joints,
        label="--press-lock-contact-joints",
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
    final_dispenser_id = grouped_dispenser_ids[-1][0] if grouped_dispenser_ids else ""
    final_holder_offsets = {
        "1": (
            args.cup_holder_place_final_dispenser_1_x_extra_offset_m,
            args.cup_holder_place_final_dispenser_1_y_extra_offset_m,
        ),
        "2": (
            args.cup_holder_place_final_dispenser_2_x_extra_offset_m,
            args.cup_holder_place_final_dispenser_2_y_extra_offset_m,
        ),
        "3": (
            args.cup_holder_place_final_dispenser_3_x_extra_offset_m,
            args.cup_holder_place_final_dispenser_3_y_extra_offset_m,
        ),
        "4": (
            args.cup_holder_place_final_dispenser_4_x_extra_offset_m,
            args.cup_holder_place_final_dispenser_4_y_extra_offset_m,
        ),
    }
    extra_x_m, extra_y_m = final_holder_offsets.get(final_dispenser_id, (0.0, 0.0))
    if abs(extra_x_m) > 1e-9 or abs(extra_y_m) > 1e-9:
        args.cup_holder_place_final_x_offset_m += extra_x_m
        args.cup_holder_place_final_y_offset_m += extra_y_m
        print(
            f"[Azas] dispenser {final_dispenser_id} final cup-holder extra offset applied: "
            f"x={extra_x_m:+.3f} m, y={extra_y_m:+.3f} m, "
            f"effective_place_final_x_offset_m={args.cup_holder_place_final_x_offset_m:.3f}, "
            f"effective_place_final_y_offset_m={args.cup_holder_place_final_y_offset_m:.3f}"
        )
    print(
        "[Azas] grouped_press_counts="
        + ",".join(f"{dispenser_id}x{count}" for dispenser_id, count in grouped_dispenser_ids)
    )
    print(f"[Azas] service_prefix={args.service_prefix}")
    print(f"[Azas] dispenser_tcp_name={args.dispenser_tcp_name}")
    print(f"[Azas] press_lock_contact_joints={args.press_lock_contact_joints or '-'}")
    print(
        f"[Azas] measured_joint_press_z_overdrive_m={max(args.press_depth_m, 0.0) + max(args.press_extra_depth_m, 0.0):.3f} "
        f"(press_depth_m={args.press_depth_m:.3f}, extra={args.press_extra_depth_m:.3f})"
    )
    print(f"[Azas] skip_measured_press_pre={str(args.skip_measured_press_pre).lower()}")
    if args.skip_measured_press_pre:
        print(
            "[Azas] source=calibration.yaml generated DISP_PRE from DISP_PLACE X/Z offset, measured DISP_PLACE, "
            "and PRESS_CONTACT FK teaching; press_pre_joints_deg ignored by default; "
            "PRESS_COMMON_PRE is used only with --press-reset-before-press"
        )
    else:
        print("[Azas] source=calibration.yaml generated DISP_PRE from DISP_PLACE X/Z offset, measured DISP_PLACE and PRESS_PRE/PRESS_CONTACT joint teaching")
    release_path = (
        "SAFE_RELEASE(Z_LIFT -> STAGING/ABOVE -> vertical RELEASE_FINAL -> OPEN -> POST_LIFT)"
        if args.skip_release_pre
        else (
            "CUP_COMMON_PRE -> DISP_PLACE -> RELEASE"
            if args.use_cup_common_pre
            else "GENERATED_DISP_PRE(X/Z offset) -> DISP_PLACE -> RELEASE"
        )
    )
    press_common_pre_path = "PRESS_COMMON_PRE -> " if args.press_reset_before_press else ""
    press_path = (
        f"SAFE_LIFT -> {press_common_pre_path}CONTACT_ENTRY_LIFT -> PRESS_CONTACT -> PRESS_Z_OVERDRIVE -> "
        "PRESS_CONTACT -> POST_PRESS_SAFE_LIFT -> REGRASP"
        if args.skip_measured_press_pre
        else (
            "SAFE_LIFT -> PRESS_PRE -> CONTACT_ENTRY_LIFT -> PRESS_CONTACT -> "
            "PRESS_Z_OVERDRIVE -> PRESS_CONTACT -> PRESS_PRE -> REGRASP"
        )
    )
    print(f"[Azas] default_path=START_SAFE_LIFT -> {release_path} -> {press_path}")
    print(
        f"[Azas] start_safety: force_start_safe_lift={args.force_start_safe_lift} "
        f"start_safe_lift_z_m={max(args.start_safe_lift_z_m, 0.0):.3f} "
        f"min_allowed_tcp_z_m={max(args.min_allowed_tcp_z_m, 0.0):.3f}"
    )
    print(
        f"[Azas] release_safety: skip_release_pre={args.skip_release_pre} "
        f"use_release_staging={args.use_release_staging} "
        f"release_start_safe_lift_m={max(args.release_start_safe_lift_m, 0.0):.3f} "
        f"release_min_transit_z_m={max(args.release_min_transit_z_m, 0.0):.3f} "
        f"release_approach_lift_m={max(args.release_approach_lift_m, 0.0):.3f} "
        f"post_release_safe_lift_m={max(args.post_release_safe_lift_m, 0.0):.3f}"
    )
    print(
        f"[Azas] cup_holder_after_sequence={str(args.place_cup_holder_after_sequence).lower()} "
        f"place_final_x_offset_m={args.cup_holder_place_final_x_offset_m:.3f} "
        f"place_final_y_offset_m={args.cup_holder_place_final_y_offset_m:.3f} "
        f"place_final_z_offset_m={args.cup_holder_place_final_z_offset_m:.3f} "
        f"rz_offset_deg={args.cup_holder_rz_offset_deg:.1f}"
    )
    try:
        resume_tracker = RecipeResumeTracker(args, dispenser_ids, grouped_dispenser_ids)
    except ValueError as exc:
        print(f"[BLOCKED] resume_state: {exc}")
        return 2
    if args.execute:
        print(
            f"[Azas] resume_state_file={resume_tracker.path} "
            f"auto_resume={str(args.resume).lower()} loaded={str(resume_tracker.loaded).lower()}"
        )

    motion: IntegratedRecipeMotion | None = None
    if args.execute and not args.legacy_subprocess_primitives:
        print("[Azas] integrated_motion=true (persistent ROS clients for move/release/re-grasp)")
        try:
            motion = IntegratedRecipeMotion(args)
            motion.preflight()
        except RuntimeError as exc:
            print(f"[FAIL] integrated preflight failed: {exc}")
            if motion is not None:
                motion.close()
            return 1
        try:
            motion.ensure_start_safe_pose()
        except RuntimeError as exc:
            print(f"[FAIL] start safe lift failed; no recipe motion sent: {exc}")
            motion.close()
            return 1
    elif args.execute:
        print("[Azas] integrated_motion=false (legacy subprocess primitives requested)")
        if args.force_start_safe_lift or args.skip_release_pre:
            print(
                "[BLOCKED] legacy subprocess primitives cannot run the start safe lift or the "
                "posture-independent safe release; use integrated motion or pass "
                "--no-force-start-safe-lift and --no-skip-release-pre explicitly"
            )
            return 2

    try:
        total_groups = len(grouped_dispenser_ids)
        for index, (dispenser_id, press_count) in enumerate(grouped_dispenser_ids, start=1):
            label_prefix = f"recipe group {index}/{total_groups} dispenser {dispenser_id} x{press_count}"
            final_regrasp = index == total_groups
            print(f"[Azas] START {label_prefix}: physical_dispenser={dispenser_id}")
            move_release_needed = resume_tracker.should_run_stage(index, "move_release")
            press_needed = resume_tracker.should_run_stage(index, "press")
            regrasp_needed = resume_tracker.should_run_stage(index, "regrasp")
            if args.execute and not (move_release_needed or press_needed or regrasp_needed):
                print(f"[Azas] SKIP {label_prefix}: completed in resume_state")
                continue
            if args.execute:
                try:
                    require_dispenser_press_contact_enabled(dispenser_id)
                except ValueError as exc:
                    print(f"[BLOCKED] {label_prefix}: {exc}")
                    return 1
            if not args.execute:
                move_release_step = "skip initial move/release" if args.skip_initial_move_release else "integrated move/release"
                print(
                    f"[PLAN] {label_prefix}: {move_release_step} -> "
                    f"integrated press {press_count} time(s) -> integrated re-grasp/lift"
                )
                print_dry_run_group_detail(args, dispenser_id, press_count)
                continue

            if move_release_needed:
                resume_tracker.start_stage(index, "move_release", dispenser_id, press_count)
            if move_release_needed and args.skip_initial_move_release:
                print(
                    f"[Azas] {label_prefix}: skipping initial move/release; "
                    "cup is assumed already released at dispenser front-hold"
                )
                resume_tracker.complete_stage(index, "move_release")
            elif move_release_needed and motion is None:
                print(f"[Azas] {label_prefix}: MOVE/RELEASE physical_dispenser={dispenser_id}")
                rc = run_command(f"{label_prefix}: move cup to front-hold and release", move_and_release_cmd(args, dispenser_id))
                if rc != 0:
                    return rc
                rc = run_command(f"{label_prefix}: RG2 full-open release verify", [str(RG2_OPEN)])
                if rc != 0:
                    return rc
                resume_tracker.complete_stage(index, "move_release")
            elif move_release_needed:
                try:
                    print(f"[Azas] {label_prefix}: MOVE/RELEASE physical_dispenser={dispenser_id}")
                    motion.move_and_release(dispenser_id)
                except RuntimeError as exc:
                    print(f"[FAIL] {label_prefix}: integrated move/release failed: {exc}")
                    return 1
                resume_tracker.complete_stage(index, "move_release")
            else:
                print(f"[Azas] SKIP {label_prefix}: move/release completed in resume_state")

            if motion is None and (press_needed or regrasp_needed):
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
            if press_needed:
                resume_tracker.start_stage(index, "press", dispenser_id, press_count)
            if press_needed and motion is None:
                print(f"[Azas] {label_prefix}: PRESS physical_dispenser={dispenser_id} count={press_count}")
                rc = run_command(
                    f"{label_prefix}: press dispenser {press_count} time(s)",
                    press_cmd(args, dispenser_id, press_count),
                )
                if rc != 0:
                    return rc
                resume_tracker.complete_stage(index, "press")
            elif press_needed:
                try:
                    print(f"[Azas] {label_prefix}: PRESS physical_dispenser={dispenser_id} count={press_count}")
                    motion.press_dispenser(dispenser_id, press_count)
                except RuntimeError as exc:
                    print(f"[FAIL] {label_prefix}: integrated press failed: {exc}")
                    return 1
                resume_tracker.complete_stage(index, "press")
            else:
                print(f"[Azas] SKIP {label_prefix}: press completed in resume_state")

            if regrasp_needed:
                resume_tracker.start_stage(index, "regrasp", dispenser_id, press_count)
            if regrasp_needed and motion is None:
                print(f"[Azas] {label_prefix}: RE-GRASP physical_dispenser={dispenser_id}")
                rc = run_command(f"{label_prefix}: re-grasp cup from front-hold", pick_cmd(args, dispenser_id))
                if rc != 0:
                    return rc
                resume_tracker.complete_stage(index, "regrasp")
            elif regrasp_needed:
                try:
                    regrasp_label = "FINAL RE-GRASP" if final_regrasp else "RE-GRASP"
                    print(f"[Azas] {label_prefix}: {regrasp_label} physical_dispenser={dispenser_id}")
                    motion.regrasp_and_lift(dispenser_id, final_regrasp=final_regrasp)
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
                resume_tracker.complete_stage(index, "regrasp")
            else:
                print(f"[Azas] SKIP {label_prefix}: re-grasp/lift completed in resume_state")

            if motion is None and regrasp_needed:
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
        if not args.execute:
            if args.place_cup_holder_after_sequence:
                print(
                    "[PLAN] after final re-grasp: CUP_HOLDER_PRE_PLACE -> "
                    "CUP_HOLDER_PLACE_FINAL -> RG2_OPEN -> CUP_HOLDER_RETREAT"
                )
        elif args.place_cup_holder_after_sequence:
            cup_holder_needed = resume_tracker.should_run_cup_holder()
            if not cup_holder_needed:
                print("[Azas] SKIP final cup-holder place: completed in resume_state")
                resume_tracker.complete_all()
            else:
                resume_tracker.start_cup_holder()
            if cup_holder_needed and motion is None:
                rc = run_command(
                    "place final cup in holder",
                    [
                        sys.executable,
                        str(ROOT / "tools" / "run" / "place_side_grip_cup_in_holder.py"),
                        "--service-prefix",
                        args.service_prefix,
                        "--config",
                        str(CALIBRATION_CONFIG),
                        "--motion-backend",
                        "direct",
                        "--approach-velocity",
                        f"{args.cup_holder_approach_velocity:.6f}",
                        "--approach-acceleration",
                        f"{args.cup_holder_approach_acceleration:.6f}",
                        "--place-velocity",
                        f"{args.cup_holder_place_velocity:.6f}",
                        "--place-acceleration",
                        f"{args.cup_holder_place_acceleration:.6f}",
                        "--retreat-velocity",
                        f"{args.cup_holder_retreat_velocity:.6f}",
                        "--retreat-acceleration",
                        f"{args.cup_holder_retreat_acceleration:.6f}",
                        "--place-final-z-offset-m",
                        f"{args.cup_holder_place_final_z_offset_m:.6f}",
                        "--place-final-x-offset-m",
                        f"{args.cup_holder_place_final_x_offset_m:.6f}",
                        "--place-final-y-offset-m",
                        f"{args.cup_holder_place_final_y_offset_m:.6f}",
                        "--rz-offset-deg",
                        f"{args.cup_holder_rz_offset_deg:.6f}",
                        "--timeout-sec",
                        f"{args.cup_holder_timeout_sec:.6f}",
                        "--target-tolerance-mm",
                        f"{args.cup_holder_target_tolerance_mm:.6f}",
                        "--execute",
                        "--confirm",
                        "ENABLE_CUP_HOLDER_PLACE",
                    ],
                )
                if rc != 0:
                    return rc
                resume_tracker.complete_all()
            elif cup_holder_needed:
                try:
                    print("[Azas] final: PLACE CUP IN HOLDER")
                    motion.place_cup_in_holder()
                except RuntimeError as exc:
                    print(f"[FAIL] final cup-holder place failed: {exc}")
                    return 1
                resume_tracker.complete_all()
        elif args.execute:
            resume_tracker.complete_all()
    finally:
        if motion is not None:
            motion.close()

    print("[PASS] measured dispenser recipe sequence completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
