from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from geometry_msgs.msg import Pose


@dataclass(frozen=True)
class LidGripConfig:
    approach_offset_m: float = 0.08
    lift_offset_m: float = 0.10
    surface_offset_m: float = 0.0
    offset_axis: str = "local_z"
    tcp_grasp_offset_x_m: float = 0.0
    tcp_grasp_offset_y_m: float = 0.0
    tcp_grasp_offset_z_m: float = 0.0
    min_grasp_z_m: float = 0.02
    max_grasp_z_m: float = 0.60


@dataclass(frozen=True)
class LidGripPlan:
    approach_pose: Pose
    grasp_pose: Pose
    lift_pose: Pose
    approach_offset_m: float
    lift_offset_m: float


def compute_lid_grip_plan(lid_pose: Pose, config: LidGripConfig) -> LidGripPlan:
    """Compute plan-only lid approach/grasp/lift candidates from a detected pose.

    The lid pose must already come from live perception and TF conversion. This
    function does not know robot coordinates, collision state, or gripper width.
    """
    _validate_config(config)
    _validate_pose(lid_pose)

    grasp_pose = _translated_by_base_offset(
        lid_pose,
        np.array(
            [
                config.tcp_grasp_offset_x_m,
                config.tcp_grasp_offset_y_m,
                config.tcp_grasp_offset_z_m,
            ],
            dtype=float,
        ),
    )
    grasp_pose = _translated_along_offset_axis(
        grasp_pose,
        config.surface_offset_m,
        config.offset_axis,
    )
    if grasp_pose.position.z < config.min_grasp_z_m or grasp_pose.position.z > config.max_grasp_z_m:
        raise ValueError(
            "LID_GRASP_Z_OUT_OF_BOUNDS: "
            f"grasp_z={grasp_pose.position.z:.3f} "
            f"outside [{config.min_grasp_z_m:.3f}, {config.max_grasp_z_m:.3f}]"
        )

    approach_pose = _translated_along_offset_axis(
        grasp_pose,
        config.approach_offset_m,
        config.offset_axis,
    )
    lift_pose = _translated_along_offset_axis(
        grasp_pose,
        config.lift_offset_m,
        config.offset_axis,
    )
    return LidGripPlan(
        approach_pose=approach_pose,
        grasp_pose=grasp_pose,
        lift_pose=lift_pose,
        approach_offset_m=config.approach_offset_m,
        lift_offset_m=config.lift_offset_m,
    )


def _validate_config(config: LidGripConfig) -> None:
    values = (
        config.approach_offset_m,
        config.lift_offset_m,
        config.surface_offset_m,
        config.tcp_grasp_offset_x_m,
        config.tcp_grasp_offset_y_m,
        config.tcp_grasp_offset_z_m,
        config.min_grasp_z_m,
        config.max_grasp_z_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("lid grip config values must be finite")
    if config.approach_offset_m <= 0.0:
        raise ValueError("approach_offset_m must be positive")
    if config.lift_offset_m <= 0.0:
        raise ValueError("lift_offset_m must be positive")
    if config.min_grasp_z_m > config.max_grasp_z_m:
        raise ValueError("min_grasp_z_m must be <= max_grasp_z_m")
    if config.offset_axis not in ("local_z", "base_z"):
        raise ValueError("offset_axis must be 'local_z' or 'base_z'")


def _validate_pose(pose: Pose) -> None:
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("lid pose values must be finite")
    norm = math.sqrt(
        pose.orientation.x * pose.orientation.x
        + pose.orientation.y * pose.orientation.y
        + pose.orientation.z * pose.orientation.z
        + pose.orientation.w * pose.orientation.w
    )
    if norm <= 1e-12:
        raise ValueError("lid pose quaternion norm must be non-zero")


def _translated_along_offset_axis(pose: Pose, distance_m: float, offset_axis: str) -> Pose:
    if offset_axis == "base_z":
        z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        z_axis = _local_z_axis(pose)
    output = _copy_pose(pose)
    output.position.x += float(distance_m) * z_axis[0]
    output.position.y += float(distance_m) * z_axis[1]
    output.position.z += float(distance_m) * z_axis[2]
    return output


def _translated_by_base_offset(pose: Pose, offset: np.ndarray) -> Pose:
    output = _copy_pose(pose)
    output.position.x += float(offset[0])
    output.position.y += float(offset[1])
    output.position.z += float(offset[2])
    return output


def _local_z_axis(pose: Pose) -> np.ndarray:
    qx = float(pose.orientation.x)
    qy = float(pose.orientation.y)
    qz = float(pose.orientation.z)
    qw = float(pose.orientation.w)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array(
        [
            2.0 * (qx * qz + qy * qw),
            2.0 * (qy * qz - qx * qw),
            1.0 - 2.0 * (qx * qx + qy * qy),
        ],
        dtype=float,
    )


def _copy_pose(source: Pose) -> Pose:
    pose = Pose()
    pose.position.x = source.position.x
    pose.position.y = source.position.y
    pose.position.z = source.position.z
    pose.orientation.x = source.orientation.x
    pose.orientation.y = source.orientation.y
    pose.orientation.z = source.orientation.z
    pose.orientation.w = source.orientation.w
    return pose
