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
from dsr_msgs2.srv import GetCurrentPosx, Ikin, MoveLine


ROOT = Path("/home/ssu/Azas")
DEFAULT_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "measured_dispenser_collision.yaml"
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
DISPENSER_IDS_BY_COLOR = {color: dispenser_id for dispenser_id, color in DISPENSER_TARGETS.items()}

DR_BASE = 0
MOVE_MODE_ABSOLUTE = 0
SYNC = 0
BLENDING_SPEED_TYPE_DUPLICATE = 0
Pose = tuple[list[float], list[list[float]]]


def parse_dispenser_ids(raw: str) -> list[str]:
    """Parse STT/recipe dispenser order into numeric dispenser IDs.

    The STT/LLM layer may hand off color names such as
    ``red,yellow,blue,green``.  The motion layer only receives the resulting
    fixed dispenser IDs and never asks for or generates cup coordinates.
    """
    values = [item.strip().lower() for item in raw.replace(";", ",").split(",") if item.strip()]
    if not values:
        raise ValueError("at least one dispenser id/color is required")

    parsed: list[str] = []
    invalid: list[str] = []
    for value in values:
        if value in DISPENSER_TARGETS:
            parsed.append(value)
        elif value in DISPENSER_IDS_BY_COLOR:
            parsed.append(DISPENSER_IDS_BY_COLOR[value])
        else:
            invalid.append(value)
    if invalid:
        allowed_colors = ",".join(DISPENSER_IDS_BY_COLOR)
        raise ValueError(
            f"unsupported dispenser id/color(s): {', '.join(invalid)}; "
            f"allowed ids: 1,2,3,4; allowed colors: {allowed_colors}"
        )
    return parsed


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
        self.ikin = self.node.create_client(Ikin, service_name(args.service_prefix, "motion/ikin"))
        self.get_posx = self.node.create_client(GetCurrentPosx, service_name(args.service_prefix, "aux_control/get_current_posx"))
        self.gripper = self.node.create_client(SetGripper, args.gripper_service)

    def close(self) -> None:
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

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

    def current_posx(self, timeout_sec: float | None = None) -> list[float]:
        req = GetCurrentPosx.Request()
        req.ref = DR_BASE
        response = self._call(
            self.get_posx,
            req,
            timeout_sec=timeout_sec or self.args.wait_service_sec,
            label="GetCurrentPosx",
        )
        if not response.success or not response.task_pos_info:
            raise RuntimeError("GetCurrentPosx returned success=false or empty task_pos_info")
        values = list(response.task_pos_info[0].data)
        if len(values) < 6:
            raise RuntimeError(f"GetCurrentPosx returned too few values: {values}")
        return [float(value) for value in values[:6]]

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
            raise RuntimeError(f"MoveLine returned success=false for {label}")
        self.wait_for_target(pos, label=label)

    def wait_for_target(self, target_pos_mm_deg: list[float], *, label: str) -> None:
        deadline = time.monotonic() + max(self.args.verify_timeout_sec, 0.1)
        last_distance = 999999.0
        while time.monotonic() < deadline:
            actual = self.current_posx(timeout_sec=5.0)
            last_distance = sum((actual[index] - target_pos_mm_deg[index]) ** 2 for index in range(3)) ** 0.5
            print(f"[Azas] verify {label}: distance={last_distance:.1f}mm tolerance={self.args.target_tolerance_mm:.1f}mm")
            if last_distance <= max(self.args.target_tolerance_mm, 0.1):
                return
            time.sleep(1.0)
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
            ("above-hold", 0.0, 0.0, self.args.move_prehold_offset_z_m, self.args.move_prehold_velocity, self.args.move_prehold_acceleration),
            ("front-hold", 0.0, 0.0, 0.0, self.args.move_velocity, self.args.move_acceleration),
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

    def regrasp_and_lift(self, dispenser_id: str) -> None:
        self.gripper_command(
            "open",
            width_m=self.args.gripper_open_width_m,
            force_n=self.args.gripper_force_n,
            label="RG2 open before re-grasp",
        )
        self.move_front_hold(
            dispenser_id,
            label="final re-grasp front-hold",
            offset_x_m=0.0,
            offset_y_m=0.0,
            offset_z_m=0.0,
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
        self.wait_for_target(target, label="post-grasp lift")


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


def press_cmd(args: argparse.Namespace, dispenser_id: str) -> str:
    target = DISPENSER_TARGETS[dispenser_id]
    service_prefix = shlex.quote(args.service_prefix)
    tcp_name = shlex.quote(args.dispenser_tcp_name)
    target_q = shlex.quote(target)
    return (
        "ros2 run azas_dispenser dispenser_press_node --ros-args "
        f"-p service_prefix:={service_prefix} "
        "-p use_taught_posx:=true "
        f"-p tcp_name:={tcp_name} "
        "-p require_tcp_for_taught_posx:=false "
        "-p allow_tcp_set_failure:=true "
        f"-p target_dispenser:={target_q} "
        "-p move_home_first:=true "
        "-p pre_home_retreat_before_home:=true "
        "-p pre_home_retreat_dx_mm:=-180.0 "
        "-p pre_home_retreat_dy_mm:=0.0 "
        "-p pre_home_retreat_min_z_mm:=520.0 -p pre_home_retreat_lift_first:=true "
        "-p pre_home_retreat_min_current_x_mm:=450.0 "
        "-p pre_home_retreat_velocity:=20.0 "
        "-p pre_home_retreat_acceleration:=25.0 "
        "-p joint1_clearance_before_home:=false "
        "-p joint1_clearance_return_home:=false "
        "-p joint1_clearance_offset_deg:=12.0 "
        "-p return_home:=true "
        "-p close_gripper_at_home:=true "
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
    parser.add_argument(
        "--dispenser-ids",
        default="1,2,3,4",
        help="comma-separated dispenser IDs or colors from STT, e.g. 1,3,2 or red,yellow,blue",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--dispenser-tcp-name", default="GripperDA_v1_jarvis")
    parser.add_argument("--move-velocity", type=float, default=30.0)
    parser.add_argument("--move-acceleration", type=float, default=30.0)
    parser.add_argument("--move-prehold-offset-x-m", type=float, default=0.0)
    parser.add_argument("--move-prehold-offset-y-m", type=float, default=0.0)
    parser.add_argument("--move-prehold-offset-z-m", type=float, default=0.0)
    parser.add_argument("--move-prehold-velocity", type=float, default=12.0)
    parser.add_argument("--move-prehold-acceleration", type=float, default=16.0)
    parser.add_argument("--move-timeout-sec", type=float, default=180.0)
    parser.add_argument("--pick-approach-velocity", type=float, default=15.0)
    parser.add_argument("--pick-approach-acceleration", type=float, default=20.0)
    parser.add_argument("--pick-pregrasp-offset-x-m", type=float, default=0.0)
    parser.add_argument("--pick-pregrasp-offset-y-m", type=float, default=0.0)
    parser.add_argument("--pick-pregrasp-offset-z-m", type=float, default=0.0)
    parser.add_argument("--pick-pregrasp-staging-velocity", type=float, default=12.0)
    parser.add_argument("--pick-pregrasp-staging-acceleration", type=float, default=16.0)
    parser.add_argument("--pick-lift-m", type=float, default=0.100)
    parser.add_argument("--pick-lift-velocity", type=float, default=12.0)
    parser.add_argument("--pick-lift-acceleration", type=float, default=16.0)
    parser.add_argument("--pick-timeout-sec", type=float, default=120.0)
    parser.add_argument("--wait-service-sec", type=float, default=8.0)
    parser.add_argument("--verify-timeout-sec", type=float, default=70.0)
    parser.add_argument("--target-tolerance-mm", type=float, default=15.0)
    parser.add_argument("--gripper-service", default="/jarvis/rg2/set_width")
    parser.add_argument("--gripper-open-width-m", type=float, default=0.110)
    parser.add_argument("--gripper-open-force-n", type=float, default=12.0)
    parser.add_argument("--gripper-grasp-width-m", type=float, default=0.075)
    parser.add_argument("--gripper-force-n", type=float, default=25.0)
    parser.add_argument("--gripper-timeout-sec", type=float, default=12.0)
    parser.add_argument("--precheck-ikin", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--ikin-sol-space", type=int, default=2)
    parser.add_argument("--legacy-subprocess-primitives", action="store_true", help="use the old helper-script-per-step implementation for fallback/debugging")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} when --execute is used")
    return parser.parse_args()


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
    print(f"[Azas] dispenser_order_raw={args.dispenser_ids}")
    print(f"[Azas] dispenser_ids={','.join(dispenser_ids)}")
    print("[Azas] dispenser_colors=" + ",".join(DISPENSER_TARGETS[item] for item in dispenser_ids))
    print(f"[Azas] service_prefix={args.service_prefix}")
    print(f"[Azas] dispenser_tcp_name={args.dispenser_tcp_name}")
    print("[Azas] source=existing measured front_hold poses and taught dispenser press poses")

    motion: IntegratedRecipeMotion | None = None
    if args.execute and not args.legacy_subprocess_primitives:
        print("[Azas] integrated_motion=true (persistent ROS clients for move/release/re-grasp)")
        motion = IntegratedRecipeMotion(args)
    elif args.execute:
        print("[Azas] integrated_motion=false (legacy subprocess primitives requested)")

    try:
        for index, dispenser_id in enumerate(dispenser_ids, start=1):
            label_prefix = f"recipe {index}/{len(dispenser_ids)} dispenser {dispenser_id}"
            if not args.execute:
                print(f"[PLAN] {label_prefix}: integrated move/release -> press -> integrated re-grasp/lift (move/release -> press -> re-grasp/lift)")
                continue

            if motion is None:
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
            rc = run_command(f"{label_prefix}: press dispenser", press_cmd(args, dispenser_id))
            if rc != 0:
                return rc

            if motion is None:
                rc = run_command(f"{label_prefix}: re-grasp cup from front-hold", pick_cmd(args, dispenser_id))
                if rc != 0:
                    return rc
            else:
                try:
                    motion.regrasp_and_lift(dispenser_id)
                except RuntimeError as exc:
                    print(f"[FAIL] {label_prefix}: integrated re-grasp/lift failed: {exc}")
                    return 1

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
