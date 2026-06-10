#!/usr/bin/env python3
"""Move to a measured fixed dispenser front-hold pose.

The target is read from measured_dispenser_collision.yaml front_hold_poses.
This is for fixed dispenser/link_6 teaching poses only; cup pose still comes
from vision in the full pipeline.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import yaml
import rclpy
import tf2_ros
from dsr_msgs2.srv import ConfigCreateTcp, GetCurrentPosx, GetCurrentTcp, SetCurrentTcp


ROOT = Path("/home/ssu/Azas")
DEFAULT_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "measured_dispenser_collision.yaml"
DIRECT_MOVEL = ROOT / "tools" / "run" / "direct_movel_xyz.py"
MOVEIT_PLAN_GUARD = ROOT / "tools" / "run" / "check_link6_pose_moveit_plan.py"
CONFIRM_PHRASE = "ENABLE_MEASURED_DISPENSER_FRONT_HOLD"
DIRECT_CONFIRM_PHRASE = "ENABLE_DIRECT_MOVEL"


def service_name(prefix: str, suffix: str) -> str:
    clean_prefix = prefix.strip("/")
    clean_suffix = suffix.strip("/")
    if not clean_prefix:
        return f"/{clean_suffix}"
    return f"/{clean_prefix}/{clean_suffix}"


def numeric_list(value: Any, label: str, count: int) -> list[float]:
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{label} must be a {count}-number list")
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain only numbers") from exc


def xyz(value: Any, label: str) -> list[float]:
    return numeric_list(value, label, 3)


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


def quaternion_to_doosan_zyz_deg(quaternion: list[float]) -> list[float]:
    """Convert ROS quaternion XYZW to the Doosan posx ZYZ Euler convention."""
    matrix = quaternion_to_matrix_xyzw(quaternion)
    return matrix_to_doosan_zyz_deg(matrix)


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
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def matvec3(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3)]


def transpose3(matrix: list[list[float]]) -> list[list[float]]:
    return [[matrix[col][row] for col in range(3)] for row in range(3)]


Pose = tuple[list[float], list[list[float]]]


def pose_inverse(pose: Pose) -> Pose:
    position, rotation = pose
    rotation_t = transpose3(rotation)
    inverse_position = [-value for value in matvec3(rotation_t, position)]
    return inverse_position, rotation_t


def pose_multiply(a: Pose, b: Pose) -> Pose:
    a_position, a_rotation = a
    b_position, b_rotation = b
    rotated_b = matvec3(a_rotation, b_position)
    return (
        [a_position[index] + rotated_b[index] for index in range(3)],
        matmul3(a_rotation, b_rotation),
    )


def transform_to_pose(transform: Any) -> Pose:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (
        [float(translation.x), float(translation.y), float(translation.z)],
        quaternion_to_matrix_xyzw(
            [float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)]
        ),
    )


def load_pose(
    config_path: Path, dispenser_id: int
) -> tuple[list[float], list[float], list[float], list[float], str, str]:
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    metadata = data.get("metadata") or {}
    reference_frame = str(metadata.get("frame_id") or "base_link")
    target_frame = str(metadata.get("measured_target_frame") or "")
    poses = data.get("front_hold_poses") or {}
    key = f"dispenser_{dispenser_id}"
    block = poses.get(key)
    if not isinstance(block, dict):
        raise ValueError(f"front_hold_poses.{key} is missing in {config_path}")
    position = xyz(block.get("position_xyz_m"), f"front_hold_poses.{key}.position_xyz_m")
    ros_rpy = xyz(block.get("rpy_deg"), f"front_hold_poses.{key}.rpy_deg")
    quaternion = numeric_list(
        block.get("quaternion_xyzw"), f"front_hold_poses.{key}.quaternion_xyzw", 4
    )
    doosan_zyz = quaternion_to_doosan_zyz_deg(quaternion)
    return position, doosan_zyz, ros_rpy, quaternion, reference_frame, target_frame


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move to measured dispenser_N front_hold pose from measured_dispenser_collision.yaml."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--dispenser-id", type=int, default=2, choices=(1, 2, 3, 4))
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--velocity", type=float, default=8.0)
    parser.add_argument("--acceleration", type=float, default=8.0)
    parser.add_argument("--timeout-sec", type=float, default=180.0)
    parser.add_argument("--wait-service-sec", type=float, default=8.0)
    parser.add_argument("--target-tolerance-mm", type=float, default=15.0)
    parser.add_argument("--verify-timeout-sec", type=float, default=70.0)
    parser.add_argument(
        "--target-offset-x-m",
        type=float,
        default=0.0,
        help="Base-frame X offset added to the measured link_6 target; use only for derived staging poses.",
    )
    parser.add_argument(
        "--target-offset-y-m",
        type=float,
        default=0.0,
        help="Base-frame Y offset added to the measured link_6 target; use only for derived staging poses.",
    )
    parser.add_argument(
        "--target-offset-z-m",
        type=float,
        default=0.0,
        help="Base-frame Z offset added to the measured link_6 target; use only for derived staging poses.",
    )
    parser.add_argument("--precheck-ikin", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--ikin-timeout-sec",
        type=float,
        default=20.0,
        help="service response timeout for each /motion/ikin precheck attempt",
    )
    parser.add_argument(
        "--ikin-retries",
        type=int,
        default=2,
        help="number of /motion/ikin precheck attempts before failing closed",
    )
    parser.add_argument("--verify-target", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--moveit-planning-guard",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Before real direct MoveLine execution, require MoveItPy to find a "
            "collision-scene-aware plan from the current state to the measured link_6 target. "
            "Failure blocks the direct motion."
        ),
    )
    parser.add_argument(
        "--moveit-guard-frame-id",
        default="",
        help="Override the MoveIt target pose frame. Defaults to metadata.frame_id from the measured config.",
    )
    parser.add_argument("--moveit-guard-ee-link", default="link_6")
    parser.add_argument("--moveit-guard-planning-group", default="manipulator")
    parser.add_argument("--moveit-guard-robot-model", default="m0609")
    parser.add_argument("--moveit-guard-config-package", default="dsr_moveit_config_m0609")
    parser.add_argument("--moveit-guard-planning-pipeline", default="pilz_industrial_motion_planner")
    parser.add_argument("--moveit-guard-planner-id", default="PTP")
    parser.add_argument("--moveit-guard-timeout-sec", type=float, default=3.0)
    parser.add_argument("--moveit-guard-attempts", type=int, default=1)
    parser.add_argument(
        "--compensate-current-tcp",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "front_hold_poses are link_6 poses. If the controller has a non-zero current TCP, "
            "convert the link_6 target into a current-TCP MoveLine target instead of moving link_6 poses directly."
        ),
    )
    parser.add_argument("--verify-link6-target", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--set-current-tcp-before-move",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "front_hold_poses are taught as link_6 poses, so real execution first "
            "sets the Doosan controller TCP to --link6-tcp-name."
        ),
    )
    parser.add_argument(
        "--link6-tcp-name",
        default="azas_link6_tcp",
        help="Doosan controller TCP name that means flange/link_6 with zero offset.",
    )
    parser.add_argument(
        "--ensure-link6-tcp",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="create/update a zero-offset controller TCP before selecting it.",
    )
    parser.add_argument("--tcp-wait-service-sec", type=float, default=5.0)
    parser.add_argument("--tcp-timeout-sec", type=float, default=8.0)
    parser.add_argument("--direct-x-min", type=float, default=0.10)
    parser.add_argument("--direct-x-max", type=float, default=0.70)
    parser.add_argument("--direct-y-min", type=float, default=-0.45)
    parser.add_argument("--direct-y-max", type=float, default=0.45)
    parser.add_argument("--direct-z-min", type=float, default=0.05)
    parser.add_argument("--direct-z-max", type=float, default=0.80)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--confirm",
        default="",
        help=f"must equal {CONFIRM_PHRASE} when --execute is used",
    )
    return parser.parse_args()


def call_service(
    node: Any,
    srv_type: Any,
    name: str,
    request: Any,
    *,
    timeout_sec: float,
    label: str,
) -> Any:
    client = node.create_client(srv_type, name)
    timeout_sec = max(timeout_sec, 0.1)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"{label} service not available: {name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        raise RuntimeError(f"{label} response timeout after {timeout_sec:.1f}s")
    if future.exception() is not None:
        raise RuntimeError(f"{label} exception: {future.exception()}")
    response = future.result()
    if response is None:
        raise RuntimeError(f"{label} returned no response")
    return response


def current_tcp_pose(node: Any, service_prefix: str, timeout_sec: float) -> Pose:
    req = GetCurrentPosx.Request()
    req.ref = 0
    response = call_service(
        node,
        GetCurrentPosx,
        service_name(service_prefix, "aux_control/get_current_posx"),
        req,
        timeout_sec=timeout_sec,
        label="GetCurrentPosx",
    )
    if not response.success or not response.task_pos_info:
        raise RuntimeError("GetCurrentPosx returned success=false or empty task_pos_info")
    values = list(response.task_pos_info[0].data)
    if len(values) < 6:
        raise RuntimeError(f"GetCurrentPosx returned too few values: {values}")
    position_m = [float(values[index]) / 1000.0 for index in range(3)]
    rotation = doosan_zyz_deg_to_matrix([float(values[index]) for index in range(3, 6)])
    return position_m, rotation


def current_tcp_name(node: Any, service_prefix: str, timeout_sec: float) -> str:
    response = call_service(
        node,
        GetCurrentTcp,
        service_name(service_prefix, "tcp/get_current_tcp"),
        GetCurrentTcp.Request(),
        timeout_sec=timeout_sec,
        label="GetCurrentTcp",
    )
    if not response.success:
        raise RuntimeError("GetCurrentTcp returned success=false")
    return str(response.info).strip()


def lookup_pose(
    node: Any,
    target_frame: str,
    source_frame: str,
    timeout_sec: float,
) -> Pose:
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    last_error = ""
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            transform = buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )
            return transform_to_pose(transform)
        except Exception as exc:  # tf2 exception types differ across installs.
            last_error = str(exc)
            time.sleep(0.05)
    # Keep listener alive until function exit; otherwise no transform subscription.
    del listener
    raise RuntimeError(
        f"TF lookup {target_frame}->{source_frame} timed out after {timeout_sec:.1f}s: {last_error}"
    )


def compensated_current_tcp_target(
    *,
    service_prefix: str,
    desired_link6_pose: Pose,
    timeout_sec: float,
) -> tuple[Pose, str, Pose]:
    rclpy.init(args=None)
    node = rclpy.create_node("azas_measured_dispenser_front_hold_tcp_compensator")
    try:
        tcp_name = current_tcp_name(node, service_prefix, timeout_sec)
        live_tcp_pose = current_tcp_pose(node, service_prefix, timeout_sec)
        live_link6_pose = lookup_pose(node, "base_link", "link_6", timeout_sec)
        link6_to_tcp = pose_multiply(pose_inverse(live_link6_pose), live_tcp_pose)
        desired_tcp_pose = pose_multiply(desired_link6_pose, link6_to_tcp)
        return desired_tcp_pose, tcp_name, link6_to_tcp
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def wait_for_link6_target(
    *,
    target_position_m: list[float],
    timeout_sec: float,
    tolerance_mm: float,
) -> bool:
    rclpy.init(args=None)
    node = rclpy.create_node("azas_measured_dispenser_front_hold_link6_verify")
    try:
        deadline = time.monotonic() + max(timeout_sec, 0.1)
        last_line = ""
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                link6_pose = lookup_pose(node, "base_link", "link_6", min(2.0, timeout_sec))
            except RuntimeError as exc:
                last_line = f"[Azas] link_6 verify TF wait: {exc}"
                print(last_line)
                time.sleep(0.5)
                continue
            actual = link6_pose[0]
            distance_mm = (
                sum((actual[index] - target_position_m[index]) ** 2 for index in range(3))
                ** 0.5
                * 1000.0
            )
            last_line = (
                "[Azas] verify link_6 xyz="
                f"[{actual[0] * 1000.0:.1f}, {actual[1] * 1000.0:.1f}, {actual[2] * 1000.0:.1f}] "
                f"target=[{target_position_m[0] * 1000.0:.1f}, {target_position_m[1] * 1000.0:.1f}, {target_position_m[2] * 1000.0:.1f}] "
                f"distance={distance_mm:.1f}mm tolerance={tolerance_mm:.1f}mm"
            )
            print(last_line)
            if distance_mm <= tolerance_mm:
                return True
            time.sleep(1.0)
        print("[FAIL] link_6 target verification timeout")
        if last_line:
            print(last_line)
        return False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def set_and_verify_current_tcp(
    *,
    service_prefix: str,
    desired_name: str,
    wait_service_sec: float,
    timeout_sec: float,
    ensure_zero_tcp: bool,
) -> tuple[bool, str]:
    rclpy.init(args=None)
    node = rclpy.create_node("azas_measured_dispenser_front_hold_tcp_guard")
    try:
        set_client = node.create_client(
            SetCurrentTcp,
            service_name(service_prefix, "tcp/set_current_tcp"),
        )
        get_client = node.create_client(
            GetCurrentTcp,
            service_name(service_prefix, "tcp/get_current_tcp"),
        )
        create_client = node.create_client(
            ConfigCreateTcp,
            service_name(service_prefix, "tcp/config_create_tcp"),
        )
        if not set_client.wait_for_service(timeout_sec=max(wait_service_sec, 0.1)):
            return False, "tcp/set_current_tcp service is not available"
        if not get_client.wait_for_service(timeout_sec=max(wait_service_sec, 0.1)):
            return False, "tcp/get_current_tcp service is not available"
        if desired_name and ensure_zero_tcp:
            if not create_client.wait_for_service(timeout_sec=max(wait_service_sec, 0.1)):
                return False, "tcp/config_create_tcp service is not available"
            create_req = ConfigCreateTcp.Request()
            create_req.name = desired_name
            create_req.pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            future = create_client.call_async(create_req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=max(timeout_sec, 0.1))
            if future.done() and future.exception() is None:
                create_response = future.result()
                if create_response is not None and create_response.success:
                    print(f"[Azas] Ensured zero-offset link_6 TCP: {desired_name}")
            else:
                return False, f"tcp/config_create_tcp timed out/failed for '{desired_name}'"

        set_req = SetCurrentTcp.Request()
        set_req.name = desired_name
        future = set_client.call_async(set_req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(timeout_sec, 0.1))
        if not future.done():
            return False, f"tcp/set_current_tcp timed out after {timeout_sec:.1f}s"
        if future.exception() is not None:
            return False, f"tcp/set_current_tcp exception: {future.exception()}"
        set_response = future.result()
        if set_response is None or not set_response.success:
            return (
                False,
                "tcp/set_current_tcp returned success=false. "
                "Register/select the controller TCP that corresponds to link_6/flange.",
            )

        get_req = GetCurrentTcp.Request()
        future = get_client.call_async(get_req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(timeout_sec, 0.1))
        if not future.done():
            return False, f"tcp/get_current_tcp timed out after {timeout_sec:.1f}s"
        if future.exception() is not None:
            return False, f"tcp/get_current_tcp exception: {future.exception()}"
        get_response = future.result()
        if get_response is None or not get_response.success:
            return False, "tcp/get_current_tcp returned success=false"
        current_name = str(get_response.info).strip()
        if current_name != desired_name.strip():
            return (
                False,
                f"current TCP is '{current_name}', expected '{desired_name.strip()}'. "
                "Refusing link_6 measured front-hold motion to avoid shifted positions.",
            )
        return True, current_name
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_moveit_planning_guard(
    *,
    args: argparse.Namespace,
    link6_position: list[float],
    link6_quaternion_xyzw: list[float],
    frame_id: str,
) -> int:
    """Require a MoveIt plan to the measured link_6 target before direct MoveLine."""
    if not args.execute:
        print(
            "[DRY-RUN] MoveIt planning guard would check current state -> measured link_6 "
            "front-hold target before direct MoveLine."
        )
        return 0
    if not args.moveit_planning_guard:
        print("[WARN] MoveIt planning guard disabled; direct MoveLine keeps only legacy local guards.")
        return 0

    cmd = [
        sys.executable,
        str(MOVEIT_PLAN_GUARD),
        "--x",
        f"{link6_position[0]:.6f}",
        "--y",
        f"{link6_position[1]:.6f}",
        "--z",
        f"{link6_position[2]:.6f}",
        "--qx",
        f"{link6_quaternion_xyzw[0]:.9f}",
        "--qy",
        f"{link6_quaternion_xyzw[1]:.9f}",
        "--qz",
        f"{link6_quaternion_xyzw[2]:.9f}",
        "--qw",
        f"{link6_quaternion_xyzw[3]:.9f}",
        "--frame-id",
        str(frame_id or args.moveit_guard_frame_id),
        "--ee-link",
        str(args.moveit_guard_ee_link),
        "--planning-group",
        str(args.moveit_guard_planning_group),
        "--robot-model",
        str(args.moveit_guard_robot_model),
        "--moveit-config-package",
        str(args.moveit_guard_config_package),
        "--planning-pipeline",
        str(args.moveit_guard_planning_pipeline),
        "--planner-id",
        str(args.moveit_guard_planner_id),
        "--planning-timeout-sec",
        f"{args.moveit_guard_timeout_sec:.6f}",
        "--planning-attempts",
        str(max(int(args.moveit_guard_attempts), 1)),
    ]
    print("[Azas] Running MoveIt planning guard before direct measured front-hold MoveLine")
    sys.stdout.flush()
    return subprocess.run(cmd, cwd=str(ROOT), check=False).returncode


def main() -> int:
    args = parse_args()
    if not args.config.is_file():
        print(f"[FAIL] measured dispenser config not found: {args.config}")
        return 2

    try:
        position, doosan_zyz, ros_rpy, quaternion, reference_frame, target_frame = load_pose(
            args.config, args.dispenser_id
        )
    except ValueError as exc:
        print(f"[FAIL] {exc}")
        return 2

    print("[Azas] Measured dispenser front-hold target")
    print(f"[Azas] config={args.config}")
    print(f"[Azas] dispenser_id={args.dispenser_id}")
    print(f"[Azas] frame_id={reference_frame or '<unspecified>'}")
    print(f"[Azas] measured_target_frame={target_frame or '<unspecified>'}")
    offset = [
        float(args.target_offset_x_m),
        float(args.target_offset_y_m),
        float(args.target_offset_z_m),
    ]
    if any(abs(value) > 1e-9 for value in offset):
        position = [position[index] + offset[index] for index in range(3)]
        print(
            "[Azas] derived staging offset_m="
            f"[{offset[0]:.3f}, {offset[1]:.3f}, {offset[2]:.3f}] "
            "applied to measured front_hold link_6 pose"
        )
    print(
        "[Azas] xyz_m="
        f"[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] "
        f"quaternion_xyzw=[{quaternion[0]:.3f}, {quaternion[1]:.3f}, {quaternion[2]:.3f}, {quaternion[3]:.3f}]"
    )
    print(
        "[Azas] yaml_ros_rpy_deg(reference)="
        f"[{ros_rpy[0]:.3f}, {ros_rpy[1]:.3f}, {ros_rpy[2]:.3f}]"
    )
    print(
        "[Azas] direct_movel_doosan_zyz_deg="
        f"[{doosan_zyz[0]:.3f}, {doosan_zyz[1]:.3f}, {doosan_zyz[2]:.3f}]"
    )
    print("[Azas] source=front_hold_poses; not the old temporary direct XYZ candidate")

    if args.execute and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
        return 2

    if args.execute and args.set_current_tcp_before_move:
        link6_tcp_name = str(args.link6_tcp_name).strip()
        printable_name = link6_tcp_name if link6_tcp_name else "<empty/default link_6 TCP>"
        print(f"[Azas] Setting Doosan TCP for measured link_6 target: {printable_name}")
        ok, tcp_output = set_and_verify_current_tcp(
            service_prefix=args.service_prefix,
            desired_name=link6_tcp_name,
            wait_service_sec=args.tcp_wait_service_sec,
            timeout_sec=args.tcp_timeout_sec,
            ensure_zero_tcp=bool(args.ensure_link6_tcp),
        )
        if not ok:
            print(f"[BLOCKED] {tcp_output}")
            return 2
        print(
            "[Azas] Current Doosan TCP verified for link_6 measured target: "
            f"{tcp_output if tcp_output else '<empty/default>'}"
        )

    guard_rc = run_moveit_planning_guard(
        args=args,
        link6_position=position,
        link6_quaternion_xyzw=quaternion,
        frame_id=str(args.moveit_guard_frame_id or reference_frame or "base_link"),
    )
    if guard_rc != 0:
        return guard_rc

    move_position = position
    move_zyz = doosan_zyz
    verify_link6_after_move = False
    if args.execute and args.compensate_current_tcp:
        try:
            desired_link6_pose = (position, quaternion_to_matrix_xyzw(quaternion))
            desired_tcp_pose, tcp_name, link6_to_tcp = compensated_current_tcp_target(
                service_prefix=args.service_prefix,
                desired_link6_pose=desired_link6_pose,
                timeout_sec=max(args.tcp_timeout_sec, args.wait_service_sec, 0.1),
            )
        except RuntimeError as exc:
            print(f"[FAIL] current TCP compensation failed: {exc}")
            return 1
        move_position, move_rotation = desired_tcp_pose
        move_zyz = matrix_to_doosan_zyz_deg(move_rotation)
        offset = link6_to_tcp[0]
        verify_link6_after_move = bool(args.verify_link6_target)
        print(
            "[Azas] Current Doosan TCP is not assumed to be link_6: "
            f"{tcp_name if tcp_name else '<empty/default>'}"
        )
        print(
            "[Azas] Compensating link_6 measured target into current-TCP MoveLine target. "
            f"estimated link_6->TCP offset_m=[{offset[0]:.4f}, {offset[1]:.4f}, {offset[2]:.4f}]"
        )
        print(
            "[Azas] compensated_current_tcp_xyz_m="
            f"[{move_position[0]:.4f}, {move_position[1]:.4f}, {move_position[2]:.4f}] "
            "doosan_zyz_deg="
            f"[{move_zyz[0]:.3f}, {move_zyz[1]:.3f}, {move_zyz[2]:.3f}]"
        )

    cmd = [
        sys.executable,
        str(DIRECT_MOVEL),
        "--service-prefix",
        args.service_prefix,
        "--x",
        f"{move_position[0]:.6f}",
        "--y",
        f"{move_position[1]:.6f}",
        "--z",
        f"{move_position[2]:.6f}",
        "--rx",
        f"{move_zyz[0]:.6f}",
        "--ry",
        f"{move_zyz[1]:.6f}",
        "--rz",
        f"{move_zyz[2]:.6f}",
        "--velocity",
        f"{args.velocity:.6f}",
        "--acceleration",
        f"{args.acceleration:.6f}",
        "--timeout-sec",
        f"{args.timeout_sec:.6f}",
        "--wait-service-sec",
        f"{args.wait_service_sec:.6f}",
        "--ikin-timeout-sec",
        f"{args.ikin_timeout_sec:.6f}",
        "--ikin-retries",
        str(max(int(args.ikin_retries), 1)),
        "--target-tolerance-mm",
        f"{args.target_tolerance_mm:.6f}",
        "--verify-timeout-sec",
        f"{args.verify_timeout_sec:.6f}",
        "--x-min",
        f"{args.direct_x_min:.6f}",
        "--x-max",
        f"{args.direct_x_max:.6f}",
        "--y-min",
        f"{args.direct_y_min:.6f}",
        "--y-max",
        f"{args.direct_y_max:.6f}",
        "--z-min",
        f"{args.direct_z_min:.6f}",
        "--z-max",
        f"{args.direct_z_max:.6f}",
    ]
    if args.precheck_ikin:
        cmd.append("--precheck-ikin")
    if args.verify_target:
        cmd.append("--verify-target")
    if args.execute:
        cmd.extend(["--execute", "--confirm", DIRECT_CONFIRM_PHRASE])

    sys.stdout.flush()
    returncode = subprocess.run(cmd, cwd=str(ROOT), check=False).returncode
    if returncode != 0:
        return returncode
    if verify_link6_after_move and not wait_for_link6_target(
        target_position_m=position,
        timeout_sec=max(args.verify_timeout_sec, 0.1),
        tolerance_mm=max(args.target_tolerance_mm, 0.1),
    ):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
