#!/usr/bin/env python3
"""Move HOME -> wrist camera alignment -> legacy camera_home observation pose.

This extracts only the old yolo_cup_pick_legacy_node observation behavior:
move to joint HOME, rotate joint_6 for the camera-facing HOME orientation, then
plan/execute a base_link pose target at camera_home_x/y/z. It does not run YOLO,
pick, or RG2.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parents[1]
PICK_DIR = ROOT_DIR / "tools" / "pick"
if str(PICK_DIR) not in sys.path:
    sys.path.insert(0, str(PICK_DIR))

from run_supervised_real_single_cup_pick import (  # noqa: E402
    CONFIRM_PHRASE,
    MoveItExecuteTrajectoryBackend,
    check_services,
    home_joint_target_degrees,
    print_stage,
    validate_motion_gates,
    validate_speed_scales,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", default=True)
    parser.add_argument("--enable-real-motion", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--one-shot", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--planning-group", default="manipulator")
    parser.add_argument("--ee-link", default="link_6")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--planning-timeout-sec", type=float, default=1.0)
    parser.add_argument("--velocity-scale", type=float, default=0.03)
    parser.add_argument("--accel-scale", type=float, default=0.03)
    parser.add_argument("--home-joint-1-deg", type=float, default=0.0)
    parser.add_argument("--home-joint-2-deg", type=float, default=0.0)
    parser.add_argument("--home-joint-3-deg", type=float, default=90.0)
    parser.add_argument("--home-joint-4-deg", type=float, default=0.0)
    parser.add_argument("--home-joint-5-deg", type=float, default=90.0)
    parser.add_argument("--home-joint-6-deg", type=float, default=0.0)
    parser.add_argument("--camera-orient-joint-6-deg", type=float, default=90.0)
    parser.add_argument("--skip-wrist-align", action="store_true")
    parser.add_argument("--camera-home-x", type=float, default=0.45)
    parser.add_argument("--camera-home-y", type=float, default=0.0)
    parser.add_argument("--camera-home-z", type=float, default=0.62)
    parser.add_argument("--retry-z", default="0.62,0.58,0.54")
    parser.add_argument("--execute-action-name", default="/execute_trajectory")
    parser.add_argument("--move-action-name", default="/move_action")
    parser.add_argument("--action-timeout-sec", type=float, default=60.0)
    parser.set_defaults(observe_only=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print_stage("START", "legacy camera_home observation move")
    print(f"real_motion={args.enable_real_motion} one_shot={args.one_shot}")
    print(
        "camera_home_target="
        f"x={args.camera_home_x:.3f} y={args.camera_home_y:.3f} z={args.camera_home_z:.3f}"
    )
    print(f"planning_group={args.planning_group} ee_link={args.ee_link}")

    if not validate_motion_gates(args):
        return 2
    if not validate_speed_scales(args):
        return 2
    if not check_services(args, strict=args.enable_real_motion):
        return 1

    home_target = home_joint_target_degrees(args)
    candidate_zs = camera_home_z_candidates(args.camera_home_z, args.retry_z)
    wrist_target = dict(home_target)
    wrist_target["joint_6"] = float(args.camera_orient_joint_6_deg)
    print_stage("PLAN_HOME_THEN_CAMERA_HOME", "legacy staged move_camera_home target")
    print(f"home_joint_target_deg={json.dumps(home_target, sort_keys=True)}")
    if args.skip_wrist_align:
        print("wrist_align=skipped")
    else:
        print(f"wrist_camera_align_target_deg={json.dumps(wrist_target, sort_keys=True)}")
    print(f"camera_home_xyz_m=({args.camera_home_x:.3f},{args.camera_home_y:.3f},{args.camera_home_z:.3f})")
    print(f"candidate_zs_m={candidate_zs}")
    print(f"motion_backend={args.execute_action_name} [moveit_msgs/action/ExecuteTrajectory]")

    if not args.enable_real_motion:
        print("[DRY-RUN] HOME, wrist camera alignment, and legacy CAMERA_HOME targets were not executed")
        return 0

    executor = MoveItExecuteTrajectoryBackend(args)
    try:
        executor._init_moveit()
        home_trajectory = executor._plan_joint_target("home", home_target)
        executor._execute_trajectory("home", home_trajectory, expected_joint_degrees=home_target)
        executor._wait_for_joint_target("home", home_target)

        if not args.skip_wrist_align:
            wrist_trajectory = executor._plan_joint_target("wrist_camera_align", wrist_target)
            executor._execute_trajectory(
                "wrist_camera_align",
                wrist_trajectory,
                expected_joint_degrees=wrist_target,
            )
            executor._wait_for_joint_target("wrist_camera_align", wrist_target)

        home_orientation = lookup_current_orientation(executor, args.base_frame, args.ee_link)
        print(
            "[INFO] home_ori="
            f"qx={home_orientation['x']:.6f} qy={home_orientation['y']:.6f} "
            f"qz={home_orientation['z']:.6f} qw={home_orientation['w']:.6f}"
        )

        last_error = None
        for index, z in enumerate(candidate_zs):
            if index > 0:
                print(f"[WARN] camera_home planning/execution retry at lower z={z:.3f}")
            pose = {
                "position": {
                    "x": float(args.camera_home_x),
                    "y": float(args.camera_home_y),
                    "z": float(z),
                },
                "orientation": home_orientation,
            }
            try:
                trajectory = executor._plan_pose("camera_home", pose)
                executor._execute_trajectory("camera_home", trajectory)
                print_stage("DONE", "legacy camera_home observation pose reached")
                print("No gripper command was sent. Keep robot/camera fixed before baseline capture.")
                return 0
            except Exception as exc:
                last_error = exc
                print(f"[WARN] camera_home z={z:.3f} failed: {exc}")
        print(f"[FAIL] camera_home execution failed for all z candidates: {last_error}")
        return 1
    except Exception as exc:
        print(f"[FAIL] legacy camera_home move stopped: {exc}")
        return 1
    finally:
        executor.node.destroy_node()
        try:
            if executor.rclpy.ok():
                executor.rclpy.shutdown()
        except Exception:
            pass


def camera_home_z_candidates(camera_home_z: float, retry_z: str) -> list[float]:
    raw_values = [camera_home_z]
    for raw in retry_z.replace(";", ",").split(","):
        stripped = raw.strip()
        if not stripped:
            continue
        raw_values.append(float(stripped))
    candidates: list[float] = []
    for z in raw_values:
        if z > camera_home_z + 1e-6:
            continue
        if all(abs(z - existing) > 1e-6 for existing in candidates):
            candidates.append(float(z))
    return candidates


def lookup_current_orientation(
    executor: MoveItExecuteTrajectoryBackend,
    base_frame: str,
    ee_link: str,
) -> dict[str, float]:
    try:
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener

        buffer = Buffer()
        TransformListener(buffer, executor.node)
        deadline = time.monotonic() + 3.0
        last_error = None
        while time.monotonic() < deadline:
            try:
                transform = buffer.lookup_transform(base_frame, ee_link, Time())
                q = transform.transform.rotation
                return normalize_quaternion(
                    {"x": q.x, "y": q.y, "z": q.z, "w": q.w}
                )
            except Exception as exc:
                last_error = exc
                executor.rclpy.spin_once(executor.node, timeout_sec=0.1)
        print(f"[WARN] TF orientation lookup failed, using MoveIt scene fallback: {last_error}")
    except Exception as exc:
        print(f"[WARN] TF orientation lookup unavailable, using MoveIt scene fallback: {exc}")

    psm = executor.moveit_py.get_planning_scene_monitor()
    with psm.read_only() as scene:
        transform = scene.current_state.get_global_link_transform(ee_link)
    matrix = [[float(value) for value in row] for row in transform]
    return quaternion_from_rotation_matrix(matrix)


def quaternion_from_rotation_matrix(matrix: list[list[float]]) -> dict[str, float]:
    m00, m01, m02 = matrix[0][0], matrix[0][1], matrix[0][2]
    m10, m11, m12 = matrix[1][0], matrix[1][1], matrix[1][2]
    m20, m21, m22 = matrix[2][0], matrix[2][1], matrix[2][2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return normalize_quaternion({"x": qx, "y": qy, "z": qz, "w": qw})


def normalize_quaternion(quaternion: dict[str, Any]) -> dict[str, float]:
    qx = float(quaternion["x"])
    qy = float(quaternion["y"])
    qz = float(quaternion["z"])
    qw = float(quaternion["w"])
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0 or not math.isfinite(norm):
        raise RuntimeError("home orientation quaternion is invalid")
    return {"x": qx / norm, "y": qy / norm, "z": qz / norm, "w": qw / norm}


if __name__ == "__main__":
    raise SystemExit(main())
