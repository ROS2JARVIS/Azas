#!/usr/bin/env python3
"""Re-grasp a cup resting at a measured dispenser front-hold pose.

This primitive is the inverse of the panel's measured front-hold release step:
it reuses `front_hold_poses.dispenser_N` from measured_dispenser_collision.yaml,
opens RG2, moves to the measured front-hold pose, soft-grasps the cup, then
lifts straight up from the observed current TCP pose.  It does not ask for or
invent cup coordinates.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Any

import rclpy
from azas_interfaces.srv import SetGripper
from dsr_msgs2.srv import GetCurrentPosj, GetCurrentPosx, MoveWait


ROOT = Path("/home/ssu/Azas")
DEFAULT_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "measured_dispenser_collision.yaml"
MOVE_FRONT_HOLD = ROOT / "tools" / "run" / "move_to_measured_dispenser_front_hold.py"
DIRECT_MOVEL = ROOT / "tools" / "run" / "direct_movel_xyz.py"
DIRECT_MOVEJ = ROOT / "tools" / "run" / "direct_movej_joints.py"
CONFIRM_PHRASE = "ENABLE_PICK_FROM_MEASURED_DISPENSER_FRONT_HOLD"
FRONT_HOLD_CONFIRM_PHRASE = "ENABLE_MEASURED_DISPENSER_FRONT_HOLD"
DIRECT_CONFIRM_PHRASE = "ENABLE_DIRECT_MOVEL"
DR_BASE = 0
MOVEJ_CONFIRM_PHRASE = "ENABLE_DIRECT_MOVEJ"


def service_name(prefix: str, suffix: str) -> str:
    clean_prefix = prefix.strip("/")
    clean_suffix = suffix.strip("/")
    return f"/{clean_prefix}/{clean_suffix}" if clean_prefix else f"/{clean_suffix}"


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


def set_gripper(
    *,
    service: str,
    command: str,
    width_m: float,
    force_n: float,
    timeout_sec: float,
) -> tuple[bool, str]:
    rclpy.init(args=None)
    node = rclpy.create_node("azas_pick_from_dispenser_front_hold_gripper")
    try:
        req = SetGripper.Request()
        req.command = command
        req.width_m = float(width_m)
        req.force_n = float(force_n)
        response = call_service(
            node,
            SetGripper,
            service,
            req,
            timeout_sec=timeout_sec,
            label=f"RG2 {command}",
        )
        return bool(response.success), str(response.message)
    except RuntimeError as exc:
        return False, str(exc)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def current_posx(service_prefix: str, timeout_sec: float) -> list[float]:
    rclpy.init(args=None)
    node = rclpy.create_node("azas_pick_from_dispenser_front_hold_current_posx")
    try:
        req = GetCurrentPosx.Request()
        req.ref = DR_BASE
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
        return [float(value) for value in values[:6]]
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



def current_posj(service_prefix: str, timeout_sec: float) -> list[float]:
    rclpy.init(args=None)
    node = rclpy.create_node("azas_pick_from_dispenser_front_hold_current_posj")
    try:
        response = call_service(
            node,
            GetCurrentPosj,
            service_name(service_prefix, "aux_control/get_current_posj"),
            GetCurrentPosj.Request(),
            timeout_sec=timeout_sec,
            label="GetCurrentPosj",
        )
        if not response.success or len(response.pos) < 6:
            raise RuntimeError("GetCurrentPosj returned success=false or too few joints")
        return [float(value) for value in response.pos[:6]]
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



def wait_for_motion_done(service_prefix: str, timeout_sec: float) -> tuple[bool, str]:
    rclpy.init(args=None)
    node = rclpy.create_node("azas_pick_from_dispenser_front_hold_move_wait")
    try:
        response = call_service(
            node,
            MoveWait,
            service_name(service_prefix, "motion/move_wait"),
            MoveWait.Request(),
            timeout_sec=timeout_sec,
            label="MoveWait",
        )
        success = bool(getattr(response, "success", True))
        return success, str(response)
    except RuntimeError as exc:
        return False, str(exc)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def run_joint1_clearance(args: argparse.Namespace) -> int:
    if args.joint1_clearance_deg <= 0.0:
        print("[Azas] pre-grasp joint_1 clearance disabled")
        return 0
    if not args.execute:
        print(
            "[DRY-RUN] pre-grasp joint_1 clearance would read current joints and move "
            f"joint_1 +{args.joint1_clearance_deg:.1f}deg before front-hold approach."
        )
        return 0
    try:
        joints = current_posj(args.service_prefix, timeout_sec=max(args.wait_service_sec, 0.1))
    except RuntimeError as exc:
        print(f"[FAIL] cannot read current joints for pre-grasp joint_1 clearance: {exc}")
        return 1
    before = joints[0]
    joints[0] = before + args.joint1_clearance_deg
    print(
        "[Azas] pre-grasp joint_1 clearance before front-hold approach: "
        f"current joints preserved; j1 {before:.1f} -> {joints[0]:.1f} deg; "
        f"j2~j6 unchanged={joints[1:]}"
    )
    cmd = [
        sys.executable,
        str(DIRECT_MOVEJ),
        "--service-prefix",
        args.service_prefix,
        "--velocity",
        f"{args.joint1_clearance_velocity:.6f}",
        "--acceleration",
        f"{args.joint1_clearance_acceleration:.6f}",
        "--timeout-sec",
        f"{args.timeout_sec:.6f}",
        "--wait-service-sec",
        f"{args.wait_service_sec:.6f}",
        "--execute",
        "--confirm",
        MOVEJ_CONFIRM_PHRASE,
    ]
    for index, value in enumerate(joints, start=1):
        cmd.extend([f"--j{index}", f"{value:.6f}"])
    return subprocess.run(cmd, cwd=str(ROOT), check=False).returncode

def run_front_hold_move(args: argparse.Namespace) -> int:
    cmd = [
        sys.executable,
        str(MOVE_FRONT_HOLD),
        "--config",
        str(args.config),
        "--service-prefix",
        args.service_prefix,
        "--dispenser-id",
        str(args.dispenser_id),
        "--velocity",
        f"{args.approach_velocity:.6f}",
        "--acceleration",
        f"{args.approach_acceleration:.6f}",
        "--timeout-sec",
        f"{args.timeout_sec:.6f}",
        "--wait-service-sec",
        f"{args.wait_service_sec:.6f}",
        "--verify-timeout-sec",
        f"{args.verify_timeout_sec:.6f}",
        "--target-tolerance-mm",
        f"{args.target_tolerance_mm:.6f}",
        "--compensate-current-tcp",
        "--verify-link6-target",
    ]
    if args.execute:
        cmd.append("--verify-target")
    else:
        cmd.append("--no-verify-target")
    if args.execute and args.precheck_ikin:
        cmd.append("--precheck-ikin")
    if not args.execute:
        cmd.append("--no-precheck-ikin")
    if args.execute:
        cmd.extend(["--execute", "--confirm", FRONT_HOLD_CONFIRM_PHRASE])
    print("[Azas] Move to measured dispenser front-hold for re-grasp")
    sys.stdout.flush()
    return subprocess.run(cmd, cwd=str(ROOT), check=False).returncode


def run_lift_from_current(args: argparse.Namespace) -> int:
    if not args.execute:
        print(
            "[DRY-RUN] post-grasp lift would read current TCP and lift "
            f"{args.lift_m:.3f} m; no robot command sent."
        )
        return 0

    try:
        pose = current_posx(args.service_prefix, timeout_sec=max(args.wait_service_sec, 0.1))
    except RuntimeError as exc:
        print(f"[FAIL] cannot read current TCP before lift: {exc}")
        return 1

    current_x_m = pose[0] / 1000.0
    current_y_m = pose[1] / 1000.0
    current_z_m = pose[2] / 1000.0
    lift_z_m = current_z_m + max(args.lift_m, 0.0)
    print(
        "[Azas] lift after re-grasp from current TCP: "
        f"current_xyz_m=[{current_x_m:.4f}, {current_y_m:.4f}, {current_z_m:.4f}] "
        f"target_z_m={lift_z_m:.4f}"
    )

    cmd = [
        sys.executable,
        str(DIRECT_MOVEL),
        "--service-prefix",
        args.service_prefix,
        "--x",
        f"{current_x_m:.6f}",
        "--y",
        f"{current_y_m:.6f}",
        "--z",
        f"{lift_z_m:.6f}",
        "--use-current-rpy",
        "--velocity",
        f"{args.lift_velocity:.6f}",
        "--acceleration",
        f"{args.lift_acceleration:.6f}",
        "--timeout-sec",
        f"{args.timeout_sec:.6f}",
        "--wait-service-sec",
        f"{args.wait_service_sec:.6f}",
        "--target-tolerance-mm",
        f"{args.target_tolerance_mm:.6f}",
        "--verify-timeout-sec",
        f"{args.verify_timeout_sec:.6f}",
        "--x-min",
        f"{args.x_min:.6f}",
        "--x-max",
        f"{args.x_max:.6f}",
        "--y-min",
        f"{args.y_min:.6f}",
        "--y-max",
        f"{args.y_max:.6f}",
        "--z-min",
        f"{args.z_min:.6f}",
        "--z-max",
        f"{args.z_max:.6f}",
    ]
    if args.precheck_ikin:
        cmd.append("--precheck-ikin")
    if args.execute:
        cmd.extend(["--verify-target", "--execute", "--confirm", DIRECT_CONFIRM_PHRASE])
    return subprocess.run(cmd, cwd=str(ROOT), check=False).returncode


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Open RG2, move to measured dispenser_N front-hold, soft-grasp the cup, "
            "then lift from the current TCP pose."
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--dispenser-id", type=int, default=2, choices=(1, 2, 3, 4))
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--approach-velocity", type=float, default=15.0)
    parser.add_argument("--approach-acceleration", type=float, default=20.0)
    parser.add_argument("--lift-m", type=float, default=0.100)
    parser.add_argument("--lift-velocity", type=float, default=12.0)
    parser.add_argument("--lift-acceleration", type=float, default=16.0)
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--wait-service-sec", type=float, default=8.0)
    parser.add_argument("--verify-timeout-sec", type=float, default=45.0)
    parser.add_argument("--target-tolerance-mm", type=float, default=15.0)
    parser.add_argument("--precheck-ikin", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gripper-service", default="/jarvis/rg2/set_width")
    parser.add_argument("--gripper-open-width-m", type=float, default=0.110)
    parser.add_argument("--gripper-grasp-width-m", type=float, default=0.075)
    parser.add_argument("--gripper-force-n", type=float, default=25.0)
    parser.add_argument("--gripper-timeout-sec", type=float, default=12.0)
    parser.add_argument("--joint1-clearance-deg", type=float, default=12.0)
    parser.add_argument("--joint1-clearance-velocity", type=float, default=20.0)
    parser.add_argument("--joint1-clearance-acceleration", type=float, default=25.0)
    parser.add_argument("--x-min", type=float, default=0.45)
    parser.add_argument("--x-max", type=float, default=0.72)
    parser.add_argument("--y-min", type=float, default=-0.35)
    parser.add_argument("--y-max", type=float, default=0.15)
    parser.add_argument("--z-min", type=float, default=0.05)
    parser.add_argument("--z-max", type=float, default=0.35)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--confirm",
        default="",
        help=f"must equal {CONFIRM_PHRASE} when --execute is used",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.config.is_file():
        print(f"[FAIL] measured dispenser config not found: {args.config}")
        return 2
    if args.execute and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
        return 2
    if args.lift_m <= 0.0:
        print("[BLOCKED] --lift-m must be positive for a safe retreat after grasp")
        return 2

    print("[Azas] Pick cup from measured dispenser front-hold")
    print(f"[Azas] config={args.config}")
    print(f"[Azas] dispenser_id={args.dispenser_id}")
    print(f"[Azas] service_prefix={args.service_prefix}")
    print("[Azas] source=front_hold_poses; no operator/LLM-generated cup coordinates")
    if not args.execute:
        print("[DRY-RUN] --execute not set; no robot or gripper command will be sent.")

    print("[Azas] RG2 full-open before re-grasp approach")
    if args.execute:
        ok, message = set_gripper(
            service=args.gripper_service,
            command="open",
            width_m=args.gripper_open_width_m,
            force_n=args.gripper_force_n,
            timeout_sec=args.gripper_timeout_sec,
        )
        print(f"[Azas] RG2 open response: success={ok} message='{message}'")
        if not ok:
            print("[FAIL] gripper open failed; refusing to approach cup for re-grasp.")
            return 1

    rc = run_joint1_clearance(args)
    if rc != 0:
        print("[FAIL] pre-grasp joint_1 clearance failed; front-hold approach skipped.")
        return rc

    rc = run_front_hold_move(args)
    if rc != 0:
        print("[FAIL] front-hold approach failed; gripper close skipped.")
        return rc

    print("[Azas] RG2 soft side-grasp at dispenser front-hold")
    if args.execute:
        ok, message = set_gripper(
            service=args.gripper_service,
            command="set_width",
            width_m=args.gripper_grasp_width_m,
            force_n=args.gripper_force_n,
            timeout_sec=args.gripper_timeout_sec,
        )
        print(f"[Azas] RG2 grasp response: success={ok} message='{message}'")
        if not ok:
            print("[FAIL] gripper grasp failed; lift skipped to avoid dragging an unsecured cup.")
            return 1
    else:
        print("[DRY-RUN] gripper grasp not sent.")

    rc = run_lift_from_current(args)
    if rc != 0:
        print("[FAIL] post-grasp lift failed.")
        return rc

    print("[PASS] dispenser front-hold cup re-grasp sequence completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
