#!/usr/bin/env python3
"""Close a held lid onto the cup already sitting in the measured cup holder.

This script does not ask for or invent cup coordinates.  It uses either a
measured ``cup_holder.lid_attach`` block from calibration.yaml, or an explicitly
allowed holder-top derived fallback based on existing measured holder data.  The
sequence is:

  pre_attach -> lid/cup-mouth contact -> small press -> joint_6 twist -> RG2 open
  -> optional joint_6 return -> retreat

The joint_6 twist is intentionally joint-space only after the lid is at the cup
mouth/contact pose, matching the physical requirement that the lid meets the cup
opening before wrist rotation.
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from dsr_msgs2.srv import GetCurrentPosj, MoveWait


ROOT = Path("/home/ssu/Azas")
DEFAULT_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"
DIRECT_MOVEL = ROOT / "tools" / "run" / "direct_movel_xyz.py"
DIRECT_MOVEJ = ROOT / "tools" / "run" / "direct_movej_joints.py"
RG2_OPEN = ROOT / "tools" / "run" / "rg2_full_open_verify.sh"
CONFIRM_PHRASE = "ENABLE_CUP_HOLDER_LID_ATTACH"
MOVEL_CONFIRM_PHRASE = "ENABLE_DIRECT_MOVEL"
MOVEJ_CONFIRM_PHRASE = "ENABLE_DIRECT_MOVEJ"


@dataclass(frozen=True)
class TargetPose:
    label: str
    xyz_m: list[float]
    rpy_rad: list[float]

    @property
    def rpy_deg(self) -> list[float]:
        return [math.degrees(value) for value in self.rpy_rad]


def service_name(prefix: str, suffix: str) -> str:
    clean = prefix.strip("/")
    return f"/{clean}/{suffix}" if clean else f"/{suffix}"


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


def load_target(block: dict[str, Any], name: str) -> TargetPose | None:
    xyz = optional_numeric_list(block.get(f"{name}_pose_xyz_m"), f"{name}_pose_xyz_m", 3)
    rpy = optional_numeric_list(block.get(f"{name}_pose_rpy_rad"), f"{name}_pose_rpy_rad", 3)
    if xyz is None or rpy is None:
        return None
    return TargetPose(name, xyz, rpy)


def offset_pose(label: str, base_xyz: list[float], rpy_rad: list[float], *, z_offset_m: float) -> TargetPose:
    return TargetPose(label, [base_xyz[0], base_xyz[1], base_xyz[2] + z_offset_m], list(rpy_rad))


def load_sequence(config_path: Path, args: argparse.Namespace) -> tuple[TargetPose, TargetPose, TargetPose, TargetPose, str]:
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    holder = data.get("cup_holder")
    if not isinstance(holder, dict):
        raise ValueError("cup_holder section is missing in calibration.yaml")

    lid_attach = holder.get("lid_attach")
    if isinstance(lid_attach, dict):
        measured = [
            load_target(lid_attach, "pre_attach"),
            load_target(lid_attach, "contact"),
            load_target(lid_attach, "press"),
            load_target(lid_attach, "retreat"),
        ]
        if all(target is not None for target in measured):
            return measured[0], measured[1], measured[2], measured[3], "cup_holder.lid_attach measured poses"

    if not args.allow_estimated_holder_top:
        raise ValueError(
            "cup_holder.lid_attach measured poses are missing. Refusing real lid attach without measured "
            "pre_attach/contact/press/retreat poses. For supervised tuning only, pass "
            "--allow-estimated-holder-top to derive from existing cup_holder.top_center_estimated_xyz_m."
        )

    top_xyz = numeric_list(
        holder.get("top_center_estimated_xyz_m"),
        "cup_holder.top_center_estimated_xyz_m",
        3,
    )
    rpy_rad: list[float] | None = None
    if isinstance(lid_attach, dict):
        rpy_rad = optional_numeric_list(lid_attach.get("rpy_rad"), "cup_holder.lid_attach.rpy_rad", 3)
    if rpy_rad is None:
        side_place = holder.get("side_grip_place")
        if not isinstance(side_place, dict):
            raise ValueError("cup_holder.side_grip_place is missing; cannot derive lid attach orientation")
        rpy_rad = numeric_list(
            side_place.get("place_final_pose_rpy_rad"),
            "cup_holder.side_grip_place.place_final_pose_rpy_rad",
            3,
        )

    contact = offset_pose("contact", top_xyz, rpy_rad, z_offset_m=args.contact_z_offset_m)
    pre_attach = offset_pose(
        "pre_attach",
        contact.xyz_m,
        rpy_rad,
        z_offset_m=args.pre_attach_lift_m,
    )
    press = offset_pose("press", top_xyz, rpy_rad, z_offset_m=args.press_z_offset_m)
    retreat = offset_pose("retreat", pre_attach.xyz_m, rpy_rad, z_offset_m=args.retreat_extra_z_m)
    return pre_attach, contact, press, retreat, "derived from existing cup_holder.top_center_estimated_xyz_m"


def print_target(target: TargetPose) -> None:
    rx, ry, rz = target.rpy_deg
    x, y, z = target.xyz_m
    print(
        f"[Azas] {target.label}: xyz_m=[{x:.6f}, {y:.6f}, {z:.6f}] "
        f"doosan_rpy_deg=[{rx:.3f}, {ry:.3f}, {rz:.3f}]"
    )


def call_service(node: Any, srv_type: Any, name: str, request: Any, *, timeout_sec: float, label: str) -> Any:
    client = node.create_client(srv_type, name)
    if not client.wait_for_service(timeout_sec=max(timeout_sec, 0.1)):
        raise RuntimeError(f"{label} service not available: {name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=max(timeout_sec, 0.1))
    if not future.done():
        raise RuntimeError(f"{label} response timeout after {timeout_sec:.1f}s")
    if future.exception() is not None:
        raise RuntimeError(f"{label} exception: {future.exception()}")
    response = future.result()
    if response is None:
        raise RuntimeError(f"{label} returned no response")
    return response


def current_posj(service_prefix: str, timeout_sec: float) -> list[float]:
    rclpy.init(args=None)
    node = rclpy.create_node("azas_cup_holder_lid_attach_current_posj")
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
    node = rclpy.create_node("azas_cup_holder_lid_attach_move_wait")
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


def run_movel(target: TargetPose, *, args: argparse.Namespace, velocity: float, acceleration: float) -> int:
    rx, ry, rz = target.rpy_deg
    cmd = [
        sys.executable,
        str(DIRECT_MOVEL),
        "--service-prefix",
        args.service_prefix,
        "--x",
        f"{target.xyz_m[0]:.6f}",
        "--y",
        f"{target.xyz_m[1]:.6f}",
        "--z",
        f"{target.xyz_m[2]:.6f}",
        "--rx",
        f"{rx:.6f}",
        "--ry",
        f"{ry:.6f}",
        "--rz",
        f"{rz:.6f}",
        "--velocity",
        f"{velocity:.6f}",
        "--acceleration",
        f"{acceleration:.6f}",
        "--timeout-sec",
        f"{args.timeout_sec:.6f}",
        "--wait-service-sec",
        f"{args.wait_service_sec:.6f}",
        "--verify-timeout-sec",
        f"{args.verify_timeout_sec:.6f}",
        "--target-tolerance-mm",
        f"{args.target_tolerance_mm:.6f}",
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
    if args.execute:
        cmd.extend(["--precheck-ikin", "--verify-target", "--execute", "--confirm", MOVEL_CONFIRM_PHRASE])
    print(f"[Azas] MoveLine step={target.label}")
    sys.stdout.flush()
    return subprocess.run(cmd, cwd=str(ROOT), check=False).returncode


def run_movej(joints_deg: list[float], *, args: argparse.Namespace, label: str) -> int:
    cmd = [
        sys.executable,
        str(DIRECT_MOVEJ),
        "--service-prefix",
        args.service_prefix,
        "--velocity",
        f"{args.twist_velocity:.6f}",
        "--acceleration",
        f"{args.twist_acceleration:.6f}",
        "--timeout-sec",
        f"{args.timeout_sec:.6f}",
        "--wait-service-sec",
        f"{args.wait_service_sec:.6f}",
    ]
    if args.execute:
        cmd.extend(["--execute", "--confirm", MOVEJ_CONFIRM_PHRASE])
    for index, value in enumerate(joints_deg, start=1):
        cmd.extend([f"--j{index}", f"{value:.6f}"])
    print(f"[Azas] MoveJoint step={label}: j6={joints_deg[5]:.2f}deg")
    sys.stdout.flush()
    rc = subprocess.run(cmd, cwd=str(ROOT), check=False).returncode
    if rc == 0 and args.execute:
        done, output = wait_for_motion_done(args.service_prefix, args.move_wait_timeout_sec)
        print(f"[Azas] move_wait after {label}: success={done} {output}")
        if not done:
            return 1
    return rc


def run_gripper_open(args: argparse.Namespace) -> int:
    print("[Azas] RG2 full open to release lid after joint_6 twist")
    if not args.execute:
        print("[DRY-RUN] --execute not set; gripper open not sent.")
        return 0
    env = os.environ.copy()
    env["RG2_SET_WIDTH_SERVICE"] = args.gripper_service
    env["RG2_FULL_OPEN_WIDTH_M"] = f"{args.gripper_open_width_m:.6f}"
    env["RG2_OPEN_FORCE_N"] = f"{args.gripper_open_force_n:.6f}"
    env["RG2_OPEN_TIMEOUT_SEC"] = f"{args.gripper_timeout_sec:.3f}"
    return subprocess.run([str(RG2_OPEN)], cwd=str(ROOT), env=env, check=False).returncode


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Close the already-gripped lid onto the cup already sitting in the measured cup holder.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--allow-estimated-holder-top", action="store_true", help="derive supervised tuning poses from existing cup_holder.top_center_estimated_xyz_m when measured lid_attach poses are absent")
    parser.add_argument("--pre-attach-lift-m", type=float, default=0.060)
    parser.add_argument("--contact-z-offset-m", type=float, default=0.040, help="Z above holder top center where lid first meets cup mouth")
    parser.add_argument("--press-z-offset-m", type=float, default=0.025, help="Z above holder top center after light press before twisting")
    parser.add_argument("--retreat-extra-z-m", type=float, default=0.000)
    parser.add_argument("--approach-velocity", type=float, default=10.0)
    parser.add_argument("--approach-acceleration", type=float, default=14.0)
    parser.add_argument("--press-velocity", type=float, default=4.0)
    parser.add_argument("--press-acceleration", type=float, default=8.0)
    parser.add_argument("--retreat-velocity", type=float, default=10.0)
    parser.add_argument("--retreat-acceleration", type=float, default=14.0)
    parser.add_argument("--twist-j6-deg", type=float, default=45.0)
    parser.add_argument("--twist-velocity", type=float, default=8.0)
    parser.add_argument("--twist-acceleration", type=float, default=12.0)
    parser.add_argument("--return-j6-after-release", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--wait-service-sec", type=float, default=8.0)
    parser.add_argument("--move-wait-timeout-sec", type=float, default=20.0)
    parser.add_argument("--verify-timeout-sec", type=float, default=35.0)
    parser.add_argument("--target-tolerance-mm", type=float, default=12.0)
    parser.add_argument("--x-min", type=float, default=0.35)
    parser.add_argument("--x-max", type=float, default=0.50)
    parser.add_argument("--y-min", type=float, default=0.15)
    parser.add_argument("--y-max", type=float, default=0.30)
    parser.add_argument("--z-min", type=float, default=0.07)
    parser.add_argument("--z-max", type=float, default=0.30)
    parser.add_argument("--gripper-service", default="/jarvis/rg2/set_width")
    parser.add_argument("--gripper-open-width-m", type=float, default=0.110)
    parser.add_argument("--gripper-open-force-n", type=float, default=12.0)
    parser.add_argument("--gripper-timeout-sec", type=float, default=12.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} when --execute is used")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.config.is_file():
        print(f"[FAIL] calibration config not found: {args.config}")
        return 2
    if args.execute and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
        return 2
    if abs(args.twist_j6_deg) > 90.0:
        print("[BLOCKED] --twist-j6-deg must stay within +/-90deg for supervised lid attach")
        return 2

    try:
        pre_attach, contact, press, retreat, source = load_sequence(args.config, args)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"[FAIL] {exc}")
        return 2

    print("[Azas] Cup-holder lid close sequence")
    print(f"[Azas] config={args.config}")
    print(f"[Azas] service_prefix={args.service_prefix}")
    print("[Azas] precondition=lid is already held by the gripper; cup is already in the cup holder")
    print(f"[Azas] source={source}; no operator/LLM-generated cup coordinates")
    print(f"[Azas] twist_j6_deg={args.twist_j6_deg:.1f}, return_j6_after_release={args.return_j6_after_release}")
    for target in [pre_attach, contact, press, retreat]:
        print_target(target)
    if not args.execute:
        print("[DRY-RUN] --execute not set; no robot or gripper command will be sent.")

    for target, velocity, acceleration in [
        (pre_attach, args.approach_velocity, args.approach_acceleration),
        (contact, args.approach_velocity, args.approach_acceleration),
        (press, args.press_velocity, args.press_acceleration),
    ]:
        rc = run_movel(target, args=args, velocity=velocity, acceleration=acceleration)
        if rc != 0:
            print(f"[FAIL] {target.label} MoveLine failed; aborting lid attach.")
            return rc

    if not args.execute:
        print(
            "[DRY-RUN] joint_6 twist would read current joints at the pressed/contact pose, "
            f"then command current_j6 + {args.twist_j6_deg:.1f}deg before releasing the lid."
        )
        base_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    else:
        try:
            base_joints = current_posj(args.service_prefix, timeout_sec=max(args.wait_service_sec, 0.1))
        except RuntimeError as exc:
            print(f"[FAIL] cannot read current joints before joint_6 twist: {exc}")
            return 1
        twisted_joints = list(base_joints)
        twisted_joints[5] = base_joints[5] + args.twist_j6_deg
        rc = run_movej(twisted_joints, args=args, label="lid joint_6 twist at cup mouth")
        if rc != 0:
            print("[FAIL] joint_6 twist failed; gripper release skipped.")
            return rc

    rc = run_gripper_open(args)
    if rc != 0:
        print("[FAIL] gripper open failed; retreat skipped to avoid dragging an unsecured lid.")
        return rc

    if args.return_j6_after_release:
        if not args.execute:
            print("[DRY-RUN] joint_6 would return to the pre-twist joint value after RG2 opens.")
        else:
            rc = run_movej(base_joints, args=args, label="joint_6 return after lid release")
            if rc != 0:
                print("[FAIL] joint_6 return failed; retreat skipped.")
                return rc

    rc = run_movel(retreat, args=args, velocity=args.retreat_velocity, acceleration=args.retreat_acceleration)
    if rc != 0:
        print("[FAIL] retreat MoveLine failed.")
        return rc

    print("[PASS] cup-holder lid attach sequence completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
