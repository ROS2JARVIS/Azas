#!/usr/bin/env python3
"""Pick a closed cup back up from the measured cup holder using side grip.

This is the reverse of place_side_grip_cup_in_holder.py for the measured
cup_holder.side_grip_place poses in calibration.yaml. It does not invent cup
coordinates: it reuses the measured active-TCP holder place pose.
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

import yaml


ROOT = Path("/home/ssu/Azas")
DEFAULT_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"
DIRECT_MOVEL = ROOT / "tools" / "run" / "direct_movel_xyz.py"
RG2_SET_WIDTH = ROOT / "tools" / "run" / "rg2_set_width_verify.py"
CONFIRM_PHRASE = "ENABLE_CUP_HOLDER_PICK"
DIRECT_CONFIRM_PHRASE = "ENABLE_DIRECT_MOVEL"


@dataclass(frozen=True)
class TargetPose:
    label: str
    xyz_m: list[float]
    rpy_rad: list[float]

    @property
    def rpy_deg(self) -> list[float]:
        return [math.degrees(value) for value in self.rpy_rad]


def numeric_list(value: Any, label: str, count: int) -> list[float]:
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{label} must be a {count}-number list")
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain only numbers") from exc


def load_target(block: dict[str, Any], name: str) -> TargetPose:
    xyz = numeric_list(block.get(f"{name}_pose_xyz_m"), f"{name}_pose_xyz_m", 3)
    rpy = numeric_list(block.get(f"{name}_pose_rpy_rad"), f"{name}_pose_rpy_rad", 3)
    return TargetPose(name, xyz, rpy)


def load_sequence(config_path: Path) -> tuple[TargetPose, TargetPose, TargetPose, float]:
    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    holder = data.get("cup_holder")
    if not isinstance(holder, dict):
        raise ValueError("cup_holder section is missing in calibration.yaml")
    block = holder.get("side_grip_place")
    if not isinstance(block, dict):
        raise ValueError("cup_holder.side_grip_place section is missing")

    pre_place = load_target(block, "pre_place")
    place_final = load_target(block, "place_final")
    retreat = load_target(block, "retreat")
    approach_lift_m = float(block.get("approach_lift_m", 0.0))
    return pre_place, place_final, retreat, approach_lift_m


def offset_target_z(target: TargetPose, offset_m: float) -> TargetPose:
    adjusted_xyz = list(target.xyz_m)
    adjusted_xyz[2] += float(offset_m)
    return TargetPose(target.label, adjusted_xyz, list(target.rpy_rad))


def print_target(target: TargetPose) -> None:
    rx, ry, rz = target.rpy_deg
    x, y, z = target.xyz_m
    print(
        f"[Azas] {target.label}: xyz_m=[{x:.6f}, {y:.6f}, {z:.6f}] "
        f"doosan_rpy_deg=[{rx:.3f}, {ry:.3f}, {rz:.3f}]"
    )


def run_movel(
    target: TargetPose,
    *,
    args: argparse.Namespace,
    velocity: float,
    acceleration: float,
) -> int:
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
        "--ikin-timeout-sec",
        f"{args.ikin_timeout_sec:.6f}",
        "--ikin-retries",
        str(max(int(args.ikin_retries), 1)),
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
        cmd.extend(["--precheck-ikin", "--verify-target", "--execute", "--confirm", DIRECT_CONFIRM_PHRASE])

    print(f"[Azas] MoveLine step={target.label}")
    sys.stdout.flush()
    return subprocess.run(cmd, cwd=str(ROOT), check=False).returncode


def run_gripper(args: argparse.Namespace, *, command: str, width_m: float, force_n: float) -> int:
    print(f"[Azas] RG2 command={command} width_m={width_m:.3f} force_n={force_n:.1f}")
    if not args.execute:
        print("[DRY-RUN] --execute not set; gripper command not sent.")
        return 0

    env = os.environ.copy()
    env["RG2_SET_WIDTH_SERVICE"] = args.gripper_service
    cmd = [
        sys.executable,
        str(RG2_SET_WIDTH),
        "--service",
        args.gripper_service,
        "--command",
        command,
        "--width-m",
        f"{width_m:.6f}",
        "--force-n",
        f"{force_n:.6f}",
        "--timeout-sec",
        f"{args.gripper_timeout_sec:.3f}",
    ]
    return subprocess.run(cmd, cwd=str(ROOT), env=env, check=False).returncode


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Open RG2, approach measured cup-holder side-grip pose, grasp, then lift."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--approach-velocity", type=float, default=12.0)
    parser.add_argument("--approach-acceleration", type=float, default=16.0)
    parser.add_argument("--descend-velocity", type=float, default=6.0)
    parser.add_argument("--descend-acceleration", type=float, default=10.0)
    parser.add_argument("--lift-velocity", type=float, default=12.0)
    parser.add_argument("--lift-acceleration", type=float, default=16.0)
    parser.add_argument(
        "--place-final-z-offset-m",
        type=float,
        default=0.0,
        help="Same measured adjustment used by place_cup_holder; added only to place_final Z.",
    )
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--wait-service-sec", type=float, default=8.0)
    parser.add_argument("--ikin-timeout-sec", type=float, default=20.0)
    parser.add_argument("--ikin-retries", type=int, default=2)
    parser.add_argument("--verify-timeout-sec", type=float, default=45.0)
    parser.add_argument("--target-tolerance-mm", type=float, default=12.0)
    parser.add_argument("--x-min", type=float, default=0.35)
    parser.add_argument("--x-max", type=float, default=0.50)
    parser.add_argument("--y-min", type=float, default=0.15)
    parser.add_argument("--y-max", type=float, default=0.30)
    parser.add_argument("--z-min", type=float, default=0.08)
    parser.add_argument("--z-max", type=float, default=0.28)
    parser.add_argument("--gripper-service", default="/jarvis/rg2/set_width")
    parser.add_argument("--gripper-open-width-m", type=float, default=0.110)
    parser.add_argument("--gripper-grasp-width-m", type=float, default=0.068)
    parser.add_argument("--gripper-force-n", type=float, default=20.0)
    parser.add_argument("--gripper-timeout-sec", type=float, default=12.0)
    parser.add_argument("--post-grasp-settle-sec", type=float, default=0.8)
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
        print(f"[FAIL] calibration config not found: {args.config}")
        return 2
    if args.execute and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
        return 2

    try:
        pre_place, place_final, retreat, approach_lift_m = load_sequence(args.config)
        if abs(args.place_final_z_offset_m) > 1e-9:
            place_final = offset_target_z(place_final, args.place_final_z_offset_m)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"[FAIL] {exc}")
        return 2

    print("[Azas] Cup holder side-grip pick sequence")
    print(f"[Azas] config={args.config}")
    print(f"[Azas] service_prefix={args.service_prefix}")
    print("[Azas] source=cup_holder.side_grip_place measured poses; no operator/LLM-generated cup coordinates")
    print(f"[Azas] approach_lift_m={approach_lift_m:.3f}")
    print(f"[Azas] place_final_z_offset_m={args.place_final_z_offset_m:.4f}")
    print_target(retreat)
    print_target(place_final)
    print_target(pre_place)
    if not args.execute:
        print("[DRY-RUN] --execute not set; no robot or gripper command will be sent.")

    rc = run_gripper(
        args,
        command="open",
        width_m=args.gripper_open_width_m,
        force_n=args.gripper_force_n,
    )
    if rc != 0:
        print("[FAIL] gripper open failed; holder approach skipped.")
        return rc

    for target, velocity, acceleration in [
        (retreat, args.approach_velocity, args.approach_acceleration),
        (place_final, args.descend_velocity, args.descend_acceleration),
    ]:
        rc = run_movel(target, args=args, velocity=velocity, acceleration=acceleration)
        if rc != 0:
            print(f"[FAIL] {target.label} MoveLine failed; aborting holder pick.")
            return rc

    rc = run_gripper(
        args,
        command="set_width",
        width_m=args.gripper_grasp_width_m,
        force_n=args.gripper_force_n,
    )
    if rc != 0:
        print("[FAIL] gripper grasp failed; lift skipped to avoid dragging an unsecured cup.")
        return rc
    if args.post_grasp_settle_sec > 0:
        print(f"[Azas] post-grasp settle before lift: {args.post_grasp_settle_sec:.2f}s")
        import time
        time.sleep(args.post_grasp_settle_sec)

    rc = run_movel(
        pre_place,
        args=args,
        velocity=args.lift_velocity,
        acceleration=args.lift_acceleration,
    )
    if rc != 0:
        print("[FAIL] post-grasp holder lift failed.")
        return rc

    print("[PASS] cup holder side-grip pick sequence completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
