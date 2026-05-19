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
import shlex
import subprocess
import sys
from pathlib import Path


ROOT = Path("/home/ssu/Azas")
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


def parse_dispenser_ids(raw: str) -> list[str]:
    values = [item.strip() for item in raw.replace(";", ",").split(",") if item.strip()]
    if not values:
        raise ValueError("at least one dispenser id is required")
    invalid = [value for value in values if value not in DISPENSER_TARGETS]
    if invalid:
        raise ValueError(f"unsupported dispenser id(s): {', '.join(invalid)}; allowed: 1,2,3,4")
    return values


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


def move_and_release_cmd(args: argparse.Namespace, dispenser_id: str) -> list[str]:
    return [
        sys.executable,
        str(MOVE_FRONT_HOLD),
        "--service-prefix",
        args.service_prefix,
        "--dispenser-id",
        dispenser_id,
        "--velocity",
        f"{args.move_velocity:.6f}",
        "--acceleration",
        f"{args.move_acceleration:.6f}",
        "--timeout-sec",
        f"{args.move_timeout_sec:.6f}",
        "--wait-service-sec",
        f"{args.wait_service_sec:.6f}",
        "--verify-target",
        "--verify-timeout-sec",
        f"{args.verify_timeout_sec:.6f}",
        "--target-tolerance-mm",
        f"{args.target_tolerance_mm:.6f}",
        "--compensate-current-tcp",
        "--verify-link6-target",
        "--execute",
        "--confirm",
        FRONT_HOLD_CONFIRM_PHRASE,
    ]


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
        "-p require_tcp_for_taught_posx:=true "
        f"-p target_dispenser:={target_q} "
        "-p move_home_first:=true "
        "-p pre_home_retreat_before_home:=true "
        "-p pre_home_retreat_dx_mm:=-140.0 "
        "-p pre_home_retreat_dy_mm:=0.0 "
        "-p pre_home_retreat_min_z_mm:=0.0 "
        "-p pre_home_retreat_min_current_x_mm:=450.0 "
        "-p pre_home_retreat_velocity:=25.0 "
        "-p pre_home_retreat_acceleration:=35.0 "
        "-p joint1_clearance_before_home:=true "
        "-p joint1_clearance_return_home:=true "
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
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--dispenser-tcp-name", default="GripperDA_v1_jarvis")
    parser.add_argument("--move-velocity", type=float, default=30.0)
    parser.add_argument("--move-acceleration", type=float, default=30.0)
    parser.add_argument("--move-timeout-sec", type=float, default=180.0)
    parser.add_argument("--pick-approach-velocity", type=float, default=15.0)
    parser.add_argument("--pick-approach-acceleration", type=float, default=20.0)
    parser.add_argument("--pick-lift-m", type=float, default=0.100)
    parser.add_argument("--pick-lift-velocity", type=float, default=12.0)
    parser.add_argument("--pick-lift-acceleration", type=float, default=16.0)
    parser.add_argument("--pick-timeout-sec", type=float, default=120.0)
    parser.add_argument("--wait-service-sec", type=float, default=8.0)
    parser.add_argument("--verify-timeout-sec", type=float, default=70.0)
    parser.add_argument("--target-tolerance-mm", type=float, default=15.0)
    parser.add_argument("--gripper-grasp-width-m", type=float, default=0.075)
    parser.add_argument("--gripper-force-n", type=float, default=25.0)
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
    if not args.execute:
        print("[DRY-RUN] --execute not set; sequence plan only, no robot command sent.")

    print("[Azas] Measured dispenser recipe sequence")
    print(f"[Azas] dispenser_ids={','.join(dispenser_ids)}")
    print(f"[Azas] service_prefix={args.service_prefix}")
    print(f"[Azas] dispenser_tcp_name={args.dispenser_tcp_name}")
    print("[Azas] source=existing measured front_hold poses and taught dispenser press poses")

    for index, dispenser_id in enumerate(dispenser_ids, start=1):
        label_prefix = f"recipe {index}/{len(dispenser_ids)} dispenser {dispenser_id}"
        if not args.execute:
            print(f"[PLAN] {label_prefix}: move/release -> press -> re-grasp/lift")
            continue

        rc = run_command(f"{label_prefix}: move cup to front-hold and release", move_and_release_cmd(args, dispenser_id))
        if rc != 0:
            return rc
        rc = run_command(f"{label_prefix}: RG2 full-open release verify", [str(RG2_OPEN)])
        if rc != 0:
            return rc
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
        rc = run_command(f"{label_prefix}: re-grasp cup from front-hold", pick_cmd(args, dispenser_id))
        if rc != 0:
            return rc
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

    print("[PASS] measured dispenser recipe sequence completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
