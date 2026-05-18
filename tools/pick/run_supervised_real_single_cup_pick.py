#!/usr/bin/env python3
"""Supervised one-shot real cup side-pick gate.

This script prepares exactly one cup side pick from the live base_link tumbler
pose pipeline. It refuses real motion unless the explicit real-motion gates are
present. When enabled, it plans approach/grasp/lift with MoveItPy and sends only
successful plan trajectories to MoveIt ExecuteTrajectory.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parents[1]
TOOLS_DIR = ROOT_DIR / "tools"
SRC_MOTION = ROOT_DIR / "src" / "azas_motion"
for path in (SCRIPT_DIR, TOOLS_DIR, SRC_MOTION):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

CONFIRM_PHRASE = "I_UNDERSTAND_THIS_WILL_MOVE_THE_ROBOT"
DEFAULT_SWEEP_JSON = Path("/tmp/azas_side_grasp_candidate_sweep.json")
DEFAULT_SWEEP_CSV = Path("/tmp/azas_side_grasp_candidate_sweep.csv")
MOVEIT_SUCCESS = 1
MIN_JOINT_MOTION_RAD = 0.002
JOINT_TARGET_TOLERANCE_RAD = 0.03


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", default=True)
    parser.add_argument("--enable-real-motion", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--one-shot", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-live-pose", type=str_to_bool, default=True)
    parser.add_argument("--tumbler-pose-topic", default="/jarvis/tumbler_dispenser/tumbler_pose")
    parser.add_argument("--planning-group", default="manipulator")
    parser.add_argument("--ee-link", default="tool0")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--max-candidates", type=int, default=300)
    parser.add_argument("--planning-timeout-sec", type=float, default=1.0)
    parser.add_argument("--velocity-scale", type=float, default=0.03)
    parser.add_argument("--accel-scale", type=float, default=0.03)
    parser.add_argument("--skip-home", action="store_true")
    parser.add_argument("--home-joint-1-deg", type=float, default=0.0)
    parser.add_argument("--home-joint-2-deg", type=float, default=0.0)
    parser.add_argument("--home-joint-3-deg", type=float, default=90.0)
    parser.add_argument("--home-joint-4-deg", type=float, default=0.0)
    parser.add_argument("--home-joint-5-deg", type=float, default=90.0)
    parser.add_argument("--home-joint-6-deg", type=float, default=0.0)
    parser.add_argument("--observe-mode", choices=("joint", "pose"), default="joint")
    parser.add_argument("--observe-joint-1-deg", type=float, default=0.0)
    parser.add_argument("--observe-joint-2-deg", type=float, default=25.0)
    parser.add_argument("--observe-joint-3-deg", type=float, default=95.0)
    parser.add_argument("--observe-joint-4-deg", type=float, default=0.0)
    parser.add_argument("--observe-joint-5-deg", type=float, default=125.0)
    parser.add_argument("--observe-joint-6-deg", type=float, default=0.0)
    parser.add_argument("--observe-x", type=float, default=0.35)
    parser.add_argument("--observe-y", type=float, default=-0.25)
    parser.add_argument("--observe-z", type=float, default=0.45)
    parser.add_argument("--observe-qx", type=float, default=0.0)
    parser.add_argument("--observe-qy", type=float, default=0.0)
    parser.add_argument("--observe-qz", type=float, default=0.0)
    parser.add_argument("--observe-qw", type=float, default=1.0)
    parser.add_argument("--skip-observe", action="store_true")
    parser.add_argument("--observe-only", action="store_true")
    parser.add_argument("--gripper-open-service", default="/jarvis/rg2/open")
    parser.add_argument("--gripper-close-service", default="/jarvis/rg2/close")
    parser.add_argument("--execute-action-name", default="/execute_trajectory")
    parser.add_argument("--move-action-name", default="/move_action")
    parser.add_argument("--action-timeout-sec", type=float, default=60.0)
    parser.add_argument("--sweep-json", type=Path, default=DEFAULT_SWEEP_JSON)
    parser.add_argument("--sweep-csv", type=Path, default=DEFAULT_SWEEP_CSV)
    parser.add_argument("--skip-sweep", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print_stage("START", "supervised one-shot cup side pick")
    print(f"real_motion={args.enable_real_motion} one_shot={args.one_shot}")

    if not validate_motion_gates(args):
        return 2
    if args.skip_observe and args.observe_only:
        print("[FAIL] --skip-observe and --observe-only are mutually exclusive")
        return 2
    if not validate_speed_scales(args):
        return 2
    if not check_services(args, strict=args.enable_real_motion):
        return 1

    if not args.skip_home and not args.skip_observe and args.observe_mode == "joint":
        if not run_home_then_observe(args):
            return 1
        if args.observe_only:
            print_stage("DONE", "observe-only stop before WAIT_OR_READ_CUP_POSE")
            print("No gripper command was sent.")
            return 0
    else:
        if not args.skip_home:
            if not run_home_pose(args):
                return 1
        else:
            print_stage("SKIP_HOME", "operator requested observe/pick without HOME baseline")

        if not args.skip_observe:
            if not run_observe_cup_pose(args):
                return 1
            if args.observe_only:
                print_stage("DONE", "observe-only stop before WAIT_OR_READ_CUP_POSE")
                print("No gripper command was sent.")
                return 0
        else:
            print_stage("SKIP_OBSERVE_CUP_POSE", "operator requested cup pose flow without observe motion")

    cup_pose = read_cup_pose(args)
    if cup_pose is None:
        return 1

    best = sweep_and_select(args, cup_pose, run=not args.skip_sweep and not dry_run_only(args))
    if best is None:
        return 1

    print_pick_poses(best)
    print_confirm_summary(args, cup_pose, best)
    print("[INFO] Waiting 5 seconds before the execution gate.")
    time.sleep(5.0)

    if not args.enable_real_motion:
        print_stage("DONE", "dry-run stop before GRIPPER_OPEN")
        print("No real robot motion or RG2 command was sent.")
        return 0

    return execute_one_pick(args, best)


def str_to_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid bool value: {value!r}")


def print_stage(name: str, detail: str) -> None:
    print(f"\n=== {name} ===")
    print(detail)


def validate_motion_gates(args: argparse.Namespace) -> bool:
    print_stage("SAFETY_GATE", "checking real-motion flags")
    if not args.one_shot:
        print("[FAIL] --one-shot must remain true; repeated pick is not allowed")
        return False
    if not args.enable_real_motion:
        print("[OK] dry-run mode; real motion disabled")
        return True
    if args.confirm != CONFIRM_PHRASE:
        print(f"[FAIL] real motion requires --confirm {CONFIRM_PHRASE}")
        print("No real robot motion or RG2 command was sent.")
        return False
    print("[OK] real-motion confirmation phrase accepted")
    return True


def validate_speed_scales(args: argparse.Namespace) -> bool:
    ok = 0.0 < args.velocity_scale <= 0.05 and 0.0 < args.accel_scale <= 0.05
    status = "OK" if ok else "FAIL"
    print(f"[{status}] velocity_scale={args.velocity_scale:.3f} accel_scale={args.accel_scale:.3f}")
    return ok


def check_services(args: argparse.Namespace, strict: bool) -> bool:
    print_stage("CHECK_SERVICES", "checking RG2 services, MoveIt planning, and execution actions")
    checks = [("MoveIt plan", "/plan_kinematic_path")]
    if args.observe_only:
        print("[INFO] observe-only mode; gripper services are not required and will not be called")
    else:
        checks = [
            ("gripper open", args.gripper_open_service),
            ("gripper close", args.gripper_close_service),
            *checks,
        ]
    service_list = command_output(["ros2", "service", "list"], timeout=5.0)
    if strict and not service_list:
        service_list = "\n".join(rclpy_graph_names("services"))
    ok = True
    for label, name in checks:
        found = name in service_list
        status = "OK" if found else ("FAIL" if strict else "WARN")
        print(f"[{status}] {label}: {name}")
        ok = ok and (found or not strict)
    move_group_nodes = command_output(["ros2", "node", "list", "--no-daemon"], timeout=5.0)
    if strict and not move_group_nodes:
        move_group_nodes = "\n".join(rclpy_graph_names("nodes"))
    move_group_found = "/move_group" in move_group_nodes
    status = "OK" if move_group_found else "WARN"
    print(f"[{status}] move_group node: /move_group")
    action_text = command_output(["ros2", "action", "list", "-t"], timeout=5.0)
    if strict and not action_text:
        action_text = "\n".join(rclpy_action_names_and_types())
    execute_action_ok = f"{args.execute_action_name} [moveit_msgs/action/ExecuteTrajectory]" in action_text
    move_action_ok = f"{args.move_action_name} [moveit_msgs/action/MoveGroup]" in action_text
    execute_status = "OK" if execute_action_ok else ("FAIL" if strict else "WARN")
    move_status = "OK" if move_action_ok else "INFO"
    print(f"[{execute_status}] execute action: {args.execute_action_name} [moveit_msgs/action/ExecuteTrajectory]")
    print(f"[{move_status}] move action: {args.move_action_name} [moveit_msgs/action/MoveGroup]")
    controller_ok = True
    if strict:
        controller_ok = check_real_motion_backend_ready()
    return (
        ok
        and (execute_action_ok or not strict)
        and controller_ok
    )


def check_real_motion_backend_ready() -> bool:
    print_stage("CHECK_REAL_MOTION_BACKEND", "ros2_control and Doosan system service gates")
    controllers = command_output(["ros2", "control", "list_controllers"], timeout=5.0)
    hardware = command_output(["ros2", "control", "list_hardware_components"], timeout=5.0)
    services = command_output(["ros2", "service", "list"], timeout=5.0)
    ok = True
    if "dsr_moveit_controller" in controllers and "active" in controllers:
        print("[OK] dsr_moveit_controller is active")
    else:
        print("[FAIL] dsr_moveit_controller is not active or controller_manager is not responding")
        ok = False
    if "m0609" in hardware and "active" in hardware:
        print("[OK] m0609 hardware component is active")
    else:
        print("[FAIL] m0609 hardware component is not active or controller_manager is not responding")
        ok = False
    required_system_services = [
        "/system/get_robot_state",
        "/system/get_last_alarm",
        "/motion/move_line",
    ]
    for service_name in required_system_services:
        if service_name in services:
            print(f"[OK] Doosan service present: {service_name}")
        else:
            print(f"[FAIL] Doosan service missing: {service_name}")
            ok = False
    robot_state = read_robot_state()
    if robot_state is None:
        print("[WARN] could not read Doosan robot_state")
    else:
        print(f"[INFO] Doosan robot_state={robot_state}")
        if robot_state not in {1, 2}:
            print("[FAIL] Doosan robot is not in STANDBY/MOVING-capable state; real motion is blocked")
            ok = False
    alarm = read_last_alarm()
    if alarm is not None:
        level = int(alarm.get("level", 0))
        group = alarm.get("group")
        index = alarm.get("index")
        print(f"[INFO] last Doosan alarm: level={level} group={group} index={index}")
        if level >= 2 and robot_state not in {1, 2}:
            print("[FAIL] Doosan controller is not recovered from the reported alarm; real motion is blocked")
            ok = False
    return ok


def read_robot_state() -> int | None:
    try:
        from dsr_msgs2.srv import GetRobotState
        import rclpy

        initialized_here = False
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True
        node = rclpy.create_node("azas_single_pick_robot_state_check")
        try:
            client = node.create_client(GetRobotState, "/system/get_robot_state")
            if not client.wait_for_service(timeout_sec=2.0):
                return None
            future = client.call_async(GetRobotState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            response = future.result()
            if response is None or not response.success:
                return None
            return int(response.robot_state)
        finally:
            node.destroy_node()
            if initialized_here and rclpy.ok():
                rclpy.shutdown()
    except Exception as exc:
        print(f"[WARN] could not read Doosan robot state: {exc}")
        return None


def read_last_alarm() -> dict[str, int] | None:
    try:
        from dsr_msgs2.srv import GetLastAlarm
        import rclpy

        initialized_here = False
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True
        node = rclpy.create_node("azas_single_pick_alarm_check")
        try:
            client = node.create_client(GetLastAlarm, "/system/get_last_alarm")
            if not client.wait_for_service(timeout_sec=2.0):
                return None
            future = client.call_async(GetLastAlarm.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            response = future.result()
            if response is None or not response.success:
                return None
            return {
                "level": int(response.log_alarm.level),
                "group": int(response.log_alarm.group),
                "index": int(response.log_alarm.index),
            }
        finally:
            node.destroy_node()
            if initialized_here and rclpy.ok():
                rclpy.shutdown()
    except Exception as exc:
        print(f"[WARN] could not read Doosan last alarm: {exc}")
        return None


def run_home_pose(args: argparse.Namespace) -> bool:
    print_stage("PLAN_HOME", "joint HOME baseline target")
    home_target = home_joint_target_degrees(args)
    print(f"home_joint_target_deg={json.dumps(home_target, sort_keys=True)}")
    print(f"motion_backend={args.execute_action_name} [moveit_msgs/action/ExecuteTrajectory]")
    print(f"speed_scale={args.velocity_scale:.3f} accel_scale={args.accel_scale:.3f}")
    if not args.enable_real_motion:
        print("[DRY-RUN] HOME target was not executed")
        return True
    try:
        executor = MoveItExecuteTrajectoryBackend(args)
        executor.execute_joint_target("home", home_target)
    except Exception as exc:
        print(f"[FAIL] HOME execution stopped: {exc}")
        return False
    return True


def run_home_then_observe(args: argparse.Namespace) -> bool:
    print_stage("PLAN_HOME_THEN_OBSERVE", "joint HOME baseline followed by joint observe target")
    home_target = home_joint_target_degrees(args)
    observe_target = observe_joint_target_degrees(args)
    print(f"home_joint_target_deg={json.dumps(home_target, sort_keys=True)}")
    print(f"observe_joint_target_deg={json.dumps(observe_target, sort_keys=True)}")
    print(f"motion_backend={args.execute_action_name} [moveit_msgs/action/ExecuteTrajectory]")
    print(f"speed_scale={args.velocity_scale:.3f} accel_scale={args.accel_scale:.3f}")
    if not args.enable_real_motion:
        print("[DRY-RUN] HOME and OBSERVE targets were not executed")
        return True
    try:
        executor = MoveItExecuteTrajectoryBackend(args)
        executor.execute_joint_sequence(
            [
                ("home", home_target),
                ("observe_cup_pose", observe_target),
            ]
        )
    except Exception as exc:
        print(f"[FAIL] HOME->OBSERVE execution stopped: {exc}")
        return False
    return True


def run_observe_cup_pose(args: argparse.Namespace) -> bool:
    print_stage("PLAN_OBSERVE_CUP_POSE", f"{args.observe_mode} observe target")
    print(f"motion_backend={args.execute_action_name} [moveit_msgs/action/ExecuteTrajectory]")
    print(f"speed_scale={args.velocity_scale:.3f} accel_scale={args.accel_scale:.3f}")
    if args.observe_mode == "joint":
        observe_target = observe_joint_target_degrees(args)
        print(f"observe_joint_target_deg={json.dumps(observe_target, sort_keys=True)}")
    else:
        observe_target = observe_pose_dict(args)
        print(f"observe_pose={json.dumps(observe_target, sort_keys=True)}")
        print(f"frame_id={args.base_frame}")
    if args.observe_mode == "pose" and args.base_frame != "base_link":
        print(f"[FAIL] observe pose frame must be base_link, got {args.base_frame!r}")
        return False
    if not args.enable_real_motion:
        print("[DRY-RUN] observe target was not executed")
        return True
    try:
        executor = MoveItExecuteTrajectoryBackend(args)
        executor.execute_observe(observe_target)
    except Exception as exc:
        print(f"[FAIL] observe pose execution stopped: {exc}")
        return False
    return True


def home_joint_target_degrees(args: argparse.Namespace) -> dict[str, float]:
    return {
        "joint_1": float(args.home_joint_1_deg),
        "joint_2": float(args.home_joint_2_deg),
        "joint_3": float(args.home_joint_3_deg),
        "joint_4": float(args.home_joint_4_deg),
        "joint_5": float(args.home_joint_5_deg),
        "joint_6": float(args.home_joint_6_deg),
    }


def observe_joint_target_degrees(args: argparse.Namespace) -> dict[str, float]:
    return {
        "joint_1": float(args.observe_joint_1_deg),
        "joint_2": float(args.observe_joint_2_deg),
        "joint_3": float(args.observe_joint_3_deg),
        "joint_4": float(args.observe_joint_4_deg),
        "joint_5": float(args.observe_joint_5_deg),
        "joint_6": float(args.observe_joint_6_deg),
    }


def observe_pose_dict(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "position": {
            "x": float(args.observe_x),
            "y": float(args.observe_y),
            "z": float(args.observe_z),
        },
        "orientation": {
            "x": float(args.observe_qx),
            "y": float(args.observe_qy),
            "z": float(args.observe_qz),
            "w": float(args.observe_qw),
        },
    }


def rclpy_graph_names(kind: str) -> list[str]:
    import rclpy

    initialized_here = False
    if not rclpy.ok():
        rclpy.init(args=None)
        initialized_here = True
    node = rclpy.create_node("azas_single_pick_graph_check")
    try:
        if kind == "services":
            return [name for name, _types in node.get_service_names_and_types()]
        if kind == "nodes":
            return list(node.get_node_names())
        raise ValueError(f"unsupported graph kind: {kind}")
    finally:
        node.destroy_node()
        if initialized_here and rclpy.ok():
            rclpy.shutdown()


def rclpy_action_names_and_types() -> list[str]:
    import rclpy

    initialized_here = False
    if not rclpy.ok():
        rclpy.init(args=None)
        initialized_here = True
    node = rclpy.create_node("azas_single_pick_action_check")
    try:
        return [
            f"{name} [{type_name}]"
            for name, type_names in node.get_action_names_and_types()
            for type_name in type_names
        ]
    finally:
        node.destroy_node()
        if initialized_here and rclpy.ok():
            rclpy.shutdown()


def read_cup_pose(args: argparse.Namespace) -> dict[str, float] | None:
    print_stage("WAIT_OR_READ_CUP_POSE", "base_link pose only")
    if not args.use_live_pose:
        print("[FAIL] this real/supervised entrypoint only accepts live tumbler pose")
        return None
    pose = read_live_pose(args.tumbler_pose_topic)
    if pose is None:
        return None
    print(f"[OK] live pose: {pose}")
    return pose


def read_live_pose(topic: str) -> dict[str, float] | None:
    result = run_command(["ros2", "topic", "echo", "--once", topic], timeout=8.0, capture=True)
    if result.returncode != 0:
        print(f"[FAIL] timed out or failed reading live pose topic: {topic}")
        return None
    frame_id = extract_scalar(result.stdout, "frame_id:")
    if frame_id != "base_link":
        print(f"[FAIL] live pose frame_id must be base_link, got {frame_id!r}")
        return None
    values = extract_position_xyz(result.stdout)
    if values is None:
        print("[FAIL] could not parse live pose position")
        return None
    return {"frame_id": frame_id, "x": values[0], "y": values[1], "z": values[2]}


def extract_scalar(text: str, prefix: str) -> str | None:
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith(prefix):
            return stripped.split(":", 1)[1].strip().strip("'\"")
    return None


def extract_position_xyz(text: str) -> tuple[float, float, float] | None:
    in_position = False
    values: dict[str, float] = {}
    for line in text.splitlines():
        stripped = line.strip()
        if stripped == "position:":
            in_position = True
            continue
        if stripped == "orientation:":
            in_position = False
        if in_position and any(stripped.startswith(f"{axis}:") for axis in ("x", "y", "z")):
            axis, raw = stripped.split(":", 1)
            values[axis] = float(raw.strip())
    if {"x", "y", "z"} <= values.keys():
        return values["x"], values["y"], values["z"]
    return None


def sweep_and_select(
    args: argparse.Namespace,
    cup_pose: dict[str, float],
    run: bool,
) -> dict[str, Any] | None:
    print_stage("SWEEP_SIDE_GRASP_CANDIDATES", "planning-only all_success candidates required")
    command = [
        sys.executable,
        str(ROOT_DIR / "tools" / "pick" / "sweep_side_grasp_planning_candidates.py"),
        "--planning-group",
        args.planning_group,
        "--ee-link",
        args.ee_link,
        "--base-frame",
        args.base_frame,
        "--cup-reference-x",
        str(cup_pose["x"]),
        "--cup-reference-y",
        str(cup_pose["y"]),
        "--cup-reference-z",
        str(cup_pose["z"]),
        "--planning-timeout-sec",
        str(args.planning_timeout_sec),
        "--max-candidates",
        str(args.max_candidates),
        "--json-output",
        str(args.sweep_json),
        "--csv-output",
        str(args.sweep_csv),
    ]
    if run:
        result = run_command(command, timeout=240.0, capture=False)
        if result.returncode != 0:
            print("[FAIL] candidate sweep failed; execution prohibited")
            return None
    else:
        print("[DRY-RUN] would run:", " ".join(command))

    best = load_best_candidate(args.sweep_json)
    if best is None:
        if args.enable_real_motion:
            print("[FAIL] all_success candidate missing; execution prohibited")
            return None
        print("[DRY-RUN] no all_success sweep output found; using preview candidate only")
        return preview_candidate(cup_pose)
    print(f"[OK] selected all_success candidate_id={best.get('candidate_id')}")
    return best


def load_best_candidate(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    results = json.loads(path.read_text(encoding="utf-8"))
    successes = [item for item in results if item.get("all_success")]
    if not successes:
        return None
    return sorted(
        successes,
        key=lambda item: (
            abs(float(item.get("height", 999.0)) - 0.06),
            -float(item.get("approach_distance_m", 0.0)),
            {"-x": 0, "+x": 1, "-y": 2, "+y": 3}.get(str(item.get("axis")), 99),
        ),
    )[0]


def preview_candidate(cup_pose: dict[str, float]) -> dict[str, Any]:
    grasp_z = cup_pose["z"] + 0.05
    return {
        "candidate_id": "dry_run_preview_not_planned",
        "axis": "-x",
        "height": 0.05,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0,
        "approach_pose": pose_dict(cup_pose["x"] + 0.175, cup_pose["y"], grasp_z),
        "grasp_pose": pose_dict(cup_pose["x"], cup_pose["y"], grasp_z),
        "lift_pose": pose_dict(cup_pose["x"], cup_pose["y"], grasp_z + 0.12),
        "all_success": False,
        "failure_code": "DRY_RUN_NOT_PLANNED",
    }


def pose_dict(x: float, y: float, z: float) -> dict[str, Any]:
    return {
        "position": {"x": x, "y": y, "z": z},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }


def print_pick_poses(candidate: dict[str, Any]) -> None:
    print_stage("PRINT_APPROACH_GRASP_LIFT_POSES", "review selected candidate")
    print(f"candidate_id={candidate.get('candidate_id')}")
    print(f"axis={candidate.get('axis')} height={candidate.get('height')}")
    print(
        "quaternion="
        f"[{candidate.get('qx')}, {candidate.get('qy')}, {candidate.get('qz')}, {candidate.get('qw')}]"
    )
    print(f"approach_pose={json.dumps(candidate.get('approach_pose'), sort_keys=True)}")
    print(f"grasp_pose={json.dumps(candidate.get('grasp_pose'), sort_keys=True)}")
    print(f"lift_pose={json.dumps(candidate.get('lift_pose'), sort_keys=True)}")
    print(f"all_success={candidate.get('all_success')} failure_code={candidate.get('failure_code')}")


def print_confirm_summary(args: argparse.Namespace, cup_pose: dict[str, float], best: dict[str, Any]) -> None:
    print_stage("USER_CONFIRM", "final review before any real command")
    print(f"cup_pose={cup_pose}")
    print(f"gripper_open={args.gripper_open_service}")
    print(f"gripper_close={args.gripper_close_service}")
    print(f"motion_backend={args.execute_action_name} [moveit_msgs/action/ExecuteTrajectory]")
    print(f"speed_scale={args.velocity_scale:.3f} accel_scale={args.accel_scale:.3f}")
    print(f"confirm_required={CONFIRM_PHRASE}")
    if args.enable_real_motion and not best.get("all_success"):
        print("[FAIL] selected candidate is not all_success; execution prohibited")


def execute_one_pick(args: argparse.Namespace, candidate: dict[str, Any]) -> int:
    print_stage("EXECUTE_ONE_PICK", "one-shot ExecuteTrajectory backend")
    if not candidate.get("all_success"):
        print("[FAIL] selected candidate is not all_success; execution prohibited")
        return 1
    try:
        executor = MoveItExecuteTrajectoryBackend(args)
        executor.run(candidate)
    except Exception as exc:
        print(f"[FAIL] execution stopped: {exc}")
        return 1
    print_stage("DONE", "supervised one-shot side pick command sequence completed")
    return 0


class MoveItExecuteTrajectoryBackend:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros2_logs")
        Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
        import rclpy
        from rclpy.node import Node

        if not rclpy.ok():
            rclpy.init(args=None)
        self.rclpy = rclpy
        self.node: Node = rclpy.create_node("azas_supervised_real_single_cup_pick")
        self._select_action_backend()
        self.moveit_py = None
        self.planning_component = None

    def run(self, candidate: dict[str, Any]) -> None:
        try:
            self._init_moveit()
            self._call_gripper(self.args.gripper_open_service, "GRIPPER_OPEN")
            for label in ("approach", "grasp"):
                trajectory = self._plan_pose(label, candidate[f"{label}_pose"])
                self._execute_trajectory(label, trajectory)
            self._call_gripper(self.args.gripper_close_service, "GRIPPER_CLOSE")
            trajectory = self._plan_pose("lift", candidate["lift_pose"])
            self._execute_trajectory("lift", trajectory)
        finally:
            self.node.destroy_node()
            try:
                if self.rclpy.ok():
                    self.rclpy.shutdown()
            except Exception:
                pass

    def execute_observe(self, observe_target: dict[str, Any]) -> None:
        label = "observe_cup_pose"
        if self.args.observe_mode == "joint":
            self.execute_joint_target(label, observe_target)
            return
        try:
            self._init_moveit()
            trajectory = self._plan_pose(label, observe_target)
            self._execute_trajectory(label, trajectory)
        finally:
            self.node.destroy_node()
            try:
                if self.rclpy.ok():
                    self.rclpy.shutdown()
            except Exception:
                pass

    def execute_joint_target(self, label: str, joint_degrees: dict[str, float]) -> None:
        self.execute_joint_sequence([(label, joint_degrees)])

    def execute_joint_sequence(self, sequence: list[tuple[str, dict[str, float]]]) -> None:
        try:
            self._init_moveit()
            for index, (label, joint_degrees) in enumerate(sequence):
                trajectory = self._plan_joint_target(label, joint_degrees)
                self._execute_trajectory(label, trajectory, expected_joint_degrees=joint_degrees)
                if index < len(sequence) - 1:
                    self._wait_for_joint_target(label, joint_degrees)
        finally:
            self.node.destroy_node()
            try:
                if self.rclpy.ok():
                    self.rclpy.shutdown()
            except Exception:
                pass

    def _select_action_backend(self) -> None:
        action_text = command_output(["ros2", "action", "list", "-t"], timeout=5.0)
        execute_available = (
            f"{self.args.execute_action_name} [moveit_msgs/action/ExecuteTrajectory]"
            in action_text
        )
        move_available = (
            f"{self.args.move_action_name} [moveit_msgs/action/MoveGroup]" in action_text
        )
        if execute_available:
            self.action_name = self.args.execute_action_name
            self.action_type = "moveit_msgs/action/ExecuteTrajectory"
            print(f"[OK] using action backend: {self.action_name} [{self.action_type}]")
            return
        if move_available:
            raise RuntimeError(
                f"{self.args.move_action_name} is available, but this script only executes "
                "already successful plan_result.trajectory via ExecuteTrajectory"
            )
        raise RuntimeError(
            f"{self.args.execute_action_name} [moveit_msgs/action/ExecuteTrajectory] not available"
        )

    def _init_moveit(self) -> None:
        from moveit.planning import MoveItPy
        from sweep_side_grasp_planning_candidates import moveit_config_dict

        self.moveit_py = MoveItPy(
            node_name="azas_supervised_single_pick_moveit",
            config_dict=moveit_config_dict("m0609", "dsr_moveit_config_m0609"),
            provide_planning_service=False,
        )
        time.sleep(2.0)
        self.planning_component = self.moveit_py.get_planning_component(
            self.args.planning_group
        )

    def _plan_pose(self, label: str, pose_dict_msg: dict[str, Any]):
        from geometry_msgs.msg import PoseStamped
        from moveit.planning import PlanRequestParameters

        print_stage(f"PLAN_{label.upper()}", "MoveItPy plan() before ExecuteTrajectory")
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.args.base_frame
        pose_msg.pose.position.x = float(pose_dict_msg["position"]["x"])
        pose_msg.pose.position.y = float(pose_dict_msg["position"]["y"])
        pose_msg.pose.position.z = float(pose_dict_msg["position"]["z"])
        pose_msg.pose.orientation.x = float(pose_dict_msg["orientation"]["x"])
        pose_msg.pose.orientation.y = float(pose_dict_msg["orientation"]["y"])
        pose_msg.pose.orientation.z = float(pose_dict_msg["orientation"]["z"])
        pose_msg.pose.orientation.w = float(pose_dict_msg["orientation"]["w"])

        params = PlanRequestParameters(self.moveit_py)
        params.planning_time = float(self.args.planning_timeout_sec)
        params.planning_pipeline = "ompl"
        params.planner_id = "RRTConnectkConfigDefault"
        params.planning_attempts = 1
        params.max_velocity_scaling_factor = float(self.args.velocity_scale)
        params.max_acceleration_scaling_factor = float(self.args.accel_scale)
        self.planning_component.set_start_state_to_current_state()
        self.planning_component.set_goal_state(
            pose_stamped_msg=pose_msg,
            pose_link=self.args.ee_link,
        )
        plan_result = self.planning_component.plan(params)
        if not plan_result:
            raise RuntimeError(f"{label} planning failed; execution prohibited")
        trajectory = getattr(plan_result, "trajectory", None)
        if trajectory is None:
            raise RuntimeError(f"{label} plan_result has no trajectory; execution prohibited")
        if hasattr(trajectory, "get_robot_trajectory_msg"):
            trajectory = trajectory.get_robot_trajectory_msg()
        print(f"[OK] {label} plan succeeded; trajectory ready for ExecuteTrajectory")
        return trajectory

    def _plan_joint_target(self, label: str, joint_degrees: dict[str, float]):
        from moveit.core.robot_state import RobotState
        from moveit.planning import PlanRequestParameters

        print_stage(f"PLAN_{label.upper()}", "MoveItPy joint target plan before ExecuteTrajectory")
        print(f"joint_target_deg={json.dumps(joint_degrees, sort_keys=True)}")
        robot_model = self.moveit_py.get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.joint_positions = {
            name: math.radians(float(degrees))
            for name, degrees in joint_degrees.items()
        }
        robot_state.update()

        params = PlanRequestParameters(self.moveit_py)
        params.planning_time = float(self.args.planning_timeout_sec)
        params.planning_pipeline = "ompl"
        params.planner_id = "RRTConnectkConfigDefault"
        params.planning_attempts = 1
        params.max_velocity_scaling_factor = float(self.args.velocity_scale)
        params.max_acceleration_scaling_factor = float(self.args.accel_scale)
        self.planning_component.set_start_state_to_current_state()
        self.planning_component.set_goal_state(robot_state=robot_state)
        plan_result = self.planning_component.plan(params)
        if not plan_result:
            raise RuntimeError(f"{label} joint target planning failed; execution prohibited")
        trajectory = getattr(plan_result, "trajectory", None)
        if trajectory is None:
            raise RuntimeError(f"{label} plan_result has no trajectory; execution prohibited")
        if hasattr(trajectory, "get_robot_trajectory_msg"):
            trajectory = trajectory.get_robot_trajectory_msg()
        print(f"[OK] {label} joint target plan succeeded; trajectory ready for ExecuteTrajectory")
        return trajectory

    def _execute_trajectory(
        self,
        label: str,
        trajectory,
        expected_joint_degrees: dict[str, float] | None = None,
    ) -> None:
        from moveit_msgs.action import ExecuteTrajectory
        from rclpy.action import ActionClient

        print_stage(f"EXECUTE_{label.upper()}", self.action_name)
        before_joints = self._read_joint_positions()
        client = ActionClient(self.node, ExecuteTrajectory, self.action_name)
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"ExecuteTrajectory action unavailable: {self.action_name}")
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = client.send_goal_async(goal)
        self.rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"{label} ExecuteTrajectory goal rejected")
        result_future = goal_handle.get_result_async()
        self.rclpy.spin_until_future_complete(
            self.node,
            result_future,
            timeout_sec=float(self.args.action_timeout_sec),
        )
        result_wrapper = result_future.result()
        if result_wrapper is None:
            raise RuntimeError(f"{label} ExecuteTrajectory timed out")
        error_code = getattr(result_wrapper.result.error_code, "val", None)
        if error_code != MOVEIT_SUCCESS:
            raise RuntimeError(f"{label} ExecuteTrajectory failed with error_code={error_code}")
        after_joints = self._read_joint_positions()
        max_delta = max_joint_delta(before_joints, after_joints)
        if max_delta is not None:
            print(f"[INFO] {label} max_joint_delta_rad={max_delta:.6f}")
            if max_delta < MIN_JOINT_MOTION_RAD:
                if expected_joint_degrees is not None and self._joint_target_reached(
                    expected_joint_degrees,
                    tolerance_rad=JOINT_TARGET_TOLERANCE_RAD,
                ):
                    print(
                        f"[OK] {label} target already reached; no extra joint motion required"
                    )
                    return
                raise RuntimeError(
                    f"{label} ExecuteTrajectory returned success but joint motion was not observed "
                    f"(max_delta_rad={max_delta:.6f})"
                )
        print(f"[OK] {label} ExecuteTrajectory succeeded")

    def _wait_for_joint_target(self, label: str, joint_degrees: dict[str, float]) -> None:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self._joint_target_reached(
                joint_degrees,
                tolerance_rad=JOINT_TARGET_TOLERANCE_RAD,
            ):
                print(f"[OK] {label} joint state reached target; planning next stage")
                time.sleep(1.0)
                return
            time.sleep(0.2)
        error = self._joint_target_error(joint_degrees)
        if error is None:
            print(
                f"[WARN] {label} joint state was not readable after execution; "
                "continuing after fixed settle delay"
            )
            time.sleep(2.0)
            return
        raise RuntimeError(
            f"{label} joint state did not reach target before next plan "
            f"(max_error_rad={error:.6f})"
        )

    def _read_joint_positions(self) -> list[float] | None:
        state = self._read_joint_state_map(timeout_sec=3.0)
        if state is None:
            print("[WARN] /joint_states was not readable for motion delta check")
            return None
        return [state[name] for name in sorted(state)]

    def _read_joint_state_map(self, timeout_sec: float) -> dict[str, float] | None:
        from sensor_msgs.msg import JointState

        latest: dict[str, float] | None = None

        def callback(msg: JointState) -> None:
            nonlocal latest
            if msg.name and len(msg.name) == len(msg.position):
                latest = dict(zip(msg.name, msg.position))

        subscription = self.node.create_subscription(JointState, "/joint_states", callback, 10)
        deadline = time.monotonic() + timeout_sec
        try:
            while latest is None and time.monotonic() < deadline:
                self.rclpy.spin_once(self.node, timeout_sec=0.1)
            return latest
        finally:
            self.node.destroy_subscription(subscription)

    def _joint_target_error(self, joint_degrees: dict[str, float]) -> float | None:
        current = self._read_joint_state_map(timeout_sec=1.0)
        if current is None:
            return None
        errors: list[float] = []
        for name, degrees in joint_degrees.items():
            if name not in current:
                return None
            errors.append(abs(current[name] - math.radians(float(degrees))))
        return max(errors) if errors else None

    def _joint_target_reached(self, joint_degrees: dict[str, float], tolerance_rad: float) -> bool:
        error = self._joint_target_error(joint_degrees)
        if error is None:
            return False
        print(f"[INFO] joint target max_error_rad={error:.6f}")
        return error <= tolerance_rad

    def _call_gripper(self, service_name: str, label: str) -> None:
        from std_srvs.srv import Trigger

        print_stage(label, service_name)
        client = self.node.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"gripper service unavailable: {service_name}")
        future = client.call_async(Trigger.Request())
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.success:
            message = getattr(response, "message", "<no response>")
            raise RuntimeError(f"gripper service failed: {service_name}: {message}")
        print(f"[OK] {label}: {response.message}")


def dry_run_only(args: argparse.Namespace) -> bool:
    return not args.enable_real_motion


def command_output(command: list[str], timeout: float) -> str:
    result = run_command(command, timeout=timeout, capture=True)
    return result.stdout if result.returncode == 0 else ""


def read_joint_positions() -> list[float] | None:
    output = command_output(["ros2", "topic", "echo", "--once", "/joint_states"], timeout=3.0)
    positions: list[float] = []
    in_position = False
    for line in output.splitlines():
        stripped = line.strip()
        if stripped == "position:":
            in_position = True
            continue
        if in_position and stripped in {"velocity:", "effort:"}:
            break
        if in_position and stripped.startswith("- "):
            try:
                positions.append(float(stripped[2:].strip()))
            except ValueError:
                return None
    return positions or None


def read_joint_state_map() -> dict[str, float] | None:
    output = command_output(["ros2", "topic", "echo", "--once", "/joint_states"], timeout=3.0)
    names: list[str] = []
    positions: list[float] = []
    section: str | None = None
    for line in output.splitlines():
        stripped = line.strip()
        if stripped == "name:":
            section = "name"
            continue
        if stripped == "position:":
            section = "position"
            continue
        if stripped in {"velocity:", "effort:"}:
            section = None
            continue
        if not stripped.startswith("- "):
            continue
        value = stripped[2:].strip().strip("'\"")
        if section == "name":
            names.append(value)
        elif section == "position":
            try:
                positions.append(float(value))
            except ValueError:
                return None
    if not names or len(names) != len(positions):
        return None
    return dict(zip(names, positions))


def joint_target_error(joint_degrees: dict[str, float]) -> float | None:
    current = read_joint_state_map()
    if current is None:
        return None
    errors: list[float] = []
    for name, degrees in joint_degrees.items():
        if name not in current:
            return None
        errors.append(abs(current[name] - math.radians(float(degrees))))
    return max(errors) if errors else None


def joint_target_reached(joint_degrees: dict[str, float], tolerance_rad: float) -> bool:
    error = joint_target_error(joint_degrees)
    if error is None:
        return False
    print(f"[INFO] joint target max_error_rad={error:.6f}")
    return error <= tolerance_rad


def max_joint_delta(before: list[float] | None, after: list[float] | None) -> float | None:
    if before is None or after is None or len(before) != len(after):
        print("[WARN] joint motion delta could not be verified from /joint_states")
        return None
    return max(abs(a - b) for a, b in zip(after, before))


def run_command(
    command: list[str],
    timeout: float,
    capture: bool,
) -> subprocess.CompletedProcess[str]:
    kwargs: dict[str, Any] = {
        "cwd": str(ROOT_DIR),
        "timeout": timeout,
        "check": False,
        "text": True,
    }
    if capture:
        kwargs.update({"stdout": subprocess.PIPE, "stderr": subprocess.STDOUT})
    try:
        return subprocess.run(command, **kwargs)
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout if isinstance(exc.stdout, str) else ""
        return subprocess.CompletedProcess(command, 124, output)


if __name__ == "__main__":
    raise SystemExit(main())
