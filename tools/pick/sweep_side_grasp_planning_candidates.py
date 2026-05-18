#!/usr/bin/env python3
"""Sweep side-grasp pose candidates through MoveItPy planning-only.

This tool creates approach/grasp/lift poses with azas_motion.alignment and calls
MoveItPy plan() only. It never commands a trajectory or gripper.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

ROOT_DIR = Path(__file__).resolve().parents[2]
SRC_MOTION = ROOT_DIR / "src" / "azas_motion"
if str(SRC_MOTION) not in sys.path:
    sys.path.insert(0, str(SRC_MOTION))

from azas_motion.alignment import SideGraspConfig, compute_side_grasp_plan  # noqa: E402
from geometry_msgs.msg import Pose, PoseStamped  # noqa: E402


DEFAULT_JSON_PATH = Path("/tmp/azas_side_grasp_candidate_sweep.json")
DEFAULT_CSV_PATH = Path("/tmp/azas_side_grasp_candidate_sweep.csv")
AXIS_PRIORITY = {"-x": 0, "+x": 1, "-y": 2, "+y": 3}


@dataclass(frozen=True)
class OrientationCandidate:
    name: str
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class Candidate:
    candidate_id: int
    axis: str
    height: float
    orientation: OrientationCandidate


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return normalize_quaternion((qx, qy, qz, qw))


def normalize_quaternion(values: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in values))
    if norm == 0.0 or not math.isfinite(norm):
        raise ValueError("invalid quaternion candidate")
    return tuple(value / norm for value in values)


def orientation_candidates() -> list[OrientationCandidate]:
    candidates: list[OrientationCandidate] = []
    seen: set[tuple[int, int, int, int]] = set()

    def add(name: str, values: tuple[float, float, float, float]) -> None:
        qx, qy, qz, qw = normalize_quaternion(values)
        key = tuple(round(value, 6) for value in (qx, qy, qz, qw))
        if key in seen:
            return
        seen.add(key)
        candidates.append(OrientationCandidate(name, qx, qy, qz, qw))

    add("identity", (0.0, 0.0, 0.0, 1.0))
    add("tool_z_down_roll_pi", quaternion_from_euler(math.pi, 0.0, 0.0))
    add("tool_z_down_pitch_pi", quaternion_from_euler(0.0, math.pi, 0.0))
    add("tool_x_forward_pitch_minus_pi_2", quaternion_from_euler(0.0, -math.pi / 2.0, 0.0))
    add("tool_x_forward_pitch_plus_pi_2", quaternion_from_euler(0.0, math.pi / 2.0, 0.0))

    rolls = (-math.pi, -math.pi / 2.0, 0.0, math.pi / 2.0, math.pi)
    pitches = (-math.pi / 2.0, 0.0, math.pi / 2.0)
    yaws = (-math.pi, -math.pi / 2.0, 0.0, math.pi / 2.0, math.pi)
    for roll in rolls:
        for pitch in pitches:
            for yaw in yaws:
                add(
                    f"r{roll:.3f}_p{pitch:.3f}_y{yaw:.3f}",
                    quaternion_from_euler(roll, pitch, yaw),
                )
    return candidates


def build_candidates(
    axes: Iterable[str],
    heights: Iterable[float],
    max_candidates: int | None,
) -> list[Candidate]:
    orientations = orientation_candidates()
    candidates: list[Candidate] = []
    candidate_id = 1
    for orientation in orientations:
        for height in heights:
            for axis in axes:
                candidates.append(Candidate(candidate_id, axis, height, orientation))
                candidate_id += 1
                if max_candidates is not None and len(candidates) >= max_candidates:
                    return candidates
    return candidates


def moveit_config_dict(robot_model: str, moveit_config_package: str) -> dict:
    from moveit_configs_utils import MoveItConfigsBuilder

    config = (
        MoveItConfigsBuilder(robot_model, "robot_description", moveit_config_package)
        .robot_description(file_path=f"config/{robot_model}.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
            load_all=False,
        )
        .to_moveit_configs()
        .to_dict()
    )
    pipeline_names = config.get("planning_pipelines", ["ompl"])
    if isinstance(pipeline_names, list):
        config["planning_pipelines"] = {"pipeline_names": pipeline_names}
    config["plan_request_params"] = {
        "planning_attempts": 1,
        "planning_pipeline": "ompl",
        "planner_id": "RRTConnectkConfigDefault",
        "max_velocity_scaling_factor": 0.1,
        "max_acceleration_scaling_factor": 0.1,
        "planning_time": 1.0,
    }
    return config


def pose_stamped(pose: Pose, frame_id: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose = pose
    return msg


def plan_pose(planning_component, moveit_py, pose_msg: PoseStamped, ee_link: str, timeout: float) -> dict:
    from moveit.planning import PlanRequestParameters

    try:
        request_parameters = PlanRequestParameters(moveit_py)
        request_parameters.planning_time = timeout
        request_parameters.planning_pipeline = "ompl"
        request_parameters.planner_id = "RRTConnectkConfigDefault"
        request_parameters.planning_attempts = 1
        request_parameters.max_velocity_scaling_factor = 0.1
        request_parameters.max_acceleration_scaling_factor = 0.1
        planning_component.set_start_state_to_current_state()
        planning_component.set_goal_state(pose_stamped_msg=pose_msg, pose_link=ee_link)
        solution = planning_component.plan(request_parameters)
        return {
            "status": "SUCCESS" if bool(solution) else "FAILED",
            "error_code": getattr(getattr(solution, "error_code", None), "val", None),
        }
    except Exception as exc:
        return {
            "status": "ERROR",
            "error_code": None,
            "error": str(exc),
        }


def result_failure_code(result: dict) -> str:
    failures = []
    for label in ("approach", "grasp", "lift"):
        pose_result = result[f"{label}_result_detail"]
        if pose_result["status"] != "SUCCESS":
            code = pose_result.get("error_code")
            failures.append(f"{label}:{pose_result['status']}:{code}")
    return "OK" if not failures else "|".join(failures)


def score_result(result: dict) -> tuple[int, float, float, int]:
    all_success_score = 0 if result["all_success"] else 1
    height_score = abs(result["height"] - 0.06)
    distance_score = -float(result["approach_distance_m"])
    axis_score = AXIS_PRIORITY.get(result["axis"], 99)
    return (all_success_score, height_score, distance_score, axis_score)


def write_outputs(results: list[dict], json_path: Path, csv_path: Path) -> None:
    json_path.write_text(json.dumps(results, indent=2, sort_keys=True), encoding="utf-8")
    fieldnames = [
        "candidate_id",
        "axis",
        "height",
        "orientation_name",
        "qx",
        "qy",
        "qz",
        "qw",
        "approach_result",
        "grasp_result",
        "lift_result",
        "all_success",
        "failure_code",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            writer.writerow({key: result.get(key) for key in fieldnames})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sweep side-grasp planning candidates with MoveItPy plan() only."
    )
    parser.add_argument("--planning-group", default="manipulator")
    parser.add_argument("--ee-link", default="tool0")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--robot-model", default="m0609")
    parser.add_argument("--moveit-config-package", default="dsr_moveit_config_m0609")
    parser.add_argument("--cup-reference-x", type=float, required=True)
    parser.add_argument("--cup-reference-y", type=float, required=True)
    parser.add_argument("--cup-reference-z", type=float, required=True)
    parser.add_argument("--side-approach-axes", default="-x,+x,-y,+y")
    parser.add_argument("--grasp-height-offsets", default="0.03,0.05,0.07,0.09")
    parser.add_argument("--cup-radius-m", type=float, default=0.035)
    parser.add_argument("--side-clearance-m", type=float, default=0.02)
    parser.add_argument("--side-approach-offset-m", type=float, default=0.12)
    parser.add_argument("--lift-offset-m", type=float, default=0.12)
    parser.add_argument("--min-grasp-z-m", type=float, default=0.03)
    parser.add_argument("--max-grasp-z-m", type=float, default=0.40)
    parser.add_argument("--planning-timeout-sec", type=float, default=1.0)
    parser.add_argument("--max-candidates", type=int, default=None)
    parser.add_argument("--stop-on-success", action="store_true")
    parser.add_argument("--json-output", type=Path, default=DEFAULT_JSON_PATH)
    parser.add_argument("--csv-output", type=Path, default=DEFAULT_CSV_PATH)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    axes = [axis.strip() for axis in args.side_approach_axes.split(",") if axis.strip()]
    heights = [
        float(height.strip())
        for height in args.grasp_height_offsets.split(",")
        if height.strip()
    ]
    candidates = build_candidates(axes, heights, args.max_candidates)
    reference_pose = Pose()
    reference_pose.position.x = args.cup_reference_x
    reference_pose.position.y = args.cup_reference_y
    reference_pose.position.z = args.cup_reference_z
    reference_pose.orientation.w = 1.0

    print("[Azas] Side grasp candidate sweep: planning-only, no execution path")
    print(
        "[Azas] Inputs: "
        f"planning_group={args.planning_group} ee_link={args.ee_link} "
        f"base_frame={args.base_frame} candidates={len(candidates)}"
    )

    try:
        from moveit.planning import MoveItPy
    except Exception as exc:
        print(f"[FAIL] MoveItPy import failed; no planning attempted: {exc}")
        return 2

    try:
        moveit_py = MoveItPy(
            node_name="azas_side_grasp_candidate_sweep",
            config_dict=moveit_config_dict(args.robot_model, args.moveit_config_package),
            provide_planning_service=False,
        )
        planning_component = moveit_py.get_planning_component(args.planning_group)
    except Exception as exc:
        print(f"[FAIL] MoveItPy planning-only initialization failed: {exc}")
        return 2

    results: list[dict] = []
    for candidate in candidates:
        try:
            plan = compute_side_grasp_plan(
                reference_pose,
                SideGraspConfig(
                    orientation_source="parameter",
                    side_grasp_qx=candidate.orientation.qx,
                    side_grasp_qy=candidate.orientation.qy,
                    side_grasp_qz=candidate.orientation.qz,
                    side_grasp_qw=candidate.orientation.qw,
                    side_approach_axis=candidate.axis,
                    side_approach_offset_m=args.side_approach_offset_m,
                    side_clearance_m=args.side_clearance_m,
                    cup_radius_m=args.cup_radius_m,
                    grasp_height_offset_m=candidate.height,
                    lift_offset_m=args.lift_offset_m,
                    min_grasp_z_m=args.min_grasp_z_m,
                    max_grasp_z_m=args.max_grasp_z_m,
                ),
            )
        except ValueError as exc:
            result = {
                "candidate_id": candidate.candidate_id,
                "axis": candidate.axis,
                "height": candidate.height,
                "orientation_name": candidate.orientation.name,
                "qx": candidate.orientation.qx,
                "qy": candidate.orientation.qy,
                "qz": candidate.orientation.qz,
                "qw": candidate.orientation.qw,
                "approach_result": "SKIPPED",
                "grasp_result": "SKIPPED",
                "lift_result": "SKIPPED",
                "all_success": False,
                "failure_code": f"CONFIG:{exc}",
            }
            results.append(result)
            continue

        approach_result = plan_pose(
            planning_component,
            moveit_py,
            pose_stamped(plan.approach_pose, args.base_frame),
            args.ee_link,
            args.planning_timeout_sec,
        )
        grasp_result = plan_pose(
            planning_component,
            moveit_py,
            pose_stamped(plan.grasp_pose, args.base_frame),
            args.ee_link,
            args.planning_timeout_sec,
        )
        lift_result = plan_pose(
            planning_component,
            moveit_py,
            pose_stamped(plan.lift_pose, args.base_frame),
            args.ee_link,
            args.planning_timeout_sec,
        )
        all_success = all(
            item["status"] == "SUCCESS"
            for item in (approach_result, grasp_result, lift_result)
        )
        result = {
            "candidate_id": candidate.candidate_id,
            "axis": candidate.axis,
            "height": candidate.height,
            "orientation_name": candidate.orientation.name,
            "qx": candidate.orientation.qx,
            "qy": candidate.orientation.qy,
            "qz": candidate.orientation.qz,
            "qw": candidate.orientation.qw,
            "approach_pose": pose_to_dict(plan.approach_pose),
            "grasp_pose": pose_to_dict(plan.grasp_pose),
            "lift_pose": pose_to_dict(plan.lift_pose),
            "approach_distance_m": plan.approach_distance_m,
            "approach_result_detail": approach_result,
            "grasp_result_detail": grasp_result,
            "lift_result_detail": lift_result,
            "approach_result": approach_result["status"],
            "grasp_result": grasp_result["status"],
            "lift_result": lift_result["status"],
            "all_success": all_success,
            "warning": plan.warning,
        }
        result["failure_code"] = result_failure_code(result)
        results.append(result)
        print(
            f"[{candidate.candidate_id:04d}] axis={candidate.axis} "
            f"height={candidate.height:.3f} quat={candidate.orientation.name} "
            f"approach={approach_result['status']}({approach_result.get('error_code')}) "
            f"grasp={grasp_result['status']}({grasp_result.get('error_code')}) "
            f"lift={lift_result['status']}({lift_result.get('error_code')}) "
            f"all_success={all_success}"
        )
        if all_success and args.stop_on_success:
            break

    write_outputs(results, args.json_output, args.csv_output)
    successes = [result for result in results if result["all_success"]]
    best = min(results, key=score_result) if results else None
    print(f"[Azas] Results written: {args.json_output} {args.csv_output}")
    print(f"[Azas] Tested candidates: {len(results)}")
    print(f"[Azas] All-success candidates: {len(successes)}")
    if best:
        print(
            "[Azas] Best candidate: "
            f"id={best['candidate_id']} axis={best['axis']} height={best['height']:.3f} "
            f"quat=[{best['qx']:.6f},{best['qy']:.6f},{best['qz']:.6f},{best['qw']:.6f}] "
            f"failure_code={best['failure_code']}"
        )
    return 0


def pose_to_dict(pose: Pose) -> dict:
    return {
        "position": {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
        },
        "orientation": {
            "x": pose.orientation.x,
            "y": pose.orientation.y,
            "z": pose.orientation.z,
            "w": pose.orientation.w,
        },
    }


if __name__ == "__main__":
    raise SystemExit(main())
