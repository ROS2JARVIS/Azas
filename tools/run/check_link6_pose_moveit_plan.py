#!/usr/bin/env python3
"""Fail-closed MoveIt planning guard for measured link_6 target poses.

This script does not execute robot motion. It asks MoveItPy to plan from the
current robot state to a supplied link_6 PoseStamped target. The intended use is
as a guard before direct Doosan MoveLine primitives that bypass MoveIt execution:
if MoveIt cannot find a collision-scene-valid plan, the caller must not send the
direct MoveLine command.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

ROOT = Path("/home/ssu/Azas")
SRC_AZAS_MOTION = ROOT / "src" / "azas_motion"
if str(SRC_AZAS_MOTION) not in sys.path:
    sys.path.insert(0, str(SRC_AZAS_MOTION))

from azas_motion.side_grasp_ik_preview_node import moveit_config_dict  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plan current state -> target link_6 pose with MoveItPy and return non-zero "
            "when the collision-scene-aware plan is unavailable. No execution is performed."
        )
    )
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--qx", type=float, required=True)
    parser.add_argument("--qy", type=float, required=True)
    parser.add_argument("--qz", type=float, required=True)
    parser.add_argument("--qw", type=float, required=True)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--ee-link", default="link_6")
    parser.add_argument("--planning-group", default="manipulator")
    parser.add_argument("--robot-model", default="m0609")
    parser.add_argument("--moveit-config-package", default="dsr_moveit_config_m0609")
    parser.add_argument("--planning-pipeline", default="pilz_industrial_motion_planner")
    parser.add_argument("--planner-id", default="PTP")
    parser.add_argument("--planning-timeout-sec", type=float, default=3.0)
    parser.add_argument("--planning-attempts", type=int, default=1)
    parser.add_argument("--max-velocity-scaling-factor", type=float, default=0.1)
    parser.add_argument("--max-acceleration-scaling-factor", type=float, default=0.1)
    parser.add_argument(
        "--require-collision-object",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Fail closed unless at least one /collision_object sample is visible before planning. "
            "This catches a missing measured collision-scene publisher before a direct MoveLine."
        ),
    )
    parser.add_argument("--collision-object-topic", default="/collision_object")
    parser.add_argument("--collision-object-timeout-sec", type=float, default=3.0)
    return parser.parse_args()


def target_pose(args: argparse.Namespace) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = str(args.frame_id)
    pose.pose.position.x = float(args.x)
    pose.pose.position.y = float(args.y)
    pose.pose.position.z = float(args.z)
    pose.pose.orientation.x = float(args.qx)
    pose.pose.orientation.y = float(args.qy)
    pose.pose.orientation.z = float(args.qz)
    pose.pose.orientation.w = float(args.qw)
    return pose



def transient_collision_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def wait_for_collision_object(args: argparse.Namespace) -> str:
    """Return the first visible collision object id, or raise to fail closed."""
    if not args.require_collision_object:
        return ""

    node = rclpy.create_node("azas_moveit_plan_guard_collision_object_wait")
    seen_ids: list[str] = []

    def on_collision_object(msg: CollisionObject) -> None:
        object_id = str(getattr(msg, "id", "") or "<unnamed>")
        if object_id not in seen_ids:
            seen_ids.append(object_id)

    node.create_subscription(
        CollisionObject,
        str(args.collision_object_topic),
        on_collision_object,
        transient_collision_qos(),
    )
    deadline = node.get_clock().now().nanoseconds / 1e9 + max(
        float(args.collision_object_timeout_sec), 0.1
    )
    try:
        while rclpy.ok() and not seen_ids:
            now = node.get_clock().now().nanoseconds / 1e9
            if now > deadline:
                raise RuntimeError(
                    f"no collision objects observed on {args.collision_object_topic} "
                    f"within {args.collision_object_timeout_sec:.1f}s"
                )
            rclpy.spin_once(node, timeout_sec=0.05)
        return seen_ids[0]
    finally:
        node.destroy_node()


def plan_target(args: argparse.Namespace) -> Any:
    from moveit.planning import MoveItPy, PlanRequestParameters

    moveit_py = MoveItPy(
        node_name="azas_link6_pose_moveit_plan_guard",
        config_dict=moveit_config_dict(
            str(args.robot_model),
            str(args.moveit_config_package),
        ),
        provide_planning_service=False,
    )
    planning_component = moveit_py.get_planning_component(str(args.planning_group))

    request_parameters = PlanRequestParameters(moveit_py)
    request_parameters.planning_time = max(float(args.planning_timeout_sec), 0.1)
    request_parameters.planning_pipeline = str(args.planning_pipeline)
    request_parameters.planner_id = str(args.planner_id)
    request_parameters.planning_attempts = max(int(args.planning_attempts), 1)
    request_parameters.max_velocity_scaling_factor = float(args.max_velocity_scaling_factor)
    request_parameters.max_acceleration_scaling_factor = float(args.max_acceleration_scaling_factor)

    pose = target_pose(args)
    planning_component.set_start_state_to_current_state()
    planning_component.set_goal_state(pose_stamped_msg=pose, pose_link=str(args.ee_link))
    solution = planning_component.plan(request_parameters)
    if not solution:
        p = pose.pose.position
        raise RuntimeError(
            "MoveItPy plan failed for "
            f"{args.ee_link} target x={p.x:.4f} y={p.y:.4f} z={p.z:.4f} "
            f"frame={pose.header.frame_id} pipeline={args.planning_pipeline} planner={args.planner_id}"
        )
    trajectory = getattr(solution, "trajectory", None)
    if trajectory is None:
        raise RuntimeError("MoveItPy returned a solution without a trajectory")
    if hasattr(trajectory, "get_robot_trajectory_msg"):
        trajectory = trajectory.get_robot_trajectory_msg()
    points = list(getattr(trajectory.joint_trajectory, "points", []))
    if not points:
        raise RuntimeError("MoveItPy plan returned an empty joint trajectory")
    return trajectory


def main() -> int:
    args = parse_args()
    print(
        "[Azas] MoveIt planning guard: current state -> "
        f"{args.ee_link} target in {args.frame_id}; collision scene must permit a plan."
    )
    rclpy.init(args=None)
    try:
        object_id = wait_for_collision_object(args)
        if object_id:
            print(f"[Azas] MoveIt planning guard saw collision scene object: {object_id}")
        trajectory = plan_target(args)
        point_count = len(trajectory.joint_trajectory.points)
        joint_names = list(trajectory.joint_trajectory.joint_names)
        print(
            "[PASS] MoveIt planning guard accepted target: "
            f"points={point_count} joints={joint_names}"
        )
        return 0
    except Exception as exc:
        print(f"[BLOCKED] MoveIt planning guard rejected target; direct MoveLine refused: {exc}")
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
