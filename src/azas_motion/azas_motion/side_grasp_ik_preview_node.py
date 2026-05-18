#!/usr/bin/env python3
"""RViz-only IK preview for the M0609 URDF.

This node consumes the Azas dry-run Path, plans each pose with MoveItPy, and
publishes the resulting joint sequence on /joint_states so RViz shows the real
M0609 model moving. It never calls ExecuteTrajectory, Doosan services, gripper
services, or any hardware API.
"""

from __future__ import annotations

from typing import List, Optional

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import JointState


DEFAULT_HOME_JOINTS = [0.0, -0.62, 1.38, 0.0, 1.22, 0.0]


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


class SideGraspIkPreviewNode(Node):
    def __init__(self) -> None:
        super().__init__("side_grasp_ik_preview_node")
        self.declare_parameter("plan_topic", "/azas/dispenser_sequence/plan")
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("ee_link", "tool0")
        self.declare_parameter("robot_model", "m0609")
        self.declare_parameter("moveit_config_package", "dsr_moveit_config_m0609")
        self.declare_parameter("planning_timeout_sec", 1.0)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("frames_per_step", 90)
        self.declare_parameter("hold_frames", 35)
        self.declare_parameter("loop_preview", False)
        self.declare_parameter("home_joints_rad", DEFAULT_HOME_JOINTS)

        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.subscription = self.create_subscription(
            Path,
            str(self.get_parameter("plan_topic").value),
            self.on_path,
            10,
        )
        self.pending_path: Optional[Path] = None
        self.planning_started = False
        self.planning_failed = False
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.preview_points: List[List[float]] = []
        self.preview_index = 0
        self.moveit_py = None
        self.planning_component = None

        rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            "Azas RViz-only M0609 IK preview waiting for Path; hardware execution is disabled."
        )

    def on_path(self, msg: Path) -> None:
        if self.planning_started or self.preview_points:
            return
        if len(msg.poses) < 2:
            self.get_logger().warning("Ignoring preview Path with fewer than two poses")
            return
        self.pending_path = msg
        self.get_logger().info(f"Received Azas dispenser sequence Path with {len(msg.poses)} poses")

    def on_timer(self) -> None:
        if self.preview_points:
            self.publish_preview_point()
            return
        self.publish_home()
        if self.pending_path is not None and not self.planning_started and not self.planning_failed:
            self.planning_started = True
            self.plan_preview(self.pending_path)

    def publish_home(self) -> None:
        joints = [float(value) for value in self.get_parameter("home_joints_rad").value]
        while len(joints) < len(self.joint_names):
            joints.append(0.0)
        self.publish_joint_state(self.joint_names, joints[: len(self.joint_names)])

    def publish_preview_point(self) -> None:
        self.publish_joint_state(self.joint_names, self.preview_points[self.preview_index])
        self.preview_index += 1
        if self.preview_index >= len(self.preview_points):
            if bool(self.get_parameter("loop_preview").value):
                self.preview_index = 0
            else:
                self.preview_index = len(self.preview_points) - 1

    def publish_joint_state(self, names: List[str], positions: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.publisher.publish(msg)

    def ensure_moveit(self) -> None:
        if self.moveit_py is not None:
            return
        from moveit.planning import MoveItPy

        robot_model = str(self.get_parameter("robot_model").value)
        moveit_config_package = str(self.get_parameter("moveit_config_package").value)
        planning_group = str(self.get_parameter("planning_group").value)
        self.moveit_py = MoveItPy(
            node_name="azas_side_grasp_ik_preview_moveit",
            config_dict=moveit_config_dict(robot_model, moveit_config_package),
            provide_planning_service=False,
        )
        self.planning_component = self.moveit_py.get_planning_component(planning_group)

    def plan_preview(self, path: Path) -> None:
        try:
            self.ensure_moveit()
            points: List[List[float]] = []
            for index, pose_stamped in enumerate(path.poses, start=1):
                trajectory = self.plan_pose(pose_stamped)
                names, positions = self.trajectory_points(trajectory)
                if not positions:
                    raise RuntimeError(f"pose {index} plan produced no joint trajectory points")
                self.joint_names = names
                target = positions[-1]
                start = points[-1] if points else self.current_home_positions(len(target))
                points.extend(self.interpolate_positions(start, target))
                self.get_logger().info(
                    f"IK preview planned sequential pose {index}/{len(path.poses)}"
                )
            self.preview_points = points
            self.preview_index = 0
            self.get_logger().info(
                f"Azas M0609 IK preview ready: {len(self.preview_points)} joint-state frames"
            )
        except Exception as exc:
            self.planning_failed = True
            self.get_logger().error(f"Azas M0609 IK preview failed closed: {exc}")

    def plan_pose(self, pose_stamped):
        from moveit.planning import PlanRequestParameters

        request_parameters = PlanRequestParameters(self.moveit_py)
        request_parameters.planning_time = float(self.get_parameter("planning_timeout_sec").value)
        request_parameters.planning_pipeline = "ompl"
        request_parameters.planner_id = "RRTConnectkConfigDefault"
        request_parameters.planning_attempts = 1
        request_parameters.max_velocity_scaling_factor = 0.1
        request_parameters.max_acceleration_scaling_factor = 0.1
        self.planning_component.set_start_state_to_current_state()
        self.planning_component.set_goal_state(
            pose_stamped_msg=pose_stamped,
            pose_link=str(self.get_parameter("ee_link").value),
        )
        solution = self.planning_component.plan(request_parameters)
        if not solution:
            p = pose_stamped.pose.position
            raise RuntimeError(f"MoveItPy plan failed for pose x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}")
        trajectory = getattr(solution, "trajectory", None)
        if trajectory is None:
            raise RuntimeError("MoveItPy solution has no trajectory")
        if hasattr(trajectory, "get_robot_trajectory_msg"):
            trajectory = trajectory.get_robot_trajectory_msg()
        return trajectory

    @staticmethod
    def trajectory_points(trajectory) -> tuple[List[str], List[List[float]]]:
        joint_trajectory = trajectory.joint_trajectory
        names = list(joint_trajectory.joint_names)
        positions = [list(point.positions) for point in joint_trajectory.points]
        return names, positions

    def current_home_positions(self, size: int) -> List[float]:
        joints = [float(value) for value in self.get_parameter("home_joints_rad").value]
        while len(joints) < size:
            joints.append(0.0)
        return joints[:size]

    def interpolate_positions(self, start: List[float], end: List[float]) -> List[List[float]]:
        steps = max(int(self.get_parameter("frames_per_step").value), 2)
        hold_frames = max(int(self.get_parameter("hold_frames").value), 0)
        size = min(len(start), len(end))
        frames: List[List[float]] = []
        for step in range(steps):
            t = step / steps
            frames.append([start[index] + (end[index] - start[index]) * t for index in range(size)])
        frames.extend([end[:size]] * hold_frames)
        return frames


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SideGraspIkPreviewNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
