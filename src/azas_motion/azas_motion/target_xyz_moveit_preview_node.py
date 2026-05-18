#!/usr/bin/env python3
"""Move a held cup to a target XYZ from the current side-grasp state.

This is RViz-only. It reads the current robot state, keeps the current
end-effector orientation, replaces only the target position, asks MoveItPy for
a plan, and publishes the resulting joint trajectory on /joint_states.
"""

from __future__ import annotations

import math
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from .side_grasp_ik_preview_node import moveit_config_dict


DEFAULT_SIDE_GRASP_JOINTS = [-0.50, 0.95, 1.35, 0.0, 2.50, 1.57]
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class TargetXyzMoveItPreviewNode(Node):
    def __init__(self) -> None:
        super().__init__("target_xyz_moveit_preview_node")
        self.declare_parameter("target_x", 0.43)
        self.declare_parameter("target_y", 0.08)
        self.declare_parameter("target_z", 0.135)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_link", "link_6")
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("robot_model", "m0609")
        self.declare_parameter("moveit_config_package", "dsr_moveit_config_m0609")
        self.declare_parameter("planning_pipeline", "pilz_industrial_motion_planner")
        self.declare_parameter("planner_id", "PTP")
        self.declare_parameter("planning_timeout_sec", 3.0)
        self.declare_parameter("planning_attempts", 1)
        self.declare_parameter("max_velocity_scaling_factor", 0.1)
        self.declare_parameter("max_acceleration_scaling_factor", 0.1)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("frames_per_step", 150)
        self.declare_parameter("hold_frames", 90)
        self.declare_parameter("loop_preview", True)
        self.declare_parameter("seed_current_state", True)
        self.declare_parameter("seed_duration_sec", 1.5)
        self.declare_parameter("side_grasp_joints_rad", DEFAULT_SIDE_GRASP_JOINTS)

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        target_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(
            PoseStamped, "/azas/target_xyz_moveit/target_pose", target_qos
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_time = self.get_clock().now()
        self.moveit_py = None
        self.planning_component = None
        self.planning_started = False
        self.planning_failed = False
        self.preview_points: List[List[float]] = []
        self.preview_index = 0
        self.joint_names = JOINT_NAMES[:]

        rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            "target_xyz_moveit_preview_node ready: current side-grasp state -> target XYZ"
        )

    def on_timer(self) -> None:
        if self.preview_points:
            self.publish_preview_point()
            return

        if bool(self.get_parameter("seed_current_state").value):
            self.publish_seed_state()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        seed_duration = float(self.get_parameter("seed_duration_sec").value)
        if elapsed < seed_duration or self.planning_started or self.planning_failed:
            return

        self.planning_started = True
        self.plan_from_current_to_target()

    def publish_seed_state(self) -> None:
        self.publish_joint_state(self.joint_names, self.side_grasp_joints())

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
        self.joint_pub.publish(msg)

    def side_grasp_joints(self) -> List[float]:
        joints = [float(value) for value in self.get_parameter("side_grasp_joints_rad").value]
        while len(joints) < len(JOINT_NAMES):
            joints.append(0.0)
        return joints[: len(JOINT_NAMES)]

    def current_target_pose(self) -> PoseStamped:
        frame_id = str(self.get_parameter("frame_id").value)
        ee_link = str(self.get_parameter("ee_link").value)
        transform = self.tf_buffer.lookup_transform(
            frame_id,
            ee_link,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.5),
        )
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(self.get_parameter("target_x").value)
        pose.pose.position.y = float(self.get_parameter("target_y").value)
        pose.pose.position.z = float(self.get_parameter("target_z").value)
        pose.pose.orientation = transform.transform.rotation
        return pose

    def ensure_moveit(self) -> None:
        if self.moveit_py is not None:
            return
        from moveit.planning import MoveItPy

        robot_model = str(self.get_parameter("robot_model").value)
        moveit_config_package = str(self.get_parameter("moveit_config_package").value)
        planning_group = str(self.get_parameter("planning_group").value)
        self.moveit_py = MoveItPy(
            node_name="azas_target_xyz_moveit_preview",
            config_dict=moveit_config_dict(robot_model, moveit_config_package),
            provide_planning_service=False,
        )
        self.planning_component = self.moveit_py.get_planning_component(planning_group)

    def plan_from_current_to_target(self) -> None:
        try:
            target_pose = self.current_target_pose()
            self.target_pub.publish(target_pose)
            self.ensure_moveit()
            trajectory = self.plan_pose(target_pose)
            names, positions = self.trajectory_points(trajectory)
            if not positions:
                raise RuntimeError("MoveItPy plan produced no joint trajectory points")
            self.joint_names = names
            self.preview_points = self.preview_positions(positions)
            self.preview_index = 0
            p = target_pose.pose.position
            self.get_logger().info(
                "MoveIt target XYZ preview ready: "
                f"x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} frames={len(self.preview_points)}"
            )
        except TransformException as exc:
            self.planning_failed = True
            self.get_logger().error(f"No current gripper TF; cannot keep side-grasp orientation: {exc}")
        except Exception as exc:
            self.planning_failed = True
            self.get_logger().error(f"MoveIt target XYZ preview failed closed: {exc}")

    def plan_pose(self, pose_stamped: PoseStamped):
        from moveit.planning import PlanRequestParameters

        request_parameters = PlanRequestParameters(self.moveit_py)
        request_parameters.planning_time = float(self.get_parameter("planning_timeout_sec").value)
        request_parameters.planning_pipeline = str(self.get_parameter("planning_pipeline").value)
        request_parameters.planner_id = str(self.get_parameter("planner_id").value)
        request_parameters.planning_attempts = int(self.get_parameter("planning_attempts").value)
        request_parameters.max_velocity_scaling_factor = float(
            self.get_parameter("max_velocity_scaling_factor").value
        )
        request_parameters.max_acceleration_scaling_factor = float(
            self.get_parameter("max_acceleration_scaling_factor").value
        )
        self.planning_component.set_start_state_to_current_state()
        self.planning_component.set_goal_state(
            pose_stamped_msg=pose_stamped,
            pose_link=str(self.get_parameter("ee_link").value),
        )
        solution = self.planning_component.plan(request_parameters)
        if not solution:
            p = pose_stamped.pose.position
            raise RuntimeError(f"MoveItPy plan failed for target x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}")
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

    def interpolate_positions(self, start: List[float], end: List[float]) -> List[List[float]]:
        steps = max(int(self.get_parameter("frames_per_step").value), 2)
        hold_frames = max(int(self.get_parameter("hold_frames").value), 0)
        size = min(len(start), len(end))
        frames: List[List[float]] = []
        for step in range(steps):
            t = step / max(steps - 1, 1)
            smooth = 0.5 - 0.5 * math.cos(math.pi * t)
            frames.append([start[index] + (end[index] - start[index]) * smooth for index in range(size)])
        frames.extend([end[:size]] * hold_frames)
        return frames

    def preview_positions(self, positions: List[List[float]]) -> List[List[float]]:
        hold_frames = max(int(self.get_parameter("hold_frames").value), 0)
        frames = [list(point) for point in positions]
        if not frames:
            return []
        frames.extend([frames[-1][:]] * hold_frames)
        return frames


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetXyzMoveItPreviewNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
