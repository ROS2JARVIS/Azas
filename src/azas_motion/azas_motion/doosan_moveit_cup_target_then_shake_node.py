#!/usr/bin/env python3
"""Execute the Azas cup-target and shake demo on the official Doosan MoveIt stack.

This follows the course-material pattern: MoveItPy plans a target pose and then
calls robot.execute(). It is intended for the Doosan virtual MoveIt stack
launched with mode:=virtual, not for real robot validation.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from .side_grasp_ik_preview_node import moveit_config_dict

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class DoosanMoveItCupTargetThenShakeNode(Node):
    def __init__(self) -> None:
        super().__init__("doosan_moveit_cup_target_then_shake_node")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("execute_motion", True)
        self.declare_parameter("start_delay_sec", 14.0)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_link", "link_6")
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("robot_model", "m0609")
        self.declare_parameter("moveit_config_package", "dsr_moveit_config_m0609")
        self.declare_parameter("planning_pipeline", "pilz_industrial_motion_planner")
        self.declare_parameter("planner_id", "PTP")
        self.declare_parameter("planning_timeout_sec", 3.0)
        self.declare_parameter("max_velocity_scaling_factor", 0.10)
        self.declare_parameter("max_acceleration_scaling_factor", 0.10)
        self.declare_parameter("controller_action_name", "/dsr_moveit_controller/follow_joint_trajectory")
        self.declare_parameter("controller_action_wait_sec", 20.0)
        self.declare_parameter("max_single_segment_joint_motion_deg", 170.0)
        self.declare_parameter("max_commanded_joint_velocity_deg_s", 120.0)
        self.declare_parameter("shake_mode", "pose")
        self.declare_parameter("lift_before_safe_shake_space", False)
        self.declare_parameter("move_to_safe_shake_space", False)
        self.declare_parameter("move_to_initial_side_grip", False)
        self.declare_parameter("move_to_detected_cup", False)
        self.declare_parameter("cup_pose_topic", "/jarvis/tumbler_dispenser/tumbler_pose")
        self.declare_parameter("cup_approach_z_offset", 0.12)
        self.declare_parameter("joint_1_deg", 119.0)
        self.declare_parameter("joint_2_deg", -41.0)
        self.declare_parameter("joint_3_deg", -120.0)
        self.declare_parameter("joint_4_deg", 32.0)
        self.declare_parameter("joint_5_deg", -103.0)
        self.declare_parameter("joint_6_deg", -137.0)
        self.declare_parameter("lift_joint_1_deg", 119.0)
        self.declare_parameter("lift_joint_2_deg", -56.0)
        self.declare_parameter("lift_joint_3_deg", -105.0)
        self.declare_parameter("lift_joint_4_deg", 32.0)
        self.declare_parameter("lift_joint_5_deg", -96.0)
        self.declare_parameter("lift_joint_6_deg", -137.0)
        self.declare_parameter("safe_joint_1_deg", 105.0)
        self.declare_parameter("safe_joint_2_deg", -45.0)
        self.declare_parameter("safe_joint_3_deg", -115.0)
        self.declare_parameter("safe_joint_4_deg", 25.0)
        self.declare_parameter("safe_joint_5_deg", -98.0)
        self.declare_parameter("safe_joint_6_deg", -125.0)
        self.declare_parameter("target_x", 0.43)
        self.declare_parameter("target_y", 0.08)
        self.declare_parameter("transport_z", 0.50)
        self.declare_parameter("shake_center_z", 0.62)
        self.declare_parameter("shake_amplitude_x", 0.03)
        self.declare_parameter("shake_amplitude_y", 0.02)
        self.declare_parameter("shake_cycles", 2)
        self.declare_parameter("relative_lift_z", 0.25)
        self.declare_parameter("safe_min_z", 0.55)
        self.declare_parameter("safe_max_z", 0.85)
        self.declare_parameter("use_fixed_safe_xy", False)

        self.latest_joint_positions: dict[str, float] = {}
        self.latest_cup_pose: PoseStamped | None = None
        self.joint_state_subscription = self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 10
        )
        self.cup_pose_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("cup_pose_topic").value),
            self.on_cup_pose,
            10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_time = self.get_clock().now()
        self.started = False
        self.timer = self.create_timer(0.25, self.on_timer)
        self.get_logger().info(
            "Ready: official Doosan MoveIt cup-target then shake executor."
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_positions.update(
            {
                name: position
                for name, position in zip(msg.name, msg.position)
                if name in JOINT_NAMES
            }
        )

    def on_cup_pose(self, msg: PoseStamped) -> None:
        frame_id = str(self.get_parameter("frame_id").value)
        if msg.header.frame_id != frame_id:
            self.get_logger().warning(
                f"Ignoring cup pose in frame '{msg.header.frame_id}', expected '{frame_id}'"
            )
            return
        self.latest_cup_pose = msg

    def on_timer(self) -> None:
        if self.started or not bool(self.get_parameter("auto_start").value):
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < float(self.get_parameter("start_delay_sec").value):
            return
        self.started = True
        self.timer.cancel()
        threading.Thread(target=self.run_sequence, daemon=True).start()

    def run_sequence(self) -> None:
        try:
            from moveit.planning import MoveItPy, PlanRequestParameters

            robot_model = str(self.get_parameter("robot_model").value)
            moveit_config_package = str(self.get_parameter("moveit_config_package").value)
            group_name = str(self.get_parameter("planning_group").value)
            execute_motion = bool(self.get_parameter("execute_motion").value)

            robot = MoveItPy(
                node_name="azas_doosan_moveit_executor",
                config_dict=moveit_config_dict(robot_model, moveit_config_package),
                provide_planning_service=False,
            )
            arm = robot.get_planning_component(group_name)
            params = PlanRequestParameters(robot)
            params.planning_pipeline = str(self.get_parameter("planning_pipeline").value)
            params.planner_id = str(self.get_parameter("planner_id").value)
            params.planning_time = float(self.get_parameter("planning_timeout_sec").value)
            params.planning_attempts = 1
            params.max_velocity_scaling_factor = float(
                self.get_parameter("max_velocity_scaling_factor").value
            )
            params.max_acceleration_scaling_factor = float(
                self.get_parameter("max_acceleration_scaling_factor").value
            )

            self.wait_for_joint_state()
            if execute_motion:
                self.wait_for_controller_action_server()

            if bool(self.get_parameter("move_to_detected_cup").value):
                current_pose = self.current_link_pose_from_tf()
                for label, pose in self.detected_cup_poses(current_pose):
                    self.plan_and_maybe_execute(robot, arm, params, label, pose, execute_motion)
                    time.sleep(0.15)

            if bool(self.get_parameter("move_to_initial_side_grip").value):
                self.plan_and_maybe_execute_state(
                    robot,
                    arm,
                    params,
                    "initial_dispenser_side_grip",
                    self.side_grip_state(robot),
                    execute_motion,
                )
                time.sleep(0.15)

            shake_mode = str(self.get_parameter("shake_mode").value)
            if shake_mode == "relative_pose":
                for label, pose in self.relative_shake_poses():
                    self.plan_and_maybe_execute(robot, arm, params, label, pose, execute_motion)
                    time.sleep(0.15)
                self.get_logger().info(
                    "DONE: collision-aware Doosan MoveIt relative-pose shake sequence."
                )
                return

            if shake_mode == "joint":
                if bool(self.get_parameter("lift_before_safe_shake_space").value):
                    self.plan_and_maybe_execute_state(
                        robot,
                        arm,
                        params,
                        "lift_clearance_from_dispenser",
                        self.lift_clearance_state(robot),
                        execute_motion,
                    )
                    time.sleep(0.15)
                if bool(self.get_parameter("move_to_safe_shake_space").value):
                    self.plan_and_maybe_execute_state(
                        robot,
                        arm,
                        params,
                        "safe_shake_space",
                        self.safe_shake_state(robot),
                        execute_motion,
                    )
                    time.sleep(0.15)
                for label, state in self.joint_shake_states(robot):
                    self.plan_and_maybe_execute_state(robot, arm, params, label, state, execute_motion)
                    time.sleep(0.15)
                self.get_logger().info("DONE: collision-aware Doosan MoveIt joint-shake sequence.")
                return

            for label, pose in self.sequence_poses():
                try:
                    self.plan_and_maybe_execute(robot, arm, params, label, pose, execute_motion)
                except RuntimeError:
                    if label.startswith("target_") or label.startswith("shake_center_start"):
                        raise
                    self.get_logger().warning(f"Skipping unreachable shake offset: {label}")
                time.sleep(0.15)
            self.get_logger().info("DONE: official Doosan MoveIt cup-target then shake sequence.")
        except Exception as exc:
            self.get_logger().error(f"FAILED: official Doosan MoveIt sequence failed: {exc}")

    def wait_for_joint_state(self) -> None:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if all(name in self.latest_joint_positions for name in JOINT_NAMES):
                return
            time.sleep(0.05)
        missing = [name for name in JOINT_NAMES if name not in self.latest_joint_positions]
        raise RuntimeError(f"missing /joint_states for joints: {missing}")

    def wait_for_controller_action_server(self) -> None:
        action_name = str(self.get_parameter("controller_action_name").value)
        wait_sec = float(self.get_parameter("controller_action_wait_sec").value)
        client = ActionClient(self, FollowJointTrajectory, action_name)
        if not client.wait_for_server(timeout_sec=wait_sec):
            raise RuntimeError(
                f"controller action server not ready after {wait_sec:.1f}s: {action_name}"
            )
        self.get_logger().info(f"Controller action server ready: {action_name}")
        client.destroy()

    def wait_for_cup_pose(self) -> PoseStamped:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self.latest_cup_pose is not None:
                return self.latest_cup_pose
            time.sleep(0.05)
        topic = str(self.get_parameter("cup_pose_topic").value)
        raise RuntimeError(f"move_to_detected_cup=true but no PoseStamped received on {topic}")

    def current_link_pose_from_tf(self):
        frame_id = str(self.get_parameter("frame_id").value)
        ee_link = str(self.get_parameter("ee_link").value)
        deadline = time.monotonic() + 3.0
        last_error = None
        while time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    frame_id,
                    ee_link,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
                pose = PoseStamped().pose
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                return pose
            except TransformException as exc:
                last_error = exc
                time.sleep(0.05)
        raise RuntimeError(f"failed to read current {frame_id}->{ee_link} TF: {last_error}")

    def detected_cup_poses(self, current_pose) -> list[tuple[str, PoseStamped]]:
        cup_pose = self.wait_for_cup_pose()
        approach_z = cup_pose.pose.position.z + float(
            self.get_parameter("cup_approach_z_offset").value
        )
        self.get_logger().info(
            "Using detected cup pose from "
            f"{self.get_parameter('cup_pose_topic').value}: "
            f"x={cup_pose.pose.position.x:.3f} y={cup_pose.pose.position.y:.3f} "
            f"z={cup_pose.pose.position.z:.3f}; approach_z={approach_z:.3f}"
        )
        return [
            (
                "detected_cup_approach",
                self.pose_with_orientation(
                    cup_pose.pose.position.x,
                    cup_pose.pose.position.y,
                    approach_z,
                    current_pose,
                ),
            ),
            (
                "detected_cup_side_grip",
                self.pose_with_orientation(
                    cup_pose.pose.position.x,
                    cup_pose.pose.position.y,
                    cup_pose.pose.position.z,
                    current_pose,
                ),
            ),
        ]

    def relative_shake_poses(self) -> list[tuple[str, PoseStamped]]:
        current = self.current_link_pose_from_tf()
        relative_lift_z = float(self.get_parameter("relative_lift_z").value)
        safe_min_z = float(self.get_parameter("safe_min_z").value)
        safe_max_z = float(self.get_parameter("safe_max_z").value)
        if safe_max_z < safe_min_z:
            safe_max_z = safe_min_z
        requested_z = current.position.z if current.position.z >= safe_min_z else current.position.z + relative_lift_z
        center_z = min(max(requested_z, safe_min_z), safe_max_z)
        use_fixed_xy = bool(self.get_parameter("use_fixed_safe_xy").value)
        center_x = float(self.get_parameter("target_x").value) if use_fixed_xy else current.position.x
        center_y = float(self.get_parameter("target_y").value) if use_fixed_xy else current.position.y
        amp_x = float(self.get_parameter("shake_amplitude_x").value)
        amp_y = float(self.get_parameter("shake_amplitude_y").value)
        cycles = max(int(self.get_parameter("shake_cycles").value), 1)

        self.get_logger().info(
            "Using current held-cup pose for shake: "
            f"current link_6 x={current.position.x:.3f} y={current.position.y:.3f} "
            f"z={current.position.z:.3f}; shake center x={center_x:.3f} "
            f"y={center_y:.3f} z={center_z:.3f} "
            f"(safe_min_z={safe_min_z:.3f}, safe_max_z={safe_max_z:.3f})"
        )

        poses = []
        if abs(center_z - current.position.z) > 0.01:
            poses.append(
                (
                    "relative_lift_clearance",
                    self.pose_with_orientation(
                        current.position.x, current.position.y, center_z, current
                    ),
                )
            )
        poses.append(
            (
                "relative_safe_shake_center",
                self.pose_with_orientation(center_x, center_y, center_z, current),
            )
        )
        for cycle in range(1, cycles + 1):
            poses.extend(
                [
                    (
                        f"shake_{cycle}_x_plus",
                        self.pose_with_orientation(center_x + amp_x, center_y, center_z, current),
                    ),
                    (
                        f"shake_{cycle}_x_minus",
                        self.pose_with_orientation(center_x - amp_x, center_y, center_z, current),
                    ),
                    (
                        f"shake_{cycle}_y_plus",
                        self.pose_with_orientation(center_x, center_y + amp_y, center_z, current),
                    ),
                    (
                        f"shake_{cycle}_y_minus",
                        self.pose_with_orientation(center_x, center_y - amp_y, center_z, current),
                    ),
                    (
                        f"shake_{cycle}_center",
                        self.pose_with_orientation(center_x, center_y, center_z, current),
                    ),
                ]
            )
        return poses

    def sequence_poses(self) -> list[tuple[str, PoseStamped]]:
        x = float(self.get_parameter("target_x").value)
        y = float(self.get_parameter("target_y").value)
        transport_z = float(self.get_parameter("transport_z").value)
        shake_z = float(self.get_parameter("shake_center_z").value)
        amp_x = float(self.get_parameter("shake_amplitude_x").value)
        amp_y = float(self.get_parameter("shake_amplitude_y").value)
        cycles = max(int(self.get_parameter("shake_cycles").value), 1)

        poses = [
            ("target_transport", self.pose(x, y, transport_z)),
            ("shake_center_start", self.pose(x, y, shake_z)),
        ]
        for index in range(cycles):
            cycle = index + 1
            poses.extend(
                [
                    (f"shake_{cycle}_x_plus", self.pose(x + amp_x, y, shake_z)),
                    (f"shake_{cycle}_x_minus", self.pose(x - amp_x, y, shake_z)),
                    (f"shake_{cycle}_y_plus", self.pose(x, y + amp_y, shake_z)),
                    (f"shake_{cycle}_y_minus", self.pose(x, y - amp_y, shake_z)),
                    (f"shake_{cycle}_center", self.pose(x, y, shake_z)),
                ]
            )
        poses.append(("safe_retreat", self.pose(x, y, transport_z)))
        return poses

    def joint_shake_states(self, robot) -> list[tuple[str, RobotState]]:
        base_deg = self.safe_shake_joints_deg() if bool(
            self.get_parameter("move_to_safe_shake_space").value
        ) else self.side_grip_joints_deg()
        cycles = max(int(self.get_parameter("shake_cycles").value), 1)
        offsets = [
            ("shake_center_start", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ]
        for cycle in range(1, cycles + 1):
            offsets.extend(
                [
                    (f"shake_{cycle}_j1_plus", [3.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                    (f"shake_{cycle}_j1_minus", [-3.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                    (f"shake_{cycle}_j5_plus", [0.0, 0.0, 0.0, 0.0, 4.0, 0.0]),
                    (f"shake_{cycle}_j5_minus", [0.0, 0.0, 0.0, 0.0, -4.0, 0.0]),
                    (f"shake_{cycle}_center", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                ]
            )
        return [
            (label, self.joint_state(robot, [value + delta for value, delta in zip(base_deg, deltas)]))
            for label, deltas in offsets
        ]

    def side_grip_state(self, robot) -> RobotState:
        joints_deg = self.side_grip_joints_deg()
        state = self.joint_state(robot, joints_deg)
        self.get_logger().info(
            "Using initial dispenser side-grip joints deg: "
            + ", ".join(f"{name}={value:.1f}" for name, value in zip(JOINT_NAMES, joints_deg))
        )
        return state

    def safe_shake_state(self, robot) -> RobotState:
        joints_deg = self.safe_shake_joints_deg()
        state = self.joint_state(robot, joints_deg)
        self.get_logger().info(
            "Using safe shake-space joints deg: "
            + ", ".join(f"{name}={value:.1f}" for name, value in zip(JOINT_NAMES, joints_deg))
        )
        return state

    def lift_clearance_state(self, robot) -> RobotState:
        joints_deg = self.lift_clearance_joints_deg()
        state = self.joint_state(robot, joints_deg)
        self.get_logger().info(
            "Using lift clearance joints deg: "
            + ", ".join(f"{name}={value:.1f}" for name, value in zip(JOINT_NAMES, joints_deg))
        )
        return state

    def side_grip_joints_deg(self) -> list[float]:
        return [
            float(self.get_parameter(f"joint_{index}_deg").value)
            for index in range(1, len(JOINT_NAMES) + 1)
        ]

    def safe_shake_joints_deg(self) -> list[float]:
        return [
            float(self.get_parameter(f"safe_joint_{index}_deg").value)
            for index in range(1, len(JOINT_NAMES) + 1)
        ]

    def lift_clearance_joints_deg(self) -> list[float]:
        return [
            float(self.get_parameter(f"lift_joint_{index}_deg").value)
            for index in range(1, len(JOINT_NAMES) + 1)
        ]

    def joint_state(self, robot, joints_deg: list[float]) -> RobotState:
        state = RobotState(robot.get_robot_model())
        state.joint_positions = {
            name: math.radians(value) for name, value in zip(JOINT_NAMES, joints_deg)
        }
        state.update()
        return state

    def current_seed_state(self, robot) -> RobotState:
        state = RobotState(robot.get_robot_model())
        if all(name in self.latest_joint_positions for name in JOINT_NAMES):
            state.joint_positions = {
                name: self.latest_joint_positions[name] for name in JOINT_NAMES
            }
        state.update()
        return state

    def pose(self, x: float, y: float, z: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter("frame_id").value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        return pose

    def pose_with_orientation(self, x: float, y: float, z: float, orientation_source) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter("frame_id").value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = orientation_source.orientation
        return pose

    def plan_and_maybe_execute_state(
        self,
        robot,
        arm,
        params,
        label: str,
        state: RobotState,
        execute: bool,
    ) -> None:
        arm.set_start_state(robot_state=self.current_seed_state(robot))
        arm.set_goal_state(robot_state=state)
        self.get_logger().info(f"Planning {label}")
        result = arm.plan(params)
        if not result:
            raise RuntimeError(f"planning failed at {label}")
        if execute:
            self.get_logger().info(f"Executing {label}")
            self.execute_trajectory(result.trajectory, label)

    def plan_and_maybe_execute(self, robot, arm, params, label: str, pose: PoseStamped, execute: bool) -> None:
        arm.set_start_state(robot_state=self.current_seed_state(robot))
        arm.set_goal_state(
            pose_stamped_msg=pose,
            pose_link=str(self.get_parameter("ee_link").value),
        )
        p = pose.pose.position
        self.get_logger().info(
            f"Planning {label}: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
        )
        result = arm.plan(params)
        if not result:
            raise RuntimeError(f"planning failed at {label}")
        if execute:
            self.get_logger().info(f"Executing {label}")
            self.execute_trajectory(result.trajectory, label)

    def execute_trajectory(self, trajectory, label: str) -> None:
        trajectory_msg = trajectory
        if hasattr(trajectory_msg, "get_robot_trajectory_msg"):
            trajectory_msg = trajectory_msg.get_robot_trajectory_msg()
        joint_trajectory = trajectory_msg.joint_trajectory
        if not joint_trajectory.points:
            raise RuntimeError(f"empty trajectory at {label}")
        self.unwrap_joint_trajectory(joint_trajectory, label)

        action_name = str(self.get_parameter("controller_action_name").value)
        wait_sec = float(self.get_parameter("controller_action_wait_sec").value)
        client = ActionClient(self, FollowJointTrajectory, action_name)
        if not client.wait_for_server(timeout_sec=wait_sec):
            raise RuntimeError(
                f"controller action server not ready after {wait_sec:.1f}s: {action_name}"
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_trajectory
        self.validate_joint_trajectory(joint_trajectory, label)
        goal_future = client.send_goal_async(goal)
        self.wait_future(goal_future, f"send goal {label}", wait_sec)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f"controller rejected trajectory at {label}")

        result_future = goal_handle.get_result_async()
        self.wait_future(result_future, f"execute {label}", wait_sec)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"controller execution failed at {label}: code={result.error_code} {result.error_string}"
            )
        self.get_logger().info(f"Controller reached {label}")

    def unwrap_joint_trajectory(self, joint_trajectory, label: str) -> None:
        previous = [
            self.latest_joint_positions.get(name, point_position)
            for name, point_position in zip(
                joint_trajectory.joint_names,
                joint_trajectory.points[0].positions,
            )
        ]
        start_positions = list(previous)
        max_delta = 0.0
        for point in joint_trajectory.points:
            positions = list(point.positions)
            for index, name in enumerate(joint_trajectory.joint_names):
                if name not in JOINT_NAMES:
                    continue
                unwrapped = self.nearest_equivalent_angle(positions[index], previous[index])
                max_delta = max(max_delta, abs(unwrapped - previous[index]))
                positions[index] = unwrapped
                previous[index] = unwrapped
            point.positions = positions
        end_deltas = [
            abs(previous[index] - start_positions[index])
            for index, name in enumerate(joint_trajectory.joint_names)
            if name in JOINT_NAMES
        ]
        max_total = max(end_deltas, default=0.0)
        max_allowed = math.radians(
            float(self.get_parameter("max_single_segment_joint_motion_deg").value)
        )
        self.get_logger().info(
            f"Prepared continuous joint trajectory for {label}; "
            f"max adjacent joint step={math.degrees(max_delta):.1f} deg, "
            f"max start-to-end joint move={math.degrees(max_total):.1f} deg"
        )
        if max_total > max_allowed:
            raise RuntimeError(
                f"refusing {label}: joint-space branch would move "
                f"{math.degrees(max_total):.1f} deg in one segment"
            )

    def validate_joint_trajectory(self, joint_trajectory, label: str) -> None:
        max_allowed = math.radians(
            float(self.get_parameter("max_commanded_joint_velocity_deg_s").value)
        )
        max_seen = 0.0
        max_seen_joint = ""
        max_seen_index = 0
        previous_time = None
        previous_positions = None

        for point_index, point in enumerate(joint_trajectory.points):
            velocities = list(point.velocities)
            if velocities:
                for joint_index, name in enumerate(joint_trajectory.joint_names):
                    if name not in JOINT_NAMES or joint_index >= len(velocities):
                        continue
                    velocity = abs(velocities[joint_index])
                    if velocity > max_seen:
                        max_seen = velocity
                        max_seen_joint = name
                        max_seen_index = point_index

            current_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            current_positions = list(point.positions)
            if previous_time is not None and previous_positions is not None:
                dt = current_time - previous_time
                if dt <= 0.0:
                    raise RuntimeError(
                        f"refusing {label}: non-increasing trajectory time at point {point_index}"
                    )
                for joint_index, name in enumerate(joint_trajectory.joint_names):
                    if name not in JOINT_NAMES or joint_index >= len(current_positions):
                        continue
                    implied_velocity = abs(
                        (current_positions[joint_index] - previous_positions[joint_index]) / dt
                    )
                    if implied_velocity > max_seen:
                        max_seen = implied_velocity
                        max_seen_joint = name
                        max_seen_index = point_index

            previous_time = current_time
            previous_positions = current_positions

        self.get_logger().info(
            f"Validated trajectory velocity for {label}; "
            f"max={math.degrees(max_seen):.1f} deg/s "
            f"(limit={math.degrees(max_allowed):.1f} deg/s)"
        )
        if max_seen > max_allowed:
            raise RuntimeError(
                f"refusing {label}: commanded velocity for {max_seen_joint} at point "
                f"{max_seen_index} would be {math.degrees(max_seen):.1f} deg/s "
                f"(limit={math.degrees(max_allowed):.1f} deg/s)"
            )

    @staticmethod
    def nearest_equivalent_angle(angle: float, reference: float) -> float:
        while angle - reference > math.pi:
            angle -= 2.0 * math.pi
        while angle - reference < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def wait_future(future, label: str, timeout_sec: float = 60.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            raise RuntimeError(f"timeout waiting for {label}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DoosanMoveItCupTargetThenShakeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
