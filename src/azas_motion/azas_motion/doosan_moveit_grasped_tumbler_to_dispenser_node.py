#!/usr/bin/env python3
"""Move an already side-grasped tumbler to the dispenser front in Doosan MoveIt."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import CollisionObject
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener

from .dispenser_targets import Position
from .dispenser_targets import dispenser_front_targets
from .dispenser_targets import hold_position
from .dispenser_targets import nearest_dispenser_order
from .dispenser_targets import parse_outlet_positions
from .dispenser_targets import safe_dispenser_transfer_targets
from .dispenser_targets import selected_outlet
from .rviz_target_markers import RvizTargetMarkers
from .side_grasp_ik_preview_node import moveit_config_dict


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class DoosanMoveItGraspedTumblerToDispenserNode(Node):
    def __init__(self) -> None:
        super().__init__("doosan_moveit_grasped_tumbler_to_dispenser_node")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("shutdown_on_complete", False)
        self.declare_parameter("execute_motion", True)
        self.declare_parameter("start_delay_sec", 14.0)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_link", "link_6")
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("robot_model", "m0609")
        self.declare_parameter("moveit_config_package", "dsr_moveit_config_m0609")
        self.declare_parameter("planning_pipeline", "pilz_industrial_motion_planner")
        self.declare_parameter("planner_id", "PTP")
        self.declare_parameter("state_planner_id", "PTP")
        self.declare_parameter("pose_planner_id", "LIN")
        self.declare_parameter("planning_timeout_sec", 3.0)
        self.declare_parameter("max_velocity_scaling_factor", 0.10)
        self.declare_parameter("max_acceleration_scaling_factor", 0.10)
        self.declare_parameter("waypoint_hold_sec", 1.5)
        self.declare_parameter("controller_action_name", "/dsr_moveit_controller/follow_joint_trajectory")
        self.declare_parameter("controller_action_wait_sec", 90.0)
        self.declare_parameter("execution_backend", "controller_action")
        self.declare_parameter("moveit_ready_wait_sec", 5.0)
        self.declare_parameter("pose_skip_position_tolerance", 0.035)
        self.declare_parameter("max_single_segment_joint_motion_deg", 170.0)
        self.declare_parameter("max_commanded_joint_velocity_deg_s", 120.0)
        self.declare_parameter("task_mode", "dispenser_front")
        self.declare_parameter("assume_already_at_side_grip", False)
        self.declare_parameter("selected_dispenser_id", 2)
        self.declare_parameter("dispenser_sequence_ids", [1, 2, 3, 4])
        self.declare_parameter("sort_dispenser_sequence_by_distance", True)
        self.declare_parameter("side_grasp_joints_deg", [159.0, -43.0, -105.0, -81.0, 85.0, 31.0])
        self.declare_parameter("joint_1_deg", 159.0)
        self.declare_parameter("joint_2_deg", -43.0)
        self.declare_parameter("joint_3_deg", -105.0)
        self.declare_parameter("joint_4_deg", -81.0)
        self.declare_parameter("joint_5_deg", 85.0)
        self.declare_parameter("joint_6_deg", 31.0)
        self.declare_parameter("front_approach_offset_x", 0.12)
        self.declare_parameter("outlet_front_offset_x", 0.02)
        self.declare_parameter("transfer_z_override", 0.20)
        self.declare_parameter("enable_safe_lift_transfer", True)
        self.declare_parameter("safe_lift_min_z", 0.40)
        self.declare_parameter("safe_lift_delta_z", 0.15)
        self.declare_parameter("safe_lift_max_z", 0.55)
        self.declare_parameter("dispenser_above_z", 0.40)
        self.declare_parameter("enable_demo_obstacle", True)
        self.declare_parameter("enable_obstacle_detour", True)
        self.declare_parameter("obstacle_size_xyz", [0.10, 0.14, 0.08])
        self.declare_parameter("obstacle_position_xyz", [0.36, 0.00, 0.22])
        self.declare_parameter("detour_y", -0.24)
        self.declare_parameter("floor_target_x", 0.42)
        self.declare_parameter("floor_target_y", -0.22)
        self.declare_parameter("floor_target_z", 0.20)
        self.declare_parameter("floor_approach_z", 0.28)
        self.declare_parameter("allow_dispenser_orientation_fallback", True)
        self.declare_parameter(
            "dispenser_outlet_positions",
            [
                0.60,
                0.08,
                0.392,
                0.60,
                0.02,
                0.392,
                0.60,
                -0.04,
                0.392,
                0.60,
                -0.10,
                0.392,
            ],
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_joint_positions: dict[str, float] = {}
        self.joint_state_subscription = self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 10
        )
        self.obstacle_publisher = None
        self.target_markers = RvizTargetMarkers(self, str(self.get_parameter("frame_id").value))
        self.moveit_robot = None
        self.start_time = self.get_clock().now()
        self.started = False
        self.timer = self.create_timer(0.25, self.on_timer)
        self.get_logger().info("Ready: side-grasped tumbler -> dispenser front executor.")

    def on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_positions.update(
            {
                name: position
                for name, position in zip(msg.name, msg.position)
                if name in JOINT_NAMES
            }
        )

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
            from moveit.planning import MoveItPy

            robot_model = str(self.get_parameter("robot_model").value)
            moveit_config_package = str(self.get_parameter("moveit_config_package").value)
            group_name = str(self.get_parameter("planning_group").value)
            execute_motion = bool(self.get_parameter("execute_motion").value)
            task_mode = str(self.get_parameter("task_mode").value)

            robot = MoveItPy(
                node_name="azas_grasped_tumbler_to_dispenser",
                config_dict=moveit_config_dict(robot_model, moveit_config_package),
                provide_planning_service=False,
            )
            self.moveit_robot = robot
            self.wait_for_moveit_ready()
            if execute_motion:
                self.get_logger().info(
                    "Using MoveIt trajectory execution through dsr_moveit_controller."
                )
            arm = robot.get_planning_component(group_name)
            params = self.plan_params(robot)
            if bool(self.get_parameter("enable_demo_obstacle").value):
                self.publish_demo_obstacle()
                time.sleep(0.5)

            side_state = self.side_grasp_state(robot, group_name)
            assume_already_at_side_grip = bool(
                self.get_parameter("assume_already_at_side_grip").value
            )
            if assume_already_at_side_grip:
                self.get_logger().info(
                    "Skipping side-grip motion; using current state as already-grasped start."
                )
            else:
                self.plan_and_maybe_execute_state(
                    robot, arm, params, "assumed_side_grasp", side_state, execute_motion
                )
                time.sleep(0.35)

            if task_mode in {"side_grip", "side_grip_hold", "hold_side_grip"}:
                self.get_logger().info("DONE: robot is holding the assumed side-grip posture.")
                return

            current_pose = self.current_link_pose(side_state, execute_motion)
            for label, pose in self.target_poses(task_mode, current_pose):
                if execute_motion and self.is_current_pose_close(label, pose, side_state):
                    continue
                try:
                    self.plan_and_maybe_execute_pose(robot, arm, params, label, pose, execute_motion)
                except RuntimeError:
                    if not bool(self.get_parameter("allow_dispenser_orientation_fallback").value):
                        raise
                    self.get_logger().warning(
                        f"{label} failed with side-grasp orientation; retrying dispenser-facing pose."
                    )
                    fallback = self.dispenser_facing_pose(pose)
                    self.plan_and_maybe_execute_pose(
                        robot, arm, params, f"{label}_dispenser_facing", fallback, execute_motion
                    )
                time.sleep(float(self.get_parameter("waypoint_hold_sec").value))

            if task_mode == "floor":
                self.get_logger().info("DONE: side-grasped tumbler moved to demo floor target.")
            elif task_mode in {"dispenser_sequence", "all_dispensers"}:
                self.get_logger().info(
                    "DONE: grasped tumbler visited dispenser sequence while staying side-grasped."
                )
            else:
                self.get_logger().info("DONE: grasped tumbler moved to dispenser front.")
        except Exception as exc:
            self.get_logger().error(f"FAILED: grasped tumbler dispenser transfer failed: {exc}")
        finally:
            if bool(self.get_parameter("shutdown_on_complete").value):
                rclpy.shutdown()

    def plan_params(self, robot):
        from moveit.planning import PlanRequestParameters

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
        return params

    def wait_for_moveit_ready(self) -> None:
        wait_sec = float(self.get_parameter("moveit_ready_wait_sec").value)
        if wait_sec > 0.0:
            time.sleep(wait_sec)
        self.get_logger().info(f"MoveItPy startup wait complete: {wait_sec:.1f}s")

    def side_grasp_state(self, robot, group_name: str) -> RobotState:
        joints_deg = [
            float(self.get_parameter(f"joint_{index}_deg").value)
            for index in range(1, len(JOINT_NAMES) + 1)
        ]
        if len(joints_deg) != len(JOINT_NAMES):
            raise ValueError("side_grasp_joints_deg must contain 6 values")
        state = RobotState(robot.get_robot_model())
        state.joint_positions = {
            name: math.radians(value) for name, value in zip(JOINT_NAMES, joints_deg)
        }
        state.update()
        self.get_logger().info(
            "Using assumed side-grasp joints deg: "
            + ", ".join(f"{name}={value:.1f}" for name, value in zip(JOINT_NAMES, joints_deg))
        )
        return state

    def current_link_pose(self, side_state: RobotState, execute_motion: bool):
        if not execute_motion:
            return side_state.get_pose(str(self.get_parameter("ee_link").value))

        frame_id = str(self.get_parameter("frame_id").value)
        ee_link = str(self.get_parameter("ee_link").value)
        deadline = time.monotonic() + 2.5
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
        self.get_logger().warning(f"TF lookup failed after side grasp; using FK pose: {last_error}")
        return side_state.get_pose(ee_link)

    def is_current_pose_close(self, label: str, target: PoseStamped, fallback_state: RobotState) -> bool:
        current = self.current_link_pose(fallback_state, execute_motion=True)
        distance = self.position_distance(current, target.pose)
        tolerance = float(self.get_parameter("pose_skip_position_tolerance").value)
        self.get_logger().info(
            f"{label} current TCP distance to target={distance:.3f}m "
            f"(skip tolerance={tolerance:.3f}m)"
        )
        if distance > tolerance:
            return False
        self.get_logger().info(
            f"Skipping {label}: target is already at the current side-grip TCP pose."
        )
        return True

    @staticmethod
    def position_distance(first, second) -> float:
        return math.sqrt(
            (first.position.x - second.position.x) ** 2
            + (first.position.y - second.position.y) ** 2
            + (first.position.z - second.position.z) ** 2
        )

    def dispenser_front_poses(self, current_pose) -> list[tuple[str, PoseStamped]]:
        outlets = self.outlet_positions()
        selected_id = int(self.get_parameter("selected_dispenser_id").value)
        outlet = selected_outlet(outlets, selected_id)
        return self.dispenser_front_poses_for_outlet(selected_id, outlet, current_pose, prefix="")

    def dispenser_front_poses_for_outlet(
        self,
        dispenser_id: int,
        outlet: Position,
        current_pose,
        prefix: str,
    ) -> list[tuple[str, PoseStamped]]:
        targets = dispenser_front_targets(
            dispenser_id,
            outlet,
            front_approach_offset_x=float(self.get_parameter("front_approach_offset_x").value),
            outlet_front_offset_x=float(self.get_parameter("outlet_front_offset_x").value),
            transfer_z_override=float(self.get_parameter("transfer_z_override").value),
            detour_y=float(self.get_parameter("detour_y").value),
            enable_obstacle_detour=bool(self.get_parameter("enable_obstacle_detour").value),
            prefix=prefix,
        )
        if bool(self.get_parameter("enable_safe_lift_transfer").value):
            targets = self.safe_dispenser_transfer_targets(
                dispenser_id, outlet, current_pose, prefix, include_initial_lift=True
            )
        hold = hold_position(
            outlet,
            outlet_front_offset_x=float(self.get_parameter("outlet_front_offset_x").value),
            transfer_z_override=float(self.get_parameter("transfer_z_override").value),
        )
        self.target_markers.publish_dispenser_target(
            self.outlet_positions(), hold, dispenser_id
        )
        self.get_logger().info(
            "Published RViz dispenser markers on /azas/dispenser_target_marker "
            f"(selected dispenser={dispenser_id}, front hold "
            f"x={hold.x:.3f} y={hold.y:.3f} z={hold.z:.3f})"
        )
        return [
            (target.label, self.pose(target.position.x, target.position.y, target.position.z, current_pose))
            for target in targets
        ]

    def safe_dispenser_transfer_targets(
        self,
        dispenser_id: int,
        outlet: Position,
        current_pose,
        prefix: str,
        *,
        include_initial_lift: bool,
    ):
        current_position = Position(
            current_pose.position.x, current_pose.position.y, current_pose.position.z
        )
        targets = safe_dispenser_transfer_targets(
            dispenser_id,
            outlet,
            current_position,
            outlet_front_offset_x=float(self.get_parameter("outlet_front_offset_x").value),
            transfer_z_override=float(self.get_parameter("transfer_z_override").value),
            safe_lift_min_z=float(self.get_parameter("safe_lift_min_z").value),
            safe_lift_delta_z=float(self.get_parameter("safe_lift_delta_z").value),
            safe_lift_max_z=float(self.get_parameter("safe_lift_max_z").value),
            dispenser_above_z=float(self.get_parameter("dispenser_above_z").value),
            include_initial_lift=include_initial_lift,
            prefix=prefix,
        )
        self.get_logger().info(
            "Using safe-lift dispenser transfer "
            f"(safe_lift_min_z={float(self.get_parameter('safe_lift_min_z').value):.3f}, "
            f"dispenser_above_z={float(self.get_parameter('dispenser_above_z').value):.3f})"
        )
        return targets

    def target_poses(self, task_mode: str, current_pose) -> list[tuple[str, PoseStamped]]:
        if task_mode in {"", "dispenser", "dispenser_front"}:
            return self.dispenser_front_poses(current_pose)
        if task_mode in {"dispenser_sequence", "all_dispensers"}:
            return self.dispenser_sequence_poses(current_pose)
        if task_mode in {"floor", "floor_target", "demo_floor"}:
            return self.floor_target_poses(current_pose)
        raise ValueError(
            "task_mode must be one of: dispenser_front, dispenser_sequence, floor, side_grip_hold"
        )

    def dispenser_sequence_poses(self, current_pose) -> list[tuple[str, PoseStamped]]:
        outlets = self.outlet_positions()
        sequence_ids = [int(value) for value in self.get_parameter("dispenser_sequence_ids").value]
        if not sequence_ids:
            sequence_ids = list(range(1, len(outlets) + 1))
        if bool(self.get_parameter("sort_dispenser_sequence_by_distance").value):
            current_position = Position(
                current_pose.position.x, current_pose.position.y, current_pose.position.z
            )
            ordered = nearest_dispenser_order(
                sequence_ids,
                outlets,
                current_position,
                outlet_front_offset_x=float(self.get_parameter("outlet_front_offset_x").value),
                transfer_z_override=float(self.get_parameter("transfer_z_override").value),
            )
            if ordered != sequence_ids:
                self.get_logger().info(
                    "Reordered dispenser sequence by current TCP distance: "
                    + " -> ".join(str(dispenser_id) for dispenser_id in ordered)
                )
            sequence_ids = ordered
        poses: list[tuple[str, PoseStamped]] = []
        for sequence_index, dispenser_id in enumerate(sequence_ids, start=1):
            if dispenser_id < 1 or dispenser_id > len(outlets):
                raise ValueError(
                    f"dispenser_sequence_ids contains {dispenser_id}, "
                    f"but valid range is 1..{len(outlets)}"
                )
            outlet = outlets[dispenser_id - 1]
            if bool(self.get_parameter("enable_safe_lift_transfer").value):
                targets = self.safe_dispenser_transfer_targets(
                    dispenser_id,
                    outlet,
                    current_pose,
                    prefix=f"seq_{sequence_index}_",
                    include_initial_lift=sequence_index == 1,
                )
                hold = hold_position(
                    outlet,
                    outlet_front_offset_x=float(self.get_parameter("outlet_front_offset_x").value),
                    transfer_z_override=float(self.get_parameter("transfer_z_override").value),
                )
                self.target_markers.publish_dispenser_target(
                    self.outlet_positions(), hold, dispenser_id
                )
                poses.extend(
                    [
                        (
                            target.label,
                            self.pose(
                                target.position.x,
                                target.position.y,
                                target.position.z,
                                current_pose,
                            ),
                        )
                        for target in targets
                    ]
                )
            else:
                poses.extend(
                    self.dispenser_front_poses_for_outlet(
                        dispenser_id,
                        outlet,
                        current_pose,
                        prefix=f"seq_{sequence_index}_",
                    )
                )
        self.target_markers.publish_dispenser_sequence_path(poses)
        self.get_logger().info(
            "Using dispenser sequence while keeping assumed side grip: "
            + " -> ".join(str(dispenser_id) for dispenser_id in sequence_ids)
        )
        return poses

    def floor_target_poses(self, current_pose) -> list[tuple[str, PoseStamped]]:
        floor_x = float(self.get_parameter("floor_target_x").value)
        floor_y = float(self.get_parameter("floor_target_y").value)
        floor_z = float(self.get_parameter("floor_target_z").value)
        approach_z = float(self.get_parameter("floor_approach_z").value)
        if floor_z < 0.18:
            self.get_logger().warning(
                f"floor_target_z={floor_z:.3f} is below demo safety floor; clamping to 0.180"
            )
            floor_z = 0.18
        if approach_z < floor_z + 0.05:
            approach_z = floor_z + 0.05
        self.get_logger().info(
            "Using RViz demo floor target "
            f"x={floor_x:.3f} y={floor_y:.3f} z={floor_z:.3f}, approach_z={approach_z:.3f}"
        )
        self.target_markers.publish_floor_target(floor_x, floor_y, floor_z, approach_z)
        self.get_logger().info(
            "Published RViz floor target markers on /azas/floor_target_marker "
            f"(red touch x={floor_x:.3f} y={floor_y:.3f} z={floor_z:.3f}, "
            f"blue approach z={approach_z:.3f})"
        )
        return [
            ("floor_approach", self.pose(floor_x, floor_y, approach_z, current_pose)),
            ("floor_touch", self.pose(floor_x, floor_y, floor_z, current_pose)),
        ]

    def pose(self, x: float, y: float, z: float, orientation_source) -> PoseStamped:
        stamped = PoseStamped()
        stamped.header.frame_id = str(self.get_parameter("frame_id").value)
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose.position.x = x
        stamped.pose.position.y = y
        stamped.pose.position.z = z
        stamped.pose.orientation = orientation_source.orientation
        return stamped

    def publish_demo_obstacle(self) -> None:
        frame_id = str(self.get_parameter("frame_id").value)
        size = [float(value) for value in self.get_parameter("obstacle_size_xyz").value]
        position = [float(value) for value in self.get_parameter("obstacle_position_xyz").value]
        if len(size) != 3 or len(position) != 3:
            raise ValueError("obstacle_size_xyz and obstacle_position_xyz must contain 3 values")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.obstacle_publisher = self.create_publisher(CollisionObject, "/collision_object", qos)

        obstacle = CollisionObject()
        obstacle.id = "azas_demo_obstacle_between_cup_and_dispenser"
        obstacle.header.frame_id = frame_id
        obstacle.header.stamp = self.get_clock().now().to_msg()

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0

        obstacle.primitives.append(primitive)
        obstacle.primitive_poses.append(pose)
        obstacle.operation = CollisionObject.ADD
        self.obstacle_publisher.publish(obstacle)
        self.get_logger().info(
            "Published RViz demo obstacle "
            f"size=({size[0]:.2f}, {size[1]:.2f}, {size[2]:.2f}) "
            f"pos=({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})"
        )

    def outlet_positions(self) -> list[Position]:
        return parse_outlet_positions(self.get_parameter("dispenser_outlet_positions").value)

    def dispenser_facing_pose(self, pose: PoseStamped) -> PoseStamped:
        stamped = PoseStamped()
        stamped.header = pose.header
        stamped.pose.position = pose.pose.position
        stamped.pose.orientation.x = 0.0
        stamped.pose.orientation.y = 1.0
        stamped.pose.orientation.z = 0.0
        stamped.pose.orientation.w = 0.0
        return stamped

    def plan_and_maybe_execute_state(self, robot, arm, params, label: str, state: RobotState, execute: bool) -> None:
        params.planner_id = str(self.get_parameter("state_planner_id").value)
        arm.set_start_state_to_current_state()
        arm.set_goal_state(robot_state=state)
        self.get_logger().info(f"Planning {label} with planner_id={params.planner_id}")
        result = arm.plan(params)
        if not result:
            raise RuntimeError(f"planning failed at {label}")
        if execute:
            self.get_logger().info(f"Executing {label}")
            self.execute_trajectory(robot, result.trajectory, label)

    def plan_and_maybe_execute_pose(self, robot, arm, params, label: str, pose: PoseStamped, execute: bool) -> None:
        params.planner_id = str(self.get_parameter("pose_planner_id").value)
        arm.set_start_state_to_current_state()
        arm.set_goal_state(
            pose_stamped_msg=pose,
            pose_link=str(self.get_parameter("ee_link").value),
        )
        p = pose.pose.position
        self.get_logger().info(
            f"Planning {label} with planner_id={params.planner_id}: "
            f"x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
        )
        result = arm.plan(params)
        if not result:
            self.get_logger().warning(
                f"{label} pose plan failed with planner_id={params.planner_id}; "
                "retrying as nearest seeded IK joint goal with PTP."
            )
            self.plan_and_maybe_execute_nearest_ik_state(robot, arm, params, label, pose, execute)
            return
        if execute:
            self.get_logger().info(f"Executing {label}")
            self.execute_trajectory(robot, result.trajectory, label)

    def plan_and_maybe_execute_nearest_ik_state(
        self, robot, arm, params, label: str, pose: PoseStamped, execute: bool
    ) -> None:
        state = self.current_seed_state(robot)
        ok = state.set_from_ik(
            joint_model_group_name=str(self.get_parameter("planning_group").value),
            geometry_pose=pose.pose,
            tip_name=str(self.get_parameter("ee_link").value),
        )
        if ok is False:
            raise RuntimeError(f"seeded IK failed at {label}")
        state.update()
        self.plan_and_maybe_execute_state(robot, arm, params, f"{label}_nearest_ik_ptp", state, execute)

    def current_seed_state(self, robot) -> RobotState:
        state = RobotState(robot.get_robot_model())
        if all(name in self.latest_joint_positions for name in JOINT_NAMES):
            state.joint_positions = {
                name: self.latest_joint_positions[name] for name in JOINT_NAMES
            }
        state.update()
        return state

    def execute_trajectory(self, robot, trajectory, label: str) -> None:
        backend = str(self.get_parameter("execution_backend").value)
        if backend == "moveit":
            ok = robot.execute(
                group_name=str(self.get_parameter("planning_group").value),
                robot_trajectory=trajectory,
                blocking=True,
            )
            if ok is False:
                raise RuntimeError(f"MoveIt execution failed at {label}")
            return
        if backend != "controller_action":
            raise RuntimeError(f"unsupported execution_backend: {backend}")
        self.send_controller_trajectory(trajectory, label)

    def send_controller_trajectory(self, trajectory, label: str) -> None:
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
    node = DoosanMoveItGraspedTumblerToDispenserNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
