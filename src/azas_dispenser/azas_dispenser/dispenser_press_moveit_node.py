#!/usr/bin/env python3
import math
import time

import rclpy
from dsr_msgs2.srv import GetCurrentPosx
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient


def get_param(node, name, default):
    descriptor = ParameterDescriptor(dynamic_typing=True)
    node.declare_parameter(name, default, descriptor)
    return node.get_parameter(name).value


def service_name(prefix, name):
    clean_prefix = prefix.strip("/")
    clean_name = name.strip("/")
    if not clean_prefix:
        return f"/{clean_name}"
    return f"/{clean_prefix}/{clean_name}"


def quat_from_rpy_deg(rx, ry, rz):
    roll = math.radians(rx)
    pitch = math.radians(ry)
    yaw = math.radians(rz)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rotate_vector_by_rpy_deg(rx, ry, rz, vector):
    roll = math.radians(rx)
    pitch = math.radians(ry)
    yaw = math.radians(rz)

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    x, y, z = vector
    return (
        (cy * cp) * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z,
        (sy * cp) * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z,
        (-sp) * x + (cp * sr) * y + (cp * cr) * z,
    )


class DispenserPressMoveItNode:
    def __init__(self):
        self.node = rclpy.create_node("dispenser_press_moveit_node")
        self.logger = self.node.get_logger()

        self.group_name = str(get_param(self.node, "group_name", "manipulator"))
        self.base_frame = str(get_param(self.node, "base_frame", "base_link"))
        self.ee_link = str(get_param(self.node, "ee_link", "link_6"))
        self.ik_link_name = str(get_param(self.node, "ik_link_name", self.ee_link))
        self.tool_offset_xyz = [
            float(v)
            for v in get_param(self.node, "tool_offset_xyz", [0.0, 0.0, 0.0])
        ]
        self.service_prefix = str(get_param(self.node, "service_prefix", "/"))
        self.keep_home_pose_from_controller = bool(
            get_param(self.node, "keep_home_pose_from_controller", True)
        )
        self.joint_names = [
            str(v)
            for v in get_param(
                self.node,
                "joint_names",
                ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            )
        ]

        self.home_joints_deg = [
            float(v)
            for v in get_param(
                self.node,
                "home_joints_deg",
                [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
            )
        ]
        self.home_tcp = [
            float(v)
            for v in get_param(
                self.node,
                "home_tcp",
                [0.368, 0.00625, 0.425],
            )
        ]
        self.home_rpy_deg = [
            float(v)
            for v in get_param(
                self.node,
                "home_rpy_deg",
                [45.0, 180.0, 45.0],
            )
        ]

        self.dispenser_x = float(get_param(self.node, "dispenser_x", 0.50))
        self.dispenser_y = float(get_param(self.node, "dispenser_y", 0.00))
        self.dispenser_y_offset = float(
            get_param(self.node, "dispenser_y_offset", 0.05)
        )
        self.dispenser_top_z = float(get_param(self.node, "dispenser_top_z", 0.38))
        # Support taught PRESS poses (x, y, z, rx, ry, rz) in millimetres/degrees
        self.use_taught_posx = bool(get_param(self.node, "use_taught_posx", False))
        self.use_taught_orientation = bool(
            get_param(self.node, "use_taught_orientation", False)
        )
        self.target_dispenser = str(get_param(self.node, "target_dispenser", "red"))
        self.taught_posx_by_name = {
            "red": [float(v) for v in get_param(self.node, "red_top_posx", [])],
            "green": [float(v) for v in get_param(self.node, "green_top_posx", [])],
            "yellow": [float(v) for v in get_param(self.node, "yellow_top_posx", [])],
            "blue": [float(v) for v in get_param(self.node, "blue_top_posx", [])],
        }
        self.approach_height = float(get_param(self.node, "approach_height", 0.05))
        self.home_lift_height = float(get_param(self.node, "home_lift_height", 0.05))
        self.press_depth = float(get_param(self.node, "press_depth", 0.03))
        self.hold_seconds = float(get_param(self.node, "hold_seconds", 0.5))
        self.approach_pause_seconds = float(
            get_param(self.node, "approach_pause_seconds", 0.5)
        )

        self.allowed_planning_time = float(
            get_param(self.node, "allowed_planning_time", 5.0)
        )
        self.max_velocity_scaling = float(
            get_param(self.node, "max_velocity_scaling", 0.15)
        )
        self.max_acceleration_scaling = float(
            get_param(self.node, "max_acceleration_scaling", 0.15)
        )
        self.goal_tolerance_rad = float(get_param(self.node, "goal_tolerance_rad", 0.01))
        self.move_home_first = bool(get_param(self.node, "move_home_first", True))
        self.return_home = bool(get_param(self.node, "return_home", True))

        self.move_group = ActionClient(self.node, MoveGroup, "/move_action")
        self.compute_ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.get_current_posx = self.node.create_client(
            GetCurrentPosx,
            service_name(self.service_prefix, "aux_control/get_current_posx"),
        )

    def destroy(self):
        self.node.destroy_node()

    def wait_for_interfaces(self):
        self.logger.info("MoveIt /move_action 액션 서버를 기다리는 중")
        self.move_group.wait_for_server()
        while rclpy.ok() and not self.compute_ik.wait_for_service(timeout_sec=1.0):
            self.logger.info("MoveIt /compute_ik 서비스를 기다리는 중")
        while rclpy.ok() and not self.get_current_posx.wait_for_service(timeout_sec=1.0):
            self.logger.info("Doosan /aux_control/get_current_posx 서비스를 기다리는 중")

    def read_current_posx(self):
        request = GetCurrentPosx.Request()
        request.ref = 0

        future = self.get_current_posx.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()
        if result is None:
            self.logger.error(f"get_current_posx 호출 실패: {future.exception()}")
            return None
        if not result.success or not result.task_pos_info:
            self.logger.error("get_current_posx가 유효한 TCP 포즈를 반환하지 않았습니다")
            return None

        data = list(result.task_pos_info[0].data)
        if len(data) < 6:
            self.logger.error(f"get_current_posx가 너무 적은 값을 반환했습니다: {data}")
            return None

        pose = data[:6]
        self.logger.info(
            "Doosan TCP 포즈: "
            f"x={pose[0]:.1f}mm, y={pose[1]:.1f}mm, z={pose[2]:.1f}mm, "
            f"rpy=({pose[3]:.1f}, {pose[4]:.1f}, {pose[5]:.1f})"
        )
        return pose

    def sync_home_pose_from_controller(self):
        pose = self.read_current_posx()
        if pose is None:
            return False
        self.home_tcp = [pose[0] / 1000.0, pose[1] / 1000.0, pose[2] / 1000.0]
        self.home_rpy_deg = [pose[3], pose[4], pose[5]]
        self.logger.info(
            "Doosan TCP 포즈를 MoveIt HOME 참조로 사용합니다: "
            f"home_tcp={self.home_tcp}, home_rpy_deg={self.home_rpy_deg}"
        )
        return True

    def make_pose(self, x, y, z):
        # Default orientation: home rpy. Callers may override by passing
        # different rpy via the overloaded helper (see execute_pose_goal).
        qx, qy, qz, qw = quat_from_rpy_deg(*self.home_rpy_deg)
        offset_x, offset_y, offset_z = rotate_vector_by_rpy_deg(
            *self.home_rpy_deg, self.tool_offset_xyz
        )
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(x) - offset_x
        pose.pose.position.y = float(y) - offset_y
        pose.pose.position.z = float(z) - offset_z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def make_pose_with_rpy(self, x, y, z, rpy_deg):
        qx, qy, qz, qw = quat_from_rpy_deg(*rpy_deg)
        offset_x, offset_y, offset_z = rotate_vector_by_rpy_deg(
            *rpy_deg, self.tool_offset_xyz
        )
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(x) - offset_x
        pose.pose.position.y = float(y) - offset_y
        pose.pose.position.z = float(z) - offset_z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def solve_ik(self, pose, label):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.group_name
        ik.ik_link_name = self.ik_link_name
        ik.pose_stamped = pose
        ik.avoid_collisions = False
        ik.timeout.sec = 1

        future = self.compute_ik.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()
        if result is None:
            self.logger.error(f"{label}: /compute_ik returned no response")
            return None
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.logger.error(
                f"{label}: IK failed with MoveIt error code {result.error_code.val}"
            )
            return None

        positions_by_name = dict(
            zip(result.solution.joint_state.name, result.solution.joint_state.position)
        )
        try:
            joints = [positions_by_name[name] for name in self.joint_names]
        except KeyError as exc:
            self.logger.error(f"{label}: IK solution missing joint {exc}")
            return None
        joints_deg = [math.degrees(value) for value in joints]
        self.logger.info(
            f"{label}: IK solution deg="
            f"{[round(value, 2) for value in joints_deg]}"
        )
        return joints

    def make_joint_constraints(self, joint_positions):
        constraints = Constraints()
        for name, position in zip(self.joint_names, joint_positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = self.goal_tolerance_rad
            joint.tolerance_below = self.goal_tolerance_rad
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    def execute_joint_goal(self, joint_positions, label):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.group_name
        request.num_planning_attempts = 10
        request.allowed_planning_time = self.allowed_planning_time
        request.max_velocity_scaling_factor = self.max_velocity_scaling
        request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        request.goal_constraints.append(self.make_joint_constraints(joint_positions))

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2

        self.logger.info(f"{label}: planning and executing through MoveIt")
        future = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.logger.error(f"{label}: MoveIt action goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()
        if result is None:
            self.logger.error(f"{label}: MoveIt action returned no result")
            return False

        error_code = result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.logger.error(f"{label}: MoveIt failed with error code {error_code}")
            return False

        self.logger.info(f"{label}: MoveIt execution complete")
        return True

    def execute_pose_goal(self, x, y, z, label, rpy_deg=None):
        if rpy_deg is None:
            pose = self.make_pose(x, y, z)
            rpy_used = self.home_rpy_deg
        else:
            pose = self.make_pose_with_rpy(x, y, z, rpy_deg)
            rpy_used = rpy_deg

        self.logger.info(
            f"{label}: pose target x={x:.3f}, y={y:.3f}, z={z:.3f}, "
            f"rpy={rpy_used}, ik_link={self.ik_link_name}, "
            f"contact_link={self.ee_link}"
        )
        joints = self.solve_ik(pose, label)
        if joints is None:
            return False
        return self.execute_joint_goal(joints, label)

    def build_steps(self):
        # If taught PRESS poses are provided, use them as the authoritative
        # PRESS pose (x,y,z,rx,ry,rz) and compute APPROACH by adding
        # `approach_height` to the Z value. Values in taught_posx are expected
        # to be in millimetres for position and degrees for rpy (consistent
        # with dispenser_press_node). Convert mm -> m for MoveIt.
        steps = []
        if self.use_taught_posx:
            top_pose = self.taught_posx_by_name.get(self.target_dispenser, [])
            if len(top_pose) >= 6:
                x_m = top_pose[0] / 1000.0
                y_m = top_pose[1] / 1000.0
                top_z_m = top_pose[2] / 1000.0
                rx_deg, ry_deg, rz_deg = top_pose[3:6]
                taught_rpy = [rx_deg, ry_deg, rz_deg] if self.use_taught_orientation else None
                approach_z = top_z_m + self.approach_height
                pressed_z = top_z_m - self.press_depth

                steps = [
                    (self.home_tcp[0], self.home_tcp[1], self.home_tcp[2] + self.home_lift_height, "lift above HOME", None),
                    (x_m, y_m, approach_z, "approach above dispenser", taught_rpy),
                    (x_m, y_m, top_z_m, "move to dispenser top", taught_rpy),
                    (x_m, y_m, pressed_z, "press dispenser pump", taught_rpy),
                    (x_m, y_m, approach_z, "retreat above dispenser", taught_rpy),
                ]
                for x_value, y_value, z_value, label, _ in steps:
                    self.logger.info(
                        f"큐에 추가된 단계: {label} -> "
                        f"x={x_value:.3f}, y={y_value:.3f}, z={z_value:.3f}"
                    )
                return steps
            else:
                self.logger.info("use_taught_posx=True but taught_posx not configured; falling back to defaults")

        # Fallback: use the configured dispenser_x/dispenser_y values (meters)
        x = self.dispenser_x
        y = self.dispenser_y + self.dispenser_y_offset
        top_z = self.dispenser_top_z
        approach_z = top_z + self.approach_height
        pressed_z = top_z - self.press_depth

        steps = [
            (self.home_tcp[0], self.home_tcp[1], self.home_tcp[2] + self.home_lift_height, "lift above HOME", None),
            (x, y, approach_z, "approach above dispenser", None),
            (x, y, top_z, "move to dispenser top", None),
            (x, y, pressed_z, "press dispenser pump", None),
            (x, y, approach_z, "retreat above dispenser", None),
        ]
        for x_value, y_value, z_value, label, _ in steps:
            self.logger.info(
                f"Queued step: {label} -> "
                f"x={x_value:.3f}, y={y_value:.3f}, z={z_value:.3f}"
            )
        return steps

    def run(self):
        self.wait_for_interfaces()

        home_joints_rad = [math.radians(v) for v in self.home_joints_deg]
        if self.move_home_first:
            if not self.execute_joint_goal(home_joints_rad, "move to HOME"):
                self.logger.error("Sequence failed while moving to HOME")
                return False
            if self.keep_home_pose_from_controller:
                if not self.sync_home_pose_from_controller():
                    self.logger.error("Sequence failed while reading HOME TCP pose")
                    return False

        steps = self.build_steps()
        for idx, step in enumerate(steps, start=1):
            # support either (x,y,z,label) or (x,y,z,label,rpy)
            if len(step) == 4:
                x, y, z, label = step
                rpy = None
            else:
                x, y, z, label, rpy = step

            self.logger.info(f"단계 {idx}/{len(steps)}: 시작 '{label}' -> x={x:.3f}, y={y:.3f}, z={z:.3f}, rpy={rpy}")
            success = self.execute_pose_goal(x, y, z, label, rpy_deg=rpy)
            if not success:
                self.logger.error(f"시퀀스가 단계에서 실패했습니다: {label}")
                return False
            self.logger.info(f"단계 {idx}/{len(steps)}: 완료 '{label}'")

            if label == "approach above dispenser" and self.approach_pause_seconds > 0:
                time.sleep(self.approach_pause_seconds)
            if label == "press dispenser pump" and self.hold_seconds > 0:
                self.logger.info(f"Holding press for {self.hold_seconds:.2f} seconds")
                time.sleep(self.hold_seconds)

        if self.return_home:
            if not self.execute_joint_goal(home_joints_rad, "return to HOME"):
                self.logger.error("Sequence failed while returning to HOME")
                return False
        self.logger.info("Dispenser press MoveIt sequence finished successfully")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = DispenserPressMoveItNode()
    try:
        success = node.run()
    finally:
        node.destroy()
        rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
