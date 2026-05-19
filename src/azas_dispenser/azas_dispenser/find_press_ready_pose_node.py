#!/usr/bin/env python3
import math

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient


def get_param(node, name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


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
    # Rotation matrix for Rz(yaw) * Ry(pitch) * Rx(roll).
    return (
        (cy * cp) * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z,
        (sy * cp) * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z,
        (-sp) * x + (cp * sr) * y + (cp * cr) * z,
    )


class FindPressReadyPoseNode:
    def __init__(self):
        self.node = rclpy.create_node("find_press_ready_pose_node")
        self.logger = self.node.get_logger()

        self.group_name = str(get_param(self.node, "group_name", "manipulator"))
        self.base_frame = str(get_param(self.node, "base_frame", "base_link"))
        self.ee_link = str(get_param(self.node, "ee_link", "link_6"))
        self.ik_link_name = str(get_param(self.node, "ik_link_name", self.ee_link))
        self.tool_offset_xyz = [
            float(v)
            for v in get_param(self.node, "tool_offset_xyz", [0.0, 0.0, 0.0])
        ]
        self.joint_names = [
            str(v)
            for v in get_param(
                self.node,
                "joint_names",
                ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            )
        ]

        self.target_x = float(get_param(self.node, "target_x", 0.50))
        self.target_y = float(get_param(self.node, "target_y", 0.05))
        self.target_z = float(get_param(self.node, "target_z", 0.70))
        self.max_solutions = int(get_param(self.node, "max_solutions", 10))
        self.goal_tolerance_rad = float(get_param(self.node, "goal_tolerance_rad", 0.01))
        self.allowed_planning_time = float(
            get_param(self.node, "allowed_planning_time", 3.0)
        )
        self.max_velocity_scaling = float(
            get_param(self.node, "max_velocity_scaling", 0.15)
        )
        self.max_acceleration_scaling = float(
            get_param(self.node, "max_acceleration_scaling", 0.15)
        )

        self.compute_ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.move_group = ActionClient(self.node, MoveGroup, "/move_action")

    def destroy(self):
        self.node.destroy_node()

    def wait_for_interfaces(self):
        while rclpy.ok() and not self.compute_ik.wait_for_service(timeout_sec=1.0):
            self.logger.info("Waiting for MoveIt /compute_ik")
        self.logger.info("Waiting for MoveIt /move_action")
        self.move_group.wait_for_server()

    def make_pose(self, rx, ry, rz):
        qx, qy, qz, qw = quat_from_rpy_deg(rx, ry, rz)
        offset_x, offset_y, offset_z = rotate_vector_by_rpy_deg(
            rx, ry, rz, self.tool_offset_xyz
        )
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = self.target_x - offset_x
        pose.pose.position.y = self.target_y - offset_y
        pose.pose.position.z = self.target_z - offset_z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def orientation_candidates(self):
        seen = set()

        # Doosan posx has multiple equivalent-looking representations near ry=180.
        # These candidates keep the tool roughly vertical while sweeping wrist rotation.
        primary = [
            (0.0, 180.0, 0.0),
            (45.0, 180.0, 45.0),
            (90.0, 180.0, 90.0),
            (135.0, 180.0, 135.0),
            (180.0, 180.0, 180.0),
            (-45.0, 180.0, -45.0),
            (-90.0, 180.0, -90.0),
            (-135.0, 180.0, -135.0),
        ]

        for item in primary:
            key = tuple(round(v, 3) for v in item)
            seen.add(key)
            yield item

        for ry in (180.0, 170.0, 160.0, 150.0, -170.0, -160.0, -150.0):
            for angle in range(-180, 181, 30):
                item = (float(angle), float(ry), float(angle))
                key = tuple(round(v, 3) for v in item)
                if key in seen:
                    continue
                seen.add(key)
                yield item

    def solve_ik(self, pose):
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
        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        positions_by_name = dict(
            zip(result.solution.joint_state.name, result.solution.joint_state.position)
        )
        try:
            return [positions_by_name[name] for name in self.joint_names]
        except KeyError:
            return None

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

    def can_plan_to_joints(self, joint_positions):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.group_name
        request.num_planning_attempts = 5
        request.allowed_planning_time = self.allowed_planning_time
        request.max_velocity_scaling_factor = self.max_velocity_scaling
        request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        request.goal_constraints.append(self.make_joint_constraints(joint_positions))

        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        future = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, "rejected"

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()
        if result is None:
            return False, "no_result"

        error_code = result.result.error_code.val
        return error_code == MoveItErrorCodes.SUCCESS, str(error_code)

    def run(self):
        self.wait_for_interfaces()
        self.logger.info(
            "Searching press-ready TCP pose at "
            f"x={self.target_x:.3f}, y={self.target_y:.3f}, z={self.target_z:.3f}"
        )
        self.logger.info(
            f"IK target link={self.ik_link_name}, contact link={self.ee_link}, "
            f"tool_offset_xyz={self.tool_offset_xyz}"
        )

        found = []
        tested = 0
        ik_count = 0
        for rx, ry, rz in self.orientation_candidates():
            tested += 1
            pose = self.make_pose(rx, ry, rz)
            joints = self.solve_ik(pose)
            if joints is None:
                self.logger.info(
                    f"candidate {tested}: IK failed rpy=({rx:.1f}, {ry:.1f}, {rz:.1f})"
                )
                continue

            ik_count += 1
            plan_ok, plan_code = self.can_plan_to_joints(joints)
            joints_deg = [round(math.degrees(v), 2) for v in joints]
            if not plan_ok:
                self.logger.info(
                    f"candidate {tested}: IK ok, plan failed code={plan_code}, "
                    f"rpy=({rx:.1f}, {ry:.1f}, {rz:.1f}), joints_deg={joints_deg}"
                )
                continue

            found.append((rx, ry, rz, joints_deg))
            self.logger.info(
                f"FOUND #{len(found)}: rpy=({rx:.1f}, {ry:.1f}, {rz:.1f}), "
                f"press_ready_joints_deg={joints_deg}"
            )
            if len(found) >= self.max_solutions:
                break

        self.logger.info(
            f"Search done. candidates_tested={tested}, ik_success={ik_count}, "
            f"plan_success={len(found)}"
        )
        if not found:
            self.logger.error("No plan-ready press pose found. Try lowering target_z.")
            return False

        self.logger.info("Copy one FOUND joint list into the dispenser press sequence.")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = FindPressReadyPoseNode()
    try:
        success = node.run()
    finally:
        node.destroy()
        rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
