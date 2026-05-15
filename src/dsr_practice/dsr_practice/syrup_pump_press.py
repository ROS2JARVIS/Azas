#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.node import Node

from .onrobot import RG


GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK = "link_6"

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = 502
GRIPPER_CLOSE_WIDTH = 0
GRIPPER_FORCE = 200

HOME_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
HOME_JOINTS_RAD = [math.radians(d) for d in HOME_JOINTS_DEG]

DOWN_ORI = {
    "x": 0.0,
    "y": 1.0,
    "z": 0.0,
    "w": 0.0,
}


class SyrupPumpPressNode(Node):
    def __init__(self):
        super().__init__("syrup_pump_press")

        self.declare_parameter("pump_x", 0.45)
        self.declare_parameter("pump_y", 0.0)
        self.declare_parameter("start_z", 0.50)
        self.declare_parameter("pump_top_z", 0.35)
        self.declare_parameter("press_depth", 0.05)
        self.declare_parameter("hold_sec", 0.5)
        self.declare_parameter("go_home_first", True)
        self.declare_parameter("return_home", False)

        self.pump_x = self.get_parameter("pump_x").value
        self.pump_y = self.get_parameter("pump_y").value
        self.start_z = self.get_parameter("start_z").value
        self.pump_top_z = self.get_parameter("pump_top_z").value
        self.press_depth = self.get_parameter("press_depth").value
        self.hold_sec = self.get_parameter("hold_sec").value
        self.go_home_first = self.get_parameter("go_home_first").value
        self.return_home = self.get_parameter("return_home").value

        self.press_z = self.pump_top_z - self.press_depth

        self.robot = MoveItPy(node_name="syrup_pump_press_moveit_py")
        self.arm = self.robot.get_planning_component(GROUP_NAME)
        self.robot_model = self.robot.get_robot_model()

        self.home_params = PlanRequestParameters(self.robot)
        self.home_params.planning_pipeline = "ompl"
        self.home_params.planner_id = "RRTConnectkConfigDefault"
        self.home_params.max_velocity_scaling_factor = 0.2
        self.home_params.max_acceleration_scaling_factor = 0.1
        self.home_params.planning_time = 3.0

        self.ptp_params = PlanRequestParameters(self.robot)
        self.ptp_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.ptp_params.planner_id = "PTP"
        self.ptp_params.max_velocity_scaling_factor = 0.15
        self.ptp_params.max_acceleration_scaling_factor = 0.1
        self.ptp_params.planning_time = 3.0

        self.lin_params = PlanRequestParameters(self.robot)
        self.lin_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.lin_params.planner_id = "LIN"
        self.lin_params.max_velocity_scaling_factor = 0.08
        self.lin_params.max_acceleration_scaling_factor = 0.05
        self.lin_params.planning_time = 3.0

        self.press_params = PlanRequestParameters(self.robot)
        self.press_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.press_params.planner_id = "LIN"
        self.press_params.max_velocity_scaling_factor = 0.02
        self.press_params.max_acceleration_scaling_factor = 0.02
        self.press_params.planning_time = 3.0

        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        time.sleep(0.5)

    def make_pose(self, x, y, z, ori=None):
        if ori is None:
            ori = DOWN_ORI

        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = ori["x"]
        pose.pose.orientation.y = ori["y"]
        pose.pose.orientation.z = ori["z"]
        pose.pose.orientation.w = ori["w"]
        return pose

    def plan_and_execute(self, pose_goal=None, state_goal=None, params=None):
        log = self.get_logger()
        self.arm.set_start_state_to_current_state()

        if pose_goal is not None:
            self.arm.set_goal_state(
                pose_stamped_msg=pose_goal,
                pose_link=EE_LINK,
            )
        elif state_goal is not None:
            self.arm.set_goal_state(robot_state=state_goal)
        else:
            log.error("No pose/state goal was provided.")
            return False

        plan_result = self.arm.plan(parameters=params) if params else self.arm.plan()
        if not plan_result:
            log.error("Planning failed.")
            return False

        self.robot.execute(
            group_name=GROUP_NAME,
            robot_trajectory=plan_result.trajectory,
            blocking=True,
        )
        return True

    def move_home(self):
        home_state = RobotState(self.robot_model)
        home_state.set_joint_group_positions(GROUP_NAME, HOME_JOINTS_RAD)
        home_state.update()
        return self.plan_and_execute(state_goal=home_state, params=self.home_params)

    def run_task(self):
        log = self.get_logger()
        log.info("=== Syrup pump press task start ===")
        log.info(
            "Target pump pose: "
            f"x={self.pump_x:.3f}, y={self.pump_y:.3f}, "
            f"start_z={self.start_z:.3f}, pump_top_z={self.pump_top_z:.3f}, "
            f"press_z={self.press_z:.3f}"
        )

        if self.press_z <= 0.0:
            log.error("Invalid press_z. Check pump_top_z and press_depth.")
            return False

        if self.go_home_first:
            log.info("[0] Move HOME")
            if not self.move_home():
                return False

        log.info("[1] Close gripper fully")
        self.gripper.move_gripper(
            width_val=GRIPPER_CLOSE_WIDTH,
            force_val=GRIPPER_FORCE,
        )
        time.sleep(1.0)

        log.info("[2] Move above syrup pump")
        if not self.plan_and_execute(
            pose_goal=self.make_pose(self.pump_x, self.pump_y, self.start_z),
            params=self.ptp_params,
        ):
            return False

        log.info("[3] Move vertically down to pump top height")
        if not self.plan_and_execute(
            pose_goal=self.make_pose(self.pump_x, self.pump_y, self.pump_top_z),
            params=self.lin_params,
        ):
            return False

        log.info("[4] Slowly press syrup pump by 5 cm")
        if not self.plan_and_execute(
            pose_goal=self.make_pose(self.pump_x, self.pump_y, self.press_z),
            params=self.press_params,
        ):
            return False

        if self.hold_sec > 0.0:
            log.info(f"[5] Hold for {self.hold_sec:.2f} sec")
            time.sleep(self.hold_sec)

        log.info("[6] Retract vertically")
        if not self.plan_and_execute(
            pose_goal=self.make_pose(self.pump_x, self.pump_y, self.start_z),
            params=self.lin_params,
        ):
            return False

        if self.return_home:
            log.info("[7] Return HOME")
            if not self.move_home():
                return False

        log.info("=== Syrup pump press task finished ===")
        return True

    def destroy_node(self):
        try:
            self.gripper.close_connection()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SyrupPumpPressNode()
    try:
        node.run_task()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
