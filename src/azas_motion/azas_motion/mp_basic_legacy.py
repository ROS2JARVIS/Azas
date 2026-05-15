#!/usr/bin/env python3
"""Legacy MoveItPy basic motion example imported from dsr_practice."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger


GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK = "link_6"

SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.3
SAFE_Y_MAX = 0.3
SAFE_Z_MIN = 0.27


def clamp_to_safe_workspace(x: float, y: float, z: float, logger):
    """Clamp a requested pose target to the legacy safety workspace."""
    safe_x = x
    safe_y = y
    safe_z = z

    if safe_x < SAFE_X_MIN:
        logger.warning(
            f"Requested x ({safe_x:.3f} m) is below safety limit "
            f"({SAFE_X_MIN:.3f} m). Clamping to SAFE_X_MIN."
        )
        safe_x = SAFE_X_MIN

    if safe_y < SAFE_Y_MIN:
        logger.warning(
            f"Requested y ({safe_y:.3f} m) is below safety limit "
            f"({SAFE_Y_MIN:.3f} m). Clamping to SAFE_Y_MIN."
        )
        safe_y = SAFE_Y_MIN
    elif safe_y > SAFE_Y_MAX:
        logger.warning(
            f"Requested y ({safe_y:.3f} m) is above safety limit "
            f"({SAFE_Y_MAX:.3f} m). Clamping to SAFE_Y_MAX."
        )
        safe_y = SAFE_Y_MAX

    if safe_z < SAFE_Z_MIN:
        logger.warning(
            f"Requested z ({safe_z:.3f} m) is below safety limit "
            f"({SAFE_Z_MIN:.3f} m). Clamping to SAFE_Z_MIN."
        )
        safe_z = SAFE_Z_MIN

    return safe_x, safe_y, safe_z


def plan_and_execute(
    robot: MoveItPy,
    planning_component,
    logger,
    pose_goal: PoseStamped = None,
    plan_parameters=None,
):
    """Plan and execute a legacy MoveItPy trajectory."""
    if pose_goal is not None:
        x = pose_goal.pose.position.x
        y = pose_goal.pose.position.y
        z = pose_goal.pose.position.z

        sx, sy, sz = clamp_to_safe_workspace(x, y, z, logger)
        pose_goal.pose.position.x = sx
        pose_goal.pose.position.y = sy
        pose_goal.pose.position.z = sz

        planning_component.set_start_state_to_current_state()
        planning_component.set_goal_state(
            pose_stamped_msg=pose_goal,
            pose_link=EE_LINK,
        )

    logger.info("Planning trajectory")
    if plan_parameters is not None:
        plan_result = planning_component.plan(parameters=plan_parameters)
    else:
        plan_result = planning_component.plan()

    if not plan_result:
        logger.error("Planning failed")
        return False

    logger.info("Executing plan")
    robot.execute(
        group_name=GROUP_NAME,
        robot_trajectory=plan_result.trajectory,
        blocking=True,
    )
    logger.info("Execution finished")
    return True


def build_home_state(robot):
    """Create the legacy M0609 home joint state."""
    home_state = RobotState(robot.get_robot_model())
    home_state.joint_positions = {
        "joint_1": math.radians(0.0),
        "joint_2": math.radians(0.0),
        "joint_3": math.radians(90.0),
        "joint_4": math.radians(0.0),
        "joint_5": math.radians(90.0),
        "joint_6": math.radians(0.0),
    }
    home_state.update()
    return home_state


def build_pose_goal():
    """Create the legacy pose goal used by the original example."""
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = BASE_FRAME
    pose_goal.pose.position.x = 0.5
    pose_goal.pose.position.y = 0.0
    pose_goal.pose.position.z = 0.5
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 1.0
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.0
    return pose_goal


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("azas_motion.mp_basic_legacy")

    try:
        robot = MoveItPy(node_name="moveit_py")
        arm = robot.get_planning_component(GROUP_NAME)
        logger.warning(
            "Running legacy mp_basic example. This node can execute robot motion."
        )

        arm.set_start_state_to_current_state()
        arm.set_goal_state(robot_state=build_home_state(robot))
        if not plan_and_execute(robot, arm, logger):
            return

        plan_and_execute(robot, arm, logger, pose_goal=build_pose_goal())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
