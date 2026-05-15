#!/usr/bin/env python3
import math

import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy



GROUP_NAME = "manipulator"   # SRDF에 정의된 planning group 이름
BASE_FRAME = "base_link"     # 로봇 베이스 프레임
EE_LINK = "link_6"           # 엔드이펙터 링크 이름 (SRDF/URDF 기준)

# HOME_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
# HOME_JOINTS_RAD = [math.radians(d) for d in HOME_JOINTS_DEG]

# 이걸로 교체
HOME_JOINTS_RAD = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(0.0),
}

# ====== 안전 작업 영역 정의 (base_link 기준) ======
SAFE_X_MIN = 0.0      # x는 0 이상
SAFE_Y_MIN = -0.3     # y 하한
SAFE_Y_MAX = 0.3      # y 상한
SAFE_Z_MIN = 0.27    # z는 이 값보다 낮아지면 안 됨
# ==================================================


def clamp_to_safe_workspace(x: float, y: float, z: float, logger):
    """안전 작업 영역으로 (x, y, z) 클램핑"""
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
    """
    공식 문서 스타일 helper: 계획 후 곧바로 실행

    - pose_goal이 주어지면:
      · 안전 영역 클램핑
      · start_state = current
      · pose 기반 goal 설정 (EE_LINK)
    - 그 다음 plan_parameters 유무에 따라 plan() 호출 후 execute
    """
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
        plan_result = planning_component.plan(
            parameters=plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    if not plan_result:
        logger.error("Planning failed")
        return False

    logger.info("Executing plan")
    robot_trajectory = plan_result.trajectory
    robot.execute(
        group_name=GROUP_NAME,
        robot_trajectory=robot_trajectory,
        blocking=True,
    )
    logger.info("Execution finished")
    return True

def main():
    rclpy.init()
    logger = get_logger("m0609.moveit_py.basic")

    robot = MoveItPy(node_name="moveit_py")
    import time
    time.sleep(2.0)

    arm = robot.get_planning_component(GROUP_NAME)
    logger.info("MoveItPy instance created")

    # ── 1) Home position으로 이동 ──────────────────────────────
    # RobotState 방식 대신 joint value map으로 goal 설정
    arm.set_start_state_to_current_state()

    robot_model = robot.get_robot_model()
    home_state = RobotState(robot_model)

    # set_joint_group_positions 대신 이걸로
    home_state.joint_positions = {
        "joint_1": math.radians(0.0),
        "joint_2": math.radians(0.0),
        "joint_3": math.radians(90.0),
        "joint_4": math.radians(0.0),
        "joint_5": math.radians(90.0),
        "joint_6": math.radians(0.0),
    }
    home_state.update()

    arm.set_goal_state(robot_state=home_state)
    plan_and_execute(robot, arm, logger)

    # ── 2) Pose goal로 이동 ────────────────────────────────────
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = BASE_FRAME
    pose_goal.pose.position.x    = 0.5
    pose_goal.pose.position.y    = 0.0
    pose_goal.pose.position.z    = 0.5
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 1.0
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.0

    plan_and_execute(robot, arm, logger, pose_goal=pose_goal)

    rclpy.shutdown()