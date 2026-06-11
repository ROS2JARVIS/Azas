"""MoveIt 모션 유틸 (순수 함수)."""

import numpy as np
import rclpy
import time
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation as R
from . import _config as cfg



def clamp_to_safe_workspace(x, y, z, logger):
    """safety.yaml 범위로 클램핑하고 경고 로그 (X, Y, Z 상/하한 모두 적용)."""
    
    # 안전 설정 파일이 제대로 로드되지 않았을 경우를 대비한 방어 코드
    if not cfg.SAFETY_CFG or 'motion' not in cfg.SAFETY_CFG:
        logger.error("SAFETY_CFG가 로드되지 않아 클램핑을 건너뜁니다.")
        return x, y, z

    # YAML 데이터에서 작업 영역 경계선 가져오기
    bounds = cfg.SAFETY_CFG['motion']['workspace_bounds_m']
    
    safe_x_min, safe_x_max = bounds['x_min'], bounds['x_max']
    safe_y_min, safe_y_max = bounds['y_min'], bounds['y_max']
    safe_z_min, safe_z_max = bounds['z_min'], bounds['z_max']

    # X축 클램핑
    if x < safe_x_min:
        logger.warning(f"x={x:.3f} -> {safe_x_min} (X 최소 한계 도달)")
        x = safe_x_min
    elif x > safe_x_max:
        logger.warning(f"x={x:.3f} -> {safe_x_max} (X 최대 한계 도달)")
        x = safe_x_max

    # Y축 클램핑
    if y < safe_y_min:
        logger.warning(f"y={y:.3f} -> {safe_y_min} (Y 최소 한계 도달)")
        y = safe_y_min
    elif y > safe_y_max:
        logger.warning(f"y={y:.3f} -> {safe_y_max} (Y 최대 한계 도달)")
        y = safe_y_max

    # Z축 클램핑
    if z < safe_z_min:
        logger.warning(f"z={z:.3f} -> {safe_z_min} (Z 최소 한계 도달)")
        z = safe_z_min
    elif z > safe_z_max:
        logger.warning(f"z={z:.3f} -> {safe_z_max} (Z 최대 한계 도달)")
        z = safe_z_max

    return x, y, z

def make_pose(x, y, z, ori) -> PoseStamped:
    """(x, y, z) + orientation dict → PoseStamped(base_link)."""
    p = PoseStamped()
    p.header.frame_id = cfg.BASE_FRAME
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = float(z)
    p.pose.orientation.x = ori["x"]
    p.pose.orientation.y = ori["y"]
    p.pose.orientation.z = ori["z"]
    p.pose.orientation.w = ori["w"]
    return p


def get_ee_matrix(moveit_robot) -> np.ndarray:
    """현재 base_link → EE_LINK 4x4 변환행렬."""
    psm = moveit_robot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        T = scene.current_state.get_global_link_transform(cfg.EE_LINK)
    return np.asarray(T, dtype=float)


def _wait_future(future, label: str, timeout_sec: float, logger) -> bool:
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not future.done():
        if time.monotonic() >= deadline:
            logger.error(f"{label} timeout after {timeout_sec:.1f}s")
            return False
        time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
    return future.done()


def _execute_with_controller_action(node, trajectory, logger, *,
                                    action_name: str,
                                    wait_sec: float) -> bool:
    trajectory_msg = trajectory
    if hasattr(trajectory_msg, "get_robot_trajectory_msg"):
        trajectory_msg = trajectory_msg.get_robot_trajectory_msg()
    joint_trajectory = trajectory_msg.joint_trajectory
    if not joint_trajectory.points:
        logger.error("Controller action execution rejected empty trajectory")
        return False

    client = ActionClient(node, FollowJointTrajectory, action_name)
    if not client.wait_for_server(timeout_sec=wait_sec):
        logger.error(f"Controller action server not ready after {wait_sec:.1f}s: {action_name}")
        client.destroy()
        return False

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = joint_trajectory
    goal_future = client.send_goal_async(goal)
    if not _wait_future(goal_future, "send controller goal", wait_sec, logger):
        client.destroy()
        return False
    goal_handle = goal_future.result()
    if not goal_handle.accepted:
        logger.error("Controller rejected trajectory")
        client.destroy()
        return False

    result_future = goal_handle.get_result_async()
    if not _wait_future(result_future, "execute controller trajectory", wait_sec, logger):
        client.destroy()
        return False
    result = result_future.result().result
    client.destroy()
    if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
        logger.error(
            f"Controller execution failed: code={result.error_code} {result.error_string}"
        )
        return False
    logger.info(f"Controller trajectory reached via {action_name}")
    return True


def plan_and_execute(robot, arm, logger,
                     pose_goal=None, state_goal=None, params=None,
                     node=None, controller_action_name=None,
                     controller_action_wait_sec: float = 30.0) -> bool:
    """Pose 또는 RobotState 목표로 plan + execute. 실패 시 False."""
    arm.set_start_state_to_current_state()

    if pose_goal is not None:
        x = pose_goal.pose.position.x
        y = pose_goal.pose.position.y
        z = pose_goal.pose.position.z
        sx, sy, sz = clamp_to_safe_workspace(x, y, z, logger)
        pose_goal.pose.position.x = sx
        pose_goal.pose.position.y = sy
        pose_goal.pose.position.z = sz
        arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=cfg.EE_LINK)
    elif state_goal is not None:
        arm.set_goal_state(robot_state=state_goal)
    else:
        logger.error("plan_and_execute: pose/state 없음")
        return False

    plan_result = arm.plan(parameters=params) if params is not None else arm.plan()
    if not plan_result:
        logger.error("Planning 실패")
        return False

    if node is not None and controller_action_name:
        return _execute_with_controller_action(
            node,
            plan_result.trajectory,
            logger,
            action_name=str(controller_action_name),
            wait_sec=float(controller_action_wait_sec),
        )

    result = robot.execute(group_name=cfg.GROUP_NAME,
                           robot_trajectory=plan_result.trajectory,
                           blocking=True)
    return bool(result)


def get_gripper_pose_by_cup(cup_theta):
    """
    컵의 주축 각도(theta)를 받아 그리퍼가 옆면(허리)을 수직 진입하여 
    파지할 수 있도록 쿼터니언 반환
    """
    yaw = -cup_theta

    # 오일러 각을 쿼터니언으로 변환
    quat = R.from_euler('xyz', [180, 0, np.degrees(yaw)], degrees=True).as_quat()
    
    ori_dict = {
        "x": float(quat[0]), 
        "y": float(quat[1]), 
        "z": float(quat[2]), 
        "w": float(quat[3])
    }
    return ori_dict
