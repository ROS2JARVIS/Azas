#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters

# ====== OnRobot RG2 설정 ======
from .onrobot import RG  # 같은 패키지 내부의 onrobot.py 사용

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = 502

# 그리퍼 폭 (raw 단위: 1/10 mm)
GRIPPER_OPEN_WIDTH = 500   # 50.0 mm
GRIPPER_CLOSE_WIDTH = 150  # 20.0 mm
GRIPPER_FORCE = 300        # 약 20 N

# ================================
# 기본 설정
# ================================
GROUP_NAME = "manipulator"   # SRDF에 정의된 planning group 이름
BASE_FRAME = "base_link"     # 로봇 베이스 프레임
EE_LINK = "link_6"           # 엔드이펙터 링크 이름 (SRDF/URDF 기준)

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

# ====== 기어 픽업/조립 포즈 (base_link 기준) ======
GEAR_TASKS = [
    {   # Gear 1
        "pick": {
            "pos": {"x": 0.393, "y":  0.094, "z": 0.280},
            "ori": {"x": 0.000, "y": 1.000, "z": 0.000, "w": 0.000},
        },
        "place": {
            "pos": {"x": 0.393, "y": -0.206, "z": 0.280},
            "ori": {"x": 0.000, "y": 1.000, "z": 0.000, "w": 0.000},
        },
    },
    {   # Gear 2
        "pick": {
            "pos": {"x": 0.392, "y":  0.200, "z": 0.280},
            "ori": {"x": 0.000, "y": 1.000, "z": 0.000, "w": 0.000},
        },
        "place": {
            "pos": {"x": 0.392, "y": -0.101, "z": 0.280},
            "ori": {"x": 0.000, "y": 1.000, "z": 0.000, "w": 0.000},
        },
    },
    {   # Gear 3
        "pick": {
            "pos": {"x": 0.486, "y":  0.153, "z": 0.280},
            "ori": {"x": 0.000, "y": 1.000, "z": 0.000, "w": 0.000},
        },
        "place": {
            "pos": {"x": 0.486, "y": -0.149, "z": 0.280},
            "ori": {"x": 0.000, "y": 1.000, "z": 0.000, "w": 0.000},
        },
    },
    {   # Gear 4
        "pick": {
            "pos": {"x": 0.427, "y":  0.148, "z": 0.280},
            "ori": {"x": 0.000, "y": 1.000, "z": 0.000, "w": 0.000},
        },
        "place": {
            "pos": {"x": 0.426, "y": -0.153, "z": 0.280},
            "ori": {"x": 0.000, "y": 1.000, "z": 0.000, "w": 0.000},
        },
    },
]

APPROACH_OFFSET = 0.05  # 위에서 접근/후퇴할 거리 [m]

# ----- 마지막 기어용 wiggle 파라미터 -----
WIGGLE_Z = 0.295      # z축 회전할 높이 (place_z보다 약간 위)
WIGGLE_YAW_DEG = 5.0   # 좌우로 회전 각도 (deg)
WIGGLE_COUNT = 3       # 좌우 반복 횟수


def quat_mul(q1, q2):
    """쿼터니언 곱: q = q1 * q2"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return x, y, z, w


def make_yaw_quat(yaw_rad):
    """z축(yaw) 회전에 해당하는 쿼터니언 생성"""
    half = yaw_rad / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


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


def main(args=None):
    rclpy.init(args=args)

    logger = get_logger("m0609_gear_assembly")
    logger.info("=== M0609 Gear Assembly 시작 ===")

    # ---- Gripper ----
    gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
    time.sleep(0.5)
    gripper.move_gripper(GRIPPER_OPEN_WIDTH, GRIPPER_FORCE)

    # ---- MoveIt ----
    robot = MoveItPy(node_name="moveit_py")
    arm = robot.get_planning_component(GROUP_NAME)
    robot_model = robot.get_robot_model()

    # ---- PlanRequestParameters (HOME / Pilz) ----
    home_params = PlanRequestParameters(robot)
    home_params.planning_pipeline = "ompl"
    home_params.planner_id = "RRTConnect"
    home_params.max_velocity_scaling_factor = 0.2
    home_params.max_acceleration_scaling_factor = 0.1
    home_params.planning_time = 2.0

    pilz_params = PlanRequestParameters(robot)
    pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
    pilz_params.planner_id = "PTP"
    pilz_params.max_velocity_scaling_factor = 0.15
    pilz_params.max_acceleration_scaling_factor = 0.1
    pilz_params.planning_time = 2.0

    # ---- HOME 자세로 이동 (joint goal) ----
    logger.info("=== HOME 자세로 이동 ===")
    home_state = RobotState(robot_model)
    home_state.joint_positions = {
        "joint_1": math.radians(0.0),
        "joint_2": math.radians(0.0),
        "joint_3": math.radians(90.0),
        "joint_4": math.radians(0.0),
        "joint_5": math.radians(90.0),
        "joint_6": math.radians(0.0),
    }
    home_state.update()

    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=home_state)
    plan_and_execute(robot, arm, logger, plan_parameters=home_params)

    # ---- PoseStamped 공용 객체 준비 ----
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = BASE_FRAME

    logger.info("=== Gear Pick & Place 시작 ===")

    total_gears = len(GEAR_TASKS)

    # -------------------------------
    #   각 기어에 대해 Pick & Place
    # -------------------------------
    for gear_idx, task in enumerate(GEAR_TASKS, start=1):
        pick = task["pick"]
        place = task["place"]

        logger.info(f"--- Gear {gear_idx} 작업 시작 ---")

        # 1) Pick 위에서 접근 (z + offset)
        pose_goal.pose.position.x = pick["pos"]["x"]
        pose_goal.pose.position.y = pick["pos"]["y"]
        pose_goal.pose.position.z = pick["pos"]["z"] + APPROACH_OFFSET

        pose_goal.pose.orientation.x = pick["ori"]["x"]
        pose_goal.pose.orientation.y = pick["ori"]["y"]
        pose_goal.pose.orientation.z = pick["ori"]["z"]
        pose_goal.pose.orientation.w = pick["ori"]["w"]

        plan_and_execute(robot, arm, logger,
                         pose_goal=pose_goal,
                         plan_parameters=pilz_params)

        # 2) Pick 위치로 내려가기
        pose_goal.pose.position.z = pick["pos"]["z"]
        plan_and_execute(robot, arm, logger,
                         pose_goal=pose_goal,
                         plan_parameters=pilz_params)

        # 3) Gripper 닫기 (집기)
        logger.info("Gripper CLOSE (20mm) – 기어 집기")
        gripper.move_gripper(width_val=GRIPPER_CLOSE_WIDTH,
                             force_val=GRIPPER_FORCE)
        time.sleep(1.0)

        # 4) 다시 위로 올라가기 (Pick z + offset)
        pose_goal.pose.position.z = pick["pos"]["z"] + APPROACH_OFFSET
        plan_and_execute(robot, arm, logger,
                         pose_goal=pose_goal,
                         plan_parameters=pilz_params)

        # 5) Place 위에서 접근
        pose_goal.pose.position.x = place["pos"]["x"]
        pose_goal.pose.position.y = place["pos"]["y"]
        pose_goal.pose.position.z = place["pos"]["z"] + APPROACH_OFFSET

        pose_goal.pose.orientation.x = place["ori"]["x"]
        pose_goal.pose.orientation.y = place["ori"]["y"]
        pose_goal.pose.orientation.z = place["ori"]["z"]
        pose_goal.pose.orientation.w = place["ori"]["w"]

        plan_and_execute(robot, arm, logger,
                         pose_goal=pose_goal,
                         plan_parameters=pilz_params)

        # -------------------------------
        #   Place 동작
        #   - 1~3번 기어: 바로 place_z로 하강
        #   - 4번 기어(마지막): z=WIGGLE_Z에서 z축 회전 wiggle 후 place_z로 하강
        # -------------------------------
        if gear_idx < total_gears:
            # 6) Place 위치로 내려가기 (일반)
            pose_goal.pose.position.z = place["pos"]["z"]
            plan_and_execute(robot, arm, logger,
                             pose_goal=pose_goal,
                             plan_parameters=pilz_params)
        else:
            # ---- 마지막 기어: z축 wiggle ----
            logger.info(
                f"마지막 기어: z={WIGGLE_Z:.3f}에서 "
                f"±{WIGGLE_YAW_DEG}deg 좌우 회전 {WIGGLE_COUNT}회씩 수행"
            )

            # 우선 WIGGLE_Z까지 z만 이동
            pose_goal.pose.position.z = WIGGLE_Z
            plan_and_execute(robot, arm, logger,
                             pose_goal=pose_goal,
                             plan_parameters=pilz_params)

            # 기준 쿼터니언 (place 자세)
            q_base = (
                place["ori"]["x"],
                place["ori"]["y"],
                place["ori"]["z"],
                place["ori"]["w"],
            )

            yaw_rad = math.radians(WIGGLE_YAW_DEG)

            # 좌우로 WIGGLE_COUNT 번 반복
            for i in range(1, WIGGLE_COUNT + 1):
                # +yaw
                q_plus = quat_mul(make_yaw_quat(+yaw_rad), q_base)
                pose_goal.pose.orientation.x = q_plus[0]
                pose_goal.pose.orientation.y = q_plus[1]
                pose_goal.pose.orientation.z = q_plus[2]
                pose_goal.pose.orientation.w = q_plus[3]

                logger.info(f"Wiggle {i}/{WIGGLE_COUNT}: +{WIGGLE_YAW_DEG:.1f} deg")
                plan_and_execute(robot, arm, logger,
                                 pose_goal=pose_goal,
                                 plan_parameters=pilz_params)

                # -yaw
                q_minus = quat_mul(make_yaw_quat(-yaw_rad), q_base)
                pose_goal.pose.orientation.x = q_minus[0]
                pose_goal.pose.orientation.y = q_minus[1]
                pose_goal.pose.orientation.z = q_minus[2]
                pose_goal.pose.orientation.w = q_minus[3]

                logger.info(f"Wiggle {i}/{WIGGLE_COUNT}: -{WIGGLE_YAW_DEG:.1f} deg")
                plan_and_execute(robot, arm, logger,
                                 pose_goal=pose_goal,
                                 plan_parameters=pilz_params)

            # 마지막에 기준 자세로 복귀
            pose_goal.pose.orientation.x = q_base[0]
            pose_goal.pose.orientation.y = q_base[1]
            pose_goal.pose.orientation.z = q_base[2]
            pose_goal.pose.orientation.w = q_base[3]

            plan_and_execute(robot, arm, logger,
                             pose_goal=pose_goal,
                             plan_parameters=pilz_params)

            # 그리고 place_z로 직선 하강
            pose_goal.pose.position.z = place["pos"]["z"]
            logger.info(f"마지막 기어: wiggle 후 place_z={place['pos']['z']:.3f}로 하강")
            plan_and_execute(robot, arm, logger,
                             pose_goal=pose_goal,
                             plan_parameters=pilz_params)

        # 7) Gripper 열기 (놓기)
        logger.info("Gripper OPEN (50mm) – 기어 놓기")
        gripper.move_gripper(width_val=GRIPPER_OPEN_WIDTH,
                             force_val=GRIPPER_FORCE)
        time.sleep(1.0)

        # 8) 다시 위로 올라가기 (Place z + offset)
        pose_goal.pose.position.z = place["pos"]["z"] + APPROACH_OFFSET
        plan_and_execute(robot, arm, logger,
                         pose_goal=pose_goal,
                         plan_parameters=pilz_params)

        logger.info(f"--- Gear {gear_idx} 작업 완료 ---")

    logger.info("=== 모든 기어 조립 완료. HOME으로 복귀 ===")

    # 마지막으로 HOME으로 복귀
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=home_state)
    plan_and_execute(robot, arm, logger, plan_parameters=home_params)

    logger.info("=== Gear Assembly 노드 종료 ===")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
