#!/usr/bin/env python3
"""Course-style MoveItPy dispenser press cycle using measured Azas press joints.

Sequence goal:
- cup to dispenser front pose
- gripper open log
- lift while open
- gripper close log
- move to measured dispenser press contact joints (from calibration.yaml)
- repeat press by lifting from measured contact pose and returning down
- return to cup grasp pose

No fake /joint_states and no display-path publisher are used. RViz observes the
Doosan controller-backed joint states, same path as real execution.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters

INVALID_PRESS_CONTACT_STATUSES = {
    "invalid",
    "invalid_reteach_required",
    "needs_reteach",
    "reteach_required",
    "확인 필요",
}
from rclpy.logging import get_logger

GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK = "link_6"
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CALIBRATION = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"


@dataclass(frozen=True)
class Config:
    dispenser_id: str
    press_count: int
    waypoint_hold_sec: float
    planning_group: str
    base_frame: str
    ee_link: str
    calibration_path: Path
    cup_lift_m: float
    press_up_m: float
    cup_pre_grasp_backoff_m: float
    cup_release_retract_m: float
    cup_place_z: float | None
    planning_time_sec: float
    moveit_execution_settle_sec: float
    press_only: bool


@dataclass(frozen=True)
class OutletCalibration:
    outlet_xyz_m: list[float]
    outlet_quat_xyzw: list[float]
    press_xyz_m: list[float]
    press_quat_xyzw: list[float]
    press_contact_joints_deg: list[float]


def _env(name: str, default: str) -> str:
    import os

    return os.environ.get(name, default)


def _env_float(name: str, default: float) -> float:
    return float(_env(name, str(default)))


def _env_int(name: str, default: int) -> int:
    return int(_env(name, str(default)))


def _env_bool(name: str, default: bool = False) -> bool:
    raw = _env(name, "1" if default else "0").strip().lower()
    return raw in {"1", "true", "yes", "on"}


def _env_optional_float(name: str) -> float | None:
    raw = _env(name, "").strip()
    if not raw:
        return None
    return float(raw)


def read_config() -> Config:
    return Config(
        dispenser_id=_env("DISPENSER_ID", "1"),
        press_count=max(_env_int("PRESS_COUNT", 2), 1),
        waypoint_hold_sec=max(_env_float("WAYPOINT_HOLD_SEC", 1.0), 0.0),
        planning_group=_env("PLANNING_GROUP", GROUP_NAME),
        base_frame=_env("BASE_FRAME", BASE_FRAME),
        ee_link=_env("EE_LINK", EE_LINK),
        calibration_path=Path(_env("CALIBRATION_PATH", str(DEFAULT_CALIBRATION))),
        cup_lift_m=_env_float("CUP_LIFT_M", 0.08),
        press_up_m=_env_float("PRESS_UP_M", 0.05),
        cup_pre_grasp_backoff_m=max(_env_float("CUP_PRE_GRASP_BACKOFF_M", 0.08), 0.0),
        cup_release_retract_m=max(_env_float("CUP_RELEASE_RETRACT_M", 0.05), 0.0),
        cup_place_z=_env_optional_float("CUP_PLACE_Z"),
        planning_time_sec=_env_float("PLANNING_TIME_SEC", 5.0),
        moveit_execution_settle_sec=max(_env_float("MOVEIT_EXECUTION_SETTLE_SEC", 5.0), 0.0),
        press_only=_env_bool("PRESS_ONLY", False),
    )


def _numeric_list(value, label: str, length: int) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{label} must be a list of {length} numbers")
    return [float(v) for v in value]


def load_outlet(cfg: Config) -> OutletCalibration:
    data = yaml.safe_load(cfg.calibration_path.read_text(encoding="utf-8")) or {}
    outlets = data.get("dispenser_outlets") or {}
    block = outlets.get(str(cfg.dispenser_id))
    if not isinstance(block, dict):
        raise ValueError(f"dispenser_outlets.{cfg.dispenser_id} missing in {cfg.calibration_path}")
    status = str(block.get("press_contact_status", "")).strip()
    if status.lower() in INVALID_PRESS_CONTACT_STATUSES:
        raise ValueError(
            f"dispenser_outlets.{cfg.dispenser_id}.press_contact_joints_deg is marked "
            f"{status!r}; refusing dispenser press cycle until PRESS{cfg.dispenser_id}_CONTACT is re-taught"
        )
    return OutletCalibration(
        outlet_xyz_m=_numeric_list(block.get("outlet_pose_xyz_m"), f"outlet {cfg.dispenser_id} outlet_pose_xyz_m", 3),
        outlet_quat_xyzw=_numeric_list(block.get("outlet_pose_quaternion_xyzw"), f"outlet {cfg.dispenser_id} outlet_pose_quaternion_xyzw", 4),
        press_xyz_m=_numeric_list(block.get("press_pose_xyz_m"), f"outlet {cfg.dispenser_id} press_pose_xyz_m", 3),
        press_quat_xyzw=_numeric_list(block.get("press_pose_quaternion_xyzw"), f"outlet {cfg.dispenser_id} press_pose_quaternion_xyzw", 4),
        press_contact_joints_deg=_numeric_list(block.get("press_contact_joints_deg"), f"outlet {cfg.dispenser_id} press_contact_joints_deg", 6),
    )


def quat_xyzw(values: list[float]) -> Quaternion:
    q = Quaternion()
    q.x, q.y, q.z, q.w = [float(v) for v in values]
    return q


def pose_goal(cfg: Config, x: float, y: float, z: float, quat_xyzw_values: list[float]) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = cfg.base_frame
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation = quat_xyzw(quat_xyzw_values)
    return pose


def pose_stamped_from_pose(cfg: Config, pose_value: Pose) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = cfg.base_frame
    pose.pose.position.x = float(pose_value.position.x)
    pose.pose.position.y = float(pose_value.position.y)
    pose.pose.position.z = float(pose_value.position.z)
    pose.pose.orientation.x = float(pose_value.orientation.x)
    pose.pose.orientation.y = float(pose_value.orientation.y)
    pose.pose.orientation.z = float(pose_value.orientation.z)
    pose.pose.orientation.w = float(pose_value.orientation.w)
    return pose


def clone_pose_with_z(pose_value: Pose, z: float) -> Pose:
    pose = Pose()
    pose.position.x = float(pose_value.position.x)
    pose.position.y = float(pose_value.position.y)
    pose.position.z = float(z)
    pose.orientation.x = float(pose_value.orientation.x)
    pose.orientation.y = float(pose_value.orientation.y)
    pose.orientation.z = float(pose_value.orientation.z)
    pose.orientation.w = float(pose_value.orientation.w)
    return pose


def plan_and_execute_pose_stamped(robot, arm, params, cfg: Config, label: str, goal: PoseStamped, logger) -> None:
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=goal, pose_link=cfg.ee_link)
    logger.info(
        f"Goal {label}: pose xyz=({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f}, {goal.pose.position.z:.3f}) "
        f"quat=[{goal.pose.orientation.x:.6f}, {goal.pose.orientation.y:.6f}, {goal.pose.orientation.z:.6f}, {goal.pose.orientation.w:.6f}]"
    )
    logger.info(f"Planning trajectory: {label}")
    result = arm.plan(parameters=params)
    if not result:
        raise RuntimeError(f"planning failed at {label}")
    logger.info(f"Executing plan: {label}")
    ok = robot.execute(group_name=cfg.planning_group, robot_trajectory=result.trajectory, blocking=True)
    if ok is False:
        raise RuntimeError(f"MoveIt execution failed at {label}")
    logger.info(f"Execution finished: {label}")
    time.sleep(cfg.waypoint_hold_sec)


def plan_and_execute_pose(robot, arm, params, cfg: Config, label: str, xyz_m: list[float], quat: list[float], logger) -> None:
    goal = pose_goal(cfg, xyz_m[0], xyz_m[1], xyz_m[2], quat)
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=goal, pose_link=cfg.ee_link)
    logger.info(f"Goal {label}: pose xyz=({xyz_m[0]:.3f}, {xyz_m[1]:.3f}, {xyz_m[2]:.3f}) quat={quat}")
    logger.info(f"Planning trajectory: {label}")
    result = arm.plan(parameters=params)
    if not result:
        raise RuntimeError(f"planning failed at {label}")
    logger.info(f"Executing plan: {label}")
    ok = robot.execute(group_name=cfg.planning_group, robot_trajectory=result.trajectory, blocking=True)
    if ok is False:
        raise RuntimeError(f"MoveIt execution failed at {label}")
    logger.info(f"Execution finished: {label}")
    time.sleep(cfg.waypoint_hold_sec)


def joint_state(model, cfg: Config, joints_deg: list[float], label: str, logger) -> RobotState:
    target = {name: math.radians(float(deg)) for name, deg in zip(JOINT_NAMES, joints_deg)}
    state = RobotState(model)
    # This assignment style is used elsewhere in this repo and is more stable
    # on this Humble MoveItPy build than set_joint_group_positions().
    state.joint_positions = target
    state.update()
    logger.info(f"Goal {label}: measured joints deg={joints_deg}")
    return state


def fk_pose_from_joints(model, joints_deg: list[float], ee_link: str, logger) -> Pose:
    state = RobotState(model)
    state.joint_positions = {name: math.radians(float(deg)) for name, deg in zip(JOINT_NAMES, joints_deg)}
    state.update()
    pose = state.get_pose(ee_link)
    logger.info(
        f"FK from measured press joints: {ee_link} xyz=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}) "
        f"quat=[{pose.orientation.x:.6f}, {pose.orientation.y:.6f}, {pose.orientation.z:.6f}, {pose.orientation.w:.6f}]"
    )
    return pose


def plan_and_execute_joints(robot, arm, model, params, cfg: Config, label: str, joints_deg: list[float], logger) -> None:
    state = joint_state(model, cfg, joints_deg, label, logger)
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=state)
    logger.info(f"Planning trajectory: {label}")
    result = arm.plan(parameters=params)
    if not result:
        raise RuntimeError(f"planning failed at {label}")
    logger.info(f"Executing plan: {label}")
    ok = robot.execute(group_name=cfg.planning_group, robot_trajectory=result.trajectory, blocking=True)
    if ok is False:
        raise RuntimeError(f"MoveIt execution failed at {label}")
    logger.info(f"Execution finished: {label}")
    time.sleep(cfg.waypoint_hold_sec)


def gripper_event(logger, state: str, detail: str) -> None:
    logger.info(f"GRIPPER_{state}: {detail}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    logger = get_logger("dispenser_press_cycle_moveit")
    try:
        cfg = read_config()
        outlet = load_outlet(cfg)
        logger.info(
            f"Ready: measured-joint dispenser press cycle. dispenser={cfg.dispenser_id} "
            f"press_count={cfg.press_count} press_only={cfg.press_only} "
            f"press_contact_joints_deg={outlet.press_contact_joints_deg}"
        )
        robot = MoveItPy(node_name="dispenser_press_cycle_moveit_py")
        arm = robot.get_planning_component(cfg.planning_group)
        model = robot.get_robot_model()
        logger.info("MoveItPy instance created")
        if cfg.moveit_execution_settle_sec > 0.0:
            logger.info(
                f"Waiting {cfg.moveit_execution_settle_sec:.1f}s for MoveIt trajectory execution action clients to connect"
            )
            time.sleep(cfg.moveit_execution_settle_sec)
        plan_params = PlanRequestParameters(robot)
        plan_params.planning_pipeline = "ompl"
        plan_params.planner_id = "RRTConnectkConfigDefault"
        plan_params.max_velocity_scaling_factor = 0.12
        plan_params.max_acceleration_scaling_factor = 0.08
        plan_params.planning_time = cfg.planning_time_sec
        logger.info(f"Planner params: pipeline=ompl planner=RRTConnectkConfigDefault planning_time={cfg.planning_time_sec:.1f}s")
        ptp_params = PlanRequestParameters(robot)
        ptp_params.planning_pipeline = "pilz_industrial_motion_planner"
        ptp_params.planner_id = "PTP"
        ptp_params.max_velocity_scaling_factor = 0.10
        ptp_params.max_acceleration_scaling_factor = 0.08
        ptp_params.planning_time = cfg.planning_time_sec
        logger.info("Joint approach params: pipeline=pilz_industrial_motion_planner planner=PTP measured press joints")
        lin_params = PlanRequestParameters(robot)
        lin_params.planning_pipeline = "pilz_industrial_motion_planner"
        lin_params.planner_id = "LIN"
        lin_params.max_velocity_scaling_factor = 0.04
        lin_params.max_acceleration_scaling_factor = 0.04
        lin_params.planning_time = cfg.planning_time_sec
        logger.info("Press params: pipeline=pilz_industrial_motion_planner planner=LIN z-only Cartesian stroke")

        # PRESS_ONLY는 RViz/프레스 검증용이다. 컵 배치/복귀 IK 경로를 모두 빼고,
        # 사용자가 실측한 프레스 조인트 자세와 그 FK 기준 Z-only 펌프만 보여준다.
        # 이 모드는 컵 좌표나 outlet IK가 섞여서 "프레스 움직임" 판단을 흐리는 것을 막는다.
        if cfg.press_only:
            logger.info(
                "PRESS_ONLY_MODE: skipping cup placement, gripper-open release, and cup return. "
                "Executing measured press joint pose followed by Z-only pump strokes."
            )
            gripper_event(logger, "CLOSE", "PRESS_ONLY: 프레스 검증을 위해 빈 그리퍼를 닫은 상태로 가정")
            plan_and_execute_joints(
                robot,
                arm,
                model,
                ptp_params,
                cfg,
                f"press_only_move_to_measured_press_contact_joints_{cfg.dispenser_id}",
                outlet.press_contact_joints_deg,
                logger,
            )
            press_contact_pose = fk_pose_from_joints(model, outlet.press_contact_joints_deg, cfg.ee_link, logger)
            press_ready_pose = clone_pose_with_z(
                press_contact_pose,
                press_contact_pose.position.z + max(cfg.press_up_m, 0.0),
            )
            logger.info(
                "PRESS_ONLY_Z_ONLY: repeating LIN strokes with fixed "
                f"x={press_contact_pose.position.x:.3f}, y={press_contact_pose.position.y:.3f}, "
                f"contact_z={press_contact_pose.position.z:.3f}, ready_z={press_ready_pose.position.z:.3f}"
            )
            plan_and_execute_pose_stamped(
                robot,
                arm,
                lin_params,
                cfg,
                "press_only_linear_lift_from_contact_to_press_ready_z_only",
                pose_stamped_from_pose(cfg, press_ready_pose),
                logger,
            )
            for index in range(1, cfg.press_count + 1):
                plan_and_execute_pose_stamped(
                    robot,
                    arm,
                    lin_params,
                    cfg,
                    f"press_only_press_{index}_down_z_only",
                    pose_stamped_from_pose(cfg, press_contact_pose),
                    logger,
                )
                plan_and_execute_pose_stamped(
                    robot,
                    arm,
                    lin_params,
                    cfg,
                    f"press_only_press_{index}_up_z_only",
                    pose_stamped_from_pose(cfg, press_ready_pose),
                    logger,
                )
            logger.info("DONE: measured dispenser press-only cycle completed by MoveItPy robot.execute().")
            return

        # 1. 컵을 디스펜서 앞에 갖다 놓기: measured outlet pose.
        cup_place = list(outlet.outlet_xyz_m)
        if cfg.cup_place_z is not None:
            cup_place[2] = cfg.cup_place_z
        plan_and_execute_pose(robot, arm, plan_params, cfg, "cup_to_measured_dispenser_front", cup_place, outlet.outlet_quat_xyzw, logger)
        gripper_event(logger, "OPEN", "컵을 디스펜서 앞에 놓기 위해 그리퍼 펴기")

        # 2. 수출구/nozzle 회피: 컵을 놓은 자리에서 바로 Z 상승하지 않는다.
        #    먼저 base_link X 방향으로 뒤로 빠진 뒤, 그 뒤쪽 위치에서만 Z를 올린다.
        cup_lift = [
            cup_place[0] - cfg.cup_release_retract_m,
            cup_place[1],
            cup_place[2] + max(cfg.cup_lift_m, 0.0),
        ]
        logger.info(
            "CUP_RELEASE_NOZZLE_AVOIDANCE: after opening gripper, move to a behind-and-up pre-lift pose "
            f"retract_x={cfg.cup_release_retract_m:.3f}m lift_z={cfg.cup_lift_m:.3f}m "
            f"from=({cup_place[0]:.3f}, {cup_place[1]:.3f}, {cup_place[2]:.3f}) "
            f"to=({cup_lift[0]:.3f}, {cup_lift[1]:.3f}, {cup_lift[2]:.3f})"
        )
        plan_and_execute_pose(robot, arm, plan_params, cfg, "move_to_behind_up_pre_lift_nozzle_avoid", cup_lift, outlet.outlet_quat_xyzw, logger)

        # 3. 프레스 자세를 위해 그리퍼 오므리기 + 측정한 접촉 조인트 자세로 이동.
        gripper_event(logger, "CLOSE", "프레스 자세 준비를 위해 그리퍼 오므리기")
        plan_and_execute_joints(
            robot,
            arm,
            model,
            ptp_params,
            cfg,
            f"move_to_measured_press_contact_joints_{cfg.dispenser_id}",
            outlet.press_contact_joints_deg,
            logger,
        )

        # 4. 프레스는 measured contact joints의 FK pose를 기준으로 한다.
        #    여기서부터는 다른 IK 해석으로 팔을 흔들지 말고, 같은 X/Y/orientation에서 Z만 LIN 이동한다.
        press_contact_pose = fk_pose_from_joints(model, outlet.press_contact_joints_deg, cfg.ee_link, logger)
        press_ready_pose = clone_pose_with_z(
            press_contact_pose,
            press_contact_pose.position.z + max(cfg.press_up_m, 0.0),
        )
        logger.info(
            "PRESS_Z_ONLY: after measured press joint, repeating LIN strokes with fixed "
            f"x={press_contact_pose.position.x:.3f}, y={press_contact_pose.position.y:.3f}, "
            f"contact_z={press_contact_pose.position.z:.3f}, ready_z={press_ready_pose.position.z:.3f}"
        )
        plan_and_execute_pose_stamped(
            robot,
            arm,
            lin_params,
            cfg,
            "linear_lift_from_contact_to_press_ready_z_only",
            pose_stamped_from_pose(cfg, press_ready_pose),
            logger,
        )
        for index in range(1, cfg.press_count + 1):
            plan_and_execute_pose_stamped(
                robot,
                arm,
                lin_params,
                cfg,
                f"press_{index}_down_z_only",
                pose_stamped_from_pose(cfg, press_contact_pose),
                logger,
            )
            plan_and_execute_pose_stamped(
                robot,
                arm,
                lin_params,
                cfg,
                f"press_{index}_up_z_only",
                pose_stamped_from_pose(cfg, press_ready_pose),
                logger,
            )

        # 5. 다시 컵 잡기: 목표점으로 바로 꽂지 않는다.
        #    컵 grasp pose보다 base_link X 방향으로 뒤(backoff)인 pre-grasp에 먼저 가고,
        #    같은 orientation으로 직선 접근 후 그리퍼를 닫는다.
        cup_pre_grasp = [
            cup_place[0] - cfg.cup_pre_grasp_backoff_m,
            cup_place[1],
            cup_place[2],
        ]
        logger.info(
            "CUP_PRE_GRASP: move behind cup before final grasp "
            f"backoff_x={cfg.cup_pre_grasp_backoff_m:.3f}m pre=({cup_pre_grasp[0]:.3f}, {cup_pre_grasp[1]:.3f}, {cup_pre_grasp[2]:.3f}) "
            f"grasp=({cup_place[0]:.3f}, {cup_place[1]:.3f}, {cup_place[2]:.3f})"
        )
        plan_and_execute_pose(robot, arm, plan_params, cfg, "return_to_cup_pre_grasp_backoff", cup_pre_grasp, outlet.outlet_quat_xyzw, logger)
        plan_and_execute_pose(robot, arm, lin_params, cfg, "linear_approach_to_cup_grasp", cup_place, outlet.outlet_quat_xyzw, logger)
        gripper_event(logger, "CLOSE", "pre-grasp에서 직선 접근 후 다시 컵 잡기")
        logger.info("DONE: measured dispenser press cycle completed by MoveItPy robot.execute().")
    except Exception as exc:
        logger.error(f"FAILED: measured dispenser press cycle failed: {exc}")
        raise
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
