#!/usr/bin/env python3

import math
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from ultralytics import YOLO

from .onrobot import RG


GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK = "link_6"

HOME_JOINTS = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(90.0),
}
HOME_JOINTS_RAD = [
    math.radians(0.0),
    math.radians(0.0),
    math.radians(90.0),
    math.radians(0.0),
    math.radians(90.0),
    math.radians(90.0),
]
ARM_JOINT_ORDER = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.35
SAFE_Y_MAX = 0.35
SAFE_Z_MIN = 0.20

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = 502
GRIPPER_OPEN_WIDTH = 1100
GRIPPER_CLOSE_WIDTH = 120
GRIPPER_FORCE = 250
GRIPPER_OPEN_TIMEOUT_SEC = 5.0
GRIPPER_STATUS_POLL_SEC = 0.15

DOWN_ORI = {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0}


@dataclass
class SideGraspPlan:
    cup_xyz: np.ndarray
    side_vec: np.ndarray
    orientation: dict
    stage_xy: np.ndarray
    pre_xy: np.ndarray
    grasp_xy: np.ndarray
    guarded_grasp_xy: np.ndarray
    grasp_z: float
    pre_z: float
    lift_z: float
    place_z: float
    place_approach_z: float
    side_direction: float
    close_backoff_m: float
    score: float = 0.0


def clamp_to_safe_workspace(x, y, z, logger, z_min=SAFE_Z_MIN, clamp_xy=True):
    if clamp_xy:
        if x < SAFE_X_MIN:
            logger.warning(f"x={x:.3f} -> {SAFE_X_MIN:.3f}")
            x = SAFE_X_MIN
        if y < SAFE_Y_MIN:
            logger.warning(f"y={y:.3f} -> {SAFE_Y_MIN:.3f}")
            y = SAFE_Y_MIN
        elif y > SAFE_Y_MAX:
            logger.warning(f"y={y:.3f} -> {SAFE_Y_MAX:.3f}")
            y = SAFE_Y_MAX
    if z < z_min:
        logger.warning(f"z={z:.3f} -> {z_min:.3f}")
        z = z_min
    return x, y, z


def make_pose(x, y, z, ori=None):
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


def parse_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def parse_axis(value):
    if isinstance(value, bool):
        return "y" if value else "x"

    normalized = str(value).strip().lower()
    if normalized in {"y", "y_axis", "axis_y", "true", "yes", "on"}:
        return "y"
    if normalized in {"x", "x_axis", "axis_x"}:
        return "x"
    return normalized


def parameter_array_or_empty(value):
    if value is None:
        return []
    return list(value)


def parse_workspace_bounds(value):
    if not isinstance(value, dict):
        return None
    required_keys = ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")
    if any(key not in value for key in required_keys):
        return None
    bounds = {key: float(value[key]) for key in required_keys}
    if bounds["x_min"] > bounds["x_max"]:
        raise ValueError("workspace_bounds_m x_min must be <= x_max")
    if bounds["y_min"] > bounds["y_max"]:
        raise ValueError("workspace_bounds_m y_min must be <= y_max")
    if bounds["z_min"] > bounds["z_max"]:
        raise ValueError("workspace_bounds_m z_min must be <= z_max")
    return bounds


def quat_dict_from_matrix(matrix):
    qx, qy, qz, qw = Rotation.from_matrix(matrix).as_quat()
    return {
        "x": float(qx),
        "y": float(qy),
        "z": float(qz),
        "w": float(qw),
    }


def quat_dict_from_euler(roll_deg, pitch_deg, yaw_deg):
    qx, qy, qz, qw = Rotation.from_euler(
        "xyz",
        [roll_deg, pitch_deg, yaw_deg],
        degrees=True,
    ).as_quat()
    return {
        "x": float(qx),
        "y": float(qy),
        "z": float(qz),
        "w": float(qw),
    }


def get_ee_matrix(moveit_robot):
    psm = moveit_robot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        transform = scene.current_state.get_global_link_transform(EE_LINK)
    return np.asarray(transform, dtype=float)


def get_ee_matrix_from_robot_state(robot_state):
    transform = robot_state.get_global_link_transform(EE_LINK)
    return np.asarray(transform, dtype=float)


class YoloCupPickNode(Node):
    def __init__(self):
        super().__init__("yolo_cup_pick_node")

        self.declare_parameter(
            "model_path",
            "/home/ssu/Azas/best.pt",
        )
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("target_class", "cup")
        self.declare_parameter("auto_pick", False)
        self.declare_parameter("auto_pick_interval", 3.0)
        self.declare_parameter("exit_after_pick", False)
        self.declare_parameter("depth_patch_radius", 7)
        self.declare_parameter("min_depth_valid_ratio", 0.03)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 1.20)
        self.declare_parameter("redetect_on_approach", True)
        self.declare_parameter("redetect_settle_sec", 0.5)
        self.declare_parameter("grasp_mode", "side")
        dynamic_param = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("side_grasp_axis", "y_axis", dynamic_param)
        self.declare_parameter("side_grasp_direction", 1.0)
        self.declare_parameter("side_approach_offset", 0.16)
        self.declare_parameter("side_staging_offset", 0.30)
        self.declare_parameter("side_far_stage_enabled", False)
        self.declare_parameter("side_short_stage_backoff_m", 0.06)
        self.declare_parameter("side_stage_y_min", SAFE_Y_MIN)
        self.declare_parameter("side_stage_y_max", SAFE_Y_MAX)
        self.declare_parameter("side_grasp_offset", 0.035)
        self.declare_parameter("side_grasp_z_offset", 0.05)
        self.declare_parameter("side_grasp_stop_backoff_m", 0.04)
        self.declare_parameter("side_close_underreach_m", 0.03)
        self.declare_parameter("side_low_retry_lift_m", 0.03)
        self.declare_parameter("side_low_retry_attempts", 5)
        self.declare_parameter("side_auto_direction_by_cup_y", False)
        self.declare_parameter("side_candidate_plan_check_enabled", True)
        self.declare_parameter("side_linear_approach_enabled", True)
        self.declare_parameter("side_final_slide_enabled", False)
        self.declare_parameter("side_fixed_grasp_z_enabled", False)
        self.declare_parameter("side_fixed_grasp_z", 0.07)
        self.declare_parameter("side_project_bbox_center_to_fixed_z", True)
        self.declare_parameter("table_collision_enabled", True)
        self.declare_parameter("table_collision_id", "side_grip_table")
        self.declare_parameter("table_surface_z", 0.0)
        self.declare_parameter("table_thickness", 0.04)
        self.declare_parameter("table_size_x", 1.20)
        self.declare_parameter("table_size_y", 1.00)
        self.declare_parameter("table_center_x", 0.45)
        self.declare_parameter("table_center_y", 0.0)
        self.declare_parameter("table_publish_repeats", 3)
        self.declare_parameter("table_collision_expand_to_workspace_walls", True)
        self.declare_parameter(
            "safety_config_path",
            "/home/ssu/Azas/src/azas_bringup/config/safety.yaml",
        )
        self.declare_parameter("safety_workspace_enforced", True)
        self.declare_parameter("workspace_boundary_collision_enabled", True)
        self.declare_parameter("workspace_boundary_collision_prefix", "side_grip_workspace")
        self.declare_parameter("workspace_boundary_wall_thickness", 0.04)
        self.declare_parameter("workspace_boundary_wall_clearance", 0.02)
        self.declare_parameter("gripper_open_settle_sec", 1.0)
        self.declare_parameter("pre_pick_joint1_clearance_deg", 12.0)
        self.declare_parameter(
            "center_check_enabled",
            True,
            ParameterDescriptor(description="Move to a high observe pose and re-detect cup center before picking."),
        )
        self.declare_parameter(
            "center_check_settle_sec",
            0.6,
            ParameterDescriptor(description="Seconds to wait for camera frames after moving to center-check pose."),
        )
        self.declare_parameter(
            "center_check_x",
            0.45,
            ParameterDescriptor(description="High observe pose X in base_link."),
        )
        self.declare_parameter(
            "center_check_y",
            0.0,
            ParameterDescriptor(description="High observe pose Y in base_link."),
        )
        self.declare_parameter(
            "center_check_z",
            0.64,
            ParameterDescriptor(description="High observe pose Z in base_link."),
        )
        self.declare_parameter(
            "side_prepose_enabled",
            False,
            ParameterDescriptor(description="Enable rule-based joint-space pre-pose before side grasp."),
        )
        self.declare_parameter(
            "side_prepose_joint_order",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            ParameterDescriptor(description="Joint name order for side_prepose_* joint arrays."),
        )
        self.declare_parameter(
            "side_prepose_split_z",
            0.18,
            ParameterDescriptor(description="Cup base z threshold to select low/high prepose."),
        )
        self.declare_parameter(
            "side_prepose_split_y",
            0.0,
            ParameterDescriptor(description="Cup base y threshold to select left/right prepose."),
        )
        self.declare_parameter(
            "side_prepose_selection_mode",
            "y",
            ParameterDescriptor(description="Prepose selection: 'y' (cup left/right) or 'z' (low/high)."),
        )
        self.declare_parameter(
            "side_prepose_joints_cup_left_rad",
            [],
            ParameterDescriptor(
                dynamic_typing=True,
                description="Joint positions (radians) for cup-left prepose; length must match side_prepose_joint_order."
            ),
        )
        self.declare_parameter(
            "side_prepose_joints_cup_right_rad",
            [],
            ParameterDescriptor(
                dynamic_typing=True,
                description="Joint positions (radians) for cup-right prepose; length must match side_prepose_joint_order."
            ),
        )
        self.declare_parameter(
            "side_move_to_initial_center_before_close",
            False,
            ParameterDescriptor(
                description="Deprecated/no-op: center move before close is blocked to avoid pushing the cup."
            ),
        )
        self.declare_parameter("side_orientation_mode", "approach")
        self.declare_parameter("side_tool_roll_deg", 0.0)
        self.declare_parameter("side_roll_deg", 0.0)
        self.declare_parameter("side_pitch_deg", 90.0)
        self.declare_parameter("side_yaw_deg", 0.0)
        self.declare_parameter("pick_z_offset", 0.20)
        self.declare_parameter("approach_offset", 0.12)
        self.declare_parameter("safe_z", 0.50)
        self.declare_parameter("min_motion_z", 0.07)
        self.declare_parameter("workspace_xy_clamp_enabled", False)
        self.declare_parameter("return_home_after_task", True)
        self.declare_parameter("verify_motion", True)
        self.declare_parameter("motion_verify_tolerance", 0.01)
        self.declare_parameter("joint_goal_tolerance_rad", 0.02)
        self.declare_parameter("min_motion_delta_m", 0.005)
        self.declare_parameter("skip_initial_home_move", False)
        self.declare_parameter("move_to_camera_home", True)
        self.declare_parameter("move_joint_home_before_camera_home", False)
        self.declare_parameter("camera_home_mode", "joint")
        self.declare_parameter("camera_home_joint_1_deg", 3.0)
        self.declare_parameter("camera_home_joint_2_deg", -12.7)
        self.declare_parameter("camera_home_joint_3_deg", 44.0)
        self.declare_parameter("camera_home_joint_4_deg", -9.0)
        self.declare_parameter("camera_home_joint_5_deg", 133.0)
        self.declare_parameter("camera_home_joint_6_deg", 90.0)
        self.declare_parameter("camera_home_x", 0.45)
        self.declare_parameter("camera_home_y", 0.00)
        self.declare_parameter("camera_home_z", 0.64)
        self.declare_parameter("camera_home_search_max_z", 0.64)
        self.declare_parameter("camera_home_search_min_z", 0.54)
        self.declare_parameter("camera_home_search_step_z", 0.02)
        self.declare_parameter("return_to_camera_home_after_attempt", True)
        self.declare_parameter("place_x", 0.45)
        self.declare_parameter("place_y", 0.0)
        self.declare_parameter("place_z", 0.30)

        self.model_path = self.get_parameter("model_path").value
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = self.get_parameter("device").value
        self.target_class = self.get_parameter("target_class").value
        self.auto_pick = parse_bool(self.get_parameter("auto_pick").value)
        self.auto_pick_interval = float(self.get_parameter("auto_pick_interval").value)
        self.exit_after_pick = parse_bool(self.get_parameter("exit_after_pick").value)
        self.depth_patch_radius = int(self.get_parameter("depth_patch_radius").value)
        self.min_depth_valid_ratio = float(
            self.get_parameter("min_depth_valid_ratio").value
        )
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.redetect_on_approach = parse_bool(
            self.get_parameter("redetect_on_approach").value
        )
        self.redetect_settle_sec = float(self.get_parameter("redetect_settle_sec").value)
        self.grasp_mode = str(self.get_parameter("grasp_mode").value).strip().lower()
        self.side_grasp_axis = parse_axis(self.get_parameter("side_grasp_axis").value)
        self.side_grasp_direction = float(
            self.get_parameter("side_grasp_direction").value
        )
        self.side_approach_offset = float(
            self.get_parameter("side_approach_offset").value
        )
        self.side_staging_offset = float(
            self.get_parameter("side_staging_offset").value
        )
        self.side_far_stage_enabled = parse_bool(
            self.get_parameter("side_far_stage_enabled").value
        )
        self.side_short_stage_backoff_m = max(
            0.0, float(self.get_parameter("side_short_stage_backoff_m").value)
        )
        self.side_stage_y_min = float(self.get_parameter("side_stage_y_min").value)
        self.side_stage_y_max = float(self.get_parameter("side_stage_y_max").value)
        self.side_grasp_offset = float(self.get_parameter("side_grasp_offset").value)
        self.side_grasp_z_offset = float(
            self.get_parameter("side_grasp_z_offset").value
        )
        self.side_grasp_stop_backoff_m = max(
            0.0, float(self.get_parameter("side_grasp_stop_backoff_m").value)
        )
        self.side_close_underreach_m = max(
            0.0, float(self.get_parameter("side_close_underreach_m").value)
        )
        self.side_low_retry_lift_m = max(
            0.0, float(self.get_parameter("side_low_retry_lift_m").value)
        )
        self.side_low_retry_attempts = max(
            0, int(self.get_parameter("side_low_retry_attempts").value)
        )
        self.side_auto_direction_by_cup_y = parse_bool(
            self.get_parameter("side_auto_direction_by_cup_y").value
        )
        self.side_candidate_plan_check_enabled = parse_bool(
            self.get_parameter("side_candidate_plan_check_enabled").value
        )
        self.side_linear_approach_enabled = parse_bool(
            self.get_parameter("side_linear_approach_enabled").value
        )
        self.side_final_slide_enabled = parse_bool(
            self.get_parameter("side_final_slide_enabled").value
        )
        self.side_fixed_grasp_z_enabled = parse_bool(
            self.get_parameter("side_fixed_grasp_z_enabled").value
        )
        self.side_fixed_grasp_z = float(
            self.get_parameter("side_fixed_grasp_z").value
        )
        self.side_project_bbox_center_to_fixed_z = parse_bool(
            self.get_parameter("side_project_bbox_center_to_fixed_z").value
        )
        self.table_collision_enabled = parse_bool(
            self.get_parameter("table_collision_enabled").value
        )
        self.table_collision_id = str(
            self.get_parameter("table_collision_id").value
        ).strip()
        self.table_surface_z = float(self.get_parameter("table_surface_z").value)
        self.table_thickness = max(
            0.001, float(self.get_parameter("table_thickness").value)
        )
        self.table_size_x = max(0.001, float(self.get_parameter("table_size_x").value))
        self.table_size_y = max(0.001, float(self.get_parameter("table_size_y").value))
        self.table_center_x = float(self.get_parameter("table_center_x").value)
        self.table_center_y = float(self.get_parameter("table_center_y").value)
        self.table_publish_repeats = max(
            1, int(self.get_parameter("table_publish_repeats").value)
        )
        self.table_collision_expand_to_workspace_walls = parse_bool(
            self.get_parameter("table_collision_expand_to_workspace_walls").value
        )
        self.safety_config_path = str(
            self.get_parameter("safety_config_path").value
        ).strip()
        self.safety_workspace_enforced = parse_bool(
            self.get_parameter("safety_workspace_enforced").value
        )
        self.workspace_boundary_collision_enabled = parse_bool(
            self.get_parameter("workspace_boundary_collision_enabled").value
        )
        self.workspace_boundary_collision_prefix = str(
            self.get_parameter("workspace_boundary_collision_prefix").value
        ).strip()
        self.workspace_boundary_wall_thickness = max(
            0.001,
            float(self.get_parameter("workspace_boundary_wall_thickness").value),
        )
        self.workspace_boundary_wall_clearance = max(
            0.0,
            float(self.get_parameter("workspace_boundary_wall_clearance").value),
        )
        self.gripper_open_settle_sec = max(
            0.0, float(self.get_parameter("gripper_open_settle_sec").value)
        )
        self.pre_pick_joint1_clearance_deg = max(
            0.0, float(self.get_parameter("pre_pick_joint1_clearance_deg").value)
        )
        self.center_check_enabled = parse_bool(
            self.get_parameter("center_check_enabled").value
        )
        self.center_check_settle_sec = max(
            0.0, float(self.get_parameter("center_check_settle_sec").value)
        )
        self.center_check_x = float(self.get_parameter("center_check_x").value)
        self.center_check_y = float(self.get_parameter("center_check_y").value)
        self.center_check_z = float(self.get_parameter("center_check_z").value)
        self.side_prepose_enabled = parse_bool(
            self.get_parameter("side_prepose_enabled").value
        )
        self.side_prepose_joint_order = list(
            self.get_parameter("side_prepose_joint_order").value
        )
        self.side_prepose_split_z = float(
            self.get_parameter("side_prepose_split_z").value
        )
        self.side_prepose_split_y = float(
            self.get_parameter("side_prepose_split_y").value
        )
        self.side_prepose_selection_mode = str(
            self.get_parameter("side_prepose_selection_mode").value
        ).strip().lower()
        self.side_prepose_joints_low_rad = []
        self.side_prepose_joints_high_rad = []
        self.side_prepose_joints_cup_left_rad = parameter_array_or_empty(
            self.get_parameter("side_prepose_joints_cup_left_rad").value
        )
        self.side_prepose_joints_cup_right_rad = parameter_array_or_empty(
            self.get_parameter("side_prepose_joints_cup_right_rad").value
        )
        self.side_move_to_initial_center_before_close = parse_bool(
            self.get_parameter("side_move_to_initial_center_before_close").value
        )
        self.side_orientation_mode = str(
            self.get_parameter("side_orientation_mode").value
        ).strip().lower()
        self.side_tool_roll_deg = float(
            self.get_parameter("side_tool_roll_deg").value
        )
        self.side_roll_deg = float(self.get_parameter("side_roll_deg").value)
        self.side_pitch_deg = float(self.get_parameter("side_pitch_deg").value)
        self.side_yaw_deg = float(self.get_parameter("side_yaw_deg").value)
        self.pick_z_offset = float(self.get_parameter("pick_z_offset").value)
        self.approach_offset = float(self.get_parameter("approach_offset").value)
        self.safe_z = float(self.get_parameter("safe_z").value)
        self.min_motion_z = float(self.get_parameter("min_motion_z").value)
        self.workspace_xy_clamp_enabled = parse_bool(
            self.get_parameter("workspace_xy_clamp_enabled").value
        )
        self.workspace_bounds_m = self.load_safety_workspace_bounds()
        self.return_home_after_task = parse_bool(
            self.get_parameter("return_home_after_task").value
        )
        self.verify_motion = parse_bool(self.get_parameter("verify_motion").value)
        self.motion_verify_tolerance = float(
            self.get_parameter("motion_verify_tolerance").value
        )
        self.joint_goal_tolerance_rad = max(
            0.0, float(self.get_parameter("joint_goal_tolerance_rad").value)
        )
        self.min_motion_delta_m = max(
            0.0, float(self.get_parameter("min_motion_delta_m").value)
        )
        self.skip_initial_home_move = parse_bool(
            self.get_parameter("skip_initial_home_move").value
        )
        self.move_to_camera_home = parse_bool(
            self.get_parameter("move_to_camera_home").value
        )
        self.move_joint_home_before_camera_home = parse_bool(
            self.get_parameter("move_joint_home_before_camera_home").value
        )
        self.camera_home_mode = str(self.get_parameter("camera_home_mode").value).strip().lower()
        self.camera_home_joint_positions = [
            math.radians(float(self.get_parameter(f"camera_home_joint_{idx}_deg").value))
            for idx in range(1, 7)
        ]
        self.camera_home_x = float(self.get_parameter("camera_home_x").value)
        self.camera_home_y = float(self.get_parameter("camera_home_y").value)
        self.camera_home_z = float(self.get_parameter("camera_home_z").value)
        self.camera_home_search_max_z = float(
            self.get_parameter("camera_home_search_max_z").value
        )
        self.camera_home_search_min_z = float(
            self.get_parameter("camera_home_search_min_z").value
        )
        self.camera_home_search_step_z = max(
            0.01, float(self.get_parameter("camera_home_search_step_z").value)
        )
        self.return_to_camera_home_after_attempt = parse_bool(
            self.get_parameter("return_to_camera_home_after_attempt").value
        )
        self.place_x = float(self.get_parameter("place_x").value)
        self.place_y = float(self.get_parameter("place_y").value)
        self.place_z = float(self.get_parameter("place_z").value)

        model_file = Path(self.model_path).expanduser()
        if not model_file.exists():
            raise FileNotFoundError(f"YOLO model not found: {model_file}")

        self.get_logger().info(f"Loading YOLO model: {model_file}")
        self.model = YOLO(str(model_file))
        self.get_logger().info(f"YOLO classes: {self.model.names}")
        if self.target_class not in self.model.names.values():
            raise ValueError(
                f"target_class='{self.target_class}' is not in model classes "
                f"{self.model.names}"
            )
        if self.grasp_mode not in {"side", "top"}:
            raise ValueError("grasp_mode must be 'side' or 'top'")
        if self.camera_home_mode not in {"joint", "pose"}:
            raise ValueError("camera_home_mode must be 'joint' or 'pose'")
        if self.side_grasp_axis not in {"x", "y"}:
            raise ValueError("side_grasp_axis must be 'x' or 'y'")
        self.side_grasp_direction = 1.0 if self.side_grasp_direction >= 0 else -1.0
        if self.side_orientation_mode not in {"approach", "euler", "home"}:
            raise ValueError(
                "side_orientation_mode must be 'approach', 'euler', or 'home'"
            )
        if self.side_far_stage_enabled and self.side_staging_offset < self.side_approach_offset:
            self.get_logger().warning(
                "side_staging_offset is smaller than side_approach_offset; "
                "using side_approach_offset for staging."
            )
            self.side_staging_offset = self.side_approach_offset
        if self.side_auto_direction_by_cup_y and self.side_grasp_axis != "y":
            self.get_logger().warning(
                "side_auto_direction_by_cup_y only applies to y-axis side grasps; "
                "using configured side_grasp_direction for this axis."
            )
        if self.side_final_slide_enabled:
            self.get_logger().warning(
                "side_final_slide_enabled was requested, but it is disabled at runtime "
                "to avoid pushing the cup after reaching the side close pose."
            )
            self.side_final_slide_enabled = False
        if (
            self.side_fixed_grasp_z_enabled
            and self.side_fixed_grasp_z < self.min_motion_z
        ):
            self.get_logger().warning(
                f"side_fixed_grasp_z={self.side_fixed_grasp_z:.3f} is below "
                f"min_motion_z={self.min_motion_z:.3f}; command min_motion_z lower "
                "only after checking table/fixture clearance."
            )
        if self.side_fixed_grasp_z_enabled:
            self.get_logger().info(
                "side_fixed_grasp_z is interpreted as a base_link Z target for "
                f"{EE_LINK}; table/cup/lid geometry is not inferred from it."
            )
        if not self.table_collision_enabled:
            self.get_logger().warning(
                "table_collision_enabled=false: MoveIt will only clamp the EE target Z, "
                "and will not avoid robot-link/table collisions."
            )

        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.last_detection = None
        self.picking = False
        self.has_picked_once = False
        self.last_pick_time = 0.0
        self.last_status = "waiting for command"

        calib_file = (
            Path(get_package_share_directory("dsr_practice"))
            / "config"
            / "T_gripper2camera.npy"
        )
        self.gripper2cam = np.load(str(calib_file)).astype(float)
        self.gripper2cam[:3, 3] /= 1000.0
        self.get_logger().info(f"Loaded hand-eye calibration: {calib_file}")

        self.gripper = None
        self.robot = None
        self.arm = None
        self.robot_model = None
        self.ompl_params = None
        self.pilz_params = None
        self.pilz_lin_params = None
        self.collision_object_pub = None
        self._motion_stack_ready = False

        self.home_ori = DOWN_ORI

        self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self._camera_info_callback,
            10,
        )
        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self._color_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self._depth_callback,
            qos_profile_sensor_data,
        )

    def ensure_motion_stack_ready(self):
        if self._motion_stack_ready:
            return True
        log = self.get_logger()

        log.info("Initializing RG2 gripper client...")
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        log.info("RG2 gripper client initialized")

        log.info("Initializing MoveItPy...")
        self.robot = MoveItPy(node_name="yolo_cup_pick_moveit_py")
        self.arm = self.robot.get_planning_component(GROUP_NAME)
        self.robot_model = self.robot.get_robot_model()
        log.info("MoveItPy initialized")

        self.collision_object_pub = self.create_publisher(
            CollisionObject,
            "/collision_object",
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.publish_table_collision_if_enabled()
        self.publish_workspace_boundary_collision_if_enabled()

        self.ompl_params = PlanRequestParameters(self.robot)
        self.ompl_params.planning_pipeline = "ompl"
        self.ompl_params.planner_id = "RRTConnect"
        self.ompl_params.max_velocity_scaling_factor = 0.2
        self.ompl_params.max_acceleration_scaling_factor = 0.1
        self.ompl_params.planning_time = 5.0

        self.pilz_params = PlanRequestParameters(self.robot)
        self.pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.pilz_params.planner_id = "PTP"
        self.pilz_params.max_velocity_scaling_factor = 0.12
        self.pilz_params.max_acceleration_scaling_factor = 0.08
        self.pilz_params.planning_time = 3.0

        self.pilz_lin_params = PlanRequestParameters(self.robot)
        self.pilz_lin_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.pilz_lin_params.planner_id = "LIN"
        self.pilz_lin_params.max_velocity_scaling_factor = 0.08
        self.pilz_lin_params.max_acceleration_scaling_factor = 0.05
        self.pilz_lin_params.planning_time = 3.0

        self._motion_stack_ready = True
        return True

    def load_safety_workspace_bounds(self):
        log = self.get_logger()
        if not self.safety_workspace_enforced:
            log.warning("safety_workspace_enforced=false: YAML workspace bounds are not enforced")
            return None
        if not self.safety_config_path:
            raise ValueError("safety_workspace_enforced=true but safety_config_path is empty")

        safety_path = Path(self.safety_config_path).expanduser()
        if not safety_path.exists():
            raise FileNotFoundError(f"safety config does not exist: {safety_path}")
        with safety_path.open("r", encoding="utf-8") as stream:
            safety_config = yaml.safe_load(stream) or {}
        motion_config = safety_config.get("motion", {})
        if not isinstance(motion_config, dict):
            raise ValueError(f"motion section is not a YAML map: {safety_path}")

        bounds = parse_workspace_bounds(motion_config.get("workspace_bounds_m"))
        if bounds is None:
            raise ValueError(
                f"motion.workspace_bounds_m must contain x/y/z min/max values: {safety_path}"
            )

        min_z = motion_config.get("min_z_m")
        if min_z is not None:
            min_z = float(min_z)
            if abs(min_z - bounds["z_min"]) > 1e-9:
                log.warning(
                    f"safety min_z_m={min_z:.3f} differs from workspace z_min="
                    f"{bounds['z_min']:.3f}; using the stricter/higher value"
                )
                bounds["z_min"] = max(bounds["z_min"], min_z)
            effective_min_z = max(self.min_motion_z, bounds["z_min"])
            if self.min_motion_z != effective_min_z:
                log.info(
                    f"min_motion_z {self.min_motion_z:.3f} -> {effective_min_z:.3f} "
                    "from safety workspace"
                )
                self.min_motion_z = effective_min_z
            bounds["z_min"] = effective_min_z

        log.info(
            "Loaded enforced safety workspace from "
            f"{safety_path}: x=[{bounds['x_min']:.3f}, {bounds['x_max']:.3f}], "
            f"y=[{bounds['y_min']:.3f}, {bounds['y_max']:.3f}], "
            f"z=[{bounds['z_min']:.3f}, {bounds['z_max']:.3f}]"
        )
        return bounds

    def validate_workspace_goal(self, x, y, z, label="pose goal"):
        if not self.safety_workspace_enforced or self.workspace_bounds_m is None:
            return True
        bounds = self.workspace_bounds_m
        violations = []
        if x < bounds["x_min"] or x > bounds["x_max"]:
            violations.append(
                f"x={x:.3f} outside [{bounds['x_min']:.3f}, {bounds['x_max']:.3f}]"
            )
        if y < bounds["y_min"] or y > bounds["y_max"]:
            violations.append(
                f"y={y:.3f} outside [{bounds['y_min']:.3f}, {bounds['y_max']:.3f}]"
            )
        if z < bounds["z_min"] or z > bounds["z_max"]:
            violations.append(
                f"z={z:.3f} outside [{bounds['z_min']:.3f}, {bounds['z_max']:.3f}]"
            )
        if violations:
            self.get_logger().error(
                f"{label} rejected by safety workspace: " + "; ".join(violations)
            )
            return False
        return True

    def make_box_collision_object(self, object_id, center_xyz, size_xyz):
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = BASE_FRAME

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in size_xyz]

        pose = Pose()
        pose.position.x = float(center_xyz[0])
        pose.position.y = float(center_xyz[1])
        pose.position.z = float(center_xyz[2])
        pose.orientation.w = 1.0

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def publish_table_collision_if_enabled(self):
        if not self.table_collision_enabled:
            return

        table_center_x = self.table_center_x
        table_center_y = self.table_center_y
        table_size_x = self.table_size_x
        table_size_y = self.table_size_y
        if (
            self.table_collision_expand_to_workspace_walls
            and self.workspace_bounds_m is not None
        ):
            clearance = (
                self.workspace_boundary_wall_clearance
                if self.workspace_boundary_collision_enabled
                else 0.0
            )
            x_min = self.workspace_bounds_m["x_min"] - clearance
            x_max = self.workspace_bounds_m["x_max"] + clearance
            y_min = self.workspace_bounds_m["y_min"] - clearance
            y_max = self.workspace_bounds_m["y_max"] + clearance
            table_center_x = (x_min + x_max) * 0.5
            table_center_y = (y_min + y_max) * 0.5
            table_size_x = x_max - x_min
            table_size_y = y_max - y_min

        center_z = self.table_surface_z - self.table_thickness * 0.5
        table = self.make_box_collision_object(
            self.table_collision_id or "side_grip_table",
            [table_center_x, table_center_y, center_z],
            [table_size_x, table_size_y, self.table_thickness],
        )

        for _ in range(self.table_publish_repeats):
            self.collision_object_pub.publish(table)
            time.sleep(0.05)

        self.get_logger().info(
            "Added table collision object "
            f"id={table.id!r}, frame={BASE_FRAME}, "
            f"surface_z={self.table_surface_z:.3f}, "
            f"size=({table_size_x:.3f}, {table_size_y:.3f}, "
            f"{self.table_thickness:.3f}), "
            f"center=({table_center_x:.3f}, {table_center_y:.3f}, "
            f"{center_z:.3f}), "
            f"expanded_to_workspace_walls={self.table_collision_expand_to_workspace_walls}"
        )

    def publish_workspace_boundary_collision_if_enabled(self):
        if not self.workspace_boundary_collision_enabled:
            return
        if not self.safety_workspace_enforced or self.workspace_bounds_m is None:
            self.get_logger().warning(
                "workspace_boundary_collision_enabled=true but safety workspace "
                "is not enforced/loaded; skipping boundary collision objects"
            )
            return

        bounds = self.workspace_bounds_m
        thickness = self.workspace_boundary_wall_thickness
        clearance = self.workspace_boundary_wall_clearance
        prefix = self.workspace_boundary_collision_prefix or "side_grip_workspace"

        x_min = bounds["x_min"] - clearance
        x_max = bounds["x_max"] + clearance
        y_min = bounds["y_min"] - clearance
        y_max = bounds["y_max"] + clearance
        z_min = bounds["z_min"]
        z_max = bounds["z_max"]
        x_span = x_max - x_min
        y_span = y_max - y_min
        z_span = z_max - z_min
        x_mid = (x_min + x_max) * 0.5
        y_mid = (y_min + y_max) * 0.5
        z_mid = (z_min + z_max) * 0.5

        objects = [
            self.make_box_collision_object(
                f"{prefix}_x_min_wall",
                [x_min - thickness * 0.5, y_mid, z_mid],
                [thickness, y_span + 2.0 * thickness, z_span],
            ),
            self.make_box_collision_object(
                f"{prefix}_x_max_wall",
                [x_max + thickness * 0.5, y_mid, z_mid],
                [thickness, y_span + 2.0 * thickness, z_span],
            ),
            self.make_box_collision_object(
                f"{prefix}_y_min_wall",
                [x_mid, y_min - thickness * 0.5, z_mid],
                [x_span + 2.0 * thickness, thickness, z_span],
            ),
            self.make_box_collision_object(
                f"{prefix}_y_max_wall",
                [x_mid, y_max + thickness * 0.5, z_mid],
                [x_span + 2.0 * thickness, thickness, z_span],
            ),
        ]

        for _ in range(self.table_publish_repeats):
            for collision_object in objects:
                self.collision_object_pub.publish(collision_object)
            time.sleep(0.05)

        self.get_logger().info(
            "Added workspace boundary collision objects "
            f"prefix={prefix!r}, thickness={thickness:.3f}, "
            f"clearance={clearance:.3f}, "
            f"x=[{x_min:.3f}, {x_max:.3f}], "
            f"y=[{y_min:.3f}, {y_max:.3f}], "
            f"z=[{z_min:.3f}, {z_max:.3f}]"
        )

    def center_check_redetect(self, initial_base_xyz):
        """Optionally move to a high pose and re-detect cup center.

        Returns a refined base_xyz or None if re-detection fails.
        This does not change the original "initial" center unless the caller chooses to.
        """
        log = self.get_logger()
        if not self.center_check_enabled:
            return None

        log.info(
            "Center-check enabled: moving to high observe pose "
            f"({self.center_check_x:.3f}, {self.center_check_y:.3f}, {self.center_check_z:.3f})"
        )
        if not self.plan_and_execute(
            pose_goal=make_pose(
                self.center_check_x,
                self.center_check_y,
                self.center_check_z,
                self.home_ori,
            ),
            params=self.ompl_params,
        ):
            log.warning("Center-check pose move failed; continuing with initial detection")
            return None

        self.spin_for_camera_update(self.center_check_settle_sec)

        if self.color_image is None:
            log.warning("Center-check skipped: no color image")
            return None
        detections = self.detect_objects(self.color_image)
        det = self.select_redetect_target(detections)
        if det is None:
            log.warning("Center-check re-detect failed: no target detection")
            return None
        refined = self.base_from_detection(det, log_prefix="center-check")
        if refined is None:
            log.warning("Center-check re-detect failed: no valid depth/base point")
            return None

        dx = float(refined[0]) - float(initial_base_xyz[0])
        dy = float(refined[1]) - float(initial_base_xyz[1])
        dz = float(refined[2]) - float(initial_base_xyz[2])
        log.info(
            "Center-check delta from initial detection: "
            f"dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} (meters)"
        )
        return refined

    def move_to_side_prepose_if_configured(self, cup_base_xyz: np.ndarray) -> bool:
        log = self.get_logger()
        if not self.side_prepose_enabled:
            return True
        joint_order = [str(name) for name in self.side_prepose_joint_order]
        if not joint_order:
            log.error("side_prepose_enabled=true but side_prepose_joint_order is empty")
            return False

        if cup_base_xyz.shape != (3,):
            log.error(f"invalid cup_base_xyz shape {cup_base_xyz.shape}; expected (3,)")
            return False

        mode = self.side_prepose_selection_mode or "y"
        if mode not in {"y", "z"}:
            log.error(
                "side_prepose_selection_mode must be 'y' or 'z' "
                f"(got {self.side_prepose_selection_mode!r})"
            )
            return False

        if mode == "z":
            use_high = float(cup_base_xyz[2]) >= self.side_prepose_split_z
            target = self.side_prepose_joints_high_rad if use_high else self.side_prepose_joints_low_rad
            label = "high" if use_high else "low"
            selector_detail = f"cup base z={float(cup_base_xyz[2]):.3f} (split_z={self.side_prepose_split_z:.3f})"
        else:
            # Cup on "left" is +Y in base_link (commonly). Use a threshold so operators can tune.
            cup_is_left = float(cup_base_xyz[1]) >= self.side_prepose_split_y
            target = self.side_prepose_joints_cup_left_rad if cup_is_left else self.side_prepose_joints_cup_right_rad
            label = "cup_left" if cup_is_left else "cup_right"
            selector_detail = f"cup base y={float(cup_base_xyz[1]):.3f} (split_y={self.side_prepose_split_y:.3f})"

        if not target:
            log.error(
                f"side_prepose_enabled=true but side_prepose_joints_{label}_rad is empty; "
                "teach joints and set params (do not hardcode in code)."
            )
            return False
        if len(target) != len(joint_order):
            log.error(
                f"side_prepose_joints_{label}_rad length mismatch: "
                f"expected {len(joint_order)}, got {len(target)}"
            )
            return False

        log.info(
            f"Move rule-based side prepose ({label}) selected by {selector_detail}"
        )
        target_state = RobotState(self.robot_model)
        target_by_name = dict(zip(joint_order, [float(val) for val in target]))
        missing = [name for name in ARM_JOINT_ORDER if name not in target_by_name]
        if missing:
            log.error(
                "side_prepose_joint_order must include all arm joints; "
                f"missing {missing}"
            )
            return False
        joint_positions = [target_by_name[name] for name in ARM_JOINT_ORDER]
        target_state.set_joint_group_positions(GROUP_NAME, joint_positions)
        target_state.update()
        return self.plan_and_execute(
            state_goal=target_state,
            params=self.pilz_params,
            joint_goal_names=ARM_JOINT_ORDER,
            joint_goal_positions=joint_positions,
        )

    def _camera_info_callback(self, msg):
        self.intrinsics = {
            "fx": msg.k[0],
            "fy": msg.k[4],
            "cx": msg.k[2],
            "cy": msg.k[5],
        }

    def _color_callback(self, msg):
        self.color_image = self.image_msg_to_bgr(msg)

    def _depth_callback(self, msg):
        self.depth_image = self.depth_msg_to_array(msg)

    def image_msg_to_bgr(self, msg):
        enc = (msg.encoding or "").lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if enc in {"rgb8", "bgr8"}:
            image = data.reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if enc == "rgb8" else image
        if enc in {"rgba8", "bgra8"}:
            image = data.reshape((msg.height, msg.width, 4))
            if enc == "rgba8":
                return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if enc == "mono8":
            image = data.reshape((msg.height, msg.width))
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        raise RuntimeError(f"unsupported color image encoding: {msg.encoding}")

    def depth_msg_to_array(self, msg):
        enc = (msg.encoding or "").lower()
        if enc in {"16uc1", "mono16"}:
            return np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
        if enc == "32fc1":
            return np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
        raise RuntimeError(f"unsupported depth image encoding: {msg.encoding}")

    def plan_and_execute(
        self,
        pose_goal=None,
        state_goal=None,
        params=None,
        joint_goal_names=None,
        joint_goal_positions=None,
    ):
        log = self.get_logger()
        start_state = self.current_robot_state_from_joint_states(timeout_sec=1.0)
        if start_state is not None:
            self.arm.set_start_state(robot_state=start_state)
            start_matrix = get_ee_matrix_from_robot_state(start_state)
        else:
            log.warning("Could not seed MoveIt start state from /joint_states; falling back to current state")
            self.arm.set_start_state_to_current_state()
            start_matrix = get_ee_matrix(self.robot)
        start_xyz = start_matrix[:3, 3].copy()
        goal_xyz = None

        if pose_goal is not None:
            x = pose_goal.pose.position.x
            y = pose_goal.pose.position.y
            z = pose_goal.pose.position.z
            x, y, z = clamp_to_safe_workspace(
                x,
                y,
                z,
                log,
                self.min_motion_z,
                self.workspace_xy_clamp_enabled,
            )
            pose_goal.pose.position.x = x
            pose_goal.pose.position.y = y
            pose_goal.pose.position.z = z
            if not self.validate_workspace_goal(x, y, z, "pose goal"):
                return False
            goal_xyz = np.array([x, y, z], dtype=float)
            log.info(
                f"Planning pose goal -> ({x:.3f}, {y:.3f}, {z:.3f}) "
                f"from ({start_xyz[0]:.3f}, {start_xyz[1]:.3f}, {start_xyz[2]:.3f})"
            )
            self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=EE_LINK)
        elif state_goal is not None:
            log.info(
                f"Planning joint/state goal from EE "
                f"({start_xyz[0]:.3f}, {start_xyz[1]:.3f}, {start_xyz[2]:.3f})"
            )
            self.arm.set_goal_state(robot_state=state_goal)
        else:
            log.error("No pose/state goal was provided")
            return False

        plan_result = self.arm.plan(parameters=params) if params else self.arm.plan()
        if not plan_result:
            log.error("Planning failed")
            return False

        execute_result = self.robot.execute(
            group_name=GROUP_NAME,
            robot_trajectory=plan_result.trajectory,
            blocking=True,
        )
        if execute_result is False:
            log.error("MoveIt execution reported failure; stopping this motion step")
            return False
        self.spin_for_camera_update(0.2)

        end_state = self.current_robot_state_from_joint_states(timeout_sec=1.0)
        if end_state is not None:
            end_matrix = get_ee_matrix_from_robot_state(end_state)
        else:
            log.warning("Could not verify EE pose from /joint_states; falling back to planning scene")
            end_matrix = get_ee_matrix(self.robot)
        end_xyz = end_matrix[:3, 3].copy()
        moved = float(np.linalg.norm(end_xyz - start_xyz))
        requested_pose_delta = (
            float(np.linalg.norm(goal_xyz - start_xyz)) if goal_xyz is not None else 0.0
        )
        if (
            goal_xyz is not None
            and requested_pose_delta >= self.min_motion_delta_m
            and self.min_motion_delta_m > 0.0
            and moved < self.min_motion_delta_m
        ):
            log.error(
                f"MoveIt execution produced no observed EE motion "
                f"(moved={moved:.3f} m < {self.min_motion_delta_m:.3f} m); "
                "stopping this motion step"
            )
            return False
        if goal_xyz is None:
            log.info(
                f"Execution finished. EE moved {moved:.3f} m -> "
                f"({end_xyz[0]:.3f}, {end_xyz[1]:.3f}, {end_xyz[2]:.3f})"
            )
            if joint_goal_names is not None and joint_goal_positions is not None:
                if not self.verify_joint_goal_reached(
                    joint_goal_names,
                    joint_goal_positions,
                ):
                    return False
        else:
            goal_error = float(np.linalg.norm(end_xyz - goal_xyz))
            log.info(
                f"Execution finished. EE moved {moved:.3f} m, "
                f"goal_error={goal_error:.3f} m -> "
                f"({end_xyz[0]:.3f}, {end_xyz[1]:.3f}, {end_xyz[2]:.3f})"
            )
            if self.verify_motion and goal_error > self.motion_verify_tolerance:
                log.error(
                    "MoveIt execution did not reach the requested pose. "
                    "Check that the real robot MoveIt/trajectory controller is running."
                )
                return False
        return True

    def can_plan_pose_goal(self, pose_goal, params=None, label="candidate"):
        log = self.get_logger()
        start_state = self.current_robot_state_from_joint_states(timeout_sec=1.0)
        if start_state is not None:
            self.arm.set_start_state(robot_state=start_state)
        else:
            log.warning(
                f"{label}: could not seed MoveIt start state from /joint_states; "
                "falling back to current state"
            )
            self.arm.set_start_state_to_current_state()

        x = pose_goal.pose.position.x
        y = pose_goal.pose.position.y
        z = pose_goal.pose.position.z
        x, y, z = clamp_to_safe_workspace(
            x,
            y,
            z,
            log,
            self.min_motion_z,
            self.workspace_xy_clamp_enabled,
        )
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        if not self.validate_workspace_goal(x, y, z, label):
            return False
        log.info(f"{label}: plan-check pose -> ({x:.3f}, {y:.3f}, {z:.3f})")
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=EE_LINK)
        plan_result = self.arm.plan(parameters=params) if params else self.arm.plan()
        if not plan_result:
            log.warning(f"{label}: plan-check failed")
            return False
        log.info(f"{label}: plan-check OK")
        return True

    def joint_position_error(self, target, current):
        return abs(math.atan2(math.sin(target - current), math.cos(target - current)))

    def verify_joint_goal_reached(self, joint_names, joint_positions):
        log = self.get_logger()
        joint_map = self.read_joint_state_map(timeout_sec=1.0)
        if joint_map is None:
            log.error("Cannot verify joint goal: no /joint_states received")
            return False

        names = [str(name) for name in joint_names]
        targets = [float(pos) for pos in joint_positions]
        missing = [name for name in names if name not in joint_map]
        if missing:
            log.error(f"Cannot verify joint goal: /joint_states missing {missing}")
            return False

        errors = [
            self.joint_position_error(target, float(joint_map[name]))
            for name, target in zip(names, targets)
        ]
        max_error = max(errors) if errors else 0.0
        if max_error > self.joint_goal_tolerance_rad:
            worst_index = int(np.argmax(errors))
            worst_name = names[worst_index]
            log.error(
                "Joint goal not reached: "
                f"{worst_name} error={max_error:.4f} rad "
                f"> tolerance={self.joint_goal_tolerance_rad:.4f} rad"
            )
            return False

        log.info(
            "Joint goal verified: "
            f"max_error={max_error:.4f} rad "
            f"<= tolerance={self.joint_goal_tolerance_rad:.4f} rad"
        )
        return True

    def move_joint_home(self):
        home_state = RobotState(self.robot_model)
        home_state.set_joint_group_positions(GROUP_NAME, HOME_JOINTS_RAD)
        home_state.update()
        if not self.plan_and_execute(
            state_goal=home_state,
            params=self.ompl_params,
            joint_goal_names=ARM_JOINT_ORDER,
            joint_goal_positions=HOME_JOINTS_RAD,
        ):
            return False

        transform = get_ee_matrix(self.robot)
        self.update_home_orientation_from_matrix(transform)
        return True

    def move_home(self):
        if self.move_to_camera_home:
            return self.move_camera_home()
        return self.move_joint_home()

    def move_camera_joint_home(self):
        log = self.get_logger()
        target_state = RobotState(self.robot_model)
        target_state.set_joint_group_positions(GROUP_NAME, self.camera_home_joint_positions)
        target_state.update()
        joint_degrees = [math.degrees(value) for value in self.camera_home_joint_positions]
        log.info(
            "Move CAMERA JOINT HOME -> "
            + ", ".join(
                f"{name}={degrees:.1f}deg"
                for name, degrees in zip(ARM_JOINT_ORDER, joint_degrees)
            )
        )
        if not self.plan_and_execute(
            state_goal=target_state,
            params=self.ompl_params,
            joint_goal_names=ARM_JOINT_ORDER,
            joint_goal_positions=self.camera_home_joint_positions,
        ):
            return False

        transform = get_ee_matrix(self.robot)
        self.update_home_orientation_from_matrix(transform)
        return True

    def update_home_orientation_from_matrix(self, transform):
        qx, qy, qz, qw = Rotation.from_matrix(transform[:3, :3]).as_quat()
        self.home_ori = {
            "x": float(qx),
            "y": float(qy),
            "z": float(qz),
            "w": float(qw),
        }

    def read_joint_state_map(self, timeout_sec=3.0, settle_sec=0.15):
        latest = None

        def callback(msg):
            nonlocal latest
            if msg.name and len(msg.name) == len(msg.position):
                latest = dict(zip(msg.name, msg.position))

        subscription = self.create_subscription(JointState, "/joint_states", callback, 10)
        deadline = time.monotonic() + max(timeout_sec, 0.1)
        settle_deadline = None
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if latest is not None and settle_deadline is None:
                    settle_deadline = time.monotonic() + max(0.0, settle_sec)
                if settle_deadline is not None and time.monotonic() >= settle_deadline:
                    break
            return latest
        finally:
            self.destroy_subscription(subscription)

    def current_robot_state_from_joint_states(self, timeout_sec=1.0):
        joint_map = self.read_joint_state_map(timeout_sec=timeout_sec)
        if joint_map is None:
            return None
        if any(name not in joint_map for name in ARM_JOINT_ORDER):
            self.get_logger().warning("/joint_states missing one or more arm joints")
            return None
        state = RobotState(self.robot_model)
        joint_positions = [float(joint_map[name]) for name in ARM_JOINT_ORDER]
        state.set_joint_group_positions(GROUP_NAME, joint_positions)
        state.update()
        return state

    def move_joint1_clearance_before_side_grip(self):
        log = self.get_logger()
        delta = float(self.pre_pick_joint1_clearance_deg)
        if delta <= 0.0:
            log.info("pre-pick joint_1 clearance detour disabled")
            return True
        state = self.read_joint_state_map(timeout_sec=3.0)
        if state is None or "joint_1" not in state:
            log.error("cannot read /joint_states for pre-pick joint_1 clearance")
            return False
        joint_order = ARM_JOINT_ORDER
        if any(name not in state for name in joint_order):
            log.error("/joint_states missing one or more arm joints for joint_1 clearance")
            return False
        joint_positions = [float(state[name]) for name in joint_order]
        before_deg = math.degrees(joint_positions[0])
        joint_positions[0] = math.radians(before_deg + delta)
        log.info(
            "pre-pick joint_1 clearance detour before side grip: "
            f"joint_1 {before_deg:.1f} -> {before_deg + delta:.1f} deg"
        )
        target_state = RobotState(self.robot_model)
        target_state.set_joint_group_positions(GROUP_NAME, joint_positions)
        target_state.update()
        return self.plan_and_execute(
            state_goal=target_state,
            params=self.ompl_params,
            joint_goal_names=joint_order,
            joint_goal_positions=joint_positions,
        )

    def move_camera_home(self):
        if self.camera_home_mode == "joint":
            return self.move_camera_joint_home()

        log = self.get_logger()
        candidate_zs = []
        search_top_z = max(self.camera_home_z, self.camera_home_search_max_z)
        search_bottom_z = min(
            self.camera_home_z,
            self.camera_home_search_max_z,
            self.camera_home_search_min_z,
        )
        z = search_top_z
        while z >= search_bottom_z - 1e-6:
            if all(abs(z - candidate) > 1e-6 for candidate in candidate_zs):
                candidate_zs.append(round(z, 4))
            z -= self.camera_home_search_step_z

        for fallback_z in (0.68, 0.62, 0.58, 0.54):
            if fallback_z > search_top_z + 1e-6:
                continue
            if all(abs(fallback_z - candidate) > 1e-6 for candidate in candidate_zs):
                candidate_zs.append(fallback_z)

        log.info(
            "Searching highest reachable CAMERA HOME Z from "
            f"{candidate_zs[0]:.3f} down to {candidate_zs[-1]:.3f} "
            f"(step={self.camera_home_search_step_z:.3f})"
        )

        for idx, z in enumerate(candidate_zs):
            if idx > 0:
                log.warning(
                    f"Camera home IK failed at higher z; retrying z={z:.3f}"
                )

            log.info(
                f"Move CAMERA HOME -> ({self.camera_home_x:.3f}, "
                f"{self.camera_home_y:.3f}, {z:.3f})"
            )
            if not self.plan_and_execute(
                pose_goal=make_pose(
                    self.camera_home_x,
                    self.camera_home_y,
                    z,
                    self.home_ori,
                ),
                params=self.ompl_params,
            ):
                continue

            self.camera_home_z = z
            transform = get_ee_matrix(self.robot)
            self.update_home_orientation_from_matrix(transform)
            return True

        return False

    def wait_until_gripper_idle(self, timeout_sec=GRIPPER_OPEN_TIMEOUT_SEC):
        if self.gripper is None:
            self.get_logger().error("Gripper client is not initialized")
            return False
        log = self.get_logger()
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            status = self.gripper.get_status()
            busy = bool(status[0])
            if not busy:
                try:
                    width_mm = self.gripper.get_width_with_offset()
                    log.info(f"Gripper ready. current width={width_mm:.1f} mm")
                except Exception as exc:
                    log.warning(f"Gripper width read failed: {exc}")
                return True
            time.sleep(GRIPPER_STATUS_POLL_SEC)

        log.warning("Timed out waiting for gripper to finish opening.")
        return False

    def open_gripper_max(self, wait=False):
        if not self.wait_until_gripper_idle():
            return False
        self.get_logger().info(
            f"Open gripper to max width={GRIPPER_OPEN_WIDTH} "
            f"({GRIPPER_OPEN_WIDTH / 10.0:.1f} mm)"
        )
        self.gripper.move_gripper(GRIPPER_OPEN_WIDTH, GRIPPER_FORCE)
        if wait:
            return self.wait_until_gripper_idle()
        return True

    def detect_objects(self, image):
        results = self.model.predict(
            source=image,
            imgsz=self.imgsz,
            conf=self.conf,
            device=self.device,
            verbose=False,
        )
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            self.last_detection = None
            return []

        detections = []
        for box in boxes:
            cls_id = int(box.cls[0])
            class_name = self.model.names.get(cls_id, str(cls_id))
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            detections.append(
                {
                    "bbox": (x1, y1, x2, y2),
                    "cx": int((x1 + x2) / 2),
                    "cy": int((y1 + y2) / 2),
                    "conf": float(box.conf[0]),
                    "class_name": class_name,
                }
            )

        target_detections = [
            det for det in detections if det["class_name"] == self.target_class
        ]
        if not target_detections:
            self.last_detection = None
        else:
            self.last_detection = max(
                target_detections,
                key=lambda det: det["conf"],
            )

        return detections

    def depth_candidates_from_bbox(self, bbox):
        x1, y1, x2, y2 = bbox
        h, w = self.depth_image.shape[:2]
        u = int(round(0.5 * (float(x1) + float(x2))))
        v = int(round(0.5 * (float(y1) + float(y2))))
        u = max(0, min(w - 1, u))
        v = max(0, min(h - 1, v))
        return [(u, v)]

    def depth_patch_at(self, u, v):
        h, w = self.depth_image.shape[:2]
        r = self.depth_patch_radius
        patch = self.depth_image[
            max(0, v - r) : min(h, v + r + 1),
            max(0, u - r) : min(w, u + r + 1),
        ]
        valid = patch[patch > 0]
        valid_ratio = valid.size / float(patch.size)
        if valid.size == 0 or valid_ratio < self.min_depth_valid_ratio:
            return None

        z_raw = float(np.median(valid))
        z_m = z_raw / 1000.0 if self.depth_image.dtype == np.uint16 else z_raw
        if z_m < self.min_depth_m or z_m > self.max_depth_m:
            return None
        return u, v, z_m, valid_ratio

    def depth_from_bbox(self, bbox, log_reason=False):
        log = self.get_logger()
        if self.depth_image is None:
            if log_reason:
                log.warning("Depth image is not ready")
            return None

        valid_samples = []
        for u, v in self.depth_candidates_from_bbox(bbox):
            sample = self.depth_patch_at(u, v)
            if sample is not None:
                valid_samples.append(sample)

        if not valid_samples:
            if log_reason:
                log.warning(
                    "No valid depth found at target bbox center. "
                    "Try a larger depth_patch_radius or lower min_depth_valid_ratio."
                )
            return None

        u, v, z_m, valid_ratio = valid_samples[0]
        if log_reason:
            log.info(
                f"Depth sample selected at bbox center ({u}, {v}): "
                f"{z_m:.3f} m, valid_ratio={valid_ratio:.2f}"
            )
        return u, v, z_m

    def pixel_to_camera(self, u, v, z_m):
        fx = self.intrinsics["fx"]
        fy = self.intrinsics["fy"]
        cx = self.intrinsics["cx"]
        cy = self.intrinsics["cy"]

        cam_x = (u - cx) * z_m / fx
        cam_y = (v - cy) * z_m / fy
        cam_z = z_m
        return np.array([cam_x, cam_y, cam_z], dtype=float)

    def camera_to_base(self, camera_xyz):
        coord = np.append(camera_xyz, 1.0)
        base2ee = get_ee_matrix(self.robot)
        base2cam = base2ee @ self.gripper2cam
        return (base2cam @ coord)[:3]

    def bbox_center_to_fixed_base_z(self, bbox, base_z):
        if self.intrinsics is None:
            return None
        x1, y1, x2, y2 = bbox
        u = 0.5 * (float(x1) + float(x2))
        v = 0.5 * (float(y1) + float(y2))
        fx = self.intrinsics["fx"]
        fy = self.intrinsics["fy"]
        cx = self.intrinsics["cx"]
        cy = self.intrinsics["cy"]
        ray_camera = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=float)
        base2ee = get_ee_matrix(self.robot)
        base2cam = base2ee @ self.gripper2cam
        origin_base = base2cam[:3, 3]
        ray_base = base2cam[:3, :3] @ ray_camera
        if abs(float(ray_base[2])) < 1e-6:
            return None
        scale = (float(base_z) - float(origin_base[2])) / float(ray_base[2])
        if scale <= 0.0:
            return None
        base_xyz = origin_base + scale * ray_base
        base_xyz[2] = float(base_z)
        return base_xyz, int(round(u)), int(round(v))

    def side_direction_for_cup(self, cup_base_xyz):
        direction = self.side_grasp_direction
        if self.side_grasp_axis == "y" and self.side_auto_direction_by_cup_y:
            direction = -1.0 if float(cup_base_xyz[1]) >= self.side_prepose_split_y else 1.0

        if self.side_grasp_axis == "y":
            cup_y = float(cup_base_xyz[1])
            candidates = [direction, -direction]
            stage_offset = (
                self.side_staging_offset
                if self.side_far_stage_enabled
                else self.side_approach_offset + self.side_short_stage_backoff_m
            )

            def y_violation(candidate_direction):
                stage_y = cup_y + candidate_direction * stage_offset
                return max(
                    self.side_stage_y_min - stage_y,
                    0.0,
                    stage_y - self.side_stage_y_max,
                )

            best_direction = min(candidates, key=y_violation)
            if best_direction != direction and y_violation(best_direction) < y_violation(direction):
                old_stage_y = cup_y + direction * stage_offset
                new_stage_y = cup_y + best_direction * stage_offset
                self.get_logger().warning(
                    "side staging Y would leave reachable workspace; "
                    f"flipping side direction {direction:.0f}->{best_direction:.0f} "
                    f"(stage_y {old_stage_y:.3f}->{new_stage_y:.3f}, "
                    f"limit=[{self.side_stage_y_min:.3f}, {self.side_stage_y_max:.3f}])"
                )
                return best_direction
        return direction

    def side_direction_candidates(self, cup_base_xyz):
        first = self.side_direction_for_cup(cup_base_xyz)
        second = -first
        return [first, second]

    def side_unit_vector(self, cup_base_xyz=None, direction=None):
        if direction is None:
            direction = (
                self.side_direction_for_cup(cup_base_xyz)
                if cup_base_xyz is not None
                else self.side_grasp_direction
            )
        if self.side_grasp_axis == "x":
            return np.array([direction, 0.0], dtype=float)
        return np.array([0.0, direction], dtype=float)

    def side_grasp_orientation(self, side_vec):
        if self.side_orientation_mode == "home":
            return self.home_ori

        if self.side_orientation_mode == "euler":
            return quat_dict_from_euler(
                self.side_roll_deg,
                self.side_pitch_deg,
                self.side_yaw_deg,
            )

        # Make the tool's local +Z direction point horizontally into the cup.
        # The local +Y axis is kept close to world +Z so the wrist is laid over
        # the table instead of keeping the top-down grasp posture.
        tool_z = np.array([-side_vec[0], -side_vec[1], 0.0], dtype=float)
        tool_z_norm = np.linalg.norm(tool_z)
        if tool_z_norm < 1e-6:
            return self.home_ori
        tool_z /= tool_z_norm

        world_up = np.array([0.0, 0.0, 1.0], dtype=float)
        tool_x = np.cross(world_up, tool_z)
        tool_x_norm = np.linalg.norm(tool_x)
        if tool_x_norm < 1e-6:
            return self.home_ori
        tool_x /= tool_x_norm
        tool_y = np.cross(tool_z, tool_x)
        tool_y /= np.linalg.norm(tool_y)

        base_from_tool = np.column_stack((tool_x, tool_y, tool_z))
        if abs(self.side_tool_roll_deg) > 1e-6:
            base_from_tool = (
                base_from_tool
                @ Rotation.from_euler(
                    "z",
                    self.side_tool_roll_deg,
                    degrees=True,
                ).as_matrix()
            )
        return quat_dict_from_matrix(base_from_tool)

    def side_plan_score(self, plan: SideGraspPlan, current_xyz):
        stage_goal = np.array(
            [plan.stage_xy[0], plan.stage_xy[1], plan.lift_z],
            dtype=float,
        )
        distance_score = float(np.linalg.norm(stage_goal - current_xyz))
        if self.side_grasp_axis != "y":
            return distance_score

        y_values = [
            float(plan.stage_xy[1]),
            float(plan.pre_xy[1]),
            float(plan.guarded_grasp_xy[1]),
        ]
        y_violation = sum(
            max(self.side_stage_y_min - y_value, 0.0, y_value - self.side_stage_y_max)
            for y_value in y_values
        )
        return distance_score + 10.0 * y_violation

    def compute_side_grasp_plan(self, cup_base_xyz, side_direction=None) -> SideGraspPlan:
        cup_xyz = np.array([float(v) for v in cup_base_xyz], dtype=float)
        if side_direction is None:
            side_direction = self.side_direction_for_cup(cup_xyz)
        side_direction = 1.0 if float(side_direction) >= 0.0 else -1.0
        side_vec = self.side_unit_vector(cup_xyz, side_direction)
        side_ori = self.side_grasp_orientation(side_vec)
        cup_xy = cup_xyz[:2]
        stage_offset = (
            self.side_staging_offset
            if self.side_far_stage_enabled
            else self.side_approach_offset + self.side_short_stage_backoff_m
        )
        stage_xy = cup_xy + side_vec * stage_offset
        pre_xy = cup_xy + side_vec * self.side_approach_offset
        grasp_xy = cup_xy + side_vec * self.side_grasp_offset
        close_backoff_m = self.side_grasp_stop_backoff_m + self.side_close_underreach_m
        guarded_grasp_xy = grasp_xy + side_vec * close_backoff_m
        if self.side_fixed_grasp_z_enabled:
            grasp_z = max(self.side_fixed_grasp_z, self.min_motion_z)
        else:
            grasp_z = max(float(cup_xyz[2]) + self.side_grasp_z_offset, self.min_motion_z)
        pre_z = grasp_z
        lift_z = max(grasp_z + self.approach_offset, self.safe_z)
        place_z = max(grasp_z, self.min_motion_z)
        place_approach_z = max(place_z + self.approach_offset, self.safe_z)
        return SideGraspPlan(
            cup_xyz=cup_xyz,
            side_vec=side_vec,
            orientation=side_ori,
            stage_xy=stage_xy,
            pre_xy=pre_xy,
            grasp_xy=grasp_xy,
            guarded_grasp_xy=guarded_grasp_xy,
            grasp_z=grasp_z,
            pre_z=pre_z,
            lift_z=lift_z,
            place_z=place_z,
            place_approach_z=place_approach_z,
            side_direction=side_direction,
            close_backoff_m=close_backoff_m,
        )

    def build_side_grasp_candidates(self, cup_base_xyz):
        current_xyz = get_ee_matrix(self.robot)[:3, 3].copy()
        candidates = []
        for direction in self.side_direction_candidates(cup_base_xyz):
            plan = self.compute_side_grasp_plan(cup_base_xyz, direction)
            plan.score = self.side_plan_score(plan, current_xyz)
            candidates.append(plan)
        return sorted(candidates, key=lambda candidate: candidate.score)

    def log_side_grasp_plan(self, plan: SideGraspPlan, prefix="Side grasp target"):
        bx, by, bz = [float(v) for v in plan.cup_xyz]
        self.get_logger().info(
            f"{prefix} base=({bx:.3f}, {by:.3f}, {bz:.3f}), "
            f"axis={self.side_grasp_axis}, dir={plan.side_direction:.0f}, "
            f"ori_mode={self.side_orientation_mode}, "
            f"tool_roll={self.side_tool_roll_deg:.1f}deg, "
            f"stage=({plan.stage_xy[0]:.3f}, {plan.stage_xy[1]:.3f}, {plan.pre_z:.3f}), "
            f"pre=({plan.pre_xy[0]:.3f}, {plan.pre_xy[1]:.3f}, {plan.pre_z:.3f}), "
            f"grasp=({plan.grasp_xy[0]:.3f}, {plan.grasp_xy[1]:.3f}, {plan.grasp_z:.3f}), "
            f"guarded=({plan.guarded_grasp_xy[0]:.3f}, {plan.guarded_grasp_xy[1]:.3f}, "
            f"{plan.grasp_z:.3f}), close_backoff={plan.close_backoff_m:.3f}m "
            f"(stop={self.side_grasp_stop_backoff_m:.3f}+underreach={self.side_close_underreach_m:.3f}), "
            f"place_z={plan.place_z:.3f}, "
            f"linear_final={self.side_linear_approach_enabled}, "
            f"final_slide={self.side_final_slide_enabled}, "
            f"pre_pick_joint1_clearance={self.pre_pick_joint1_clearance_deg:.1f}deg"
        )

    def side_final_approach_params(self):
        return self.pilz_lin_params if self.side_linear_approach_enabled else self.pilz_params

    def spin_for_camera_update(self, duration_sec):
        end_time = time.time() + max(0.0, duration_sec)
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def select_redetect_target(self, detections):
        if self.color_image is None:
            return None

        candidates = [
            det for det in detections if det["class_name"] == self.target_class
        ]
        if not candidates:
            return None

        h, w = self.color_image.shape[:2]
        image_cx = w / 2.0
        image_cy = h / 2.0
        return min(
            candidates,
            key=lambda det: (det["cx"] - image_cx) ** 2
            + (det["cy"] - image_cy) ** 2,
        )

    def base_from_detection(self, detection, log_prefix):
        if (
            self.grasp_mode == "side"
            and self.side_fixed_grasp_z_enabled
            and self.side_project_bbox_center_to_fixed_z
        ):
            projected = self.bbox_center_to_fixed_base_z(
                detection["bbox"],
                self.side_fixed_grasp_z,
            )
            if projected is not None:
                base_xyz, u, v = projected
                self.get_logger().info(
                    f"{log_prefix} fixed-z side target from bbox center "
                    f"pixel=({u}, {v}), base_z={self.side_fixed_grasp_z:.3f} m "
                    f"-> base=({base_xyz[0]:.3f}, {base_xyz[1]:.3f}, "
                    f"{base_xyz[2]:.3f})"
                )
                return base_xyz
            self.get_logger().warning(
                f"{log_prefix} fixed-z bbox-center projection failed; "
                "falling back to depth-based target"
            )

        depth_info = self.depth_from_bbox(detection["bbox"], log_reason=True)
        if depth_info is None:
            return None

        u, v, z_m = depth_info
        camera_xyz = self.pixel_to_camera(u, v, z_m)
        base_xyz = self.camera_to_base(camera_xyz)
        self.get_logger().info(
            f"{log_prefix} pixel=({u}, {v}), depth={z_m:.3f} m, "
            f"camera=({camera_xyz[0]:.3f}, {camera_xyz[1]:.3f}, "
            f"{camera_xyz[2]:.3f}) -> base=({base_xyz[0]:.3f}, "
            f"{base_xyz[1]:.3f}, {base_xyz[2]:.3f})"
        )
        return base_xyz

    def pick_and_place(self, base_xyz):
        if self.grasp_mode == "side":
            task_ok = self.pick_and_place_side(base_xyz)
            if task_ok:
                return True
        else:
            task_ok = self.pick_and_place_top(base_xyz)

        if task_ok and self.return_home_after_task:
            self.get_logger().info("return home after task")
            return self.move_home()
        return task_ok

    def refine_target_from_current_view(self, log):
        if not self.redetect_on_approach:
            return None

        log.info("redetect target after approach")
        self.spin_for_camera_update(self.redetect_settle_sec)
        if self.color_image is None:
            return None

        detections = self.detect_objects(self.color_image.copy())
        target = self.select_redetect_target(detections)
        if target is None:
            log.warning("redetect target not found; using initial target")
            return None
        return self.base_from_detection(target, "[redetect]")

    def execute_side_grasp_plan(self, plan: SideGraspPlan):
        log = self.get_logger()
        side_close_xy = (
            plan.pre_xy if self.side_final_slide_enabled else plan.guarded_grasp_xy
        )
        side_close_label = (
            "move horizontally to side pre-grasp"
            if self.side_final_slide_enabled
            else "move horizontally to guarded side-grasp close pose"
        )
        side_close_params = (
            self.pilz_params
            if self.side_final_slide_enabled
            else self.side_final_approach_params()
        )
        log.info("move to outside side-staging pose")
        if not self.plan_and_execute(
            pose_goal=make_pose(plan.stage_xy[0], plan.stage_xy[1], plan.lift_z, plan.orientation),
            params=self.ompl_params,
        ):
            return False

        log.info("open gripper at outside high side-staging pose")
        if not self.open_gripper_max(wait=True):
            return False
        if self.gripper_open_settle_sec > 0.0:
            log.info(
                f"wait {self.gripper_open_settle_sec:.2f}s for RG2 full-open before low approach"
            )
            time.sleep(self.gripper_open_settle_sec)

        active_pre_z = None
        for attempt in range(self.side_low_retry_attempts + 1):
            try_pre_z = plan.pre_z + attempt * self.side_low_retry_lift_m
            if attempt == 0:
                log.info(
                    "lower at outside side-staging pose with OMPL "
                    f"(z={try_pre_z:.3f})"
                )
            else:
                log.warning(
                    "retry lower at outside side-staging pose above table "
                    f"(z={try_pre_z:.3f}, retry={attempt}/{self.side_low_retry_attempts})"
                )
            if self.plan_and_execute(
                pose_goal=make_pose(
                    plan.stage_xy[0],
                    plan.stage_xy[1],
                    try_pre_z,
                    plan.orientation,
                ),
                params=self.ompl_params,
            ):
                active_pre_z = try_pre_z
                break
        if active_pre_z is None:
            return False

        log.info(f"{side_close_label} (z={active_pre_z:.3f})")
        if not self.plan_and_execute(
            pose_goal=make_pose(
                side_close_xy[0],
                side_close_xy[1],
                active_pre_z,
                plan.orientation,
            ),
            params=side_close_params,
        ):
            return False

        if not self.wait_until_gripper_idle():
            return False

        if self.redetect_on_approach:
            log.warning(
                "low side-grip redetect/reposition is skipped because moving again "
                "near the cup can push it; use center_check_enabled for high-pose "
                "re-detection instead."
            )

        log.info("no extra center/slide motion; closing at guarded side-grasp close pose")

        if self.side_move_to_initial_center_before_close:
            log.warning(
                "side_move_to_initial_center_before_close is disabled at runtime "
                "because moving to the cup center before close can push the cup."
            )

        log.info("close gripper for guarded side grasp")
        self.gripper.move_gripper(GRIPPER_CLOSE_WIDTH, GRIPPER_FORCE)
        time.sleep(1.0)
        log.info("side grasp complete; holding cup for downstream rule-based task")
        return True

    def pick_and_place_side(self, base_xyz):
        log = self.get_logger()
        initial_base = np.array([float(v) for v in base_xyz], dtype=float)

        refined_base = self.center_check_redetect(initial_base)
        cup_base = initial_base if refined_base is None else np.array(refined_base, dtype=float)
        candidates = self.build_side_grasp_candidates(cup_base)
        if not candidates:
            log.error("No side-grasp candidates generated")
            return False

        for idx, candidate in enumerate(candidates, start=1):
            log.info(
                f"Side candidate {idx}: dir={candidate.side_direction:.0f}, "
                f"score={candidate.score:.3f}, "
                f"ready=({candidate.stage_xy[0]:.3f}, {candidate.stage_xy[1]:.3f}, {candidate.lift_z:.3f}), "
                f"close=({candidate.guarded_grasp_xy[0]:.3f}, {candidate.guarded_grasp_xy[1]:.3f}, {candidate.pre_z:.3f})"
            )

        if not self.move_to_side_prepose_if_configured(cup_base):
            return False
        if not self.move_joint1_clearance_before_side_grip():
            return False

        for candidate in candidates:
            if self.side_candidate_plan_check_enabled:
                ready_pose = make_pose(
                    candidate.stage_xy[0],
                    candidate.stage_xy[1],
                    candidate.lift_z,
                    candidate.orientation,
                )
                if not self.can_plan_pose_goal(
                    ready_pose,
                    self.ompl_params,
                    label=f"side candidate dir={candidate.side_direction:.0f} ready",
                ):
                    continue

            self.log_side_grasp_plan(
                candidate,
                prefix=f"Selected side-grasp candidate dir={candidate.side_direction:.0f}",
            )
            if self.execute_side_grasp_plan(candidate):
                return True
            log.warning(
                f"Side candidate dir={candidate.side_direction:.0f} failed before gripper close; "
                "trying next candidate if available"
            )

        log.error("No feasible side-grasp candidate succeeded")
        return False

    def pick_and_place_top(self, base_xyz):
        log = self.get_logger()
        bx, by, bz = [float(v) for v in base_xyz]
        pick_z = bz + self.pick_z_offset
        approach_z = max(pick_z + self.approach_offset, self.safe_z)
        place_approach_z = max(self.place_z + self.approach_offset, self.safe_z)

        log.info(
            f"Cup base point=({bx:.3f}, {by:.3f}, {bz:.3f}), "
            f"pick_z={pick_z:.3f}"
        )

        self.open_gripper_max(wait=False)

        steps = [
            ("move above cup", make_pose(bx, by, approach_z, self.home_ori)),
        ]
        for label, pose in steps:
            log.info(label)
            if not self.plan_and_execute(pose_goal=pose, params=self.ompl_params):
                return False

        if not self.wait_until_gripper_idle():
            return False

        refined_base = self.refine_target_from_current_view(log)
        if refined_base is not None:
            bx, by, bz = [float(v) for v in refined_base]
            pick_z = bz + self.pick_z_offset
            approach_z = max(pick_z + self.approach_offset, self.safe_z)
            log.info(
                f"refined cup base=({bx:.3f}, {by:.3f}, {bz:.3f}), "
                f"pick_z={pick_z:.3f}"
            )
            if not self.plan_and_execute(
                pose_goal=make_pose(bx, by, approach_z, self.home_ori),
                params=self.ompl_params,
            ):
                return False

        log.info("move down to cup")
        if not self.plan_and_execute(
            pose_goal=make_pose(bx, by, pick_z, self.home_ori),
            params=self.pilz_params,
        ):
            return False

        log.info("close gripper")
        self.gripper.move_gripper(GRIPPER_CLOSE_WIDTH, GRIPPER_FORCE)
        time.sleep(1.0)

        move_steps = [
            ("lift cup", make_pose(bx, by, approach_z, self.home_ori), self.pilz_lin_params),
            (
                "move above syrup pump front",
                make_pose(self.place_x, self.place_y, place_approach_z, self.home_ori),
                self.ompl_params,
            ),
            (
                "place cup",
                make_pose(self.place_x, self.place_y, self.place_z, self.home_ori),
                self.pilz_params,
            ),
        ]
        for label, pose, params in move_steps:
            log.info(label)
            if not self.plan_and_execute(pose_goal=pose, params=params):
                return False

        log.info("open gripper")
        self.open_gripper_max(wait=True)
        time.sleep(1.0)

        log.info("retract")
        return self.plan_and_execute(
            pose_goal=make_pose(self.place_x, self.place_y, place_approach_z,
                                self.home_ori),
            params=self.pilz_params,
        )

    def start_pick_from_detection(self):
        log = self.get_logger()
        if self.picking:
            log.warning("Already picking")
            self.last_status = "already picking"
            return
        if self.color_image is None or self.depth_image is None or self.intrinsics is None:
            log.warning("Waiting for color/depth/camera_info")
            self.last_status = "waiting for color/depth/camera_info"
            return
        if self.last_detection is None:
            log.warning(f"No {self.target_class} detection available")
            self.last_status = f"no {self.target_class} detection"
            return

        self.last_status = "initializing robot motion"
        if not self.ensure_motion_stack_ready():
            self.last_status = "motion init failed"
            return

        self.last_status = f"pick requested: {self.target_class}"
        base_xyz = self.base_from_detection(self.last_detection, "[initial]")
        if base_xyz is None:
            log.error(f"No valid depth around {self.target_class} bbox")
            self.last_status = f"no valid depth for {self.target_class}"
            return

        self.picking = True
        self.last_status = "moving robot"
        task_ok = False
        try:
            task_ok = self.pick_and_place(base_xyz)
            if task_ok:
                self.has_picked_once = True
                self.last_pick_time = time.time()
                self.last_status = "pick finished"
            else:
                if self.auto_pick:
                    self.last_pick_time = time.time()
                    log.warning(
                        "auto_pick attempt failed; returning to camera home and retrying after interval"
                    )
                self.last_status = "pick failed"
        finally:
            if (
                self.return_to_camera_home_after_attempt
                and not (task_ok and self.grasp_mode == "side")
            ):
                log.info("return to camera home after pick attempt")
                if self.move_camera_home():
                    if task_ok:
                        self.last_status = "pick finished; returned camera home"
                    else:
                        self.last_status = "pick failed; returned camera home"
                else:
                    log.error("Failed to return to camera home after pick attempt")
                    self.last_status = "camera home return failed"
            self.picking = False

    def draw_detections(self, image, detections):
        for detection in detections:
            x1, y1, x2, y2 = detection["bbox"]
            conf = detection["conf"]
            class_name = detection.get("class_name", "")

            if class_name == self.target_class:
                color = (0, 255, 0)
                thickness = 2
            elif class_name == "lid":
                color = (255, 0, 0)
                thickness = 2
            else:
                color = (180, 180, 180)
                thickness = 1

            cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)

            label = f"{class_name} {conf:.2f}"
            if class_name == self.target_class:
                depth_info = self.depth_from_bbox(detection["bbox"])
                if depth_info is not None:
                    u, v, z_m = depth_info
                    label += f" {z_m:.2f}m"
                    cv2.circle(image, (u, v), 5, (0, 0, 255), -1)

            cv2.putText(
                image,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )
        self.draw_hud(image, detections)
        return image

    def draw_hud(self, image, detections):
        counts = Counter(det["class_name"] for det in detections)
        count_text = " ".join(
            f"{name}:{counts[name]}" for name in sorted(counts)
        ) or "none"
        mode = "AUTO" if self.auto_pick else "MANUAL"
        target_state = "ready" if self.last_detection is not None else "not found"
        picked_state = "picked" if self.has_picked_once else "waiting"

        lines = [
            (
                f"[{mode}] {self.grasp_mode} target={self.target_class} "
                f"conf>={self.conf:.2f} "
                "p:pick a:auto r:reset ESC:quit"
            ),
            f"detections: {count_text} | target: {target_state} | {picked_state}",
            f"status: {self.last_status}",
        ]
        color = (0, 255, 255) if self.auto_pick else (230, 230, 230)
        for idx, text in enumerate(lines):
            y = 26 + idx * 24
            cv2.putText(
                image,
                text,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                color,
                2,
                cv2.LINE_AA,
            )

    def run(self):
        log = self.get_logger()
        if self.skip_initial_home_move:
            log.info("Skip initial home move; opening preview before motion stack initialization")
        elif self.move_to_camera_home:
            if not self.ensure_motion_stack_ready():
                log.error("Motion stack initialization failed")
                return
            if self.move_joint_home_before_camera_home:
                log.info("Move JOINT HOME before high camera home")
                if not self.move_joint_home():
                    log.error("Joint home move failed")
                    return
            log.info("Move HIGH CAMERA HOME")
            if not self.move_camera_home():
                log.error("High camera home move failed")
                return
        else:
            if not self.ensure_motion_stack_ready():
                log.error("Motion stack initialization failed")
                return
            log.info("Move JOINT HOME")
            if not self.move_joint_home():
                log.error("Joint home move failed")
                return

        if self._motion_stack_ready:
            self.open_gripper_max(wait=True)

        window = "YOLO Cup Pick - p pick, a auto, r reset, esc quit"
        cv2.namedWindow(window)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.exit_after_pick and self.has_picked_once and not self.picking:
                log.info("exit_after_pick=true and one pick completed; closing side_grip node")
                break
            if self.color_image is None:
                continue

            frame = self.color_image.copy()
            detections = self.detect_objects(frame)
            frame = self.draw_detections(frame, detections)
            cv2.imshow(window, frame)

            now = time.time()
            can_auto_pick = (
                self.auto_pick
                and self.last_detection is not None
                and not self.has_picked_once
                and not self.picking
                and (now - self.last_pick_time) >= self.auto_pick_interval
            )
            if can_auto_pick:
                self.start_pick_from_detection()

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if key in (ord("p"), ord("P")):
                log.info("pick key pressed")
                self.start_pick_from_detection()
            elif key in (ord("a"), ord("A")):
                self.auto_pick = not self.auto_pick
                self.last_pick_time = time.time()
                self.last_status = f"auto_pick {'ON' if self.auto_pick else 'OFF'}"
                log.info(f"auto_pick {'ON' if self.auto_pick else 'OFF'}")
            elif key in (ord("r"), ord("R")):
                self.has_picked_once = False
                self.last_pick_time = 0.0
                self.last_status = "pick state reset"
                log.info("pick state reset")

        cv2.destroyAllWindows()

    def destroy_node(self):
        try:
            if self.gripper is not None:
                self.gripper.close_connection()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloCupPickNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
