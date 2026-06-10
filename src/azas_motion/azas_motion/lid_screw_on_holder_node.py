#!/usr/bin/env python3
"""ArUco-based lid pick and screw-on primitive for a cup in the holder."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from azas_interfaces.srv import SetGripper
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from dsr_msgs2.srv import GetCurrentPosj, MoveJoint, MoveWait
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


BASE_FRAME = "base_link"
EE_LINK = "link_6"
GROUP_NAME = "manipulator"
JOINT_ORDER = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
TRAJECTORY_ACTION_NAME = "/dsr01/dsr_moveit_controller/follow_joint_trajectory"
MOVE_MODE_ABSOLUTE = 0
SYNC = 0
BLENDING_SPEED_TYPE_DUPLICATE = 0
DOWN_MATRIX = np.array(
    [
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=float,
)
HARDWARE_CONFIRM_PHRASE = "ENABLE_REAL_ROBOT_MOTION"


@dataclass(frozen=True)
class ArucoLidPose:
    marker_id: int
    center_xyz: np.ndarray
    marker_yaw_rad: float
    marker_matrix_base: np.ndarray


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def service_name(prefix: str, name: str) -> str:
    clean_prefix = str(prefix).strip("/")
    clean_name = str(name).strip("/")
    if not clean_prefix:
        return f"/{clean_name}"
    return f"/{clean_prefix}/{clean_name}"


def load_yaml(path_value: str, label: str) -> dict:
    path = Path(path_value).expanduser()
    if not path.exists():
        raise FileNotFoundError(f"{label} YAML does not exist: {path}")
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{label} YAML is not a map: {path}")
    return data


def quat_dict_from_matrix(matrix: np.ndarray) -> dict[str, float]:
    m00, m01, m02 = [float(value) for value in matrix[0]]
    m10, m11, m12 = [float(value) for value in matrix[1]]
    m20, m21, m22 = [float(value) for value in matrix[2]]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (m21 - m12) / scale
        qy = (m02 - m20) / scale
        qz = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / scale
        qx = 0.25 * scale
        qy = (m01 + m10) / scale
        qz = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / scale
        qx = (m01 + m10) / scale
        qy = 0.25 * scale
        qz = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / scale
        qx = (m02 + m20) / scale
        qy = (m12 + m21) / scale
        qz = 0.25 * scale
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        raise ValueError("rotation matrix produced a zero quaternion")
    return {
        "x": float(qx / norm),
        "y": float(qy / norm),
        "z": float(qz / norm),
        "w": float(qw / norm),
    }


def z_rotation_matrix(yaw_rad: float) -> np.ndarray:
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def top_down_orientation_from_yaw(yaw_rad: float) -> dict[str, float]:
    return quat_dict_from_matrix(z_rotation_matrix(yaw_rad) @ DOWN_MATRIX)


def make_pose(x: float, y: float, z: float, ori: dict[str, float]) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = BASE_FRAME
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.x = float(ori["x"])
    pose.pose.orientation.y = float(ori["y"])
    pose.pose.orientation.z = float(ori["z"])
    pose.pose.orientation.w = float(ori["w"])
    return pose


def parse_workspace_bounds(config: dict) -> dict[str, float] | None:
    motion = config.get("motion", {})
    if not isinstance(motion, dict):
        return None
    bounds = motion.get("workspace_bounds_m")
    if not isinstance(bounds, dict):
        return None
    required = ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")
    if any(key not in bounds for key in required):
        return None
    parsed = {key: float(bounds[key]) for key in required}
    min_z = motion.get("min_z_m")
    if min_z is not None:
        parsed["z_min"] = max(parsed["z_min"], float(min_z))
    return parsed


def get_ee_matrix(moveit_robot) -> np.ndarray:
    monitor = moveit_robot.get_planning_scene_monitor()
    with monitor.read_only() as scene:
        transform = scene.current_state.get_global_link_transform(EE_LINK)
    return np.asarray(transform, dtype=float)


class LidScrewOnHolderNode(Node):
    def __init__(self) -> None:
        super().__init__("lid_screw_on_holder_node")

        self.declare_parameter("trigger_service", "/azas/lid_screw_on_holder/run")
        self.declare_parameter("status_topic", "/azas/lid_screw_on_holder/status")
        self.declare_parameter("execute_motion", False)
        self.declare_parameter("hardware_confirm", "")
        self.declare_parameter("shutdown_on_complete", False)
        self.declare_parameter("service_prefix", "/dsr01")
        self.declare_parameter("frame_id", BASE_FRAME)
        self.declare_parameter("ee_link", EE_LINK)
        self.declare_parameter("planning_group", GROUP_NAME)
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("cup_pose_topic", "/jarvis/tumbler_dispenser/tumbler_pose")
        self.declare_parameter("raw_joint_state_topic", "/dsr01/joint_states")
        self.declare_parameter(
            "calibration_config_path",
            "/home/ssu/Azas/src/azas_bringup/config/calibration.yaml",
        )
        self.declare_parameter(
            "safety_config_path",
            "/home/ssu/Azas/src/azas_bringup/config/safety.yaml",
        )
        self.declare_parameter(
            "hand_eye_matrix_path",
            "/home/ssu/Azas/src/azas_perception/config/T_gripper2camera.npy",
        )
        self.declare_parameter("safety_workspace_enforced", True)
        self.declare_parameter("move_to_observe_before_detect", True)
        self.declare_parameter("observe_motion_backend", "move_joint")
        self.declare_parameter("observe_joint_1_deg", 3.0)
        self.declare_parameter("observe_joint_2_deg", -12.7)
        self.declare_parameter("observe_joint_3_deg", 44.0)
        self.declare_parameter("observe_joint_4_deg", -9.0)
        self.declare_parameter("observe_joint_5_deg", 133.0)
        self.declare_parameter("observe_joint_6_deg", 90.0)
        self.declare_parameter("observe_joint_velocity_deg_s", 18.0)
        self.declare_parameter("observe_joint_acceleration_deg_s", 22.0)
        self.declare_parameter("observe_joint_time_sec", 0.0)
        self.declare_parameter("observe_settle_sec", 0.5)
        self.declare_parameter("lid_aruco_dictionary", "DICT_6X6_250")
        self.declare_parameter("lid_aruco_marker_id", -1)
        self.declare_parameter("lid_aruco_marker_length_m", 0.0)
        self.declare_parameter("lid_aruco_max_tilt_deg", 40.0)
        self.declare_parameter("lid_aruco_detect_timeout_sec", 2.0)
        self.declare_parameter("lid_aruco_detect_poll_sec", 0.10)
        self.declare_parameter("lid_pick_tcp_z_offset_m", 0.0)
        self.declare_parameter("lid_pick_approach_lift_m", 0.10)
        self.declare_parameter("lid_pick_yaw_offset_deg", 0.0)
        self.declare_parameter("lid_holder_tcp_z_m", 0.0)
        self.declare_parameter("lid_holder_x_offset_m", 0.0)
        self.declare_parameter("lid_holder_y_offset_m", 0.0)
        self.declare_parameter("lid_holder_approach_lift_m", 0.10)
        self.declare_parameter("lid_holder_yaw_offset_deg", 0.0)
        self.declare_parameter("require_cup_pose_at_holder", True)
        self.declare_parameter("cup_pose_max_age_sec", 2.0)
        self.declare_parameter("cup_pose_wait_timeout_sec", 6.0)
        self.declare_parameter("cup_pose_max_xy_error_m", 0.060)
        self.declare_parameter("aruco_redetect_settle_sec", 0.50)
        self.declare_parameter("regrip_max_xy_error_m", 0.040)
        self.declare_parameter("screw_cycles", 2)
        self.declare_parameter("screw_turn_deg", 180.0)
        self.declare_parameter("screw_turn_direction", 1.0)
        self.declare_parameter("screw_motion_backend", "move_joint")
        self.declare_parameter("screw_joint_velocity_scale", 0.04)
        self.declare_parameter("screw_joint_acceleration_scale", 0.03)
        self.declare_parameter("screw_move_joint_velocity_deg_s", 18.0)
        self.declare_parameter("screw_move_joint_acceleration_deg_s", 22.0)
        self.declare_parameter("screw_move_joint_time_sec", 0.0)
        self.declare_parameter("pose_velocity_scale", 0.10)
        self.declare_parameter("pose_acceleration_scale", 0.08)
        self.declare_parameter("lin_velocity_scale", 0.06)
        self.declare_parameter("lin_acceleration_scale", 0.04)
        self.declare_parameter("planning_time_sec", 4.0)
        self.declare_parameter("verify_motion", True)
        self.declare_parameter("pose_goal_tolerance_m", 0.015)
        self.declare_parameter("joint_goal_tolerance_rad", 0.03)
        self.declare_parameter("trajectory_action_name", TRAJECTORY_ACTION_NAME)
        self.declare_parameter("trajectory_action_wait_sec", 20.0)
        self.declare_parameter("trajectory_execution_timeout_sec", 90.0)
        self.declare_parameter("max_single_segment_joint_motion_deg", 180.0)
        self.declare_parameter("move_joint_service_timeout_sec", 60.0)
        self.declare_parameter("move_joint_wait_timeout_sec", 60.0)
        self.declare_parameter("move_joint_verify_tolerance_deg", 2.0)
        self.declare_parameter("joint6_raw_min_deg", -360.0)
        self.declare_parameter("joint6_raw_max_deg", 360.0)
        self.declare_parameter("gripper_set_service", "/jarvis/rg2/set_width")
        self.declare_parameter("gripper_service_timeout_sec", 5.0)
        self.declare_parameter("lid_gripper_close_width_m", 0.012)
        self.declare_parameter("lid_gripper_release_width_m", 0.080)
        self.declare_parameter("lid_gripper_force_n", 40.0)
        self.declare_parameter("lid_grip_settle_sec", 0.8)
        self.declare_parameter("lid_release_settle_sec", 0.4)
        self.declare_parameter("return_wrist_after_final", True)
        self.declare_parameter("retreat_after_sequence", True)

        self.execute_motion = parse_bool(self.get_parameter("execute_motion").value)
        self.hardware_confirm = str(self.get_parameter("hardware_confirm").value)
        self.hardware_armed = (
            self.execute_motion and self.hardware_confirm == HARDWARE_CONFIRM_PHRASE
        )
        if self.execute_motion and not self.hardware_armed:
            raise ValueError(
                "execute_motion=true requires "
                f"hardware_confirm:={HARDWARE_CONFIRM_PHRASE}"
            )

        self.frame_id = str(self.get_parameter("frame_id").value)
        if self.frame_id != BASE_FRAME:
            raise ValueError("lid_screw_on_holder_node currently requires frame_id=base_link")
        self.ee_link = str(self.get_parameter("ee_link").value)
        self.planning_group = str(self.get_parameter("planning_group").value)
        if self.ee_link != EE_LINK or self.planning_group != GROUP_NAME:
            self.get_logger().warning(
                f"Using local constants ee_link={EE_LINK}, group={GROUP_NAME}; "
                f"received ee_link={self.ee_link}, group={self.planning_group}"
            )

        self.bridge = CvBridge()
        self.color_image = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_joint_positions: dict[str, float] = {}
        self.latest_raw_joint_positions: dict[str, float] = {}
        self.last_cup_pose = None
        self.last_cup_pose_received_time = None
        self.last_lid_pose: ArucoLidPose | None = None
        self.running_lock = threading.Lock()
        self.moveit_lock = threading.Lock()

        self.workspace_bounds = self.load_workspace_bounds()
        self.holder_target_xyz, self.holder_center_xy = self.load_holder_target()
        self.validate_configuration()
        self.gripper2cam = self.load_hand_eye_matrix()

        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.on_camera_info,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("color_topic").value),
            self.on_color_image,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("cup_pose_topic").value),
            self.on_cup_pose,
            10,
        )
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(
            JointState,
            str(self.get_parameter("raw_joint_state_topic").value),
            self.on_raw_joint_state,
            10,
        )
        self.trajectory_action = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("trajectory_action_name").value),
        )
        service_prefix = str(self.get_parameter("service_prefix").value)
        self.move_joint = self.create_client(
            MoveJoint,
            service_name(service_prefix, "motion/move_joint"),
        )
        self.move_wait = self.create_client(
            MoveWait,
            service_name(service_prefix, "motion/move_wait"),
        )
        self.current_posj = self.create_client(
            GetCurrentPosj,
            service_name(service_prefix, "aux_control/get_current_posj"),
        )
        self.gripper_set = self.create_client(
            SetGripper,
            str(self.get_parameter("gripper_set_service").value),
        )

        self.robot = None
        self.arm = None
        self.robot_model = None
        self.ptp_params = None
        self.lin_params = None
        self.screw_params = None

        self.create_service(
            Trigger,
            str(self.get_parameter("trigger_service").value),
            self.on_trigger,
        )
        self.publish_status(
            "READY",
            f"service={self.get_parameter('trigger_service').value} "
            f"execute_motion={self.execute_motion}",
        )

    def ensure_moveit_initialized(self) -> bool:
        if self.robot is not None:
            return True
        with self.moveit_lock:
            if self.robot is not None:
                return True
            try:
                self.publish_status("MOVEIT_INIT", "initializing MoveItPy")
                robot = MoveItPy(node_name="lid_screw_on_holder_moveit_py")
                arm = robot.get_planning_component(GROUP_NAME)
                robot_model = robot.get_robot_model()
                self.robot = robot
                self.arm = arm
                self.robot_model = robot_model
                self.ptp_params = self.make_plan_params("PTP", "pose")
                self.lin_params = self.make_plan_params("LIN", "lin")
                self.screw_params = self.make_plan_params("PTP", "screw")
                self.publish_status("MOVEIT_READY")
                if self.hardware_armed:
                    self.wait_for_trajectory_action_server("MoveIt trajectory controller")
                return True
            except Exception as exc:
                self.robot = None
                self.arm = None
                self.robot_model = None
                self.ptp_params = None
                self.lin_params = None
                self.screw_params = None
                self.get_logger().error(f"MoveItPy initialization failed: {exc}")
                return False

    def make_plan_params(self, planner_id: str, mode: str) -> PlanRequestParameters:
        params = PlanRequestParameters(self.robot)
        params.planning_pipeline = "pilz_industrial_motion_planner"
        params.planner_id = planner_id
        params.planning_time = float(self.get_parameter("planning_time_sec").value)
        if mode == "screw":
            params.max_velocity_scaling_factor = float(
                self.get_parameter("screw_joint_velocity_scale").value
            )
            params.max_acceleration_scaling_factor = float(
                self.get_parameter("screw_joint_acceleration_scale").value
            )
        elif mode == "lin":
            params.max_velocity_scaling_factor = float(
                self.get_parameter("lin_velocity_scale").value
            )
            params.max_acceleration_scaling_factor = float(
                self.get_parameter("lin_acceleration_scale").value
            )
        else:
            params.max_velocity_scaling_factor = float(
                self.get_parameter("pose_velocity_scale").value
            )
            params.max_acceleration_scaling_factor = float(
                self.get_parameter("pose_acceleration_scale").value
            )
        return params

    def publish_status(self, state: str, detail: str = "") -> None:
        msg = String()
        msg.data = state if not detail else f"{state}: {detail}"
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def load_workspace_bounds(self) -> dict[str, float] | None:
        if not parse_bool(self.get_parameter("safety_workspace_enforced").value):
            self.get_logger().warning("safety_workspace_enforced=false")
            return None
        safety = load_yaml(str(self.get_parameter("safety_config_path").value), "safety")
        bounds = parse_workspace_bounds(safety)
        if bounds is None:
            raise ValueError("safety.yaml motion.workspace_bounds_m is missing/invalid")
        self.get_logger().info(
            "Loaded workspace bounds "
            f"x=[{bounds['x_min']:.3f},{bounds['x_max']:.3f}] "
            f"y=[{bounds['y_min']:.3f},{bounds['y_max']:.3f}] "
            f"z=[{bounds['z_min']:.3f},{bounds['z_max']:.3f}]"
        )
        return bounds

    def load_holder_target(self) -> tuple[np.ndarray, np.ndarray]:
        calibration = load_yaml(
            str(self.get_parameter("calibration_config_path").value),
            "calibration",
        )
        holder = calibration.get("cup_holder", {})
        if not isinstance(holder, dict):
            raise ValueError("calibration.yaml cup_holder section is missing/invalid")
        center = holder.get("top_center_estimated_xyz_m") or holder.get(
            "bottom_insert_center_pose_xyz_m"
        )
        if not isinstance(center, list) or len(center) < 2 or any(value is None for value in center[:2]):
            raise ValueError("cup_holder center X/Y is missing or null")
        center_xy = np.array([float(center[0]), float(center[1])], dtype=float)

        lid_screw = holder.get("lid_screw", {})
        if isinstance(lid_screw, dict):
            measured = lid_screw.get("tcp_pose_xyz_m")
            if isinstance(measured, list) and len(measured) >= 3:
                if any(value is None for value in measured[:3]):
                    raise ValueError("cup_holder.lid_screw.tcp_pose_xyz_m contains null")
                return np.array([float(value) for value in measured[:3]], dtype=float), center_xy

        holder_z = float(self.get_parameter("lid_holder_tcp_z_m").value)
        if holder_z <= 0.0:
            raise ValueError(
                "lid_holder_tcp_z_m must be measured and > 0.0, or add "
                "cup_holder.lid_screw.tcp_pose_xyz_m to calibration.yaml"
            )
        target = np.array(
            [
                center_xy[0] + float(self.get_parameter("lid_holder_x_offset_m").value),
                center_xy[1] + float(self.get_parameter("lid_holder_y_offset_m").value),
                holder_z,
            ],
            dtype=float,
        )
        return target, center_xy

    def validate_configuration(self) -> None:
        if getattr(cv2, "aruco", None) is None:
            raise ValueError("OpenCV aruco module is not available")
        self.aruco_dictionary()
        if not hasattr(cv2, "solvePnP") or not hasattr(cv2, "Rodrigues"):
            raise ValueError("OpenCV solvePnP/Rodrigues pose APIs are not available")
        if float(self.get_parameter("lid_aruco_marker_length_m").value) <= 0.0:
            raise ValueError("lid_aruco_marker_length_m must be measured and > 0.0")
        if float(self.get_parameter("lid_pick_tcp_z_offset_m").value) <= 0.0:
            raise ValueError("lid_pick_tcp_z_offset_m must be measured and > 0.0")
        self.validate_workspace_point(self.holder_target_xyz, "holder lid screw target")

    def load_hand_eye_matrix(self) -> np.ndarray:
        path = Path(str(self.get_parameter("hand_eye_matrix_path").value)).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"hand-eye matrix does not exist: {path}")
        matrix = np.load(str(path)).astype(float)
        if matrix.shape != (4, 4):
            raise ValueError(f"hand-eye matrix must be 4x4: {path}")
        matrix[:3, 3] /= 1000.0
        self.get_logger().info(f"Loaded hand-eye matrix: {path}")
        return matrix

    def validate_workspace_point(self, xyz: np.ndarray, label: str) -> bool:
        if self.workspace_bounds is None:
            return True
        x, y, z = [float(value) for value in xyz]
        bounds = self.workspace_bounds
        violations = []
        if x < bounds["x_min"] or x > bounds["x_max"]:
            violations.append("x")
        if y < bounds["y_min"] or y > bounds["y_max"]:
            violations.append("y")
        if z < bounds["z_min"] or z > bounds["z_max"]:
            violations.append("z")
        if violations:
            raise ValueError(
                f"{label} outside safety workspace on {','.join(violations)}: "
                f"({x:.3f}, {y:.3f}, {z:.3f})"
            )
        return True

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(
            [
                [float(msg.k[0]), 0.0, float(msg.k[2])],
                [0.0, float(msg.k[4]), float(msg.k[5])],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        coeffs = np.asarray(msg.d, dtype=float).reshape(-1, 1)
        self.dist_coeffs = coeffs if coeffs.size else np.zeros((5, 1), dtype=float)

    def on_color_image(self, msg: Image) -> None:
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def on_cup_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != BASE_FRAME:
            self.get_logger().warning(
                f"Ignoring cup pose frame_id={msg.header.frame_id}; expected {BASE_FRAME}"
            )
            return
        self.last_cup_pose = msg
        self.last_cup_pose_received_time = self.get_clock().now()

    def on_joint_state(self, msg: JointState) -> None:
        if msg.name and len(msg.name) == len(msg.position):
            self.latest_joint_positions = dict(zip(msg.name, msg.position))

    def on_raw_joint_state(self, msg: JointState) -> None:
        if msg.name and len(msg.name) == len(msg.position):
            self.latest_raw_joint_positions = dict(zip(msg.name, msg.position))

    def aruco_dictionary(self):
        aruco = cv2.aruco
        dictionary_name = str(self.get_parameter("lid_aruco_dictionary").value)
        dictionary_id = getattr(aruco, dictionary_name, None)
        if dictionary_id is None:
            try:
                dictionary_id = int(dictionary_name)
            except ValueError as exc:
                raise ValueError(f"Unknown ArUco dictionary {dictionary_name!r}") from exc
        if hasattr(aruco, "getPredefinedDictionary"):
            return aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(aruco, "Dictionary_get"):
            return aruco.Dictionary_get(dictionary_id)
        raise ValueError("OpenCV aruco dictionary factory is unavailable")

    def detect_aruco_markers(self, image):
        aruco = cv2.aruco
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        dictionary = self.aruco_dictionary()
        if hasattr(aruco, "ArucoDetector"):
            detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            parameters = aruco.DetectorParameters_create()
            corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)
        if ids is None or len(ids) == 0:
            return [], []
        return corners, [int(value) for value in np.asarray(ids).reshape(-1)]

    def select_marker_index(self, corners, ids) -> int | None:
        marker_id = int(self.get_parameter("lid_aruco_marker_id").value)
        if marker_id >= 0:
            for index, found_id in enumerate(ids):
                if found_id == marker_id:
                    return index
            return None
        areas = [
            abs(float(cv2.contourArea(np.asarray(marker_corners).reshape(4, 2).astype(np.float32))))
            for marker_corners in corners
        ]
        return int(np.argmax(areas)) if areas else None

    def estimate_marker_pose_from_corners(self, corners, marker_length: float):
        half = float(marker_length) / 2.0
        object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )
        image_points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        if not success:
            raise ValueError("ArUco marker solvePnP failed")
        return np.asarray(rvec, dtype=float).reshape(3), np.asarray(tvec, dtype=float).reshape(3)

    def wait_for_camera_inputs(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(timeout_sec, 0.1)
        while rclpy.ok() and time.monotonic() < deadline:
            if self.color_image is not None and self.camera_matrix is not None:
                return True
            time.sleep(0.05)
        return False

    def wait_for_trajectory_action_server(self, label: str) -> bool:
        if not self.hardware_armed:
            return True
        timeout = float(self.get_parameter("trajectory_action_wait_sec").value)
        action_name = str(self.get_parameter("trajectory_action_name").value)
        if self.trajectory_action.wait_for_server(timeout_sec=timeout):
            return True
        self.get_logger().error(
            f"{label}: trajectory action server unavailable after {timeout:.1f}s: {action_name}"
        )
        return False

    def wait_for_doosan_joint_services(self, label: str) -> bool:
        if not self.hardware_armed:
            return True
        timeout = max(float(self.get_parameter("move_joint_service_timeout_sec").value), 0.1)
        services = [
            (self.move_joint, "motion/move_joint"),
            (self.move_wait, "motion/move_wait"),
            (self.current_posj, "aux_control/get_current_posj"),
        ]
        for client, name in services:
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f"{label}: {name} unavailable after {timeout:.1f}s")
                return False
        return True

    def read_current_posj_deg(self, label: str) -> list[float] | None:
        if self.current_posj is not None and self.current_posj.service_is_ready():
            future = self.current_posj.call_async(GetCurrentPosj.Request())
            timeout = max(float(self.get_parameter("move_joint_service_timeout_sec").value), 0.1)
            result = self.wait_for_future(future, f"{label}: get_current_posj", timeout)
            if result is not None and getattr(result, "success", False):
                joints = [float(value) for value in result.conv_posj]
                if len(joints) == 6:
                    return joints
                self.get_logger().error(
                    f"{label}: get_current_posj returned {len(joints)} joints, expected 6"
                )
                return None

        if any(name not in self.latest_raw_joint_positions for name in JOINT_ORDER):
            self.get_logger().error(f"{label}: raw joint state is not available")
            return None
        return [
            math.degrees(float(self.latest_raw_joint_positions[name]))
            for name in JOINT_ORDER
        ]

    def validate_move_joint_target_deg(self, joints_deg: list[float], label: str) -> bool:
        if len(joints_deg) != 6:
            self.get_logger().error(f"{label}: expected 6 joints, got {len(joints_deg)}")
            return False
        joint6 = float(joints_deg[5])
        joint6_min = float(self.get_parameter("joint6_raw_min_deg").value)
        joint6_max = float(self.get_parameter("joint6_raw_max_deg").value)
        if joint6 < joint6_min or joint6 > joint6_max:
            self.get_logger().error(
                f"{label}: refusing joint_6 target {joint6:.1f}deg outside "
                f"[{joint6_min:.1f}, {joint6_max:.1f}]deg"
            )
            return False
        return True

    def call_move_joint_deg(
        self,
        joints_deg: list[float],
        label: str,
        velocity_deg_s: float,
        acceleration_deg_s2: float,
        move_time_sec: float,
    ) -> bool:
        if not self.hardware_armed:
            self.publish_status(
                "MOVE_JOINT_PLAN_ONLY",
                f"{label}: joints_deg={[round(value, 1) for value in joints_deg]}",
            )
            return True
        if not self.validate_move_joint_target_deg(joints_deg, label):
            return False
        if not self.wait_for_doosan_joint_services(label):
            return False

        req = MoveJoint.Request()
        req.pos = [float(value) for value in joints_deg]
        req.vel = max(float(velocity_deg_s), 0.1)
        req.acc = max(float(acceleration_deg_s2), 0.1)
        req.time = max(float(move_time_sec), 0.0)
        req.radius = 0.0
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC

        self.publish_status(
            "MOVE_JOINT",
            f"{label}: joints_deg={[round(value, 1) for value in req.pos]} "
            f"vel={req.vel:.1f} acc={req.acc:.1f} time={req.time:.2f}",
        )
        timeout = max(float(self.get_parameter("move_joint_service_timeout_sec").value), 0.1)
        result = self.wait_for_future(self.move_joint.call_async(req), label, timeout)
        if result is None or not getattr(result, "success", False):
            self.get_logger().error(f"{label}: MoveJoint returned success=false")
            return False

        wait_timeout = max(float(self.get_parameter("move_joint_wait_timeout_sec").value), 0.1)
        wait_result = self.wait_for_future(
            self.move_wait.call_async(MoveWait.Request()),
            f"{label}: move_wait",
            wait_timeout,
        )
        if wait_result is not None and hasattr(wait_result, "success") and not wait_result.success:
            self.get_logger().error(f"{label}: move_wait returned success=false")
            return False
        return self.verify_move_joint_deg(joints_deg, label)

    def verify_move_joint_deg(self, target_deg: list[float], label: str) -> bool:
        tolerance = max(float(self.get_parameter("move_joint_verify_tolerance_deg").value), 0.0)
        if tolerance <= 0.0:
            return True
        current = self.read_current_posj_deg(label)
        if current is None:
            return False
        errors = [
            abs((float(now) - float(target) + 180.0) % 360.0 - 180.0)
            for now, target in zip(current, target_deg)
        ]
        max_error = max(errors, default=0.0)
        if max_error > tolerance:
            self.get_logger().error(
                f"{label}: MoveJoint max joint error={max_error:.2f}deg "
                f"> tolerance={tolerance:.2f}deg; "
                f"current={[round(value, 2) for value in current]}"
            )
            return False
        self.publish_status("MOVE_JOINT_DONE", f"{label}: max_error={max_error:.2f}deg")
        return True

    def move_to_observe_if_configured(self) -> bool:
        if not parse_bool(self.get_parameter("move_to_observe_before_detect").value):
            return True
        joints = [
            math.radians(float(self.get_parameter(f"observe_joint_{index}_deg").value))
            for index in range(1, 7)
        ]
        self.publish_status("MOVE_OBSERVE", "moving to taught lid observe posture")
        backend = str(self.get_parameter("observe_motion_backend").value).strip().lower()
        if self.hardware_armed and backend in {"move_joint", "movej", "doosan"}:
            joints_deg = [
                float(self.get_parameter(f"observe_joint_{index}_deg").value)
                for index in range(1, 7)
            ]
            if not self.call_move_joint_deg(
                joints_deg,
                "lid observe posture",
                float(self.get_parameter("observe_joint_velocity_deg_s").value),
                float(self.get_parameter("observe_joint_acceleration_deg_s").value),
                float(self.get_parameter("observe_joint_time_sec").value),
            ):
                return False
            time.sleep(float(self.get_parameter("observe_settle_sec").value))
            return True
        if not self.move_joint_positions(joints, "lid observe posture", self.ptp_params):
            return False
        time.sleep(float(self.get_parameter("observe_settle_sec").value))
        return True

    def detect_lid_pose(self, label: str) -> ArucoLidPose | None:
        if self.color_image is None or self.camera_matrix is None:
            self.get_logger().warning(f"{label}: camera inputs not ready")
            return None
        timeout = max(0.0, float(self.get_parameter("lid_aruco_detect_timeout_sec").value))
        poll_sec = max(0.02, float(self.get_parameter("lid_aruco_detect_poll_sec").value))
        deadline = time.monotonic() + timeout
        corners = []
        ids = []
        marker_index = None
        while rclpy.ok():
            corners, ids = self.detect_aruco_markers(self.color_image.copy())
            marker_index = self.select_marker_index(corners, ids)
            if marker_index is not None:
                break
            if time.monotonic() >= deadline:
                break
            time.sleep(poll_sec)
        if marker_index is None:
            dictionary_name = str(self.get_parameter("lid_aruco_dictionary").value)
            marker_id = int(self.get_parameter("lid_aruco_marker_id").value)
            if ids:
                self.get_logger().warning(
                    f"{label}: detected ArUco ids={ids}, but expected lid_aruco_marker_id={marker_id}"
                )
            else:
                self.get_logger().warning(
                    f"{label}: no ArUco markers detected using dictionary={dictionary_name}"
                )
            return None

        marker_length = float(self.get_parameter("lid_aruco_marker_length_m").value)
        try:
            rvec, tvec = self.estimate_marker_pose_from_corners(
                corners[marker_index],
                marker_length,
            )
        except ValueError as exc:
            self.get_logger().warning(f"{label}: {exc}")
            return None
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        camera_from_marker = np.eye(4, dtype=float)
        camera_from_marker[:3, :3] = np.asarray(rotation_matrix, dtype=float)
        camera_from_marker[:3, 3] = tvec
        base_from_camera = get_ee_matrix(self.robot) @ self.gripper2cam
        base_from_marker = base_from_camera @ camera_from_marker

        normal = base_from_marker[:3, :3] @ np.array([0.0, 0.0, 1.0])
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(normal[2]))))))
        max_tilt = float(self.get_parameter("lid_aruco_max_tilt_deg").value)
        if tilt_deg > max_tilt:
            self.get_logger().warning(f"{label}: ArUco tilt {tilt_deg:.1f}deg > {max_tilt:.1f}deg")
            return None

        marker_x = base_from_marker[:3, :3] @ np.array([1.0, 0.0, 0.0])
        yaw_rad = math.atan2(float(marker_x[1]), float(marker_x[0]))
        pose = ArucoLidPose(
            marker_id=ids[marker_index],
            center_xyz=base_from_marker[:3, 3].copy(),
            marker_yaw_rad=yaw_rad,
            marker_matrix_base=base_from_marker,
        )
        self.last_lid_pose = pose
        self.publish_status(
            "LID_ARUCO",
            f"{label}: id={pose.marker_id} xyz=({pose.center_xyz[0]:.3f},"
            f"{pose.center_xyz[1]:.3f},{pose.center_xyz[2]:.3f}) "
            f"yaw={math.degrees(yaw_rad):.1f}deg tilt={tilt_deg:.1f}deg",
        )
        return pose

    def cup_pose_at_holder_status(self) -> tuple[bool, str]:
        if not parse_bool(self.get_parameter("require_cup_pose_at_holder").value):
            return True, "require_cup_pose_at_holder=false"
        if self.last_cup_pose is None or self.last_cup_pose_received_time is None:
            return False, "No fresh cup pose received at holder validation"
        age = (self.get_clock().now() - self.last_cup_pose_received_time).nanoseconds / 1e9
        max_age = float(self.get_parameter("cup_pose_max_age_sec").value)
        if age > max_age:
            return False, f"Cup pose is stale: age={age:.2f}s > {max_age:.2f}s"
        cup_xy = np.array(
            [
                float(self.last_cup_pose.pose.position.x),
                float(self.last_cup_pose.pose.position.y),
            ],
            dtype=float,
        )
        xy_error = float(np.linalg.norm(cup_xy - self.holder_center_xy))
        limit = float(self.get_parameter("cup_pose_max_xy_error_m").value)
        if xy_error > limit:
            return False, f"Cup pose is not at holder: xy_error={xy_error:.3f}m > {limit:.3f}m"
        return True, f"age={age:.2f}s xy_error={xy_error:.3f}m"

    def validate_cup_pose_at_holder(self) -> bool:
        ok, detail = self.cup_pose_at_holder_status()
        if ok:
            if "require_cup_pose_at_holder=false" in detail:
                self.get_logger().warning(detail)
            else:
                self.publish_status("CUP_AT_HOLDER", detail)
            return True
        self.get_logger().error(detail)
        return False

    def wait_for_cup_pose_at_holder(self) -> bool:
        if not parse_bool(self.get_parameter("require_cup_pose_at_holder").value):
            self.get_logger().warning("require_cup_pose_at_holder=false")
            return True
        timeout = max(0.0, float(self.get_parameter("cup_pose_wait_timeout_sec").value))
        self.publish_status("WAIT_CUP_POSE", f"timeout={timeout:.1f}s")
        deadline = time.monotonic() + timeout
        last_detail = "No fresh cup pose received at holder validation"
        while rclpy.ok():
            ok, detail = self.cup_pose_at_holder_status()
            if ok:
                self.publish_status("CUP_AT_HOLDER", detail)
                return True
            last_detail = detail
            if time.monotonic() >= deadline:
                break
            time.sleep(0.05)
        self.get_logger().error(f"{last_detail} after waiting {timeout:.1f}s")
        return False

    def pose_pair_for_lid_pick(self, lid_pose: ArucoLidPose, target_z: float | None = None):
        pick_z = (
            float(lid_pose.center_xyz[2])
            + float(self.get_parameter("lid_pick_tcp_z_offset_m").value)
            if target_z is None
            else target_z
        )
        yaw = lid_pose.marker_yaw_rad + math.radians(float(self.get_parameter("lid_pick_yaw_offset_deg").value))
        ori = top_down_orientation_from_yaw(yaw)
        approach_z = max(pick_z + float(self.get_parameter("lid_pick_approach_lift_m").value), pick_z)
        return (
            make_pose(float(lid_pose.center_xyz[0]), float(lid_pose.center_xyz[1]), approach_z, ori),
            make_pose(float(lid_pose.center_xyz[0]), float(lid_pose.center_xyz[1]), pick_z, ori),
        )

    def pose_pair_for_holder(self, lid_pose: ArucoLidPose):
        yaw = lid_pose.marker_yaw_rad + math.radians(float(self.get_parameter("lid_holder_yaw_offset_deg").value))
        ori = top_down_orientation_from_yaw(yaw)
        target_z = float(self.holder_target_xyz[2])
        approach_z = max(target_z + float(self.get_parameter("lid_holder_approach_lift_m").value), target_z)
        return (
            make_pose(float(self.holder_target_xyz[0]), float(self.holder_target_xyz[1]), approach_z, ori),
            make_pose(float(self.holder_target_xyz[0]), float(self.holder_target_xyz[1]), target_z, ori),
        )

    def current_joint_positions(self) -> list[float] | None:
        if any(name not in self.latest_joint_positions for name in JOINT_ORDER):
            return None
        return [float(self.latest_joint_positions[name]) for name in JOINT_ORDER]

    @staticmethod
    def nearest_equivalent_angle(angle: float, reference: float) -> float:
        return float(reference) + math.atan2(
            math.sin(float(angle) - float(reference)),
            math.cos(float(angle) - float(reference)),
        )

    def move_joint_positions(self, positions: list[float], label: str, params) -> bool:
        state = RobotState(self.robot_model)
        state.set_joint_group_positions(GROUP_NAME, positions)
        state.update()
        return self.plan_and_maybe_execute(
            label,
            state_goal=state,
            params=params,
            joint_goal_positions=positions,
        )

    def move_joint6_relative(self, delta_deg: float, label: str) -> bool:
        backend = str(self.get_parameter("screw_motion_backend").value).strip().lower()
        if self.hardware_armed and backend in {"move_joint", "movej", "doosan"}:
            if not self.wait_for_doosan_joint_services(label):
                return False
            current_deg = self.read_current_posj_deg(label)
            if current_deg is None:
                return False
            target_deg = list(current_deg)
            before = float(target_deg[5])
            target_deg[5] = before + float(delta_deg)
            self.publish_status(
                "JOINT6_TURN",
                f"{label}: {before:.1f}deg -> {target_deg[5]:.1f}deg",
            )
            return self.call_move_joint_deg(
                target_deg,
                label,
                float(self.get_parameter("screw_move_joint_velocity_deg_s").value),
                float(self.get_parameter("screw_move_joint_acceleration_deg_s").value),
                float(self.get_parameter("screw_move_joint_time_sec").value),
            )

        positions = self.current_joint_positions()
        if positions is None:
            self.get_logger().error(f"{label}: /joint_states missing arm joints")
            return False
        index = JOINT_ORDER.index("joint_6")
        before = positions[index]
        positions[index] = before + math.radians(delta_deg)
        self.publish_status(
            "JOINT6_TURN",
            f"{label}: {math.degrees(before):.1f}deg -> {math.degrees(positions[index]):.1f}deg",
        )
        return self.move_joint_positions(positions, label, self.screw_params)

    def plan_and_maybe_execute(self, label: str, pose_goal=None, state_goal=None, params=None, joint_goal_positions=None) -> bool:
        if pose_goal is not None:
            xyz = np.array(
                [
                    pose_goal.pose.position.x,
                    pose_goal.pose.position.y,
                    pose_goal.pose.position.z,
                ],
                dtype=float,
            )
            self.validate_workspace_point(xyz, label)
        self.arm.set_start_state_to_current_state()
        start_xyz = get_ee_matrix(self.robot)[:3, 3].copy()
        if pose_goal is not None:
            self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=EE_LINK)
        elif state_goal is not None:
            self.arm.set_goal_state(robot_state=state_goal)
        else:
            raise ValueError("pose_goal or state_goal is required")
        plan_result = self.arm.plan(parameters=params) if params else self.arm.plan()
        if not plan_result:
            self.get_logger().error(f"{label}: planning failed")
            return False
        if not self.hardware_armed:
            self.publish_status("PLAN_ONLY", label)
            return True
        if not self.wait_for_trajectory_action_server(label):
            return False
        if not self.execute_trajectory_via_controller(plan_result.trajectory, label):
            self.get_logger().error(f"{label}: execution failed")
            return False
        time.sleep(0.1)
        if pose_goal is not None and parse_bool(self.get_parameter("verify_motion").value):
            end_xyz = get_ee_matrix(self.robot)[:3, 3].copy()
            goal_xyz = np.array(
                [
                    pose_goal.pose.position.x,
                    pose_goal.pose.position.y,
                    pose_goal.pose.position.z,
                ],
                dtype=float,
            )
            error = float(np.linalg.norm(end_xyz - goal_xyz))
            if error > float(self.get_parameter("pose_goal_tolerance_m").value):
                self.get_logger().error(f"{label}: pose error={error:.3f}m")
                return False
            moved = float(np.linalg.norm(end_xyz - start_xyz))
            self.publish_status("MOTION_DONE", f"{label}: moved={moved:.3f}m error={error:.3f}m")
        elif joint_goal_positions is not None and parse_bool(self.get_parameter("verify_motion").value):
            if not self.verify_joint_goal(joint_goal_positions, label):
                return False
        return True

    def execute_trajectory_via_controller(self, trajectory, label: str) -> bool:
        trajectory_msg = trajectory
        if hasattr(trajectory_msg, "get_robot_trajectory_msg"):
            trajectory_msg = trajectory_msg.get_robot_trajectory_msg()
        joint_trajectory = trajectory_msg.joint_trajectory
        if not joint_trajectory.points:
            self.get_logger().error(f"{label}: empty trajectory")
            return False
        try:
            self.unwrap_joint_trajectory_for_raw_controller(joint_trajectory, label)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_trajectory
        wait_sec = float(self.get_parameter("trajectory_action_wait_sec").value)
        goal_future = self.trajectory_action.send_goal_async(goal)
        goal_result = self.wait_for_future(goal_future, f"send trajectory {label}", wait_sec)
        if goal_result is None or not goal_result.accepted:
            self.get_logger().error(f"{label}: trajectory goal rejected")
            return False

        timeout_sec = float(self.get_parameter("trajectory_execution_timeout_sec").value)
        result_future = goal_result.get_result_async()
        result = self.wait_for_future(result_future, f"execute trajectory {label}", timeout_sec)
        if result is None:
            return False
        action_result = result.result
        if action_result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"{label}: controller failed code={action_result.error_code} "
                f"{action_result.error_string}"
            )
            return False
        self.publish_status("MOTION_EXECUTED", label)
        return True

    def unwrap_joint_trajectory_for_raw_controller(self, joint_trajectory, label: str) -> None:
        previous = [
            float(self.latest_raw_joint_positions.get(name, point_position))
            for name, point_position in zip(
                joint_trajectory.joint_names,
                joint_trajectory.points[0].positions,
            )
        ]
        start_positions = list(previous)
        max_step = 0.0
        joint6_min = math.radians(float(self.get_parameter("joint6_raw_min_deg").value))
        joint6_max = math.radians(float(self.get_parameter("joint6_raw_max_deg").value))
        for point in joint_trajectory.points:
            positions = list(point.positions)
            for index, name in enumerate(joint_trajectory.joint_names):
                if name not in JOINT_ORDER or index >= len(positions):
                    continue
                unwrapped = self.nearest_equivalent_angle(float(positions[index]), previous[index])
                if name == "joint_6" and (unwrapped < joint6_min or unwrapped > joint6_max):
                    raise RuntimeError(
                        f"{label}: refusing raw-controller joint_6 target "
                        f"{math.degrees(unwrapped):.1f}deg outside "
                        f"[{math.degrees(joint6_min):.1f}, {math.degrees(joint6_max):.1f}]deg"
                    )
                max_step = max(max_step, abs(unwrapped - previous[index]))
                positions[index] = unwrapped
                previous[index] = unwrapped
            point.positions = positions

        end_deltas = [
            abs(previous[index] - start_positions[index])
            for index, name in enumerate(joint_trajectory.joint_names)
            if name in JOINT_ORDER
        ]
        max_total = max(end_deltas, default=0.0)
        max_allowed = math.radians(
            float(self.get_parameter("max_single_segment_joint_motion_deg").value)
        )
        self.get_logger().info(
            f"{label}: prepared raw-controller trajectory; "
            f"max adjacent step={math.degrees(max_step):.1f}deg "
            f"max start-to-end move={math.degrees(max_total):.1f}deg"
        )
        if max_total > max_allowed:
            raise RuntimeError(
                f"{label}: refusing raw-controller branch move "
                f"{math.degrees(max_total):.1f}deg > {math.degrees(max_allowed):.1f}deg"
            )

    def verify_joint_goal(self, target_positions: list[float], label: str) -> bool:
        current = self.current_joint_positions()
        if current is None:
            self.get_logger().error(f"{label}: cannot verify joint goal")
            return False
        errors = [
            abs(math.atan2(math.sin(target - now), math.cos(target - now)))
            for target, now in zip(target_positions, current)
        ]
        max_error = max(errors) if errors else 0.0
        tolerance = float(self.get_parameter("joint_goal_tolerance_rad").value)
        if max_error > tolerance:
            self.get_logger().error(f"{label}: max joint error={max_error:.4f}rad")
            return False
        return True

    def wait_for_future(self, future, label: str, timeout_sec: float):
        deadline = time.monotonic() + max(timeout_sec, 0.1)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().error(f"{label}: timeout after {timeout_sec:.1f}s")
            return None
        if future.exception() is not None:
            self.get_logger().error(f"{label}: service raised {future.exception()}")
            return None
        return future.result()

    def command_gripper(self, command: str, width_m: float, label: str) -> bool:
        if not self.hardware_armed:
            self.publish_status("GRIPPER_PLAN_ONLY", f"{label}: {command} width_m={width_m:.3f}")
            return True
        timeout = float(self.get_parameter("gripper_service_timeout_sec").value)
        if not self.gripper_set.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("gripper_set_service unavailable")
            return False
        req = SetGripper.Request()
        req.command = command
        req.width_m = float(width_m)
        req.force_n = float(self.get_parameter("lid_gripper_force_n").value)
        future = self.gripper_set.call_async(req)
        result = self.wait_for_future(future, label, timeout)
        if result is None or not result.success:
            self.get_logger().error(f"{label}: gripper command failed")
            return False
        return True

    def run_sequence(self) -> tuple[bool, str]:
        if not self.wait_for_camera_inputs(3.0):
            return False, "camera inputs are not ready"
        if not self.ensure_moveit_initialized():
            return False, "MoveItPy initialization failed; check /joint_states and MoveIt planning pipelines"
        if not self.move_to_observe_if_configured():
            return False, "observe posture move failed"
        if not self.wait_for_cup_pose_at_holder():
            return False, "cup pose is not validated at holder"
        time.sleep(float(self.get_parameter("aruco_redetect_settle_sec").value))
        lid_pose = self.detect_lid_pose("initial lid")
        if lid_pose is None:
            return False, "lid ArUco was not detected"

        open_width = float(self.get_parameter("lid_gripper_release_width_m").value)
        close_width = float(self.get_parameter("lid_gripper_close_width_m").value)
        if not self.command_gripper("set_width", open_width, "preopen lid gripper"):
            return False, "gripper preopen failed"
        approach, pick = self.pose_pair_for_lid_pick(lid_pose)
        if not self.plan_and_maybe_execute("lid pick approach", pose_goal=approach, params=self.ptp_params):
            return False, "lid pick approach failed"
        if not self.plan_and_maybe_execute("lid pick descend", pose_goal=pick, params=self.lin_params):
            return False, "lid pick descend failed"
        if not self.command_gripper("set_width", close_width, "close on lid"):
            return False, "lid grip failed"
        time.sleep(float(self.get_parameter("lid_grip_settle_sec").value))
        if not self.plan_and_maybe_execute("lift lid", pose_goal=approach, params=self.lin_params):
            return False, "lid lift failed"

        holder_approach, holder_target = self.pose_pair_for_holder(lid_pose)
        if not self.plan_and_maybe_execute("holder approach", pose_goal=holder_approach, params=self.ptp_params):
            return False, "holder approach failed"
        if not self.plan_and_maybe_execute("lower lid onto holder cup", pose_goal=holder_target, params=self.lin_params):
            return False, "holder target descend failed"

        turn_deg = (
            float(self.get_parameter("screw_turn_deg").value)
            * (1.0 if float(self.get_parameter("screw_turn_direction").value) >= 0.0 else -1.0)
        )
        cycles = max(1, int(self.get_parameter("screw_cycles").value))
        for cycle in range(1, cycles + 1):
            if not self.move_joint6_relative(turn_deg, f"screw cycle {cycle} tighten"):
                return False, "joint_6 tighten failed"
            if not self.command_gripper("set_width", open_width, f"screw cycle {cycle} release"):
                return False, "lid release failed"
            time.sleep(float(self.get_parameter("lid_release_settle_sec").value))
            reset_needed = cycle < cycles or parse_bool(self.get_parameter("return_wrist_after_final").value)
            if reset_needed and not self.move_joint6_relative(-turn_deg, f"screw cycle {cycle} reset open wrist"):
                return False, "joint_6 reset failed"
            if cycle < cycles:
                if not self.plan_and_maybe_execute("holder observe for regrip", pose_goal=holder_approach, params=self.lin_params):
                    return False, "holder regrip observe failed"
                time.sleep(float(self.get_parameter("aruco_redetect_settle_sec").value))
                lid_pose = self.detect_lid_pose(f"regrip cycle {cycle + 1}")
                if lid_pose is None:
                    return False, "lid ArUco redetect failed"
                xy_error = float(np.linalg.norm(lid_pose.center_xyz[:2] - self.holder_target_xyz[:2]))
                if xy_error > float(self.get_parameter("regrip_max_xy_error_m").value):
                    return False, f"lid regrip xy drift too large: {xy_error:.3f}m"
                _, regrip_target = self.pose_pair_for_lid_pick(
                    lid_pose,
                    target_z=float(self.holder_target_xyz[2]),
                )
                if not self.plan_and_maybe_execute("lid regrip target", pose_goal=regrip_target, params=self.lin_params):
                    return False, "lid regrip target failed"
                if not self.command_gripper("set_width", close_width, f"regrip cycle {cycle + 1}"):
                    return False, "lid regrip failed"
                time.sleep(float(self.get_parameter("lid_grip_settle_sec").value))

        if parse_bool(self.get_parameter("retreat_after_sequence").value):
            if not self.plan_and_maybe_execute("retreat after lid screw", pose_goal=holder_approach, params=self.lin_params):
                return False, "retreat failed"
        return True, "lid screw-on holder sequence complete"

    def on_trigger(self, _request, response):
        if not self.running_lock.acquire(blocking=False):
            response.success = False
            response.message = "lid screw sequence already running"
            return response
        try:
            self.publish_status("STARTED")
            ok, message = self.run_sequence()
            self.publish_status("DONE" if ok else "FAILED", message)
            response.success = ok
            response.message = message
            if ok and parse_bool(self.get_parameter("shutdown_on_complete").value):
                threading.Thread(target=rclpy.shutdown, daemon=True).start()
            return response
        except Exception as exc:
            self.get_logger().error(f"lid screw sequence exception: {exc}")
            response.success = False
            response.message = str(exc)
            self.publish_status("FAILED", str(exc))
            return response
        finally:
            self.running_lock.release()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidScrewOnHolderNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
