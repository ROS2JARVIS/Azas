#!/usr/bin/env python3

import math
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO

from azas_gripper.onrobot import RG


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


def clamp_to_safe_workspace(x, y, z, logger, z_min=SAFE_Z_MIN):
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


class YoloCupPickNode(Node):
    def __init__(self):
        super().__init__("yolo_cup_pick_node")

        self.declare_parameter(
            "model_path",
            "/home/ssu/ros2_ws/yolo_runs/cup_yolov8n_ft1/weights/best.pt",
        )
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("target_class", "cup")
        self.declare_parameter("auto_pick", False)
        self.declare_parameter("auto_pick_interval", 3.0)
        self.declare_parameter("pick_depth_ratio", 0.55)
        self.declare_parameter("depth_patch_radius", 7)
        self.declare_parameter("min_depth_valid_ratio", 0.03)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 1.20)
        self.declare_parameter("redetect_on_approach", True)
        self.declare_parameter("redetect_settle_sec", 0.5)
        self.declare_parameter("grasp_mode", "side")
        dynamic_param = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("side_grasp_axis", "y_axis", dynamic_param)
        self.declare_parameter("side_grasp_direction", -1.0)
        self.declare_parameter("side_approach_offset", 0.12)
        self.declare_parameter("side_staging_offset", 0.24)
        self.declare_parameter("side_grasp_offset", 0.035)
        self.declare_parameter("side_grasp_z_offset", 0.05)
        self.declare_parameter("side_orientation_mode", "approach")
        self.declare_parameter("side_tool_roll_deg", 0.0)
        self.declare_parameter("side_roll_deg", 0.0)
        self.declare_parameter("side_pitch_deg", 90.0)
        self.declare_parameter("side_yaw_deg", 0.0)
        self.declare_parameter("pick_z_offset", 0.20)
        self.declare_parameter("approach_offset", 0.12)
        self.declare_parameter("safe_z", 0.50)
        self.declare_parameter("min_motion_z", 0.12)
        self.declare_parameter("return_home_after_task", True)
        self.declare_parameter("verify_motion", True)
        self.declare_parameter("motion_verify_tolerance", 0.01)
        self.declare_parameter("move_to_camera_home", True)
        self.declare_parameter("camera_home_x", 0.45)
        self.declare_parameter("camera_home_y", 0.00)
        self.declare_parameter("camera_home_z", 0.62)
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
        self.pick_depth_ratio = float(self.get_parameter("pick_depth_ratio").value)
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
        self.side_grasp_offset = float(self.get_parameter("side_grasp_offset").value)
        self.side_grasp_z_offset = float(
            self.get_parameter("side_grasp_z_offset").value
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
        self.return_home_after_task = parse_bool(
            self.get_parameter("return_home_after_task").value
        )
        self.verify_motion = parse_bool(self.get_parameter("verify_motion").value)
        self.motion_verify_tolerance = float(
            self.get_parameter("motion_verify_tolerance").value
        )
        self.move_to_camera_home = parse_bool(
            self.get_parameter("move_to_camera_home").value
        )
        self.camera_home_x = float(self.get_parameter("camera_home_x").value)
        self.camera_home_y = float(self.get_parameter("camera_home_y").value)
        self.camera_home_z = float(self.get_parameter("camera_home_z").value)
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
        if self.side_grasp_axis not in {"x", "y"}:
            raise ValueError("side_grasp_axis must be 'x' or 'y'")
        self.side_grasp_direction = 1.0 if self.side_grasp_direction >= 0 else -1.0
        if self.side_orientation_mode not in {"approach", "euler", "home"}:
            raise ValueError(
                "side_orientation_mode must be 'approach', 'euler', or 'home'"
            )
        if self.side_staging_offset < self.side_approach_offset:
            self.get_logger().warning(
                "side_staging_offset is smaller than side_approach_offset; "
                "using side_approach_offset for staging."
            )
            self.side_staging_offset = self.side_approach_offset

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.last_detection = None
        self.picking = False
        self.has_picked_once = False
        self.last_pick_time = 0.0
        self.last_status = "waiting for command"

        calib_file = (
            Path(get_package_share_directory("azas_perception"))
            / "config"
            / "T_gripper2camera.npy"
        )
        self.gripper2cam = np.load(str(calib_file)).astype(float)
        self.gripper2cam[:3, 3] /= 1000.0
        self.get_logger().info(f"Loaded hand-eye calibration: {calib_file}")

        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

        self.get_logger().info("Initializing MoveItPy...")
        self.robot = MoveItPy(node_name="yolo_cup_pick_moveit_py")
        self.arm = self.robot.get_planning_component(GROUP_NAME)
        self.robot_model = self.robot.get_robot_model()
        self.get_logger().info("MoveItPy initialized")

        self.ompl_params = PlanRequestParameters(self.robot)
        self.ompl_params.planning_pipeline = "ompl"
        self.ompl_params.planner_id = "RRTConnect"
        self.ompl_params.max_velocity_scaling_factor = 0.2
        self.ompl_params.max_acceleration_scaling_factor = 0.1
        self.ompl_params.planning_time = 3.0

        self.pilz_params = PlanRequestParameters(self.robot)
        self.pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.pilz_params.planner_id = "PTP"
        self.pilz_params.max_velocity_scaling_factor = 0.12
        self.pilz_params.max_acceleration_scaling_factor = 0.08
        self.pilz_params.planning_time = 3.0

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
            10,
        )
        self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self._depth_callback,
            10,
        )

    def _camera_info_callback(self, msg):
        self.intrinsics = {
            "fx": msg.k[0],
            "fy": msg.k[4],
            "cx": msg.k[2],
            "cy": msg.k[5],
        }

    def _color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def plan_and_execute(self, pose_goal=None, state_goal=None, params=None):
        log = self.get_logger()
        self.arm.set_start_state_to_current_state()
        start_matrix = get_ee_matrix(self.robot)
        start_xyz = start_matrix[:3, 3].copy()
        goal_xyz = None

        if pose_goal is not None:
            x = pose_goal.pose.position.x
            y = pose_goal.pose.position.y
            z = pose_goal.pose.position.z
            x, y, z = clamp_to_safe_workspace(x, y, z, log, self.min_motion_z)
            pose_goal.pose.position.x = x
            pose_goal.pose.position.y = y
            pose_goal.pose.position.z = z
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

        self.robot.execute(
            group_name=GROUP_NAME,
            robot_trajectory=plan_result.trajectory,
            blocking=True,
        )
        self.spin_for_camera_update(0.2)

        end_matrix = get_ee_matrix(self.robot)
        end_xyz = end_matrix[:3, 3].copy()
        moved = float(np.linalg.norm(end_xyz - start_xyz))
        if goal_xyz is None:
            log.info(
                f"Execution finished. EE moved {moved:.3f} m -> "
                f"({end_xyz[0]:.3f}, {end_xyz[1]:.3f}, {end_xyz[2]:.3f})"
            )
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

    def move_joint_home(self):
        home_state = RobotState(self.robot_model)
        home_state.set_joint_group_positions(GROUP_NAME, HOME_JOINTS_RAD)
        home_state.update()
        if not self.plan_and_execute(state_goal=home_state, params=self.ompl_params):
            return False

        transform = get_ee_matrix(self.robot)
        self.update_home_orientation_from_matrix(transform)
        return True

    def move_home(self):
        if self.move_to_camera_home:
            return self.move_camera_home()
        return self.move_joint_home()

    def update_home_orientation_from_matrix(self, transform):
        qx, qy, qz, qw = Rotation.from_matrix(transform[:3, :3]).as_quat()
        self.home_ori = {
            "x": float(qx),
            "y": float(qy),
            "z": float(qz),
            "w": float(qw),
        }

    def move_camera_home(self):
        log = self.get_logger()
        candidate_zs = []
        for z in (self.camera_home_z, 0.62, 0.58, 0.54):
            if z > self.camera_home_z + 1e-6:
                continue
            if all(abs(z - candidate) > 1e-6 for candidate in candidate_zs):
                candidate_zs.append(z)

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
                params=self.pilz_params,
            ):
                continue

            self.camera_home_z = z
            transform = get_ee_matrix(self.robot)
            self.update_home_orientation_from_matrix(transform)
            return True

        return False

    def wait_until_gripper_idle(self, timeout_sec=GRIPPER_OPEN_TIMEOUT_SEC):
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
        x_ratios = [0.50, 0.35, 0.65, 0.25, 0.75]
        y_ratios = [
            self.pick_depth_ratio,
            0.45,
            0.35,
            0.65,
            0.25,
            0.75,
        ]

        points = []
        seen = set()
        for yr in y_ratios:
            for xr in x_ratios:
                u = int(x1 + xr * (x2 - x1))
                v = int(y1 + yr * (y2 - y1))
                u = max(0, min(w - 1, u))
                v = max(0, min(h - 1, v))
                if (u, v) not in seen:
                    points.append((u, v))
                    seen.add((u, v))
        return points

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
                    "No valid depth found inside target bbox. "
                    "Try a larger depth_patch_radius or lower min_depth_valid_ratio."
                )
            return None

        # Prefer the closest valid surface in the bbox. Transparent cups often
        # expose background/table depth, so closest valid depth is usually safer.
        u, v, z_m, valid_ratio = min(valid_samples, key=lambda sample: sample[2])
        if log_reason:
            log.info(
                f"Depth sample selected at ({u}, {v}): "
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

    def side_unit_vector(self):
        if self.side_grasp_axis == "x":
            return np.array([self.side_grasp_direction, 0.0], dtype=float)
        return np.array([0.0, self.side_grasp_direction], dtype=float)

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

    def pick_and_place_side(self, base_xyz):
        log = self.get_logger()
        bx, by, bz = [float(v) for v in base_xyz]
        side_vec = self.side_unit_vector()
        side_ori = self.side_grasp_orientation(side_vec)
        stage_xy = (
            np.array([bx, by], dtype=float) + side_vec * self.side_staging_offset
        )
        pre_xy = np.array([bx, by], dtype=float) + side_vec * self.side_approach_offset
        grasp_xy = np.array([bx, by], dtype=float) + side_vec * self.side_grasp_offset
        grasp_z = max(bz + self.side_grasp_z_offset, self.min_motion_z)
        pre_z = grasp_z
        lift_z = max(grasp_z + self.approach_offset, self.safe_z)
        place_approach_z = max(self.place_z + self.approach_offset, self.safe_z)

        log.info(
            f"Side grasp target base=({bx:.3f}, {by:.3f}, {bz:.3f}), "
            f"axis={self.side_grasp_axis}, dir={self.side_grasp_direction:.0f}, "
            f"ori_mode={self.side_orientation_mode}, "
            f"tool_roll={self.side_tool_roll_deg:.1f}deg, "
            f"stage=({stage_xy[0]:.3f}, {stage_xy[1]:.3f}, {pre_z:.3f}), "
            f"pre=({pre_xy[0]:.3f}, {pre_xy[1]:.3f}, {pre_z:.3f}), "
            f"grasp=({grasp_xy[0]:.3f}, {grasp_xy[1]:.3f}, {grasp_z:.3f})"
        )

        self.open_gripper_max(wait=False)

        steps = [
            (
                "move to outside side-staging pose",
                make_pose(stage_xy[0], stage_xy[1], lift_z, side_ori),
            ),
            (
                "lower at outside side-staging pose",
                make_pose(stage_xy[0], stage_xy[1], pre_z, side_ori),
            ),
            (
                "move horizontally to side pre-grasp",
                make_pose(pre_xy[0], pre_xy[1], pre_z, side_ori),
            ),
        ]
        for label, pose in steps:
            log.info(label)
            if not self.plan_and_execute(pose_goal=pose, params=self.pilz_params):
                return False

        if not self.wait_until_gripper_idle():
            return False

        refined_base = self.refine_target_from_current_view(log)
        if refined_base is not None:
            bx, by, bz = [float(v) for v in refined_base]
            pre_xy = np.array([bx, by], dtype=float) + side_vec * self.side_approach_offset
            grasp_xy = np.array([bx, by], dtype=float) + side_vec * self.side_grasp_offset
            grasp_z = max(bz + self.side_grasp_z_offset, self.min_motion_z)
            pre_z = grasp_z
            lift_z = max(grasp_z + self.approach_offset, self.safe_z)
            log.info(
                f"refined side grasp=({grasp_xy[0]:.3f}, {grasp_xy[1]:.3f}, "
                f"{grasp_z:.3f})"
            )
            if not self.plan_and_execute(
                pose_goal=make_pose(pre_xy[0], pre_xy[1], pre_z, side_ori),
                params=self.pilz_params,
            ):
                return False

        log.info("slide horizontally into cup side")
        if not self.plan_and_execute(
            pose_goal=make_pose(grasp_xy[0], grasp_xy[1], grasp_z, side_ori),
            params=self.pilz_params,
        ):
            return False

        log.info("close gripper for side grasp")
        self.gripper.move_gripper(GRIPPER_CLOSE_WIDTH, GRIPPER_FORCE)
        time.sleep(1.0)

        move_steps = [
            ("lift cup", make_pose(grasp_xy[0], grasp_xy[1], lift_z, side_ori)),
            (
                "move above syrup pump front",
                make_pose(self.place_x, self.place_y, place_approach_z, side_ori),
            ),
            (
                "place cup",
                make_pose(self.place_x, self.place_y, self.place_z, side_ori),
            ),
        ]
        for label, pose in move_steps:
            log.info(label)
            if not self.plan_and_execute(pose_goal=pose, params=self.pilz_params):
                return False

        log.info("open gripper")
        self.open_gripper_max(wait=True)

        log.info("retract")
        return self.plan_and_execute(
            pose_goal=make_pose(self.place_x, self.place_y, place_approach_z,
                                side_ori),
            params=self.pilz_params,
        )

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
            if not self.plan_and_execute(pose_goal=pose, params=self.pilz_params):
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
                params=self.pilz_params,
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
            ("lift cup", make_pose(bx, by, approach_z, self.home_ori)),
            (
                "move above syrup pump front",
                make_pose(self.place_x, self.place_y, place_approach_z, self.home_ori),
            ),
            (
                "place cup",
                make_pose(self.place_x, self.place_y, self.place_z, self.home_ori),
            ),
        ]
        for label, pose in move_steps:
            log.info(label)
            if not self.plan_and_execute(pose_goal=pose, params=self.pilz_params):
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

        self.last_status = f"pick requested: {self.target_class}"
        base_xyz = self.base_from_detection(self.last_detection, "[initial]")
        if base_xyz is None:
            log.error(f"No valid depth around {self.target_class} bbox")
            self.last_status = f"no valid depth for {self.target_class}"
            return

        self.picking = True
        self.last_status = "moving robot"
        try:
            if self.pick_and_place(base_xyz):
                self.has_picked_once = True
                self.last_pick_time = time.time()
                self.last_status = "pick finished"
            else:
                self.last_status = "pick failed"
        finally:
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
        log.info("Move JOINT HOME")
        if not self.move_joint_home():
            log.error("Joint home move failed")
            return

        if self.move_to_camera_home:
            log.info("Move HIGH CAMERA HOME")
            if not self.move_camera_home():
                log.error("High camera home move failed")
                return

        self.open_gripper_max(wait=True)

        window = "YOLO Cup Pick - p pick, a auto, r reset, esc quit"
        cv2.namedWindow(window)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
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
