import time

import rclpy
from azas_interfaces.action import PickAndAlign
from azas_interfaces.msg import CupDetection
from azas_motion.alignment import (
    PICK_PLACE_STATES,
    ObservePoseConfig,
    SideGraspConfig,
    compute_observe_pose,
    compute_no_motion_pick_plan,
    compute_side_grasp_plan,
)
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class PickAndAlignActionServer(Node):
    """MVP-1 orchestration boundary.

    The action owns sequence state only. Perception/calibration provide robot-frame
    poses, azas_motion plans/executes MoveItPy motions, and azas_gripper controls RG2.
    LLM/VLA output is intentionally excluded from coordinate generation.
    """

    def __init__(self):
        super().__init__("pick_and_align_action_server")
        self.declare_parameter("execution_mode", "no_motion")
        self.declare_parameter("tumbler_pose_topic", "/jarvis/tumbler_dispenser/tumbler_pose")
        self.declare_parameter("pose_wait_timeout_sec", 5.0)
        self.declare_parameter("cup_detection_topic", "/azas/cup_detection")
        self.declare_parameter("require_upright_detection_status", True)
        self.declare_parameter("require_base_link_pose", True)
        self.declare_parameter("gripper_open_service", "/jarvis/rg2/open")
        self.declare_parameter("gripper_close_service", "/jarvis/rg2/close")
        self.declare_parameter("enable_gripper_service_calls", False)
        self.declare_parameter("grasp_mode", "side")
        self.declare_parameter("observe_pose_x", 0.35)
        self.declare_parameter("observe_pose_y", -0.25)
        self.declare_parameter("observe_pose_z", 0.45)
        self.declare_parameter("observe_qx", 0.0)
        self.declare_parameter("observe_qy", 0.0)
        self.declare_parameter("observe_qz", 0.0)
        self.declare_parameter("observe_qw", 1.0)
        self.declare_parameter("observe_frame", "base_link")
        self.declare_parameter("approach_z_offset_m", 0.10)
        self.declare_parameter("lift_z_offset_m", 0.12)
        self.declare_parameter("side_grasp_orientation_source", "parameter")
        self.declare_parameter("side_grasp_qx", 0.0)
        self.declare_parameter("side_grasp_qy", 0.0)
        self.declare_parameter("side_grasp_qz", 0.0)
        self.declare_parameter("side_grasp_qw", 1.0)
        self.declare_parameter("side_approach_axis", "-x")
        self.declare_parameter("side_approach_offset_m", 0.12)
        self.declare_parameter("side_clearance_m", 0.02)
        self.declare_parameter("cup_radius_m", 0.035)
        self.declare_parameter("grasp_height_offset_m", 0.06)
        self.declare_parameter("lift_offset_m", 0.12)
        self.declare_parameter("min_grasp_z_m", 0.03)
        self.declare_parameter("max_grasp_z_m", 0.40)
        self._callback_group = ReentrantCallbackGroup()
        self._latest_tumbler_pose = None
        self._latest_detection_status = ""
        self._pose_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("tumbler_pose_topic").value),
            self._on_tumbler_pose,
            10,
            callback_group=self._callback_group,
        )
        self._detection_sub = self.create_subscription(
            CupDetection,
            str(self.get_parameter("cup_detection_topic").value),
            self._on_cup_detection,
            10,
            callback_group=self._callback_group,
        )
        self._server = ActionServer(
            self,
            PickAndAlign,
            "/azas/pick_and_align",
            self.execute_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().warn(
            "PickAndAlign server started in no-motion capable mode. "
            "It does not command Doosan, MoveIt, or real RG2 hardware."
        )

    def _on_tumbler_pose(self, msg: PoseStamped) -> None:
        self._latest_tumbler_pose = msg

    def _on_cup_detection(self, msg: CupDetection) -> None:
        self._latest_detection_status = msg.status

    def execute_callback(self, goal_handle):
        # The action message has an execute_motion field for the future real
        # path, but this server intentionally ignores it today. Real movement is
        # blocked until measured calibration, MoveIt execution, RG2 behavior,
        # collision/workspace bounds, and operator gates are all implemented.
        execution_mode = str(self.get_parameter("execution_mode").value).strip().lower()
        if execution_mode == "skeleton":
            return self._execute_skeleton(goal_handle)
        if execution_mode != "no_motion":
            return self._fail_result(
                goal_handle,
                "UNSUPPORTED_EXECUTION_MODE",
                f"Unsupported execution_mode={execution_mode!r}; no real robot motion was commanded",
            )
        return self._execute_no_motion(goal_handle)

    def _execute_skeleton(self, goal_handle):
        feedback = PickAndAlign.Feedback()
        for state in PICK_PLACE_STATES:
            feedback.state = state
            feedback.detail = "skeleton state; subsystem execution pending"
            goal_handle.publish_feedback(feedback)

        result = PickAndAlign.Result()
        result.success = False
        result.error_code = "SKELETON_ONLY"
        result.message = (
            "PickAndAlign action contract is present, but calibrated perception, RG2, "
            "and MoveItPy execution are not yet connected."
        )
        goal_handle.succeed()
        return result

    def _execute_no_motion(self, goal_handle):
        # This path validates perception/pose plumbing and produces grasp
        # candidate poses for diagnostics. It must remain side-effect free:
        # no Doosan trajectory execution and no real RG2 commands.
        feedback = PickAndAlign.Feedback()
        if bool(self.get_parameter("enable_gripper_service_calls").value):
            return self._fail_result(
                goal_handle,
                "REAL_GRIPPER_NOT_SUPPORTED_IN_NO_MOTION",
                (
                    "enable_gripper_service_calls=true is not allowed in execution_mode=no_motion; "
                    "no gripper, Doosan, MoveIt, or real robot command was sent"
                ),
            )
        observe_detail = self._observe_pose_feedback_detail()
        if observe_detail.startswith("invalid"):
            return self._fail_result(
                goal_handle,
                "INVALID_OBSERVE_POSE_CONFIG",
                f"{observe_detail}; no real robot motion was commanded",
            )
        self._publish_feedback(
            goal_handle,
            feedback,
            "PLAN_OBSERVE_CUP_POSE_NO_MOTION",
            observe_detail,
        )
        self._publish_feedback(
            goal_handle,
            feedback,
            "DETECT_CUP_PENDING",
            (
                "OBSERVE_CUP_POSE is planning-only in this action; waiting for "
                f"PoseStamped on {self.get_parameter('tumbler_pose_topic').value}"
            ),
        )
        self._publish_feedback(
            goal_handle,
            feedback,
            "WAIT_TUMBLER_POSE",
            f"Waiting for PoseStamped on {self.get_parameter('tumbler_pose_topic').value}",
        )
        pose_msg = self._wait_for_tumbler_pose()
        if pose_msg is None:
            orientation_error = self._cup_orientation_error_code()
            if orientation_error:
                return self._fail_result(
                    goal_handle,
                    orientation_error,
                    f"Cup detection was not upright: status={self._latest_detection_status!r}; "
                    "no real robot motion was commanded",
                )
            return self._fail_result(
                goal_handle,
                "TUMBLER_POSE_TIMEOUT",
                "Timed out waiting for tumbler pose; no real robot motion was commanded",
            )
        orientation_error = self._cup_orientation_error_code()
        if orientation_error:
            return self._fail_result(
                goal_handle,
                orientation_error,
                f"Cup detection was not upright: status={self._latest_detection_status!r}; "
                "no real robot motion was commanded",
            )
        if bool(self.get_parameter("require_base_link_pose").value) and pose_msg.header.frame_id != "base_link":
            return self._fail_result(
                goal_handle,
                "TUMBLER_POSE_NOT_BASE_LINK",
                f"Expected base_link pose, got {pose_msg.header.frame_id!r}; no real robot motion was commanded",
            )

        grasp_mode = str(self.get_parameter("grasp_mode").value).strip().lower()
        if grasp_mode == "side":
            return self._execute_side_no_motion(goal_handle, feedback, pose_msg)
        if grasp_mode == "vertical":
            return self._execute_vertical_no_motion(goal_handle, feedback, pose_msg)
        return self._fail_result(
            goal_handle,
            "UNSUPPORTED_GRASP_MODE",
            f"Unsupported grasp_mode={grasp_mode!r}; no real robot motion was commanded",
        )

    def _execute_vertical_no_motion(self, goal_handle, feedback, pose_msg):
        detail = f"pose frame={pose_msg.header.frame_id} {self._pose_xyz(pose_msg.pose)}"
        self._publish_feedback(goal_handle, feedback, "PLAN_PICK_APPROACH_NO_MOTION", detail)
        try:
            plan = compute_no_motion_pick_plan(
                pose_msg.pose,
                approach_z_offset_m=float(self.get_parameter("approach_z_offset_m").value),
                lift_z_offset_m=float(self.get_parameter("lift_z_offset_m").value),
            )
        except ValueError as exc:
            return self._fail_result(
                goal_handle,
                "INVALID_NO_MOTION_PICK_CONFIG",
                f"{exc}; no real robot motion was commanded",
            )
        self.get_logger().info(
            "No-motion pick plan: "
            f"pick={self._pose_xyz(plan.pick_pose)} "
            f"approach={self._pose_xyz(plan.approach_pose)} "
            f"lift={self._pose_xyz(plan.lift_pose)}"
        )

        self._publish_no_motion_gripper_feedback(goal_handle, feedback, "open")

        self._publish_feedback(goal_handle, feedback, "FAKE_APPROACH", self._pose_xyz(plan.approach_pose))
        self._publish_no_motion_gripper_feedback(goal_handle, feedback, "close")

        self._publish_feedback(goal_handle, feedback, "FAKE_LIFT", self._pose_xyz(plan.lift_pose))
        self._publish_feedback(goal_handle, feedback, "DONE_NO_MOTION", "No real robot motion was commanded")

        result = PickAndAlign.Result()
        result.success = True
        result.error_code = "NO_MOTION_PICK_SEQUENCE_OK"
        result.message = (
            "No-motion pick sequence completed from base_link tumbler pose. "
            "No real robot motion was commanded."
        )
        goal_handle.succeed()
        return result

    def _execute_side_no_motion(self, goal_handle, feedback, pose_msg):
        try:
            plan = compute_side_grasp_plan(
                pose_msg.pose,
                self._side_grasp_config(),
            )
        except ValueError as exc:
            return self._fail_result(
                goal_handle,
                self._side_grasp_error_code(str(exc)),
                f"{exc}; no real robot motion was commanded",
            )
        plan_detail = (
            f"pose frame={pose_msg.header.frame_id} "
            f"axis={plan.approach_axis} distance={plan.approach_distance_m:.3f} "
            f"approach=({self._pose_xyz(plan.approach_pose)}) "
            f"grasp=({self._pose_xyz(plan.grasp_pose)}) "
            f"lift=({self._pose_xyz(plan.lift_pose)}) "
            f"quat=({self._pose_quat(plan.grasp_pose)}) warning={plan.warning}"
        )
        self._publish_feedback(goal_handle, feedback, "COMPUTE_SIDE_GRASP", plan_detail)
        self.get_logger().warn(
            "No-motion side grasp plan: "
            f"{plan_detail}. No Doosan, MoveIt, or real RG2 command will be sent."
        )

        self._publish_feedback(
            goal_handle,
            feedback,
            "SIDE_APPROACH_NO_MOTION",
            (
                f"{self._pose_xyz(plan.approach_pose)} "
                f"axis={plan.approach_axis} distance={plan.approach_distance_m:.3f}"
            ),
        )
        self._publish_feedback(
            goal_handle,
            feedback,
            "SIDE_PICK_NO_MOTION",
            f"{self._pose_xyz(plan.grasp_pose)} warning={plan.warning}",
        )
        self._publish_no_motion_gripper_feedback(goal_handle, feedback, "close")

        self._publish_feedback(
            goal_handle,
            feedback,
            "SIDE_LIFT_NO_MOTION",
            self._pose_xyz(plan.lift_pose),
        )
        self._publish_feedback(
            goal_handle,
            feedback,
            "DONE_NO_MOTION",
            "No real robot motion was commanded; side grasp orientation is placeholder",
        )

        result = PickAndAlign.Result()
        result.success = True
        result.error_code = "NO_MOTION_SIDE_GRASP_OK"
        result.message = (
            "No-motion side grasp sequence completed from base_link cup reference pose. "
            "No real robot motion was commanded. Side grasp orientation is placeholder."
        )
        goal_handle.succeed()
        return result

    def _wait_for_tumbler_pose(self):
        timeout_sec = float(self.get_parameter("pose_wait_timeout_sec").value)
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        observed = self._latest_tumbler_pose
        while observed is None and time.monotonic() < deadline:
            if self._cup_orientation_error_code():
                break
            time.sleep(0.05)
            observed = self._latest_tumbler_pose
        return observed

    def _publish_no_motion_gripper_feedback(self, goal_handle, feedback, command: str) -> None:
        state = f"GRIPPER_{command.upper()}_NO_MOTION"
        self._publish_feedback(
            goal_handle,
            feedback,
            state,
            f"Skipped gripper {command}; execution_mode=no_motion never calls RG2 services",
        )

    def _side_grasp_config(self) -> SideGraspConfig:
        return SideGraspConfig(
            orientation_source=str(self.get_parameter("side_grasp_orientation_source").value),
            side_grasp_qx=float(self.get_parameter("side_grasp_qx").value),
            side_grasp_qy=float(self.get_parameter("side_grasp_qy").value),
            side_grasp_qz=float(self.get_parameter("side_grasp_qz").value),
            side_grasp_qw=float(self.get_parameter("side_grasp_qw").value),
            side_approach_axis=str(self.get_parameter("side_approach_axis").value),
            side_approach_offset_m=float(self.get_parameter("side_approach_offset_m").value),
            side_clearance_m=float(self.get_parameter("side_clearance_m").value),
            cup_radius_m=float(self.get_parameter("cup_radius_m").value),
            grasp_height_offset_m=float(self.get_parameter("grasp_height_offset_m").value),
            lift_offset_m=float(self.get_parameter("lift_offset_m").value),
            min_grasp_z_m=float(self.get_parameter("min_grasp_z_m").value),
            max_grasp_z_m=float(self.get_parameter("max_grasp_z_m").value),
        )

    def _fail_result(self, goal_handle, error_code: str, message: str):
        result = PickAndAlign.Result()
        result.success = False
        result.error_code = error_code
        result.message = message
        goal_handle.succeed()
        return result

    @staticmethod
    def _publish_feedback(goal_handle, feedback, state: str, detail: str) -> None:
        feedback.state = state
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)
        time.sleep(0.05)

    @staticmethod
    def _pose_xyz(pose: Pose) -> str:
        return f"x={pose.position.x:.3f} y={pose.position.y:.3f} z={pose.position.z:.3f}"

    @staticmethod
    def _pose_quat(pose: Pose) -> str:
        return (
            f"qx={pose.orientation.x:.6f} qy={pose.orientation.y:.6f} "
            f"qz={pose.orientation.z:.6f} qw={pose.orientation.w:.6f}"
        )

    @staticmethod
    def _side_grasp_error_code(error_text: str) -> str:
        if error_text.startswith("UNSUPPORTED_SIDE_APPROACH_AXIS"):
            return "UNSUPPORTED_SIDE_APPROACH_AXIS"
        if error_text.startswith("SIDE_GRASP_Z_OUT_OF_BOUNDS"):
            return "SIDE_GRASP_Z_OUT_OF_BOUNDS"
        return "INVALID_SIDE_GRASP_CONFIG"

    def _cup_orientation_error_code(self) -> str:
        if not bool(self.get_parameter("require_upright_detection_status").value):
            return ""
        status = self._latest_detection_status
        if not status:
            return ""
        if status.startswith("detected:upright"):
            return ""
        if "unknown_orientation" in status or "orientation=unknown" in status:
            return "CUP_ORIENTATION_UNKNOWN"
        if status.startswith("rejected:") or status.startswith("detected:"):
            return "CUP_ORIENTATION_NOT_UPRIGHT"
        return ""

    def _observe_pose_feedback_detail(self) -> str:
        try:
            pose = compute_observe_pose(
                ObservePoseConfig(
                    x=float(self.get_parameter("observe_pose_x").value),
                    y=float(self.get_parameter("observe_pose_y").value),
                    z=float(self.get_parameter("observe_pose_z").value),
                    qx=float(self.get_parameter("observe_qx").value),
                    qy=float(self.get_parameter("observe_qy").value),
                    qz=float(self.get_parameter("observe_qz").value),
                    qw=float(self.get_parameter("observe_qw").value),
                )
            )
        except ValueError as exc:
            return f"invalid OBSERVE_CUP_POSE config: {exc}"
        frame_id = str(self.get_parameter("observe_frame").value).strip() or "base_link"
        return (
            f"OBSERVE_CUP_POSE candidate frame={frame_id} {self._pose_xyz(pose)} "
            f"{self._pose_quat(pose)}; planning-only/no-motion, no MoveIt execute"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PickAndAlignActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
