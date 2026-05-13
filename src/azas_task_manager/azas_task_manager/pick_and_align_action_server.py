import time

import rclpy
from azas_interfaces.action import PickAndAlign
from azas_motion.alignment import compute_no_motion_pick_plan
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


PDF_PICK_PLACE_STATES = (
    "HOME",
    "pick_approach",
    "pick",
    "pick_approach",
    "place_approach",
    "place",
    "place_approach",
)


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
        self.declare_parameter("require_base_link_pose", True)
        self.declare_parameter("fake_gripper_open_service", "/jarvis/rg2/open")
        self.declare_parameter("fake_gripper_close_service", "/jarvis/rg2/close")
        self.declare_parameter("call_fake_gripper_services", False)
        self.declare_parameter("approach_z_offset_m", 0.10)
        self.declare_parameter("lift_z_offset_m", 0.12)
        self._callback_group = ReentrantCallbackGroup()
        self._latest_tumbler_pose = None
        self._pose_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("tumbler_pose_topic").value),
            self._on_tumbler_pose,
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

    def execute_callback(self, goal_handle):
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
        for state in PDF_PICK_PLACE_STATES:
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
        feedback = PickAndAlign.Feedback()
        self._publish_feedback(
            goal_handle,
            feedback,
            "WAIT_TUMBLER_POSE",
            f"Waiting for PoseStamped on {self.get_parameter('tumbler_pose_topic').value}",
        )
        pose_msg = self._wait_for_tumbler_pose()
        if pose_msg is None:
            return self._fail_result(
                goal_handle,
                "TUMBLER_POSE_TIMEOUT",
                "Timed out waiting for tumbler pose; no real robot motion was commanded",
            )
        if bool(self.get_parameter("require_base_link_pose").value) and pose_msg.header.frame_id != "base_link":
            return self._fail_result(
                goal_handle,
                "TUMBLER_POSE_NOT_BASE_LINK",
                f"Expected base_link pose, got {pose_msg.header.frame_id!r}; no real robot motion was commanded",
            )

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

        self._publish_feedback(goal_handle, feedback, "FAKE_GRIPPER_OPEN", "No real RG2 command; optional fake Trigger only")
        if not self._call_fake_gripper_if_enabled(
            str(self.get_parameter("fake_gripper_open_service").value),
            "open",
        ):
            return self._fail_result(
                goal_handle,
                "FAKE_GRIPPER_OPEN_FAILED",
                "Fake gripper open failed; no real robot motion was commanded",
            )

        self._publish_feedback(goal_handle, feedback, "FAKE_APPROACH", self._pose_xyz(plan.approach_pose))
        self._publish_feedback(goal_handle, feedback, "FAKE_GRIPPER_CLOSE", "No real RG2 command; optional fake Trigger only")
        if not self._call_fake_gripper_if_enabled(
            str(self.get_parameter("fake_gripper_close_service").value),
            "close",
        ):
            return self._fail_result(
                goal_handle,
                "FAKE_GRIPPER_CLOSE_FAILED",
                "Fake gripper close failed; no real robot motion was commanded",
            )

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

    def _wait_for_tumbler_pose(self):
        timeout_sec = float(self.get_parameter("pose_wait_timeout_sec").value)
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        observed = self._latest_tumbler_pose
        while observed is None and time.monotonic() < deadline:
            time.sleep(0.05)
            observed = self._latest_tumbler_pose
        return observed

    def _call_fake_gripper_if_enabled(self, service_name: str, command: str) -> bool:
        if not bool(self.get_parameter("call_fake_gripper_services").value):
            self.get_logger().info(
                f"Skipping fake gripper {command}; call_fake_gripper_services=false"
            )
            return True
        self.get_logger().warn(
            f"Calling fake gripper {command} service {service_name} with std_srvs/srv/Trigger; "
            "does not command real RG2 and has no real-command fallback"
        )
        client = self.create_client(Trigger, service_name, callback_group=self._callback_group)
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Fake gripper service unavailable: {service_name}")
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().error(f"Fake gripper service timed out: {service_name}")
            return False
        response = future.result()
        if response is None or not response.success:
            message = getattr(response, "message", "<no response>")
            self.get_logger().error(f"Fake gripper service failed: {service_name}: {message}")
            return False
        self.get_logger().info(f"Fake gripper {command} accepted: {response.message}")
        return True

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
