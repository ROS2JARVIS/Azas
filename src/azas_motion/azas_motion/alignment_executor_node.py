import json
import math

import rclpy
from azas_motion.alignment import (
    PICK_PLACE_STATES,
    ObservePoseConfig,
    SideGraspConfig,
    compute_observe_pose,
    compute_side_grasp_plan,
)
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class AlignmentExecutorNode(Node):
    """Planning-only MoveIt boundary for pose feasibility checks.

    This node intentionally does not contain confirmed EE_LINK/GROUP_NAME/outlet/TCP
    constants. Those must come from MoveIt config and calibration YAML after hardware
    verification.
    """

    def __init__(self):
        super().__init__("alignment_executor_node")
        self.declare_parameter("enable_planning_only", True)
        self.declare_parameter("allow_execute", False)
        self.declare_parameter("planning_group", "")
        self.declare_parameter("ee_link", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("approach_pose_topic", "")
        self.declare_parameter("grasp_pose_topic", "")
        self.declare_parameter("lift_pose_topic", "")
        self.declare_parameter("planning_timeout_sec", 5.0)
        self.declare_parameter("use_fake_side_grasp_plan", False)
        self.declare_parameter("use_observe_pose_plan", False)
        self.declare_parameter("observe_pose_x", 0.35)
        self.declare_parameter("observe_pose_y", -0.25)
        self.declare_parameter("observe_pose_z", 0.45)
        self.declare_parameter("observe_qx", 0.0)
        self.declare_parameter("observe_qy", 0.0)
        self.declare_parameter("observe_qz", 0.0)
        self.declare_parameter("observe_qw", 1.0)
        self.declare_parameter("observe_frame", "base_link")
        self.declare_parameter("cup_reference_x", math.nan)
        self.declare_parameter("cup_reference_y", math.nan)
        self.declare_parameter("cup_reference_z", math.nan)
        self.declare_parameter("side_grasp_qx", 0.0)
        self.declare_parameter("side_grasp_qy", 0.0)
        self.declare_parameter("side_grasp_qz", 0.0)
        self.declare_parameter("side_grasp_qw", 1.0)
        self.declare_parameter("side_approach_axis", "-x")
        self._observe_pose = None
        self._approach_pose = None
        self._grasp_pose = None
        self._lift_pose = None
        self._moveit_py = None
        self._planning_component = None
        self._planning_attempted = False
        self._configure_pose_subscriptions()
        self._configure_observe_pose_plan()
        self._configure_fake_side_grasp_plan()
        self.get_logger().warn(
            "AlignmentExecutorNode is planning-only by default. It does not command "
            "Doosan motion, MoveIt execution, or RG2 hardware. Required sequence: "
            + " -> ".join(PICK_PLACE_STATES)
        )
        self.create_timer(1.0, self._report_planning_only_readiness)

    def _configure_pose_subscriptions(self) -> None:
        topics = (
            ("approach", "approach_pose_topic", self._on_approach_pose),
            ("grasp", "grasp_pose_topic", self._on_grasp_pose),
            ("lift", "lift_pose_topic", self._on_lift_pose),
        )
        for label, parameter_name, callback in topics:
            topic = str(self.get_parameter(parameter_name).value).strip()
            if not topic:
                self.get_logger().info(
                    f"No {label} pose topic configured; planning-only checker will wait idle"
                )
                continue
            self.create_subscription(PoseStamped, topic, callback, 10)
            self.get_logger().info(f"Subscribed to {label} pose topic for planning-only: {topic}")

    def _configure_fake_side_grasp_plan(self) -> None:
        if not bool(self.get_parameter("use_fake_side_grasp_plan").value):
            return
        reference_x = float(self.get_parameter("cup_reference_x").value)
        reference_y = float(self.get_parameter("cup_reference_y").value)
        reference_z = float(self.get_parameter("cup_reference_z").value)
        if not all(math.isfinite(value) for value in (reference_x, reference_y, reference_z)):
            self.get_logger().error(
                self._planning_log(
                    "fake_side_grasp_plan",
                    "failed",
                    error="cup_reference_x/y/z must be provided when use_fake_side_grasp_plan=true",
                    real_readiness=False,
                )
            )
            return
        reference_pose = Pose()
        reference_pose.position.x = reference_x
        reference_pose.position.y = reference_y
        reference_pose.position.z = reference_z
        reference_pose.orientation.w = 1.0
        try:
            plan = compute_side_grasp_plan(
                reference_pose,
                SideGraspConfig(
                    orientation_source="parameter",
                    side_grasp_qx=float(self.get_parameter("side_grasp_qx").value),
                    side_grasp_qy=float(self.get_parameter("side_grasp_qy").value),
                    side_grasp_qz=float(self.get_parameter("side_grasp_qz").value),
                    side_grasp_qw=float(self.get_parameter("side_grasp_qw").value),
                    side_approach_axis=str(self.get_parameter("side_approach_axis").value),
                ),
            )
        except ValueError as exc:
            self.get_logger().error(
                self._planning_log(
                    "fake_side_grasp_plan",
                    "failed",
                    error=str(exc),
                    real_readiness=False,
                )
            )
            return
        base_frame = str(self.get_parameter("base_frame").value).strip() or "base_link"
        self._approach_pose = self._pose_stamped(plan.approach_pose, base_frame)
        self._grasp_pose = self._pose_stamped(plan.grasp_pose, base_frame)
        self._lift_pose = self._pose_stamped(plan.lift_pose, base_frame)
        self.get_logger().warn(
            self._planning_log(
                "fake_side_grasp_plan",
                "ready",
                warning=plan.warning,
                real_readiness=False,
            )
        )

    def _configure_observe_pose_plan(self) -> None:
        if not bool(self.get_parameter("use_observe_pose_plan").value):
            return
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
            self.get_logger().error(
                self._planning_log(
                    "observe_pose_plan",
                    "failed",
                    error=str(exc),
                    real_readiness=False,
                )
            )
            return
        frame_id = (
            str(self.get_parameter("observe_frame").value).strip()
            or str(self.get_parameter("base_frame").value).strip()
            or "base_link"
        )
        self._observe_pose = self._pose_stamped(pose, frame_id)
        self.get_logger().warn(
            self._planning_log(
                "OBSERVE_CUP_POSE",
                "ready",
                pose=self._pose_dict(self._observe_pose),
                real_readiness=False,
            )
        )

    def _on_approach_pose(self, msg: PoseStamped) -> None:
        self._approach_pose = msg

    def _on_grasp_pose(self, msg: PoseStamped) -> None:
        self._grasp_pose = msg

    def _on_lift_pose(self, msg: PoseStamped) -> None:
        self._lift_pose = msg

    def _report_planning_only_readiness(self) -> None:
        if self._planning_attempted:
            return
        if not bool(self.get_parameter("enable_planning_only").value):
            self._planning_attempted = True
            self.get_logger().warn("Planning-only checker disabled; no motion is commanded")
            return
        if bool(self.get_parameter("allow_execute").value):
            self._planning_attempted = True
            self.get_logger().error(
                self._planning_log(
                    "planning_only_gate",
                    "failed",
                    error=(
                        "allow_execute=true is refused; no MoveIt execution or Doosan "
                        "command is available here"
                    ),
                    real_readiness=False,
                )
            )
            return

        planning_group = str(self.get_parameter("planning_group").value).strip()
        ee_link = str(self.get_parameter("ee_link").value).strip()
        base_frame = str(self.get_parameter("base_frame").value).strip()
        if not planning_group or not ee_link:
            self._planning_attempted = True
            self.get_logger().error(
                self._planning_log(
                    "planning_only_gate",
                    "failed",
                    error="planning_group and ee_link must be set before planning",
                    planning_group=planning_group,
                    ee_link=ee_link,
                    real_readiness=False,
                )
            )
            return
        if not base_frame:
            self._planning_attempted = True
            self.get_logger().error("Planning-only fail-closed: base_frame must be set")
            return

        moveit_ok, moveit_message = self._ensure_moveitpy(planning_group)
        if not moveit_ok:
            self._planning_attempted = True
            self.get_logger().error(
                self._planning_log(
                    "moveitpy_init",
                    "failed",
                    error=moveit_message,
                    hint=(
                        "source /opt/ros/humble/setup.bash; "
                        "source /home/ssu/Azas/install/setup.bash"
                    ),
                    real_readiness=False,
                )
            )
            return

        if self._observe_pose is not None:
            self._planning_attempted = True
            result = self._plan_pose("observe", self._observe_pose, planning_group, ee_link)
            self.get_logger().warn(
                self._planning_log(
                    "observe_pose_planning_only_summary",
                    "complete",
                    results=[result],
                    real_readiness=False,
                )
            )
            return

        if None in (self._approach_pose, self._grasp_pose, self._lift_pose):
            self.get_logger().warn(
                "Planning-only checker ready but waiting for approach/grasp/lift PoseStamped "
                "topics. No execution path exists in this node."
            )
            return

        self._planning_attempted = True
        results = [
            self._plan_pose("approach", self._approach_pose, planning_group, ee_link),
            self._plan_pose("grasp", self._grasp_pose, planning_group, ee_link),
            self._plan_pose("lift", self._lift_pose, planning_group, ee_link),
        ]
        self.get_logger().warn(
            self._planning_log(
                "side_grasp_planning_only_summary",
                "complete",
                results=results,
                real_readiness=False,
            )
        )

    def _ensure_moveitpy(self, planning_group: str) -> tuple[bool, str]:
        try:
            from moveit.planning import MoveItPy
        except Exception as exc:  # pragma: no cover - environment dependent
            return False, str(exc)
        if self._moveit_py is None:
            try:
                self._moveit_py = MoveItPy(
                    node_name="azas_side_grasp_planning_only",
                    provide_planning_service=False,
                )
            except Exception as exc:  # pragma: no cover - environment dependent
                return False, f"MoveItPy construction failed: {exc}"
        if self._planning_component is None:
            try:
                self._planning_component = self._moveit_py.get_planning_component(planning_group)
            except Exception as exc:  # pragma: no cover - environment dependent
                return False, f"PlanningComponent construction failed: {exc}"
        return True, "MoveItPy planning component available"

    def _plan_pose(
        self,
        label: str,
        pose_msg: PoseStamped,
        planning_group: str,
        ee_link: str,
    ) -> dict:
        try:
            from moveit.planning import PlanRequestParameters

            request_parameters = PlanRequestParameters(self._moveit_py)
            request_parameters.planning_time = float(
                self.get_parameter("planning_timeout_sec").value
            )
            self._planning_component.set_start_state_to_current_state()
            self._planning_component.set_goal_state(
                pose_stamped_msg=pose_msg,
                pose_link=ee_link,
            )
            solution = self._planning_component.plan(request_parameters)
            success = bool(solution)
            error_code = getattr(getattr(solution, "error_code", None), "val", None)
            result = {
                "label": label,
                "status": "planned" if success else "failed",
                "planning_group": planning_group,
                "ee_link": ee_link,
                "frame_id": pose_msg.header.frame_id,
                "error_code": error_code,
                "real_readiness": False,
            }
            log_fields = dict(result)
            log_status = str(log_fields.pop("status"))
            self.get_logger().warn(self._planning_log(f"{label}_pose", log_status, **log_fields))
            return result
        except Exception as exc:  # pragma: no cover - environment dependent
            result = {
                "label": label,
                "status": "failed",
                "planning_group": planning_group,
                "ee_link": ee_link,
                "frame_id": pose_msg.header.frame_id,
                "error": str(exc),
                "real_readiness": False,
            }
            log_fields = dict(result)
            log_fields.pop("status")
            self.get_logger().error(self._planning_log(f"{label}_pose", "failed", **log_fields))
            return result

    @staticmethod
    def _pose_stamped(pose: Pose, frame_id: str) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.pose = pose
        return msg

    @staticmethod
    def _pose_dict(pose_msg: PoseStamped) -> dict:
        pose = pose_msg.pose
        return {
            "frame_id": pose_msg.header.frame_id,
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": pose.orientation.x,
            "qy": pose.orientation.y,
            "qz": pose.orientation.z,
            "qw": pose.orientation.w,
        }

    @staticmethod
    def _planning_log(event: str, status: str, **fields) -> str:
        payload = {"event": event, "status": status, "planning_only": True}
        payload.update(fields)
        return json.dumps(payload, sort_keys=True)


def main(args=None):
    rclpy.init(args=args)
    node = AlignmentExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
