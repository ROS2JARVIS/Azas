from __future__ import annotations

import copy
import json
import math
import threading
import time

import rclpy
from azas_interfaces.srv import SetGripper
from azas_motion.lid_grip import LidGripConfig, compute_lid_grip_plan
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String

try:
    from dsr_msgs2.srv import (
        GetCurrentPosj,
        GetCurrentPosx,
        GetLastAlarm,
        Ikin,
        MoveJoint,
        MoveLine,
        MovePeriodic,
        ReleaseComplianceCtrl,
        ReleaseForce,
        SetDesiredForce,
        TaskComplianceCtrl,
    )
except ImportError:  # pragma: no cover - depends on the robot workspace overlay
    GetCurrentPosj = None
    GetCurrentPosx = None
    GetLastAlarm = None
    Ikin = None
    MoveJoint = None
    MoveLine = None
    MovePeriodic = None
    ReleaseComplianceCtrl = None
    ReleaseForce = None
    SetDesiredForce = None
    TaskComplianceCtrl = None


DR_BASE = 0
DR_TOOL = 1
MOVE_MODE_ABSOLUTE = 0
MOVE_MODE_RELATIVE = 1
SYNC = 0
BLENDING_SPEED_TYPE_DUPLICATE = 0
DR_FC_MOD_REL = 1
HARDWARE_CONFIRM_PHRASE = "ENABLE_REAL_ROBOT_MOTION"


def service_name(prefix: str, name: str) -> str:
    clean_prefix = prefix.strip("/")
    clean_name = name.strip("/")
    if not clean_prefix:
        return f"/{clean_name}"
    return f"/{clean_prefix}/{clean_name}"


class LidGripPlannerNode(Node):
    """Supervised lid grip pose generator.

    The node consumes a base_link lid pose from perception and publishes
    approach/grasp/lift PoseStamped candidates. Doosan motion and RG2 service
    calls are disabled by default and require explicit hardware gate
    parameters plus a p-key trigger from the preview node.
    """

    def __init__(self):
        super().__init__("lid_grip_planner_node")
        self.declare_parameter("lid_pose_topic", "/jarvis/lid_gripper/lid_pose")
        self.declare_parameter("trigger_topic", "/jarvis/lid_gripper/grip_request")
        self.declare_parameter("approach_pose_topic", "/jarvis/lid_gripper/approach_pose")
        self.declare_parameter("grasp_pose_topic", "/jarvis/lid_gripper/grasp_pose")
        self.declare_parameter("lift_pose_topic", "/jarvis/lid_gripper/lift_pose")
        self.declare_parameter("status_topic", "/jarvis/lid_gripper/status")
        self.declare_parameter("require_base_link_pose", True)
        self.declare_parameter("plan_on_pose", True)
        self.declare_parameter("log_pose_plans", False)
        self.declare_parameter("log_korean_status", True)
        self.declare_parameter("log_json_status", False)
        self.declare_parameter("approach_offset_m", 0.08)
        self.declare_parameter("lift_offset_m", 0.10)
        self.declare_parameter("surface_offset_m", 0.0)
        self.declare_parameter("offset_axis", "local_z")
        self.declare_parameter("tcp_grasp_offset_x_m", 0.0)
        self.declare_parameter("tcp_grasp_offset_y_m", 0.0)
        self.declare_parameter("tcp_grasp_offset_z_m", 0.0)
        self.declare_parameter("min_grasp_z_m", 0.02)
        self.declare_parameter("max_grasp_z_m", 0.60)
        self.declare_parameter("enable_hardware", False)
        self.declare_parameter("hardware_confirm", "")
        self.declare_parameter("allow_service_control_without_moveit", False)
        self.declare_parameter("service_prefix", "")
        self.declare_parameter("rx", 180.0)
        self.declare_parameter("ry", 0.0)
        self.declare_parameter("rz", 180.0)
        self.declare_parameter("use_lid_pose_yaw_for_pick", False)
        self.declare_parameter(
            "lid_pose_yaw_axis",
            "x",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("lid_pose_yaw_offset_deg", 0.0)
        self.declare_parameter("lid_pose_yaw_equivalence_deg", 180.0)
        self.declare_parameter("line_velocity", 15.0)
        self.declare_parameter("line_acceleration", 30.0)
        self.declare_parameter("move_timeout_sec", 10.0)
        self.declare_parameter("precheck_ikin", True)
        self.declare_parameter("ikin_sol_space", 2)
        self.declare_parameter("ikin_timeout_sec", 5.0)
        self.declare_parameter("verify_motion_reached", True)
        self.declare_parameter("motion_verify_timeout_sec", 20.0)
        self.declare_parameter("motion_target_tolerance_m", 0.015)
        self.declare_parameter("visual_refine_before_grasp", False)
        self.declare_parameter("visual_refine_sample_count", 10)
        self.declare_parameter("visual_refine_timeout_sec", 2.0)
        self.declare_parameter("visual_refine_min_sample_interval_sec", 0.03)
        self.declare_parameter("visual_refine_max_yaw_std_deg", 3.0)
        self.declare_parameter("visual_refine_max_position_std_m", 0.005)
        self.declare_parameter("visual_refine_apply_xy", True)
        self.declare_parameter("visual_refine_apply_yaw", True)
        self.declare_parameter("visual_refine_fallback_to_initial_plan", True)
        self.declare_parameter("settle_seconds_before_grasp", 0.5)
        self.declare_parameter("hold_seconds_after_grasp", 0.2)
        self.declare_parameter("enable_lid_twist_after_grasp", False)
        self.declare_parameter("lid_twist_target_x_m", math.nan)
        self.declare_parameter("lid_twist_target_y_m", math.nan)
        self.declare_parameter("lid_twist_target_z_m", math.nan)
        self.declare_parameter("lid_twist_rx", math.nan)
        self.declare_parameter("lid_twist_ry", math.nan)
        self.declare_parameter("lid_twist_rz", math.nan)
        self.declare_parameter("lid_twist_use_force_control", False)
        self.declare_parameter("lid_twist_use_force_spiral", False)
        self.declare_parameter("lid_twist_press_down_m", 0.0)
        self.declare_parameter("lid_twist_rz_delta_deg", -30.0)
        self.declare_parameter("lid_twist_turn_step_deg", 90.0)
        self.declare_parameter("lid_twist_transfer_clearance_m", 0.0)
        self.declare_parameter("lid_twist_release_lift_m", 0.03)
        self.declare_parameter("lid_twist_min_z_m", 0.02)
        self.declare_parameter("lid_twist_max_z_m", 0.60)
        self.declare_parameter("lid_twist_transfer_max_z_m", 0.60)
        self.declare_parameter("lid_twist_transfer_velocity", 30.0)
        self.declare_parameter("lid_twist_press_velocity", 8.0)
        self.declare_parameter("lid_twist_turn_velocity", 8.0)
        self.declare_parameter("lid_twist_acceleration", 10.0)
        self.declare_parameter("lid_twist_hold_seconds_before_turn", 0.3)
        self.declare_parameter("lid_twist_hold_seconds_after_turn", 0.5)
        self.declare_parameter("lid_twist_down_force_n", 8.0)
        self.declare_parameter("lid_twist_force_ref", "base")
        self.declare_parameter("lid_twist_force_rotation_mode", "movel")
        self.declare_parameter("lid_twist_force_service_timeout_sec", 5.0)
        self.declare_parameter("lid_twist_regrip_cycles", 1)
        self.declare_parameter("lid_twist_regrip_turn_deg", math.nan)
        self.declare_parameter("lid_twist_regrip_reset_between_cycles", False)
        self.declare_parameter("lid_twist_regrip_gripper_wait_sec", 0.5)
        self.declare_parameter("lid_twist_force_settle_seconds", 0.4)
        self.declare_parameter("lid_twist_force_release_time", 0.2)
        self.declare_parameter("lid_twist_preseat_periodic_before_turn", False)
        self.declare_parameter("lid_twist_preseat_periodic_x_amp_mm", 0.0)
        self.declare_parameter("lid_twist_preseat_periodic_y_amp_mm", 0.0)
        self.declare_parameter("lid_twist_preseat_periodic_z_amp_mm", 1.0)
        self.declare_parameter("lid_twist_preseat_periodic_rx_amp_deg", 0.0)
        self.declare_parameter("lid_twist_preseat_periodic_ry_amp_deg", 3.0)
        self.declare_parameter("lid_twist_preseat_periodic_rz_amp_deg", 5.0)
        self.declare_parameter("lid_twist_preseat_periodic_period_sec", 1.0)
        self.declare_parameter("lid_twist_preseat_periodic_acc_time_sec", 0.2)
        self.declare_parameter("lid_twist_preseat_periodic_repeat", 2)
        self.declare_parameter("lid_twist_preseat_periodic_ref", "tool")
        self.declare_parameter("lid_twist_compliance_x_stiffness", 3000.0)
        self.declare_parameter("lid_twist_compliance_y_stiffness", 3000.0)
        self.declare_parameter("lid_twist_compliance_z_stiffness", 300.0)
        self.declare_parameter("lid_twist_compliance_rx_stiffness", 200.0)
        self.declare_parameter("lid_twist_compliance_ry_stiffness", 200.0)
        self.declare_parameter("lid_twist_compliance_rz_stiffness", 200.0)
        self.declare_parameter("motion_target_orientation_tolerance_deg", 3.0)
        self.declare_parameter("enable_gripper_service_calls", False)
        self.declare_parameter("execute_gripper_on_pose", False)
        self.declare_parameter("gripper_set_service", "/jarvis/rg2/set_width")
        self.declare_parameter("gripper_preopen_width_m", math.nan)
        self.declare_parameter("gripper_grasp_width_m", math.nan)
        self.declare_parameter("gripper_force_n", math.nan)
        self.declare_parameter("gripper_wait_timeout_sec", 2.0)
        self.declare_parameter("continue_after_gripper_grasp_failure", False)
        self.declare_parameter("gripper_grasp_failure_wait_sec", 2.0)

        self._latest_pose: PoseStamped | None = None
        self._latest_pose_lock = threading.Lock()
        self._last_plan_stamp = 0.0
        self._sequence_lock = threading.Lock()
        self._approach_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("approach_pose_topic").value),
            10,
        )
        self._grasp_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("grasp_pose_topic").value),
            10,
        )
        self._lift_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("lift_pose_topic").value),
            10,
        )
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self._service_prefix = str(self.get_parameter("service_prefix").value)
        self._hardware_armed = self._is_hardware_armed()
        self._move_joint_client = None
        self._move_line_client = None
        self._move_periodic_client = None
        self._ikin_client = None
        self._current_posj_client = None
        self._current_posx_client = None
        self._last_alarm_client = None
        self._task_compliance_client = None
        self._set_desired_force_client = None
        self._release_force_client = None
        self._release_compliance_client = None
        self._move_joint_service_name = service_name(self._service_prefix, "motion/move_joint")
        self._move_line_service_name = service_name(self._service_prefix, "motion/move_line")
        self._move_periodic_service_name = service_name(self._service_prefix, "motion/move_periodic")
        self._ikin_service_name = service_name(self._service_prefix, "motion/ikin")
        self._current_posj_service_name = service_name(
            self._service_prefix,
            "aux_control/get_current_posj",
        )
        self._current_posx_service_name = service_name(
            self._service_prefix,
            "aux_control/get_current_posx",
        )
        self._last_alarm_service_name = service_name(
            self._service_prefix,
            "system/get_last_alarm",
        )
        self._task_compliance_service_name = service_name(
            self._service_prefix,
            "force/task_compliance_ctrl",
        )
        self._set_desired_force_service_name = service_name(
            self._service_prefix,
            "force/set_desired_force",
        )
        self._release_force_service_name = service_name(
            self._service_prefix,
            "force/release_force",
        )
        self._release_compliance_service_name = service_name(
            self._service_prefix,
            "force/release_compliance_ctrl",
        )
        if self._hardware_armed:
            if MoveLine is None:
                self.get_logger().error(
                    "dsr_msgs2 is unavailable; lid MoveLine execution is disabled"
                )
                self._hardware_armed = False
            else:
                if MoveJoint is not None:
                    self._move_joint_client = self.create_client(
                        MoveJoint,
                        self._move_joint_service_name,
                    )
                self._move_line_client = self.create_client(
                    MoveLine,
                    self._move_line_service_name,
                )
                if MovePeriodic is not None:
                    self._move_periodic_client = self.create_client(
                        MovePeriodic,
                        self._move_periodic_service_name,
                    )
                if Ikin is not None:
                    self._ikin_client = self.create_client(Ikin, self._ikin_service_name)
                if GetCurrentPosj is not None:
                    self._current_posj_client = self.create_client(
                        GetCurrentPosj,
                        self._current_posj_service_name,
                    )
                if GetCurrentPosx is not None:
                    self._current_posx_client = self.create_client(
                        GetCurrentPosx,
                        self._current_posx_service_name,
                    )
                if GetLastAlarm is not None:
                    self._last_alarm_client = self.create_client(
                        GetLastAlarm,
                        self._last_alarm_service_name,
                    )
                if TaskComplianceCtrl is not None:
                    self._task_compliance_client = self.create_client(
                        TaskComplianceCtrl,
                        self._task_compliance_service_name,
                    )
                if SetDesiredForce is not None:
                    self._set_desired_force_client = self.create_client(
                        SetDesiredForce,
                        self._set_desired_force_service_name,
                    )
                if ReleaseForce is not None:
                    self._release_force_client = self.create_client(
                        ReleaseForce,
                        self._release_force_service_name,
                    )
                if ReleaseComplianceCtrl is not None:
                    self._release_compliance_client = self.create_client(
                        ReleaseComplianceCtrl,
                        self._release_compliance_service_name,
                    )
        self._gripper_client = self.create_client(
            SetGripper,
            str(self.get_parameter("gripper_set_service").value),
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("lid_pose_topic").value),
            self._on_lid_pose,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("trigger_topic").value),
            self._on_grip_request,
            10,
        )
        self.get_logger().warn(
            "[뚜껑픽] 노드 준비 완료: p 입력이 들어오면 hardware gate 확인 후 "
            "approach -> grasp -> lift 순서로 실행합니다. "
            "enable_lid_twist_after_grasp=true이면 lift 이후 teach point에서 뚜껑 회전을 이어서 실행합니다."
        )

    def _on_lid_pose(self, msg: PoseStamped) -> None:
        with self._latest_pose_lock:
            self._latest_pose = copy.deepcopy(msg)
        if self._sequence_lock.locked():
            return
        if not bool(self.get_parameter("plan_on_pose").value):
            return
        self._plan_from_pose(
            msg,
            request_source="pose",
            allow_gripper=bool(self.get_parameter("execute_gripper_on_pose").value),
            allow_motion=False,
        )

    def _on_grip_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"accepted": True, "raw": msg.data}
        if payload.get("accepted") is False:
            self._publish_status(
                "failed",
                error="P_KEY_WITHOUT_VALID_LID_DETECTION",
                request=payload,
                real_motion=False,
            )
            return
        pose_snapshot = self._latest_pose_snapshot()
        if pose_snapshot is None:
            self._publish_status(
                "failed",
                error="NO_BASE_LINK_LID_POSE_FOR_TRIGGER",
                request=payload,
                real_motion=False,
            )
            return

        if not self._sequence_lock.acquire(blocking=False):
            self._publish_status(
                "trigger_ignored_motion_in_progress",
                request=payload,
                real_motion=self._hardware_armed,
            )
            return

        self._publish_status(
            "trigger_received",
            request=payload,
            hardware_armed=self._hardware_armed,
            real_motion=self._hardware_armed,
        )
        threading.Thread(
            target=self._run_trigger_sequence,
            args=(pose_snapshot,),
            daemon=True,
        ).start()

    def _run_trigger_sequence(self, pose_snapshot: PoseStamped) -> None:
        try:
            self._plan_from_pose(
                pose_snapshot,
                request_source="p_key",
                allow_gripper=bool(self.get_parameter("enable_gripper_service_calls").value),
                allow_motion=True,
            )
        except Exception as exc:  # pragma: no cover - final safety net for hardware path
            self._publish_status(
                "failed",
                error=f"trigger sequence exception: {exc}",
                real_motion=self._hardware_armed,
            )
        finally:
            self._sequence_lock.release()

    def _plan_from_pose(
        self,
        msg: PoseStamped,
        request_source: str,
        allow_gripper: bool,
        allow_motion: bool,
    ) -> None:
        if bool(self.get_parameter("require_base_link_pose").value) and msg.header.frame_id != "base_link":
            self._publish_status(
                "failed",
                error="LID_POSE_NOT_BASE_LINK",
                request_source=request_source,
                frame_id=msg.header.frame_id,
                real_motion=False,
            )
            return

        try:
            plan = compute_lid_grip_plan(msg.pose, self._config())
        except ValueError as exc:
            self._publish_status(
                "failed",
                error=str(exc),
                request_source=request_source,
                real_motion=False,
            )
            return

        self._approach_pub.publish(self._pose_stamped(plan.approach_pose, msg))
        self._grasp_pub.publish(self._pose_stamped(plan.grasp_pose, msg))
        self._lift_pub.publish(self._pose_stamped(plan.lift_pose, msg))
        self._last_plan_stamp = time.monotonic()
        self._publish_status(
            "planned",
            request_source=request_source,
            frame_id=msg.header.frame_id,
            approach_offset_m=plan.approach_offset_m,
            lift_offset_m=plan.lift_offset_m,
            approach=self._pose_xyz(plan.approach_pose),
            grasp=self._pose_xyz(plan.grasp_pose),
            lift=self._pose_xyz(plan.lift_pose),
            motion_allowed=bool(allow_motion),
            hardware_armed=self._hardware_armed,
            real_motion=False,
            **self._pick_rz_selection(plan.grasp_pose)[1],
        )

        if allow_motion:
            self._try_motion_sequence(msg, plan, allow_gripper=allow_gripper)
        elif allow_gripper:
            self._try_gripper_sequence()

    def _latest_pose_snapshot(self) -> PoseStamped | None:
        with self._latest_pose_lock:
            if self._latest_pose is None:
                return None
            return copy.deepcopy(self._latest_pose)

    def _is_hardware_armed(self) -> bool:
        return all(
            (
                bool(self.get_parameter("enable_hardware").value),
                str(self.get_parameter("hardware_confirm").value) == HARDWARE_CONFIRM_PHRASE,
                bool(self.get_parameter("allow_service_control_without_moveit").value),
            )
        )

    def _try_motion_sequence(self, source_msg: PoseStamped, plan, allow_gripper: bool) -> None:
        if not bool(self.get_parameter("enable_hardware").value):
            self._publish_status(
                "trigger_planned_no_motion",
                note="enable_hardware is false; p key produced plan only",
                real_motion=False,
            )
            return
        if not self._hardware_armed:
            self._publish_status(
                "failed",
                error=(
                    "hardware gates incomplete; require enable_hardware:=true, "
                    f"hardware_confirm:={HARDWARE_CONFIRM_PHRASE}, and "
                    "allow_service_control_without_moveit:=true"
                ),
                real_motion=False,
            )
            return
        if self._move_line_client is None:
            self._publish_status(
                "failed",
                error="MoveLine client unavailable",
                real_motion=False,
            )
            return
        timeout_sec = float(self.get_parameter("move_timeout_sec").value)
        if not self._move_line_client.wait_for_service(timeout_sec=max(timeout_sec, 0.0)):
            self._publish_status(
                "failed",
                error=f"MoveLine service unavailable: {self._move_line_service_name}",
                real_motion=False,
            )
            return
        if bool(self.get_parameter("verify_motion_reached").value):
            if self._current_posx_client is None:
                self._publish_status(
                    "failed",
                    error="GetCurrentPosx client unavailable; refusing motion without target verification",
                    real_motion=False,
                )
                return
            if not self._current_posx_client.wait_for_service(timeout_sec=max(timeout_sec, 0.0)):
                self._publish_status(
                    "failed",
                    error=f"GetCurrentPosx service unavailable: {self._current_posx_service_name}",
                    real_motion=False,
                )
                return
        gripper_targets = None
        if allow_gripper:
            gripper_targets = self._gripper_targets()
            if gripper_targets is None:
                return
            if not self._gripper_client.wait_for_service(
                timeout_sec=max(float(self.get_parameter("gripper_wait_timeout_sec").value), 0.0)
            ):
                self._publish_status(
                    "failed",
                    error=f"gripper service unavailable: {self.get_parameter('gripper_set_service').value}",
                    real_motion=False,
                )
                return

        use_visual_refine = bool(self.get_parameter("visual_refine_before_grasp").value)
        initial_steps = (("approach_lid", plan.approach_pose),)
        full_steps = (
            ("approach_lid", plan.approach_pose),
            ("grasp_lid", plan.grasp_pose),
            ("lift_lid", plan.lift_pose),
        )
        if not self._precheck_ikin_steps(initial_steps if use_visual_refine else full_steps):
            return
        if gripper_targets is not None:
            preopen_width, _grasp_width, force_n = gripper_targets
            if not self._call_gripper_sync("preopen", preopen_width, force_n):
                return

        if use_visual_refine:
            if not self._call_movel("approach_lid", plan.approach_pose):
                return
            refined_steps = self._try_visual_refine_steps(source_msg, plan)
            if refined_steps is None:
                return
            if not self._precheck_ikin_steps(refined_steps):
                return
            motion_steps = refined_steps
        else:
            motion_steps = full_steps

        for label, target in motion_steps:
            if not self._call_movel(label, target):
                return
            if label == "grasp_lid":
                if not self._finish_grasp_at_current_pose(gripper_targets):
                    return
        sequence_steps = [label for label, _target in motion_steps]
        if use_visual_refine:
            sequence_steps.insert(0, "approach_lid")
        if bool(self.get_parameter("enable_lid_twist_after_grasp").value):
            if not self._try_lid_twist_sequence():
                return
            sequence_steps.extend(
                [
                    "lid_twist_transfer_high",
                    "lid_twist_transfer",
                    "lid_twist_press",
                    "lid_twist_turn_clockwise_steps",
                    "lid_twist_final_preopen",
                    "lid_twist_home",
                ]
            )
        self._publish_status(
            "motion_sequence_requested",
            steps=sequence_steps,
            real_motion=True,
            note="Doosan MoveLine requests were sent through the configured service",
        )

    def _finish_grasp_at_current_pose(self, gripper_targets) -> bool:
        settle_seconds = max(float(self.get_parameter("settle_seconds_before_grasp").value), 0.0)
        if settle_seconds > 0.0:
            self._publish_status(
                "settling_before_grasp",
                seconds=round(settle_seconds, 3),
                real_motion=True,
            )
            time.sleep(settle_seconds)
        if gripper_targets is not None:
            _preopen_width, grasp_width, force_n = gripper_targets
            if not self._call_gripper_sync("grasp", grasp_width, force_n, publish_failure=False):
                if not bool(self.get_parameter("continue_after_gripper_grasp_failure").value):
                    self._publish_status(
                        "failed",
                        error="RG2 grasp request timed out or returned success=false",
                        real_motion=True,
                    )
                    return False
                fallback_wait = max(
                    float(self.get_parameter("gripper_grasp_failure_wait_sec").value),
                    0.0,
                )
                self._publish_status(
                    "gripper_grasp_continue_after_failure",
                    seconds=round(fallback_wait, 3),
                    real_motion=True,
                )
                time.sleep(fallback_wait)
        time.sleep(float(self.get_parameter("hold_seconds_after_grasp").value))
        return True

    def _try_visual_refine_steps(self, source_msg: PoseStamped, fallback_plan):
        sample_count = max(int(self.get_parameter("visual_refine_sample_count").value), 1)
        timeout_sec = max(float(self.get_parameter("visual_refine_timeout_sec").value), 0.1)
        min_interval_sec = max(
            float(self.get_parameter("visual_refine_min_sample_interval_sec").value),
            0.0,
        )
        axis = self._lid_pose_yaw_axis()
        self._publish_status(
            "visual_refine_collecting",
            sample_count=sample_count,
            timeout_sec=round(timeout_sec, 3),
            axis=axis,
            real_motion=True,
        )
        samples = self._collect_visual_refine_samples(
            sample_count=sample_count,
            timeout_sec=timeout_sec,
            min_interval_sec=min_interval_sec,
            axis=axis,
        )
        if len(samples) < sample_count:
            return self._visual_refine_fallback_steps(
                fallback_plan,
                reason="not_enough_samples",
                details={
                    "samples": len(samples),
                    "required_samples": sample_count,
                },
            )

        stats = self._visual_refine_stats(samples, axis=axis)
        max_position_std_m = max(
            float(self.get_parameter("visual_refine_max_position_std_m").value),
            0.0,
        )
        max_yaw_std_deg = max(
            float(self.get_parameter("visual_refine_max_yaw_std_deg").value),
            0.0,
        )
        apply_xy = bool(self.get_parameter("visual_refine_apply_xy").value)
        apply_yaw = bool(self.get_parameter("visual_refine_apply_yaw").value)
        if apply_xy and stats["position_std_m"] > max_position_std_m:
            return self._visual_refine_fallback_steps(
                fallback_plan,
                reason="position_unstable",
                details={
                    "samples": len(samples),
                    "position_std_m": round(stats["position_std_m"], 5),
                    "max_position_std_m": round(max_position_std_m, 5),
                },
            )
        if apply_yaw and stats["yaw_std_deg"] > max_yaw_std_deg:
            return self._visual_refine_fallback_steps(
                fallback_plan,
                reason="yaw_unstable",
                details={
                    "samples": len(samples),
                    "yaw_std_deg": round(stats["yaw_std_deg"], 3),
                    "max_yaw_std_deg": round(max_yaw_std_deg, 3),
                },
            )

        refined_msg = copy.deepcopy(source_msg)
        if apply_xy:
            refined_msg.pose.position.x = stats["mean_x"]
            refined_msg.pose.position.y = stats["mean_y"]
        if apply_yaw:
            refined_msg.pose.orientation = copy.deepcopy(samples[-1][0].pose.orientation)
        try:
            refined_plan = compute_lid_grip_plan(refined_msg.pose, self._config())
        except ValueError as exc:
            return self._visual_refine_fallback_steps(
                fallback_plan,
                reason="plan_failed",
                details={"error": str(exc)},
            )

        refined_rz = stats["pick_rz_deg"] if apply_yaw else self._pick_rz_deg(refined_msg.pose)
        align_pos = self._pose_to_dsr_pos_with_rz(refined_plan.approach_pose, refined_rz)
        grasp_pos = self._pose_to_dsr_pos_with_rz(refined_plan.grasp_pose, refined_rz)
        lift_pos = self._pose_to_dsr_pos_with_rz(refined_plan.lift_pose, refined_rz)
        self._approach_pub.publish(self._pose_stamped(refined_plan.approach_pose, refined_msg))
        self._grasp_pub.publish(self._pose_stamped(refined_plan.grasp_pose, refined_msg))
        self._lift_pub.publish(self._pose_stamped(refined_plan.lift_pose, refined_msg))
        self._publish_status(
            "visual_refine_result",
            samples=len(samples),
            position_std_m=round(stats["position_std_m"], 5),
            yaw_std_deg=round(stats["yaw_std_deg"], 3),
            mean_x=round(stats["mean_x"], 4),
            mean_y=round(stats["mean_y"], 4),
            mean_yaw_deg=round(stats["mean_yaw_deg"], 3),
            pick_rz_deg=round(refined_rz, 3),
            axis=axis,
            mean_yaw_x_deg=self._round_or_none(stats.get("mean_yaw_x_deg"), 3),
            mean_yaw_y_deg=self._round_or_none(stats.get("mean_yaw_y_deg"), 3),
            pick_rz_x_deg=self._round_or_none(stats.get("pick_rz_x_deg"), 3),
            pick_rz_y_deg=self._round_or_none(stats.get("pick_rz_y_deg"), 3),
            apply_xy=apply_xy,
            apply_yaw=apply_yaw,
            real_motion=True,
        )
        return (
            ("visual_refine_align_lid", align_pos),
            ("grasp_lid", grasp_pos),
            ("lift_lid", lift_pos),
        )

    def _visual_refine_fallback_steps(self, fallback_plan, *, reason: str, details: dict):
        if not bool(self.get_parameter("visual_refine_fallback_to_initial_plan").value):
            detail = f"VISUAL_REFINE_FAILED_WITHOUT_FALLBACK: {reason}"
            if details:
                detail += f" {details}"
            self._publish_status(
                "failed",
                error=detail,
                real_motion=False,
            )
            return None
        self._publish_status(
            "visual_refine_fallback",
            reason=reason,
            **details,
            real_motion=True,
        )
        return (
            ("grasp_lid", fallback_plan.grasp_pose),
            ("lift_lid", fallback_plan.lift_pose),
        )

    def _collect_visual_refine_samples(
        self,
        *,
        sample_count: int,
        timeout_sec: float,
        min_interval_sec: float,
        axis: str,
    ) -> list[tuple[PoseStamped, float]]:
        start_stamp_key = self._pose_stamp_key(self._latest_pose_snapshot())
        seen_stamp_keys = set()
        samples: list[tuple[PoseStamped, float]] = []
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline and len(samples) < sample_count:
            snapshot = self._latest_pose_snapshot()
            if snapshot is None:
                time.sleep(max(min_interval_sec, 0.01))
                continue
            stamp_key = self._pose_stamp_key(snapshot)
            if stamp_key is None or stamp_key == start_stamp_key or stamp_key in seen_stamp_keys:
                time.sleep(max(min_interval_sec, 0.01))
                continue
            if (
                bool(self.get_parameter("require_base_link_pose").value)
                and snapshot.header.frame_id != "base_link"
            ):
                seen_stamp_keys.add(stamp_key)
                time.sleep(max(min_interval_sec, 0.01))
                continue
            yaw = self._pose_axis_yaw_deg(snapshot.pose, axis)
            if yaw is None:
                seen_stamp_keys.add(stamp_key)
                time.sleep(max(min_interval_sec, 0.01))
                continue
            samples.append((snapshot, yaw))
            seen_stamp_keys.add(stamp_key)
            time.sleep(max(min_interval_sec, 0.01))
        return samples

    @staticmethod
    def _pose_stamp_key(msg: PoseStamped | None) -> tuple[int, int] | None:
        if msg is None:
            return None
        return (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))

    def _visual_refine_stats(self, samples: list[tuple[PoseStamped, float]], *, axis: str) -> dict:
        xs = [float(sample.pose.position.x) for sample, _yaw in samples]
        ys = [float(sample.pose.position.y) for sample, _yaw in samples]
        yaws = [float(yaw) for _sample, yaw in samples]
        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)
        position_std_m = math.sqrt(
            sum((x - mean_x) ** 2 + (y - mean_y) ** 2 for x, y in zip(xs, ys)) / len(xs)
        )
        period_deg = float(self.get_parameter("lid_pose_yaw_equivalence_deg").value)
        mean_yaw = self._mean_equivalent_angle_deg(yaws, period_deg=period_deg)
        yaw_std = math.sqrt(
            sum(
                self._equivalent_angle_delta_deg(yaw, mean_yaw, period_deg=period_deg) ** 2
                for yaw in yaws
            )
            / len(yaws)
        )
        offset_deg = float(self.get_parameter("lid_pose_yaw_offset_deg").value)
        fixed_rz = float(self.get_parameter("rz").value)
        requested_rz = mean_yaw + offset_deg
        pick_rz = self._nearest_equivalent_angle_deg(
            requested_rz,
            reference_deg=fixed_rz,
            period_deg=period_deg,
        )
        x_stats = self._axis_yaw_stats(samples, "x", period_deg, offset_deg, fixed_rz)
        y_stats = self._axis_yaw_stats(samples, "y", period_deg, offset_deg, fixed_rz)
        return {
            "mean_x": mean_x,
            "mean_y": mean_y,
            "position_std_m": position_std_m,
            "axis": axis,
            "mean_yaw_deg": mean_yaw,
            "yaw_std_deg": yaw_std,
            "pick_rz_deg": pick_rz,
            "mean_yaw_x_deg": x_stats["mean_yaw_deg"],
            "yaw_x_std_deg": x_stats["yaw_std_deg"],
            "pick_rz_x_deg": x_stats["pick_rz_deg"],
            "mean_yaw_y_deg": y_stats["mean_yaw_deg"],
            "yaw_y_std_deg": y_stats["yaw_std_deg"],
            "pick_rz_y_deg": y_stats["pick_rz_deg"],
        }

    def _axis_yaw_stats(
        self,
        samples: list[tuple[PoseStamped, float]],
        axis: str,
        period_deg: float,
        offset_deg: float,
        fixed_rz: float,
    ) -> dict:
        yaws = []
        for sample, _selected_yaw in samples:
            yaw = self._pose_axis_yaw_deg(sample.pose, axis)
            if yaw is not None:
                yaws.append(float(yaw))
        if not yaws:
            return {
                "mean_yaw_deg": None,
                "yaw_std_deg": None,
                "pick_rz_deg": None,
            }
        mean_yaw = self._mean_equivalent_angle_deg(yaws, period_deg=period_deg)
        yaw_std = math.sqrt(
            sum(
                self._equivalent_angle_delta_deg(yaw, mean_yaw, period_deg=period_deg) ** 2
                for yaw in yaws
            )
            / len(yaws)
        )
        pick_rz = self._nearest_equivalent_angle_deg(
            mean_yaw + offset_deg,
            reference_deg=fixed_rz,
            period_deg=period_deg,
        )
        return {
            "mean_yaw_deg": mean_yaw,
            "yaw_std_deg": yaw_std,
            "pick_rz_deg": pick_rz,
        }

    @staticmethod
    def _mean_equivalent_angle_deg(angles_deg: list[float], *, period_deg: float) -> float:
        if not angles_deg:
            return math.nan
        if not (math.isfinite(period_deg) and abs(period_deg) > 1e-9):
            period_deg = 360.0
        scale = 360.0 / abs(period_deg)
        sin_sum = 0.0
        cos_sum = 0.0
        for angle in angles_deg:
            radians = math.radians(float(angle) * scale)
            sin_sum += math.sin(radians)
            cos_sum += math.cos(radians)
        if abs(sin_sum) <= 1e-12 and abs(cos_sum) <= 1e-12:
            return float(angles_deg[-1])
        return math.degrees(math.atan2(sin_sum, cos_sum)) / scale

    @staticmethod
    def _equivalent_angle_delta_deg(angle_deg: float, reference_deg: float, *, period_deg: float) -> float:
        if not (math.isfinite(period_deg) and abs(period_deg) > 1e-9):
            period_deg = 360.0
        period = abs(period_deg)
        return (float(angle_deg) - float(reference_deg) + period / 2.0) % period - period / 2.0

    def _call_movel(self, label: str, pose) -> bool:
        if isinstance(pose, (list, tuple)):
            pos = [float(value) for value in pose]
        else:
            pos = self._pose_to_dsr_pos(pose)
        return self._call_movel_pos(
            label,
            pos,
            velocity=float(self.get_parameter("line_velocity").value),
            acceleration=float(self.get_parameter("line_acceleration").value),
            verify_orientation=False,
        )

    def _call_movel_pos(
        self,
        label: str,
        pos_mm_deg: list[float],
        *,
        velocity: float,
        acceleration: float,
        verify_orientation: bool,
    ) -> bool:
        req = MoveLine.Request()
        req.pos = [float(value) for value in pos_mm_deg]
        req.vel = [float(velocity)] * 2
        req.acc = [float(acceleration)] * 2
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        self.get_logger().warn(
            f"[뚜껑픽] 이동 시작: {self._step_ko(label)} "
            f"target=({req.pos[0] / 1000.0:.3f}, {req.pos[1] / 1000.0:.3f}, "
            f"{req.pos[2] / 1000.0:.3f}) m rpy=({req.pos[3]:.1f}, {req.pos[4]:.1f}, {req.pos[5]:.1f}) deg"
        )
        future = self._move_line_client.call_async(req)
        result = self._wait_for_future(future, label, float(self.get_parameter("move_timeout_sec").value))
        if result is None:
            self._publish_status(
                "failed",
                error=f"{label} timed out or failed",
                real_motion=True,
            )
            return False
        if not bool(result.success):
            self._publish_status(
                "failed",
                error=f"{label} returned success=false",
                real_motion=True,
            )
            return False
        if not self._verify_target_reached(label, req.pos, verify_orientation=verify_orientation):
            return False
        return True

    def _call_movel_relative_pos(
        self,
        label: str,
        delta_mm_deg: list[float],
        *,
        velocity: float,
        acceleration: float,
        ref: int,
    ) -> bool:
        req = MoveLine.Request()
        req.pos = [float(value) for value in delta_mm_deg]
        req.vel = [float(velocity)] * 2
        req.acc = [float(acceleration)] * 2
        req.time = 0.0
        req.radius = 0.0
        req.ref = int(ref)
        req.mode = MOVE_MODE_RELATIVE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        ref_label = "tool" if req.ref == DR_TOOL else "base"
        self.get_logger().warn(
            f"[뚜껑픽] 상대 이동 시작: {self._step_ko(label)} ref={ref_label} "
            f"delta_mm=({req.pos[0]:.1f}, {req.pos[1]:.1f}, {req.pos[2]:.1f}) "
            f"delta_rpy=({req.pos[3]:.1f}, {req.pos[4]:.1f}, {req.pos[5]:.1f}) deg"
        )
        future = self._move_line_client.call_async(req)
        result = self._wait_for_future(
            future,
            label,
            float(self.get_parameter("move_timeout_sec").value),
        )
        if result is None:
            self._publish_status(
                "failed",
                error=f"{label} timed out or failed",
                real_motion=True,
            )
            return False
        if not bool(result.success):
            self._publish_status(
                "failed",
                error=f"{label} returned success=false",
                real_motion=True,
            )
            return False
        return True

    def _call_movej_relative_pos(
        self,
        label: str,
        delta_deg: list[float],
        *,
        velocity: float,
        acceleration: float,
    ) -> bool:
        if self._move_joint_client is None:
            self._publish_status(
                "failed",
                error=f"MoveJoint client unavailable: {self._move_joint_service_name}",
                real_motion=False,
            )
            return False
        if not self._move_joint_client.wait_for_service(
            timeout_sec=max(float(self.get_parameter("move_timeout_sec").value), 0.0)
        ):
            self._publish_status(
                "failed",
                error=f"MoveJoint service unavailable: {self._move_joint_service_name}",
                real_motion=False,
            )
            return False

        before_joints = self._current_posj(timeout_sec=1.0)
        req = MoveJoint.Request()
        req.pos = [float(value) for value in delta_deg]
        req.vel = float(velocity)
        req.acc = float(acceleration)
        req.time = 0.0
        req.radius = 0.0
        req.mode = MOVE_MODE_RELATIVE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        self._publish_status(
            "lid_twist_joint_relative_start",
            step=label,
            delta_joints_deg=[round(value, 3) for value in req.pos],
            before_joints_deg=(
                [round(value, 3) for value in before_joints] if before_joints is not None else None
            ),
            velocity=round(float(velocity), 3),
            acceleration=round(float(acceleration), 3),
            real_motion=True,
        )
        future = self._move_joint_client.call_async(req)
        result = self._wait_for_future(
            future,
            label,
            float(self.get_parameter("move_timeout_sec").value),
        )
        if result is None:
            self._publish_status(
                "failed",
                error=f"{label} timed out or failed",
                real_motion=True,
            )
            return False
        if not bool(result.success):
            self._publish_status(
                "failed",
                error=f"{label} returned success=false",
                real_motion=True,
            )
            return False
        after_joints = self._current_posj(timeout_sec=1.0)
        self._publish_status(
            "lid_twist_joint_relative_done",
            step=label,
            delta_joints_deg=[round(value, 3) for value in req.pos],
            after_joints_deg=(
                [round(value, 3) for value in after_joints] if after_joints is not None else None
            ),
            real_motion=True,
        )
        return True

    def _pose_to_dsr_pos(self, pose) -> list[float]:
        return self._pose_to_dsr_pos_with_rz(pose, self._pick_rz_deg(pose))

    def _pose_to_dsr_pos_with_rz(self, pose, rz_deg: float) -> list[float]:
        return [
            float(pose.position.x) * 1000.0,
            float(pose.position.y) * 1000.0,
            float(pose.position.z) * 1000.0,
            float(self.get_parameter("rx").value),
            float(self.get_parameter("ry").value),
            float(rz_deg),
        ]

    def _pick_rz_deg(self, pose) -> float:
        return self._pick_rz_selection(pose)[0]

    def _pick_rz_selection(self, pose) -> tuple[float, dict]:
        fixed_rz = float(self.get_parameter("rz").value)
        yaw_x = self._pose_axis_yaw_deg(pose, "x")
        yaw_y = self._pose_axis_yaw_deg(pose, "y")
        fields = {
            "pick_yaw_enabled": False,
            "pick_rz_deg": round(fixed_rz, 3),
            "lid_pose_x_yaw_deg": self._round_or_none(yaw_x, 3),
            "lid_pose_y_yaw_deg": self._round_or_none(yaw_y, 3),
        }
        if not bool(self.get_parameter("use_lid_pose_yaw_for_pick").value):
            return fixed_rz, fields
        axis = self._lid_pose_yaw_axis()
        yaw = yaw_y if axis == "y" else yaw_x
        fields.update(
            {
                "pick_yaw_enabled": True,
                "pick_yaw_valid": False,
                "lid_pose_yaw_axis": axis,
                "fixed_rz_deg": round(fixed_rz, 3),
            }
        )
        if yaw is None:
            self.get_logger().warn(
                "[뚜껑픽] ArUco yaw 사용 요청이 있었지만 pose orientation이 유효하지 않아 "
                f"기존 rz={fixed_rz:.1f} deg를 사용합니다"
            )
            return fixed_rz, fields
        offset_deg = float(self.get_parameter("lid_pose_yaw_offset_deg").value)
        requested_rz = yaw + offset_deg
        equivalence_deg = float(self.get_parameter("lid_pose_yaw_equivalence_deg").value)
        selected_rz = self._nearest_equivalent_angle_deg(
            requested_rz,
            reference_deg=fixed_rz,
            period_deg=equivalence_deg,
        )
        fields.update(
            {
                "pick_yaw_valid": True,
                "lid_pose_yaw_deg": round(yaw, 3),
                "lid_pose_yaw_offset_deg": round(offset_deg, 3),
                "lid_pose_yaw_equivalence_deg": round(equivalence_deg, 3),
                "requested_pick_rz_deg": round(requested_rz, 3),
                "pick_rz_deg": round(selected_rz, 3),
            }
        )
        return selected_rz, fields

    @staticmethod
    def _nearest_equivalent_angle_deg(
        angle_deg: float,
        *,
        reference_deg: float,
        period_deg: float,
    ) -> float:
        if not (
            math.isfinite(angle_deg)
            and math.isfinite(reference_deg)
            and math.isfinite(period_deg)
            and abs(period_deg) > 1e-9
        ):
            return angle_deg
        period = abs(period_deg)
        turns = round((reference_deg - angle_deg) / period)
        return angle_deg + turns * period

    @staticmethod
    def _pose_axis_yaw_deg(pose, axis_name: str) -> float | None:
        qx = float(pose.orientation.x)
        qy = float(pose.orientation.y)
        qz = float(pose.orientation.z)
        qw = float(pose.orientation.w)
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not math.isfinite(norm) or norm <= 1e-12:
            return None
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        axis = str(axis_name).strip().lower()
        if axis == "y":
            vector_x = 2.0 * (qx * qy - qz * qw)
            vector_y = 1.0 - 2.0 * (qx * qx + qz * qz)
        else:
            vector_x = 1.0 - 2.0 * (qy * qy + qz * qz)
            vector_y = 2.0 * (qx * qy + qz * qw)
        if abs(vector_x) <= 1e-12 and abs(vector_y) <= 1e-12:
            return None
        return math.degrees(math.atan2(vector_y, vector_x))

    @staticmethod
    def _round_or_none(value, digits: int):
        if value is None:
            return None
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(numeric):
            return None
        return round(numeric, digits)

    def _lid_pose_yaw_axis(self) -> str:
        value = self.get_parameter("lid_pose_yaw_axis").value
        if isinstance(value, bool):
            return "y" if value else "x"
        axis = str(value).strip().lower()
        if axis in {"y", "axis_y", "marker_y", "aruco_y"}:
            return "y"
        if axis in {"x", "axis_x", "marker_x", "aruco_x"}:
            return "x"
        self.get_logger().warn(
            f"[뚜껑픽] lid_pose_yaw_axis='{value}' 값이 유효하지 않아 x축을 사용합니다"
        )
        return "x"

    def _precheck_ikin_steps(self, steps) -> bool:
        if not bool(self.get_parameter("precheck_ikin").value):
            return True
        if self._ikin_client is None:
            self._publish_status(
                "failed",
                error="Ikin client unavailable; refusing motion without IK precheck",
                real_motion=False,
            )
            return False

        timeout_sec = max(float(self.get_parameter("ikin_timeout_sec").value), 0.1)
        if not self._ikin_client.wait_for_service(timeout_sec=timeout_sec):
            self._publish_status(
                "failed",
                error=f"Ikin service unavailable: {self._ikin_service_name}",
                real_motion=False,
            )
            return False

        for label, pose in steps:
            req = Ikin.Request()
            if isinstance(pose, (list, tuple)):
                req.pos = [float(value) for value in pose]
            else:
                req.pos = self._pose_to_dsr_pos(pose)
            req.sol_space = int(self.get_parameter("ikin_sol_space").value)
            req.ref = DR_BASE
            future = self._ikin_client.call_async(req)
            result = self._wait_for_future(future, f"{label}_ikin", timeout_sec)
            if result is None or not bool(result.success):
                self._publish_status(
                    "failed",
                    error=f"{label} Ikin precheck failed",
                    target_mm_deg=[round(value, 3) for value in req.pos],
                    real_motion=False,
                )
                return False
            joints_deg = [float(value) for value in result.conv_posj]
            self._publish_status(
                "ikin_result",
                step=label,
                sol_space=req.sol_space,
                joints_deg=[round(value, 3) for value in joints_deg],
                target_mm_deg=[round(value, 3) for value in req.pos],
                real_motion=True,
            )
        return True

    def _verify_target_reached(
        self,
        label: str,
        target_pos_mm_deg: list[float],
        *,
        verify_orientation: bool = False,
    ) -> bool:
        if not bool(self.get_parameter("verify_motion_reached").value):
            return True
        tolerance_m = max(float(self.get_parameter("motion_target_tolerance_m").value), 0.001)
        orientation_tolerance_deg = max(
            float(self.get_parameter("motion_target_orientation_tolerance_deg").value),
            0.1,
        )
        timeout_sec = max(float(self.get_parameter("motion_verify_timeout_sec").value), 0.1)
        deadline = time.monotonic() + timeout_sec
        last_actual = None
        last_distance_m = math.inf
        last_orientation_error_deg = math.inf
        while rclpy.ok() and time.monotonic() < deadline:
            actual = self._current_posx(timeout_sec=2.0)
            if actual is not None:
                last_actual = actual
                last_distance_m = math.sqrt(
                    sum((actual[index] - target_pos_mm_deg[index]) ** 2 for index in range(3))
                ) / 1000.0
                if verify_orientation:
                    last_orientation_error_deg = max(
                        abs(self._angle_delta_deg(actual[index], target_pos_mm_deg[index]))
                        for index in range(3, 6)
                    )
                orientation_ok = (
                    not verify_orientation
                    or last_orientation_error_deg <= orientation_tolerance_deg
                )
                if last_distance_m <= tolerance_m and orientation_ok:
                    actual_rpy = (
                        [round(float(value), 3) for value in actual[3:6]]
                        if len(actual) >= 6
                        else None
                    )
                    target_rpy = [round(float(value), 3) for value in target_pos_mm_deg[3:6]]
                    rpy_error = (
                        [
                            round(
                                self._angle_delta_deg(actual[index], target_pos_mm_deg[index]),
                                3,
                            )
                            for index in range(3, 6)
                        ]
                        if len(actual) >= 6
                        else None
                    )
                    self._publish_status(
                        "motion_target_reached",
                        step=label,
                        distance_m=round(last_distance_m, 4),
                        tolerance_m=round(tolerance_m, 4),
                        target_rpy_deg=target_rpy,
                        actual_rpy_deg=actual_rpy,
                        rpy_error_deg=rpy_error,
                        orientation_error_deg=(
                            round(last_orientation_error_deg, 3)
                            if verify_orientation and math.isfinite(last_orientation_error_deg)
                            else None
                        ),
                        orientation_tolerance_deg=(
                            round(orientation_tolerance_deg, 3)
                            if verify_orientation
                            else None
                        ),
                        real_motion=True,
                    )
                    return True
            time.sleep(0.2)

        self._publish_status(
            "failed",
            error=f"{label} target was not reached after MoveLine accepted",
            target_mm=[round(value, 3) for value in target_pos_mm_deg[:3]],
            actual_mm=(
                [round(value, 3) for value in last_actual[:3]]
                if last_actual is not None
                else None
            ),
            target_rpy_deg=[round(float(value), 3) for value in target_pos_mm_deg[3:6]],
            actual_rpy_deg=(
                [round(float(value), 3) for value in last_actual[3:6]]
                if last_actual is not None and len(last_actual) >= 6
                else None
            ),
            rpy_error_deg=(
                [
                    round(self._angle_delta_deg(last_actual[index], target_pos_mm_deg[index]), 3)
                    for index in range(3, 6)
                ]
                if last_actual is not None and len(last_actual) >= 6
                else None
            ),
            distance_m=round(last_distance_m, 4) if math.isfinite(last_distance_m) else None,
            tolerance_m=round(tolerance_m, 4),
            orientation_error_deg=(
                round(last_orientation_error_deg, 3)
                if verify_orientation and math.isfinite(last_orientation_error_deg)
                else None
            ),
            orientation_tolerance_deg=(
                round(orientation_tolerance_deg, 3)
                if verify_orientation
                else None
            ),
            last_alarm=self._read_last_alarm(),
            real_motion=True,
        )
        return False

    def _try_lid_twist_sequence(self) -> bool:
        if self._lid_twist_force_control_enabled():
            return self._try_lid_twist_force_sequence()
        if self._preseat_periodic_enabled() and self._lid_twist_force_rotation_mode() == "j6":
            return self._try_lid_twist_periodic_j6_sequence()

        twist_steps = self._lid_twist_steps()
        if twist_steps is None:
            return False
        ikin_steps = [(label, pos) for label, pos, _velocity, _verify_orientation, _hold in twist_steps]
        if not self._precheck_ikin_steps(ikin_steps):
            return False

        acceleration = float(self.get_parameter("lid_twist_acceleration").value)
        for label, pos, velocity, verify_orientation, hold_seconds in twist_steps:
            if not self._call_movel_pos(
                label,
                pos,
                velocity=velocity,
                acceleration=acceleration,
                verify_orientation=verify_orientation,
            ):
                return False
            if hold_seconds > 0.0:
                self._publish_status(
                    "lid_twist_holding",
                    step=label,
                    seconds=round(hold_seconds, 3),
                    real_motion=True,
                )
                time.sleep(hold_seconds)
        return True

    def _try_lid_twist_periodic_j6_sequence(self) -> bool:
        plan = self._lid_twist_force_plan(force_control=False)
        if plan is None:
            return False
        transfer_steps, turn_steps, release_step = plan
        ikin_steps = [(label, pos) for label, pos, _velocity, _verify_orientation, _hold in transfer_steps]
        if release_step[0] == "motion":
            ikin_steps.append((release_step[1], release_step[2]))
        if not self._precheck_ikin_steps(ikin_steps):
            return False
        required_clients = [
            (self._move_periodic_client, self._move_periodic_service_name),
            (self._move_joint_client, self._move_joint_service_name),
            (self._current_posj_client, self._current_posj_service_name),
        ]
        for client, name in required_clients:
            if client is None:
                self._publish_status(
                    "failed",
                    error=f"{name} client unavailable; dsr_msgs2 service type may be missing",
                    real_motion=False,
                )
                return False
        if not self._move_periodic_client.wait_for_service(
            timeout_sec=max(float(self.get_parameter("move_timeout_sec").value), 0.0)
        ):
            self._publish_status(
                "failed",
                error=f"MovePeriodic service unavailable: {self._move_periodic_service_name}",
                real_motion=False,
            )
            return False
        if not self._move_joint_client.wait_for_service(
            timeout_sec=max(float(self.get_parameter("move_timeout_sec").value), 0.0)
        ):
            self._publish_status(
                "failed",
                error=f"MoveJoint service unavailable: {self._move_joint_service_name}",
                real_motion=False,
            )
            return False

        acceleration = float(self.get_parameter("lid_twist_acceleration").value)
        for label, pos, velocity, verify_orientation, hold_seconds in transfer_steps:
            if not self._call_movel_pos(
                label,
                pos,
                velocity=velocity,
                acceleration=acceleration,
                verify_orientation=verify_orientation,
            ):
                return False
            if hold_seconds > 0.0:
                self._publish_status(
                    "lid_twist_holding",
                    step=label,
                    seconds=round(hold_seconds, 3),
                    real_motion=True,
                )
                time.sleep(hold_seconds)

        hold_before_turn = max(
            float(self.get_parameter("lid_twist_hold_seconds_before_turn").value),
            0.0,
        )
        if hold_before_turn > 0.0:
            self._publish_status(
                "lid_twist_holding",
                step="lid_twist_periodic_settle",
                seconds=round(hold_before_turn, 3),
                real_motion=True,
            )
            time.sleep(hold_before_turn)
        if not self._call_lid_twist_preseat_periodic():
            self._recover_lid_twist_abort(release_step, acceleration)
            return False
        for action in turn_steps:
            if not self._execute_lid_twist_action(action, acceleration):
                self._recover_lid_twist_abort(release_step, acceleration)
                return False
        hold_after_turn = max(
            float(self.get_parameter("lid_twist_hold_seconds_after_turn").value),
            0.0,
        )
        if hold_after_turn > 0.0:
            self._publish_status(
                "lid_twist_holding",
                step="lid_twist_turn_complete",
                seconds=round(hold_after_turn, 3),
                real_motion=True,
            )
            time.sleep(hold_after_turn)
        if not self._open_gripper_after_lid_twist():
            return False
        if not self._execute_lid_twist_action(release_step, acceleration):
            return False
        return True

    def _try_lid_twist_force_sequence(self) -> bool:
        plan = self._lid_twist_force_plan()
        if plan is None:
            return False
        transfer_steps, turn_steps, release_step = plan
        ikin_steps = [(label, pos) for label, pos, _velocity, _verify_orientation, _hold in transfer_steps]
        ikin_steps.extend(
            (action[1], action[2])
            for action in turn_steps
            if action[0] == "motion"
        )
        if release_step[0] == "motion":
            ikin_steps.append((release_step[1], release_step[2]))
        if not self._precheck_ikin_steps(ikin_steps):
            return False

        acceleration = float(self.get_parameter("lid_twist_acceleration").value)
        for label, pos, velocity, verify_orientation, hold_seconds in transfer_steps:
            if not self._call_movel_pos(
                label,
                pos,
                velocity=velocity,
                acceleration=acceleration,
                verify_orientation=verify_orientation,
            ):
                return False
            if hold_seconds > 0.0:
                self._publish_status(
                    "lid_twist_holding",
                    step=label,
                    seconds=round(hold_seconds, 3),
                    real_motion=True,
                )
                time.sleep(hold_seconds)

        force_enabled = False
        force_released = True
        try:
            if not self._enable_lid_twist_force():
                self._recover_lid_twist_abort(release_step, acceleration)
                return False
            force_enabled = True
            hold_before_turn = max(
                float(self.get_parameter("lid_twist_hold_seconds_before_turn").value),
                0.0,
            )
            if hold_before_turn > 0.0:
                self._publish_status(
                    "lid_twist_holding",
                    step="lid_twist_force_settle",
                    seconds=round(hold_before_turn, 3),
                    real_motion=True,
                )
                time.sleep(hold_before_turn)
            if self._preseat_periodic_enabled():
                if not self._call_lid_twist_preseat_periodic():
                    self._recover_lid_twist_abort(release_step, acceleration)
                    return False
            for action in turn_steps:
                if not self._execute_lid_twist_action(action, acceleration):
                    self._recover_lid_twist_abort(release_step, acceleration)
                    return False
            hold_after_turn = max(
                float(self.get_parameter("lid_twist_hold_seconds_after_turn").value),
                0.0,
            )
            if hold_after_turn > 0.0:
                self._publish_status(
                    "lid_twist_holding",
                    step="lid_twist_turn_complete",
                    seconds=round(hold_after_turn, 3),
                    real_motion=True,
                )
                time.sleep(hold_after_turn)
        finally:
            if force_enabled:
                force_released = self._release_lid_twist_force()
        if not force_released:
            self._publish_status(
                "failed",
                error="failed to release force/compliance after lid twist; refusing release motion",
                real_motion=True,
            )
            return False

        if not self._open_gripper_after_lid_twist():
            return False

        if not self._execute_lid_twist_action(release_step, acceleration):
            return False
        return True

    def _execute_lid_twist_action(self, action, acceleration: float) -> bool:
        kind = action[0]
        if kind == "gripper":
            if not self._call_lid_twist_gripper_action(action[1]):
                return False
            hold_seconds = float(action[2])
            if hold_seconds > 0.0:
                self._publish_status(
                    "lid_twist_holding",
                    step=f"lid_twist_regrip_{action[1]}",
                    seconds=round(hold_seconds, 3),
                    real_motion=True,
                )
                time.sleep(hold_seconds)
            return True
        if kind == "motion":
            _kind, label, pos, velocity, verify_orientation, hold_seconds = action
            if not self._call_movel_pos(
                label,
                pos,
                velocity=velocity,
                acceleration=acceleration,
                verify_orientation=verify_orientation,
            ):
                return False
        elif kind == "relative_motion":
            _kind, label, delta, velocity, ref, hold_seconds = action
            if not self._call_movel_relative_pos(
                label,
                delta,
                velocity=velocity,
                acceleration=acceleration,
                ref=ref,
            ):
                return False
        elif kind == "relative_joint":
            _kind, label, delta, velocity, hold_seconds = action
            if not self._call_movej_relative_pos(
                label,
                delta,
                velocity=velocity,
                acceleration=acceleration,
            ):
                return False
        else:
            self._publish_status(
                "failed",
                error=f"unsupported lid twist action kind: {kind}",
                real_motion=False,
            )
            return False

        if hold_seconds > 0.0:
            self._publish_status(
                "lid_twist_holding",
                step=label,
                seconds=round(hold_seconds, 3),
                real_motion=True,
            )
            time.sleep(hold_seconds)
        return True

    def _recover_lid_twist_abort(self, release_step, acceleration: float) -> None:
        self._publish_status(
            "lid_twist_abort_recovery",
            real_motion=True,
        )
        self._release_lid_twist_force()
        self._execute_lid_twist_action(release_step, acceleration)

    def _lid_twist_force_plan(self, *, force_control: bool = True):
        target = self._lid_twist_target_pos()
        if target is None:
            return None
        rz_delta_deg = float(self.get_parameter("lid_twist_rz_delta_deg").value)
        turn_step_deg = abs(float(self.get_parameter("lid_twist_turn_step_deg").value))
        transfer_clearance_m = max(
            float(self.get_parameter("lid_twist_transfer_clearance_m").value),
            0.0,
        )
        release_lift_m = max(float(self.get_parameter("lid_twist_release_lift_m").value), 0.0)
        min_z_m = float(self.get_parameter("lid_twist_min_z_m").value)
        max_z_m = float(self.get_parameter("lid_twist_max_z_m").value)
        transfer_max_z_m = float(self.get_parameter("lid_twist_transfer_max_z_m").value)
        if not math.isfinite(rz_delta_deg) or abs(rz_delta_deg) <= 1e-9:
            self._publish_status(
                "failed",
                error="lid_twist_rz_delta_deg must be finite and non-zero for force lid twist",
                real_motion=False,
            )
            return None
        if not math.isfinite(turn_step_deg) or turn_step_deg <= 0.0:
            self._publish_status(
                "failed",
                error="lid_twist_turn_step_deg must be positive",
                real_motion=False,
            )
            return None

        target_high = target.copy()
        target_high[2] += transfer_clearance_m * 1000.0
        turn_velocity = float(self.get_parameter("lid_twist_turn_velocity").value)
        hold_after_turn = max(
            float(self.get_parameter("lid_twist_hold_seconds_after_turn").value),
            0.0,
        )
        turn_action_plan = self._lid_twist_force_turn_actions(
            target,
            rz_delta_deg,
            turn_step_deg,
            turn_velocity,
            hold_after_turn,
        )
        if turn_action_plan is None:
            return None
        turn_steps, turned, closed_turn_deg = turn_action_plan
        turn_motion_steps = [
            action
            for action in turn_steps
            if action[0] in ("motion", "relative_motion")
        ]
        absolute_turn_motion_steps = [action for action in turn_steps if action[0] == "motion"]
        release = turned.copy()
        release_delta_z = release_lift_m * 1000.0
        if transfer_clearance_m > 0.0:
            release[2] = target_high[2]
            release_delta_z = target_high[2] - target[2]
        else:
            release[2] += release_lift_m * 1000.0
        validation_steps = []
        if transfer_clearance_m > 0.0:
            validation_steps.append(("lid_twist_transfer_high", target_high, transfer_max_z_m))
        validation_steps.extend([
            ("lid_twist_transfer", target, max_z_m),
            ("lid_twist_home", release, transfer_max_z_m if transfer_clearance_m > 0.0 else max_z_m),
        ])
        validation_steps.extend(
            (label, pos, max_z_m)
            for _kind, label, pos, _velocity, _verify_orientation, _hold in absolute_turn_motion_steps
        )
        for label, pos, upper_z_m in validation_steps:
            z_m = pos[2] / 1000.0
            if z_m < min_z_m or z_m > upper_z_m:
                self._publish_status(
                    "failed",
                    error=(
                        f"{label} z={z_m:.3f} outside "
                        f"[{min_z_m:.3f}, {upper_z_m:.3f}]"
                    ),
                    target_mm=[round(value, 3) for value in pos[:3]],
                    real_motion=False,
                )
                return None

        transfer_velocity = float(self.get_parameter("lid_twist_transfer_velocity").value)
        press_velocity = float(self.get_parameter("lid_twist_press_velocity").value)
        revolution = abs(closed_turn_deg) / 360.0
        self._publish_status(
            "lid_twist_planned",
            target_mm_deg=[round(value, 3) for value in target],
            target_high_mm_deg=[round(value, 3) for value in target_high],
            pressed_mm_deg=[round(value, 3) for value in target],
            turned_mm_deg=[round(value, 3) for value in turned],
            press_down_m=0.0,
            rz_delta_deg=round(rz_delta_deg, 3),
            turn_step_deg=round(turn_step_deg, 3),
            turn_count=len(turn_motion_steps) if turn_motion_steps else 1,
            transfer_clearance_m=round(transfer_clearance_m, 4),
            force_control=bool(force_control),
            force_rotation_mode=self._lid_twist_force_rotation_mode(),
            revolution=round(revolution, 3),
            regrip_cycles=1,
            post_turn_gripper_open=True,
            down_force_n=round(float(self.get_parameter("lid_twist_down_force_n").value), 3),
            preseat_periodic=bool(self._preseat_periodic_enabled()),
            preseat_periodic_amp=self._preseat_periodic_amp(),
            preseat_periodic_period_sec=round(
                float(self.get_parameter("lid_twist_preseat_periodic_period_sec").value),
                3,
            ),
            preseat_periodic_repeat=int(self.get_parameter("lid_twist_preseat_periodic_repeat").value),
            real_motion=True,
        )

        transfer_steps = []
        if transfer_clearance_m > 0.0:
            transfer_steps.append(("lid_twist_transfer_high", target_high, transfer_velocity, True, 0.0))
        transfer_steps.append(("lid_twist_transfer", target, press_velocity, True, 0.0))
        release_step = (
            "relative_motion",
            "lid_twist_home",
            [0.0, 0.0, release_delta_z, 0.0, 0.0, 0.0],
            transfer_velocity,
            DR_BASE,
            0.0,
        )
        return transfer_steps, turn_steps, release_step

    def _lid_twist_force_turn_actions(
        self,
        target: list[float],
        rz_delta_deg: float,
        turn_step_deg: float,
        turn_velocity: float,
        hold_after_turn: float,
    ):
        rotation_mode = self._lid_twist_force_rotation_mode()
        turn_deltas = self._lid_twist_turn_delta_steps(rz_delta_deg, turn_step_deg)
        if rotation_mode == "j6":
            actions = [
                (
                    "relative_joint",
                    self._lid_twist_turn_label(index, len(turn_deltas), rz_delta_deg),
                    [0.0, 0.0, 0.0, 0.0, 0.0, delta],
                    turn_velocity,
                    hold_after_turn if index == len(turn_deltas) else 0.0,
                )
                for index, delta in enumerate(turn_deltas, start=1)
            ]
        else:
            actions = [
                (
                    "relative_motion",
                    self._lid_twist_turn_label(index, len(turn_deltas), rz_delta_deg),
                    [0.0, 0.0, 0.0, 0.0, 0.0, delta],
                    turn_velocity,
                    DR_TOOL,
                    hold_after_turn if index == len(turn_deltas) else 0.0,
                )
                for index, delta in enumerate(turn_deltas, start=1)
            ]
        turned = target.copy()
        turned[5] += sum(turn_deltas)
        return actions, turned, sum(turn_deltas)

    def _enable_lid_twist_force(self) -> bool:
        if not self._force_twist_clients_available():
            return False
        stiffness = self._lid_twist_compliance_stiffness()
        force_ref = self._lid_twist_force_ref()
        down_force_n = max(float(self.get_parameter("lid_twist_down_force_n").value), 0.0)
        if down_force_n <= 0.0:
            self._publish_status(
                "failed",
                error="lid_twist_down_force_n must be positive for force lid twist",
                real_motion=False,
            )
            return False

        task_req = TaskComplianceCtrl.Request()
        task_req.stx = stiffness
        task_req.ref = force_ref
        task_req.time = min(
            max(float(self.get_parameter("lid_twist_force_settle_seconds").value), 0.0),
            1.0,
        )
        force_timeout_sec = self._lid_twist_force_timeout_sec()
        if not self._call_bool_service(
            self._task_compliance_client,
            task_req,
            "lid_twist_task_compliance",
            self._task_compliance_service_name,
            force_timeout_sec,
        ):
            return False

        force_req = SetDesiredForce.Request()
        force_req.fd = [0.0, 0.0, -down_force_n, 0.0, 0.0, 0.0]
        force_req.dir = [0, 0, 1, 0, 0, 0]
        force_req.ref = force_ref
        force_req.time = min(
            max(float(self.get_parameter("lid_twist_force_settle_seconds").value), 0.0),
            1.0,
        )
        force_req.mod = DR_FC_MOD_REL
        if not self._call_bool_service(
            self._set_desired_force_client,
            force_req,
            "lid_twist_set_force",
            self._set_desired_force_service_name,
            force_timeout_sec,
        ):
            self._release_lid_twist_force()
            return False

        self._publish_status(
            "lid_twist_force_enabled",
            down_force_n=round(down_force_n, 3),
            ref="base" if force_ref == DR_BASE else "tool",
            stiffness=[round(value, 3) for value in stiffness],
            real_motion=True,
        )
        return True

    def _call_lid_twist_preseat_periodic(self) -> bool:
        if self._move_periodic_client is None:
            self._publish_status(
                "failed",
                error="MovePeriodic client unavailable; cannot execute pre-seat periodic motion",
                real_motion=False,
            )
            return False
        if not self._move_periodic_client.wait_for_service(
            timeout_sec=max(float(self.get_parameter("move_timeout_sec").value), 0.0)
        ):
            self._publish_status(
                "failed",
                error=f"MovePeriodic service unavailable: {self._move_periodic_service_name}",
                real_motion=False,
            )
            return False

        period_sec = max(
            float(self.get_parameter("lid_twist_preseat_periodic_period_sec").value),
            0.01,
        )
        acc_time_sec = max(
            float(self.get_parameter("lid_twist_preseat_periodic_acc_time_sec").value),
            0.0,
        )
        repeat = max(int(self.get_parameter("lid_twist_preseat_periodic_repeat").value), 1)
        amp = self._preseat_periodic_amp()
        periodic = [period_sec] * 6

        req = MovePeriodic.Request()
        req.amp = amp
        req.periodic = periodic
        req.acc = acc_time_sec
        req.repeat = repeat
        req.ref = self._preseat_periodic_ref()
        req.sync_type = SYNC
        self._publish_status(
            "lid_twist_preseat_periodic_start",
            amp=[round(value, 3) for value in amp],
            period_sec=round(period_sec, 3),
            acc_time_sec=round(acc_time_sec, 3),
            repeat=repeat,
            ref="tool" if req.ref == DR_TOOL else "base",
            real_motion=True,
        )
        return self._call_bool_service(
            self._move_periodic_client,
            req,
            "lid_twist_preseat_periodic",
            self._move_periodic_service_name,
            float(self.get_parameter("move_timeout_sec").value),
        )

    def _preseat_periodic_enabled(self) -> bool:
        return bool(self.get_parameter("lid_twist_preseat_periodic_before_turn").value)

    def _preseat_periodic_amp(self) -> list[float]:
        return [
            float(self.get_parameter("lid_twist_preseat_periodic_x_amp_mm").value),
            float(self.get_parameter("lid_twist_preseat_periodic_y_amp_mm").value),
            float(self.get_parameter("lid_twist_preseat_periodic_z_amp_mm").value),
            float(self.get_parameter("lid_twist_preseat_periodic_rx_amp_deg").value),
            float(self.get_parameter("lid_twist_preseat_periodic_ry_amp_deg").value),
            float(self.get_parameter("lid_twist_preseat_periodic_rz_amp_deg").value),
        ]

    def _preseat_periodic_ref(self) -> int:
        value = str(self.get_parameter("lid_twist_preseat_periodic_ref").value).strip().lower()
        if value == "base":
            return DR_BASE
        return DR_TOOL

    def _release_lid_twist_force(self) -> bool:
        ok = True
        force_timeout_sec = self._lid_twist_force_timeout_sec()
        release_time = min(
            max(float(self.get_parameter("lid_twist_force_release_time").value), 0.0),
            1.0,
        )
        if self._release_force_client is not None:
            req = ReleaseForce.Request()
            req.time = release_time
            ok = self._call_bool_service(
                self._release_force_client,
                req,
                "lid_twist_release_force",
                self._release_force_service_name,
                force_timeout_sec,
                publish_failure=False,
            ) and ok
        if self._release_compliance_client is not None:
            ok = self._call_bool_service(
                self._release_compliance_client,
                ReleaseComplianceCtrl.Request(),
                "lid_twist_release_compliance",
                self._release_compliance_service_name,
                force_timeout_sec,
                publish_failure=False,
            ) and ok
        self._publish_status(
            "lid_twist_force_released",
            success=ok,
            real_motion=True,
        )
        return ok

    def _call_lid_twist_gripper_action(self, command: str) -> bool:
        if not bool(self.get_parameter("enable_gripper_service_calls").value):
            self._publish_status(
                "failed",
                error="lid twist gripper action requires enable_gripper_service_calls:=true",
                real_motion=False,
            )
            return False
        targets = self._gripper_targets()
        if targets is None:
            return False
        timeout_sec = float(self.get_parameter("gripper_wait_timeout_sec").value)
        if not self._gripper_client.wait_for_service(timeout_sec=max(timeout_sec, 0.0)):
            self._publish_status(
                "failed",
                error=f"gripper service unavailable: {self.get_parameter('gripper_set_service').value}",
                real_motion=False,
            )
            return False
        preopen_width, grasp_width, force_n = targets
        if command == "preopen":
            return self._call_gripper_sync("preopen", preopen_width, force_n)
        if command == "grasp":
            return self._call_gripper_sync("grasp", grasp_width, force_n)
        self._publish_status(
            "failed",
            error=f"unsupported lid twist gripper action: {command}",
            real_motion=False,
        )
        return False

    def _open_gripper_after_lid_twist(self) -> bool:
        if not self._call_lid_twist_gripper_action("preopen"):
            return False
        gripper_wait = max(
            float(self.get_parameter("lid_twist_regrip_gripper_wait_sec").value),
            0.0,
        )
        if gripper_wait > 0.0:
            self._publish_status(
                "lid_twist_holding",
                step="lid_twist_final_preopen",
                seconds=round(gripper_wait, 3),
                real_motion=True,
            )
            time.sleep(gripper_wait)
        return True

    def _force_twist_clients_available(self) -> bool:
        required = [
            (self._task_compliance_client, self._task_compliance_service_name),
            (self._set_desired_force_client, self._set_desired_force_service_name),
            (self._release_force_client, self._release_force_service_name),
            (self._release_compliance_client, self._release_compliance_service_name),
        ]
        rotation_mode = self._lid_twist_force_rotation_mode()
        if self._preseat_periodic_enabled():
            required.append((self._move_periodic_client, self._move_periodic_service_name))
        if rotation_mode == "j6":
            required.extend([
                (self._move_joint_client, self._move_joint_service_name),
                (self._current_posj_client, self._current_posj_service_name),
            ])
        timeout_sec = self._lid_twist_force_timeout_sec()
        for client, name in required:
            if client is None:
                self._publish_status(
                    "failed",
                    error=f"{name} client unavailable; dsr_msgs2 service type may be missing",
                    real_motion=False,
                )
                return False
            if not client.wait_for_service(
                timeout_sec=timeout_sec,
            ):
                self._publish_status(
                    "failed",
                    error=f"{name} service unavailable",
                    real_motion=False,
                )
                return False
        return True

    def _call_bool_service(
        self,
        client,
        request,
        label: str,
        service: str,
        timeout_sec: float,
        *,
        publish_failure: bool = True,
    ) -> bool:
        if client is None:
            if publish_failure:
                self._publish_status(
                    "failed",
                    error=f"{label} client unavailable: {service}",
                    real_motion=False,
                )
            return False
        self._publish_status(
            "lid_twist_service_call",
            step=label,
            service=service,
            timeout_sec=round(timeout_sec, 3),
            real_motion=True,
        )
        future = client.call_async(request)
        result = self._wait_for_future(future, label, timeout_sec)
        if result is None or not bool(result.success):
            if publish_failure:
                self._publish_status(
                    "failed",
                    error=f"{label} returned success=false",
                    service=service,
                    real_motion=True,
                )
            return False
        return True

    def _lid_twist_force_timeout_sec(self) -> float:
        return max(float(self.get_parameter("lid_twist_force_service_timeout_sec").value), 0.1)

    def _lid_twist_force_control_enabled(self) -> bool:
        return bool(self.get_parameter("lid_twist_use_force_control").value) or bool(
            self.get_parameter("lid_twist_use_force_spiral").value
        )

    def _lid_twist_compliance_stiffness(self) -> list[float]:
        return [
            float(self.get_parameter("lid_twist_compliance_x_stiffness").value),
            float(self.get_parameter("lid_twist_compliance_y_stiffness").value),
            float(self.get_parameter("lid_twist_compliance_z_stiffness").value),
            float(self.get_parameter("lid_twist_compliance_rx_stiffness").value),
            float(self.get_parameter("lid_twist_compliance_ry_stiffness").value),
            float(self.get_parameter("lid_twist_compliance_rz_stiffness").value),
        ]

    def _lid_twist_force_ref(self) -> int:
        value = str(self.get_parameter("lid_twist_force_ref").value).strip().lower()
        if value == "tool":
            return DR_TOOL
        return DR_BASE

    def _lid_twist_force_rotation_mode(self) -> str:
        value = str(self.get_parameter("lid_twist_force_rotation_mode").value).strip().lower()
        if value in {"j6", "joint", "joint6", "movej", "move_joint"}:
            return "j6"
        return "movel"

    def _lid_twist_steps(self):
        target = self._lid_twist_target_pos()
        if target is None:
            return None
        press_down_m = max(float(self.get_parameter("lid_twist_press_down_m").value), 0.0)
        rz_delta_deg = float(self.get_parameter("lid_twist_rz_delta_deg").value)
        turn_step_deg = abs(float(self.get_parameter("lid_twist_turn_step_deg").value))
        transfer_clearance_m = max(
            float(self.get_parameter("lid_twist_transfer_clearance_m").value),
            0.0,
        )
        release_lift_m = max(float(self.get_parameter("lid_twist_release_lift_m").value), 0.0)
        min_z_m = float(self.get_parameter("lid_twist_min_z_m").value)
        max_z_m = float(self.get_parameter("lid_twist_max_z_m").value)
        transfer_max_z_m = float(self.get_parameter("lid_twist_transfer_max_z_m").value)
        if not math.isfinite(rz_delta_deg):
            self._publish_status(
                "failed",
                error="lid_twist_rz_delta_deg must be finite",
                real_motion=False,
            )
            return None
        if not math.isfinite(turn_step_deg) or turn_step_deg <= 0.0:
            self._publish_status(
                "failed",
                error="lid_twist_turn_step_deg must be positive",
                real_motion=False,
            )
            return None

        target_high = target.copy()
        target_high[2] += transfer_clearance_m * 1000.0
        pressed = target.copy()
        pressed[2] -= press_down_m * 1000.0
        turn_targets = self._lid_twist_turn_targets(pressed, rz_delta_deg, turn_step_deg)
        turned = turn_targets[-1] if turn_targets else pressed.copy()
        release = turned.copy()
        release[2] += release_lift_m * 1000.0

        validation_steps = []
        if transfer_clearance_m > 0.0:
            validation_steps.append(("lid_twist_transfer_high", target_high))
        validation_steps.extend([
            ("lid_twist_transfer", target),
            ("lid_twist_press", pressed),
            ("lid_twist_release", release),
        ])
        validation_steps.extend(
            (self._lid_twist_turn_label(index, len(turn_targets), rz_delta_deg), pos)
            for index, pos in enumerate(turn_targets, start=1)
        )
        for label, pos in validation_steps:
            z_m = pos[2] / 1000.0
            upper_z_m = transfer_max_z_m if label == "lid_twist_transfer_high" else max_z_m
            if z_m < min_z_m or z_m > upper_z_m:
                self._publish_status(
                    "failed",
                    error=(
                        f"{label} z={z_m:.3f} outside "
                        f"[{min_z_m:.3f}, {upper_z_m:.3f}]"
                    ),
                    target_mm=[round(value, 3) for value in pos[:3]],
                    real_motion=False,
                )
                return None

        transfer_velocity = float(self.get_parameter("lid_twist_transfer_velocity").value)
        press_velocity = float(self.get_parameter("lid_twist_press_velocity").value)
        turn_velocity = float(self.get_parameter("lid_twist_turn_velocity").value)
        hold_before_turn = max(
            float(self.get_parameter("lid_twist_hold_seconds_before_turn").value),
            0.0,
        )
        hold_after_turn = max(
            float(self.get_parameter("lid_twist_hold_seconds_after_turn").value),
            0.0,
        )
        self._publish_status(
            "lid_twist_planned",
            target_mm_deg=[round(value, 3) for value in target],
            target_high_mm_deg=[round(value, 3) for value in target_high],
            pressed_mm_deg=[round(value, 3) for value in pressed],
            turned_mm_deg=[round(value, 3) for value in turned],
            press_down_m=round(press_down_m, 4),
            rz_delta_deg=round(rz_delta_deg, 3),
            turn_step_deg=round(turn_step_deg, 3),
            turn_count=len(turn_targets),
            transfer_clearance_m=round(transfer_clearance_m, 4),
            real_motion=True,
        )
        steps = []
        if transfer_clearance_m > 0.0:
            steps.append(("lid_twist_transfer_high", target_high, transfer_velocity, True, 0.0))
        steps.extend([
            ("lid_twist_transfer", target, press_velocity, True, 0.0),
            ("lid_twist_press", pressed, press_velocity, True, hold_before_turn),
        ])
        steps.extend(
            (
                self._lid_twist_turn_label(index, len(turn_targets), rz_delta_deg),
                pos,
                turn_velocity,
                True,
                hold_after_turn if index == len(turn_targets) else 0.0,
            )
            for index, pos in enumerate(turn_targets, start=1)
        )
        steps.append(("lid_twist_release", release, press_velocity, True, 0.0))
        return steps

    @staticmethod
    def _lid_twist_turn_targets(pressed_pos: list[float], total_delta_deg: float, step_deg: float) -> list[list[float]]:
        if abs(total_delta_deg) <= 1e-9:
            return []
        direction = 1.0 if total_delta_deg > 0.0 else -1.0
        remaining = abs(total_delta_deg)
        cumulative = 0.0
        targets: list[list[float]] = []
        while remaining > 1e-9:
            increment = min(step_deg, remaining)
            cumulative += direction * increment
            target = pressed_pos.copy()
            target[5] = pressed_pos[5] + cumulative
            targets.append(target)
            remaining -= increment
        return targets

    @staticmethod
    def _lid_twist_turn_delta_steps(total_delta_deg: float, step_deg: float) -> list[float]:
        if abs(total_delta_deg) <= 1e-9:
            return []
        direction = 1.0 if total_delta_deg > 0.0 else -1.0
        remaining = abs(total_delta_deg)
        deltas: list[float] = []
        while remaining > 1e-9:
            increment = min(step_deg, remaining)
            deltas.append(direction * increment)
            remaining -= increment
        return deltas

    @staticmethod
    def _lid_twist_turn_label(index: int, total: int, delta_deg: float) -> str:
        direction = "clockwise" if delta_deg > 0.0 else "counterclockwise"
        return f"lid_twist_turn_{direction}_{index:02d}_of_{total:02d}"

    @staticmethod
    def _lid_twist_cycle_turn_label(
        phase: str,
        cycle_index: int,
        cycle_total: int,
        step_index: int,
        step_total: int,
        delta_deg: float,
    ) -> str:
        direction = "clockwise" if delta_deg > 0.0 else "counterclockwise"
        return (
            f"lid_twist_{phase}_{direction}_"
            f"c{cycle_index:02d}_of_{cycle_total:02d}_"
            f"{step_index:02d}_of_{step_total:02d}"
        )

    def _lid_twist_target_pos(self) -> list[float] | None:
        values = [
            float(self.get_parameter("lid_twist_target_x_m").value) * 1000.0,
            float(self.get_parameter("lid_twist_target_y_m").value) * 1000.0,
            float(self.get_parameter("lid_twist_target_z_m").value) * 1000.0,
            float(self.get_parameter("lid_twist_rx").value),
            float(self.get_parameter("lid_twist_ry").value),
            float(self.get_parameter("lid_twist_rz").value),
        ]
        if not all(math.isfinite(value) for value in values):
            self._publish_status(
                "failed",
                error=(
                    "lid twist target is required before post-grasp rotation; set "
                    "lid_twist_target_x_m/y_m/z_m and lid_twist_rx/ry/rz from measured teach point"
                ),
                real_motion=False,
            )
            return None
        return values

    def _current_posj(self, timeout_sec: float) -> list[float] | None:
        if self._current_posj_client is None:
            return None
        req = GetCurrentPosj.Request()
        future = self._current_posj_client.call_async(req)
        result = self._wait_for_future(future, "get_current_posj", timeout_sec)
        if result is None or not bool(result.success):
            return None
        values = [float(value) for value in result.pos]
        if len(values) < 6:
            return None
        return values[:6]

    def _current_posx(self, timeout_sec: float) -> list[float] | None:
        if self._current_posx_client is None:
            return None
        req = GetCurrentPosx.Request()
        req.ref = DR_BASE
        future = self._current_posx_client.call_async(req)
        result = self._wait_for_future(future, "get_current_posx", timeout_sec)
        if result is None or not bool(result.success) or not result.task_pos_info:
            return None
        values = [float(value) for value in result.task_pos_info[0].data]
        if len(values) < 6:
            return None
        return values[:6]

    def _read_last_alarm(self) -> dict | None:
        if self._last_alarm_client is None:
            return None
        if not self._last_alarm_client.wait_for_service(timeout_sec=0.5):
            return None
        future = self._last_alarm_client.call_async(GetLastAlarm.Request())
        result = self._wait_for_future(future, "get_last_alarm", 1.0)
        if result is None or not bool(result.success):
            return None
        alarm = result.log_alarm
        return {
            "level": int(alarm.level),
            "group": int(alarm.group),
            "index": int(alarm.index),
            "param": list(alarm.param),
        }

    def _try_gripper_sequence(self) -> None:
        targets = self._gripper_targets()
        if targets is None:
            return
        preopen_width, grasp_width, force_n = targets
        timeout_sec = float(self.get_parameter("gripper_wait_timeout_sec").value)
        if not self._gripper_client.wait_for_service(timeout_sec=max(timeout_sec, 0.0)):
            self._publish_status(
                "failed",
                error=f"gripper service unavailable: {self.get_parameter('gripper_set_service').value}",
                real_motion=False,
            )
            return

        self._call_gripper("preopen", preopen_width, force_n)
        self._publish_status(
            "gripper_preopen_requested",
            width_m=preopen_width,
            force_n=force_n,
            real_motion=False,
            note="RG2 request only; no Doosan motion was executed",
        )
        self._call_gripper("grasp", grasp_width, force_n)
        self._publish_status(
            "gripper_grasp_requested",
            width_m=grasp_width,
            force_n=force_n,
            real_motion=False,
            note="RG2 request only; no Doosan motion was executed",
        )

    def _gripper_targets(self):
        preopen_width = float(self.get_parameter("gripper_preopen_width_m").value)
        grasp_width = float(self.get_parameter("gripper_grasp_width_m").value)
        force_n = float(self.get_parameter("gripper_force_n").value)
        if not all(math.isfinite(value) and value >= 0.0 for value in (preopen_width, grasp_width)):
            self._publish_status(
                "failed",
                error=(
                    "measured gripper_preopen_width_m and gripper_grasp_width_m "
                    "are required before RG2 service calls"
                ),
                real_motion=False,
            )
            return None
        if not math.isfinite(force_n) or force_n <= 0.0:
            self._publish_status(
                "failed",
                error="measured gripper_force_n is required before RG2 service calls",
                real_motion=False,
            )
            return None
        return preopen_width, grasp_width, force_n

    def _call_gripper(self, command: str, width_m: float, force_n: float) -> None:
        req = SetGripper.Request()
        req.command = command
        req.width_m = float(width_m)
        req.force_n = float(force_n)
        future = self._gripper_client.call_async(req)
        future.add_done_callback(lambda done: self._on_gripper_done(command, done))

    def _call_gripper_sync(
        self,
        command: str,
        width_m: float,
        force_n: float,
        *,
        publish_failure: bool = True,
    ) -> bool:
        req = SetGripper.Request()
        req.command = command
        req.width_m = float(width_m)
        req.force_n = float(force_n)
        future = self._gripper_client.call_async(req)
        result = self._wait_for_future(
            future,
            f"gripper_{command}",
            float(self.get_parameter("gripper_wait_timeout_sec").value),
        )
        if result is None:
            if publish_failure:
                self._publish_status(
                    "failed",
                    error=f"RG2 {command} request timed out or failed",
                    real_motion=True,
                )
            return False
        self._publish_status(
            "gripper_result",
            command=command,
            success=bool(result.success),
            message=str(result.message),
            real_motion=True,
        )
        return bool(result.success)

    def _wait_for_future(self, future, label: str, timeout_sec: float):
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"[뚜껑픽] 실패: {self._step_ko(label)} 서비스 응답 대기 "
                    f"{timeout_sec:.1f}초 초과"
                )
                return None
            time.sleep(0.01)
        if not future.done():
            return None
        if future.exception() is not None:
            self.get_logger().error(
                f"[뚜껑픽] 실패: {self._step_ko(label)} 서비스 예외: {future.exception()}"
            )
            return None
        return future.result()

    def _on_gripper_done(self, command: str, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._publish_status(
                "failed",
                error=f"RG2 {command} request failed: {exc}",
                real_motion=False,
            )
            return
        self._publish_status(
            "gripper_result",
            command=command,
            success=bool(result.success),
            message=str(result.message),
            real_motion=False,
        )

    def _config(self) -> LidGripConfig:
        return LidGripConfig(
            approach_offset_m=float(self.get_parameter("approach_offset_m").value),
            lift_offset_m=float(self.get_parameter("lift_offset_m").value),
            surface_offset_m=float(self.get_parameter("surface_offset_m").value),
            offset_axis=str(self.get_parameter("offset_axis").value),
            tcp_grasp_offset_x_m=float(self.get_parameter("tcp_grasp_offset_x_m").value),
            tcp_grasp_offset_y_m=float(self.get_parameter("tcp_grasp_offset_y_m").value),
            tcp_grasp_offset_z_m=float(self.get_parameter("tcp_grasp_offset_z_m").value),
            min_grasp_z_m=float(self.get_parameter("min_grasp_z_m").value),
            max_grasp_z_m=float(self.get_parameter("max_grasp_z_m").value),
        )

    def _publish_status(self, status: str, **fields) -> None:
        payload = {"status": status, "node": "lid_grip_planner_node"}
        payload.update(fields)
        text = json.dumps(payload, sort_keys=True)
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        if not self._should_log_status(status, fields):
            return
        if bool(self.get_parameter("log_korean_status").value):
            message = self._human_status_text(status, fields)
            if status == "failed":
                self.get_logger().error(message)
            else:
                self.get_logger().info(message)
        if not bool(self.get_parameter("log_json_status").value):
            return
        if status == "failed":
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)

    def _should_log_status(self, status: str, fields: dict) -> bool:
        request_source = fields.get("request_source")
        if request_source == "pose" and not bool(self.get_parameter("log_pose_plans").value):
            return False
        return True

    def _human_status_text(self, status: str, fields: dict) -> str:
        if status == "trigger_received":
            hardware = "ON" if fields.get("hardware_armed") else "OFF"
            return f"[뚜껑픽] 1/6 p 입력 수신: hardware_armed={hardware}"
        if status == "planned":
            grasp = fields.get("grasp", {})
            approach = fields.get("approach", {})
            lift = fields.get("lift", {})
            motion = "ON" if fields.get("motion_allowed") else "OFF"
            rz_text = ""
            if fields.get("pick_rz_deg") is not None:
                rz_text = f", pick_rz={fields.get('pick_rz_deg')}deg"
                if fields.get("pick_yaw_enabled"):
                    rz_text += (
                        f", axis={fields.get('lid_pose_yaw_axis')}"
                        f", aruco_yaw={fields.get('lid_pose_yaw_deg')}deg"
                        f", yaw_x={fields.get('lid_pose_x_yaw_deg')}deg"
                        f", yaw_y={fields.get('lid_pose_y_yaw_deg')}deg"
                    )
            return (
                "[뚜껑픽] 2/6 목표 계산 완료: "
                f"approach z={self._fmt_coord(approach, 'z')} m, "
                f"grasp z={self._fmt_coord(grasp, 'z')} m, "
                f"lift z={self._fmt_coord(lift, 'z')} m, motion={motion}{rz_text}"
            )
        if status == "ikin_result":
            target = fields.get("target_mm_deg", [])
            return (
                f"[뚜껑픽] IK 확인 완료: {self._step_ko(str(fields.get('step', '')))} "
                f"target_mm={self._fmt_target_mm(target)}"
            )
        if status == "motion_target_reached":
            target_rpy = fields.get("target_rpy_deg")
            actual_rpy = fields.get("actual_rpy_deg")
            rpy_error = fields.get("rpy_error_deg")
            rpy_text = ""
            if target_rpy is not None and actual_rpy is not None:
                rpy_text = (
                    f" target_rpy={target_rpy} actual_rpy={actual_rpy} "
                    f"rpy_error={rpy_error}"
                )
            return (
                f"[뚜껑픽] 이동 도착 확인: {self._step_ko(str(fields.get('step', '')))} "
                f"오차={fields.get('distance_m')} m{rpy_text}"
            )
        if status == "gripper_result":
            command = str(fields.get("command", ""))
            success = "성공" if fields.get("success") else "실패"
            return f"[뚜껑픽] 그리퍼 {self._gripper_command_ko(command)} {success}: {fields.get('message', '')}"
        if status == "settling_before_grasp":
            return f"[뚜껑픽] 파지 위치에서 정지 대기: {fields.get('seconds')}초 후 그리퍼를 닫습니다"
        if status == "visual_refine_collecting":
            return (
                "[뚜껑보정] approach 위치에서 ArUco 재관측 시작: "
                f"samples={fields.get('sample_count')} "
                f"timeout={fields.get('timeout_sec')}초 "
                f"axis={fields.get('axis')}"
            )
        if status == "visual_refine_result":
            return (
                "[뚜껑보정] 재관측 완료: "
                f"samples={fields.get('samples')} "
                f"xy_std={fields.get('position_std_m')}m "
                f"yaw_std={fields.get('yaw_std_deg')}deg "
                f"mean=({fields.get('mean_x')}, {fields.get('mean_y')}) "
                f"axis={fields.get('axis')} "
                f"yaw={fields.get('mean_yaw_deg')}deg "
                f"pick_rz={fields.get('pick_rz_deg')}deg "
                f"(x: yaw={fields.get('mean_yaw_x_deg')}deg/rz={fields.get('pick_rz_x_deg')}deg, "
                f"y: yaw={fields.get('mean_yaw_y_deg')}deg/rz={fields.get('pick_rz_y_deg')}deg)"
            )
        if status == "visual_refine_fallback":
            return (
                "[뚜껑보정] 재관측 실패, 기존 좌표로 진행: "
                f"reason={fields.get('reason')} "
                f"samples={fields.get('samples')} "
                f"xy_std={fields.get('position_std_m')}m "
                f"yaw_std={fields.get('yaw_std_deg')}deg"
            )
        if status == "gripper_grasp_continue_after_failure":
            return (
                "[뚜껑픽] 그리퍼 닫기 확인 실패, 대기 후 lift 계속 진행: "
                f"wait={fields.get('seconds')}초"
            )
        if status == "lid_twist_planned":
            target_high = fields.get("target_high_mm_deg", [])
            pressed = fields.get("pressed_mm_deg", [])
            turned = fields.get("turned_mm_deg", [])
            mode = (
                f"force_{fields.get('force_rotation_mode')}"
                if fields.get("force_control")
                else "position"
            )
            return (
                f"[뚜껑회전] 목표 계산 완료({mode}): "
                f"safe target_mm={self._fmt_target_mm(target_high)} "
                f"press target_mm={self._fmt_target_mm(pressed)} "
                f"turn_rz={self._fmt_rz(turned)} deg "
                f"turn_count={fields.get('turn_count')} "
                f"step={fields.get('turn_step_deg')} deg "
                f"clearance={fields.get('transfer_clearance_m')} m "
                f"press_down={fields.get('press_down_m')} m "
                f"rz_delta={fields.get('rz_delta_deg')} deg"
            )
        if status == "lid_twist_force_enabled":
            return (
                "[뚜껑회전] 힘 제어 ON: "
                f"down_force={fields.get('down_force_n')} N "
                f"ref={fields.get('ref')} stiffness={fields.get('stiffness')}"
            )
        if status == "lid_twist_service_call":
            return (
                f"[뚜껑회전] {self._step_ko(str(fields.get('step', '')))} "
                f"서비스 호출: {fields.get('service')} "
                f"timeout={fields.get('timeout_sec')}초"
            )
        if status == "lid_twist_abort_recovery":
            return "[뚜껑회전] 회전 실패 복구: 힘 해제 후 안전 상승을 시도합니다"
        if status == "lid_twist_preseat_periodic_start":
            return (
                "[뚜껑회전] pre-seat periodic 비틀림 안착 시작: "
                f"amp={fields.get('amp')} period={fields.get('period_sec')}초 "
                f"repeat={fields.get('repeat')} ref={fields.get('ref')}"
            )
        if status == "lid_twist_joint_relative_start":
            before = fields.get("before_joints_deg")
            before_j6 = before[5] if isinstance(before, list) and len(before) >= 6 else None
            delta = fields.get("delta_joints_deg")
            delta_j6 = delta[5] if isinstance(delta, list) and len(delta) >= 6 else None
            return (
                f"[뚜껑회전] J6 상대 회전 시작: {self._step_ko(str(fields.get('step', '')))} "
                f"before_j6={before_j6}deg delta_j6={delta_j6}deg "
                f"vel={fields.get('velocity')} acc={fields.get('acceleration')}"
            )
        if status == "lid_twist_joint_relative_done":
            after = fields.get("after_joints_deg")
            after_j6 = after[5] if isinstance(after, list) and len(after) >= 6 else None
            delta = fields.get("delta_joints_deg")
            delta_j6 = delta[5] if isinstance(delta, list) and len(delta) >= 6 else None
            return (
                f"[뚜껑회전] J6 상대 회전 완료: {self._step_ko(str(fields.get('step', '')))} "
                f"after_j6={after_j6}deg delta_j6={delta_j6}deg"
            )
        if status == "lid_twist_force_released":
            success = "성공" if fields.get("success") else "실패"
            return f"[뚜껑회전] 힘 제어 해제 {success}"
        if status == "lid_twist_holding":
            return (
                f"[뚜껑회전] {self._step_ko(str(fields.get('step', '')))} "
                f"정지 대기: {fields.get('seconds')}초"
            )
        if status == "gripper_preopen_requested":
            return (
                "[뚜껑픽] 그리퍼 preopen 요청: "
                f"width={fields.get('width_m')} m force={fields.get('force_n')} N"
            )
        if status == "gripper_grasp_requested":
            return (
                "[뚜껑픽] 그리퍼 grasp 요청: "
                f"width={fields.get('width_m')} m force={fields.get('force_n')} N"
            )
        if status == "motion_sequence_requested":
            return "[뚜껑픽] 6/6 sequence 완료: approach -> grasp -> lift 요청 완료"
        if status == "trigger_ignored_motion_in_progress":
            return "[뚜껑픽] p 입력 무시: 이전 motion sequence가 아직 진행 중입니다"
        if status == "trigger_planned_no_motion":
            return "[뚜껑픽] motion 미실행: enable_hardware=false라 plan만 생성했습니다"
        if status == "failed":
            detail = str(fields.get("error", "unknown"))
            step = fields.get("step")
            if step:
                detail = f"{self._step_ko(str(step))}: {detail}"
            target = fields.get("target_mm") or fields.get("target_mm_deg")
            actual = fields.get("actual_mm")
            extra = ""
            if target is not None:
                extra += f" target={target}"
            if actual is not None:
                extra += f" actual={actual}"
            if fields.get("distance_m") is not None:
                extra += f" distance={fields.get('distance_m')}m"
            if fields.get("tolerance_m") is not None:
                extra += f" tolerance={fields.get('tolerance_m')}m"
            if fields.get("orientation_error_deg") is not None:
                extra += f" orientation_error={fields.get('orientation_error_deg')}deg"
            return f"[뚜껑픽] 실패: {detail}{extra}"
        return f"[뚜껑픽] {status}: {fields}"

    @staticmethod
    def _step_ko(label: str) -> str:
        if label.startswith("lid_twist_turn_clockwise_") and "_ikin" not in label:
            return label.replace("lid_twist_turn_clockwise_", "시계방향 회전 ")
        if label.startswith("lid_twist_turn_counterclockwise_") and "_ikin" not in label:
            return label.replace("lid_twist_turn_counterclockwise_", "반시계방향 회전 ")
        if label.startswith("lid_twist_reset_clockwise_") and "_ikin" not in label:
            return label.replace("lid_twist_reset_clockwise_", "되감기 시계방향 회전 ")
        if label.startswith("lid_twist_reset_counterclockwise_") and "_ikin" not in label:
            return label.replace("lid_twist_reset_counterclockwise_", "되감기 반시계방향 회전 ")
        if label.startswith("lid_twist_turn_clockwise_") and label.endswith("_ikin"):
            core = label.removesuffix("_ikin")
            return core.replace("lid_twist_turn_clockwise_", "시계방향 회전 IK ")
        if label.startswith("lid_twist_turn_counterclockwise_") and label.endswith("_ikin"):
            core = label.removesuffix("_ikin")
            return core.replace("lid_twist_turn_counterclockwise_", "반시계방향 회전 IK ")
        if label.startswith("lid_twist_reset_clockwise_") and label.endswith("_ikin"):
            core = label.removesuffix("_ikin")
            return core.replace("lid_twist_reset_clockwise_", "되감기 시계방향 회전 IK ")
        if label.startswith("lid_twist_reset_counterclockwise_") and label.endswith("_ikin"):
            core = label.removesuffix("_ikin")
            return core.replace("lid_twist_reset_counterclockwise_", "되감기 반시계방향 회전 IK ")
        return {
            "approach_lid": "1단계 접근 위치",
            "grasp_lid": "2단계 파지 위치",
            "lift_lid": "3단계 상승 위치",
            "lid_twist_transfer_high": "4단계 고정 위치 위 안전 이동",
            "lid_twist_transfer": "5단계 회전 위치 수직 하강",
            "lid_twist_press": "6단계 아래로 눌러 접촉",
            "lid_twist_turn_clockwise": "7단계 시계방향 회전",
            "lid_twist_release": "8단계 압력 해제 상승",
            "lid_twist_home": "8단계 그리퍼 open 후 안전 위치 이동",
            "visual_refine_align_lid": "1.5단계 ArUco 보정 위치 정렬",
            "approach_lid_ikin": "접근 위치 IK",
            "visual_refine_align_lid_ikin": "ArUco 보정 위치 IK",
            "grasp_lid_ikin": "파지 위치 IK",
            "lift_lid_ikin": "상승 위치 IK",
            "lid_twist_transfer_high_ikin": "고정 위치 위 안전 이동 IK",
            "lid_twist_transfer_ikin": "회전 위치 IK",
            "lid_twist_press_ikin": "누르기 위치 IK",
            "lid_twist_turn_clockwise_ikin": "시계방향 회전 IK",
            "lid_twist_release_ikin": "압력 해제 위치 IK",
            "lid_twist_home_ikin": "그리퍼 open 후 안전 위치 IK",
            "lid_twist_task_compliance": "힘 제어 컴플라이언스 설정",
            "lid_twist_set_force": "아래 방향 목표 힘 설정",
            "lid_twist_force_settle": "힘 안정화",
            "lid_twist_periodic_settle": "periodic 비틀림 전 안정화",
            "lid_twist_preseat_periodic": "pre-seat periodic 비틀림 안착",
            "lid_twist_turn_complete": "TCP 기준 상대 회전 완료",
            "lid_twist_release_force": "목표 힘 해제",
            "lid_twist_release_compliance": "컴플라이언스 해제",
            "lid_twist_final_preopen": "회전 완료 후 그리퍼 열기",
            "lid_twist_regrip_preopen": "재파지 그리퍼 열기",
            "lid_twist_regrip_grasp": "재파지 그리퍼 닫기",
            "gripper_preopen": "그리퍼 열기",
            "gripper_grasp": "그리퍼 닫기",
            "get_current_posj": "현재 관절 위치 확인",
            "get_current_posx": "현재 TCP 위치 확인",
            "get_last_alarm": "마지막 알람 확인",
        }.get(label, label)

    @staticmethod
    def _gripper_command_ko(command: str) -> str:
        return {
            "preopen": "열기",
            "open": "열기",
            "grasp": "닫기",
            "close": "닫기",
            "set_width": "폭 설정",
        }.get(command, command)

    @staticmethod
    def _fmt_coord(value: dict, key: str) -> str:
        coord = value.get(key) if isinstance(value, dict) else None
        return "?" if coord is None else f"{float(coord):.4f}"

    @staticmethod
    def _fmt_target_mm(target: list) -> str:
        if not target:
            return "?"
        return "[" + ", ".join(f"{float(value):.1f}" for value in target[:3]) + "]"

    @staticmethod
    def _fmt_rz(target: list) -> str:
        if len(target) < 6:
            return "?"
        return f"{float(target[5]):.1f}"

    @staticmethod
    def _angle_delta_deg(actual: float, target: float) -> float:
        return (float(actual) - float(target) + 180.0) % 360.0 - 180.0

    @staticmethod
    def _pose_stamped(pose, source: PoseStamped) -> PoseStamped:
        msg = PoseStamped()
        msg.header = source.header
        msg.pose = pose
        return msg

    @staticmethod
    def _pose_xyz(pose) -> dict:
        return {
            "x": round(float(pose.position.x), 4),
            "y": round(float(pose.position.y), 4),
            "z": round(float(pose.position.z), 4),
        }


def main(args=None):
    rclpy.init(args=args)
    node = LidGripPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
