from __future__ import annotations

import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from azas_interfaces.msg import CupDetection
from dsr_msgs2.srv import MoveJoint, MoveWait
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class RouteDecision:
    route: str
    status: str
    confidence: float


@dataclass(frozen=True)
class DetectionOverlay:
    center_u: int
    center_v: int
    width: int
    height: int
    orientation: str
    class_name: str
    classifier_confidence: Optional[float]


class AutoCupFlowRouter(Node):
    def __init__(self) -> None:
        super().__init__("auto_cup_flow_router")

        self.declare_parameter("router_confirm", "")
        self.declare_parameter("enable_real_motion", False)
        self.declare_parameter("observe_joints_deg", [3.0, -12.7, 44.0, -9.0, 133.0, 90.0])
        self.declare_parameter("observe_vel", 30.0)
        self.declare_parameter("observe_acc", 30.0)
        self.declare_parameter("observe_time", 0.0)
        self.declare_parameter("motion_timeout_sec", 25.0)
        self.declare_parameter("service_prefix", "")
        self.declare_parameter("gripper_open_service", "/jarvis/rg2/open")

        self.declare_parameter("detection_topic", "/azas/cup_detection")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("perception_launch", "azas_bringup yolo_perception.launch.py")
        self.declare_parameter("yolo_model_path", "/home/ssu/Azas/local_models/best.pt")
        self.declare_parameter("classifier_path", "/home/ssu/Azas/cup_classifier_best.pth")
        self.declare_parameter("classifier_arch", "resnet18")
        self.declare_parameter("classifier_min_confidence", 0.70)
        self.declare_parameter("route_timeout_sec", 30.0)
        self.declare_parameter("route_stable_required_samples", 5)
        self.declare_parameter("route_stable_min_sec", 0.8)
        self.declare_parameter("route_hold_sec", 3.5)
        self.declare_parameter("show_classification_window", True)
        self.declare_parameter("window_name", "Azas cup route classifier")

        self.declare_parameter("side_launch", "dsr_practice yolo_cup_pick_node.launch.py")
        self.declare_parameter("cup_uprighting_launch", "azas_cup_uprighting yolo_cup_uprighting.launch.py")
        self.declare_parameter("moveit_controller_name", "/dsr_moveit_controller")
        self.declare_parameter("controller_action_name", "/dsr_moveit_controller/follow_joint_trajectory")
        self.declare_parameter("side_extra_args", "")
        self.declare_parameter("cup_uprighting_extra_args", "")
        # 사이드 그립에서 base x가 +20mm 정도 어긋나는 실측 보정값
        self.declare_parameter("side_target_x_offset_m", -0.02)

        self.declare_parameter("color_scan_at_start", True)
        self.declare_parameter(
            "color_scan_command",
            "bash /home/ssu/Azas/tools/run/run_color_scan_stage.sh",
        )
        self.declare_parameter("recipe_after_success", True)
        self.declare_parameter(
            "recipe_command",
            "cd /home/ssu/Azas && source /opt/ros/humble/setup.bash && "
            "mkdir -p /tmp/azas_ros_logs && export ROS_LOG_DIR=/tmp/azas_ros_logs && "
            "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-9} && "
            "export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0} && "
            "export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4} && "
            "if [ -f /home/ssu/ws_moveit/install/setup.bash ]; then source /home/ssu/ws_moveit/install/setup.bash; fi && "
            "if [ -f /home/ssu/ros2_ws/install/setup.bash ]; then source /home/ssu/ros2_ws/install/setup.bash; fi && "
            "if [ -f /home/ssu/Azas/install/setup.bash ]; then source /home/ssu/Azas/install/setup.bash; "
            "else source /home/ssu/Azas/install/local_setup.bash; fi && "
            "export PYTHONPATH=/home/ssu/Azas/tools/run/python_compat:${PYTHONPATH:-} && "
            "python3 tools/run/run_color_recipe_sequence.py --execute --confirm",
        )
        # 키오스크/음성 주문(latest_recipe.json) 없이 색을 직접 내릴 때: 예) "red:2,blue:1"
        self.declare_parameter("recipe_colors", "")
        # 디스펜서 누르기 종료 후 디스펜서 앞의 컵을 마지막으로 재파지할 때 z 실측 보정값
        self.declare_parameter("final_regrasp_z_offset_m", -0.02)
        # 잡기 직전 pre 위치(cup_place 기준 X offset) 보정값. 스크립트 기본 -0.09에 -30mm 추가
        self.declare_parameter("cup_pre_from_place_x_offset_m", -0.12)
        # 컵홀더에 놓을 때 보정값과 place 목표 z 안전 하한 (필요 시 조정)
        self.declare_parameter("cup_holder_place_z_offset_m", -0.03)
        self.declare_parameter("cup_holder_place_y_offset_m", 0.0)
        self.declare_parameter("cup_holder_z_min_m", 0.08)
        self.declare_parameter("lid_shake_after_recipe", True)
        self.declare_parameter(
            "lid_shake_command",
            "bash /home/ssu/Azas/tools/run/run_lid_close_then_shake_chain.sh",
        )

        self._latest_detection: Optional[CupDetection] = None
        self._latest_image: Optional[np.ndarray] = None
        self._image_lock = threading.Lock()
        self._window_enabled = bool(self.get_parameter("show_classification_window").value)
        self._children: list[subprocess.Popen[str]] = []
        self._child_node_failures: dict[str, list[str]] = {}

        self.create_subscription(
            CupDetection,
            str(self.get_parameter("detection_topic").value),
            self._on_detection,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("color_topic").value),
            self._on_image,
            10,
        )

    def run(self) -> int:
        if not self._confirmed():
            return 2

        self.get_logger().info("auto cup router: color scan -> observe -> open -> classify -> route")
        perception = None
        try:
            if not self._run_color_scan_sequence():
                return 1
            if not self._move_observe("initial observe"):
                return 1
            if not self._open_gripper("initial gripper full-open"):
                return 1

            perception = self._start_perception()
            decision = self._wait_for_route_decision()
            if decision is None:
                self.get_logger().error("route decision failed: no stable upright/lying classification")
                return 1
            self._stop_process(perception, "perception")
            perception = None

            if decision.route == "side_grasp":
                success = self._run_side_grasp(decision)
            else:
                success = self._run_cup_uprighting(decision)
            if not success:
                return 1
            if not self._run_recipe_sequence():
                return 1
            if not self._run_lid_shake_sequence():
                return 1
            self.get_logger().info("auto cup router: selected flow completed; router exiting")
            return 0
        finally:
            self._stop_process(perception, "perception")
            self._stop_all_children()
            self._destroy_window()

    def _confirmed(self) -> bool:
        if not bool(self.get_parameter("enable_real_motion").value):
            self.get_logger().error("enable_real_motion must be true for this router")
            return False
        token = str(self.get_parameter("router_confirm").value)
        if token != "ENABLE_AUTO_CUP_ROUTER":
            self.get_logger().error("router_confirm must be ENABLE_AUTO_CUP_ROUTER")
            return False
        return True

    def _on_detection(self, msg: CupDetection) -> None:
        self._latest_detection = msg

    def _on_image(self, msg: Image) -> None:
        if not self._window_enabled:
            return
        try:
            image = self._image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f"classification window disabled: image conversion failed: {exc}")
            self._window_enabled = False
            return
        with self._image_lock:
            self._latest_image = image

    def _start_perception(self) -> subprocess.Popen[str]:
        classifier_path = str(self.get_parameter("classifier_path").value)
        cmd = self._launch_command(str(self.get_parameter("perception_launch").value))
        cmd.extend([
            f"model_path:={self.get_parameter('yolo_model_path').value}",
            f"orientation_classifier_path:={classifier_path}",
            f"orientation_classifier_arch:={self.get_parameter('classifier_arch').value}",
            f"orientation_classifier_min_confidence:={self.get_parameter('classifier_min_confidence').value}",
        ])
        self.get_logger().info("starting perception with cup classifier: " + " ".join(cmd))
        proc = self._popen(cmd, "perception")
        return proc

    def _wait_for_route_decision(self) -> Optional[RouteDecision]:
        timeout = float(self.get_parameter("route_timeout_sec").value)
        required = max(1, int(self.get_parameter("route_stable_required_samples").value))
        stable_min_sec = max(0.0, float(self.get_parameter("route_stable_min_sec").value))
        hold_sec = max(3.0, float(self.get_parameter("route_hold_sec").value))
        end_time = time.monotonic() + timeout
        stable_route: Optional[str] = None
        stable_since = 0.0
        stable_count = 0
        last_reported_count = 0
        decided: Optional[RouteDecision] = None
        hold_until: Optional[float] = None

        self.get_logger().info(
            f"waiting for stable route: samples={required}, min_sec={stable_min_sec:.2f}, view_hold={hold_sec:.2f}s"
        )
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
            detection = self._latest_detection
            if detection is not None:
                route = self._route_from_status(detection.status)
                if route is not None:
                    decision = RouteDecision(route, detection.status, float(detection.confidence))
                    now = time.monotonic()
                    if stable_route == route:
                        stable_count += 1
                    else:
                        stable_route = route
                        stable_since = now
                        stable_count = 1
                        last_reported_count = 0
                    stable_elapsed = now - stable_since
                    if stable_count >= required and stable_count != last_reported_count:
                        last_reported_count = stable_count
                        self.get_logger().info(
                            f"route candidate stable: {route} samples={stable_count} elapsed={stable_elapsed:.2f}s"
                        )
                    if stable_count >= required and stable_elapsed >= stable_min_sec:
                        decided = decision
                        if hold_until is None:
                            hold_until = time.monotonic() + hold_sec
                            self.get_logger().info(
                                f"route decided: {decided.route} confidence={decided.confidence:.3f} status={decided.status}"
                            )

            self._show_classification_frame(decided)
            if decided is not None and hold_until is not None and time.monotonic() >= hold_until:
                return decided
        return decided

    @staticmethod
    def _route_from_status(status: str) -> Optional[str]:
        normalized = status.strip().lower()
        if normalized.startswith("detected:upright"):
            return "side_grasp"
        if normalized.startswith("rejected:lying"):
            return "cup_uprighting"
        return None

    def _show_classification_frame(self, decision: Optional[RouteDecision]) -> None:
        if not self._window_enabled:
            return
        with self._image_lock:
            image = None if self._latest_image is None else self._latest_image.copy()
        if image is None:
            return
        status = self._latest_detection.status if self._latest_detection is not None else "waiting"
        route = decision.route if decision else "stabilizing"
        overlay = self._overlay_from_status(status)
        if overlay is not None:
            self._draw_detection_overlay(image, overlay, route)
        self._draw_text_panel(image, route, status)
        try:
            cv2.imshow(str(self.get_parameter("window_name").value), image)
            cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().warn(f"classification window disabled: {exc}")
            self._window_enabled = False

    @staticmethod
    def _overlay_from_status(status: str) -> Optional[DetectionOverlay]:
        bbox_match = re.search(r"bbox=(\d+)x(\d+)", status)
        center_match = re.search(r"center=\((\d+),(\d+)\)", status)
        orientation_match = re.search(r"orientation=([^\s]+)", status)
        if bbox_match is None or center_match is None:
            return None
        return DetectionOverlay(
            center_u=int(center_match.group(1)),
            center_v=int(center_match.group(2)),
            width=int(bbox_match.group(1)),
            height=int(bbox_match.group(2)),
            orientation=orientation_match.group(1) if orientation_match else "unknown",
            class_name=AutoCupFlowRouter._status_field(status, "class") or "cup",
            classifier_confidence=AutoCupFlowRouter._status_float(
                status,
                "orientation_classifier_confidence",
            ),
        )

    @staticmethod
    def _status_field(status: str, key: str) -> Optional[str]:
        match = re.search(rf"{re.escape(key)}=([^\s]+)", status)
        return match.group(1) if match else None

    @staticmethod
    def _status_float(status: str, key: str) -> Optional[float]:
        value = AutoCupFlowRouter._status_field(status, key)
        if value is None:
            return None
        try:
            return float(value)
        except ValueError:
            return None

    @staticmethod
    def _draw_detection_overlay(image: np.ndarray, overlay: DetectionOverlay, route: str) -> None:
        image_h, image_w = image.shape[:2]
        x1 = max(0, overlay.center_u - overlay.width // 2)
        y1 = max(0, overlay.center_v - overlay.height // 2)
        x2 = min(image_w - 1, overlay.center_u + overlay.width // 2)
        y2 = min(image_h - 1, overlay.center_v + overlay.height // 2)
        color = (50, 210, 90) if route == "side_grasp" or overlay.orientation == "upright" else (0, 150, 255)
        if route == "stabilizing":
            color = (0, 220, 255)
        AutoCupFlowRouter._draw_corner_box(image, x1, y1, x2, y2, color)
        cv2.circle(image, (overlay.center_u, overlay.center_v), 5, color, -1)
        conf = "" if overlay.classifier_confidence is None else f" {overlay.classifier_confidence:.2f}"
        label = f"{overlay.orientation}{conf} -> {route}"
        AutoCupFlowRouter._draw_label(image, label, x1, max(30, y1 - 34), color)

    @staticmethod
    def _draw_corner_box(image: np.ndarray, x1: int, y1: int, x2: int, y2: int, color: tuple[int, int, int]) -> None:
        thickness = 3
        corner = max(18, min((x2 - x1) // 4, (y2 - y1) // 4, 44))
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 1)
        for start, end in [
            ((x1, y1), (x1 + corner, y1)),
            ((x1, y1), (x1, y1 + corner)),
            ((x2, y1), (x2 - corner, y1)),
            ((x2, y1), (x2, y1 + corner)),
            ((x1, y2), (x1 + corner, y2)),
            ((x1, y2), (x1, y2 - corner)),
            ((x2, y2), (x2 - corner, y2)),
            ((x2, y2), (x2, y2 - corner)),
        ]:
            cv2.line(image, start, end, color, thickness)

    @staticmethod
    def _draw_label(image: np.ndarray, text: str, x: int, y: int, color: tuple[int, int, int]) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.62
        thickness = 2
        (text_w, text_h), baseline = cv2.getTextSize(text, font, scale, thickness)
        x2 = min(image.shape[1] - 8, x + text_w + 18)
        y1 = max(8, y - text_h - 10)
        cv2.rectangle(image, (x, y1), (x2, y + baseline + 8), color, -1)
        cv2.putText(image, text, (x + 9, y), font, scale, (20, 20, 20), thickness)

    @staticmethod
    def _draw_text_panel(image: np.ndarray, route: str, status: str) -> None:
        panel = image.copy()
        margin = 14
        panel_h = 96
        cv2.rectangle(panel, (margin, margin), (image.shape[1] - margin, panel_h), (12, 16, 20), -1)
        cv2.addWeighted(panel, 0.78, image, 0.22, 0, image)

        route_color = (50, 210, 90) if route == "side_grasp" else (0, 150, 255)
        if route == "stabilizing":
            route_color = (0, 220, 255)
        AutoCupFlowRouter._draw_pill(image, route.replace("_", " ").upper(), margin + 14, 48, route_color)

        orientation = AutoCupFlowRouter._status_field(status, "orientation") or "waiting"
        cls = AutoCupFlowRouter._status_field(status, "class") or "cup"
        conf = AutoCupFlowRouter._status_float(status, "orientation_classifier_confidence")
        conf_text = "--" if conf is None else f"{conf:.2f}"
        detail = f"{cls} / {orientation} / classifier {conf_text}"
        cv2.putText(image, detail, (margin + 210, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (235, 245, 245), 2)

        compact_status = AutoCupFlowRouter._compact_status(status)
        cv2.putText(image, compact_status, (margin + 18, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (170, 225, 205), 1)

    @staticmethod
    def _draw_pill(image: np.ndarray, text: str, x: int, y: int, color: tuple[int, int, int]) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.56
        thickness = 2
        (text_w, text_h), baseline = cv2.getTextSize(text, font, scale, thickness)
        cv2.rectangle(image, (x, y - text_h - 12), (x + text_w + 28, y + baseline + 10), color, -1)
        cv2.putText(image, text, (x + 14, y), font, scale, (18, 22, 24), thickness)

    @staticmethod
    def _compact_status(status: str) -> str:
        parts = []
        for key in ["center", "orientation_classifier_result"]:
            value = AutoCupFlowRouter._status_field(status, key)
            if value:
                parts.append(f"{key}={value}")
        return "  ".join(parts) if parts else status[:120]

    def _run_side_grasp(self, decision: RouteDecision) -> bool:
        self.get_logger().info(f"route=side_grasp: launching existing side grasp flow ({decision.status})")
        cmd = self._launch_command(str(self.get_parameter("side_launch").value))
        cmd.extend([
            "auto_pick:=true",
            "grasp_mode:=side",
            "exit_after_pick:=true",
            "move_to_camera_home:=false",
            "skip_initial_home_move:=true",
            "return_home_after_task:=false",
            "return_to_camera_home_after_attempt:=false",
            "center_check_enabled:=false",
            "redetect_on_approach:=false",
            "verify_motion:=true",
            "side_fixed_grasp_z_enabled:=true",
            "side_fixed_grasp_z:=0.07",
            "side_project_bbox_center_to_fixed_z:=true",
            "min_motion_z:=0.07",
            "side_candidate_plan_check_enabled:=true",
            "side_far_stage_enabled:=false",
            "side_short_stage_backoff_m:=0.08",
            "side_approach_offset:=0.18",
            "side_grasp_stop_backoff_m:=0.04",
            "side_close_underreach_m:=0.03",
            "side_final_slide_enabled:=false",
            "side_move_to_initial_center_before_close:=false",
            "side_linear_approach_enabled:=true",
            "side_low_retry_lift_m:=0.03",
            "side_low_retry_attempts:=5",
            "workspace_xy_clamp_enabled:=false",
            "table_collision_enabled:=true",
            "table_surface_z:=0.0",
            "table_thickness:=0.04",
            "table_size_x:=1.10",
            "table_size_y:=0.65",
            "table_center_x:=0.29",
            "table_center_y:=0.0",
            "dispenser_collision_enabled:=true",
            f"moveit_controller_name:={self.get_parameter('moveit_controller_name').value}",
            f"side_target_x_offset_m:={float(self.get_parameter('side_target_x_offset_m').value)}",
            "start_joint_state_relay:=false",
            f"model_path:={self.get_parameter('yolo_model_path').value}",
        ])
        cmd.extend(self._split_extra_args(str(self.get_parameter("side_extra_args").value)))
        return self._run_process(cmd, "side_grasp")

    def _run_cup_uprighting(self, decision: RouteDecision) -> bool:
        self.get_logger().info(f"route=cup_uprighting: launching optimized cup-uprighting flow ({decision.status})")
        cmd = self._launch_command(str(self.get_parameter("cup_uprighting_launch").value))
        cmd.extend([
            "auto_pick:=true",
            "exit_after_pick:=true",
            "skip_initial_home_move:=true",
            f"model_path:={self.get_parameter('yolo_model_path').value}",
            f"moveit_controller_name:={self.get_parameter('moveit_controller_name').value}",
            f"controller_action_name:={self.get_parameter('controller_action_name').value}",
        ])
        cmd.extend(self._split_extra_args(str(self.get_parameter("cup_uprighting_extra_args").value)))
        return self._run_process(cmd, "cup_uprighting")

    def _run_color_scan_sequence(self) -> bool:
        if not bool(self.get_parameter("color_scan_at_start").value):
            self.get_logger().info("color_scan_at_start=false; skipping dispenser color scan")
            return True
        command = str(self.get_parameter("color_scan_command").value).strip()
        if not command:
            self.get_logger().warning("color_scan_command is empty; skipping dispenser color scan")
            return True
        self.get_logger().info("moving to color_scan_pose and scanning dispensers before cup pick")
        return self._run_process(["bash", "-c", command], "color_scan")

    def _run_recipe_sequence(self) -> bool:
        if not bool(self.get_parameter("recipe_after_success").value):
            self.get_logger().info("recipe_after_success=false; skipping dispenser recipe sequence")
            return True
        command = str(self.get_parameter("recipe_command").value).strip()
        if not command:
            self.get_logger().warning("recipe_command is empty; skipping dispenser recipe sequence")
            return True
        colors = str(self.get_parameter("recipe_colors").value).strip()
        if colors:
            command += f" --colors {shlex.quote(colors)}"
            self.get_logger().info(f"recipe colors given directly: {colors}")
        cup_pre_x = float(self.get_parameter("cup_pre_from_place_x_offset_m").value)
        command += f" --cup-pre-from-place-x-offset-m {cup_pre_x}"
        regrasp_z = float(self.get_parameter("final_regrasp_z_offset_m").value)
        place_z = float(self.get_parameter("cup_holder_place_z_offset_m").value)
        place_y = float(self.get_parameter("cup_holder_place_y_offset_m").value)
        z_min = float(self.get_parameter("cup_holder_z_min_m").value)
        command += (
            f" --final-regrasp-extra-z-offset-m {regrasp_z}"
            f" --cup-holder-place-final-z-offset-m {place_z}"
            f" --cup-holder-place-final-y-offset-m {place_y}"
            f" --cup-holder-z-min-m {z_min}"
        )
        self.get_logger().info("pick flow succeeded; starting integrated dispenser recipe sequence")
        return self._run_process(["bash", "-c", command], "recipe")

    def _run_lid_shake_sequence(self) -> bool:
        if not bool(self.get_parameter("lid_shake_after_recipe").value):
            self.get_logger().info("lid_shake_after_recipe=false; skipping lid close / shake chain")
            return True
        command = str(self.get_parameter("lid_shake_command").value).strip()
        if not command:
            self.get_logger().warning("lid_shake_command is empty; skipping lid close / shake chain")
            return True
        self.get_logger().info("recipe succeeded; starting lid close -> holder re-pick -> shake chain")
        return self._run_process(["bash", "-c", command], "lid_shake")

    def _move_observe(self, label: str) -> bool:
        prefix = str(self.get_parameter("service_prefix").value).strip().strip("/")
        base = f"/{prefix}/motion" if prefix else "/motion"
        service = f"{base}/move_joint"
        wait_service = f"{base}/move_wait"
        client = self.create_client(MoveJoint, service)
        timeout = float(self.get_parameter("motion_timeout_sec").value)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{label}: service unavailable: {service}")
            return False
        req = MoveJoint.Request()
        req.pos = [float(v) for v in self.get_parameter("observe_joints_deg").value]
        req.vel = float(self.get_parameter("observe_vel").value)
        req.acc = float(self.get_parameter("observe_acc").value)
        req.time = float(self.get_parameter("observe_time").value)
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        self.get_logger().info(f"{label}: MoveJoint via {service}: " + ", ".join(f"{v:.1f}" for v in req.pos))
        future = client.call_async(req)
        if not self._spin_future(future, timeout):
            self.get_logger().error(f"{label}: MoveJoint timed out after {timeout:.1f}s")
            return False
        if not bool(future.result().success):
            self.get_logger().error(f"{label}: MoveJoint returned failure")
            return False
        wait_client = self.create_client(MoveWait, wait_service)
        if wait_client.wait_for_service(timeout_sec=2.0):
            wait_future = wait_client.call_async(MoveWait.Request())
            if self._spin_future(wait_future, timeout) and bool(wait_future.result().success):
                self.get_logger().info(f"{label}: MoveWait completed")
        return True

    def _open_gripper(self, label: str) -> bool:
        service = str(self.get_parameter("gripper_open_service").value)
        client = self.create_client(Trigger, service)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{label}: service unavailable: {service}")
            return False
        self.get_logger().info(f"{label}: opening RG2 via {service}")
        future = client.call_async(Trigger.Request())
        if not self._spin_future(future, 8.0):
            self.get_logger().error(f"{label}: RG2 open timed out")
            return False
        result = future.result()
        if not bool(result.success):
            self.get_logger().error(f"{label}: RG2 open failed: {result.message}")
            return False
        self.get_logger().info(f"{label}: {result.message}")
        return True

    def _spin_future(self, future, timeout_sec: float) -> bool:
        end_time = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    @staticmethod
    def _launch_command(spec: str) -> list[str]:
        parts = spec.split()
        if len(parts) != 2:
            raise ValueError(f"launch spec must be '<package> <launch.py>': {spec!r}")
        return ["ros2", "launch", parts[0], parts[1]]

    @staticmethod
    def _split_extra_args(raw: str) -> list[str]:
        return [part for part in raw.split() if part]

    @staticmethod
    def _subprocess_env() -> dict[str, str]:
        # 다른 워크스페이스(예: ~/ros2_ws)에 같은 이름의 stale 패키지가 있으면
        # 터미널 source 순서에 따라 자식 launch가 엉뚱한 사본을 잡을 수 있다.
        # 이 라우터가 설치된 워크스페이스의 경로를 검색 변수 맨 앞으로 올려서
        # 자식 프로세스가 항상 같은 워크스페이스의 패키지를 먼저 찾게 한다.
        env = os.environ.copy()
        try:
            from ament_index_python.packages import get_package_prefix
            # install/<pkg> 두 단계 위 = 워크스페이스 루트. symlink 설치는 egg-info가
            # build/<pkg>에 있으므로 install/만 올리면 entry point 탐색이 또 어긋난다.
            ws_root = os.path.dirname(os.path.dirname(get_package_prefix("azas_task_manager"))) + os.sep
        except Exception:
            return env
        for var in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "CMAKE_PREFIX_PATH",
                    "PYTHONPATH", "PATH", "LD_LIBRARY_PATH"):
            value = env.get(var)
            if not value:
                continue
            entries = value.split(os.pathsep)
            own = [e for e in entries if e.startswith(ws_root) or e + os.sep == ws_root]
            rest = [e for e in entries if e not in own]
            env[var] = os.pathsep.join(own + rest)
        return env

    def _popen(self, cmd: list[str], label: str) -> subprocess.Popen[str]:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,
            env=self._subprocess_env(),
        )
        self._children.append(proc)
        threading.Thread(target=self._forward_output, args=(proc, label), daemon=True).start()
        return proc

    def _run_process(self, cmd: list[str], label: str) -> bool:
        self.get_logger().info(f"{label}: " + " ".join(cmd))
        self._child_node_failures.pop(label, None)
        proc = self._popen(cmd, label)
        code = proc.wait()
        # ros2 launch는 내부 노드가 죽어도 exit code 0으로 끝나므로
        # 출력에서 감지한 노드 비정상 종료를 별도로 확인한다.
        failures = self._child_node_failures.pop(label, None)
        if failures:
            self.get_logger().error(f"{label}: node failure detected: " + "; ".join(failures))
            return False
        if code == 0:
            self.get_logger().info(f"{label}: completed successfully")
            return True
        self.get_logger().error(f"{label}: process exited with code {code}")
        return False

    _NODE_DIED_PATTERN = re.compile(r"\[ERROR\] \[(?P<node>[^\]]+)\]: process has died.*exit code (?P<code>-?\d+)")
    _SHUTDOWN_PATTERN = re.compile(r"sending signal 'SIG(INT|TERM)'|user interrupted with ctrl-c")

    def _forward_output(self, proc: subprocess.Popen[str], label: str) -> None:
        if proc.stdout is None:
            return
        # 단계별 출력을 파일로도 남겨 실패 시 터미널 스크롤백 없이 진단할 수 있게 한다.
        log_dir = "/tmp/azas_router_logs"
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, f"{label}_{time.strftime('%Y%m%d_%H%M%S')}_{proc.pid}.log")
        shutting_down = False
        with open(log_path, "w", encoding="utf-8", errors="replace") as log_file:
            for line in proc.stdout:
                text = line.rstrip()
                self.get_logger().info(f"{label}> {text}")
                log_file.write(text + "\n")
                if not shutting_down and self._SHUTDOWN_PATTERN.search(text):
                    shutting_down = True
                match = self._NODE_DIED_PATTERN.search(text)
                # launch 종료 신호 이후의 죽음(SIGINT 받은 KeyboardInterrupt 등)만 정상 정리로
                # 간주한다. 종료 신호 전이라면 음수 exit code(SIGSEGV -11 등)도 실패다.
                if match and not shutting_down:
                    self._child_node_failures.setdefault(label, []).append(
                        f"{match.group('node')} exit code {match.group('code')}")

    def _stop_process(self, proc: Optional[subprocess.Popen[str]], label: str) -> None:
        if proc is None or proc.poll() is not None:
            return
        self.get_logger().info(f"stopping {label}")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass

    def _stop_all_children(self) -> None:
        for proc in list(self._children):
            self._stop_process(proc, "child")

    def _destroy_window(self) -> None:
        if not self._window_enabled:
            return
        try:
            cv2.destroyWindow(str(self.get_parameter("window_name").value))
        except Exception:
            pass

    @staticmethod
    def _image_to_bgr(msg: Image) -> np.ndarray:
        dtype = np.uint8 if msg.encoding.lower() in {"rgb8", "bgr8", "8uc3"} else np.uint8
        channels = 3
        array = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, msg.width, channels))
        if msg.encoding.lower() == "rgb8":
            return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        return array.copy()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = AutoCupFlowRouter()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
