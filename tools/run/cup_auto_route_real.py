#!/usr/bin/env python3
"""Route a detected cup to real side-grip or cup-uprighting flow.

This script does not generate robot coordinates. It listens to the existing
camera/YOLO/classifier pipeline on /azas/cup_detection and then starts one of
the already-tested real robot flows.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from azas_interfaces.msg import CupDetection
from rclpy.node import Node


ROOT = Path(__file__).resolve().parents[2]


def bash_join(parts: list[str]) -> str:
    return " ".join(part for part in parts if part)


def source_prefix() -> str:
    return bash_join(
        [
            "cd /home/ssu/Azas &&",
            "source /opt/ros/humble/setup.bash &&",
            "if [ -f /home/ssu/ws_moveit/install/setup.bash ]; then source /home/ssu/ws_moveit/install/setup.bash; fi &&",
            "source /home/ssu/ros2_ws/install/setup.bash &&",
            "source /home/ssu/Azas/install/setup.bash",
        ]
    )


def perception_command(args: argparse.Namespace) -> str:
    return bash_join(
        [
            source_prefix(),
            "&&",
            "ros2 launch azas_bringup yolo_perception.launch.py",
            f"model_path:={args.yolo_model}",
            f"color_topic:={args.color_topic}",
            f"depth_topic:={args.depth_topic}",
            f"camera_info_topic:={args.camera_info_topic}",
            "confidence_threshold:=0.35",
            "target_class_names:=cup,tumbler,bottle,lid",
            "selection_policy:=largest_bbox",
            f"orientation_classifier_path:={args.classifier}",
            "orientation_classifier_arch:=resnet18",
            f"orientation_classifier_min_confidence:={args.classifier_min_confidence:.3f}",
            "orientation_classifier_device:=cpu",
            "orientation_classifier_pad:=0.25",
            "device:=cpu",
        ]
    )


def perception_viewer_command(args: argparse.Namespace) -> str:
    return bash_join(
        [
            source_prefix(),
            "&&",
            "DISPLAY=${DISPLAY:-:0}",
            "XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}",
            "python3 /home/ssu/Azas/tools/perception/view_cup_detection_overlay.py",
            f"--image-topic {args.color_topic}",
            f"--cup-detection-topic {args.cup_detection_topic}",
            "--window-name 'Azas upright/lying classifier'",
            "--width 1280",
            "--height 720",
        ]
    )


def side_grip_command(args: argparse.Namespace) -> str:
    auto_pick = "false" if args.manual_side_pick else "true"
    clean_prefix = str(args.service_prefix).strip().strip("/")
    controller_name = (
        f"/{clean_prefix}/dsr_moveit_controller"
        if clean_prefix
        else "/dsr_moveit_controller"
    )
    return bash_join(
        [
            source_prefix(),
            "&&",
            "export PYTHONPATH=/home/ssu/Azas/tools/run/python_compat:${PYTHONPATH:-} &&",
            "DISPLAY=${DISPLAY:-:0}",
            "XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}",
            "ros2 launch dsr_practice yolo_cup_pick_node.launch.py",
            f"model_path:={args.side_yolo_model}",
            "conf:=0.35",
            "imgsz:=640",
            "device:=cpu",
            "target_class:=cup",
            f"auto_pick:={auto_pick}",
            "auto_pick_interval:=3.0",
            "pick_depth_ratio:=0.55",
            "depth_patch_radius:=7",
            "min_depth_valid_ratio:=0.03",
            "min_depth_m:=0.15",
            "max_depth_m:=1.20",
            "redetect_on_approach:=false",
            "redetect_settle_sec:=0.5",
            "grasp_mode:=side",
            "side_grasp_axis:=y_axis",
            "side_grasp_direction:=1.0",
            "side_auto_direction_by_cup_y:=true",
            "side_candidate_plan_check_enabled:=true",
            "side_far_stage_enabled:=false",
            "side_approach_offset:=0.18",
            "side_short_stage_backoff_m:=0.08",
            "side_grasp_offset:=0.025",
            "side_grasp_z_offset:=0.05",
            "side_grasp_stop_backoff_m:=0.04",
            "side_close_underreach_m:=0.03",
            "side_low_retry_lift_m:=0.03",
            "side_low_retry_attempts:=5",
            "side_linear_approach_enabled:=true",
            "side_final_slide_enabled:=false",
            "side_fixed_grasp_z_enabled:=true",
            "side_fixed_grasp_z:=0.07",
            "side_project_bbox_center_to_fixed_z:=true",
            "side_orientation_mode:=approach",
            "side_tool_roll_deg:=0.0",
            "side_roll_deg:=0.0",
            "side_pitch_deg:=90.0",
            "side_yaw_deg:=0.0",
            "table_collision_enabled:=true",
            "table_surface_z:=0.0",
            "table_thickness:=0.04",
            "table_size_x:=1.10",
            "table_size_y:=0.65",
            "table_center_x:=0.29",
            "table_center_y:=0.0",
            "dispenser_collision_enabled:=true",
            "dispenser_collision_config_path:=/home/ssu/Azas/src/azas_bringup/config/measured_dispenser_collision.yaml",
            "dispenser_collision_publish_period_sec:=1.0",
            "dispenser_collision_publish_objects:=true",
            "dispenser_collision_publish_markers:=true",
            "center_check_enabled:=false",
            "center_check_settle_sec:=0.6",
            "center_check_x:=0.45",
            "center_check_y:=0.0",
            "center_check_z:=0.64",
            "side_prepose_enabled:=false",
            "side_prepose_split_z:=0.18",
            "side_move_to_initial_center_before_close:=false",
            "verify_motion:=true",
            "motion_verify_tolerance:=0.03",
            "joint_goal_tolerance_rad:=0.02",
            "move_to_camera_home:=true",
            "move_joint_home_before_camera_home:=false",
            "camera_home_mode:=joint",
            "camera_home_joint_1_deg:=3.0",
            "camera_home_joint_2_deg:=-12.7",
            "camera_home_joint_3_deg:=44.0",
            "camera_home_joint_4_deg:=-9.0",
            "camera_home_joint_5_deg:=133.0",
            "camera_home_joint_6_deg:=90.0",
            "camera_home_x:=0.45",
            "camera_home_y:=0.0",
            "camera_home_z:=0.64",
            "camera_home_search_max_z:=0.64",
            "camera_home_search_min_z:=0.54",
            "camera_home_search_step_z:=0.02",
            "min_motion_z:=0.07",
            "workspace_xy_clamp_enabled:=false",
            "return_home_after_task:=false",
            "return_to_camera_home_after_attempt:=true",
            "place_x:=0.45",
            "place_y:=0.0",
            "place_z:=0.30",
            f"moveit_controller_name:={controller_name}",
            "start_joint_state_relay:=true",
        ]
    )


def uprighting_command(args: argparse.Namespace) -> str:
    return bash_join(
        [
            source_prefix(),
            "&&",
            f"YOLO_MODEL_PATH={args.uprighting_yolo_model}",
            "YOLO_OBSERVE_OFFSET_X_M=-0.07",
            "YOLO_OBSERVE_OFFSET_Y_M=0.00",
            "YOLO_OBSERVE_SAFE_Z_M=0.55",
            "YOLO_MOUTH_UP_TOL_DEG=30",
            "YOLO_GRASP_LEFT_TOL_DEG=35",
            "YOLO_MOUTH_UP_MAX_ITERS=8",
            "YOLO_MOUTH_UP_OPPOSITE_THRESH_DEG=90",
            "YOLO_MOUTH_UP_OPPOSITE_MAX_STEP_DEG=95",
            "YOLO_MOUTH_UP_OPPOSITE_GAIN=0.80",
            "YOLO_GRASP_BODY_OFFSET_M=0.025",
            "/home/ssu/Azas/tools/run/run_yolo_cup_uprighting.sh",
            "preview_only:=false",
            "show_window:=true",
        ]
    )


def initial_observe_command(args: argparse.Namespace) -> str:
    service_prefix_args = []
    if str(args.service_prefix).strip():
        service_prefix_args = ["--service-prefix", str(args.service_prefix).strip()]
    return bash_join(
        [
            source_prefix(),
            "&&",
            "python3 /home/ssu/Azas/tools/run/direct_movej_joints.py",
            *service_prefix_args,
            "--j1 3.0",
            "--j2 -12.7",
            "--j3 44.0",
            "--j4 -9.0",
            "--j5 133.0",
            "--j6 90.0",
            "--velocity 10.0",
            "--acceleration 10.0",
            f"--wait-service-sec {args.observe_wait_service_sec:.1f}",
            f"--timeout-sec {args.observe_timeout_sec:.1f}",
            "--execute",
            "--confirm ENABLE_DIRECT_MOVEJ",
        ]
    )


def gripper_open_command() -> str:
    return bash_join(
        [
            source_prefix(),
            "&&",
            "/home/ssu/Azas/tools/run/rg2_full_open_verify.sh",
        ]
    )


class CupRouteObserver(Node):
    def __init__(self, topic: str, stable_count: int):
        super().__init__("cup_auto_route_observer")
        self.stable_count = max(1, int(stable_count))
        self.last_route = ""
        self.last_status = ""
        self.count = 0
        self.route = ""
        self.status = ""
        self.create_subscription(CupDetection, topic, self._on_detection, 10)

    def _on_detection(self, msg: CupDetection) -> None:
        status = msg.status or ""
        route = route_from_status(status)
        if route == "unknown":
            self.get_logger().info(
                "[Route] 아직 실행 조건이 아닙니다. "
                f"컵 상태를 계속 확인합니다: {status}"
            )
            return
        if route == self.last_route:
            self.count += 1
        else:
            self.last_route = route
            self.count = 1
        self.last_status = status
        action = "실제 side grip 자동 실행" if route == "side_grip" else "cup_uprighting 실제 모션 실행"
        self.get_logger().info(
            f"[Route] 후보={route} 안정도={self.count}/{self.stable_count}. "
            f"조건 충족 시 다음 실행: {action}. status={status}"
        )
        if self.count >= self.stable_count:
            self.route = route
            self.status = status


def route_from_status(status: str) -> str:
    normalized = status.strip().lower()
    if normalized.startswith("detected:upright"):
        return "side_grip"
    if normalized.startswith("rejected:lying"):
        return "cup_uprighting"
    return "unknown"


def start_process(cmd: str, label: str) -> subprocess.Popen:
    print(f"[Azas route] start {label}:\n{cmd}", flush=True)
    return subprocess.Popen(["bash", "-lc", cmd], cwd=str(ROOT), preexec_fn=os.setsid)


def stop_process(proc: subprocess.Popen | None, label: str) -> None:
    if proc is None or proc.poll() is not None:
        return
    print(f"[Azas route] stop {label}", flush=True)
    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=5.0)


def wait_for_route(args: argparse.Namespace) -> tuple[str, str]:
    rclpy.init()
    node = CupRouteObserver(args.cup_detection_topic, args.stable_count)
    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.route:
                return node.route, node.status
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return "", ""


def validate_paths(args: argparse.Namespace) -> None:
    required = [
        ("YOLO model", args.yolo_model),
        ("orientation classifier", args.classifier),
        ("side-grip YOLO model", args.side_yolo_model),
        ("cup-uprighting YOLO model", args.uprighting_yolo_model),
    ]
    missing = [f"{label}: {path}" for label, path in required if not Path(path).exists()]
    if missing:
        raise FileNotFoundError("missing required model files:\n" + "\n".join(missing))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Route cup to real side grip or cup uprighting.")
    parser.add_argument("--execute", action="store_true", help="Actually launch the selected real motion flow.")
    parser.add_argument(
        "--skip-initial-observe",
        action="store_true",
        help="Skip the initial observe joint move and RG2 full-open step.",
    )
    parser.add_argument(
        "--manual-side-pick",
        action="store_true",
        help="Keep side grip in supervised mode with auto_pick:=false. Default is fully automatic.",
    )
    parser.add_argument("--start-detector", action="store_true", default=True)
    parser.add_argument("--no-start-detector", dest="start_detector", action="store_false")
    parser.add_argument("--show-perception-window", action="store_true", default=True)
    parser.add_argument("--no-show-perception-window", dest="show_perception_window", action="store_false")
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    parser.add_argument("--stable-count", type=int, default=2)
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--observe-wait-service-sec", type=float, default=30.0)
    parser.add_argument("--observe-timeout-sec", type=float, default=30.0)
    parser.add_argument("--cup-detection-topic", default="/azas/cup_detection")
    parser.add_argument("--yolo-model", default="/home/ssu/Azas/local_models/best.pt")
    parser.add_argument("--side-yolo-model", default="/home/ssu/Azas/local_models/best.pt")
    parser.add_argument("--uprighting-yolo-model", default="/home/ssu/yolo_cup_uprighting/best.pt")
    parser.add_argument("--classifier", default="/home/ssu/Azas/cup_classifier_best.pth")
    parser.add_argument("--classifier-min-confidence", type=float, default=0.70)
    parser.add_argument("--color-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    validate_paths(args)

    if not args.skip_initial_observe:
        print(
            "[Azas route] 시작 준비: observe 위치로 먼저 이동하고 RG2를 full-open 합니다.",
            flush=True,
        )
        observe_cmd = initial_observe_command(args)
        open_cmd = gripper_open_command()
        if args.execute:
            print("[Azas route] observe 이동 실행: camera_home joint observe.", flush=True)
            completed = subprocess.run(["bash", "-lc", observe_cmd], cwd=str(ROOT), check=False)
            if completed.returncode != 0:
                print(
                    f"[Azas route] observe 이동 실패. returncode={completed.returncode}. "
                    "인식/모션 라우팅을 시작하지 않습니다.",
                    flush=True,
                )
                return int(completed.returncode)
            print("[Azas route] RG2 full-open 실행.", flush=True)
            completed = subprocess.run(["bash", "-lc", open_cmd], cwd=str(ROOT), check=False)
            if completed.returncode != 0:
                print(
                    f"[Azas route] RG2 full-open 실패. returncode={completed.returncode}. "
                    "인식/모션 라우팅을 시작하지 않습니다.",
                    flush=True,
                )
                return int(completed.returncode)
        else:
            print("[Azas route] dry run: observe 이동 명령:", flush=True)
            print(observe_cmd, flush=True)
            print("[Azas route] dry run: RG2 full-open 명령:", flush=True)
            print(open_cmd, flush=True)
    else:
        print("[Azas route] --skip-initial-observe: 초기 observe 이동/RG2 open을 생략합니다.", flush=True)

    detector = None
    viewer = None
    if args.start_detector:
        print(
            "[Azas route] 컵/뚜껑 탐지와 upright/lying classifier를 시작합니다. "
            f"YOLO={args.yolo_model}, classifier={args.classifier}",
            flush=True,
        )
        detector = start_process(perception_command(args), "cup/lid detector + orientation classifier")
        if args.show_perception_window:
            print(
                "[Azas route] upright/lying 판별 확인용 카메라 overlay 창을 띄웁니다.",
                flush=True,
            )
            viewer = start_process(perception_viewer_command(args), "upright/lying perception viewer")
        time.sleep(2.0)

    try:
        print(
            f"[Azas route] /azas/cup_detection에서 안정적인 판별을 기다립니다 "
            f"(stable_count={args.stable_count}, timeout={args.timeout_sec:.1f}s).",
            flush=True,
        )
        route, status = wait_for_route(args)
    finally:
        stop_process(viewer, "upright/lying perception viewer")
        stop_process(detector, "cup/lid detector")

    if not route:
        print(
            f"[Azas route] no stable upright/lying decision within {args.timeout_sec:.1f}s; no motion launched",
            flush=True,
        )
        return 2

    if route == "side_grip":
        print(
            "[Azas route] 판별 결과: upright 컵입니다. "
            "실제 side grip 자동 실행 경로로 넘어갑니다.",
            flush=True,
        )
    else:
        print(
            "[Azas route] 판별 결과: lying 컵입니다. "
            "cup_uprighting 실제 모션 경로로 넘어갑니다.",
            flush=True,
        )
    print(f"[Azas route] selected={route} status={status}", flush=True)
    selected_cmd = side_grip_command(args) if route == "side_grip" else uprighting_command(args)
    if not args.execute:
        print("[Azas route] dry run only. Selected command:", flush=True)
        print(selected_cmd, flush=True)
        return 0

    print("[Azas route] 선택된 실제 실행 명령을 시작합니다.", flush=True)
    completed = subprocess.run(["bash", "-lc", selected_cmd], cwd=str(ROOT), check=False)
    print(f"[Azas route] 선택된 실행이 종료되었습니다. returncode={completed.returncode}", flush=True)
    return int(completed.returncode)


if __name__ == "__main__":
    sys.exit(main())
