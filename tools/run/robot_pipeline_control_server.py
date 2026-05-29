#!/usr/bin/env python3
"""Local button panel for supervised Azas robot pipeline commands."""

from __future__ import annotations

import json
import os
import re
import shlex
import signal
import subprocess
import time
from dataclasses import asdict, dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

try:
    import psutil
except ImportError:  # pragma: no cover - local panel can still run without tree cleanup.
    psutil = None


ROOT = Path(__file__).resolve().parents[2]
HTML_PATH = ROOT / "docs" / "robot_pipeline_control.html"
ROS_SETUP = (
    "source /opt/ros/humble/setup.bash && "
    "mkdir -p /tmp/azas_ros_logs && export ROS_LOG_DIR=/tmp/azas_ros_logs && "
    "if [ -f /home/ssu/ws_moveit/install/setup.bash ]; then "
    "source /home/ssu/ws_moveit/install/setup.bash; "
    "fi && "
    "if [ -f /home/ssu/ros2_ws/install/setup.bash ]; then "
    "source /home/ssu/ros2_ws/install/setup.bash; "
    "fi && "
    f"if [ -f {shlex.quote(str(ROOT / 'install' / 'setup.bash'))} ]; then "
    f"source {shlex.quote(str(ROOT / 'install' / 'setup.bash'))}; "
    "else "
    f"source {shlex.quote(str(ROOT / 'install' / 'local_setup.bash'))}; "
    "fi && "
    f"export PYTHONPATH={shlex.quote(str(ROOT / 'tools' / 'run' / 'python_compat'))}:${{PYTHONPATH:-}}"
)
DEFAULT_ROBOT_HOST = "192.168.1.100"
DEFAULT_ROS_DOMAIN_ID = "9"
DEFAULT_YOLO_MODEL_PATH = ROOT / "local_models" / "best.pt"
PR20_YOLO_MODEL_PATH = Path("/home/ssu/Azas/best.pt")
DEFAULT_DISPENSER_TCP_NAME = "GripperDA_v1_jarvis"
FAST_MOVE_VELOCITY = "30"
FAST_MOVE_ACCELERATION = "30"
RVIZ_PREVIEW_ROS_DOMAIN_ID = "79"
BACKGROUND_LOG_DIR = ROOT / "log" / "panel"
ROBOT_STATE_NAMES = {
    0: "STATE_INITIALIZING",
    1: "STATE_STANDBY",
    2: "STATE_MOVING",
    3: "STATE_SAFE_OFF",
    4: "STATE_TEACHING",
    5: "STATE_SAFE_STOP",
    6: "STATE_EMERGENCY_STOP",
    7: "STATE_HOMMING",
    8: "STATE_RECOVERY",
    9: "STATE_SAFE_STOP2",
    10: "STATE_SAFE_OFF2",
    15: "STATE_NOT_READY",
}
CAMERA_TABLE_VIEW_JOINTS = {
    "j1": "0",
    "j2": "-5",
    "j3": "50",
    "j4": "0",
    "j5": "135",
    "j6": "0",
}
DISPENSER_PRESS_TARGETS = {
    "1": "red",
    "2": "green",
    "3": "yellow",
    "4": "blue",
}


@dataclass(frozen=True)
class Step:
    key: str
    label: str
    kind: str
    command: str
    implemented: bool
    real_motion: bool
    note: str


STEPS = [
    Step(
        "connect_robot",
        "로봇 연결 / 스마트 재연결",
        "background",
        "tools/run/run_doosan_real_no_motion_m0609.sh",
        True,
        False,
        "준비됨/시작중이면 유지하고, stale 상태일 때만 정리 후 시작",
    ),
    Step("status_check", "연결 확인", "run", "ros2 service list | grep /dsr01/motion", True, False, "명령 후보만 있음: /dsr01/motion 서비스가 보여야 통과"),
    Step("connect_gripper", "그리퍼 연결", "background", "ros2 launch azas_gripper rg2_trigger.launch.py", True, False, "RG2 Trigger 서비스(/jarvis/rg2/open, close, set_width) 시작"),
    Step("start_camera", "RealSense 카메라 시작", "background", "ros2 launch realsense2_camera rs_launch.py", True, False, "RealSense 드라이버와 color/aligned-depth 토픽 시작; 화면 창은 별도 버튼 사용"),
    Step("start_camera_view", "RealSense 컬러 화면 보기", "background", "rqt_image_view /camera/camera/color/image_raw", True, False, "카메라 color image 토픽을 rqt_image_view 창으로 표시"),
    Step("detect_cup_lid", "컵/뚜껑 탐지 토픽 시작", "background", "ros2 launch azas_bringup yolo_perception.launch.py", True, False, "YOLO 탐지 결과를 /azas/cup_detection으로 publish; 이 노드는 화면 창을 띄우지 않음"),
    Step(
        "start_collision_scene",
        "MoveIt 충돌 장면 시작",
        "background",
        "measured_dispenser_collision_scene_node + tumbler_collision_scene_node",
        True,
        False,
        "디스펜서 박스와 감지 텀블러를 /collision_object로 publish; direct Doosan 명령은 아직 이 장면을 자동 회피에 쓰지 않음",
    ),
    Step("home_robot", "로봇 원위치 / HOME", "run", "tools/run/direct_movej_joints.py --j1 0 --j2 0 --j3 90 --j4 0 --j5 90 --j6 0", True, True, "실제모션 후보: HOME 관절값 [0, 0, 90, 0, 90, 0]"),
    Step(
        "lift_robot",
        "카메라 테이블 보기 자세 / J5 안전",
        "run",
        "tools/run/direct_movej_joints.py --j1 0 --j2 -5 --j3 50 --j4 0 --j5 135 --j6 0",
        True,
        True,
        "MoveLine IK 대신 실측 관절 자세 사용: joint_2=-5°, joint_3=50°, joint_5=135° 상한으로 테이블 보기",
    ),
    Step("voice_input", "음성 입력", "run", "ros2 launch azas_voice azas_voice.launch.py", True, False, "STT/레시피 노드"),
    Step("recipe_generate", "레시피 생성", "blocked", "", False, False, "음성/레시피 토픽 통합 버튼은 별도 연결 필요"),
    Step(
        "side_grip",
        "PR #20 RealSense 컵 인식 후 side grip",
        "background",
        "ros2 launch dsr_practice yolo_cup_pick_node.launch.py auto_pick:=false grasp_mode:=side",
        True,
        True,
        "PR #20 merged manual side-grip flow: 카메라 창에서 cup 탐지 후 p 키로 side-grip 실행",
    ),
    Step("gripper_soft_grasp", "그리퍼 살짝 잡기", "run", "ros2 service call /jarvis/rg2/set_width azas_interfaces/srv/SetGripper", True, True, "큰 컵용: 완전 close 대신 폭 75mm/약한 힘으로 살짝 오므림"),
    Step("gripper_open", "그리퍼 full open / 컵 놓기 검증", "run", "tools/run/rg2_full_open_verify.sh", True, True, "컵을 배출구 아래에 둔 뒤 RG2 full-open 명령 success=True 검증"),
    Step(
        "move_to_dispenser_1",
        "고정 디스펜서 1 배출구 아래로 컵 이동",
        "run",
        "tools/run/move_to_measured_dispenser_front_hold.py --dispenser-id 1",
        True,
        True,
        "실제모션 후보: front_hold_poses.dispenser_1 좌표 사용; 이동/검증 성공 후 RG2 full-open success 검증",
    ),
    Step(
        "move_to_dispenser_2",
        "고정 디스펜서 2 배출구 아래로 컵 이동",
        "run",
        "tools/run/move_to_measured_dispenser_front_hold.py --dispenser-id 2",
        True,
        True,
        "실제모션 후보: front_hold_poses.dispenser_2 좌표 사용; 이동/검증 성공 후 RG2 full-open success 검증",
    ),
    Step(
        "move_to_dispenser_3",
        "고정 디스펜서 3 배출구 아래로 컵 이동",
        "run",
        "tools/run/move_to_measured_dispenser_front_hold.py --dispenser-id 3",
        True,
        True,
        "실제모션 후보: front_hold_poses.dispenser_3 좌표 사용; 이동/검증 성공 후 RG2 full-open success 검증",
    ),
    Step(
        "move_to_dispenser_4",
        "고정 디스펜서 4 배출구 아래로 컵 이동",
        "run",
        "tools/run/move_to_measured_dispenser_front_hold.py --dispenser-id 4",
        True,
        True,
        "실제모션 후보: front_hold_poses.dispenser_4 좌표 사용; 이동/검증 성공 후 RG2 full-open success 검증",
    ),
    Step(
        "press_dispenser_1",
        "디스펜서 1 누르기 / red",
        "run",
        "ros2 run azas_dispenser dispenser_press_node --ros-args -p target_dispenser:=red",
        True,
        True,
        "feature/dispenser 원본 taught posx red 경로 사용: 컵 놓기 후 뒤로 후퇴→HOME 이동→RG2 full-close→transit→press→retreat→HOME 복귀",
    ),
    Step(
        "press_dispenser_2",
        "디스펜서 2 누르기 / green",
        "run",
        "ros2 run azas_dispenser dispenser_press_node --ros-args -p target_dispenser:=green",
        True,
        True,
        "feature/dispenser 원본 taught posx green 경로 사용: 컵 놓기 후 뒤로 후퇴→HOME 이동→RG2 full-close→transit→press→retreat→HOME 복귀",
    ),
    Step(
        "press_dispenser_3",
        "디스펜서 3 누르기 / yellow",
        "run",
        "ros2 run azas_dispenser dispenser_press_node --ros-args -p target_dispenser:=yellow",
        True,
        True,
        "feature/dispenser 원본 taught posx yellow 경로 사용: 컵 놓기 후 뒤로 후퇴→HOME 이동→RG2 full-close→transit→press→retreat→HOME 복귀",
    ),
    Step(
        "press_dispenser_4",
        "디스펜서 4 누르기 / blue",
        "run",
        "ros2 run azas_dispenser dispenser_press_node --ros-args -p target_dispenser:=blue",
        True,
        True,
        "feature/dispenser 원본 taught posx blue 경로 사용: 컵 놓기 후 뒤로 후퇴→HOME 이동→RG2 full-close→transit→press→retreat→HOME 복귀",
    ),
    Step(
        "pick_from_dispenser_1",
        "디스펜서 1 앞 컵 다시 잡기 / side grip",
        "run",
        "tools/run/pick_from_measured_dispenser_front_hold.py --dispenser-id 1",
        True,
        True,
        "front_hold_poses.dispenser_1 재사용: RG2 open→측정 front-hold 접근→soft side-grip→수직 lift",
    ),
    Step(
        "pick_from_dispenser_2",
        "디스펜서 2 앞 컵 다시 잡기 / side grip",
        "run",
        "tools/run/pick_from_measured_dispenser_front_hold.py --dispenser-id 2",
        True,
        True,
        "front_hold_poses.dispenser_2 재사용: RG2 open→측정 front-hold 접근→soft side-grip→수직 lift",
    ),
    Step(
        "pick_from_dispenser_3",
        "디스펜서 3 앞 컵 다시 잡기 / side grip",
        "run",
        "tools/run/pick_from_measured_dispenser_front_hold.py --dispenser-id 3",
        True,
        True,
        "front_hold_poses.dispenser_3 재사용: RG2 open→측정 front-hold 접근→soft side-grip→수직 lift",
    ),
    Step(
        "pick_from_dispenser_4",
        "디스펜서 4 앞 컵 다시 잡기 / side grip",
        "run",
        "tools/run/pick_from_measured_dispenser_front_hold.py --dispenser-id 4",
        True,
        True,
        "front_hold_poses.dispenser_4 재사용: RG2 open→측정 front-hold 접근→soft side-grip→수직 lift",
    ),
    Step(
        "run_dispenser_recipe_sequence",
        "레시피 디스펜서 통합 실행 / move→press→pick",
        "run",
        "tools/run/run_measured_dispenser_recipe_sequence.py --dispenser-ids 1,2,3,4",
        True,
        True,
        "설정의 RECIPE_DISPENSER_IDS 순서대로 실행. 컵 이동/놓기와 다시 side-grip 집기는 통합 ROS 클라이언트로 처리해 반복 명령 실행 시간을 줄임",
    ),
    Step("repeat_dispense", "5,6 반복", "blocked", "", False, True, "레시피별 디스펜서 ID 반복 로직 필요"),
    Step("pick_lid", "뚜껑을 집기", "blocked", "", False, True, "뚜껑 좌표/그리퍼 폭 필요"),
    Step(
        "place_cup_holder",
        "컵을 컵홀더에 놓기 / side grip",
        "run",
        "tools/run/place_side_grip_cup_in_holder.py",
        True,
        True,
        "실제모션 후보: 측정된 side_grip_place pre_place→place_final→RG2 full-open→retreat",
    ),
    Step("attach_lid", "뚜껑을 컵에 끼우기", "blocked", "", False, True, "뚜껑 체결 동작 미구현"),
    Step(
        "shake_rviz_preview",
        "쉐이킹 RViz 미리보기 / 무모션",
        "background",
        "tools/run/run_rule_based_dispenser_then_shake_sim.sh",
        True,
        False,
        "실제 로봇 미사용: 별도 ROS_DOMAIN_ID에서 쉐이킹 궤적/마커를 RViz로 표시",
    ),
    Step("shake_closed_cup", "컵홀더 컵 다시 잡기 후 쉐이킹", "run", "tools/run/pick_from_cup_holder_side_grip.py && tools/run/run_rule_based_shake_real.sh", True, True, "시작 시 컵홀더에 놓인 닫힌 컵을 측정된 cup_holder.side_grip_place pose로 다시 side-grip 픽업한 뒤, J3 양수 고정 및 J4/J5/J6 트위스트 쉐이킹을 실행"),
    Step("remove_lid", "뚜껑을 열기/제거하기", "blocked", "", False, True, "뚜껑 제거 동작 미구현"),
    Step("pour_cocktail", "칵테일을 다른 컵에 붓기", "blocked", "", False, True, "따르기 경로 미구현"),
]

processes: dict[str, subprocess.Popen[str]] = {}
process_logs: dict[str, Path] = {}

DOOSAN_STACK_PATTERNS = (
    "run_doosan_real_no_motion_m0609.sh",
    "run_emulator",
    "dsr_bringup2/lib/dsr_bringup2",
    "dsr_bringup2_moveit.launch.py",
    "dsr_controller2",
    "dsr_moveit_controller",
    "ros2_control_node",
    "controller_manager",
    "joint_state_broadcaster",
    "robot_state_publisher",
    "virtual_node",
    "move_group",
    "moveit_simple_controller_manager",
    "dsr_moveit_config_m0609",
)

AUXILIARY_STACK_PATTERNS = (
    "rg2_gripper_node",
    "yolo_perception.launch.py",
    "azas_voice.launch.py",
    "rqt_image_view",
    "run_rule_based_shake_real.sh",
    "run_cup_target_then_shake_rviz.sh",
    "cup_target_then_shake_rviz.launch.py",
    "run_rule_based_dispenser_then_shake_sim.sh",
    "tumbler_shake_sequence.launch.py",
    "tumbler_shake_sequence_node",
    "shake_visualizer_node",
    "m0609_shake_joint_state_node",
    "measured_dispenser_collision_scene_node",
    "tumbler_collision_scene_node",
)

RG2_STACK_PATTERNS = (
    "rg2_gripper_node",
)

CAMERA_STACK_PATTERNS = (
    "realsense2_camera rs_launch.py",
    "realsense2_camera_node",
)

SIDE_GRIP_STACK_PATTERNS = (
    "yolo_cup_pick_node.launch.py",
    "yolo_cup_pick_node_legacy.launch.py",
    "yolo_cup_pick_legacy_node",
    "dsr_practice/yolo_cup_pick_node",
    "yolo_cup_pick_node --ros-args",
    "yolo_cup_pick_moveit_py",
    "joint_state_relay_legacy",
    "dsr_practice/joint_state_relay",
    "joint_state_relay --ros-args",
    "measured_dispenser_collision_scene_node",
)

RUN_STEP_STACK_PATTERNS = (
    "dispenser_press_node",
    "direct_movej_joints.py",
    "rg2_full_open_verify.sh",
    "move_to_measured_dispenser_front_hold.py",
    "pick_from_measured_dispenser_front_hold.py",
    "run_measured_dispenser_recipe_sequence.py",
    "place_side_grip_cup_in_holder.py",
    "pick_from_cup_holder_side_grip.py",
    "ros2 service call /dsr01/",
    "ros2 control list_controllers",
)

PANEL_PROTECTED_PATTERNS = (
    "robot_pipeline_control_server.py",
    "run_robot_pipeline_control_panel.sh",
)


def command_line(proc: Any) -> str:
    try:
        cmdline = proc.info.get("cmdline") if hasattr(proc, "info") else proc.cmdline()
    except Exception:
        return ""
    if not cmdline:
        return ""
    return " ".join(str(part) for part in cmdline)


def tail_file(path: Path | None, *, max_chars: int = 8000) -> str:
    if path is None or not path.exists():
        return ""
    try:
        with path.open("rb") as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            handle.seek(max(0, size - max_chars), os.SEEK_SET)
            data = handle.read()
    except OSError as exc:
        return f"[Azas] failed to read log {path}: {exc}"
    return data.decode("utf-8", errors="replace")


def background_log_path(step_key: str) -> Path:
    BACKGROUND_LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return BACKGROUND_LOG_DIR / f"{step_key}-{stamp}.log"


def terminate_process_tree(proc: subprocess.Popen[str], *, label: str, grace_sec: float = 3.0) -> list[str]:
    """Terminate a Popen process and its children without killing the panel server."""
    events: list[str] = []
    if proc.poll() is not None:
        return events
    if psutil is not None:
        try:
            root = psutil.Process(proc.pid)
            targets = root.children(recursive=True) + [root]
            for target in targets:
                if target.pid == os.getpid():
                    continue
                events.append(f"{label}: terminate pid={target.pid} cmd={command_line(target)[:160]}")
                target.terminate()
            _, alive = psutil.wait_procs(targets, timeout=grace_sec)
            for target in alive:
                if target.pid == os.getpid():
                    continue
                events.append(f"{label}: kill pid={target.pid} cmd={command_line(target)[:160]}")
                target.kill()
            return events
        except psutil.Error as exc:
            events.append(f"{label}: psutil tree cleanup failed: {exc}")

    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=grace_sec)
    except subprocess.TimeoutExpired:
        proc.kill()
        events.append(f"{label}: killed pid={proc.pid}")
    else:
        events.append(f"{label}: stopped pid={proc.pid}")
    return events


def cleanup_doosan_stack(*, grace_sec: float = 3.0) -> list[str]:
    """Best-effort cleanup of stale Doosan/MoveIt graph processes before reconnect."""
    events: list[str] = []
    old = processes.pop("connect_robot", None)
    if old is not None:
        events.extend(terminate_process_tree(old, label="stored connect_robot", grace_sec=grace_sec))

    if psutil is None:
        return events

    current_pid = os.getpid()
    candidates: list[Any] = []
    for proc in psutil.process_iter(["pid", "cmdline", "name"]):
        if proc.pid == current_pid:
            continue
        cmd = command_line(proc)
        if not cmd:
            continue
        if any(protected in cmd for protected in PANEL_PROTECTED_PATTERNS):
            continue
        if any(pattern in cmd for pattern in DOOSAN_STACK_PATTERNS):
            candidates.append(proc)

    if not candidates:
        events.append("cleanup: no stale Doosan/MoveIt processes found")
        return events

    for proc in candidates:
        events.append(f"cleanup: terminate pid={proc.pid} cmd={command_line(proc)[:160]}")
        try:
            proc.terminate()
        except psutil.Error as exc:
            events.append(f"cleanup: terminate failed pid={proc.pid}: {exc}")

    _, alive = psutil.wait_procs(candidates, timeout=grace_sec)
    for proc in alive:
        try:
            events.append(f"cleanup: kill pid={proc.pid} cmd={command_line(proc)[:160]}")
            proc.kill()
        except psutil.Error as exc:
            events.append(f"cleanup: kill failed pid={proc.pid}: {exc}")

    return events


def cleanup_matching_processes(
    patterns: tuple[str, ...],
    *,
    label: str,
    grace_sec: float = 3.0,
) -> list[str]:
    """Terminate panel-related stale processes that are not tracked in this process."""
    events: list[str] = []
    if psutil is None:
        events.append(f"{label}: psutil unavailable; only tracked panel processes can be stopped")
        return events

    current_pid = os.getpid()
    candidates: list[Any] = []
    seen: set[int] = set()
    for proc in psutil.process_iter(["pid", "cmdline", "name"]):
        if proc.pid == current_pid or proc.pid in seen:
            continue
        cmd = command_line(proc)
        if not cmd:
            continue
        if any(protected in cmd for protected in PANEL_PROTECTED_PATTERNS):
            continue
        if any(pattern in cmd for pattern in patterns):
            candidates.append(proc)
            seen.add(proc.pid)

    if not candidates:
        events.append(f"{label}: no matching stale processes found")
        return events

    for proc in candidates:
        try:
            events.append(f"{label}: terminate pid={proc.pid} cmd={command_line(proc)[:160]}")
            proc.terminate()
        except psutil.Error as exc:
            events.append(f"{label}: terminate failed pid={proc.pid}: {exc}")

    _, alive = psutil.wait_procs(candidates, timeout=grace_sec)
    for proc in alive:
        try:
            events.append(f"{label}: kill pid={proc.pid} cmd={command_line(proc)[:160]}")
            proc.kill()
        except psutil.Error as exc:
            events.append(f"{label}: kill failed pid={proc.pid}: {exc}")

    return events


def cleanup_rg2_stack(*, grace_sec: float = 2.0) -> list[str]:
    """Best-effort cleanup of stale RG2 bridge nodes before reconnect.

    `/jarvis/rg2/set_width` returning success only proves the ROS wrapper accepted
    the request; the current RG2 bridge does not expose real finger feedback.  A
    stale wrapper can therefore make panel steps look successful while the
    physical gripper does not move.  Reconnecting the gripper step should always
    replace old RG2 wrappers instead of trusting an existing service name.
    """
    events: list[str] = []
    old = processes.pop("connect_gripper", None)
    if old is not None:
        events.extend(terminate_process_tree(old, label="stored connect_gripper", grace_sec=grace_sec))
    events.extend(cleanup_matching_processes(RG2_STACK_PATTERNS, label="rg2 cleanup", grace_sec=grace_sec))
    return events


def cleanup_camera_stack(*, grace_sec: float = 2.0) -> list[str]:
    """Best-effort cleanup of stale RealSense drivers before restart."""
    events: list[str] = []
    old = processes.pop("start_camera", None)
    if old is not None:
        events.extend(terminate_process_tree(old, label="stored start_camera", grace_sec=grace_sec))
    events.extend(cleanup_matching_processes(CAMERA_STACK_PATTERNS, label="camera cleanup", grace_sec=grace_sec))
    return events


def cleanup_side_grip_stack(*, grace_sec: float = 2.0) -> list[str]:
    """Best-effort cleanup of stale one-shot side-grip processes before retry."""
    events: list[str] = []
    old = processes.pop("side_grip", None)
    if old is not None:
        events.extend(terminate_process_tree(old, label="stored side_grip", grace_sec=grace_sec))
    events.extend(cleanup_matching_processes(SIDE_GRIP_STACK_PATTERNS, label="side_grip cleanup", grace_sec=grace_sec))
    return events


def cleanup_run_step_stack(*, grace_sec: float = 3.0) -> list[str]:
    """Best-effort cleanup of stale one-shot motion/ROS CLI commands.

    These commands are normally launched as blocking panel steps.  If a timeout,
    browser refresh, or operator interrupt leaves a child process alive, the next
    panel run can observe old services/actions/nodes and behave inconsistently.
    The explicit cleanup button therefore treats these as robot-stack residue.
    """
    return cleanup_matching_processes(
        RUN_STEP_STACK_PATTERNS,
        label="run-step cleanup",
        grace_sec=grace_sec,
    )


def stop_ros2_daemon() -> list[str]:
    """Stop the ROS 2 CLI daemon so cleanup starts the next run from a fresh graph cache."""
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(env.get("AZAS_PANEL_ROS_DOMAIN_ID") or env.get("ROS_DOMAIN_ID") or DEFAULT_ROS_DOMAIN_ID)
    env["ROS_LOCALHOST_ONLY"] = str(env.get("ROS_LOCALHOST_ONLY") or "0")
    try:
        completed = subprocess.run(
            ["bash", "-lc", f"{ROS_SETUP} && ros2 daemon stop"],
            cwd=str(ROOT),
            env=env,
            text=True,
            capture_output=True,
            timeout=8.0,
        )
    except subprocess.TimeoutExpired:
        return ["ros2 daemon stop: timed out"]
    except Exception as exc:  # pragma: no cover - operator diagnostics only.
        return [f"ros2 daemon stop: failed: {exc}"]

    output = " ".join(part.strip() for part in (completed.stdout, completed.stderr) if part.strip())
    if completed.returncode == 0:
        return [f"ros2 daemon stop: ok{': ' + output if output else ''}"]
    return [f"ros2 daemon stop: rc={completed.returncode}{': ' + output if output else ''}"]


def find_existing_doosan_launch() -> tuple[int | None, str]:
    """Return one existing Doosan launch PID/cmd if a bringup is already starting/running."""
    if psutil is None:
        return None, ""
    current_pid = os.getpid()
    launch_markers = (
        "run_doosan_real_no_motion_m0609.sh",
        "dsr_bringup2_moveit.launch.py",
    )
    for proc in psutil.process_iter(["pid", "cmdline", "name"]):
        if proc.pid == current_pid:
            continue
        cmd = command_line(proc)
        if not cmd:
            continue
        if any(protected in cmd for protected in PANEL_PROTECTED_PATTERNS):
            continue
        if any(marker in cmd for marker in launch_markers):
            return int(proc.pid), cmd
    return None, ""


def infer_rt_host(robot_host: str) -> str:
    """Return the local source IP used to reach the robot, if discoverable."""
    robot_host = robot_host.strip()
    if not robot_host or robot_host.startswith("<"):
        return ""
    try:
        completed = subprocess.run(
            ["ip", "route", "get", robot_host],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return ""
    parts = completed.stdout.split()
    if "src" not in parts:
        return ""
    index = parts.index("src") + 1
    return parts[index] if index < len(parts) else ""


def robot_graph_ready(service_prefix: str) -> bool:
    """Check that an existing Doosan graph is responsive before starting another."""
    clean = service_prefix.strip("/") or "dsr01"
    state_service = f"/{clean}/system/get_robot_state"
    check_motion_service = f"/{clean}/motion/check_motion"
    state_rc, state_output = ros2_call_empty_service(
        state_service,
        "dsr_msgs2/srv/GetRobotState",
        timeout_sec=6.0,
    )
    if state_rc != 0:
        return False
    motion_rc, motion_output = ros2_call_empty_service(
        check_motion_service,
        "dsr_msgs2/srv/CheckMotion",
        timeout_sec=6.0,
    )
    return (
        motion_rc == 0
        and "success=True" in state_output
        and "success=True" in motion_output
    )


def ros2_call(command: str, timeout_sec: float = 8.0) -> tuple[int, str]:
    cmd = f"{ROS_SETUP} && timeout {max(timeout_sec, 0.1):.1f}s {command}"
    completed = subprocess.run(
        ["bash", "-lc", cmd],
        cwd=str(ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=max(timeout_sec + 3.0, 1.0),
        check=False,
    )
    return completed.returncode, completed.stdout


def ros2_call_empty_service(
    service_name: str,
    service_type: str,
    *,
    timeout_sec: float = 8.0,
) -> tuple[int, str]:
    """Call an empty ROS service without ros2cli .srv text parsing.

    Some Doosan dsr_msgs2 installations ship generated .srv files with C++-style
    comment headers. The Python service classes import and call correctly, but
    `ros2 service call` can reject the service type while parsing the .srv text.
    """
    script = ROOT / "tools" / "run" / "ros_call_empty_service.py"
    command = (
        f"python3 {shlex.quote(str(script))} "
        f"{shlex.quote(service_name)} {shlex.quote(service_type)} "
        f"--timeout {max(timeout_sec, 0.1):.1f}"
    )
    return ros2_call(command, timeout_sec=timeout_sec + 1.0)


def ros_service_names(timeout_sec: float = 6.0) -> tuple[set[str], str]:
    rc, output = ros2_call("ros2 service list --no-daemon", timeout_sec=timeout_sec)
    if rc != 0:
        return set(), output
    return {line.strip() for line in output.splitlines() if line.strip().startswith("/")}, output


def action_server_count(action_name: str, timeout_sec: float = 4.0) -> tuple[int, str]:
    rc, output = ros2_call(f"ros2 action info {shlex.quote(action_name)}", timeout_sec=timeout_sec)
    if rc != 0:
        return 0, output
    match = re.search(r"Action servers:\s*(\d+)", output)
    return (int(match.group(1)) if match else 0), output


def wait_for_action_server(action_name: str, *, timeout_sec: float = 15.0) -> tuple[bool, str]:
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    last_output = ""
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        count, output = action_server_count(action_name)
        last_output = output
        if count > 0:
            return True, f"action server became ready after {attempt} check(s): {action_name}\n{output}"
        time.sleep(0.5)
    return False, f"action server did not become ready within {timeout_sec:.1f}s: {action_name}\n{last_output}"


def motion_services_ready(service_prefix: str) -> tuple[bool, str]:
    clean = service_prefix.strip("/") or "dsr01"
    required = {
        f"/{clean}/motion/move_line",
        f"/{clean}/motion/move_joint",
        f"/{clean}/motion/ikin",
        f"/{clean}/motion/check_motion",
    }
    services, output = ros_service_names(timeout_sec=6.0)
    missing = sorted(required - services)
    if missing:
        return False, "missing motion services: " + ", ".join(missing) + "\n--- services ---\n" + output
    return True, "motion services are present"


def wait_for_motion_services_ready(
    service_prefix: str,
    *,
    timeout_sec: float = 35.0,
    proc: subprocess.Popen[str] | None = None,
) -> tuple[bool, str]:
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    last_output = ""
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        ready, output = motion_services_ready(service_prefix)
        last_output = output
        if ready:
            return True, f"motion services became ready after {attempt} check(s)\n{output}"
        if proc is not None and proc.poll() is not None:
            return False, f"connect process exited while waiting for motion services\n{output}"
        time.sleep(1.0)
    return False, f"motion services did not become ready within {timeout_sec:.1f}s\n{last_output}"


def parse_robot_state_id(text: str) -> int | None:
    match = re.search(r"(?:robot_state|state)[:=]\s*(\d+)", text)
    return int(match.group(1)) if match else None


def robot_state_name(state_id: int | None) -> str:
    if state_id is None:
        return "UNKNOWN"
    return ROBOT_STATE_NAMES.get(state_id, f"UNKNOWN_STATE_{state_id}")


def status_check_failure(output: str) -> str | None:
    state_id = parse_robot_state_id(output)
    if state_id is None:
        return "[FAIL] could not parse robot_state; refusing to mark status_check as passed."
    if state_id != 1:
        return (
            f"[FAIL] robot_state={state_id}({robot_state_name(state_id)}) is not "
            "STATE_STANDBY(1). Robot is connected, but real motion is not ready."
        )
    action_match = re.search(r"Action servers:\s*(\d+)", output)
    if action_match is not None and int(action_match.group(1)) < 1:
        return (
            "[FAIL] MoveIt trajectory action server is not available: "
            "/dsr01/dsr_moveit_controller/follow_joint_trajectory"
        )
    return None


def run_output_failure(step: Step, output: str) -> str | None:
    if step.key in {"shake_closed_cup", "side_grip"}:
        failure_markers = [
            "]: FAILED",
            " returned success=false",
            " Ikin returned success=false",
            " refusing MoveLine ",
            " refusing MoveJoint",
            "MoveIt state validity is invalid",
            "Hardware gates are incomplete",
            "enable_hardware was requested but hardware gates are incomplete",
        ]
        if step.key == "side_grip":
            failure_markers.extend(
                [
                    "Action client not connected to action server",
                    "Failed to send trajectory",
                    "Completed trajectory execution with status ABORTED",
                    "MoveIt execution did not reach the requested pose",
                    "High camera home move failed",
                    "No valid depth around cup bbox",
                    "Exiting after one auto-pick attempt (success=False)",
                ]
            )
        if any(marker in output for marker in failure_markers):
            return f"[FAIL] {step.key} reported an internal failure even though ros2 launch exited cleanly."
    return None


def text_output(output: str | bytes | None) -> str:
    if output is None:
        return ""
    if isinstance(output, bytes):
        return output.decode("utf-8", errors="replace")
    return output


def required_services_for_step(step: Step, service_prefix: str) -> list[str]:
    clean = service_prefix.strip("/") or "dsr01"
    if step.key == "gripper_open":
        return ["/jarvis/rg2/set_width"]
    if step.key == "gripper_soft_grasp":
        return ["/jarvis/rg2/set_width"]
    if step.key.startswith("teach_front_hold_"):
        return [
            f"/{clean}/system/get_robot_state",
        ]
    if step.key == "side_grip":
        return [
            f"/{clean}/motion/move_joint",
            f"/{clean}/motion/check_motion",
            f"/{clean}/system/get_robot_state",
        ]
    if step.key in {"home_robot", "lift_robot"}:
        return [
            f"/{clean}/motion/move_joint",
            f"/{clean}/motion/check_motion",
            f"/{clean}/system/get_robot_state",
        ]
    if step.key.startswith("move_to_dispenser_"):
        return [
            f"/{clean}/motion/move_line",
            f"/{clean}/motion/ikin",
            f"/{clean}/motion/check_motion",
            f"/{clean}/system/get_robot_state",
            f"/{clean}/tcp/get_current_tcp",
            f"/{clean}/aux_control/get_current_posx",
            "/jarvis/rg2/set_width",
        ]
    if step.key.startswith("press_dispenser_"):
        return [
            "/jarvis/rg2/set_width",
            f"/{clean}/motion/move_joint",
            f"/{clean}/motion/move_line",
            f"/{clean}/motion/move_wait",
            f"/{clean}/motion/check_motion",
            f"/{clean}/system/get_robot_state",
            f"/{clean}/aux_control/get_current_posx",
            f"/{clean}/tcp/set_current_tcp",
            f"/{clean}/tcp/get_current_tcp",
        ]
    if step.key.startswith("pick_from_dispenser_"):
        return [
            "/jarvis/rg2/set_width",
            f"/{clean}/motion/move_joint",
            f"/{clean}/motion/move_line",
            f"/{clean}/motion/move_wait",
            f"/{clean}/motion/ikin",
            f"/{clean}/motion/check_motion",
            f"/{clean}/system/get_robot_state",
            f"/{clean}/tcp/get_current_tcp",
            f"/{clean}/aux_control/get_current_posj",
            f"/{clean}/aux_control/get_current_posx",
        ]
    if step.key == "run_dispenser_recipe_sequence":
        return [
            "/jarvis/rg2/set_width",
            f"/{clean}/motion/move_joint",
            f"/{clean}/motion/move_line",
            f"/{clean}/motion/move_wait",
            f"/{clean}/motion/ikin",
            f"/{clean}/motion/check_motion",
            f"/{clean}/system/get_robot_state",
            f"/{clean}/tcp/get_current_tcp",
            f"/{clean}/tcp/set_current_tcp",
            f"/{clean}/aux_control/get_current_posx",
        ]
    if step.key == "place_cup_holder":
        return [
            "/jarvis/rg2/set_width",
            f"/{clean}/motion/move_line",
            f"/{clean}/motion/ikin",
            f"/{clean}/motion/check_motion",
            f"/{clean}/system/get_robot_state",
            f"/{clean}/aux_control/get_current_posx",
        ]
    if step.key == "shake_closed_cup":
        return [
            "/jarvis/rg2/set_width",
            f"/{clean}/motion/move_line",
            f"/{clean}/motion/move_joint",
            f"/{clean}/motion/move_wait",
            f"/{clean}/motion/ikin",
            f"/{clean}/motion/check_motion",
            f"/{clean}/aux_control/get_current_posx",
            f"/{clean}/aux_control/get_current_posj",
            f"/{clean}/system/get_robot_state",
            "/check_state_validity",
        ]
    return []


def required_service_wait_timeout(step: Step) -> float:
    """Wait long enough for the Doosan service graph to settle before blocking.

    Doosan bringup can publish MoveIt/RViz/gripper services before dsr_controller2
    finishes exporting motion/tcp/aux/system services.  A single immediate
    service-list check makes panel steps look randomly blocked even though the
    same services appear a few seconds later.
    """

    if (
        step.key.startswith("move_to_dispenser_")
        or step.key.startswith("press_dispenser_")
        or step.key.startswith("pick_from_dispenser_")
        or step.key == "run_dispenser_recipe_sequence"
        or step.key == "place_cup_holder"
    ):
        return 35.0
    if step.key in {"home_robot", "lift_robot", "side_grip", "shake_closed_cup"}:
        return 30.0
    if step.key in {"gripper_open", "gripper_soft_grasp"}:
        return 12.0
    return 8.0


def missing_required_services(step: Step, service_prefix: str) -> tuple[list[str], str]:
    required = required_services_for_step(step, service_prefix)
    if not required:
        return [], ""
    ready, wait_output = wait_for_required_services(
        required,
        timeout_sec=required_service_wait_timeout(step),
    )
    if ready:
        # Trust the successful wait sample.  ROS 2/DDS service discovery can
        # briefly return a partial graph on the very next `ros2 service list`,
        # especially after earlier panel steps have just exercised Doosan and
        # MoveIt services.  Re-listing here caused false blocks that reported
        # both "required services became ready" and "missing" in the same
        # response.  Real motion is still gated below by direct service calls
        # (`get_robot_state`, `check_motion`) before any movement command runs.
        return [], wait_output
    services, output = ros_service_names(timeout_sec=2.0)
    missing = [service for service in required if service not in services]
    return missing, wait_output + "\n--- final service list ---\n" + output


def gripper_services_for_step(step: Step, service_prefix: str) -> list[str]:
    return [
        service
        for service in required_services_for_step(step, service_prefix)
        if service.startswith("/jarvis/rg2/")
    ]


def wait_for_required_services(
    required: list[str],
    *,
    timeout_sec: float = 20.0,
    proc: subprocess.Popen[str] | None = None,
) -> tuple[bool, str]:
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    last_output = ""
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        services, output = ros_service_names(timeout_sec=2.0)
        missing = [service for service in required if service not in services]
        last_output = output
        if not missing:
            return True, f"required services became ready after {attempt} check(s): {', '.join(required)}"
        if proc is not None and proc.poll() is not None:
            return (
                False,
                "service provider process exited while waiting\n"
                f"missing: {', '.join(missing)}\n--- services ---\n{output}",
            )
        time.sleep(0.5)
    missing_text = ", ".join(required)
    return (
        False,
        f"required services did not become ready within {timeout_sec:.1f}s: {missing_text}\n"
        f"--- services ---\n{last_output}",
    )



def wait_for_collision_object_sample(
    *,
    env: dict[str, str],
    timeout_sec: float = 10.0,
    proc: subprocess.Popen[str] | None = None,
) -> tuple[bool, str]:
    """Wait until the MoveIt collision scene publisher emits at least one object."""
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    last_output = ""
    while time.monotonic() < deadline:
        if proc is not None and proc.poll() is not None:
            return False, "collision scene process exited while waiting\n" + tail_file(process_logs.get("start_collision_scene"))
        result = subprocess.run(
            ["bash", "-lc", "timeout 2s ros2 topic echo /collision_object --once"],
            cwd=str(ROOT),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=3.0,
            check=False,
        )
        last_output = result.stdout[-2000:]
        if result.returncode == 0 and "id:" in result.stdout:
            return True, "collision object sample observed on /collision_object\n" + last_output
        time.sleep(0.5)
    return False, (
        f"no collision object sample observed on /collision_object within {timeout_sec:.1f}s\n"
        f"--- last output ---\n{last_output}"
    )


def wait_for_camera_topic_samples(
    *,
    env: dict[str, str],
    timeout_sec: float = 15.0,
    proc: subprocess.Popen[str] | None = None,
) -> tuple[bool, str]:
    topics = [
        ("/camera/camera/color/image_raw", "color image"),
        ("/camera/camera/aligned_depth_to_color/image_raw", "aligned depth"),
        ("/camera/camera/color/camera_info", "camera info"),
    ]
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    seen: set[str] = set()
    last_output = ""
    while time.monotonic() < deadline:
        if proc is not None and proc.poll() is not None:
            return False, "camera process exited while waiting\n" + tail_file(process_logs.get("start_camera"))
        for topic, label in topics:
            if topic in seen:
                continue
            result = subprocess.run(
                ["bash", "-lc", f"timeout 2s ros2 topic echo {shlex.quote(topic)} --once"],
                cwd=str(ROOT),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=3.0,
                check=False,
            )
            last_output = result.stdout[-1200:]
            if result.returncode == 0 and ("header:" in result.stdout or "height:" in result.stdout):
                seen.add(topic)
                last_output = f"{label} sample observed on {topic}\n" + last_output
        if len(seen) == len(topics):
            return True, "camera samples observed:\n" + "\n".join(f"- {topic}" for topic, _ in topics)
        time.sleep(0.4)
    missing = [topic for topic, _ in topics if topic not in seen]
    return False, (
        f"camera topics did not all publish within {timeout_sec:.1f}s\n"
        f"seen: {', '.join(sorted(seen)) or '<none>'}\n"
        f"missing: {', '.join(missing)}\n"
        f"--- last output ---\n{last_output}"
    )


def wait_for_cup_detection_sample(
    *,
    env: dict[str, str],
    timeout_sec: float = 10.0,
    proc: subprocess.Popen[str] | None = None,
) -> tuple[bool, str]:
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    last_output = ""
    while time.monotonic() < deadline:
        if proc is not None and proc.poll() is not None:
            return False, "YOLO process exited while waiting\n" + tail_file(process_logs.get("detect_cup_lid"))
        result = subprocess.run(
            ["bash", "-lc", "timeout 2s ros2 topic echo /azas/cup_detection --once"],
            cwd=str(ROOT),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=3.0,
            check=False,
        )
        last_output = result.stdout[-2000:]
        if result.returncode == 0 and ("status:" in result.stdout or "pose:" in result.stdout):
            return True, "cup detection sample observed on /azas/cup_detection\n" + last_output
        time.sleep(0.5)
    return False, (
        f"no cup detection sample observed on /azas/cup_detection within {timeout_sec:.1f}s\n"
        "카메라 화면에 직립 텀블러/컵이 보이면 다시 확인하세요.\n"
        f"--- last output ---\n{last_output}"
    )


def camera_snapshot_jpeg() -> tuple[bool, bytes, str]:
    """Capture one RealSense color frame and return it as JPEG bytes."""
    script = r"""
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

frame = None

def to_bgr(msg):
    enc = (msg.encoding or "").lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        image = data.reshape((msg.height, msg.width, 3))
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if enc == "rgb8" else image
    if enc in ("rgba8", "bgra8"):
        image = data.reshape((msg.height, msg.width, 4))
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR) if enc == "rgba8" else cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if enc == "mono8":
        image = data.reshape((msg.height, msg.width))
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    raise RuntimeError(f"unsupported image encoding: {msg.encoding}")

def callback(msg):
    global frame
    if frame is None:
        frame = to_bgr(msg)

rclpy.init()
node = rclpy.create_node("azas_panel_camera_snapshot")
node.create_subscription(Image, "/camera/camera/color/image_raw", callback, qos_profile_sensor_data)
deadline = time.time() + 3.0
try:
    while rclpy.ok() and frame is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if frame is None:
        raise RuntimeError("no /camera/camera/color/image_raw frame within 3s")
    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    sys.stdout.buffer.write(encoded.tobytes())
finally:
    node.destroy_node()
    rclpy.shutdown()
"""
    env = shell_env({})
    try:
        completed = subprocess.run(
            ["bash", "-lc", f"{ROS_SETUP} && python3 -c {shlex.quote(script)}"],
            cwd=str(ROOT),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=6.0,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return False, b"", "camera snapshot timed out"
    except Exception as exc:  # pragma: no cover - operator diagnostics only.
        return False, b"", f"camera snapshot failed: {exc}"
    if completed.returncode != 0 or not completed.stdout:
        return False, b"", completed.stderr.decode("utf-8", errors="replace")[-2000:]
    return True, completed.stdout, ""


def side_grip_preflight(env: dict[str, str], service_prefix: str) -> tuple[bool, str]:
    """Validate PR #20 manual side-grip prerequisites before starting MoveItPy.

    The PR #20 path opens its YOLO preview only after model/calibration loading
    and MoveItPy initialization. If prerequisites are missing, the operator can
    otherwise see "camera window does not open" while the real failure happened
    earlier in startup.
    """
    checks: list[str] = []
    ok = True

    model_path = PR20_YOLO_MODEL_PATH
    if model_path.exists():
        checks.append(f"[OK] YOLO model: {model_path}")
    else:
        ok = False
        checks.append(f"[FAIL] YOLO model missing: {model_path}")

    calibration_candidates = [
        ROOT / "install" / "dsr_practice" / "share" / "dsr_practice" / "config" / "T_gripper2camera.npy",
        ROOT / "src" / "dsr_practice" / "config" / "T_gripper2camera.npy",
        ROOT / "install" / "azas_perception" / "share" / "azas_perception" / "config" / "T_gripper2camera.npy",
    ]
    calibration_path = next((path for path in calibration_candidates if path.exists()), None)
    if calibration_path is not None:
        checks.append(f"[OK] hand-eye calibration: {calibration_path}")
    else:
        ok = False
        checks.append(
            "[FAIL] hand-eye calibration missing: "
            + ", ".join(str(path) for path in calibration_candidates)
        )

    camera_ready, camera_output = wait_for_camera_topic_samples(env=env, timeout_sec=8.0)
    checks.append("--- camera topics ---\n" + camera_output)
    if not camera_ready:
        ok = False

    clean = service_prefix.strip("/") or "dsr01"
    action_name = f"/{clean}/dsr_moveit_controller/follow_joint_trajectory"
    action_ready, action_output = wait_for_action_server(action_name, timeout_sec=5.0)
    checks.append("--- MoveIt action ---\n" + action_output)
    if not action_ready:
        ok = False

    return ok, "\n".join(checks)


def ensure_gripper_services(step: Step, payload: dict[str, Any], service_prefix: str) -> tuple[bool, str]:
    required = gripper_services_for_step(step, service_prefix)
    if not required:
        return True, ""

    services, service_output = ros_service_names(timeout_sec=2.0)
    missing = [service for service in required if service not in services]
    if not missing:
        return True, "gripper services are present: " + ", ".join(required)

    connect_step = next(item for item in STEPS if item.key == "connect_gripper")
    old = processes.get("connect_gripper")
    events = [
        "그리퍼 서비스가 없어 자동으로 그리퍼 연결을 먼저 시도합니다.",
        f"missing: {', '.join(missing)}",
    ]

    if old is None or old.poll() is not None:
        cmd = command_for(connect_step, payload)
        env = shell_env(payload)
        log_path = background_log_path("connect_gripper")
        log_handle = log_path.open("w", encoding="utf-8", buffering=1)
        log_handle.write(f"[Azas panel] auto command: {cmd}\n\n")
        proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            cwd=str(ROOT),
            env=env,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        log_handle.close()
        processes["connect_gripper"] = proc
        process_logs["connect_gripper"] = log_path
        events.append(f"auto-started connect_gripper pid={proc.pid} log={log_path}")
    else:
        proc = old
        events.append(f"connect_gripper already running pid={proc.pid}")

    ready, wait_output = wait_for_required_services(required, timeout_sec=20.0, proc=proc)
    events.append(wait_output)
    if ready:
        return True, "\n".join(events)

    events.append("--- connect_gripper log tail ---")
    events.append(tail_file(process_logs.get("connect_gripper")))
    events.append("--- services before auto-connect ---")
    events.append(service_output)
    return False, "\n".join(events)


def requires_doosan_motion(step: Step) -> bool:
    return (
        step.key in {"home_robot", "lift_robot", "side_grip", "shake_closed_cup"}
        or step.key.startswith("move_to_dispenser_")
        or step.key.startswith("press_dispenser_")
        or step.key.startswith("pick_from_dispenser_")
        or step.key == "run_dispenser_recipe_sequence"
        or step.key == "place_cup_holder"
    )


def doosan_robot_ready(service_prefix: str) -> tuple[bool, str]:
    clean = service_prefix.strip("/") or "dsr01"
    state_output = ""
    state_rc = 1
    state_attempts: list[str] = []
    for attempt in range(1, 4):
        state_rc, state_output = ros2_call(
            f"python3 {shlex.quote(str(ROOT / 'tools' / 'run' / 'ros_call_empty_service.py'))} "
            f"/{clean}/system/get_robot_state dsr_msgs2/srv/GetRobotState --timeout 8.0",
            timeout_sec=8.0,
        )
        state_attempts.append(f"attempt {attempt}: rc={state_rc}\n{state_output}".rstrip())
        if state_rc == 0:
            break
        # DDS/service discovery can briefly drop a direct service call right
        # after long Doosan motions.  Re-sample the service graph before
        # fail-closing so a transient hidden by `ros2 service list` does not
        # abort the whole recipe sequence.
        wait_for_required_services(
            [f"/{clean}/system/get_robot_state", f"/{clean}/motion/check_motion"],
            timeout_sec=6.0,
        )
        time.sleep(0.5)
    if state_rc != 0:
        return False, "--- get_robot_state failed after retries ---\n" + "\n--- retry ---\n".join(state_attempts)

    state_id = parse_robot_state_id(state_output)
    if state_id is None:
        return (
            False,
            "--- get_robot_state ---\n"
            + state_output
            + "\n[Azas] could not parse robot_state; refusing motion.",
        )
    if state_id != 1:
        return (
            False,
            "--- get_robot_state ---\n"
            + state_output
            + f"\n[Azas] robot_state={state_id}({robot_state_name(state_id)}) is not "
            "STATE_STANDBY(1); refusing motion.",
        )

    motion_output = ""
    motion_rc = 1
    motion_attempts: list[str] = []
    for attempt in range(1, 4):
        motion_rc, motion_output = ros2_call(
            f"python3 {shlex.quote(str(ROOT / 'tools' / 'run' / 'ros_call_empty_service.py'))} "
            f"/{clean}/motion/check_motion dsr_msgs2/srv/CheckMotion --timeout 8.0",
            timeout_sec=8.0,
        )
        motion_attempts.append(f"attempt {attempt}: rc={motion_rc}\n{motion_output}".rstrip())
        if motion_rc == 0:
            break
        wait_for_required_services(
            [f"/{clean}/system/get_robot_state", f"/{clean}/motion/check_motion"],
            timeout_sec=6.0,
        )
        time.sleep(0.5)
    if motion_rc != 0:
        return False, "--- check_motion failed after retries ---\n" + "\n--- retry ---\n".join(motion_attempts)

    return True, "--- get_robot_state ---\n" + state_output + "\n--- check_motion ---\n" + motion_output


def parse_numeric_array(text: str) -> list[float]:
    match = re.search(r"(?:data|pos)[:=]\s*(?:array\()?\[([^\]]+)\]", text, re.S)
    if not match:
        return []
    values = re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", match.group(1))
    return [float(value) for value in values]


def current_posx_mm(service_prefix: str) -> tuple[list[float], str]:
    clean = service_prefix.strip("/") or "dsr01"
    rc, output = ros2_call(
        f"ros2 service call /{clean}/aux_control/get_current_posx "
        "dsr_msgs2/srv/GetCurrentPosx '{ref: 0}'"
    )
    if rc != 0:
        return [], output
    return parse_numeric_array(output), output


def last_alarm(service_prefix: str) -> str:
    clean = service_prefix.strip("/") or "dsr01"
    _rc, output = ros2_call(
        f"ros2 service call /{clean}/system/get_last_alarm "
        "dsr_msgs2/srv/GetLastAlarm '{}'"
    )
    return output


def motion_status(service_prefix: str) -> tuple[str, str]:
    clean = service_prefix.strip("/") or "dsr01"
    rc, output = ros2_call_empty_service(
        f"/{clean}/motion/check_motion",
        "dsr_msgs2/srv/CheckMotion",
    )
    if rc != 0:
        return "unknown", output
    match = re.search(r"status=(\d+)|status:\s*(\d+)", output)
    status = next((group for group in match.groups() if group is not None), "unknown") if match else "unknown"
    return status, output


def distance_mm(a: list[float], b: list[float]) -> float:
    return sum((a[index] - b[index]) ** 2 for index in range(3)) ** 0.5


def wait_for_xyz_target(
    service_prefix: str,
    target_xyz_mm: list[float],
    *,
    tolerance_mm: float = 15.0,
    timeout_sec: float = 70.0,
) -> tuple[bool, str]:
    deadline = time.monotonic() + timeout_sec
    lines: list[str] = []
    while time.monotonic() < deadline:
        pose, pose_output = current_posx_mm(service_prefix)
        if pose:
            dist = distance_mm(pose, target_xyz_mm)
            lines.append(
                f"[Azas] verify pose xyz=[{pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}] "
                f"target=[{target_xyz_mm[0]:.1f}, {target_xyz_mm[1]:.1f}, {target_xyz_mm[2]:.1f}] "
                f"distance={dist:.1f}mm"
            )
            if dist <= tolerance_mm:
                status, status_output = motion_status(service_prefix)
                return True, "\n".join(lines + [status_output])
        else:
            lines.append("[Azas] verify pose read failed:\n" + pose_output)

        time.sleep(1.0)

    status, status_output = motion_status(service_prefix)
    alarm_output = last_alarm(service_prefix)
    return False, "\n".join(lines[-8:] + ["--- motion status ---", status_output, "--- last alarm ---", alarm_output])


def target_xyz_for_step(step_key: str) -> list[float] | None:
    return None




def requires_collision_scene_step(key: str) -> bool:
    return (
        key == "run_dispenser_recipe_sequence"
        or key == "shake_closed_cup"
        or key.startswith("move_to_dispenser_")
        or key.startswith("pick_from_dispenser_")
    )


def with_collision_scene_prereq(selected: list[str]) -> list[str]:
    ordered = list(selected)
    if "side_grip" in ordered:
        # PR #20 node also moves to camera-home internally, but the supervised
        # panel must make the operator-visible sequence explicit and safe:
        # lift the robot first, then start RealSense, then run manual side-grip.
        side_index = ordered.index("side_grip")
        prerequisites = ["lift_robot", "start_camera"]
        for prereq in reversed(prerequisites):
            if prereq not in ordered:
                ordered.insert(side_index, prereq)
    if any(requires_collision_scene_step(key) for key in ordered):
        ordered = ["start_collision_scene"] + [key for key in ordered if key != "start_collision_scene"]
    return ordered

def run_timeout_for_step(step: Step) -> float:
    if step.key == "run_dispenser_recipe_sequence":
        return 900.0
    if step.key == "side_grip":
        return 900.0
    if step.key == "place_cup_holder":
        return 240.0
    return 180.0


def shell_env(payload: dict[str, Any]) -> dict[str, str]:
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(
        payload.get("ros_domain_id")
        or env.get("AZAS_PANEL_ROS_DOMAIN_ID")
        or env.get("ROS_DOMAIN_ID")
        or DEFAULT_ROS_DOMAIN_ID
    )
    env["ROS_LOCALHOST_ONLY"] = str(env.get("ROS_LOCALHOST_ONLY") or "0")
    env["ROBOT_HOST"] = str(payload.get("robot_host") or env.get("ROBOT_HOST") or DEFAULT_ROBOT_HOST)
    env["ROBOT_NAME"] = str(payload.get("robot_name") or env.get("ROBOT_NAME") or "dsr01")
    env["SERVICE_PREFIX"] = str(payload.get("service_prefix") or env.get("SERVICE_PREFIX") or "dsr01")
    env["RG2_IP"] = str(payload.get("rg2_ip") or env.get("RG2_IP") or "192.168.1.1")
    env["SELECTED_DISPENSER_ID"] = str(
        payload.get("selected_dispenser_id") or env.get("SELECTED_DISPENSER_ID") or "2"
    )
    env["RECIPE_DISPENSER_IDS"] = str(
        payload.get("recipe_dispenser_ids") or env.get("RECIPE_DISPENSER_IDS") or "1,2,3,4"
    )
    env["CUP_HOLDER_PLACE_FINAL_Z_OFFSET_M"] = str(
        payload.get("cup_holder_place_final_z_offset_m")
        or env.get("CUP_HOLDER_PLACE_FINAL_Z_OFFSET_M")
        or "-0.020"
    )
    # Operational-only offset for the pre-shake cup-holder re-grasp.
    # This intentionally does not modify calibration.yaml and is separate from the
    # cup-holder placement offset so lowering the shake pickup does not push the
    # cup deeper during place_cup_holder.
    env["CUP_HOLDER_PICK_Z_OFFSET_M"] = str(
        payload.get("cup_holder_pick_z_offset_m")
        or env.get("CUP_HOLDER_PICK_Z_OFFSET_M")
        or "-0.020"
    )
    env["RT_HOST"] = str(
        payload.get("rt_host")
        or env.get("RT_HOST")
        or infer_rt_host(env["ROBOT_HOST"])
        or "192.168.137.50"
    )
    env["DOOSAN_NO_MOTION_CONFIRM"] = "CONNECT_DOOSAN_NO_MOTION"
    return env


def command_for(step: Step, payload: dict[str, Any]) -> str:
    service_prefix = str(payload.get("service_prefix") or "dsr01")

    def tumbler_scene_once(action: str, *, object_id: str = "carried_tumbler", dispenser_id: str = "1") -> str:
        return (
            "timeout 5s python3 -m azas_motion.tumbler_collision_scene_node --ros-args "
            f"-p action:={shlex.quote(action)} "
            f"-p object_id:={shlex.quote(object_id)} "
            f"-p dispenser_id:={shlex.quote(dispenser_id)} "
            "-p publish_once:=true"
        )

    if step.key == "connect_robot":
        robot_host = str(payload.get("robot_host") or os.environ.get("ROBOT_HOST") or DEFAULT_ROBOT_HOST)
        robot_name = str(payload.get("robot_name") or os.environ.get("ROBOT_NAME") or "dsr01")
        rt_host = str(
            payload.get("rt_host")
            or os.environ.get("RT_HOST")
            or infer_rt_host(robot_host)
            or "<RT_HOST>"
        )
        return (
            f"cd {ROOT} && ROBOT_HOST={shlex.quote(robot_host)} "
            f"ROBOT_NAME={shlex.quote(robot_name)} RT_HOST={shlex.quote(rt_host)} "
            "DOOSAN_NO_MOTION_CONFIRM=CONNECT_DOOSAN_NO_MOTION "
            f"{step.command}"
        )
    if step.key == "status_check":
        clean = service_prefix.strip("/") or "dsr01"
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "echo '--- nodes ---' && ros2 node list && "
            "echo '--- required motion service types ---' && "
            f"ros2 service type /{clean}/motion/move_line && "
            f"ros2 service type /{clean}/motion/move_joint && "
            "echo '--- robot state ---' && "
            f"timeout 9s python3 {shlex.quote(str(ROOT / 'tools' / 'run' / 'ros_call_empty_service.py'))} "
            f"/{clean}/system/get_robot_state dsr_msgs2/srv/GetRobotState --timeout 8.0 && "
            "echo '--- check motion ---' && "
            f"timeout 9s python3 {shlex.quote(str(ROOT / 'tools' / 'run' / 'ros_call_empty_service.py'))} "
            f"/{clean}/motion/check_motion dsr_msgs2/srv/CheckMotion --timeout 8.0 && "
            "echo '--- trajectory action ---' && "
            f"ros2 action info /{clean}/dsr_moveit_controller/follow_joint_trajectory"
        )
    if step.key == "lift_robot":
        joints = {
            name: str(os.environ.get(f"CAMERA_TABLE_VIEW_{name.upper()}", value))
            for name, value in CAMERA_TABLE_VIEW_JOINTS.items()
        }
        return (
            f"cd {ROOT} && {ROS_SETUP} && python3 tools/run/direct_movej_joints.py "
            f"--service-prefix {service_prefix} "
            f"--j1 {shlex.quote(joints['j1'])} --j2 {shlex.quote(joints['j2'])} "
            f"--j3 {shlex.quote(joints['j3'])} --j4 {shlex.quote(joints['j4'])} "
            f"--j5 {shlex.quote(joints['j5'])} --j6 {shlex.quote(joints['j6'])} "
            f"--velocity {FAST_MOVE_VELOCITY} --acceleration {FAST_MOVE_ACCELERATION} "
            "--j5-min-deg -135 --j5-max-deg 135 --timeout-sec 60 "
            "--execute --confirm ENABLE_DIRECT_MOVEJ"
        )
    if step.key == "home_robot":
        return (
            f"cd {ROOT} && {ROS_SETUP} && python3 tools/run/direct_movej_joints.py "
            f"--service-prefix {service_prefix} --j1 0 --j2 0 --j3 90 "
            f"--j4 0 --j5 90 --j6 0 --velocity {FAST_MOVE_VELOCITY} --acceleration {FAST_MOVE_ACCELERATION} "
            "--execute --confirm ENABLE_DIRECT_MOVEJ"
        )
    if step.key == "connect_gripper":
        rg2_ip = str(payload.get("rg2_ip") or os.environ.get("RG2_IP") or "192.168.1.1")
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            f"ros2 launch azas_gripper rg2_trigger.launch.py ip:={shlex.quote(rg2_ip)} "
            "port:=502 connect:=true open_width:=1100 close_width:=0 force:=300 settle_seconds:=0.6"
        )
    if step.key == "start_collision_scene":
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "python3 -m azas_motion.measured_dispenser_collision_scene_node & "
            "python3 -m azas_motion.tumbler_collision_scene_node --ros-args "
            "-p action:=publish_detected "
            "-p object_id:=detected_tumbler "
            "-p use_lidded_height:=true"
        )
    if step.key == "start_camera":
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "ros2 launch realsense2_camera rs_launch.py "
            "camera_name:=camera "
            "enable_color:=true enable_depth:=true align_depth.enable:=true"
        )
    if step.key == "start_camera_view":
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "DISPLAY=${DISPLAY:-:0} "
            "XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority} "
            "ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw"
        )
    if step.key == "detect_cup_lid":
        return f"cd {ROOT} && {ROS_SETUP} && ros2 launch azas_bringup yolo_perception.launch.py"
    if step.key == "voice_input":
        return f"cd {ROOT} && {ROS_SETUP} && ros2 launch azas_voice azas_voice.launch.py"
    if step.key == "side_grip":
        return (
            f"cd {ROOT} && "
            "source /opt/ros/humble/setup.bash && "
            "source /home/ssu/ws_moveit/install/setup.bash && "
            "source /home/ssu/ros2_ws/install/setup.bash && "
            "if [ \"${AZAS_SIDE_GRIP_BUILD:-0}\" = \"1\" ]; then "
            "colcon build --symlink-install --packages-select dsr_practice; "
            "fi && "
            "source /home/ssu/Azas/install/setup.bash && "
            f"export PYTHONPATH={shlex.quote(str(ROOT / 'tools' / 'run' / 'python_compat'))}:${{PYTHONPATH:-}} && "
            "DISPLAY=${DISPLAY:-:0} "
            "XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority} "
            "ros2 launch dsr_practice yolo_cup_pick_node.launch.py "
            "model_path:=/home/ssu/Azas/best.pt "
            "conf:=0.35 "
            "imgsz:=640 "
            "device:=cpu "
            "target_class:=cup "
            "auto_pick:=false "
            "auto_pick_interval:=3.0 "
            "pick_depth_ratio:=0.55 "
            "depth_patch_radius:=7 "
            "min_depth_valid_ratio:=0.03 "
            "min_depth_m:=0.15 "
            "max_depth_m:=1.20 "
            "redetect_on_approach:=false "
            "redetect_settle_sec:=0.5 "
            "grasp_mode:=side "
            "side_grasp_axis:=y_axis "
            "side_grasp_direction:=1.0 "
            "side_auto_direction_by_cup_y:=true "
            "side_far_stage_enabled:=false "
            "side_staging_offset:=0.30 "
            "side_approach_offset:=0.18 "
            "side_short_stage_backoff_m:=0.08 "
            "side_stage_y_min:=-0.35 "
            "side_stage_y_max:=0.35 "
            "side_grasp_offset:=0.025 "
            "side_grasp_z_offset:=0.05 "
            "side_grasp_stop_backoff_m:=0.04 "
            "side_close_underreach_m:=0.05 "
            "side_low_retry_lift_m:=0.03 "
            "side_low_retry_attempts:=5 "
            "side_linear_approach_enabled:=false "
            "side_final_slide_enabled:=false "
            "side_fixed_grasp_z_enabled:=true "
            "side_fixed_grasp_z:=0.07 "
            "side_project_bbox_center_to_fixed_z:=true "
            "side_orientation_mode:=approach "
            "side_tool_roll_deg:=0.0 "
            "side_roll_deg:=0.0 "
            "side_pitch_deg:=90.0 "
            "side_yaw_deg:=0.0 "
            "table_collision_enabled:=true "
            "table_surface_z:=0.0 "
            "table_thickness:=0.04 "
            "table_size_x:=1.20 "
            "table_size_y:=1.00 "
            "table_center_x:=0.45 "
            "table_center_y:=0.0 "
            "dispenser_collision_enabled:=true "
            f"dispenser_collision_config_path:={shlex.quote(str(ROOT / 'src' / 'azas_bringup' / 'config' / 'measured_dispenser_collision.yaml'))} "
            "dispenser_collision_publish_period_sec:=1.0 "
            "dispenser_collision_publish_objects:=true "
            "dispenser_collision_publish_markers:=true "
            "center_check_enabled:=false "
            "center_check_settle_sec:=0.6 "
            "center_check_x:=0.45 "
            "center_check_y:=0.0 "
            "center_check_z:=0.64 "
            "side_prepose_enabled:=false "
            "side_prepose_split_z:=0.18 "
            "side_move_to_initial_center_before_close:=false "
            "pre_pick_joint1_clearance_deg:=0.0 "
            "verify_motion:=true "
            "motion_verify_tolerance:=0.03 "
            "joint_goal_tolerance_rad:=0.02 "
            "move_to_camera_home:=true "
            "move_joint_home_before_camera_home:=false "
            "camera_home_mode:=joint "
            "camera_home_joint_1_deg:=3.0 "
            "camera_home_joint_2_deg:=-12.7 "
            "camera_home_joint_3_deg:=44.0 "
            "camera_home_joint_4_deg:=-9.0 "
            "camera_home_joint_5_deg:=133.0 "
            "camera_home_joint_6_deg:=90.0 "
            "camera_home_x:=0.45 "
            "camera_home_y:=0.0 "
            "camera_home_z:=0.64 "
            "camera_home_search_max_z:=0.64 "
            "camera_home_search_min_z:=0.54 "
            "camera_home_search_step_z:=0.02 "
            "min_motion_z:=0.07 "
            "workspace_xy_clamp_enabled:=false "
            "return_home_after_task:=false "
            "return_to_camera_home_after_attempt:=true "
            "place_x:=0.45 "
            "place_y:=0.0 "
            "place_z:=0.30 "
            "moveit_controller_name:=/dsr01/dsr_moveit_controller "
            "start_joint_state_relay:=true"
        )
    if step.key == "gripper_open":
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "tools/run/rg2_full_open_verify.sh"
        )
    if step.key == "gripper_soft_grasp":
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "timeout 12s ros2 service call /jarvis/rg2/set_width "
            "azas_interfaces/srv/SetGripper "
            "\"{command: 'set_width', width_m: 0.075, force_n: 25.0}\""
        )
    if step.key == "shake_rviz_preview":
        return (
            f"cd {ROOT} && ROS_DOMAIN_ID={RVIZ_PREVIEW_ROS_DOMAIN_ID} "
            "TARGET_X=0.430 TARGET_Y=0.080 TARGET_Z=0.135 "
            "SHAKE_DELAY_SEC=4.0 SHAKE_CENTER_X=0.430 SHAKE_CENTER_Y=0.080 "
            "SHAKE_CENTER_Z=0.620 "
            "SHAKE_AMPLITUDE_X=0.100 SHAKE_AMPLITUDE_Y=0.040 "
            "SHAKE_AMPLITUDE_Z=0.055 SHAKE_CYCLES=4 "
            "SHAKE_TWIST_RX_DEG=6.0 SHAKE_TWIST_RY_DEG=3.0 SHAKE_TWIST_RZ_DEG=22.0 "
            "APPROACH_LINE_TIME=3.5 SHAKE_LINE_TIME=0.40 MIN_SHAKE_Z=0.550 "
            "tools/run/run_cup_target_then_shake_rviz.sh"
        )
    if step.key.startswith("teach_front_hold_"):
        dispenser_id = step.key.rsplit("_", 1)[-1]
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "python3 tools/run/teach_measured_dispenser_front_hold.py "
            f"--dispenser-id {shlex.quote(dispenser_id)} "
            "--write --confirm ENABLE_TEACH_MEASURED_DISPENSER_FRONT_HOLD"
        )
    if step.key.startswith("move_to_dispenser_"):
        dispenser_id = step.key.rsplit("_", 1)[-1]
        move_front_hold_base = (
            f"python3 tools/run/move_to_measured_dispenser_front_hold.py "
            f"--service-prefix {service_prefix} --dispenser-id {shlex.quote(dispenser_id)} "
            "--timeout-sec 180 --verify-target --verify-timeout-sec 70 "
            "--ikin-timeout-sec 20 --ikin-retries 2 "
            "--target-tolerance-mm 15 --compensate-current-tcp --verify-link6-target --no-moveit-planning-guard "
        )
        # Newly taught side-grip front-hold poses are the verified reachable poses.
        # Do not synthesize an above/retreat pose here: for the current side-grip
        # orientation, even +Z staging can be IK-infeasible.
        final_stage = (
            move_front_hold_base
            + f"--velocity {FAST_MOVE_VELOCITY} --acceleration {FAST_MOVE_ACCELERATION} "
            "--target-offset-x-m 0.0 --target-offset-y-m 0.0 --target-offset-z-m 0.0 "
            "--execute --confirm ENABLE_MEASURED_DISPENSER_FRONT_HOLD"
        )
        return (
            f"cd {ROOT} && {ROS_SETUP} && {final_stage}"
            " && tools/run/rg2_full_open_verify.sh"
            f" && {tumbler_scene_once('detach', object_id='carried_tumbler', dispenser_id=dispenser_id)}"
            f" && {tumbler_scene_once('add_dispenser', object_id=f'tumbler_at_dispenser_{dispenser_id}', dispenser_id=dispenser_id)}"
        )
    if step.key.startswith("press_dispenser_"):
        dispenser_id = step.key.rsplit("_", 1)[-1]
        target = DISPENSER_PRESS_TARGETS.get(dispenser_id, "red")
        tcp_name = str(
            payload.get("dispenser_tcp_name")
            or os.environ.get("DISPENSER_TCP_NAME")
            or DEFAULT_DISPENSER_TCP_NAME
        ).strip()
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "ros2 run azas_dispenser dispenser_press_node --ros-args "
            f"-p service_prefix:={shlex.quote(service_prefix)} "
            "-p use_taught_posx:=true "
            f"-p tcp_name:={shlex.quote(tcp_name)} "
            "-p require_tcp_for_taught_posx:=false "
            "-p allow_tcp_set_failure:=true "
            f"-p target_dispenser:={shlex.quote(target)} "
            "-p move_home_first:=true "
            "-p pre_home_retreat_before_home:=true "
            "-p pre_home_retreat_dx_mm:=-180.0 "
            "-p pre_home_retreat_dy_mm:=0.0 "
            "-p pre_home_retreat_min_z_mm:=520.0 -p pre_home_retreat_lift_first:=true "
            "-p pre_home_retreat_min_current_x_mm:=0.0 "
            "-p pre_home_retreat_velocity:=20.0 "
            "-p pre_home_retreat_acceleration:=25.0 "
            "-p joint1_clearance_before_home:=false "
            "-p joint1_clearance_return_home:=false "
            "-p joint1_clearance_offset_deg:=12.0 "
            "-p return_home:=true "
            "-p close_gripper_at_home:=true "
            "-p gripper_service:=/jarvis/rg2/set_width "
            "-p gripper_close_width:=0.0 "
            "-p gripper_close_force:=30.0 "
            "-p gripper_wait_timeout:=12.0 "
            "-p strict_pose_verification:=false "
            "-p service_wait_timeout_sec:=10.0 "
            "-p pose_position_tolerance_mm:=8.0 "
            "-p pose_orientation_tolerance_deg:=6.0 "
            "-p line_velocity:=20.0 "
            "-p line_acceleration:=30.0 "
            "-p travel_line_velocity:=45.0 "
            "-p travel_line_acceleration:=70.0 "
            "-p joint_velocity:=40.0 "
            "-p joint_acceleration:=50.0"
        )
    if step.key.startswith("pick_from_dispenser_"):
        dispenser_id = step.key.rsplit("_", 1)[-1]
        return (
            f"cd {ROOT} && {ROS_SETUP} && python3 tools/run/pick_from_measured_dispenser_front_hold.py "
            f"--service-prefix {service_prefix} --dispenser-id {shlex.quote(dispenser_id)} "
            "--approach-velocity 20.0 --approach-acceleration 25.0 "
            "--pregrasp-staging --pregrasp-offset-x-m 0.0 --pregrasp-offset-y-m 0.0 "
            "--pregrasp-offset-z-m 0.060 --pregrasp-staging-velocity 12.0 "
            "--pregrasp-staging-acceleration 20.0 --joint1-clearance-deg 0.0 "
            "--lift-m 0.100 --lift-velocity 18.0 --lift-acceleration 24.0 "
            "--timeout-sec 120 --wait-service-sec 8 --verify-timeout-sec 45 "
            "--target-tolerance-mm 15 --gripper-grasp-width-m 0.075 --gripper-force-n 25.0 "
            "--x-min 0.10 "
            "--execute --confirm ENABLE_PICK_FROM_MEASURED_DISPENSER_FRONT_HOLD"
            f" && {tumbler_scene_once('remove_world', object_id=f'tumbler_at_dispenser_{dispenser_id}', dispenser_id=dispenser_id)}"
            f" && {tumbler_scene_once('attach', object_id='carried_tumbler', dispenser_id=dispenser_id)}"
        )
    if step.key == "run_dispenser_recipe_sequence":
        recipe_ids = str(
            payload.get("recipe_dispenser_ids")
            or os.environ.get("RECIPE_DISPENSER_IDS")
            or "1,2,3,4"
        ).strip()
        tcp_name = str(
            payload.get("dispenser_tcp_name")
            or os.environ.get("DISPENSER_TCP_NAME")
            or DEFAULT_DISPENSER_TCP_NAME
        ).strip()
        return (
            f"cd {ROOT} && {ROS_SETUP} && python3 tools/run/run_measured_dispenser_recipe_sequence.py "
            f"--service-prefix {service_prefix} "
            f"--dispenser-ids {shlex.quote(recipe_ids)} "
            f"--dispenser-tcp-name {shlex.quote(tcp_name)} "
            "--execute --confirm ENABLE_MEASURED_DISPENSER_RECIPE_SEQUENCE"
        )
    if step.key == "place_cup_holder":
        place_final_z_offset_m = str(
            payload.get("cup_holder_place_final_z_offset_m")
            or os.environ.get("CUP_HOLDER_PLACE_FINAL_Z_OFFSET_M")
            or "-0.020"
        ).strip()
        return (
            f"cd {ROOT} && {ROS_SETUP} && python3 tools/run/place_side_grip_cup_in_holder.py "
            f"--service-prefix {service_prefix} "
            "--config /home/ssu/Azas/install/azas_bringup/share/azas_bringup/config/calibration.yaml "
            "--approach-velocity 15.0 --approach-acceleration 20.0 "
            f"--place-final-z-offset-m {shlex.quote(place_final_z_offset_m)} "
            "--place-velocity 6.0 --place-acceleration 10.0 "
            "--retreat-velocity 12.0 --retreat-acceleration 16.0 "
            "--timeout-sec 90.0 --target-tolerance-mm 12.0 --verify-timeout-sec 45.0 "
            "--z-max 0.28 "
            "--execute --confirm ENABLE_CUP_HOLDER_PLACE"
            f" && {tumbler_scene_once('detach', object_id='carried_tumbler')}"
            f" && {tumbler_scene_once('add_holder', object_id='tumbler_in_holder')}"
        )
    if step.key == "shake_closed_cup":
        pick_z_offset_m = str(
            payload.get("cup_holder_pick_z_offset_m")
            or os.environ.get("CUP_HOLDER_PICK_Z_OFFSET_M")
            or "-0.020"
        ).strip()
        holder_pick = (
            "python3 tools/run/pick_from_cup_holder_side_grip.py "
            f"--service-prefix {service_prefix} "
            "--config /home/ssu/Azas/install/azas_bringup/share/azas_bringup/config/calibration.yaml "
            "--approach-velocity 12.0 --approach-acceleration 16.0 "
            "--descend-velocity 6.0 --descend-acceleration 10.0 "
            "--lift-velocity 12.0 --lift-acceleration 16.0 "
            f"--place-final-z-offset-m {shlex.quote(pick_z_offset_m)} "
            "--timeout-sec 90.0 --target-tolerance-mm 12.0 --verify-timeout-sec 45.0 "
            "--ikin-timeout-sec 20.0 --ikin-retries 2 "
            "--gripper-grasp-width-m 0.068 --gripper-force-n 35.0 "
            "--post-grasp-settle-sec 0.8 "
            "--z-max 0.28 "
            "--execute --confirm ENABLE_CUP_HOLDER_PICK"
        )
        return (
            f"cd {ROOT} && "
            f"{ROS_SETUP} && "
            "echo '[Azas] SHAKE START: 컵홀더에 놓인 닫힌 컵을 측정 pose로 다시 side-grip 픽업한 뒤 흔듭니다.' && "
            "echo '[Azas] 순서: RG2 open -> 컵홀더 retreat 접근 -> holder final pose에서 soft grasp -> holder lift -> 관절 쉐이킹.' && "
            "echo '[Azas] 주의: 컵 좌표를 새로 만들지 않고 calibration.yaml cup_holder.side_grip_place 측정값만 사용합니다.' && "
            f"{holder_pick}"
            f" && {tumbler_scene_once('remove_world', object_id='tumbler_in_holder')}"
            f" && {tumbler_scene_once('attach', object_id='carried_tumbler')}"
            " && "
            f"SERVICE_PREFIX={service_prefix} GRASPED_CUP_TEST_MODE=true SKIP_CUP_HOLDER_PICK=true "
            "REQUIRE_ROBOT_STANDBY=true SHAKE_CONTROL_MODE=joint SHAKE_CYCLES=3 "
            "JOINT_SHAKE_BASE_J1_DEG=0.0 JOINT_SHAKE_BASE_J2_DEG=-35.0 "
            "JOINT_SHAKE_BASE_J3_DEG=50.0 JOINT_SHAKE_BASE_J4_DEG=0.0 "
            "JOINT_SHAKE_BASE_J5_DEG=70.0 JOINT_SHAKE_BASE_J6_DEG=0.0 "
            "JOINT_SHAKE_J3_AMPLITUDE_DEG=0.0 JOINT_SHAKE_J4_AMPLITUDE_DEG=18.0 "
            "JOINT_SHAKE_J5_AMPLITUDE_DEG=20.0 JOINT_SHAKE_J6_AMPLITUDE_DEG=24.0 "
            "JOINT_SHAKE_J1_MIN_DEG=-20.0 JOINT_SHAKE_J1_MAX_DEG=5.0 "
            "JOINT_SHAKE_J2_MIN_DEG=-80.0 JOINT_SHAKE_J2_MAX_DEG=5.0 "
            "JOINT_SHAKE_J3_MIN_DEG=0.0 JOINT_SHAKE_J3_MAX_DEG=135.0 "
            "JOINT_SHAKE_MAX_SINGLE_DELTA_DEG=75.0 "
            "ENFORCE_WRIST_JOINT_LIMITS=false WRIST_MIN_DEG=-135.0 WRIST_MAX_DEG=135.0 "
            "JOINT5_MIN_DEG=40.0 JOINT5_MAX_DEG=100.0 "
            "APPROACH_JOINT_VELOCITY=18.0 APPROACH_JOINT_ACCELERATION=22.0 "
            "APPROACH_JOINT_TIME=2.6 SHAKE_JOINT_VELOCITY=90.0 "
            "SHAKE_JOINT_ACCELERATION=120.0 SHAKE_JOINT_TIME=0.0 "
            "JOINT_SHAKE_PEAK_VELOCITY_LIMIT_DEG_S=130.0 "
            "VERIFY_JOINT_TARGETS=true JOINT_TARGET_TOLERANCE_DEG=8.0 "
            "JOINT_TARGET_WAIT_EXTRA_SEC=3.0 JOINT_TARGET_POLL_SEC=0.05 "
            "REQUIRE_STATE_VALIDITY_FOR_JOINT_SHAKE=true "
            "tools/run/run_rule_based_shake_real.sh"
        )
    return ""


def run_step(step: Step, payload: dict[str, Any]) -> dict[str, Any]:
    if not step.implemented:
        return {"key": step.key, "status": "blocked", "output": step.note}
    if step.real_motion and not payload.get("armed"):
        return {"key": step.key, "status": "blocked", "output": "실제 모션 허용 체크가 꺼져 있습니다."}
    if step.real_motion:
        service_prefix = str(payload.get("service_prefix") or "dsr01")
        gripper_ready, gripper_output = ensure_gripper_services(step, payload, service_prefix)
        if not gripper_ready:
            return {
                "key": step.key,
                "status": "blocked",
                "output": (
                    "필수 그리퍼 ROS 서비스가 없어 실제 동작을 막았습니다.\n"
                    "자동 그리퍼 연결도 준비 상태까지 가지 못했습니다.\n"
                    f"{gripper_output}"
                ),
            }
        missing, service_output = missing_required_services(step, service_prefix)
        if missing:
            return {
                "key": step.key,
                "status": "blocked",
                "output": (
                    "필수 ROS 서비스가 없어 실제 동작을 막았습니다.\n"
                    f"missing: {', '.join(missing)}\n"
                    f"--- gripper auto-check ---\n{gripper_output}\n"
                    "--- current services ---\n"
                    f"{service_output}"
                ),
            }
        if requires_doosan_motion(step):
            ready, ready_output = doosan_robot_ready(service_prefix)
            if not ready:
                return {
                    "key": step.key,
                    "status": "blocked",
                    "output": (
                        "로봇이 motion-ready 상태가 아니라 실제 동작을 막았습니다.\n"
                        "티치펜던트에서 비상정지/보호정지/안전구역/servo 상태를 확인한 뒤 다시 시도하세요.\n"
                        f"{ready_output}"
                    ),
                }
            if step.key == "side_grip":
                clean = service_prefix.strip("/") or "dsr01"
                action_name = f"/{clean}/dsr_moveit_controller/follow_joint_trajectory"
                action_ready, action_output = wait_for_action_server(action_name, timeout_sec=15.0)
                if not action_ready:
                    return {
                        "key": step.key,
                        "status": "blocked",
                        "output": (
                            "MoveIt trajectory action server가 없어 side_grip 실제 동작을 막았습니다.\n"
                            "로봇 연결을 다시 시작해서 dsr_moveit_controller action server가 뜨는지 확인하세요.\n"
                            f"{action_output}"
                        ),
                    }
    if step.key == "connect_robot" and not (
        payload.get("robot_host") or os.environ.get("ROBOT_HOST") or DEFAULT_ROBOT_HOST
    ):
        return {"key": step.key, "status": "blocked", "output": "ROBOT_HOST가 필요합니다."}

    env = shell_env(payload)
    preflight_output = ""
    if step.key == "side_grip":
        cleanup_events = cleanup_side_grip_stack(grace_sec=3.0)
        # Let DDS forget stale PR #20 picker/relay nodes before MoveItPy starts.
        time.sleep(1.0)
        service_prefix = str(payload.get("service_prefix") or "dsr01")
        preflight_ok, preflight_details = side_grip_preflight(env, service_prefix)
        preflight_output = "\n".join(
            cleanup_events
            + [
                "--- PR #20 side_grip preflight ---",
                preflight_details,
            ]
        ).strip()
        if not preflight_ok:
            return {
                "key": step.key,
                "status": "blocked",
                "output": (
                    "PR #20 side grip 실행 전 조건이 충족되지 않아 시작하지 않았습니다.\n"
                    f"{preflight_output}"
                ),
            }

    cmd = command_for(step, payload)
    if step.kind == "background":
        restart_output = ""
        if step.key == "connect_robot":
            ready, ready_output = motion_services_ready(env["SERVICE_PREFIX"])
            if ready:
                robot_ready, robot_ready_output = doosan_robot_ready(env["SERVICE_PREFIX"])
                if not robot_ready:
                    return {
                        "key": step.key,
                        "status": "blocked",
                        "output": (
                            "Doosan motion 서비스는 보이지만 로봇이 motion-ready 상태가 아닙니다. "
                            "재시작하지 않습니다.\n"
                            f"{ready_output}\n"
                            "티치펜던트/컨트롤러에서 빨간 상태(SAFE_OFF/보호정지/서보 상태)를 해제해 "
                            "STATE_STANDBY(1)로 만든 뒤 다시 확인하세요.\n"
                            f"{robot_ready_output}"
                        ),
                    }
                return {
                    "key": step.key,
                    "status": "running",
                    "output": (
                        "이미 Doosan motion 서비스가 보이고 로봇이 STATE_STANDBY(1)입니다. "
                        "재시작하지 않습니다.\n"
                        f"{ready_output}\n{robot_ready_output}"
                    ),
                }
            old = processes.get(step.key)
            if old and old.poll() is None:
                ready, waited_output = wait_for_motion_services_ready(
                    env["SERVICE_PREFIX"],
                    timeout_sec=20.0,
                    proc=old,
                )
                if ready:
                    robot_ready, robot_ready_output = doosan_robot_ready(env["SERVICE_PREFIX"])
                    if robot_ready:
                        return {
                            "key": step.key,
                            "status": "running",
                            "output": (
                                "기존 로봇 연결 프로세스가 계속 실행 중이고 motion 서비스가 준비됐습니다.\n"
                                f"{waited_output}\n{robot_ready_output}"
                            ),
                            "pid": old.pid,
                        }
                    return {
                        "key": step.key,
                        "status": "blocked",
                        "output": (
                            "기존 로봇 연결 프로세스가 motion 서비스를 띄웠지만 로봇이 "
                            "STATE_STANDBY(1)가 아닙니다.\n"
                            f"{waited_output}\n{robot_ready_output}"
                        ),
                        "pid": old.pid,
                    }
                log_tail = tail_file(process_logs.get(step.key))
                return {
                    "key": step.key,
                    "status": "starting",
                    "output": (
                        "로봇 연결 프로세스가 이미 시작 중입니다. 반복 재시작하지 않습니다.\n"
                        "motion 서비스가 아직 없으면 티치펜던트/컨트롤러 상태, 네트워크, RT_HOST를 확인하세요.\n"
                        "정말 죽였다가 다시 시작하려면 '실행 중지' 후 '로봇 연결 / 스마트 재연결'을 다시 누르세요.\n"
                        f"pid={old.pid}\n"
                        f"--- readiness ---\n{waited_output}\n"
                        f"--- log tail ---\n{log_tail}"
                    ),
                    "pid": old.pid,
                }
            existing_pid, existing_cmd = find_existing_doosan_launch()
            if existing_pid is not None:
                return {
                    "key": step.key,
                    "status": "starting",
                    "output": (
                        "기존 Doosan bringup이 아직 실행/시작 중이라 반복 재시작하지 않습니다.\n"
                        "motion 서비스가 없으면 로봇 컨트롤러 안전상태/비상정지/보호정지/네트워크/RT_HOST를 먼저 확인하세요.\n"
                        "정말 중복 노드를 정리하고 다시 시작하려면 '실행 중지' 후 '로봇 연결 / 스마트 재연결'을 다시 누르세요.\n"
                        f"pid={existing_pid}\ncmd={existing_cmd[:500]}\n"
                        f"--- readiness ---\n{ready_output}"
                    ),
                    "pid": existing_pid,
                }
            cleanup_events = cleanup_doosan_stack()
            # Give DDS/service discovery a short moment to forget killed duplicate nodes.
            time.sleep(1.5)
            restart_output = "\n".join(cleanup_events)
        elif step.key == "connect_gripper":
            cleanup_events = cleanup_rg2_stack()
            # DDS may keep stale service names briefly after a killed RG2 wrapper.
            time.sleep(1.0)
            restart_output = "\n".join(cleanup_events)
        elif step.key == "start_camera":
            cleanup_events = cleanup_camera_stack()
            # Avoid duplicate /camera/camera nodes from previous panel attempts.
            time.sleep(1.0)
            restart_output = "\n".join(cleanup_events)
        elif step.key == "side_grip":
            # Cleanup and PR #20 preflight already ran above. Do not repeat it here;
            # repeated cleanup sleeps were making the manual picker feel frozen.
            restart_output = preflight_output
        else:
            old = processes.get(step.key)
            if old and old.poll() is None:
                return {
                    "key": step.key,
                    "status": "running",
                    "output": "이미 실행 중입니다.\n" + tail_file(process_logs.get(step.key)),
                    "pid": old.pid,
                }
        log_path = background_log_path(step.key)
        log_handle = log_path.open("w", encoding="utf-8", buffering=1)
        log_handle.write(f"[Azas panel] command: {cmd}\n\n")
        proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            cwd=str(ROOT),
            env=env,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        log_handle.close()
        processes[step.key] = proc
        process_logs[step.key] = log_path
        if step.key == "connect_robot":
            ready, waited_output = wait_for_motion_services_ready(
                env["SERVICE_PREFIX"],
                timeout_sec=35.0,
                proc=proc,
            )
            if ready:
                robot_ready, robot_ready_output = doosan_robot_ready(env["SERVICE_PREFIX"])
                output = f"{cmd}\n--- log ---\n{log_path}\n--- readiness ---\n{waited_output}\n{robot_ready_output}"
                if restart_output:
                    output = f"{restart_output}\n--- start command ---\n{output}"
                if robot_ready:
                    return {
                        "key": step.key,
                        "status": "started",
                        "pid": proc.pid,
                        "output": output,
                    }
                return {
                    "key": step.key,
                    "status": "blocked",
                    "pid": proc.pid,
                    "output": (
                        "로봇 연결 프로세스는 시작됐고 motion 서비스도 보이지만 "
                        "로봇이 STATE_STANDBY(1)가 아닙니다.\n"
                        + output
                    ),
                }
            if proc.poll() is None:
                output = (
                    f"{cmd}\n--- log ---\n{log_path}\n--- readiness ---\n{waited_output}\n"
                    "아직 시작 중입니다. 몇 초 뒤 '연결 확인'만 다시 눌러주세요."
                )
                if restart_output:
                    output = f"{restart_output}\n--- start command ---\n{output}"
                return {
                    "key": step.key,
                    "status": "starting",
                    "pid": proc.pid,
                    "output": output,
                }
        else:
            # PR #20 side_grip is a manual OpenCV-window node. It should stay
            # alive waiting for the operator's `p`/Esc key, so the panel must
            # return quickly instead of blocking until that node exits.
            time.sleep(3.0 if step.key == "side_grip" else 2.0)
        if proc.poll() is not None:
            output = tail_file(log_path)
            if restart_output:
                output = f"{restart_output}\n--- start output ---\n{output}"
            return {"key": step.key, "status": "failed", "returncode": proc.returncode, "output": output}
        if step.key == "connect_gripper":
            required = ["/jarvis/rg2/open", "/jarvis/rg2/close", "/jarvis/rg2/set_width"]
            ready, waited_output = wait_for_required_services(required, timeout_sec=12.0, proc=proc)
            output = f"{cmd}\n--- log ---\n{log_path}\n--- readiness ---\n{waited_output}"
            if restart_output:
                output = f"{restart_output}\n--- start command ---\n{output}"
            if ready:
                output += (
                    "\n[Azas] RG2 ROS services are ready. Note: azas_gripper RG2 wrapper has no physical "
                    "finger-position feedback, so movement still must be visually confirmed."
                )
                return {"key": step.key, "status": "started", "pid": proc.pid, "output": output}
            output += "\n[Azas] RG2 bridge is still starting; retry gripper connection if services stay absent."
            return {"key": step.key, "status": "starting", "pid": proc.pid, "output": output}
        if step.key == "start_collision_scene":
            ready, waited_output = wait_for_collision_object_sample(env=env, timeout_sec=10.0, proc=proc)
            output = f"{cmd}\n--- log ---\n{log_path}\n--- readiness ---\n{waited_output}"
            if restart_output:
                output = f"{restart_output}\n--- start command ---\n{output}"
            if ready:
                return {"key": step.key, "status": "started", "pid": proc.pid, "output": output}
            return {"key": step.key, "status": "starting", "pid": proc.pid, "output": output}
        if step.key == "start_camera":
            ready, waited_output = wait_for_camera_topic_samples(env=env, timeout_sec=15.0, proc=proc)
            output = f"{cmd}\n--- log ---\n{log_path}\n--- readiness ---\n{waited_output}"
            if ready:
                return {"key": step.key, "status": "started", "pid": proc.pid, "output": output}
            return {"key": step.key, "status": "starting", "pid": proc.pid, "output": output}
        if step.key == "detect_cup_lid":
            ready, waited_output = wait_for_cup_detection_sample(env=env, timeout_sec=10.0, proc=proc)
            output = f"{cmd}\n--- log ---\n{log_path}\n--- readiness ---\n{waited_output}"
            if ready:
                return {"key": step.key, "status": "started", "pid": proc.pid, "output": output}
            return {"key": step.key, "status": "starting", "pid": proc.pid, "output": output}
        output = f"{cmd}\n--- log ---\n{log_path}"
        if restart_output:
            output = f"{restart_output}\n--- start command ---\n{cmd}"
            output += f"\n--- log ---\n{log_path}"
        if step.key == "side_grip":
            output += (
                "\n[Azas] PR #20 manual side_grip 노드를 백그라운드로 시작했습니다. "
                "YOLO/OpenCV 창에서 컵을 확인한 뒤 p 키를 누르면 잡기 동작이 실행되고, "
                "Esc/q로 종료합니다. 패널은 수동 입력 대기 때문에 더 이상 3분씩 블로킹하지 않습니다."
                f"\n--- log tail ---\n{tail_file(log_path, max_chars=4000)}"
            )
        return {
            "key": step.key,
            "status": "started",
            "pid": proc.pid,
            "output": output,
        }

    try:
        completed = subprocess.run(
            ["bash", "-lc", cmd],
            cwd=str(ROOT),
            env=env,
            input="ENABLE_REAL_ROBOT_MOTION\n",
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=run_timeout_for_step(step),
            check=False,
        )
        output = completed.stdout
        if preflight_output:
            output = f"{preflight_output}\n--- command output ---\n{output}"
        if completed.returncode == 0:
            failure = run_output_failure(step, output)
            if failure is not None:
                output = f"{output}\n{failure}\n"
                return {
                    "key": step.key,
                    "status": "failed",
                    "returncode": 1,
                    "output": output,
                }
            if step.key == "status_check":
                failure = status_check_failure(output)
                if failure is not None:
                    output = f"{output}\n{failure}\n"
                    return {
                        "key": step.key,
                        "status": "failed",
                        "returncode": 1,
                        "output": output,
                    }
            target_xyz = target_xyz_for_step(step.key)
            if target_xyz is not None:
                reached, verify_output = wait_for_xyz_target(env["SERVICE_PREFIX"], target_xyz)
                output = f"{output}\n--- post-motion verification ---\n{verify_output}\n"
                if not reached:
                    return {
                        "key": step.key,
                        "status": "failed",
                        "returncode": 1,
                        "output": output,
                    }
        return {
            "key": step.key,
            "status": "passed" if completed.returncode == 0 else "failed",
            "returncode": completed.returncode,
            "output": output,
        }
    except subprocess.TimeoutExpired as exc:
        output = text_output(exc.stdout)
        if preflight_output:
            output = f"{preflight_output}\n--- command output ---\n{output}"
        return {"key": step.key, "status": "timeout", "output": output}


def stop_all() -> dict[str, Any]:
    stopped: list[dict[str, Any]] = []
    for key, proc in list(processes.items()):
        if proc.poll() is None:
            events = terminate_process_tree(proc, label=key, grace_sec=5.0)
            stopped.append({"key": key, "pid": proc.pid, "events": events})
    return {"stopped": stopped}


def cleanup_all_processes() -> dict[str, Any]:
    """Explicit operator cleanup button: stop tracked jobs plus stale robot/panel helpers."""
    stopped = stop_all()
    events: list[str] = []
    events.extend(cleanup_run_step_stack(grace_sec=3.0))
    events.extend(cleanup_side_grip_stack(grace_sec=3.0))
    events.extend(cleanup_camera_stack(grace_sec=3.0))
    events.extend(cleanup_doosan_stack(grace_sec=3.0))
    events.extend(
        cleanup_matching_processes(
            AUXILIARY_STACK_PATTERNS,
            label="aux cleanup",
            grace_sec=3.0,
        )
    )
    events.extend(stop_ros2_daemon())
    return {"stopped": stopped.get("stopped", []), "cleanup": events}


class Handler(BaseHTTPRequestHandler):
    def send_json(self, data: Any, status: int = 200) -> None:
        body = json.dumps(data, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in {"/", "/index.html"}:
            body = HTML_PATH.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/steps":
            preview_payload = {
                "robot_host": os.environ.get("ROBOT_HOST", DEFAULT_ROBOT_HOST),
                "robot_name": os.environ.get("ROBOT_NAME", "dsr01"),
                "service_prefix": os.environ.get("SERVICE_PREFIX", "dsr01"),
                "rg2_ip": os.environ.get("RG2_IP", "192.168.1.1"),
                "dispenser_tcp_name": os.environ.get(
                    "DISPENSER_TCP_NAME", DEFAULT_DISPENSER_TCP_NAME
                ),
                "selected_dispenser_id": os.environ.get("SELECTED_DISPENSER_ID", "2"),
            }
            data = []
            for step in STEPS:
                item = asdict(step)
                item["resolved_command"] = command_for(step, preview_payload) if step.implemented else ""
                data.append(item)
            self.send_json(data)
            return
        if path == "/api/camera_snapshot.jpg":
            ok, body, error = camera_snapshot_jpeg()
            if not ok:
                self.send_response(503)
                message = (error or "camera snapshot unavailable").encode("utf-8", errors="replace")
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(message)))
                self.end_headers()
                self.wfile.write(message)
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_json({"error": "not found"}, 404)

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        payload = json.loads(self.rfile.read(length) or b"{}")
        path = urlparse(self.path).path
        if path == "/api/run":
            selected = with_collision_scene_prereq([str(key) for key in payload.get("selected") or []])
            steps_by_key = {step.key: step for step in STEPS}
            results = [
                run_step(steps_by_key[key], payload)
                for key in selected
                if key in steps_by_key
            ]
            self.send_json({"results": results})
            return
        if path == "/api/stop":
            self.send_json(stop_all())
            return
        if path == "/api/cleanup":
            self.send_json(cleanup_all_processes())
            return
        self.send_json({"error": "not found"}, 404)

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[panel] {self.address_string()} {fmt % args}")


def main() -> int:
    host = os.environ.get("AZAS_PANEL_HOST", "127.0.0.1")
    port = int(os.environ.get("AZAS_PANEL_PORT", "8765"))
    server = ThreadingHTTPServer((host, port), Handler)
    print(f"[Azas] Robot pipeline panel: http://{host}:{port}")
    print("[Azas] Press Ctrl+C to stop the panel server.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_all()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
