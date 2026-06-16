#!/usr/bin/env python3
"""Local button panel for supervised Azas robot pipeline commands."""

from __future__ import annotations

import errno
import json
import os
import re
import shlex
import shutil
import signal
import subprocess
import threading
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

try:
    import yaml
except ImportError:  # pragma: no cover - panel can still report a fail-closed blocker.
    yaml = None


ROOT = Path(__file__).resolve().parents[2]
HTML_PATH = ROOT / "docs" / "robot_pipeline_control.html"
ROS_SETUP = (
    "source /opt/ros/humble/setup.bash && "
    "mkdir -p /tmp/azas_ros_logs && export ROS_LOG_DIR=/tmp/azas_ros_logs && "
    "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-9} && "
    "export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1} && "
    "export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4} && "
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
DEFAULT_RT_HOST = "0.0.0.0"
DEFAULT_ROS_DOMAIN_ID = "9"
DEFAULT_YOLO_MODEL_PATH = ROOT / "local_models" / "best.pt"
CUP_UPRIGHTING_YOLO_MODEL_PATH = (
    ROOT / "src" / "azas_perception" / "config" / "yolo_cup_uprighting_best.pt"
)
PR20_YOLO_MODEL_PATH = DEFAULT_YOLO_MODEL_PATH
DEFAULT_DISPENSER_TCP_NAME = "GripperDA_v1_jarvis"
DEFAULT_LINK6_TCP_NAME = "azas_link6_tcp"
CALIBRATION_CONFIG_PATH = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"
HAND_EYE_TF_TARGET_FRAME = "base_link"
HAND_EYE_TF_SOURCE_FRAME = "camera_color_optical_frame"
FAST_MOVE_VELOCITY = "30"
FAST_MOVE_ACCELERATION = "30"
RVIZ_PREVIEW_ROS_DOMAIN_ID = "79"
BACKGROUND_LOG_DIR = ROOT / "log" / "panel"
COMMAND_OVERRIDES_PATH = ROOT / "outputs" / "panel_command_overrides.json"
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
    "j2": "10",
    "j3": "32",
    "j4": "0",
    "j5": "100",
    "j6": "90",
}
_DISPENSER_PRESS_TARGETS_DEFAULT: dict[str, str] = {
    "1": "red",
    "2": "green",
    "3": "yellow",
    "4": "blue",
}
DISPENSER_COLOR_MAP_PATH = ROOT / "outputs" / "dispenser_color_map.json"
DISPENSER_COLOR_MAP_FAILED_PATH = ROOT / "outputs" / "dispenser_color_map.json.failed"
LATEST_RECIPE_PATH = ROOT / "outputs" / "latest_recipe.json"


def measured_color_scan_joints() -> dict[str, str]:
    """Return operator-measured color-scan joints from calibration.yaml.

    Falls back to the legacy camera-view joints only if the measured config is
    unavailable, so the panel remains usable while still preferring calibration.
    """

    joints = dict(CAMERA_TABLE_VIEW_JOINTS)
    if yaml is None or not CALIBRATION_CONFIG_PATH.exists():
        return joints
    try:
        data = yaml.safe_load(CALIBRATION_CONFIG_PATH.read_text(encoding="utf-8")) or {}
        values = data.get("color_scan_pose", {}).get("joints_deg")
        if not isinstance(values, list) or len(values) != 6:
            return joints
        parsed = [float(value) for value in values]
    except Exception:
        return joints
    return {f"j{index + 1}": f"{value:.6g}" for index, value in enumerate(parsed)}


def load_command_overrides() -> dict[str, str]:
    if not COMMAND_OVERRIDES_PATH.exists():
        return {}
    try:
        loaded = json.loads(COMMAND_OVERRIDES_PATH.read_text(encoding="utf-8"))
    except Exception:
        return {}
    if not isinstance(loaded, dict):
        return {}
    step_keys = {step.key for step in STEPS} if "STEPS" in globals() else set()
    return {
        str(key): str(value)
        for key, value in loaded.items()
        if isinstance(value, str) and (not step_keys or str(key) in step_keys)
    }


def save_command_override(step_key: str, command: str) -> dict[str, str]:
    step_keys = {step.key for step in STEPS}
    if step_key not in step_keys:
        raise ValueError(f"unknown step key: {step_key}")
    overrides = load_command_overrides()
    command = command.strip()
    if command:
        overrides[step_key] = command
    else:
        overrides.pop(step_key, None)
    COMMAND_OVERRIDES_PATH.parent.mkdir(parents=True, exist_ok=True)
    COMMAND_OVERRIDES_PATH.write_text(
        json.dumps(overrides, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return overrides


def _load_dispenser_press_targets() -> dict[str, str]:
    base = dict(_DISPENSER_PRESS_TARGETS_DEFAULT)
    if DISPENSER_COLOR_MAP_PATH.exists():
        try:
            loaded = json.loads(DISPENSER_COLOR_MAP_PATH.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                base.update({str(k): str(v) for k, v in loaded.items()})
        except Exception:
            pass
    return base


DISPENSER_PRESS_TARGETS: dict[str, str] = _load_dispenser_press_targets()


def _read_json_file(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json_file_immediately(path: Path, data: Any) -> None:
    """Atomically write JSON and flush it to disk before returning to the UI."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.{threading.get_ident()}.tmp")
    with tmp.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)
    dir_fd = os.open(str(path.parent), os.O_RDONLY)
    try:
        os.fsync(dir_fd)
    finally:
        os.close(dir_fd)


def _unlink_file_immediately(path: Path) -> None:
    if not path.exists():
        return
    path.unlink()
    dir_fd = os.open(str(path.parent), os.O_RDONLY)
    try:
        os.fsync(dir_fd)
    finally:
        os.close(dir_fd)


def _normalize_color_map(raw: Any) -> dict[str, str]:
    if not isinstance(raw, dict):
        raise ValueError("color map must be a JSON object")
    normalized = {str(key): str(value).lower().strip() for key, value in raw.items()}
    return {key: normalized.get(key, "") for key in ("1", "2", "3", "4")}


def _file_timestamp(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"exists": False, "mtime": None, "age_sec": None}
    stat = path.stat()
    return {
        "exists": True,
        "mtime": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(stat.st_mtime)),
        "age_sec": round(max(time.time() - stat.st_mtime, 0.0), 3),
    }


def _compact_dispenser_sequence(sequence: list[str]) -> str:
    groups: list[str] = []
    index = 0
    while index < len(sequence):
        dispenser_id = sequence[index]
        count = 1
        index += 1
        while index < len(sequence) and sequence[index] == dispenser_id:
            count += 1
            index += 1
        groups.append(f"{dispenser_id}x{count}")
    return ",".join(groups)


def _recipe_color_pumps(recipe: Any) -> tuple[list[tuple[str, int]], list[str]]:
    color_pumps: list[tuple[str, int]] = []
    issues: list[str] = []

    def add(color: Any, count: Any = 1) -> None:
        color_name = str(color).lower().strip()
        if not color_name:
            return
        try:
            pump_count = int(count)
        except (TypeError, ValueError):
            issues.append(f"invalid pump count for color: {color_name}")
            return
        if pump_count < 1:
            issues.append(f"pump count must be >=1 for color: {color_name}")
            return
        color_pumps.append((color_name, pump_count))

    if isinstance(recipe, list):
        for item in recipe:
            if isinstance(item, dict):
                color = item.get("color") or item.get("name")
                count = item.get("pumps", item.get("count", item.get("presses", 1)))
                if color:
                    add(color, count)
            else:
                add(item, 1)
        return color_pumps, issues

    if not isinstance(recipe, dict):
        issues.append("recipe JSON must be an object or list")
        return color_pumps, issues

    colors = recipe.get("colors")
    pumps = None
    for key in ("pumps", "presses", "counts"):
        if key in recipe:
            pumps = recipe.get(key)
            break
    if isinstance(colors, list):
        if not isinstance(pumps, dict):
            pumps = {}
        for raw_color in colors:
            color = str(raw_color).lower().strip()
            add(color, pumps.get(color, pumps.get(str(raw_color), 1)))
        return color_pumps, issues

    if isinstance(pumps, dict):
        for color, count in pumps.items():
            add(color, count)
        return color_pumps, issues

    metadata_keys = {"source", "note", "notes", "created_at", "updated_at"}
    for color, count in recipe.items():
        if str(color).lower().strip() in metadata_keys:
            continue
        if isinstance(count, (int, float, str)):
            add(color, count)

    if not color_pumps:
        issues.append("recipe has no executable colors")
    return color_pumps, issues


def _color_recipe_direct_arg(payload: dict[str, Any]) -> str:
    recipe_override = str(payload.get("recipe_dispenser_ids") or "").strip()
    if not recipe_override:
        return ""
    tokens = [token.strip() for token in re.split(r"[,;]+", recipe_override) if token.strip()]
    numeric_dispenser_override = bool(tokens) and all(
        re.match(r"^[1-4](?:\s*(?:x|:)\s*\d+)?$", token.lower())
        for token in tokens
    )
    if numeric_dispenser_override:
        return f" --dispenser-ids {shlex.quote(recipe_override)}"
    direct_color_map = json.dumps(_load_dispenser_press_targets(), ensure_ascii=False)
    return (
        f" --colors {shlex.quote(recipe_override)}"
        f" --color-map-json {shlex.quote(direct_color_map)}"
    )


def color_recipe_sequence_command(payload: dict[str, Any]) -> str:
    cup_holder_x_offset_m = str(
        payload.get("cup_holder_place_final_x_offset_m")
        or os.environ.get("CUP_HOLDER_PLACE_FINAL_X_OFFSET_M")
        or "0.015"
    ).strip()
    cup_holder_rz_offset_deg = str(
        payload.get("cup_holder_rz_offset_deg")
        or os.environ.get("CUP_HOLDER_RZ_OFFSET_DEG")
        or "-1.0"
    ).strip()
    return (
        f"cd {ROOT} && {ROS_SETUP} && "
        "python3 tools/run/run_color_recipe_sequence.py --execute --confirm"
        f"{_color_recipe_direct_arg(payload)}"
        f" --cup-holder-place-final-x-offset-m {shlex.quote(cup_holder_x_offset_m)}"
        f" --cup-holder-rz-offset-deg {shlex.quote(cup_holder_rz_offset_deg)}"
    )


def chain_recipe_after_manual_command(manual_cmd: str, payload: dict[str, Any], label: str) -> str:
    recipe_cmd = color_recipe_sequence_command(payload)
    return (
        f"( {manual_cmd} ); "
        "manual_rc=$?; "
        "if [ ${manual_rc} -eq 0 ]; then "
        f"echo '[Azas] {label} 성공 메시지 확인 -> 통합 디스펜서 색상 레시피를 자동 실행합니다.'; "
        "echo '[Azas] auto_integrated_dispenser_recipe=true'; "
        f"{recipe_cmd}; "
        "else "
        f"echo '[Azas] {label} 실패/중단 rc='${{manual_rc}}' -> 디스펜서 레시피 실행을 건너뜁니다.'; "
        "exit ${manual_rc}; "
        "fi"
    )


def chain_shake_after_lid_command(lid_cmd: str, payload: dict[str, Any]) -> str:
    """Run holder re-pick + shake immediately after ArUco lid close success.

    The lid-grip launch is an OpenCV/manual ROS launch that stays alive after a
    successful `p`-triggered sequence.  Waiting for the process to exit would
    block the next motion indefinitely, so the panel chain watches the planner's
    `/jarvis/lid_gripper/status` success event, terminates the lid preview
    launch, then starts the existing measured cup-holder re-pick/shake command.
    """

    steps_by_key = {step.key: step for step in STEPS}
    shake_cmd = command_for(steps_by_key["shake_closed_cup"], payload)
    wait_script = ROOT / "tools" / "run" / "wait_for_lid_grip_status.py"
    wait_cmd = (
        f"cd {ROOT} && {ROS_SETUP} && "
        f"python3 {shlex.quote(str(wait_script))} "
        "--timeout-sec \"${LID_GRIP_STATUS_TIMEOUT_SEC:-240}\" "
        "--success-status motion_sequence_requested"
    )
    return (
        f"( {lid_cmd} ) & "
        "lid_pid=$!; "
        f"( {wait_cmd} ) & "
        "wait_pid=$!; "
        "while true; do "
        "if ! kill -0 ${wait_pid} 2>/dev/null; then "
        "wait ${wait_pid}; wait_rc=$?; break; "
        "fi; "
        "if ! kill -0 ${lid_pid} 2>/dev/null; then "
        "wait ${lid_pid}; lid_rc=$?; "
        "sleep 1; "
        "if ! kill -0 ${wait_pid} 2>/dev/null; then "
        "wait ${wait_pid}; wait_rc=$?; break; "
        "fi; "
        "echo '[Azas] lid_grip_close launch exited before ArUco success status; shake chain blocked.'; "
        "kill -TERM ${wait_pid} 2>/dev/null || true; "
        "wait ${wait_pid} 2>/dev/null || true; "
        "if [ ${lid_rc} -eq 0 ]; then exit 1; else exit ${lid_rc}; fi; "
        "fi; "
        "sleep 1; "
        "done; "
        "kill -TERM ${lid_pid} 2>/dev/null || true; "
        "wait ${lid_pid} 2>/dev/null || true; "
        "if [ ${wait_rc} -eq 0 ]; then "
        "echo '[Azas] ArUco lid_grip_close 성공 status 확인 -> 컵홀더 컵 다시 잡기 후 쉐이킹으로 바로 넘어갑니다.'; "
        "echo '[Azas] auto_holder_pick_then_shake=true'; "
        f"{shake_cmd}; "
        "else "
        "echo '[Azas] ArUco lid_grip_close 실패/타임아웃 -> 컵홀더 재픽업/쉐이킹을 건너뜁니다.'; "
        "exit ${wait_rc}; "
        "fi"
    )


def hand_eye_static_tf_command(*, compose_timeout_sec: float = 30.0) -> str:
    """Start the measured hand-eye TF publisher without inventing camera poses."""
    return (
        "ros2 run azas_perception hand_eye_static_tf_node --ros-args "
        f"-p compose_timeout_sec:={compose_timeout_sec:.1f} "
        "-p allow_direct_fallback:=false"
    )


def tmux_stack_start_command(payload: dict[str, Any]) -> str:
    robot_host = str(payload.get("robot_host") or os.environ.get("ROBOT_HOST") or DEFAULT_ROBOT_HOST)
    robot_name = str(payload.get("robot_name") or os.environ.get("ROBOT_NAME") or "dsr01")
    rt_host = str(payload.get("rt_host") or os.environ.get("RT_HOST") or DEFAULT_RT_HOST)
    rg2_ip = str(payload.get("rg2_ip") or os.environ.get("RG2_IP") or "192.168.1.1")
    ros_domain_id = str(
        payload.get("ros_domain_id")
        or os.environ.get("AZAS_PANEL_ROS_DOMAIN_ID")
        or os.environ.get("ROS_DOMAIN_ID")
        or DEFAULT_ROS_DOMAIN_ID
    )
    ros_localhost_only = str(os.environ.get("ROS_LOCALHOST_ONLY") or "0")
    return (
        f"cd {ROOT} && "
        "bash tools/run/stop_azas_all.sh && "
        "sleep 2 && "
        f"SESSION={shlex.quote(PANEL_TMUX_SESSION)} "
        f"ROS_DOMAIN_ID={shlex.quote(ros_domain_id)} "
        f"ROS_LOCALHOST_ONLY={shlex.quote(ros_localhost_only)} "
        f"ROBOT_HOST={shlex.quote(robot_host)} "
        f"ROBOT_NAME={shlex.quote(robot_name)} "
        f"RT_HOST={shlex.quote(rt_host)} "
        f"RG2_IP={shlex.quote(rg2_ip)} "
        "bash tools/run/start_azas_tmux_stack.sh"
    )


def dispenser_color_map_status() -> dict[str, Any]:
    """Read outputs/dispenser_color_map.json and derive physical dispenser order.

    If the color scan result is missing or unusable, fall back to a conservative
    physical dispenser sweep (1,2,3,4 once each) per operator request.  The
    `.failed` file is still reported so the operator can see why fallback was
    selected.
    """

    issues: list[str] = []
    output_file = _file_timestamp(DISPENSER_COLOR_MAP_PATH)
    failed_file = _file_timestamp(DISPENSER_COLOR_MAP_FAILED_PATH)
    failed_map: dict[str, str] | None = None
    if DISPENSER_COLOR_MAP_FAILED_PATH.exists():
        try:
            failed_map = _normalize_color_map(_read_json_file(DISPENSER_COLOR_MAP_FAILED_PATH))
        except Exception as exc:
            issues.append(f"failed-file read error: {exc}")

    if not DISPENSER_COLOR_MAP_PATH.exists():
        issues.append(f"missing color map: {DISPENSER_COLOR_MAP_PATH}")
        if failed_map and all(value == "unknown" for value in failed_map.values()):
            issues.append(f"failed map is all unknown: {DISPENSER_COLOR_MAP_FAILED_PATH}")
        fallback_sequence = ["1", "2", "3", "4"]
        return {
            "ok": True,
            "fallback": True,
            "fallback_reason": "; ".join(issues),
            "map": None,
            "failed_map": failed_map,
            "recipe": None,
            "sequence": fallback_sequence,
            "sequence_csv": ",".join(fallback_sequence),
            "sequence_compact": _compact_dispenser_sequence(fallback_sequence),
            "source": str(DISPENSER_COLOR_MAP_PATH),
            "failed_source": str(DISPENSER_COLOR_MAP_FAILED_PATH),
            "output_file": output_file,
            "failed_file": failed_file,
            "issues": issues,
        }

    try:
        color_map = _normalize_color_map(_read_json_file(DISPENSER_COLOR_MAP_PATH))
    except Exception as exc:
        issues.append(f"color map read error: {exc}")
        fallback_sequence = ["1", "2", "3", "4"]
        return {
            "ok": True,
            "fallback": True,
            "fallback_reason": "; ".join(issues),
            "map": None,
            "failed_map": failed_map,
            "recipe": None,
            "sequence": fallback_sequence,
            "sequence_csv": ",".join(fallback_sequence),
            "sequence_compact": _compact_dispenser_sequence(fallback_sequence),
            "source": str(DISPENSER_COLOR_MAP_PATH),
            "failed_source": str(DISPENSER_COLOR_MAP_FAILED_PATH),
            "output_file": output_file,
            "failed_file": failed_file,
            "issues": issues,
        }

    unknown_ids = [did for did, color in color_map.items() if not color or color == "unknown"]
    if unknown_ids:
        issues.append(f"unknown dispenser colors: {','.join(unknown_ids)}")

    if not LATEST_RECIPE_PATH.exists():
        issues.append(f"missing recipe: {LATEST_RECIPE_PATH}")
        recipe = None
    else:
        try:
            recipe = _read_json_file(LATEST_RECIPE_PATH)
        except Exception as exc:
            recipe = None
            issues.append(f"recipe read error: {exc}")

    color_to_id: dict[str, str] = {}
    for dispenser_id, color in color_map.items():
        if not color or color == "unknown":
            continue
        if color in color_to_id:
            issues.append(f"duplicate color mapping: {color}")
        color_to_id[color] = dispenser_id

    sequence: list[str] = []
    if recipe is not None:
        color_pumps, recipe_issues = _recipe_color_pumps(recipe)
        issues.extend(recipe_issues)
        for color, count in color_pumps:
            dispenser_id = color_to_id.get(color)
            if not dispenser_id:
                issues.append(f"recipe color has no dispenser: {color}")
                continue
            sequence.extend([dispenser_id] * count)

    if not sequence:
        issues.append("no executable dispenser sequence derived; using fallback 1,2,3,4")
        sequence = ["1", "2", "3", "4"]

    return {
        "ok": True,
        "fallback": bool(issues),
        "fallback_reason": "; ".join(issues) if issues else "",
        "map": color_map,
        "failed_map": failed_map,
        "recipe": recipe,
        "sequence": sequence,
        "sequence_csv": ",".join(sequence),
        "sequence_compact": _compact_dispenser_sequence(sequence),
        "source": str(DISPENSER_COLOR_MAP_PATH),
        "failed_source": str(DISPENSER_COLOR_MAP_FAILED_PATH),
        "output_file": output_file,
        "failed_file": failed_file,
        "recipe_source": str(LATEST_RECIPE_PATH),
        "issues": issues,
    }


def _number_list(value: Any, *, length: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) < length:
        raise ValueError(f"{label} must be a list with at least {length} numeric values")
    try:
        return [float(item) for item in value[:length]]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} contains a non-numeric value") from exc


def measured_dispenser_press_pose(dispenser_id: str) -> tuple[list[float], list[float]]:
    """Return measured base_link press pose for dispenser_N from calibration.yaml."""
    if yaml is None:
        raise RuntimeError("PyYAML is not available, cannot read calibration.yaml")
    data = yaml.safe_load(CALIBRATION_CONFIG_PATH.read_text(encoding="utf-8")) or {}
    outlets = data.get("dispenser_outlets") or {}
    block = outlets.get(str(dispenser_id))
    if not isinstance(block, dict):
        raise ValueError(f"dispenser_outlets.{dispenser_id} is missing in {CALIBRATION_CONFIG_PATH}")
    xyz_m = _number_list(
        block.get("press_pose_xyz_m"),
        length=3,
        label=f"dispenser_outlets.{dispenser_id}.press_pose_xyz_m",
    )
    rpy_deg = _number_list(
        block.get("press_pose_rpy_deg"),
        length=3,
        label=f"dispenser_outlets.{dispenser_id}.press_pose_rpy_deg",
    )
    return xyz_m, rpy_deg


def fail_closed_shell(message: str) -> str:
    return f"echo {shlex.quote('[BLOCKED] ' + message)} >&2; exit 2"


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
        "로봇 연결 / tmux 통합 재연결",
        "background",
        "tools/run/stop_azas_all.sh && sleep 2 && tools/run/start_azas_tmux_stack.sh",
        True,
        False,
        "검증된 현장 명령으로 stop_azas_all 후 azas-logic tmux 스택을 시작",
    ),
    Step(
        "start_tmux_stack",
        "tmux 연결 스택 시작",
        "background",
        "tools/run/start_azas_tmux_stack.sh",
        True,
        False,
        "검증된 tmux 방식으로 stop_azas_all 후 azas-logic 세션에 로봇, RG2 그리퍼, RealSense, joint relay를 분리 시작",
    ),
    Step(
        "stop_azas_all",
        "전체 정지 / ROS 정리",
        "run",
        "tools/run/stop_azas_all.sh",
        True,
        False,
        "azas tmux 스택과 모든 ROS 노드/좀비 프로세스를 종료하고 FastDDS 공유메모리 잔여물(/dev/shm/fastrtps_*)을 정리. 패널/에이전트 프로세스는 보호됨. 정리 후 'tmux 연결 스택 시작'으로 재시작",
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
        "workspace_collision_scene.launch.py + rg2_link6_tcp.launch.py + tumbler_collision_scene_node",
        True,
        False,
        "safety.yaml 바닥/양쪽 벽, measured dispenser 박스, link_6 부착 RG2 그리퍼 envelope, 감지 텀블러를 PlanningScene/RViz로 publish",
    ),
    Step(
        "rviz_cocktail_collision_preview",
        "RViz 칵테일 전체 동작 미리보기 / 충돌영역 반영",
        "run",
        "tools/run/run_cocktail_collision_rviz_preview.sh",
        True,
        False,
        "가상 Doosan+MoveIt RViz에서 컵 놓기→프레스→다시 잡기 전체 코스를 충돌 오브젝트 포함으로 검증. 실로봇 명령은 보내지 않음",
    ),
    Step(
        "stop_cocktail_motion_preview",
        "RViz/가상 칵테일 preview 정리",
        "run",
        "tools/run/stop_cocktail_motion_preview.sh",
        True,
        False,
        "실제 로봇 실행 전에 virtual/emulator/RViz preview 세션을 정리해 real 서비스와 섞이지 않게 함",
    ),
    Step(
        "check_one_click_cocktail_ready",
        "실제 통합 칵테일 실행 readiness 확인",
        "run",
        "tools/run/check_one_click_cocktail_ready.sh",
        True,
        False,
        "real/virtual 세션, Doosan motion 서비스, RG2 서비스를 확인하고 현재 one-click 실행 가능 상태를 출력",
    ),
    Step(
        "check_one_click_cocktail_result",
        "실제 통합 칵테일 결과 로그 확인",
        "run",
        "tools/run/check_one_click_cocktail_result.sh",
        True,
        False,
        "one-click 실제 실행 로그에서 컵놓기→프레스→다시잡기 완료 증거와 실패 marker를 판정",
    ),
    Step(
        "run_one_click_cocktail_real",
        "실제 통합 칵테일 one-click 실행",
        "run",
        "tools/run/run_one_click_cocktail_real.sh",
        True,
        True,
        "실제 로봇 연결/그리퍼/충돌장면 준비 후 컵놓기→프레스→다시잡기 통합 사이클을 한 번에 실행",
    ),
    Step(
        "run_cocktail_now_real",
        "실제 칵테일 NOW 실행",
        "run",
        "tools/run/run_cocktail_now_real.sh",
        True,
        True,
        "preview 정리, readiness/config 검증, 실제 로봇 연결, 컵놓기→프레스→다시잡기와 결과 판정을 한 진입점으로 실행",
    ),
    Step("home_robot", "로봇 원위치 / HOME", "run", "tools/run/direct_movej_joints.py --j1 0 --j2 0 --j3 90 --j4 0 --j5 90 --j6 0", True, True, "실제모션 후보: HOME 관절값 [0, 0, 90, 0, 90, 0]"),
    Step(
        "lift_robot",
        "기본 카메라 보기 자세",
        "run",
        "tools/run/direct_movej_joints.py --j1 0 --j2 10 --j3 32 --j4 0 --j5 100 --j6 90",
        True,
        True,
        "기본 카메라 보기 관절 자세: [0, 10, 32, 0, 100, 90]°",
    ),
    Step(
        "side_grip_camera_home",
        "side-grip 카메라 홈 자세",
        "run",
        "tools/run/direct_movej_joints.py --j1 3.0 --j2 -12.7 --j3 44.0 --j4 -9.0 --j5 133.0 --j6 90.0",
        True,
        True,
        "창현 side-grip 노드의 camera_home_mode:=joint 기본 관절 자세로 이동해 컵 인식 시야를 맞춤",
    ),
    Step(
        "move_to_color_scan_pose",
        "색상 스캔 검증 포즈 이동 [visible-handle]",
        "run",
        "tools/run/direct_movej_joints.py --j1 0 --j2 10 --j3 32 --j4 0 --j5 100 --j6 90 --velocity 30 --acceleration 30 --execute --confirm ENABLE_DIRECT_MOVEJ",
        True,
        True,
        "색상 스캔 전 2026-06-08 검증 포즈로 이동. 카메라 화면에서 보이는 디스펜서 핸들 색을 직접 검출할 수 있는 자세",
    ),
    Step(
        "rviz_color_scan_pose_preview",
        "색상 스캔 자세 RViz 미리보기 / 무모션",
        "background",
        "tools/run/show_color_scan_pose_rviz.sh",
        True,
        False,
        "RViz-only /joint_states로 color_scan_pose [0,10,32,0,100,90]°를 표시. 실제 로봇 명령 없음",
    ),
    Step(
        "color_scan",
        "디스펜서 색상 스캔",
        "run",
        "tools/run/dispenser_color_scan_ros.sh",
        True,
        False,
        "카메라 화면의 visible colored handle blob을 직접 검출해 왼쪽→오른쪽을 디스펜서 1~4로 매핑하고 outputs/dispenser_color_map.json 저장. TF 투영은 보조 경로",
    ),
    Step("voice_input", "수빈 STT/주문 UI 시작", "background", "ros2 launch azas_voice azas_voice.launch.py", True, False, "voice screen(8090) + STT topic(/stt_result) → recipe mapper → conversation manager. 로봇 좌표/모션은 만들지 않음"),
    Step(
        "listen_stt_recipe",
        "수빈 STT 레시피 확정 대기 (60초)",
        "run",
        "tools/run/listen_stt_recipe.py --timeout 60",
        True,
        False,
        "사용자가 메뉴를 말하고 '응'으로 확정하면 /azas/voice/confirmed_recipe_decision 수신 → outputs/latest_recipe.json 저장",
    ),
    Step(
        "run_color_recipe_sequence",
        "통합 디스펜서 레시피 실행",
        "run",
        "tools/run/run_color_recipe_sequence.py --execute --confirm",
        True,
        True,
        "latest_recipe.json + dispenser_color_map.json → 컵 놓기→프레스→컵 다시 잡기/다음 디스펜서 이동. 재집기 Z 상승은 특이점 시 IK MoveJoint로 우회하고 legacy 저자세 직행 fallback은 사용하지 않음",
    ),
    Step(
        "side_grip",
        "PR #20 RealSense 컵 인식 후 side grip",
        "background",
        "SIDE_TARGET_X_OFFSET_M=-0.020 SIDE_TARGET_JOINT6_INSET_M=0.070 SIDE_TARGET_JOINT6_INSET_SIGN=1.0 bash tools/run/run_changhyun_side_grip_direct.sh",
        True,
        True,
        "OpenCV 창에서 컵 확인 후 p 키로 side-grip 실행. 패널은 direct runner를 tmux로 띄우며 기본 X 보정은 -20mm, y축 side-grip target 보정은 컵 방향 70mm",
    ),
    Step(
        "cup_uprighting",
        "소명 누운 컵 세우기 / cup uprighting",
        "background",
        "ros2 launch azas_cup_uprighting yolo_cup_uprighting.launch.py",
        True,
        True,
        "RealSense + YOLO 기반 누운 컵 직립화. OpenCV 창에서 컵 확인 후 p 키로 실행, Esc/q로 종료하는 수동 실제모션 단계",
    ),
    Step("gripper_soft_grasp", "그리퍼 살짝 잡기", "run", "ros2 service call /jarvis/rg2/set_width azas_interfaces/srv/SetGripper", True, True, "큰 컵용: 완전 close 대신 폭 75mm/약한 힘으로 살짝 오므림"),
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
        "디스펜서 1 누르기 / measured",
        "run",
        "ros2 run azas_dispenser dispenser_press_node --ros-args -p use_taught_posx:=false",
        True,
        True,
        "calibration.yaml dispenser_outlets.1 press_pose 측정값 사용: 현재 위치 수직상승→프레스 위치→하강 누름→상승→후퇴 대기",
    ),
    Step(
        "press_dispenser_2",
        "디스펜서 2 누르기 / measured",
        "run",
        "ros2 run azas_dispenser dispenser_press_node --ros-args -p use_taught_posx:=false",
        True,
        True,
        "calibration.yaml dispenser_outlets.2 press_pose 측정값 사용: 현재 위치 수직상승→프레스 위치→하강 누름→상승→후퇴 대기",
    ),
    Step(
        "press_dispenser_3",
        "디스펜서 3 누르기 / measured",
        "run",
        "ros2 run azas_dispenser dispenser_press_node --ros-args -p use_taught_posx:=false",
        True,
        True,
        "calibration.yaml dispenser_outlets.3 press_pose 측정값 사용: 현재 위치 수직상승→프레스 위치→하강 누름→상승→후퇴 대기",
    ),
    Step(
        "press_dispenser_4",
        "디스펜서 4 누르기 / measured",
        "run",
        "ros2 run azas_dispenser dispenser_press_node --ros-args -p use_taught_posx:=false",
        True,
        True,
        "calibration.yaml dispenser_outlets.4 press_pose 측정값 사용: 현재 위치 수직상승→프레스 위치→하강 누름→상승→후퇴 대기",
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
    Step(
        "pick_lid",
        "뚜껑 grip pose 계획 / 빨간 스티커",
        "background",
        "",
        True,
        False,
        "YOLO lid + 빨간 원형 스티커 + depth 평면으로 base_link lid pose와 approach/grasp/lift 후보를 발행. 실제 로봇 모션은 실행하지 않음",
    ),
    Step(
        "lid_view_pose",
        "뚜껑 보기 카메라 자세",
        "run",
        "tools/run/direct_movej_joints.py --j1 3.0 --j2 -12.7 --j3 44.0 --j4 -9.0 --j5 133.0 --j6 90.0",
        True,
        True,
        "강개발자 lid_grip_close 실행 전 손목 카메라가 뚜껑/ArUco를 보도록 이동. 현재는 side-grip 카메라 홈과 동일한 검증 후보 자세",
    ),
    Step(
        "lid_grip_close",
        "컵 뚜껑 잡고 닫기 / ArUco lid twist",
        "background",
        "",
        True,
        True,
        "강개발자 로직: ArUco DICT_6X6_250 id0 뚜껑 pose를 p키로 확정한 뒤 RG2 파지→lift→teach point 이동→J6 단계 회전으로 뚜껑을 닫음",
    ),
    Step(
        "place_cup_holder",
        "컵을 컵홀더에 놓기 / side grip",
        "run",
        "tools/run/place_side_grip_cup_in_holder.py",
        True,
        True,
        "실제모션 후보: 측정된 side_grip_place pre_place→place_final→RG2 full-open→retreat",
    ),
    Step(
        "shake_rviz_preview",
        "쉐이킹 RViz 미리보기 / 무모션",
        "background",
        "tools/run/run_rule_based_dispenser_then_shake_sim.sh",
        True,
        False,
        "실제 로봇 미사용: 별도 ROS_DOMAIN_ID에서 쉐이킹 궤적/마커를 RViz로 표시",
    ),
    Step("shake_closed_cup", "컵홀더 컵 다시 잡기 후 쉐이킹", "run", "tools/run/pick_from_cup_holder_side_grip.py && tools/run/run_rule_based_shake_real.sh", True, True, "시작 시 컵홀더에 놓인 닫힌 컵을 측정된 cup_holder.side_grip_place pose로 다시 side-grip 픽업한 뒤, J3 양수 고정 및 J4/J5/J6 트위스트 쉐이킹을 실행. 쉐이킹 성공 시 컵을 든 채 카메라 포즈(J=[3, -12.7, 44, -9, 133, 90])로 복귀해 손 검출/핸드오버 준비"),
    Step(
        "start_hand_detection",
        "손 검출 시작 / 무모션",
        "background",
        "bash tools/run/run_human_hand_detection.sh",
        True,
        False,
        "perception 전용: MediaPipe로 펼친 손바닥을 추적해 /azas/human_hand_detection으로 발행. 로봇 모션 없음",
    ),
    Step(
        "start_hand_detection_view",
        "손 검출 화면 보기",
        "background",
        "rqt_image_view /azas/human_hand_detection/overlay",
        True,
        False,
        "손 검출 overlay(랜드마크/STABLE 라벨)를 rqt_image_view 창으로 표시. 손 검출 시작 버튼이 먼저 켜져 있어야 영상이 나옴",
    ),
    Step(
        "handover_cup_to_palm",
        "쉐이킹 후 손바닥에 컵 건네기",
        "run",
        "tools/run/handover_cup_to_palm.py",
        True,
        True,
        "실제모션 HRI: 손 검출이 먼저 켜져 있어야 함. 손바닥 위로 이동 후 외력 감시하며 저속 하강, 컵 release. "
        "첫 사용 전 스펀지 테스트로 --release-tcp-above-palm-m 튜닝 필수",
    ),
]

processes: dict[str, subprocess.Popen[str]] = {}
process_logs: dict[str, Path] = {}
tmux_jobs: dict[str, dict[str, str]] = {}
RUN_LOCK = threading.Lock()
ROS_ENV_LOCK = threading.Lock()
ROS_ENV_CACHE: dict[str, str] | None = None
# Use the same tmux session as the field-tested manual workflow for long-lived
# panel jobs. The integrated reconnect command is intentionally excluded below:
# it runs stop_azas_all.sh, which kills azas-logic before recreating it, so that
# command must be launched by the panel server outside tmux.
PANEL_TMUX_SESSION = "azas-logic"
PANEL_TMUX_STEPS = {
    "connect_gripper",
    "start_camera",
    "start_camera_view",
    "detect_cup_lid",
    "start_collision_scene",
    "rviz_color_scan_pose_preview",
    "voice_input",
    "pick_lid",
    "side_grip",
    "cup_uprighting",
    "lid_grip_close",
    "shake_rviz_preview",
    "start_hand_detection",
    "start_hand_detection_view",
}
PANEL_HIDDEN_STEP_KEYS = {
    "rviz_cocktail_collision_preview",
    "rviz_color_scan_pose_preview",
    "shake_rviz_preview",
    "stop_cocktail_motion_preview",
    "check_one_click_cocktail_ready",
    "check_one_click_cocktail_result",
    "run_cocktail_now_real",
    "start_camera_view",
    "detect_cup_lid",
    "run_one_click_cocktail_real",
    "move_to_dispenser_1",
    "move_to_dispenser_2",
    "move_to_dispenser_3",
    "move_to_dispenser_4",
    "press_dispenser_1",
    "press_dispenser_2",
    "press_dispenser_3",
    "press_dispenser_4",
    "pick_from_dispenser_1",
    "pick_from_dispenser_2",
    "pick_from_dispenser_3",
    "pick_from_dispenser_4",
}
PANEL_DIRECT_TMUX_STEPS = {
    # Match the successful field workflow: the panel opens the same tmux launch
    # command and does not pre-block on slow ROS graph/service introspection.
    # The launched node/MoveIt stack still performs the actual motion checks.
    "side_grip",
    "cup_uprighting",
    "lid_grip_close",
}
PANEL_FIELD_VERIFIED_DIRECT_TMUX_STEPS = {
    # Only commands that have been observed working from the same terminal/tmux
    # mechanism are allowed to start from the panel. Static package/launch checks
    # are not enough for real-motion GUI workflows.
    "side_grip",
    "cup_uprighting",
    "lid_grip_close",
}

DOOSAN_STACK_PATTERNS = (
    "run_doosan_real_m0609.sh",
    "run_doosan_real_no_motion_m0609.sh",
    "run_emulator",
    "dsr_bringup2/lib/dsr_bringup2",
    "dsr_bringup2_moveit.launch.py",
    "dsr_controller2",
    "dsr_moveit_controller",
    "ros2_control_node",
    "controller_manager",
    "joint_state_broadcaster",
    "joint_state_relay_legacy",
    "dsr_practice/joint_state_relay",
    "joint_state_relay --ros-args",
    "azas_joint_state_relay",
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
    "lid_sticker_grip_planning.launch.py",
    "lid_grip_planner_node",
    "lid_sticker_detector_node",
    "yolo_cup_uprighting.launch.py",
    "run_rule_based_shake_real.sh",
    "run_cup_target_then_shake_rviz.sh",
    "cup_target_then_shake_rviz.launch.py",
    "show_color_scan_pose_rviz.sh",
    "color_scan_pose_rviz.launch.py",
    "run_rule_based_dispenser_then_shake_sim.sh",
    "tumbler_shake_sequence.launch.py",
    "tumbler_shake_sequence_node",
    "shake_visualizer_node",
    "m0609_shake_joint_state_node",
    "robot_connection_control.launch.py",
    "yolo_to_floor_place.launch.py",
    "tumbler_floor_place.launch.py",
    "tumbler_floor_place_node",
    "cup_detection_pose_bridge_node",
    "hand_eye_static_tf_node",
    "measured_dispenser_collision_scene_node",
    "collision_scene_rviz_publisher",
    "tumbler_collision_scene_node",
    "link6_gripper_collision_node",
    "rg2_link6_tcp.launch.py",
    "azas_rg2_link6_tcp_state_publisher",
    "--frame-id world --child-frame-id base_link",
)

COLLISION_SCENE_STACK_PATTERNS = (
    "workspace_collision_scene.launch.py",
    "workspace_collision_scene_node",
    "measured_dispenser_collision_scene_node",
    "collision_scene_rviz_publisher",
    "tumbler_collision_scene_node",
    "link6_gripper_collision_node",
    "rg2_link6_tcp.launch.py",
    "azas_rg2_link6_tcp_state_publisher",
    "--frame-id world --child-frame-id base_link",
)

RG2_STACK_PATTERNS = (
    "rg2_gripper_node",
)

CAMERA_STACK_PATTERNS = (
    "realsense2_camera rs_launch.py",
    "realsense2_camera_node",
)

SIDE_GRIP_STACK_PATTERNS = (
    "run_changhyun_side_grip_direct.sh",
    "yolo_cup_pick_node.launch.py",
    "yolo_cup_pick_node_legacy.launch.py",
    "yolo_cup_pick_legacy_node",
    "dsr_practice/yolo_cup_pick_node",
    "yolo_cup_pick_node --ros-args",
    "yolo_cup_pick_moveit_py",
    "hand_eye_static_tf_node",
    "link6_gripper_collision_node",
    "workspace_collision_scene_node",
    "--frame-id world --child-frame-id base_link",
)

CUP_UPRIGHTING_STACK_PATTERNS = (
    "yolo_cup_uprighting.launch.py",
    "azas_cup_uprighting/yolo_cup_uprighting",
    "yolo_cup_uprighting --ros-args",
    "yolo_cup_uprighting_py",
)

LID_GRIP_STACK_PATTERNS = (
    "lid_grip_close",
    "lid_sticker_detector_node",
    "lid_grip_planner_node",
    "lid_detection_pose_bridge_node",
)

RUN_STEP_STACK_PATTERNS = (
    "dispenser_press_node",
    "direct_movej_joints.py",
    "dispenser_color_scan_ros.sh",
    "run_one_click_cocktail_real.sh",
    "run_cocktail_now_real.sh",
    "run_cocktail_collision_rviz_preview.sh",
    "stop_cocktail_motion_preview.sh",
    "stop_azas_all.sh",
    "rg2_full_open_verify.sh",
    "move_to_measured_dispenser_front_hold.py",
    "pick_from_measured_dispenser_front_hold.py",
    "run_measured_dispenser_recipe_sequence.py",
    "run_color_recipe_sequence.py",
    "place_side_grip_cup_in_holder.py",
    "pick_from_cup_holder_side_grip.py",
    "teach_measured_dispenser_front_hold.py",
    "direct_movel_xyz.py",
    "ros2 service call /dsr01/",
    "ros2 control list_controllers",
)

PANEL_PROTECTED_PATTERNS = (
    "robot_pipeline_control_server.py",
    "run_robot_pipeline_control_panel.sh",
)

AGENT_PROTECTED_PATTERNS = (
    "codex",
    "codex-linux-sandbox",
    ".codex",
    "omx",
    "oh-my-codex",
    "tmux",
    "bwrap",
)


def command_line(proc: Any) -> str:
    try:
        cmdline = proc.info.get("cmdline") if hasattr(proc, "info") else proc.cmdline()
    except Exception:
        return ""
    if not cmdline:
        return ""
    return " ".join(str(part) for part in cmdline)


def installed_executable(package_name: str, executable_name: str) -> bool:
    """Best-effort check for an installed ROS package console script."""

    candidates = [
        ROOT / "install" / package_name / "lib" / package_name / executable_name,
        Path("/home/ssu/ros2_ws/install") / package_name / "lib" / package_name / executable_name,
    ]
    return any(path.exists() and os.access(path, os.X_OK) for path in candidates)


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


def ros_command_env() -> dict[str, str]:
    """Return a cached environment with ROS overlays already sourced.

    Panel status probes can call ros2 many times.  Re-sourcing every workspace
    for each probe adds about a second before the actual DDS/service operation
    starts.  Cache the sourced environment once per panel process and run short
    ros2 commands inside that environment.
    """

    global ROS_ENV_CACHE
    with ROS_ENV_LOCK:
        if ROS_ENV_CACHE is not None:
            return dict(ROS_ENV_CACHE)
        script = (
            f"{ROS_SETUP} && "
            "python3 - <<'PY'\n"
            "import json, os\n"
            "print(json.dumps(dict(os.environ)))\n"
            "PY"
        )
        completed = subprocess.run(
            ["bash", "-lc", script],
            cwd=str(ROOT),
            env=os.environ.copy(),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=8.0,
            check=False,
        )
        if completed.returncode != 0:
            raise RuntimeError("failed to source ROS environment:\n" + completed.stdout[-4000:])
        try:
            ROS_ENV_CACHE = {str(k): str(v) for k, v in json.loads(completed.stdout).items()}
        except json.JSONDecodeError as exc:
            raise RuntimeError("failed to parse sourced ROS environment:\n" + completed.stdout[-4000:]) from exc
        return dict(ROS_ENV_CACHE)


def background_log_path(step_key: str) -> Path:
    BACKGROUND_LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return BACKGROUND_LOG_DIR / f"{step_key}-{stamp}.log"


def tmux_window_name(step_key: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "-", step_key).strip("-")[:48] or "step"


def tmux_available() -> bool:
    return shutil.which("tmux") is not None


def ensure_panel_tmux_session(env: dict[str, str]) -> None:
    if not tmux_available():
        raise RuntimeError("tmux 명령을 찾을 수 없습니다.")
    has_session = subprocess.run(
        ["tmux", "has-session", "-t", PANEL_TMUX_SESSION],
        cwd=str(ROOT),
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if has_session.returncode != 0:
        subprocess.run(
            [
                "tmux",
                "new-session",
                "-d",
                "-s",
                PANEL_TMUX_SESSION,
                "-n",
                "monitor",
                "bash -lc 'echo \"[Azas panel tmux] monitor\"; exec bash'",
            ],
            cwd=str(ROOT),
            env=env,
            check=True,
        )
    subprocess.run(
        ["tmux", "set-option", "-t", PANEL_TMUX_SESSION, "remain-on-exit", "on"],
        cwd=str(ROOT),
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def kill_panel_tmux_window(step_key: str, env: dict[str, str]) -> None:
    if not tmux_available():
        return
    subprocess.run(
        ["tmux", "kill-window", "-t", f"{PANEL_TMUX_SESSION}:{tmux_window_name(step_key)}"],
        cwd=str(ROOT),
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def capture_panel_tmux_window(step_key: str, env: dict[str, str], *, max_chars: int = 6000) -> str:
    if not tmux_available():
        return ""
    result = subprocess.run(
        ["tmux", "capture-pane", "-t", f"{PANEL_TMUX_SESSION}:{tmux_window_name(step_key)}", "-p", "-S", "-220"],
        cwd=str(ROOT),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    return result.stdout[-max_chars:]


def start_panel_tmux_window(step_key: str, cmd: str, env: dict[str, str]) -> Path:
    ensure_panel_tmux_session(env)
    kill_panel_tmux_window(step_key, env)
    log_path = background_log_path(f"tmux_{step_key}")
    log_path.write_text(f"[Azas panel tmux] command: {cmd}\n\n", encoding="utf-8")
    exports = " ".join(
        f"export {name}={shlex.quote(str(env.get(name, '')))};"
        for name in (
            "ROS_DOMAIN_ID",
            "ROS_LOCALHOST_ONLY",
            "FASTDDS_BUILTIN_TRANSPORTS",
            "ROBOT_HOST",
            "ROBOT_NAME",
            "SERVICE_PREFIX",
            "DISPLAY",
            "XAUTHORITY",
        )
    )
    shell_cmd = (
        f"{exports} cd {shlex.quote(str(ROOT))}; "
        "set -o pipefail; "
        f"({cmd}) 2>&1 | tee -a {shlex.quote(str(log_path))}; "
        "rc=${PIPESTATUS[0]}; "
        "echo; echo \"[Azas panel tmux] command exited rc=${rc}\" | tee -a "
        f"{shlex.quote(str(log_path))}; "
        "exec bash"
    )
    subprocess.run(
        [
            "tmux",
            "new-window",
            "-t",
            PANEL_TMUX_SESSION,
            "-n",
            tmux_window_name(step_key),
            "bash -lc " + shlex.quote(shell_cmd),
        ],
        cwd=str(ROOT),
        env=env,
        check=True,
    )
    tmux_jobs[step_key] = {
        "session": PANEL_TMUX_SESSION,
        "window": tmux_window_name(step_key),
        "log": str(log_path),
    }
    process_logs[step_key] = log_path
    return log_path


def run_background_step_in_tmux(
    step: Step,
    cmd: str,
    env: dict[str, str],
    *,
    restart_output: str = "",
) -> dict[str, Any]:
    try:
        log_path = start_panel_tmux_window(step.key, cmd, env)
    except Exception as exc:
        return {
            "key": step.key,
            "status": "failed",
            "output": f"tmux 창 실행을 시작하지 못했습니다: {exc}\n--- command ---\n{cmd}",
        }

    base_output = f"{cmd}\n--- tmux ---\nsession={PANEL_TMUX_SESSION} window={tmux_window_name(step.key)}\n--- log ---\n{log_path}"
    if restart_output:
        base_output = f"{restart_output}\n--- start command ---\n{base_output}"

    if step.key == "connect_robot":
        output = (
            f"{base_output}\n"
            "--- tmux tail ---\n"
            f"{capture_panel_tmux_window(step.key, env, max_chars=4000)}\n"
            "[Azas] robot tmux window started. 패널은 여기서 ROS graph/service 조회로 블로킹하지 않습니다. "
            "준비 확인은 몇 초 뒤 '연결 확인'을 누르거나 tmux 로그를 보세요."
        )
        return {"key": step.key, "status": "started", "output": output}

    if step.key == "connect_gripper":
        output = (
            f"{base_output}\n"
            "--- tmux tail ---\n"
            f"{capture_panel_tmux_window(step.key, env, max_chars=4000)}\n"
            "[Azas] gripper tmux window started. 서비스 확인은 status_check/로그에서 분리해서 봅니다."
        )
        return {"key": step.key, "status": "started", "output": output}

    if step.key == "start_camera":
        # RealSense launch is long-running and can publish normally while the
        # local ros2cli graph daemon is stale or blocked.  Keep the panel
        # behavior aligned with the working terminal/tmux workflow: start the
        # camera in its own window and let downstream vision nodes consume it.
        output = (
            f"{base_output}\n"
            "--- readiness ---\n"
            "camera tmux window started; topic sampling is advisory and is not used as a panel blocking gate.\n"
            "--- tmux tail ---\n"
            f"{capture_panel_tmux_window(step.key, env, max_chars=4000)}"
        )
        return {"key": step.key, "status": "started", "output": output}

    if step.key == "start_collision_scene":
        output = (
            f"{base_output}\n"
            "--- tmux tail ---\n"
            f"{capture_panel_tmux_window(step.key, env, max_chars=4000)}\n"
            "[Azas] collision scene tmux window started. TF/collision topic echo를 패널 실행 경로에서 블로킹하지 않습니다."
        )
        return {"key": step.key, "status": "started", "output": output}

    output = base_output + "\n--- tmux tail ---\n" + capture_panel_tmux_window(step.key, env, max_chars=4000)
    if step.key == "side_grip":
        output += (
            "\n[Azas] side_grip은 tmux 창에서 OpenCV 화면을 띄워 대기합니다. "
            "컵을 확인한 뒤 p 키를 누르면 잡기 동작이 실행됩니다. "
            "디스펜서 collision은 켠 상태이며 pre_pick_joint1_clearance_deg=12.0으로 보정했습니다."
        )
    return {"key": step.key, "status": "started", "output": output}


def terminate_process_tree(proc: subprocess.Popen[str], *, label: str, grace_sec: float = 3.0) -> list[str]:
    """Terminate a Popen process and its children without killing the panel server."""
    events: list[str] = []
    if proc.poll() is not None:
        return events
    try:
        # Panel-spawned commands use start_new_session=True.  Signal the whole
        # process group first so ros2 launch children do not keep executing after
        # the wrapper shell exits.
        os.killpg(proc.pid, signal.SIGINT)
        events.append(f"{label}: SIGINT process group pgid={proc.pid}")
    except ProcessLookupError:
        return events
    except OSError as exc:
        events.append(f"{label}: process-group SIGINT failed: {exc}")
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

    try:
        proc.wait(timeout=grace_sec)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
            events.append(f"{label}: SIGKILL process group pgid={proc.pid}")
        except OSError:
            proc.kill()
            events.append(f"{label}: killed pid={proc.pid}")
    else:
        events.append(f"{label}: stopped pid={proc.pid}")
    return events


def protected_pids() -> set[int]:
    """Return the panel process and its ancestors, which cleanup must not kill."""
    pids = {os.getpid()}
    if psutil is None:
        return pids
    try:
        current = psutil.Process(os.getpid())
        pids.update(parent.pid for parent in current.parents())
    except psutil.Error:
        pass
    return pids


def is_protected_process(proc: Any, protected: set[int] | None = None) -> bool:
    """Protect the panel plus Codex/OMX/tmux agent processes from cleanup scans."""
    protected = protected or protected_pids()
    if proc.pid in protected:
        return True
    cmd = command_line(proc)
    if any(pattern in cmd for pattern in PANEL_PROTECTED_PATTERNS):
        return True
    lowered = cmd.lower()
    return any(pattern in lowered for pattern in AGENT_PROTECTED_PATTERNS)


def terminate_psutil_tree(proc: Any, *, label: str, grace_sec: float = 3.0) -> list[str]:
    """Terminate a matched stale process and its descendants, with agent guards."""
    events: list[str] = []
    protected = protected_pids()
    if is_protected_process(proc, protected):
        events.append(f"{label}: skip protected pid={proc.pid} cmd={command_line(proc)[:160]}")
        return events
    try:
        targets = proc.children(recursive=True) + [proc]
    except psutil.Error as exc:
        events.append(f"{label}: inspect failed pid={proc.pid}: {exc}")
        return events

    killable = [target for target in targets if not is_protected_process(target, protected)]
    skipped = [target for target in targets if target not in killable]
    for target in skipped:
        events.append(f"{label}: skip protected pid={target.pid} cmd={command_line(target)[:160]}")
    for target in killable:
        try:
            events.append(f"{label}: terminate pid={target.pid} cmd={command_line(target)[:160]}")
            target.terminate()
        except psutil.Error as exc:
            events.append(f"{label}: terminate failed pid={target.pid}: {exc}")

    _, alive = psutil.wait_procs(killable, timeout=grace_sec)
    for target in alive:
        if is_protected_process(target, protected):
            events.append(f"{label}: skip protected alive pid={target.pid} cmd={command_line(target)[:160]}")
            continue
        try:
            events.append(f"{label}: kill pid={target.pid} cmd={command_line(target)[:160]}")
            target.kill()
        except psutil.Error as exc:
            events.append(f"{label}: kill failed pid={target.pid}: {exc}")
    return events


def cleanup_doosan_stack(*, grace_sec: float = 3.0) -> list[str]:
    """Best-effort cleanup of stale Doosan/MoveIt graph processes before reconnect."""
    events: list[str] = []
    old = processes.pop("connect_robot", None)
    if old is not None:
        events.extend(terminate_process_tree(old, label="stored connect_robot", grace_sec=grace_sec))

    _service_ready_cache.clear()

    if psutil is None:
        return events

    protected = protected_pids()
    candidates: list[Any] = []
    for proc in psutil.process_iter(["pid", "cmdline", "name"]):
        if is_protected_process(proc, protected):
            continue
        cmd = command_line(proc)
        if not cmd:
            continue
        if any(pattern in cmd for pattern in DOOSAN_STACK_PATTERNS):
            candidates.append(proc)

    if not candidates:
        events.append("cleanup: no stale Doosan/MoveIt processes found")
        return events

    for proc in candidates:
        events.extend(terminate_psutil_tree(proc, label="cleanup", grace_sec=grace_sec))

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

    protected = protected_pids()
    candidates: list[Any] = []
    seen: set[int] = set()
    for proc in psutil.process_iter(["pid", "cmdline", "name"]):
        if proc.pid in seen or is_protected_process(proc, protected):
            continue
        cmd = command_line(proc)
        if not cmd:
            continue
        if any(pattern in cmd for pattern in patterns):
            candidates.append(proc)
            seen.add(proc.pid)

    if not candidates:
        events.append(f"{label}: no matching stale processes found")
        return events

    for proc in candidates:
        events.extend(terminate_psutil_tree(proc, label=label, grace_sec=grace_sec))

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
    _service_ready_cache.clear()
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
    hand_eye = processes.pop("hand_eye_static_tf", None)
    if hand_eye is not None:
        events.extend(terminate_process_tree(hand_eye, label="stored hand_eye_static_tf", grace_sec=grace_sec))
    events.extend(cleanup_matching_processes(SIDE_GRIP_STACK_PATTERNS, label="side_grip cleanup", grace_sec=grace_sec))
    return events


def cleanup_cup_uprighting_stack(*, grace_sec: float = 2.0) -> list[str]:
    """Best-effort cleanup of stale cup-uprighting nodes without killing TF/scene."""
    events: list[str] = []
    old = processes.pop("cup_uprighting", None)
    if old is not None:
        events.extend(terminate_process_tree(old, label="stored cup_uprighting", grace_sec=grace_sec))
    events.extend(
        cleanup_matching_processes(
            CUP_UPRIGHTING_STACK_PATTERNS,
            label="cup_uprighting cleanup",
            grace_sec=grace_sec,
        )
    )
    return events


def cleanup_collision_scene_stack(*, grace_sec: float = 2.0) -> list[str]:
    """Replace stale PlanningScene publishers before starting a shared scene.

    The operator relies on one consistent safety scene for table, walls,
    dispenser, detected tumbler, and the RG2 envelope attached to link_6.  Stale
    duplicate scene publishers make RViz/MoveIt hard to reason about, so the
    panel restarts this stack as a single unit.
    """
    events: list[str] = []
    old = processes.pop("start_collision_scene", None)
    if old is not None:
        events.extend(terminate_process_tree(old, label="stored start_collision_scene", grace_sec=grace_sec))
    events.extend(
        cleanup_matching_processes(
            COLLISION_SCENE_STACK_PATTERNS,
            label="collision-scene cleanup",
            grace_sec=grace_sec,
        )
    )
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
        "run_doosan_real_m0609.sh",
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
    cmd = f"timeout {max(timeout_sec, 0.1):.1f}s {command}"
    completed = subprocess.run(
        ["bash", "-lc", cmd],
        cwd=str(ROOT),
        env=ros_command_env(),
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


def ros_node_names(timeout_sec: float = 4.0) -> tuple[set[str], str]:
    rc, output = ros2_call("ros2 node list --no-daemon", timeout_sec=timeout_sec)
    if rc != 0:
        return set(), output
    return {line.strip() for line in output.splitlines() if line.strip().startswith("/")}, output


def doosan_virtual_nodes_present(service_prefix: str, timeout_sec: float = 4.0) -> tuple[bool, str]:
    clean = service_prefix.strip("/") or "dsr01"
    nodes, output = ros_node_names(timeout_sec=timeout_sec)
    found = sorted(
        node for node in nodes
        if node in {f"/{clean}/virtual_node", "/virtual_node"} or node.endswith("/virtual_node")
    )
    if found:
        return True, "virtual Doosan node(s) detected: " + ", ".join(found) + "\n" + output
    return False, output


# Per-process service cache: once a service is confirmed ready, skip re-checking
# for SERVICE_CACHE_TTL seconds. Avoids ~2s `ros2 service list` calls per step.
_service_ready_cache: dict[str, float] = {}
SERVICE_CACHE_TTL = 600.0


def _cache_services(confirmed: set[str] | list[str]) -> None:
    now = time.monotonic()
    for svc in confirmed:
        _service_ready_cache[svc] = now


def _all_cached(required: list[str]) -> bool:
    if not required:
        return True
    cutoff = time.monotonic() - SERVICE_CACHE_TTL
    return all(_service_ready_cache.get(svc, 0.0) > cutoff for svc in required)


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


def motion_services_ready(service_prefix: str) -> tuple[bool, str, set[str]]:
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
        return False, "missing motion services: " + ", ".join(missing) + "\n--- services ---\n" + output, set()
    return True, "motion services are present", services


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
        ready, output, services = motion_services_ready(service_prefix)
        last_output = output
        if ready:
            if services:
                _cache_services(services)
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
    if step.key == "lid_view_pose":
        return [
            f"/{clean}/motion/move_joint",
            f"/{clean}/motion/check_motion",
            f"/{clean}/system/get_robot_state",
        ]
    if step.key == "lid_grip_close":
        return [
            "/jarvis/rg2/set_width",
            f"/{clean}/motion/move_line",
            f"/{clean}/motion/move_joint",
            f"/{clean}/motion/move_periodic",
            f"/{clean}/motion/ikin",
            f"/{clean}/motion/check_motion",
            f"/{clean}/system/get_robot_state",
            f"/{clean}/aux_control/get_current_posj",
            f"/{clean}/aux_control/get_current_posx",
        ]
    if step.key in {"home_robot", "lift_robot", "side_grip_camera_home", "move_to_color_scan_pose"}:
        return [
            f"/{clean}/motion/move_joint",
            f"/{clean}/motion/move_wait",
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
    if step.key == "run_color_recipe_sequence":
        return [
            "/jarvis/rg2/set_width",
            f"/{clean}/motion/move_line",
            f"/{clean}/motion/move_joint",
            f"/{clean}/motion/move_wait",
            f"/{clean}/motion/fkin",
            f"/{clean}/motion/ikin",
            f"/{clean}/motion/check_motion",
            f"/{clean}/system/get_robot_state",
            f"/{clean}/tcp/get_current_tcp",
            f"/{clean}/aux_control/get_current_posj",
            f"/{clean}/aux_control/get_current_posx",
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
    if step.key == "handover_cup_to_palm":
        return [
            "/jarvis/rg2/set_width",
            f"/{clean}/motion/move_line",
            f"/{clean}/motion/ikin",
            f"/{clean}/motion/check_motion",
            f"/{clean}/aux_control/get_current_posx",
            f"/{clean}/aux_control/get_tool_force",
            f"/{clean}/system/get_robot_state",
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
        or step.key == "run_color_recipe_sequence"
        or step.key == "place_cup_holder"
        or step.key == "lid_grip_close"
    ):
        return 35.0
    if step.key in {"home_robot", "lift_robot", "side_grip_camera_home", "lid_view_pose", "move_to_color_scan_pose", "side_grip", "shake_closed_cup", "handover_cup_to_palm"}:
        return 30.0
    if step.key == "gripper_soft_grasp":
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
    # Fast path: all required services were recently confirmed → skip ros2 service list.
    # Skip when proc is given: the caller is waiting for a freshly-spawned process to
    # register its services, so a cached entry from the previous run must not mask the
    # fact that the new process has not finished initialising yet.
    if proc is None and _all_cached(required):
        return True, f"required services confirmed via cache (TTL {SERVICE_CACHE_TTL:.0f}s): {', '.join(required)}"

    deadline = time.monotonic() + max(timeout_sec, 0.1)
    last_output = ""
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        services, output = ros_service_names(timeout_sec=2.0)
        missing = [service for service in required if service not in services]
        last_output = output
        if not missing:
            _cache_services(services)
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
    """Wait until workspace collision objects are visible."""
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    last_collision_output = ""
    saw_collision = False
    while time.monotonic() < deadline:
        if proc is not None and proc.poll() is not None:
            return False, "collision scene process exited while waiting\n" + tail_file(process_logs.get("start_collision_scene"))
        collision_result = subprocess.run(
            ["bash", "-lc", "timeout 2s ros2 topic echo /collision_object --once"],
            cwd=str(ROOT),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=3.0,
            check=False,
        )
        last_collision_output = collision_result.stdout[-2000:]
        if collision_result.returncode == 0 and "id:" in collision_result.stdout:
            saw_collision = True

        if saw_collision:
            return (
                True,
                "collision object sample observed on /collision_object\n"
                + last_collision_output,
            )
        time.sleep(0.5)
    return False, (
        f"scene readiness incomplete within {timeout_sec:.1f}s "
        f"(workspace={saw_collision})\n"
        f"--- last /collision_object ---\n{last_collision_output}"
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


def realsense_usb_visible() -> tuple[bool, str]:
    """Return whether an Intel RealSense device is visible to the OS."""
    try:
        result = subprocess.run(
            ["lsusb"],
            cwd=str(ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=3.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, f"[FAIL] lsusb check failed: {exc}"
    output = result.stdout.strip()
    if result.returncode != 0:
        return False, "[FAIL] lsusb returned non-zero\n" + output
    visible = any(
        ("Intel" in line and "RealSense" in line)
        or "8086:0b" in line.lower()
        for line in output.splitlines()
    )
    if visible:
        return True, "[OK] RealSense USB device visible\n" + output
    return (
        False,
        "[FAIL] RealSense USB device is not visible to lsusb. "
        "카메라 ROS 재시작으로는 복구되지 않습니다.\n" + output,
    )


def wait_for_tf_transform(
    *,
    env: dict[str, str],
    target_frame: str,
    source_frame: str,
    timeout_sec: float = 10.0,
    proc: subprocess.Popen[str] | None = None,
) -> tuple[bool, str]:
    """Wait until tf2 can transform source_frame into target_frame."""
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    last_output = ""
    attempt = 0
    while time.monotonic() < deadline:
        if proc is not None and proc.poll() is not None:
            return False, "TF provider process exited while waiting\n" + tail_file(process_logs.get("hand_eye_static_tf"))
        attempt += 1
        result = subprocess.run(
            [
                "bash",
                "-lc",
                "timeout 2s ros2 run tf2_ros tf2_echo "
                f"{shlex.quote(target_frame)} {shlex.quote(source_frame)}",
            ],
            cwd=str(ROOT),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=3.0,
            check=False,
        )
        last_output = result.stdout[-2000:]
        if "Translation:" in result.stdout and "Rotation:" in result.stdout:
            return (
                True,
                f"TF ready after {attempt} check(s): {target_frame} <- {source_frame}\n"
                + last_output,
            )
        time.sleep(0.5)
    return (
        False,
        f"TF not ready within {timeout_sec:.1f}s: {target_frame} <- {source_frame}\n"
        f"--- last tf2_echo output ---\n{last_output}",
    )


def ensure_hand_eye_tf(env: dict[str, str], *, timeout_sec: float = 20.0) -> tuple[bool, str]:
    """Ensure the measured hand-eye publisher connects the RealSense tree to base_link."""
    ready, output = wait_for_tf_transform(
        env=env,
        target_frame=HAND_EYE_TF_TARGET_FRAME,
        source_frame=HAND_EYE_TF_SOURCE_FRAME,
        timeout_sec=8.0,
    )
    if ready:
        return True, output

    events = [
        "hand-eye TF not currently available; starting measured hand_eye_static_tf_node",
        output,
    ]
    proc = processes.get("hand_eye_static_tf")
    if proc is None or proc.poll() is not None:
        cmd = f"cd {ROOT} && {ROS_SETUP} && {hand_eye_static_tf_command(compose_timeout_sec=30.0)}"
        log_path = background_log_path("hand_eye_static_tf")
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
        processes["hand_eye_static_tf"] = proc
        process_logs["hand_eye_static_tf"] = log_path
        events.append(f"auto-started hand_eye_static_tf pid={proc.pid} log={log_path}")
    else:
        events.append(f"hand_eye_static_tf already running pid={proc.pid}")

    ready, wait_output = wait_for_tf_transform(
        env=env,
        target_frame=HAND_EYE_TF_TARGET_FRAME,
        source_frame=HAND_EYE_TF_SOURCE_FRAME,
        timeout_sec=timeout_sec,
        proc=proc,
    )
    events.append(wait_output)
    if not ready:
        events.append("--- hand_eye_static_tf log tail ---")
        events.append(tail_file(process_logs.get("hand_eye_static_tf")))
    return ready, "\n".join(events)


def ensure_world_base_tf(env: dict[str, str], *, timeout_sec: float = 5.0) -> tuple[bool, str]:
    """Ensure the MoveIt planning frame can reach the robot base frame."""
    ready, output = wait_for_tf_transform(
        env=env,
        target_frame="world",
        source_frame="base_link",
        timeout_sec=5.0,
    )
    if ready:
        return True, output

    events = [
        "world -> base_link TF not currently available; starting identity static TF",
        output,
    ]
    proc = processes.get("world_base_static_tf")
    if proc is None or proc.poll() is not None:
        cmd = (
            f"cd {ROOT} && {ROS_SETUP} && "
            "ros2 run tf2_ros static_transform_publisher "
            "--x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 "
            "--frame-id world --child-frame-id base_link"
        )
        log_path = background_log_path("world_base_static_tf")
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
        processes["world_base_static_tf"] = proc
        process_logs["world_base_static_tf"] = log_path
        events.append(f"auto-started world_base_static_tf pid={proc.pid} log={log_path}")
    else:
        events.append(f"world_base_static_tf already running pid={proc.pid}")

    ready, wait_output = wait_for_tf_transform(
        env=env,
        target_frame="world",
        source_frame="base_link",
        timeout_sec=timeout_sec,
        proc=proc,
    )
    events.append(wait_output)
    if not ready:
        events.append("--- world_base_static_tf log tail ---")
        events.append(tail_file(process_logs.get("world_base_static_tf")))
    return ready, "\n".join(events)


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
    proc = None
    try:
        proc = subprocess.Popen(
            ["bash", "-lc", f"{ROS_SETUP} && python3 -c {shlex.quote(script)}"],
            cwd=str(ROOT),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
        stdout, stderr = proc.communicate(timeout=6.0)
    except subprocess.TimeoutExpired:
        if proc is not None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=1.0)
            except Exception:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except Exception:
                    pass
        return False, b"", "camera snapshot timed out"
    except Exception as exc:  # pragma: no cover - operator diagnostics only.
        return False, b"", f"camera snapshot failed: {exc}"
    if proc.returncode != 0 or not stdout:
        return False, b"", stderr.decode("utf-8", errors="replace")[-2000:]
    return True, stdout, ""


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

    usb_ok, usb_output = realsense_usb_visible()
    checks.append("--- RealSense USB ---\n" + usb_output)
    if not usb_ok:
        ok = False
        return ok, "\n".join(checks)

    camera_ready, camera_output = wait_for_camera_topic_samples(env=env, timeout_sec=5.0)
    if not camera_ready:
        # 카메라가 depth 없이 켜져 있을 수 있음 → 자동 재시작
        checks.append("[AUTO] 카메라 토픽 불완전 — depth 포함 자동 재시작 중...")
        cleanup_camera_stack()
        time.sleep(1.5)
        camera_cmd = (
            f"cd {ROOT} && {ROS_SETUP} && "
            "ros2 launch realsense2_camera rs_launch.py "
            "camera_name:=camera "
            "initial_reset:=true reconnect_timeout:=5.0 "
            "enable_color:=true enable_depth:=true align_depth.enable:=true "
            "rgb_camera.color_profile:=640x480x30 "
            "depth_module.depth_profile:=640x480x30"
        )
        log_path = background_log_path("start_camera")
        log_handle = log_path.open("w", encoding="utf-8", buffering=1)
        camera_proc = subprocess.Popen(
            ["bash", "-lc", camera_cmd],
            cwd=str(ROOT),
            env=env,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        log_handle.close()
        processes["start_camera"] = camera_proc
        process_logs["start_camera"] = log_path
        camera_ready, camera_output = wait_for_camera_topic_samples(env=env, timeout_sec=20.0, proc=camera_proc)
        checks.append("[AUTO] 카메라 재시작 후 토픽 확인:\n" + camera_output)
    else:
        checks.append("--- camera topics ---\n" + camera_output)
    if not camera_ready:
        ok = False

    clean = service_prefix.strip("/") or "dsr01"
    action_name = f"/{clean}/dsr_moveit_controller/follow_joint_trajectory"
    action_ready, action_output = wait_for_action_server(action_name, timeout_sec=5.0)
    checks.append("--- MoveIt action ---\n" + action_output)
    if not action_ready:
        checks.append(
            "[WARN] MoveIt action introspection timed out. Continuing because "
            "field runs can execute through the side-grip node even when ros2 action info is slow."
        )

    world_base_ready, world_base_output = ensure_world_base_tf(env, timeout_sec=5.0)
    checks.append("--- world/base TF ---\n" + world_base_output)
    if not world_base_ready:
        ok = False

    tf_ready, tf_output = ensure_hand_eye_tf(env, timeout_sec=20.0)
    checks.append("--- hand-eye TF ---\n" + tf_output)
    if not tf_ready:
        ok = False

    return ok, "\n".join(checks)


def cup_uprighting_preflight(env: dict[str, str], service_prefix: str) -> tuple[bool, str]:
    """Fail closed before the cup-uprighting MoveItPy node can command motion."""
    checks: list[str] = []
    ok = True

    if CUP_UPRIGHTING_YOLO_MODEL_PATH.exists():
        checks.append(f"[OK] cup_uprighting YOLO model: {CUP_UPRIGHTING_YOLO_MODEL_PATH}")
    else:
        ok = False
        checks.append(f"[FAIL] cup_uprighting YOLO model missing: {CUP_UPRIGHTING_YOLO_MODEL_PATH}")

    usb_ok, usb_output = realsense_usb_visible()
    checks.append("--- RealSense USB ---\n" + usb_output)
    if not usb_ok:
        ok = False
        return ok, "\n".join(checks)

    camera_ready, camera_output = wait_for_camera_topic_samples(env=env, timeout_sec=5.0)
    checks.append("--- camera topics ---\n" + camera_output)
    if not camera_ready:
        ok = False

    clean = service_prefix.strip("/") or "dsr01"
    action_name = f"/{clean}/dsr_moveit_controller/follow_joint_trajectory"
    action_ready, action_output = wait_for_action_server(action_name, timeout_sec=5.0)
    checks.append("--- MoveIt action ---\n" + action_output)
    if not action_ready:
        ok = False

    world_base_ready, world_base_output = ensure_world_base_tf(env, timeout_sec=5.0)
    checks.append("--- world/base TF ---\n" + world_base_output)
    if not world_base_ready:
        ok = False

    tf_ready, tf_output = ensure_hand_eye_tf(env, timeout_sec=20.0)
    checks.append("--- hand-eye TF ---\n" + tf_output)
    if not tf_ready:
        ok = False

    world_tf_ready, world_tf_output = wait_for_tf_transform(
        env=env,
        target_frame="world",
        source_frame=HAND_EYE_TF_SOURCE_FRAME,
        timeout_sec=5.0,
    )
    checks.append("--- MoveIt planning-frame TF ---\n" + world_tf_output)
    if not world_tf_ready:
        ok = False

    return ok, "\n".join(checks)


def lid_grip_preflight(env: dict[str, str], service_prefix: str) -> tuple[bool, str]:
    checks: list[str] = []
    ok = True

    for package, executable in (
        ("azas_perception", "lid_sticker_detector_node"),
        ("azas_perception", "cup_detection_pose_bridge_node"),
        ("azas_perception", "hand_eye_static_tf_node"),
        ("azas_motion", "lid_grip_planner_node"),
    ):
        rc, output = ros2_call(
            f"ros2 pkg executables {shlex.quote(package)}",
            timeout_sec=3.0,
        )
        line = f"{package} {executable}"
        if rc == 0 and line in output:
            checks.append(f"[OK] executable: {line}")
        else:
            ok = False
            checks.append(f"[FAIL] executable missing: {line}\n{output}")

    clean = service_prefix.strip("/") or "dsr01"
    required = [
        "/jarvis/rg2/set_width",
        f"/{clean}/motion/move_line",
        f"/{clean}/motion/move_joint",
        f"/{clean}/motion/move_periodic",
        f"/{clean}/motion/ikin",
        f"/{clean}/motion/check_motion",
        f"/{clean}/system/get_robot_state",
        f"/{clean}/aux_control/get_current_posj",
        f"/{clean}/aux_control/get_current_posx",
    ]
    services_ok, services_output = wait_for_required_services(required, timeout_sec=12.0)
    checks.append("--- lid required services ---\n" + services_output)
    if not services_ok:
        ok = False

    world_base_ready, world_base_output = ensure_world_base_tf(env, timeout_sec=5.0)
    checks.append("--- world/base TF ---\n" + world_base_output)
    if not world_base_ready:
        ok = False

    tf_ready, tf_output = ensure_hand_eye_tf(env, timeout_sec=20.0)
    checks.append("--- hand-eye TF ---\n" + tf_output)
    if not tf_ready:
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
        step.key
        in {
            "home_robot",
            "lift_robot",
            "side_grip_camera_home",
            "lid_view_pose",
            "side_grip",
            "lid_grip_close",
            "shake_closed_cup",
        }
        or step.key.startswith("move_to_dispenser_")
        or step.key.startswith("press_dispenser_")
        or step.key.startswith("pick_from_dispenser_")
        or step.key == "run_color_recipe_sequence"
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


def real_motion_readiness_gate(
    service_prefix: str,
    *,
    motion_timeout_sec: float = 35.0,
) -> tuple[bool, str]:
    clean = service_prefix.strip("/") or "dsr01"
    motion_ready, motion_output = wait_for_motion_services_ready(
        clean,
        timeout_sec=motion_timeout_sec,
    )
    if not motion_ready:
        return False, "--- motion services ---\n" + motion_output
    robot_ready, robot_output = doosan_robot_ready(clean)
    output = "--- motion services ---\n" + motion_output + "\n" + robot_output
    if not robot_ready:
        return False, output
    action_name = f"/{clean}/dsr_moveit_controller/follow_joint_trajectory"
    action_ready, action_output = wait_for_action_server(action_name, timeout_sec=8.0)
    output += "\n--- MoveIt action ---\n" + action_output
    if not action_ready:
        return False, output
    return True, output


def manual_logic_preflight(step: Step, env: dict[str, str], service_prefix: str) -> tuple[bool, str]:
    checks: list[str] = []
    motion_ok, motion_output = real_motion_readiness_gate(
        service_prefix,
        motion_timeout_sec=25.0,
    )
    checks.append(motion_output)
    if not motion_ok:
        return False, "\n".join(checks)

    required_gripper = ["/jarvis/rg2/open", "/jarvis/rg2/close", "/jarvis/rg2/set_width"]
    gripper_ok, gripper_output = wait_for_required_services(
        required_gripper,
        timeout_sec=8.0,
    )
    checks.append("--- gripper services ---\n" + gripper_output)
    if not gripper_ok:
        return False, "\n".join(checks)

    camera_ok, camera_output = wait_for_camera_topic_samples(env=env, timeout_sec=10.0)
    checks.append("--- camera topics ---\n" + camera_output)
    if not camera_ok:
        return False, "\n".join(checks)

    if step.key == "side_grip":
        side_ok, side_output = side_grip_preflight(env, service_prefix)
        checks.append("--- side_grip preflight ---\n" + side_output)
        if not side_ok:
            return False, "\n".join(checks)
    elif step.key == "cup_uprighting":
        cup_ok, cup_output = cup_uprighting_preflight(env, service_prefix)
        checks.append("--- cup_uprighting preflight ---\n" + cup_output)
        if not cup_ok:
            return False, "\n".join(checks)
    elif step.key == "lid_grip_close":
        lid_ok, lid_output = lid_grip_preflight(env, service_prefix)
        checks.append("--- lid_grip_close preflight ---\n" + lid_output)
        if not lid_ok:
            return False, "\n".join(checks)

    return True, "\n".join(checks)


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
        key in {
            # Direct joint/line motions still need the shared PlanningScene
            # visible and current in RViz/operator review.  Some of these
            # commands do not consume MoveIt collisions directly, but every
            # real robot task should run in the same table/wall/dispenser scene.
            "home_robot",
            "lift_robot",
            "side_grip_camera_home",
            "lid_view_pose",
            "move_to_color_scan_pose",
            "place_cup_holder",
            "shake_closed_cup",
            "run_color_recipe_sequence",
        }
        or key == "run_color_recipe_sequence"
        or key.startswith("move_to_dispenser_")
        or key.startswith("press_dispenser_")
        or key.startswith("pick_from_dispenser_")
    )


def with_collision_scene_prereq(selected: list[str]) -> list[str]:
    ordered: list[str] = []

    def append_once(key: str) -> None:
        if key not in ordered:
            ordered.append(key)

    for key in selected:
        if key == "color_scan":
            for prereq in (
                "connect_robot",
                "status_check",
                "start_collision_scene",
                "move_to_color_scan_pose",
                "start_camera",
            ):
                append_once(prereq)
        elif requires_collision_scene_step(key):
            for prereq in ("connect_robot", "status_check", "start_collision_scene"):
                append_once(prereq)
        append_once(key)

    return ordered


def configure_manual_recipe_chain(selected: list[str], payload: dict[str, Any]) -> list[str]:
    """Run dispenser recipe inside manual OpenCV tmux step after success.

    The manual PR #20 / cup-uprighting steps are long-running GUI commands.
    If the panel loop keeps `run_color_recipe_sequence` as a separate next
    step, it starts immediately after the tmux window is opened, before the
    operator presses `p`.  Instead, mark the manual command for shell-level
    chaining and remove the separate recipe step from the server loop.
    """
    manual_keys = {"side_grip", "cup_uprighting"}
    if not any(key in selected for key in manual_keys):
        return selected
    payload["_auto_recipe_after_manual_logic"] = True
    return [key for key in selected if key != "run_color_recipe_sequence"]


def run_timeout_for_step(step: Step) -> float:
    if step.key == "side_grip":
        return 300.0
    if step.key == "cup_uprighting":
        return 900.0
    if step.key == "side_grip_camera_home":
        return 180.0
    if step.key == "lid_grip_close":
        return 900.0
    if step.key == "run_color_recipe_sequence":
        return 1200.0
    if step.key == "run_one_click_cocktail_real":
        return 1500.0
    if step.key == "rviz_cocktail_collision_preview":
        return 1500.0
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
    env["FASTDDS_BUILTIN_TRANSPORTS"] = str(env.get("FASTDDS_BUILTIN_TRANSPORTS") or "UDPv4")
    env["ROBOT_HOST"] = str(payload.get("robot_host") or env.get("ROBOT_HOST") or DEFAULT_ROBOT_HOST)
    env["ROBOT_NAME"] = str(payload.get("robot_name") or env.get("ROBOT_NAME") or "dsr01")
    env["SERVICE_PREFIX"] = str(payload.get("service_prefix") or env.get("SERVICE_PREFIX") or "dsr01")
    env["RG2_IP"] = str(payload.get("rg2_ip") or env.get("RG2_IP") or "192.168.1.1")
    env["SELECTED_DISPENSER_ID"] = str(
        payload.get("selected_dispenser_id") or env.get("SELECTED_DISPENSER_ID") or "2"
    )
    env["RECIPE_DISPENSER_IDS"] = str(
        payload.get("recipe_dispenser_ids") or env.get("RECIPE_DISPENSER_IDS") or ""
    )
    env["CUP_HOLDER_PLACE_FINAL_Z_OFFSET_M"] = str(
        payload.get("cup_holder_place_final_z_offset_m")
        or env.get("CUP_HOLDER_PLACE_FINAL_Z_OFFSET_M")
        or "-0.030"
    )
    env["CUP_HOLDER_PLACE_FINAL_X_OFFSET_M"] = str(
        payload.get("cup_holder_place_final_x_offset_m")
        or env.get("CUP_HOLDER_PLACE_FINAL_X_OFFSET_M")
        or "0.015"
    )
    env["CUP_HOLDER_PLACE_FINAL_Y_OFFSET_M"] = str(
        payload.get("cup_holder_place_final_y_offset_m")
        or env.get("CUP_HOLDER_PLACE_FINAL_Y_OFFSET_M")
        or "-0.010"
    )
    env["CUP_HOLDER_RZ_OFFSET_DEG"] = str(
        payload.get("cup_holder_rz_offset_deg")
        or env.get("CUP_HOLDER_RZ_OFFSET_DEG")
        or "-1.0"
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
        or DEFAULT_RT_HOST
    )
    env["DOOSAN_REAL_MOTION_CONFIRM"] = "ENABLE_DOOSAN_REAL_MOTION_BRINGUP"
    # Panel-run Python scripts should flush logs while the browser polls
    # /api/running_logs; otherwise operators only see output after completion.
    env["PYTHONUNBUFFERED"] = "1"
    return env


def command_for(step: Step, payload: dict[str, Any]) -> str:
    command_override = load_command_overrides().get(step.key)
    if command_override:
        return command_override

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
        return tmux_stack_start_command(payload)
    if step.key == "start_tmux_stack":
        return tmux_stack_start_command(payload)
    if step.key == "status_check":
        clean = service_prefix.strip("/") or "dsr01"
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "echo '--- nodes ---' && "
            "(timeout 0.5s ros2 node list || echo '[WARN] ros2 node list timed out; continuing with direct service checks') && "
            "echo '--- required motion service types ---' && "
            f"(timeout 0.5s ros2 service type /{clean}/motion/move_line || echo '[WARN] move_line service type lookup timed out') && "
            f"(timeout 0.5s ros2 service type /{clean}/motion/move_joint || echo '[WARN] move_joint service type lookup timed out') && "
            "echo '--- robot state ---' && "
            f"timeout 4s python3 {shlex.quote(str(ROOT / 'tools' / 'run' / 'ros_call_empty_service.py'))} "
            f"/{clean}/system/get_robot_state dsr_msgs2/srv/GetRobotState --timeout 3.0 && "
            "echo '--- check motion ---' && "
            f"timeout 4s python3 {shlex.quote(str(ROOT / 'tools' / 'run' / 'ros_call_empty_service.py'))} "
            f"/{clean}/motion/check_motion dsr_msgs2/srv/CheckMotion --timeout 3.0 && "
            "echo '--- trajectory action ---' && "
            f"(timeout 0.5s ros2 action info /{clean}/dsr_moveit_controller/follow_joint_trajectory || "
            "echo '[WARN] trajectory action info timed out') && "
            "echo '--- rviz joint_states relay sample ---' && "
            "(timeout 0.5s ros2 topic echo /joint_states --once || "
            "echo '[WARN] no /joint_states sample; RViz robot model may stay frozen even while /dsr01/joint_states moves') && "
            "echo '--- lid/ArUco package executables ---' && "
            "(timeout 1s ros2 pkg executables azas_perception | grep -E 'lid_sticker_detector_node|hand_eye_static_tf_node' || "
            "echo '[WARN] azas_perception lid/hand-eye executables not visible') && "
            "(timeout 1s ros2 pkg executables azas_motion | grep -E 'lid_grip_planner_node' || "
            "echo '[WARN] azas_motion lid_grip_planner_node executable not visible') && "
            "echo '--- vision TF note ---' && "
            "echo '[INFO] camera/hand-eye TF is checked after RealSense + MoveIt collision scene startup, not during core robot status_check.'"
        )
    if step.key == "run_color_recipe_sequence":
        return color_recipe_sequence_command(payload)
    if step.key == "rviz_cocktail_collision_preview":
        recipe_dispenser_ids = str(payload.get("recipe_dispenser_ids") or "").strip()
        recipe_env = ""
        if recipe_dispenser_ids:
            recipe_env = f"RECIPE_DISPENSER_IDS={shlex.quote(recipe_dispenser_ids)} "
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            f"{recipe_env}DISPENSER_COLLISION_OBJECTS=1 "
            "tools/run/run_cocktail_collision_rviz_preview.sh"
        )
    if step.key == "stop_cocktail_motion_preview":
        return f"cd {ROOT} && tools/run/stop_cocktail_motion_preview.sh"
    if step.key == "stop_azas_all":
        # No ROS_SETUP: the cleanup script must not depend on (or re-spawn) the
        # ROS daemon it is about to kill.
        return f"cd {ROOT} && bash tools/run/stop_azas_all.sh"
    if step.key == "check_one_click_cocktail_ready":
        robot_host = str(payload.get("robot_host") or os.environ.get("ROBOT_HOST") or DEFAULT_ROBOT_HOST)
        robot_name = str(payload.get("robot_name") or os.environ.get("ROBOT_NAME") or service_prefix)
        recipe_dispenser_ids = str(payload.get("recipe_dispenser_ids") or "").strip()
        recipe_env = ""
        if recipe_dispenser_ids:
            recipe_env = f"RECIPE_DISPENSER_IDS={shlex.quote(recipe_dispenser_ids)} "
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            f"{recipe_env}"
            f"ROBOT_HOST={shlex.quote(robot_host)} ROBOT_NAME={shlex.quote(robot_name)} SERVICE_PREFIX={shlex.quote(service_prefix)} "
            "tools/run/check_one_click_cocktail_ready.sh"
        )
    if step.key == "check_one_click_cocktail_result":
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            f"SERVICE_PREFIX={shlex.quote(service_prefix)} "
            "tools/run/check_one_click_cocktail_result.sh"
        )
    if step.key == "run_one_click_cocktail_real":
        recipe_dispenser_ids = str(payload.get("recipe_dispenser_ids") or "").strip()
        recipe_env = ""
        if recipe_dispenser_ids:
            recipe_env = f"RECIPE_DISPENSER_IDS={shlex.quote(recipe_dispenser_ids)} "
        robot_host = str(payload.get("robot_host") or os.environ.get("ROBOT_HOST") or DEFAULT_ROBOT_HOST)
        robot_name = str(payload.get("robot_name") or os.environ.get("ROBOT_NAME") or service_prefix)
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE "
            f"{recipe_env}"
            f"ROBOT_HOST={shlex.quote(robot_host)} ROBOT_NAME={shlex.quote(robot_name)} SERVICE_PREFIX={shlex.quote(service_prefix)} "
            "tools/run/run_one_click_cocktail_real.sh"
        )
    if step.key == "run_cocktail_now_real":
        recipe_dispenser_ids = str(payload.get("recipe_dispenser_ids") or "").strip()
        recipe_arg = ""
        if recipe_dispenser_ids:
            recipe_arg = f" {shlex.quote(recipe_dispenser_ids)}"
        robot_host = str(payload.get("robot_host") or os.environ.get("ROBOT_HOST") or DEFAULT_ROBOT_HOST)
        robot_name = str(payload.get("robot_name") or os.environ.get("ROBOT_NAME") or service_prefix)
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE "
            f"ROBOT_HOST={shlex.quote(robot_host)} ROBOT_NAME={shlex.quote(robot_name)} SERVICE_PREFIX={shlex.quote(service_prefix)} "
            f"tools/run/run_cocktail_now_real.sh{recipe_arg}"
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
            "--j5-min-deg -135 --j5-max-deg 135 --timeout-sec 60 --motion-timeout-sec 120 "
            "--execute --confirm ENABLE_DIRECT_MOVEJ"
        )
    if step.key == "side_grip_camera_home":
        return (
            f"cd {ROOT} && {ROS_SETUP} && python3 tools/run/direct_movej_joints.py "
            f"--service-prefix {service_prefix} "
            "--j1 3.0 --j2 -12.7 --j3 44.0 --j4 -9.0 --j5 133.0 --j6 90.0 "
            "--velocity 20 --acceleration 20 "
            "--j5-min-deg -150 --j5-max-deg 150 --timeout-sec 60 --motion-timeout-sec 120 "
            "--execute --confirm ENABLE_DIRECT_MOVEJ"
        )
    if step.key == "lid_view_pose":
        return (
            f"cd {ROOT} && {ROS_SETUP} && python3 tools/run/direct_movej_joints.py "
            f"--service-prefix {service_prefix} "
            "--j1 3.0 --j2 -12.7 --j3 44.0 --j4 -9.0 --j5 133.0 --j6 90.0 "
            "--velocity 15 --acceleration 15 "
            "--j5-min-deg -150 --j5-max-deg 150 --timeout-sec 60 --motion-timeout-sec 120 "
            "--execute --confirm ENABLE_DIRECT_MOVEJ"
        )
    if step.key == "home_robot":
        return (
            f"cd {ROOT} && {ROS_SETUP} && python3 tools/run/direct_movej_joints.py "
            f"--service-prefix {service_prefix} --j1 0 --j2 0 --j3 90 "
            f"--j4 0 --j5 90 --j6 0 --velocity {FAST_MOVE_VELOCITY} --acceleration {FAST_MOVE_ACCELERATION} "
            "--motion-timeout-sec 120 --execute --confirm ENABLE_DIRECT_MOVEJ"
        )
    if step.key == "move_to_color_scan_pose":
        joints = measured_color_scan_joints()
        return (
            f"cd {ROOT} && {ROS_SETUP} && python3 tools/run/direct_movej_joints.py "
            f"--service-prefix {service_prefix} "
            f"--j1 {shlex.quote(joints['j1'])} --j2 {shlex.quote(joints['j2'])} "
            f"--j3 {shlex.quote(joints['j3'])} --j4 {shlex.quote(joints['j4'])} "
            f"--j5 {shlex.quote(joints['j5'])} --j6 {shlex.quote(joints['j6'])} "
            "--velocity 30 --acceleration 30 --timeout-sec 60 --motion-timeout-sec 120 "
            "--execute --confirm ENABLE_DIRECT_MOVEJ"
        )
    if step.key == "connect_gripper":
        rg2_ip = str(payload.get("rg2_ip") or os.environ.get("RG2_IP") or "192.168.1.1")
        gripper_pkg_bash = ROOT / "install" / "azas_gripper" / "share" / "azas_gripper" / "package.bash"
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            f"source {shlex.quote(str(gripper_pkg_bash))} && "
            f"ros2 launch {shlex.quote(str(ROOT / 'install' / 'azas_gripper' / 'share' / 'azas_gripper' / 'launch' / 'rg2_trigger.launch.py'))} "
            f"ip:={shlex.quote(rg2_ip)} "
            "port:=502 connect:=true open_width:=1100 close_width:=0 force:=300 settle_seconds:=0.6"
        )
    if step.key == "start_collision_scene":
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "("
            "ros2 launch azas_bringup workspace_collision_scene.launch.py "
            "publish_collision_objects:=true "
            "table_collision_enabled:=true "
            "workspace_boundary_collision_enabled:=true "
            "table_collision_expand_to_workspace_walls:=true "
            "dispenser_collision_enabled:=true "
            "dispenser_collision_publish_objects:=true "
            "dispenser_collision_publish_markers:=true & "
            "ros2 launch azas_bringup rg2_link6_tcp.launch.py "
            "publish_gripper_collision:=false & "
            "timeout 12s ros2 run azas_motion link6_gripper_collision_node "
            "--ros-args -p operation:=remove -p publish_once:=true -p publish_markers:=false || true; "
            "ros2 run tf2_ros static_transform_publisher "
            "--x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 "
            "--frame-id world --child-frame-id base_link & "
            f"{hand_eye_static_tf_command(compose_timeout_sec=30.0)} & "
            f"python3 {shlex.quote(str(ROOT / 'src' / 'azas_bringup' / 'azas_bringup' / 'collision_scene_rviz_publisher.py'))} "
            "--ros-args "
            f"-p safety_config_path:={shlex.quote(str(ROOT / 'src' / 'azas_bringup' / 'config' / 'safety.yaml'))} "
            f"-p dispenser_collision_config_path:={shlex.quote(str(ROOT / 'src' / 'azas_bringup' / 'config' / 'measured_dispenser_collision.yaml'))} "
            f"-p calibration_path:={shlex.quote(str(ROOT / 'src' / 'azas_bringup' / 'config' / 'calibration.yaml'))} "
            "-p publish_workspace_ceiling:=false & "
            "python3 -m azas_motion.tumbler_collision_scene_node --ros-args "
            "-p action:=publish_detected "
            "-p object_id:=detected_tumbler "
            "-p use_lidded_height:=true"
            ")"
        )
    if step.key == "start_camera":
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "ros2 launch realsense2_camera rs_launch.py "
            "camera_name:=camera "
            "initial_reset:=true reconnect_timeout:=5.0 "
            "enable_color:=true enable_depth:=true align_depth.enable:=true "
            "rgb_camera.color_profile:=640x480x30 "
            "depth_module.depth_profile:=640x480x30"
        )
    if step.key == "start_camera_view":
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "DISPLAY=${DISPLAY:-:0} "
            "XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority} "
            "ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw"
        )
    if step.key == "start_hand_detection_view":
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "DISPLAY=${DISPLAY:-:0} "
            "XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority} "
            "ros2 run rqt_image_view rqt_image_view /azas/human_hand_detection/overlay"
        )
    if step.key == "detect_cup_lid":
        return f"cd {ROOT} && {ROS_SETUP} && ros2 launch azas_bringup yolo_perception.launch.py"
    if step.key == "pick_lid":
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "ros2 launch azas_bringup lid_sticker_grip_planning.launch.py"
        )
    if step.key == "lid_grip_close":
        direct_script = ROOT / "tools" / "run" / "run_kang_lid_grip_close_direct.sh"
        manual_cmd = (
            f"cd {ROOT} && "
            f"SERVICE_PREFIX={shlex.quote(service_prefix)} "
            "DISPLAY=${DISPLAY:-:0} "
            "XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority} "
            "LID_ROS_LOCALHOST_ONLY=${LID_ROS_LOCALHOST_ONLY:-1} "
            "LID_TCP_GRASP_OFFSET_Z_M=${LID_TCP_GRASP_OFFSET_Z_M:--0.032} "
            "MOVE_TO_LID_VIEW_POSE=true "
            f"bash {shlex.quote(str(direct_script))}"
        )
        if payload.get("_auto_shake_after_lid_grip_close", True):
            return chain_shake_after_lid_command(manual_cmd, payload)
        return manual_cmd
    if step.key == "cup_uprighting":
        direct_script = ROOT / "tools" / "run" / "run_somyeong_cup_uprighting_direct.sh"
        manual_cmd = (
            f"cd {ROOT} && "
            f"SERVICE_PREFIX={shlex.quote(service_prefix)} "
            "DISPLAY=${DISPLAY:-:0} "
            "XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority} "
            f"MODEL_PATH={shlex.quote(str(CUP_UPRIGHTING_YOLO_MODEL_PATH))} "
            "EXIT_AFTER_PICK=true "
            f"bash {shlex.quote(str(direct_script))}"
        )
        if payload.get("_auto_recipe_after_manual_logic"):
            return chain_recipe_after_manual_command(manual_cmd, payload, "소명 cup_uprighting")
        return manual_cmd
    if step.key == "voice_input":
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            "echo '[Azas] 수빈 STT/주문 UI: voice screen http://localhost:8090' && "
            "echo '[Azas] 메뉴를 말하거나 테스트 발화 입력 후, 응/시작으로 확정하면 listen_stt_recipe가 latest_recipe.json을 저장합니다.' && "
            "ros2 launch azas_voice azas_voice.launch.py run_voice_screen:=true"
        )
    if step.key == "side_grip":
        direct_script = ROOT / "tools" / "run" / "run_changhyun_side_grip_direct.sh"
        side_target_x_offset_m = str(
            payload.get("side_target_x_offset_m")
            or os.environ.get("SIDE_TARGET_X_OFFSET_M")
            or "-0.020"
        )
        side_target_joint6_inset_m = str(
            payload.get("side_target_joint6_inset_m")
            or os.environ.get("SIDE_TARGET_JOINT6_INSET_M")
            or "0.070"
        )
        side_target_joint6_inset_sign = str(
            payload.get("side_target_joint6_inset_sign")
            or os.environ.get("SIDE_TARGET_JOINT6_INSET_SIGN")
            or "1.0"
        )
        manual_cmd = (
            f"cd {ROOT} && "
            f"SERVICE_PREFIX={shlex.quote(service_prefix)} "
            f"SIDE_TARGET_X_OFFSET_M={shlex.quote(side_target_x_offset_m)} "
            f"SIDE_TARGET_JOINT6_INSET_M={shlex.quote(side_target_joint6_inset_m)} "
            f"SIDE_TARGET_JOINT6_INSET_SIGN={shlex.quote(side_target_joint6_inset_sign)} "
            "DISPLAY=${DISPLAY:-:0} "
            "XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority} "
            f"bash {shlex.quote(str(direct_script))}"
        )
        if payload.get("_auto_recipe_after_manual_logic", True):
            return chain_recipe_after_manual_command(manual_cmd, payload, "창현 side_grip")
        return manual_cmd
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
            "--target-tolerance-mm 15 --no-set-current-tcp-before-move --compensate-current-tcp "
            "--direct-x-max 0.95 "
            "--verify-link6-target --no-moveit-planning-guard "
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
        try:
            press_xyz_m, press_rpy_deg = measured_dispenser_press_pose(dispenser_id)
        except Exception as exc:
            return fail_closed_shell(
                f"measured press pose for dispenser_{dispenser_id} is unavailable: {exc}"
            )
        tcp_name = str(
            payload.get("dispenser_tcp_name")
            or os.environ.get("DISPENSER_TCP_NAME")
            or DEFAULT_DISPENSER_TCP_NAME
        ).strip()
        return (
            f"cd {ROOT} && {ROS_SETUP} && "
            f"echo {shlex.quote('[Azas] measured press pose dispenser_' + dispenser_id + ': xyz_m=' + str(press_xyz_m) + ' rpy_deg=' + str(press_rpy_deg) + ' source=calibration.yaml dispenser_outlets.' + dispenser_id + '; legacy taught/color posx disabled')} && "
            "ros2 run azas_dispenser dispenser_press_node --ros-args "
            f"-p service_prefix:={shlex.quote(service_prefix)} "
            "-p use_taught_posx:=false "
            "-p use_home_as_reference:=false "
            "-p keep_home_orientation:=false "
            f"-p dispenser_x:={press_xyz_m[0]:.6f} "
            f"-p dispenser_y:={press_xyz_m[1]:.6f} "
            "-p dispenser_y_offset:=0.0 "
            f"-p dispenser_top_z:={press_xyz_m[2]:.6f} "
            f"-p rx:={press_rpy_deg[0]:.6f} "
            f"-p ry:={press_rpy_deg[1]:.6f} "
            f"-p rz:={press_rpy_deg[2]:.6f} "
            "-p press_count:=1 "
            # calibration.yaml press_pose_xyz_m is the taught final press pose.
            # Do not subtract an extra legacy pump depth here.
            "-p press_depth:=0.0 "
            f"-p tcp_name:={shlex.quote(tcp_name)} "
            "-p require_tcp_for_taught_posx:=false "
            "-p allow_tcp_set_failure:=false "
            "-p move_home_first:=false "
            "-p pre_home_retreat_before_home:=false "
            "-p pre_home_retreat_dx_mm:=-180.0 "
            "-p pre_home_retreat_dy_mm:=0.0 "
            "-p pre_home_retreat_min_z_mm:=520.0 -p pre_home_retreat_lift_first:=true "
            "-p pre_home_retreat_min_current_x_mm:=0.0 "
            "-p pre_home_retreat_velocity:=20.0 "
            "-p pre_home_retreat_acceleration:=25.0 "
            "-p joint1_clearance_before_home:=false "
            "-p joint1_clearance_return_home:=false "
            "-p joint1_clearance_offset_deg:=12.0 "
            "-p return_home:=false "
            "-p close_gripper_at_home:=false "
            "-p post_press_retreat_after_sequence:=true "
            "-p post_press_retreat_dx_mm:=-120.0 "
            "-p post_press_retreat_dy_mm:=0.0 "
            "-p post_press_retreat_wait_seconds:=1.0 "
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
            "--timeout-sec 120 --wait-service-sec 15 --verify-timeout-sec 45 "
            "--target-tolerance-mm 15 --gripper-grasp-width-m 0.075 --gripper-force-n 25.0 "
            "--x-min 0.10 --x-max 0.95 "
            "--execute --confirm ENABLE_PICK_FROM_MEASURED_DISPENSER_FRONT_HOLD"
            f" && {tumbler_scene_once('remove_world', object_id=f'tumbler_at_dispenser_{dispenser_id}', dispenser_id=dispenser_id)}"
            f" && {tumbler_scene_once('attach', object_id='carried_tumbler', dispenser_id=dispenser_id)}"
        )
    if step.key == "place_cup_holder":
        place_final_z_offset_m = str(
            payload.get("cup_holder_place_final_z_offset_m")
            or os.environ.get("CUP_HOLDER_PLACE_FINAL_Z_OFFSET_M")
            or "-0.030"
        ).strip()
        place_final_x_offset_m = str(
            payload.get("cup_holder_place_final_x_offset_m")
            or os.environ.get("CUP_HOLDER_PLACE_FINAL_X_OFFSET_M")
            or "0.015"
        ).strip()
        place_final_y_offset_m = str(
            payload.get("cup_holder_place_final_y_offset_m")
            or os.environ.get("CUP_HOLDER_PLACE_FINAL_Y_OFFSET_M")
            or "-0.010"
        ).strip()
        cup_holder_rz_offset_deg = str(
            payload.get("cup_holder_rz_offset_deg")
            or os.environ.get("CUP_HOLDER_RZ_OFFSET_DEG")
            or "-1.0"
        ).strip()
        return (
            f"cd {ROOT} && {ROS_SETUP} && python3 tools/run/place_side_grip_cup_in_holder.py "
            f"--service-prefix {service_prefix} "
            "--config /home/ssu/Azas/install/azas_bringup/share/azas_bringup/config/calibration.yaml "
            "--motion-backend moveit "
            "--moveit-planning-pipeline ompl --moveit-planner-id RRTConnectkConfigDefault "
            "--moveit-planning-time-sec 8.0 --moveit-planning-attempts 5 "
            "--moveit-velocity-scaling 0.08 --moveit-acceleration-scaling 0.06 "
            "--approach-velocity 80.0 --approach-acceleration 20.0 "
            f"--place-final-x-offset-m {shlex.quote(place_final_x_offset_m)} "
            f"--place-final-y-offset-m {shlex.quote(place_final_y_offset_m)} "
            f"--place-final-z-offset-m {shlex.quote(place_final_z_offset_m)} "
            f"--rz-offset-deg {shlex.quote(cup_holder_rz_offset_deg)} "
            "--place-velocity 80.0 --place-acceleration 10.0 "
            "--retreat-velocity 80.0 --retreat-acceleration 16.0 "
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
            "--approach-velocity 40.0 --approach-acceleration 40.0 "
            "--descend-velocity 40.0 --descend-acceleration 40.0 "
            "--lift-velocity 40.0 --lift-acceleration 40.0 "
            f"--place-final-z-offset-m {shlex.quote(pick_z_offset_m)} "
            "--timeout-sec 90.0 --target-tolerance-mm 12.0 --verify-timeout-sec 45.0 "
            "--ikin-timeout-sec 20.0 --ikin-retries 2 "
            "--gripper-grasp-width-m 0.068 --gripper-force-n 25.0 "
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
            "REQUIRE_STATE_VALIDITY_FOR_JOINT_SHAKE=false "
            "REAL_ROBOT_MOTION_CONFIRM=ENABLE_REAL_ROBOT_MOTION "
            "tools/run/run_rule_based_shake_real.sh"
            " && echo '[Azas] SHAKE DONE: 손 검출/핸드오버를 위해 카메라 포즈로 복귀합니다 (컵 파지 유지).' && "
            "python3 tools/run/direct_movej_joints.py "
            f"--service-prefix {service_prefix} "
            "--j1 3.0 --j2 -12.7 --j3 44.0 --j4 -9.0 --j5 133.0 --j6 90.0 "
            "--velocity 15 --acceleration 15 "
            "--j5-min-deg -150 --j5-max-deg 150 --timeout-sec 60 --motion-timeout-sec 120 "
            "--execute --confirm ENABLE_DIRECT_MOVEJ"
        )
    if step.key == "handover_cup_to_palm":
        release_height_m = str(
            payload.get("handover_release_tcp_above_palm_m")
            or os.environ.get("HANDOVER_RELEASE_TCP_ABOVE_PALM_M")
            or "0.08"
        ).strip()
        return (
            f"cd {ROOT} && "
            f"{ROS_SETUP} && "
            "echo '[Azas] HANDOVER START: 펼친 손바닥을 추적해 컵을 손 위에 내려놓습니다.' && "
            "echo '[Azas] 전제: 손 검출 시작 버튼이 켜져 있고, 받는 사람이 손바닥을 펴고 멈춰 있어야 합니다.' && "
            "echo '[Azas] 안전: 하강은 2cm 스텝마다 외력을 확인하고, 손이 움직이면 자동 후퇴합니다.' && "
            "python3 tools/run/handover_cup_to_palm.py "
            f"--service-prefix {service_prefix} "
            f"--release-tcp-above-palm-m {shlex.quote(release_height_m)} "
            "--transit-velocity 10.0 --transit-acceleration 14.0 "
            "--descent-velocity 4.0 --descent-acceleration 6.0 "
            "--force-abort-delta-n 10.0 "
            "--execute --confirm ENABLE_HUMAN_PALM_HANDOVER "
            "--approve-motion ENABLE_HUMAN_PALM_HANDOVER_MOTION "
            "--approve-release RELEASE_CUP_NOW"
        )
    if step.command.strip():
        return f"cd {ROOT} && {ROS_SETUP} && {step.command}"
    return ""


def run_step(step: Step, payload: dict[str, Any]) -> dict[str, Any]:
    if not step.implemented:
        return {"key": step.key, "status": "blocked", "output": step.note}
    if step.real_motion and not payload.get("armed"):
        return {"key": step.key, "status": "blocked", "output": "실제 모션 허용 체크가 꺼져 있습니다."}
    if step.key in PANEL_DIRECT_TMUX_STEPS:
        env = shell_env(payload)
        service_prefix = str(payload.get("service_prefix") or "dsr01")
        if step.key not in PANEL_FIELD_VERIFIED_DIRECT_TMUX_STEPS:
            cmd = command_for(step, payload)
            return {
                "key": step.key,
                "status": "blocked",
                "output": (
                    "이 단계는 패널에 저장된 명령 후보는 있지만, 아직 동일한 terminal/tmux 방식으로 "
                    "실제 성공 검증이 끝나지 않아 패널에서 실행하지 않았습니다.\n"
                    "먼저 터미널/tmux에서 성공 로그를 확인한 뒤 패널 허용 목록에 올려야 합니다.\n"
                    "--- command candidate ---\n"
                    f"{cmd}\n"
                ),
            }
        cleanup_output = ""
        if step.key in {"side_grip", "cup_uprighting"}:
            if step.key == "side_grip":
                cleanup_output = "\n".join(cleanup_side_grip_stack(grace_sec=3.0))
                label = "창현 side_grip"
            else:
                cleanup_output = "\n".join(cleanup_cup_uprighting_stack(grace_sec=3.0))
                label = "소명 cup_uprighting"
            time.sleep(1.0)
            cmd = command_for(step, payload)
            restart_output = "\n".join(
                part
                for part in (
                    f"[Azas] field-verified tmux mode: {label}은 ROS CLI discovery preflight로 막지 않고 검증된 tmux 명령을 직접 실행합니다.",
                    "[Azas] 전제: 먼저 'tmux 연결 스택 시작'으로 robot/gripper/camera/joint_relay 창이 떠 있어야 합니다.",
                    cleanup_output,
                )
                if part
            )
            return run_background_step_in_tmux(step, cmd, env, restart_output=restart_output)
        elif step.key == "cup_uprighting":
            cleanup_output = "\n".join(cleanup_cup_uprighting_stack(grace_sec=3.0))
            time.sleep(1.0)
        elif step.key == "lid_grip_close":
            cleanup_output = "\n".join(
                cleanup_matching_processes(LID_GRIP_STACK_PATTERNS, label="lid_grip cleanup", grace_sec=3.0)
            )
            time.sleep(1.0)
        preflight_ok, preflight_output = manual_logic_preflight(step, env, service_prefix)
        if not preflight_ok:
            return {
                "key": step.key,
                "status": "blocked",
                "output": (
                    "패널 수동 로직 실행 전 준비 조건이 충족되지 않아 시작하지 않았습니다.\n"
                    + (cleanup_output + "\n" if cleanup_output else "")
                    + preflight_output
                ),
            }
        cmd = command_for(step, payload)
        restart_output = "\n".join(
            [
                "[Azas] direct tmux mode: 최소 준비 게이트 통과 후 현장 tmux launch 명령을 실행합니다.",
                "[Azas] 확인됨: motion services, robot_state=STANDBY, check_motion, MoveIt action, gripper services, camera topics.",
                cleanup_output,
                preflight_output,
            ]
        )
        return run_background_step_in_tmux(step, cmd, env, restart_output=restart_output)
    if step.real_motion and step.key not in {"run_one_click_cocktail_real", "run_cocktail_now_real"}:
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
                action_ready, action_output = wait_for_action_server(action_name, timeout_sec=3.0)
                if not action_ready:
                    # ROS action graph introspection can stall on the field setup
                    # even when MoveIt execution works.  The side-grip node still
                    # performs its own MoveIt planning/execution checks, so this
                    # panel gate is advisory only.
                    print(
                        "[Azas panel] warning: side_grip action introspection timed out; continuing\n"
                        + action_output,
                        flush=True,
                    )
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
    if step.key == "cup_uprighting":
        cleanup_events = cleanup_cup_uprighting_stack(grace_sec=3.0)
        time.sleep(1.0)
        service_prefix = str(payload.get("service_prefix") or "dsr01")
        preflight_ok, preflight_details = cup_uprighting_preflight(env, service_prefix)
        preflight_output = "\n".join(
            cleanup_events
            + [
                "--- cup_uprighting preflight ---",
                preflight_details,
            ]
        ).strip()
        if not preflight_ok:
            return {
                "key": step.key,
                "status": "blocked",
                "output": (
                    "cup_uprighting 실행 전 조건이 충족되지 않아 시작하지 않았습니다.\n"
                    f"{preflight_output}"
                ),
            }

    cmd = command_for(step, payload)
    if step.kind == "background":
        restart_output = ""
        if step.key in {"connect_robot", "start_tmux_stack"}:
            cleanup_events: list[str] = []
            cleanup_events.extend(cleanup_side_grip_stack(grace_sec=3.0))
            cleanup_events.extend(cleanup_camera_stack(grace_sec=3.0))
            cleanup_events.extend(cleanup_rg2_stack(grace_sec=3.0))
            restart_output = "\n".join(
                part
                for part in (
                    "[Azas] tmux 통합 재연결: stop_azas_all.sh가 azas-logic tmux 세션을 종료하므로 "
                    "패널 서버가 tmux 밖에서 터미널과 같은 stop -> start 명령을 직접 실행합니다.",
                    "[Azas] reconnect pre-cleanup: 이전 side_grip/camera/RG2 잔여 프로세스를 먼저 정리합니다.",
                    "\n".join(cleanup_events),
                )
                if part
            )
        elif step.key == "connect_gripper":
            cleanup_events = cleanup_rg2_stack()
            # DDS may keep stale service names briefly after a killed RG2 wrapper.
            time.sleep(1.0)
            restart_output = "\n".join(cleanup_events)
        elif step.key == "start_camera":
            cleanup_events = cleanup_camera_stack()
            # Avoid duplicate /camera/camera nodes from previous panel attempts.
            # Do not probe camera topics here: ros2cli graph/topic calls can
            # wedge in the field and leave stale daemon/query processes.  The
            # RealSense tmux window is the source of truth for startup logs.
            time.sleep(0.4)
            restart_output = "\n".join(cleanup_events)
        elif step.key == "side_grip":
            # Cleanup and PR #20 preflight already ran above. Do not repeat it here;
            # repeated cleanup sleeps were making the manual picker feel frozen.
            restart_output = preflight_output
        elif step.key == "start_collision_scene":
            cleanup_events = cleanup_collision_scene_stack(grace_sec=2.0)
            time.sleep(0.5)
            restart_output = "\n".join(cleanup_events)
        else:
            old = processes.get(step.key)
            if old and old.poll() is None:
                return {
                    "key": step.key,
                    "status": "running",
                    "output": "이미 실행 중입니다.\n" + tail_file(process_logs.get(step.key)),
                    "pid": old.pid,
                }
        if step.key in PANEL_TMUX_STEPS:
            return run_background_step_in_tmux(step, cmd, env, restart_output=restart_output)
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
        if step.key in {"start_tmux_stack", "connect_robot"}:
            try:
                proc.wait(timeout=90.0)
            except subprocess.TimeoutExpired:
                output = (
                    f"{cmd}\n--- log ---\n{log_path}\n"
                    "tmux 통합 재연결 명령이 90초 안에 종료되지 않았습니다.\n"
                    + tail_file(log_path, max_chars=8000)
                )
                if restart_output:
                    output = f"{restart_output}\n--- start command ---\n{output}"
                return {"key": step.key, "status": "starting", "pid": proc.pid, "output": output}
            output = f"{cmd}\n--- log ---\n{log_path}\n--- start output ---\n{tail_file(log_path, max_chars=10000)}"
            if restart_output:
                output = f"{restart_output}\n--- start command ---\n{output}"
            if proc.returncode != 0:
                return {"key": step.key, "status": "failed", "returncode": proc.returncode, "output": output}
            return {
                "key": step.key,
                "status": "passed",
                "pid": proc.pid,
                "output": (
                    output
                    + "\n[Azas] tmux 통합 재연결 명령 완료. robot/gripper/camera/joint_relay는 각 tmux 창 로그를 기준으로 확인합니다. "
                    "ROS CLI daemon/discovery 오류 때문에 이 단계에서 후속 조회로 차단하지 않습니다."
                ),
            }

        time.sleep(2.0)
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
                return {"key": step.key, "status": "passed", "pid": proc.pid, "output": output}
            output += "\n[Azas] RG2 bridge is still starting; retry gripper connection if services stay absent."
            return {"key": step.key, "status": "starting", "pid": proc.pid, "output": output}
        if step.key == "start_collision_scene":
            ready, waited_output = wait_for_collision_object_sample(env=env, timeout_sec=10.0, proc=proc)
            tf_ready = False
            tf_output = ""
            if ready:
                tf_ready, tf_output = wait_for_tf_transform(
                    env=env,
                    target_frame=HAND_EYE_TF_TARGET_FRAME,
                    source_frame=HAND_EYE_TF_SOURCE_FRAME,
                    timeout_sec=12.0,
                    proc=proc,
                )
            output = f"{cmd}\n--- log ---\n{log_path}\n--- readiness ---\n{waited_output}"
            if ready:
                output += f"\n--- hand-eye TF readiness ---\n{tf_output}"
            if restart_output:
                output = f"{restart_output}\n--- start command ---\n{output}"
            if ready and tf_ready:
                return {"key": step.key, "status": "passed", "pid": proc.pid, "output": output}
            return {"key": step.key, "status": "starting", "pid": proc.pid, "output": output}
        if step.key == "start_camera":
            ready, waited_output = wait_for_camera_topic_samples(env=env, timeout_sec=15.0, proc=proc)
            output = f"{cmd}\n--- log ---\n{log_path}\n--- readiness ---\n{waited_output}"
            if ready:
                return {"key": step.key, "status": "passed", "pid": proc.pid, "output": output}
            return {"key": step.key, "status": "starting", "pid": proc.pid, "output": output}
        if step.key == "detect_cup_lid":
            ready, waited_output = wait_for_cup_detection_sample(env=env, timeout_sec=10.0, proc=proc)
            output = f"{cmd}\n--- log ---\n{log_path}\n--- readiness ---\n{waited_output}"
            if ready:
                return {"key": step.key, "status": "passed", "pid": proc.pid, "output": output}
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
        timeout_sec = run_timeout_for_step(step)
        log_path = background_log_path(step.key)
        log_handle = log_path.open("w", encoding="utf-8", buffering=1)
        log_handle.write(f"[Azas panel] command: {cmd}\n\n")
        proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            cwd=str(ROOT),
            env=env,
            stdin=subprocess.PIPE,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        processes[step.key] = proc
        process_logs[step.key] = log_path
        if proc.stdin is not None:
            try:
                proc.stdin.write("ENABLE_REAL_ROBOT_MOTION\n")
                proc.stdin.close()
            except OSError:
                pass
        deadline = time.monotonic() + timeout_sec
        while proc.poll() is None:
            if time.monotonic() >= deadline:
                terminate_process_tree(proc, label=step.key, grace_sec=3.0)
                try:
                    log_handle.close()
                except OSError:
                    pass
                output = tail_file(log_path)
                if preflight_output:
                    output = f"{preflight_output}\n--- command output ---\n{output}"
                return {"key": step.key, "status": "timeout", "output": output}
            time.sleep(0.25)
        log_handle.close()
        output = tail_file(log_path, max_chars=50000)
        completed_returncode = proc.returncode
        if preflight_output:
            output = f"{preflight_output}\n--- command output ---\n{output}"
        if completed_returncode == 0:
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
            if step.key == "color_scan":
                try:
                    color_map = json.loads(DISPENSER_COLOR_MAP_PATH.read_text(encoding="utf-8"))
                    DISPENSER_PRESS_TARGETS.clear()
                    DISPENSER_PRESS_TARGETS.update({str(k): str(v) for k, v in color_map.items()})
                    lines = ["--- 색상 스캔 결과 ---"]
                    for did in sorted(color_map.keys(), key=lambda x: int(x) if x.isdigit() else x):
                        lines.append(f"  디스펜서 {did}: {color_map[did]}")
                    output = f"{output}\n" + "\n".join(lines) + "\n"
                    if not color_map:
                        return {
                            "key": step.key,
                            "status": "failed",
                            "returncode": 1,
                            "output": output + "[color_scan] 결과가 비어 있습니다.\n",
                        }
                    unknown = [str(did) for did, color in color_map.items() if str(color).lower() == "unknown"]
                    if unknown:
                        output += "[color_scan] WARNING: unknown result for dispenser(s): " + ", ".join(sorted(unknown)) + "\n"
                except Exception as exc:
                    output = f"{output}\n[color_scan] 결과 파일 읽기 실패: {exc}\n"
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
            "status": "passed" if completed_returncode == 0 else "failed",
            "returncode": completed_returncode,
            "output": output,
        }
    except OSError as exc:
        output = f"[Azas] command launch failed: {exc}"
        if preflight_output:
            output = f"{preflight_output}\n--- command output ---\n{output}"
        return {"key": step.key, "status": "failed", "output": output}




def running_log_snapshot(*, max_chars: int = 10000) -> list[dict[str, Any]]:
    """Return live tails for processes launched by this panel.

    Foreground `/api/run` steps also register their temporary log file while the
    request is still running, so the browser can poll this endpoint instead of
    showing only "실행 중".
    """

    snapshots: list[dict[str, Any]] = []
    for key, proc in list(processes.items()):
        log_path = process_logs.get(key)
        status = "running" if proc.poll() is None else "exited"
        snapshots.append(
            {
                "key": key,
                "pid": proc.pid,
                "status": status,
                "returncode": proc.returncode,
                "log_path": str(log_path) if log_path else "",
                "tail": tail_file(log_path, max_chars=max_chars),
            }
        )
    return snapshots

def stop_all() -> dict[str, Any]:
    stopped: list[dict[str, Any]] = []
    for key, proc in list(processes.items()):
        if proc.poll() is None:
            events = terminate_process_tree(proc, label=key, grace_sec=5.0)
            stopped.append({"key": key, "pid": proc.pid, "events": events})
        processes.pop(key, None)
    return {"stopped": stopped}


def cleanup_all_processes() -> dict[str, Any]:
    """Explicit operator cleanup button: stop tracked jobs plus stale robot/panel helpers."""
    stopped = stop_all()
    events: list[str] = []
    events.extend(cleanup_run_step_stack(grace_sec=3.0))
    events.extend(cleanup_side_grip_stack(grace_sec=3.0))
    events.extend(cleanup_collision_scene_stack(grace_sec=3.0))
    events.extend(cleanup_camera_stack(grace_sec=3.0))
    events.extend(cleanup_rg2_stack(grace_sec=3.0))
    events.extend(cleanup_doosan_stack(grace_sec=3.0))
    events.extend(
        cleanup_matching_processes(
            AUXILIARY_STACK_PATTERNS,
            label="aux cleanup",
            grace_sec=3.0,
        )
    )
    events.extend(stop_ros2_daemon())
    process_logs.clear()
    return {"stopped": stopped.get("stopped", []), "cleanup": events}


class Handler(BaseHTTPRequestHandler):
    def send_json(self, data: Any, status: int = 200) -> None:
        body = json.dumps(data, ensure_ascii=False, indent=2).encode("utf-8")
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except BrokenPipeError:
            # Browser polling can cancel a request while logs are still being read.
            # Do not flood panel logs with tracebacks for harmless client disconnects.
            return

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
            command_overrides = load_command_overrides()
            preview_payload = {
                "robot_host": os.environ.get("ROBOT_HOST", DEFAULT_ROBOT_HOST),
                "robot_name": os.environ.get("ROBOT_NAME", "dsr01"),
                "service_prefix": os.environ.get("SERVICE_PREFIX", "dsr01"),
                "rg2_ip": os.environ.get("RG2_IP", "192.168.1.1"),
                "dispenser_tcp_name": os.environ.get(
                    "DISPENSER_TCP_NAME", DEFAULT_DISPENSER_TCP_NAME
                ),
                "selected_dispenser_id": os.environ.get("SELECTED_DISPENSER_ID", "2"),
                "cup_holder_place_final_x_offset_m": os.environ.get(
                    "CUP_HOLDER_PLACE_FINAL_X_OFFSET_M", "0.015"
                ),
                "cup_holder_place_final_y_offset_m": os.environ.get(
                    "CUP_HOLDER_PLACE_FINAL_Y_OFFSET_M", "-0.010"
                ),
                "cup_holder_place_final_z_offset_m": os.environ.get(
                    "CUP_HOLDER_PLACE_FINAL_Z_OFFSET_M", "-0.040"
                ),
                "cup_holder_rz_offset_deg": os.environ.get(
                    "CUP_HOLDER_RZ_OFFSET_DEG", "-1.0"
                ),
            }
            data = []
            for step in STEPS:
                if step.key in PANEL_HIDDEN_STEP_KEYS:
                    continue
                item = asdict(step)
                item["resolved_command"] = command_for(step, preview_payload) if step.implemented else ""
                item["command_saved"] = step.key in command_overrides
                data.append(item)
            self.send_json(data)
            return
        if path == "/api/running_logs":
            self.send_json({"logs": running_log_snapshot()})
            return
        if path == "/api/dispenser_color_map":
            self.send_json(dispenser_color_map_status())
            return
        if path == "/api/camera_snapshot.jpg":
            if os.environ.get("AZAS_PANEL_ENABLE_CAMERA_SNAPSHOT", "0") not in {"1", "true", "TRUE"}:
                message = b"camera snapshot endpoint disabled in field panel"
                try:
                    self.send_response(404)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.send_header("Content-Length", str(len(message)))
                    self.end_headers()
                    self.wfile.write(message)
                except BrokenPipeError:
                    pass
                return
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
            if not RUN_LOCK.acquire(blocking=False):
                self.send_json(
                    {
                        "error": "another pipeline step is already running",
                        "results": [
                            {
                                "key": "pipeline",
                                "status": "blocked",
                                "output": "이미 다른 실행 요청이 처리 중입니다. 현재 단계가 끝난 뒤 다시 실행하세요.",
                            }
                        ],
                    },
                    409,
                )
                return
            try:
                raw_selected = [str(key) for key in payload.get("selected") or []]
                if payload.get("selected_already_expanded"):
                    selected = list(dict.fromkeys(raw_selected))
                else:
                    selected = with_collision_scene_prereq(raw_selected)
                    selected = list(dict.fromkeys(selected))
                selected = configure_manual_recipe_chain(selected, payload)
                steps_by_key = {step.key: step for step in STEPS}
                results = []
                for key in selected:
                    if key in PANEL_HIDDEN_STEP_KEYS:
                        results.append(
                            {
                                "key": key,
                                "status": "blocked",
                                "output": "이 단계는 패널에서 제거된 내부/구버전 단계라 실행하지 않았습니다.",
                            }
                        )
                        break
                    step = steps_by_key.get(key)
                    if step is None:
                        continue
                    result = run_step(step, payload)
                    results.append(result)
                    status = str(result.get("status") or "")
                    # Fail closed for server-side multi-step requests too.
                    # This prevents a queued motion step from running while a
                    # prerequisite is still starting, failed, timed out, or
                    # waiting for a prerequisite or failed motion.
                    if status in {"failed", "blocked", "timeout", "starting"}:
                        break
                self.send_json({"execution_order": selected, "results": results})
            finally:
                RUN_LOCK.release()
            return
        if path == "/api/dispenser_color_map":
            new_map = payload.get("map")
            if not isinstance(new_map, dict):
                self.send_json({"error": "body must be {\"map\": {\"1\": \"red\", ...}}"}, 400)
                return
            validated = {str(k): str(v) for k, v in new_map.items()}
            DISPENSER_PRESS_TARGETS.clear()
            DISPENSER_PRESS_TARGETS.update(validated)
            _write_json_file_immediately(DISPENSER_COLOR_MAP_PATH, validated)
            _unlink_file_immediately(DISPENSER_COLOR_MAP_FAILED_PATH)
            self.send_json(dispenser_color_map_status())
            return
        if path == "/api/command_override":
            step_key = str(payload.get("key") or "")
            command = str(payload.get("command") or "")
            try:
                overrides = save_command_override(step_key, command)
            except ValueError as exc:
                self.send_json({"error": str(exc)}, 400)
                return
            self.send_json({"overrides": overrides})
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
    try:
        server = ThreadingHTTPServer((host, port), Handler)
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            print(f"[Azas] panel port is already in use: http://{host}:{port}", flush=True)
            print("[Azas] Open the existing panel, or start a second one with:", flush=True)
            print(f"  AZAS_PANEL_PORT={port + 1} bash tools/run/run_robot_pipeline_control_panel.sh", flush=True)
            return 98
        raise
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
