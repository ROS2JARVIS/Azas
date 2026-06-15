from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import threading
from collections import deque

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - keeps pure helper tests ROS-free.
    rclpy = None
    Node = object
    String = None


ALLOWED_DISPENSERS = ("red", "yellow", "green", "blue")
DEFAULT_RESUME_STATE_FILE = Path("/home/ssu/Azas/outputs/auto_cup_flow_resume.json")
DEFAULT_RESUME_EVENTS_FILE = Path("/home/ssu/Azas/outputs/auto_cup_flow_events.jsonl")
DEFAULT_DISPENSER_RESUME_STATE_FILE = Path("/home/ssu/Azas/outputs/measured_dispenser_recipe_resume.json")

# 라우터 stdout에서 단계 전환을 감지해 UI에 보여줄 한국어 단계명으로 변환한다.
# (auto_cup_flow_router의 로그 문구가 바뀌면 여기도 같이 갱신할 것)
STAGE_MARKERS: tuple[tuple[str, str], ...] = (
    ("auto cup router: color scan", "디스펜서 색 스캔"),
    ("route decided: side_grasp", "컵 픽업 (세워진 컵)"),
    ("route decided: cup_uprighting", "컵 픽업 (쓰러진 컵)"),
    ("starting integrated dispenser recipe sequence", "디스펜서 레시피 진행"),
    ("resume_state loaded", "중단 지점 복구"),
    ("resume_state step_start", "디스펜서 레시피 진행"),
    ("starting lid close", "뚜껑 체결 / 쉐이킹"),
    ("selected flow completed", "완료"),
)


def recipe_colors_from_decision(
    decision: dict[str, object],
    *,
    max_repeats_per_dispenser: int = 3,
    default_amount: int = 1,
) -> str:
    """confirmed decision JSON을 라우터 recipe_colors 문자열로 변환한다.

    dispenser_amounts가 있으면 그 양을, 없으면 dispenser_ids마다 default_amount를 쓴다.
    예: {"dispenser_amounts": {"yellow": 2, "blue": 1}} -> "yellow:2,blue:1"
    """
    if decision.get("intent") != "make_cocktail":
        return ""

    amounts_payload = decision.get("dispenser_amounts", {})
    amounts: dict[str, int] = {}
    if isinstance(amounts_payload, dict):
        for color in ALLOWED_DISPENSERS:
            try:
                amount = int(amounts_payload.get(color, 0))
            except (TypeError, ValueError):
                amount = 0
            amounts[color] = max(0, min(amount, max_repeats_per_dispenser))

    if not any(amounts.values()):
        raw_ids = decision.get("dispenser_ids", [])
        if not isinstance(raw_ids, list):
            return ""
        for raw_id in raw_ids:
            color = str(raw_id).strip()
            if color in ALLOWED_DISPENSERS:
                amounts[color] = max(
                    amounts.get(color, 0),
                    min(default_amount, max_repeats_per_dispenser),
                )

    parts = [f"{color}:{amounts[color]}" for color in ALLOWED_DISPENSERS if amounts.get(color, 0) > 0]
    return ",".join(parts)


def stage_from_line(line: str) -> str | None:
    for marker, stage in STAGE_MARKERS:
        if marker in line:
            return stage
    return None


def load_resume_snapshot(path: str | Path = DEFAULT_RESUME_STATE_FILE) -> dict[str, object] | None:
    state_path = Path(path)
    if not state_path.is_file():
        return None
    try:
        payload = json.loads(state_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def recipe_colors_from_resume_snapshot(snapshot: dict[str, object] | None) -> str:
    if not isinstance(snapshot, dict):
        return ""
    recipe = snapshot.get("recipe")
    if not isinstance(recipe, dict):
        return ""
    return str(recipe.get("recipe_colors") or "").strip()


class VoicePipelineExecutorNode(Node):
    """Confirmed voice recipe -> full auto cup flow (pick -> recipe -> lid -> shake).

    voice_dispenser_executor_node가 디스펜서 프레스 단발만 실행하는 것과 달리,
    이 노드는 검증된 auto_cup_flow_router 전체 파이프라인을 래퍼 스크립트로 실행한다.
    """

    def __init__(self):
        super().__init__("voice_pipeline_executor_node")
        self.declare_parameter("confirmed_decision_topic", "/azas/voice/confirmed_recipe_decision")
        self.declare_parameter("recovery_command_topic", "/azas/voice/recovery_command")
        self.declare_parameter("status_topic", "/azas/voice/pipeline_status")
        self.declare_parameter("enable_hardware_execution", False)
        self.declare_parameter("require_confirmed", True)
        self.declare_parameter("flow_script", "/home/ssu/Azas/tools/run/run_voice_auto_cup_flow.sh")
        self.declare_parameter("service_prefix", "dsr01")
        self.declare_parameter("max_repeats_per_dispenser", 3)
        self.declare_parameter("default_amount", 1)
        self.declare_parameter("resume_state_file", str(DEFAULT_RESUME_STATE_FILE))
        self.declare_parameter("resume_events_file", str(DEFAULT_RESUME_EVENTS_FILE))
        self.declare_parameter("dispenser_resume_state_file", str(DEFAULT_DISPENSER_RESUME_STATE_FILE))

        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("confirmed_decision_topic").value),
            self._on_confirmed_decision,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("recovery_command_topic").value),
            self._on_recovery_command,
            10,
        )

        self._lock = threading.Lock()
        self._active_proc: subprocess.Popen[str] | None = None
        self._active_recipe_id: str | None = None

        self.get_logger().info(
            "Voice pipeline executor ready: "
            f"enable_hardware_execution={bool(self.get_parameter('enable_hardware_execution').value)}"
        )

    def _on_confirmed_decision(self, msg: String) -> None:
        try:
            decision = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._publish_status("blocked", reason="invalid_confirmed_decision_json", error=str(exc))
            return

        if bool(self.get_parameter("require_confirmed").value) and not decision.get("confirmed"):
            self._publish_status("blocked", reason="decision_not_confirmed", decision=decision)
            return

        recipe_colors = recipe_colors_from_decision(
            decision,
            max_repeats_per_dispenser=int(self.get_parameter("max_repeats_per_dispenser").value),
            default_amount=int(self.get_parameter("default_amount").value),
        )
        if not recipe_colors:
            self._publish_status("blocked", reason="no_executable_recipe_colors", decision=decision)
            return

        self._start_pipeline(decision, recipe_colors, resume_mode="normal", trigger="confirmed_recipe")

    def _on_recovery_command(self, msg: String) -> None:
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._publish_status("blocked", reason="invalid_recovery_command_json", error=str(exc))
            return
        intent = str(command.get("intent") or "")
        if intent == "clear_recovery":
            self._clear_recovery_state()
            self._publish_status("recovery_cleared", stage="복구 기록 초기화")
            return

        snapshot = load_resume_snapshot(str(self.get_parameter("resume_state_file").value))
        if intent == "recheck_recovery":
            self._publish_recovery_check(snapshot)
            return
        if intent not in {"resume_flow", "restart_flow"}:
            self._publish_status("blocked", reason="unsupported_recovery_intent", intent=intent)
            return
        if not snapshot:
            self._publish_status(
                "blocked",
                reason="no_resume_state",
                required_user_action="저장된 복구 상태가 없습니다. 새 주문을 먼저 시작하세요.",
            )
            return

        recipe_colors = recipe_colors_from_resume_snapshot(snapshot)
        if not recipe_colors:
            self._publish_status(
                "blocked",
                reason="resume_state_missing_recipe",
                recovery_snapshot=snapshot,
                required_user_action="저장된 주문 정보가 없어 처음부터 새 메뉴를 주문해야 합니다.",
            )
            return
        status = str(snapshot.get("status") or "")
        if intent == "resume_flow" and status == "completed":
            self._publish_status("completed", reason="resume_state_already_completed", recovery_snapshot=snapshot)
            return
        if intent == "resume_flow" and snapshot.get("auto_recoverable") is False:
            self._publish_status(
                "blocked",
                reason=str(snapshot.get("blocker") or "manual_recovery_required"),
                recovery_snapshot=snapshot,
                required_user_action=snapshot.get("required_user_action")
                or "하드웨어 상태를 조치한 뒤 복구 다시 확인이라고 말하세요.",
            )
            return

        resume_mode = "restart" if intent == "restart_flow" else "resume"
        self._start_pipeline(command, recipe_colors, resume_mode=resume_mode, trigger="voice_recovery")

    def _publish_recovery_check(self, snapshot: dict[str, object] | None) -> None:
        if not snapshot:
            self._publish_status(
                "blocked",
                reason="no_resume_state",
                required_user_action="저장된 복구 상태가 없습니다.",
            )
            return
        if snapshot.get("auto_recoverable") is False:
            self._publish_status(
                "blocked",
                reason=str(snapshot.get("blocker") or "manual_recovery_required"),
                recovery_snapshot=snapshot,
                required_user_action=snapshot.get("required_user_action"),
            )
            return
        self._publish_status(
            "recovery_ready",
            stage="복구 가능 상태",
            next_stage=snapshot.get("next_stage"),
            recovery_snapshot=snapshot,
        )

    def _clear_recovery_state(self) -> None:
        for raw_path in (
            self.get_parameter("resume_state_file").value,
            self.get_parameter("resume_events_file").value,
            self.get_parameter("dispenser_resume_state_file").value,
        ):
            try:
                Path(str(raw_path)).unlink()
            except FileNotFoundError:
                pass
            except OSError as exc:
                self.get_logger().warn(f"failed to clear recovery file {raw_path}: {exc}")

    def _start_pipeline(
        self,
        decision: dict[str, object],
        recipe_colors: str,
        *,
        resume_mode: str,
        trigger: str,
    ) -> None:
        with self._lock:
            if self._active_proc is not None and self._active_proc.poll() is None:
                self._publish_status(
                    "busy",
                    reason="pipeline_already_running",
                    active_recipe_id=self._active_recipe_id,
                    rejected_recipe_id=decision.get("recipe_id"),
                )
                return
            self._active_recipe_id = str(decision.get("recipe_id") or "")

        command = [
            "bash",
            str(self.get_parameter("flow_script").value),
            recipe_colors,
        ]
        self._publish_status(
            "starting",
            recipe_id=decision.get("recipe_id"),
            recipe_colors=recipe_colors,
            command=command,
            hardware_enabled=bool(self.get_parameter("enable_hardware_execution").value),
            resume_mode=resume_mode,
            trigger=trigger,
        )

        if not bool(self.get_parameter("enable_hardware_execution").value):
            self._publish_status(
                "dry_run",
                recipe_colors=recipe_colors,
                command=command,
                resume_mode=resume_mode,
                trigger=trigger,
            )
            return

        env = os.environ.copy()
        env["ROUTER_CONFIRM"] = "ENABLE_AUTO_CUP_ROUTER"
        env["SERVICE_PREFIX"] = str(self.get_parameter("service_prefix").value)
        env["AUTO_FLOW_RESUME_MODE"] = resume_mode
        env["AUTO_FLOW_RESUME_STATE_FILE"] = str(self.get_parameter("resume_state_file").value)
        env["AUTO_FLOW_RESUME_EVENTS_FILE"] = str(self.get_parameter("resume_events_file").value)
        env["AUTO_FLOW_DISPENSER_RESUME_STATE_FILE"] = str(
            self.get_parameter("dispenser_resume_state_file").value
        )
        try:
            proc = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
                env=env,
            )
        except OSError as exc:
            self._publish_status("failed", reason="flow_failed_to_start", error=str(exc))
            return

        with self._lock:
            self._active_proc = proc
        threading.Thread(
            target=self._monitor_pipeline,
            args=(proc, recipe_colors, resume_mode),
            daemon=True,
        ).start()

    def _monitor_pipeline(self, proc: subprocess.Popen[str], recipe_colors: str, resume_mode: str) -> None:
        last_stage = ""
        output_tail: deque[str] = deque(maxlen=20)
        if proc.stdout is not None:
            for line in proc.stdout:
                line = line.rstrip()
                if line:
                    output_tail.append(line)
                    if any(marker in line for marker in ("[FAIL]", "[ERROR]", "process exited", "service=")):
                        self.get_logger().warn(f"flow> {line}")
                stage = stage_from_line(line)
                if stage and stage != last_stage:
                    last_stage = stage
                    self._publish_status("running", stage=stage, recipe_colors=recipe_colors)
        code = proc.wait()
        with self._lock:
            self._active_proc = None
            self._active_recipe_id = None
        if code == 0:
            self._publish_status("completed", recipe_colors=recipe_colors)
        else:
            self._publish_status(
                "failed",
                recipe_colors=recipe_colors,
                returncode=code,
                last_stage=last_stage,
                resume_mode=resume_mode,
                output_tail=list(output_tail),
            )

    def _publish_status(self, status: str, **fields: object) -> None:
        msg = String()
        msg.data = json.dumps({"status": status, **fields}, ensure_ascii=False)
        self._status_pub.publish(msg)
        if status in {"blocked", "failed", "busy"}:
            self.get_logger().warn(msg.data)
        else:
            self.get_logger().info(msg.data)

    def destroy_node(self):
        with self._lock:
            proc = self._active_proc
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    if rclpy is None:
        raise RuntimeError("rclpy is required to run voice_pipeline_executor_node")
    rclpy.init(args=args)
    node = VoicePipelineExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
