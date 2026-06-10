from __future__ import annotations

import json
import shutil
import subprocess
import threading
from collections import deque
from dataclasses import dataclass

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - keeps pure helper tests ROS-free.
    rclpy = None
    Node = object
    String = None


ALLOWED_DISPENSERS = ("red", "yellow", "green", "blue")


@dataclass(frozen=True)
class DispenserPressRequest:
    target_dispenser: str
    repeat_index: int
    repeat_total: int
    recipe_id: str | None


def _clamp_amount(value: object, default: int = 1, max_repeats: int = 3) -> int:
    try:
        amount = int(value)
    except (TypeError, ValueError):
        amount = default
    return max(0, min(amount, max_repeats))


def requests_from_decision(
    decision: dict[str, object],
    *,
    max_repeats_per_dispenser: int = 3,
    default_amount: int = 1,
) -> list[DispenserPressRequest]:
    if decision.get("intent") != "make_cocktail":
        return []

    recipe_id = decision.get("recipe_id")
    recipe_id = str(recipe_id) if recipe_id is not None else None

    raw_ids = decision.get("dispenser_ids", [])
    if not isinstance(raw_ids, list):
        return []

    amounts = decision.get("dispenser_amounts", {})
    if not isinstance(amounts, dict):
        amounts = {}

    requests: list[DispenserPressRequest] = []
    for raw_id in raw_ids:
        dispenser_id = str(raw_id).strip()
        if dispenser_id not in ALLOWED_DISPENSERS:
            continue

        amount = _clamp_amount(
            amounts.get(dispenser_id, default_amount),
            default=default_amount,
            max_repeats=max_repeats_per_dispenser,
        )
        for repeat_index in range(1, amount + 1):
            requests.append(
                DispenserPressRequest(
                    target_dispenser=dispenser_id,
                    repeat_index=repeat_index,
                    repeat_total=amount,
                    recipe_id=recipe_id,
                )
            )
    return requests


def build_dispenser_launch_command(
    request: DispenserPressRequest,
    *,
    launch_file: str,
    service_prefix: str,
    tcp_name: str,
    restore_tcp_after_run: bool,
    require_tcp_for_taught_posx: bool,
    allow_tcp_set_failure: bool,
    joint_velocity: float,
    joint_acceleration: float,
    line_velocity: float,
    line_acceleration: float,
) -> list[str]:
    command = [
        "ros2",
        "launch",
        "azas_dispenser",
        launch_file,
        f"target_dispenser:={request.target_dispenser}",
        f"service_prefix:={service_prefix}",
    ]
    # ros2 launch rejects an empty value ("tcp_name:="). When no TCP is named,
    # omit the argument so the launch file keeps its own default.
    if tcp_name:
        command.append(f"tcp_name:={tcp_name}")
    command += [
        f"restore_tcp_after_run:={str(restore_tcp_after_run).lower()}",
        f"require_tcp_for_taught_posx:={str(require_tcp_for_taught_posx).lower()}",
        f"allow_tcp_set_failure:={str(allow_tcp_set_failure).lower()}",
        f"joint_velocity:={joint_velocity}",
        f"joint_acceleration:={joint_acceleration}",
        f"line_velocity:={line_velocity}",
        f"line_acceleration:={line_acceleration}",
    ]
    return command


class VoiceDispenserExecutorNode(Node):
    """Convert confirmed voice recipe JSON into azas_dispenser launch commands."""

    def __init__(self):
        super().__init__("voice_dispenser_executor_node")
        self.declare_parameter("confirmed_decision_topic", "/azas/voice/confirmed_recipe_decision")
        self.declare_parameter("status_topic", "/azas/voice/dispenser_execution_status")
        self.declare_parameter("enable_hardware_execution", False)
        self.declare_parameter("require_confirmed", True)
        self.declare_parameter("dispenser_launch_file", "dispenser_press.launch.py")
        self.declare_parameter("service_prefix", "/")
        self.declare_parameter("tcp_name", "")
        self.declare_parameter("restore_tcp_after_run", True)
        self.declare_parameter("require_tcp_for_taught_posx", True)
        self.declare_parameter("allow_tcp_set_failure", False)
        self.declare_parameter("max_repeats_per_dispenser", 3)
        self.declare_parameter("default_amount", 1)
        self.declare_parameter("joint_velocity", 10.0)
        self.declare_parameter("joint_acceleration", 10.0)
        self.declare_parameter("line_velocity", 15.0)
        self.declare_parameter("line_acceleration", 25.0)

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

        self._queue: deque[tuple[dict[str, object], DispenserPressRequest]] = deque()
        self._condition = threading.Condition()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            "Voice dispenser executor ready: "
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

        requests = requests_from_decision(
            decision,
            max_repeats_per_dispenser=int(self.get_parameter("max_repeats_per_dispenser").value),
            default_amount=int(self.get_parameter("default_amount").value),
        )
        if not requests:
            self._publish_status("blocked", reason="no_executable_dispenser_requests", decision=decision)
            return

        with self._condition:
            for request in requests:
                self._queue.append((decision, request))
            self._condition.notify()

        self._publish_status(
            "queued",
            recipe_id=decision.get("recipe_id"),
            command_count=len(requests),
            targets=[request.target_dispenser for request in requests],
        )

    def _worker_loop(self) -> None:
        while True:
            with self._condition:
                while not self._queue:
                    self._condition.wait()
                decision, request = self._queue.popleft()
            self._execute_request(decision, request)

    def _execute_request(self, decision: dict[str, object], request: DispenserPressRequest) -> None:
        command = build_dispenser_launch_command(
            request,
            launch_file=str(self.get_parameter("dispenser_launch_file").value),
            service_prefix=str(self.get_parameter("service_prefix").value),
            tcp_name=str(self.get_parameter("tcp_name").value),
            restore_tcp_after_run=bool(self.get_parameter("restore_tcp_after_run").value),
            require_tcp_for_taught_posx=bool(self.get_parameter("require_tcp_for_taught_posx").value),
            allow_tcp_set_failure=bool(self.get_parameter("allow_tcp_set_failure").value),
            joint_velocity=float(self.get_parameter("joint_velocity").value),
            joint_acceleration=float(self.get_parameter("joint_acceleration").value),
            line_velocity=float(self.get_parameter("line_velocity").value),
            line_acceleration=float(self.get_parameter("line_acceleration").value),
        )
        self._publish_status(
            "starting",
            recipe_id=decision.get("recipe_id"),
            target_dispenser=request.target_dispenser,
            repeat_index=request.repeat_index,
            repeat_total=request.repeat_total,
            command=command,
            hardware_enabled=bool(self.get_parameter("enable_hardware_execution").value),
        )

        if not bool(self.get_parameter("enable_hardware_execution").value):
            self._publish_status(
                "dry_run",
                target_dispenser=request.target_dispenser,
                repeat_index=request.repeat_index,
                repeat_total=request.repeat_total,
                command=command,
            )
            return

        if shutil.which("ros2") is None:
            self._publish_status("failed", target_dispenser=request.target_dispenser, reason="ros2_not_found")
            self._abort_pending_requests("ros2_not_found")
            return

        try:
            completed = subprocess.run(command, check=False)
        except OSError as exc:
            self._publish_status(
                "failed",
                target_dispenser=request.target_dispenser,
                reason="launch_failed_to_start",
                error=str(exc),
            )
            self._abort_pending_requests("launch_failed_to_start")
            return

        if completed.returncode == 0:
            self._publish_status("completed", target_dispenser=request.target_dispenser)
        else:
            self._publish_status(
                "failed",
                target_dispenser=request.target_dispenser,
                returncode=completed.returncode,
            )
            self._abort_pending_requests("dispenser_launch_failed")

    def _publish_status(self, status: str, **fields: object) -> None:
        msg = String()
        msg.data = json.dumps({"status": status, **fields}, ensure_ascii=False)
        self._status_pub.publish(msg)
        if status in {"blocked", "failed"}:
            self.get_logger().warn(msg.data)
        else:
            self.get_logger().info(msg.data)

    def _abort_pending_requests(self, reason: str) -> None:
        with self._condition:
            dropped = len(self._queue)
            self._queue.clear()
        if dropped:
            self._publish_status("aborted", reason=reason, dropped_requests=dropped)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("rclpy is required to run voice_dispenser_executor_node")
    rclpy.init(args=args)
    node = VoiceDispenserExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
