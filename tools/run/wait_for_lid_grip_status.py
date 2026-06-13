#!/usr/bin/env python3
"""Wait for the supervised lid-grip/twist sequence to report success.

The lid-grip launch keeps its OpenCV/perception nodes alive after a successful
`p`-triggered sequence.  Panel shell chaining therefore needs a small ROS topic
gate that exits as soon as the planner publishes its terminal success/failure
status instead of waiting for the operator to close the preview window.
"""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Wait for /jarvis/lid_gripper/status success/failure JSON."
    )
    parser.add_argument("--topic", default="/jarvis/lid_gripper/status")
    parser.add_argument("--timeout-sec", type=float, default=900.0)
    parser.add_argument(
        "--success-status",
        action="append",
        default=["motion_sequence_requested"],
        help="status value that means the lid close sequence completed successfully",
    )
    parser.add_argument(
        "--failure-status",
        action="append",
        default=["failed"],
        help="status value that means the lid close sequence failed",
    )
    parser.add_argument(
        "--ignore-pretrigger-failures",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "ignore retryable failed statuses before an accepted lid trigger starts; "
            "this keeps early p-key/no-pose events from aborting the shake chain"
        ),
    )
    return parser.parse_args()


class LidGripStatusWaiter(Node):
    def __init__(
        self,
        topic: str,
        success_statuses: set[str],
        failure_statuses: set[str],
        *,
        ignore_pretrigger_failures: bool,
    ):
        super().__init__("azas_wait_for_lid_grip_status")
        self._success_statuses = success_statuses
        self._failure_statuses = failure_statuses
        self._ignore_pretrigger_failures = ignore_pretrigger_failures
        self._sequence_started = False
        self.result_code: int | None = None
        self.result_text = ""
        self.create_subscription(String, topic, self._on_status, 10)
        print(f"[Azas] waiting for lid grip status on {topic}", flush=True)

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"status": msg.data}
        status = str(payload.get("status", "")).strip()
        if not status:
            return
        if self._marks_sequence_started(status, payload):
            self._sequence_started = True
        if status in self._success_statuses:
            print(f"[Azas] lid_grip_status={status} payload={payload}", flush=True)
            self.result_code = 0
            self.result_text = f"success status observed: {status}"
        elif status in self._failure_statuses:
            if self._should_ignore_failure(payload):
                print(f"[Azas] lid_grip_status_ignored={status} payload={payload}", flush=True)
                return
            print(f"[Azas] lid_grip_status={status} payload={payload}", flush=True)
            self.result_code = 1
            self.result_text = f"failure status observed: {status}"
        else:
            print(f"[Azas] lid_grip_status={status} payload={payload}", flush=True)

    @staticmethod
    def _marks_sequence_started(status: str, payload: dict) -> bool:
        if status == "trigger_received":
            return True
        if str(payload.get("request_source") or "") == "p_key":
            return True
        if payload.get("real_motion") is True:
            return True
        if payload.get("motion_allowed") is True:
            return True
        return False

    def _should_ignore_failure(self, payload: dict) -> bool:
        if not self._ignore_pretrigger_failures:
            return False
        if self._sequence_started:
            return False
        if payload.get("real_motion") is True:
            return False
        return True


def main() -> int:
    args = parse_args()
    success_statuses = {str(item) for item in args.success_status}
    failure_statuses = {str(item) for item in args.failure_status}
    timeout_sec = max(float(args.timeout_sec), 0.1)

    rclpy.init(args=None)
    node = LidGripStatusWaiter(
        args.topic,
        success_statuses,
        failure_statuses,
        ignore_pretrigger_failures=bool(args.ignore_pretrigger_failures),
    )
    deadline = time.monotonic() + timeout_sec
    try:
        while rclpy.ok() and node.result_code is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.result_code is not None:
            print(f"[Azas] {node.result_text}", flush=True)
            return node.result_code
        print(f"[Azas][FAIL] lid grip status wait timed out after {timeout_sec:.1f}s", flush=True)
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
