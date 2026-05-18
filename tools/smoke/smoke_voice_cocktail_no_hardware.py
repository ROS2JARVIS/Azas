#!/usr/bin/env python3
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceCocktailNoHardwareSmoke(Node):
    def __init__(self):
        super().__init__("voice_cocktail_no_hardware_smoke")
        self._stt_pub = self.create_publisher(String, "/stt_result", 10)
        self._decisions: list[dict[str, object]] = []
        self._statuses: list[dict[str, object]] = []
        self._plans: list[dict[str, object]] = []
        self.create_subscription(String, "/azas/voice/recipe_decision", self._on_decision, 10)
        self.create_subscription(String, "/azas/cocktail/status", self._on_status, 10)
        self.create_subscription(String, "/azas/cocktail/task_plan", self._on_plan, 10)

    def _append_json(self, target: list[dict[str, object]], msg: String) -> None:
        try:
            target.append(json.loads(msg.data))
        except json.JSONDecodeError:
            target.append({"status": "invalid_json", "raw": msg.data})

    def _on_decision(self, msg: String) -> None:
        self._append_json(self._decisions, msg)

    def _on_status(self, msg: String) -> None:
        self._append_json(self._statuses, msg)

    def _on_plan(self, msg: String) -> None:
        self._append_json(self._plans, msg)

    def publish_stt(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._stt_pub.publish(msg)

    def latest_decision(self) -> dict[str, object]:
        return self._decisions[-1] if self._decisions else {}

    def saw_complete(self) -> bool:
        return any(item.get("status") == "dry_run_complete" for item in self._statuses)

    def saw_blocked(self) -> bool:
        return any(item.get("status") == "blocked" for item in self._statuses)


def main() -> int:
    text = "오늘 기분이 우울한데 칵테일 추천해줘"
    if len(sys.argv) > 1:
        text = " ".join(sys.argv[1:])

    rclpy.init()
    node = VoiceCocktailNoHardwareSmoke()
    deadline = time.monotonic() + 10.0

    while time.monotonic() < deadline and (
        node.count_subscribers("/stt_result") == 0
        or node.count_publishers("/azas/cocktail/status") == 0
    ):
        rclpy.spin_once(node, timeout_sec=0.1)

    for _ in range(3):
        node.publish_stt(text)
        rclpy.spin_once(node, timeout_sec=0.1)

    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.saw_complete():
            decision = node.latest_decision()
            if not decision.get("valid") or not decision.get("dispenser_ids"):
                print("[FAIL] voice decision is not executable")
                print(json.dumps(decision, ensure_ascii=False))
                node.destroy_node()
                rclpy.shutdown()
                return 1
            print("[PASS] voice text reached cocktail dry-run without camera or robot")
            print(json.dumps(decision, ensure_ascii=False))
            node.destroy_node()
            rclpy.shutdown()
            return 0
        if node.saw_blocked():
            print("[FAIL] cocktail dry-run blocked")
            for item in node._statuses:
                print(json.dumps(item, ensure_ascii=False))
            node.destroy_node()
            rclpy.shutdown()
            return 1

    print("[FAIL] timed out waiting for no-hardware voice dry-run")
    for item in node._decisions:
        print(json.dumps(item, ensure_ascii=False))
    for item in node._statuses:
        print(json.dumps(item, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    sys.exit(main())
