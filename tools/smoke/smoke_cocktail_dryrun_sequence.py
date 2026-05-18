#!/usr/bin/env python3
import json
import sys
import time

import rclpy
from azas_interfaces.msg import CupDetection
from rclpy.node import Node
from std_msgs.msg import String


class CocktailDryRunSmoke(Node):
    def __init__(self):
        super().__init__("cocktail_dryrun_smoke")
        self._detections = self.create_publisher(CupDetection, "/azas/cup_detection", 10)
        self._decisions = self.create_publisher(String, "/azas/voice/recipe_decision", 10)
        self._statuses: list[dict[str, object]] = []
        self._plans: list[dict[str, object]] = []
        self.create_subscription(String, "/azas/cocktail/status", self._on_status, 10)
        self.create_subscription(String, "/azas/cocktail/task_plan", self._on_plan, 10)

    def _on_status(self, msg: String) -> None:
        try:
            self._statuses.append(json.loads(msg.data))
        except json.JSONDecodeError:
            self._statuses.append({"status": "invalid_json", "raw": msg.data})

    def _on_plan(self, msg: String) -> None:
        try:
            self._plans.append(json.loads(msg.data))
        except json.JSONDecodeError:
            self._plans.append({"status": "invalid_json", "raw": msg.data})

    def publish_detection(self, status: str) -> None:
        msg = CupDetection()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.status = status
        msg.source = "smoke"
        msg.confidence = 0.99
        self._detections.publish(msg)

    def publish_decision(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "valid": True,
                "utterance": "노랑 빨강 칵테일",
                "normalized": "노랑빨강칵테일",
                "intent": "make_cocktail",
                "recipe_id": "custom_color_selection",
                "dispenser_ids": ["yellow", "red"],
                "confirmation": "smoke",
                "error": None,
            },
            ensure_ascii=False,
        )
        self._decisions.publish(msg)

    def saw_complete(self) -> bool:
        return any(item.get("status") == "dry_run_complete" for item in self._statuses)

    def saw_blocked(self) -> bool:
        return any(item.get("status") == "blocked" for item in self._statuses)

    def latest_plan_phases(self) -> set[str]:
        if not self._plans:
            return set()
        steps = self._plans[-1].get("steps", [])
        if not isinstance(steps, list):
            return set()
        return {
            str(step.get("phase"))
            for step in steps
            if isinstance(step, dict) and step.get("phase")
        }


def main() -> int:
    rclpy.init()
    node = CocktailDryRunSmoke()
    deadline = time.monotonic() + 8.0

    # Let discovery connect before publishing the one-shot inputs.
    while time.monotonic() < deadline and node.count_publishers("/azas/cocktail/status") == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    for _ in range(5):
        # The cocktail dry-run planner consumes symbolic cup/lid presence only.
        # Motion-facing cup poses are produced by the perception bridge from
        # live statuses that start with "detected:upright"; this smoke does not
        # publish /jarvis/tumbler_dispenser/tumbler_pose or call any gripper.
        node.publish_detection("detected:cup bbox=100x100 depth_raw=300.0")
        node.publish_detection("detected:lid bbox=80x80 depth_raw=260.0")
        rclpy.spin_once(node, timeout_sec=0.1)

    node.publish_decision()
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.saw_complete():
            required_phases = {
                "VERIFY_CALIBRATION",
                "TRANSFORM_CUP_TO_BASE",
                "PICK_CUP",
                "ALIGN_CUP_UNDER_DISPENSER",
                "PRESS_DISPENSER",
                "PLACE_AND_PRESS_LID",
                "SHAKE_CUP",
            }
            missing = sorted(required_phases - node.latest_plan_phases())
            if missing:
                print("[FAIL] cocktail plan is missing required phases: " + ", ".join(missing))
                for item in node._plans:
                    print(json.dumps(item, ensure_ascii=False))
                node.destroy_node()
                rclpy.shutdown()
                return 1
            print("[PASS] cocktail dry-run sequence reached dry_run_complete")
            node.destroy_node()
            rclpy.shutdown()
            return 0
        if node.saw_blocked():
            print("[FAIL] cocktail dry-run sequence blocked")
            for item in node._statuses:
                print(json.dumps(item, ensure_ascii=False))
            node.destroy_node()
            rclpy.shutdown()
            return 1

    print("[FAIL] timed out waiting for dry_run_complete")
    for item in node._statuses:
        print(json.dumps(item, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    sys.exit(main())
