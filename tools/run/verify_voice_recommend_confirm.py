#!/usr/bin/env python3
"""Verify the recommend -> "응" -> execute chain makes EXACTLY the recommended recipe.

Flow checked:
  1. publish a recommendation utterance ("추천해줘") to the STT topic
  2. capture what the mapper recommended on /azas/voice/recipe_decision
  3. publish a confirm utterance ("응") to the STT topic
  4. capture /azas/voice/confirmed_recipe_decision
  5. assert recipe_id / dispenser_ids / dispenser_amounts are carried through UNCHANGED
  6. (if the executor is running) check the queued targets match the recommendation

Prereq: the voice stack must be running (recipe_mapper + conversation_manager),
e.g. via tools/run/run_voice_dispenser_sim_m0609.sh.

Usage:
    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 tools/run/verify_voice_recommend_confirm.py

Env overrides:
    STT_TOPIC (default /stt_result)
    RECOMMEND_UTTERANCE (default "추천해줘")
    CONFIRM_UTTERANCE   (default "응")
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

STT_TOPIC = os.environ.get("STT_TOPIC", "/stt_result")
RECOMMEND = os.environ.get("RECOMMEND_UTTERANCE", "추천해줘")
CONFIRM = os.environ.get("CONFIRM_UTTERANCE", "응")
MAX_REPEATS = 3


def dispenser_ids(decision):
    return [str(x).strip() for x in (decision.get("dispenser_ids") or [])]


def expected_targets(decision):
    """Mirror requests_from_decision: expand dispenser_ids by clamped amounts."""
    amounts = decision.get("dispenser_amounts") or {}
    out = []
    for did in dispenser_ids(decision):
        try:
            amount = int(amounts.get(did, 1))
        except (TypeError, ValueError):
            amount = 1
        amount = max(0, min(amount, MAX_REPEATS))
        out += [did] * amount
    return out


class Verifier(Node):
    def __init__(self):
        super().__init__("verify_voice_recommend_confirm")
        self.recommended = None
        self.confirmed = None
        self.status_msgs = []
        self.create_subscription(String, "/azas/voice/recipe_decision", self._on_recipe, 10)
        self.create_subscription(
            String, "/azas/voice/confirmed_recipe_decision", self._on_confirmed, 10
        )
        self.create_subscription(
            String, "/azas/voice/dispenser_execution_status", self._on_status, 10
        )
        self.pub = self.create_publisher(String, STT_TOPIC, 10)

    def _on_recipe(self, msg):
        try:
            decision = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        # Only the recommendation (make_cocktail); ignore the "confirm" echo.
        if decision.get("intent") == "make_cocktail":
            self.recommended = decision

    def _on_confirmed(self, msg):
        try:
            self.confirmed = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_status(self, msg):
        try:
            self.status_msgs.append(json.loads(msg.data))
        except json.JSONDecodeError:
            pass


def spin_until(node, predicate, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def main():
    rclpy.init()
    node = Verifier()
    failures = []

    print(f"[verify] waiting for a subscriber on {STT_TOPIC} (recipe mapper)...")
    if not spin_until(node, lambda: node.pub.get_subscription_count() > 0, 15.0):
        print(f"[verify] FAIL: nothing subscribed to {STT_TOPIC}. Is the voice stack running?")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(f"[verify] (1) recommend  -> {RECOMMEND!r}")
    node.pub.publish(String(data=RECOMMEND))
    if not spin_until(node, lambda: node.recommended is not None, 10.0):
        print("[verify] FAIL: no recommendation on /azas/voice/recipe_decision.")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    rec = node.recommended
    print(
        f"[verify]     recommended: recipe_id={rec.get('recipe_id')} "
        f"dispenser_ids={dispenser_ids(rec)} amounts={rec.get('dispenser_amounts')}"
    )

    time.sleep(1.0)
    print(f"[verify] (2) confirm    -> {CONFIRM!r}")
    node.pub.publish(String(data=CONFIRM))
    if not spin_until(node, lambda: node.confirmed is not None, 10.0):
        print("[verify] FAIL: no /azas/voice/confirmed_recipe_decision (confirm unrecognized or no pending).")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    conf = node.confirmed
    print(
        f"[verify]     confirmed:   recipe_id={conf.get('recipe_id')} "
        f"dispenser_ids={dispenser_ids(conf)} amounts={conf.get('dispenser_amounts')} "
        f"confirmed={conf.get('confirmed')}"
    )

    print("[verify] (3) compare recommended vs confirmed (must be identical):")
    if conf.get("recipe_id") != rec.get("recipe_id"):
        failures.append("recipe_id changed")
    if dispenser_ids(conf) != dispenser_ids(rec):
        failures.append("dispenser_ids changed")
    if (conf.get("dispenser_amounts") or None) != (rec.get("dispenser_amounts") or None):
        failures.append("dispenser_amounts changed")
    if conf.get("confirmed") is not True:
        failures.append("confirmed flag not true")
    for f in failures:
        print(f"[verify]     ✗ {f}")
    if not failures:
        print("[verify]     ✓ recipe_id, dispenser_ids, dispenser_amounts all preserved")

    print("[verify] (4) executor queued targets vs recommendation:")
    spin_until(node, lambda: any(s.get("status") == "queued" for s in node.status_msgs), 8.0)
    queued = next((s for s in node.status_msgs if s.get("status") == "queued"), None)
    if queued is None:
        print("[verify]     (no 'queued' status — executor disabled, or all amounts 0)")
    else:
        want = expected_targets(conf)
        got = queued.get("targets")
        print(f"[verify]     queued targets={got}  expected={want}")
        if got != want:
            failures.append("executor targets != recommended expansion")
            print("[verify]     ✗ executor targets differ from recommendation")
        else:
            print("[verify]     ✓ executor will press exactly the recommended dispensers")

    print()
    if failures:
        print("[verify] RESULT: FAIL ✗ -", "; ".join(failures))
    else:
        print("[verify] RESULT: PASS ✅ — '응' makes exactly what was recommended")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
