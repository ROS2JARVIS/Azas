#!/usr/bin/env python3
"""Recover perception failures by returning HOME, then OBSERVE.

This helper does not invent cup coordinates. It only inspects
`/azas/cup_detection` and, when the status is not actionable, delegates to the
existing legacy camera-home move:

    HOME -> wrist camera alignment -> camera_home OBSERVE

Use `--dry-run` first. Real motion requires the same explicit confirmation
phrase as `move_to_legacy_camera_home.py`.

Exit codes:
    0: upright confirmed; side-grab flow may proceed.
   10: lying confirmed; hand off to the lying-upright flow.
   20: still unknown after retries; operator/retry escalation required.
    1: HOME -> OBSERVE recovery motion failed.
    2: unsafe or invalid real-motion request.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import subprocess
import sys
import time

import rclpy


CONFIRM_PHRASE = "I_UNDERSTAND_THIS_WILL_MOVE_THE_ROBOT"
ROOT_DIR = Path(__file__).resolve().parents[2]
MOVE_CAMERA_HOME = ROOT_DIR / "tools" / "run" / "move_to_legacy_camera_home.py"
EXIT_UPRIGHT = 0
EXIT_LYING = 10
EXIT_OPERATOR_CHECK = 20
EXIT_MOTION_FAILED = 1
EXIT_INVALID_REQUEST = 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cup-detection-topic", default="/azas/cup_detection")
    parser.add_argument("--wait-sec", type=float, default=5.0)
    parser.add_argument("--max-retries", type=int, default=2)
    parser.add_argument("--settle-sec", type=float, default=1.0)
    parser.add_argument("--dry-run", action="store_true", default=True)
    parser.add_argument("--enable-real-motion", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--velocity-scale", type=float, default=0.03)
    parser.add_argument("--accel-scale", type=float, default=0.03)
    parser.add_argument("--camera-home-x", type=float, default=0.45)
    parser.add_argument("--camera-home-y", type=float, default=0.0)
    parser.add_argument("--camera-home-z", type=float, default=0.62)
    parser.add_argument("--camera-orient-joint-6-deg", type=float, default=90.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    max_retries = max(int(args.max_retries), 0)
    for attempt in range(max_retries + 1):
        status = read_cup_detection_status(args.cup_detection_topic, args.wait_sec)
        state = classify_status(status)

        print("=== CUP_DETECTION_CHECK ===")
        print(f"attempt={attempt}/{max_retries}")
        print(f"topic={args.cup_detection_topic}")
        print(f"status={status or 'NO_SAMPLE_TIMEOUT'}")
        print(f"classified_state={state}")

        if state == "upright":
            print("[OK] Cup is upright. FINAL_STATE=upright NEXT=side_grab")
            return EXIT_UPRIGHT
        if state == "lying":
            print(
                "[OK] Cup is lying. FINAL_STATE=lying NEXT=lying_upright_flow "
                "After that flow, move OBSERVE and recheck before side_grab."
            )
            return EXIT_LYING

        if attempt >= max_retries:
            break

        print("=== FAILURE_RECOVERY ===")
        print("Cup state is unknown or unavailable; moving HOME -> OBSERVE before recheck.")
        move_result = run_home_then_observe(args)
        if move_result != 0:
            print(f"[FAIL] HOME -> OBSERVE recovery failed with code={move_result}")
            return EXIT_MOTION_FAILED
        time.sleep(max(float(args.settle_sec), 0.0))

    print(
        "[FAIL] FINAL_STATE=unknown NEXT=operator_check "
        f"after {max_retries} HOME -> OBSERVE recoveries"
    )
    return EXIT_OPERATOR_CHECK


def read_cup_detection_status(topic: str, wait_sec: float) -> str:
    from azas_interfaces.msg import CupDetection

    rclpy.init()
    node = rclpy.create_node("azas_recover_home_observe_on_cup_failure")
    result = {"status": ""}

    def callback(msg: CupDetection) -> None:
        result["status"] = msg.status

    node.create_subscription(CupDetection, topic, callback, 10)
    deadline = time.monotonic() + max(wait_sec, 0.0)
    try:
        while rclpy.ok() and not result["status"] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return result["status"]


def classify_status(status: str) -> str:
    if not status:
        return "failure"
    normalized = status.strip().lower()
    orientation_match = re.search(r"\borientation=([a-z_]+)", normalized)
    orientation = orientation_match.group(1) if orientation_match else ""
    if normalized.startswith("detected:upright") or orientation == "upright":
        return "upright"
    if normalized.startswith("rejected:lying") or orientation == "lying":
        return "lying"
    if (
        "unknown_orientation" in normalized
        or orientation == "unknown"
        or "no_tumbler_detection" in normalized
        or "invalid_depth" in normalized
    ):
        return "failure"
    if normalized.startswith("rejected:") or normalized.startswith("no_"):
        return "failure"
    return "failure"


def run_home_then_observe(args: argparse.Namespace) -> int:
    command = [
        sys.executable,
        str(MOVE_CAMERA_HOME),
        "--velocity-scale",
        f"{args.velocity_scale}",
        "--accel-scale",
        f"{args.accel_scale}",
        "--camera-home-x",
        f"{args.camera_home_x}",
        "--camera-home-y",
        f"{args.camera_home_y}",
        "--camera-home-z",
        f"{args.camera_home_z}",
        "--camera-orient-joint-6-deg",
        f"{args.camera_orient_joint_6_deg}",
    ]
    if args.enable_real_motion:
        if args.confirm != CONFIRM_PHRASE:
            print(
                "[FAIL] Real motion requested without the required confirmation phrase. "
                "No motion command was sent."
            )
            return EXIT_INVALID_REQUEST
        command.extend(["--enable-real-motion", "--confirm", args.confirm])
    else:
        command.append("--dry-run")

    completed = subprocess.run(command, check=False)
    return int(completed.returncode)


if __name__ == "__main__":
    raise SystemExit(main())
