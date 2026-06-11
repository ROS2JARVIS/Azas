#!/usr/bin/env python3
"""Static guard: the hand detection node must stay perception-only (no motion)."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NODE = ROOT / "tools" / "perception" / "human_hand_detection_node.py"

FORBIDDEN = (
    "MoveLine",
    "MoveJoint",
    "MoveWait",
    "motion/move",
    "SetGripper",
    "rg2",
    "create_client",
)
REQUIRED = (
    "/azas/human_hand_detection",
    "no_motion_hri_perception_only",
    "PointStamped",
)


def main() -> int:
    text = NODE.read_text(encoding="utf-8")
    failures: list[str] = []
    for needle in FORBIDDEN:
        if needle in text:
            failures.append(f"forbidden motion-related token present: {needle!r}")
    for needle in REQUIRED:
        if needle not in text:
            failures.append(f"required token missing: {needle!r}")
    if failures:
        for failure in failures:
            print(f"[FAIL] {NODE.relative_to(ROOT)}: {failure}")
        return 1
    print("[PASS] human hand detection node is perception-only (no motion clients, required topics present)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
