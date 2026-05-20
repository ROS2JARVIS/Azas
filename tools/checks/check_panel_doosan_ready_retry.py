#!/usr/bin/env python3
"""Static regression check for panel retrying transient Doosan readiness calls."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PANEL = ROOT / "tools" / "run" / "robot_pipeline_control_server.py"


def main() -> int:
    source = PANEL.read_text(encoding="utf-8")
    required = [
        "get_robot_state failed after retries",
        "check_motion failed after retries",
        "for attempt in range(1, 4):",
        "DDS/service discovery can briefly drop a direct service call",
        "wait_for_required_services(",
    ]
    for token in required:
        if token not in source:
            print(f"[FAIL] panel readiness retry missing token: {token}")
            return 1
    print("[PASS] panel Doosan readiness retry is wired")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
