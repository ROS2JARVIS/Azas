#!/usr/bin/env python3
"""Static guard for RG2 reconnect and stronger close/grasp commands in the panel."""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PANEL = ROOT / "tools" / "run" / "robot_pipeline_control_server.py"
PICK = ROOT / "tools" / "run" / "pick_from_measured_dispenser_front_hold.py"
RECIPE = ROOT / "tools" / "run" / "run_measured_dispenser_recipe_sequence.py"
CLOSE = ROOT / "tools" / "run" / "rg2_full_close_verify.sh"


def require(path: Path, needle: str) -> None:
    text = path.read_text(encoding="utf-8")
    if needle not in text:
        raise SystemExit(f"[FAIL] missing in {path.relative_to(ROOT)}: {needle}")


def main() -> int:
    require(PANEL, "def cleanup_rg2_stack")
    require(PANEL, "RG2_STACK_PATTERNS")
    require(PANEL, "elif step.key == \"connect_gripper\":")
    require(PANEL, "cleanup_rg2_stack()")
    require(PANEL, "rg2_trigger.launch.py")
    require(PANEL, "force:=300")
    require(PANEL, "{command: 'set_width', width_m: 0.075, force_n: 25.0}")
    require(PANEL, "-p gripper_close_force:=30.0")
    require(PANEL, "--gripper-force-n 25.0")
    require(PICK, "default=25.0")
    require(PICK, "command=\"set_width\"")
    require(RECIPE, "-p gripper_close_force:=30.0")
    require(RECIPE, "default=25.0")
    require(CLOSE, "RG2_CLOSE_FORCE_N:-30.0")
    require(ROOT / "tools" / "run" / "rg2_full_open_verify.sh", "RG2_OPEN_FORCE_N:-25.0")
    require(ROOT / "tools" / "run" / "rg2_full_open_verify.sh", "rg2_set_width_verify.py")
    require(CLOSE, "rg2_set_width_verify.py")
    print("[PASS] panel RG2 reconnect cleanup and stronger gripper force guards are present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
