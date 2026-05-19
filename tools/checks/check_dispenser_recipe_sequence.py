#!/usr/bin/env python3
"""Static regression check for the measured dispenser recipe sequence panel step."""

from __future__ import annotations

import importlib.util
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PANEL_PATH = ROOT / "tools" / "run" / "robot_pipeline_control_server.py"
RECIPE_SCRIPT = ROOT / "tools" / "run" / "run_measured_dispenser_recipe_sequence.py"


def load_panel_module():
    spec = importlib.util.spec_from_file_location("robot_pipeline_control_server", PANEL_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load {PANEL_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    if not RECIPE_SCRIPT.is_file():
        print(f"[FAIL] missing recipe script: {RECIPE_SCRIPT}")
        return 1

    panel = load_panel_module()
    steps = {step.key: step for step in panel.STEPS}
    step = steps.get("run_dispenser_recipe_sequence")
    if step is None or not step.implemented or not step.real_motion:
        print("[FAIL] panel recipe sequence step is not real/implemented")
        return 1

    services = panel.required_services_for_step(step, "dsr01")
    required = {
        "/jarvis/rg2/set_width",
        "/dsr01/motion/move_joint",
        "/dsr01/motion/move_line",
        "/dsr01/motion/move_wait",
        "/dsr01/motion/ikin",
        "/dsr01/motion/check_motion",
        "/dsr01/system/get_robot_state",
        "/dsr01/tcp/get_current_tcp",
        "/dsr01/tcp/set_current_tcp",
        "/dsr01/aux_control/get_current_posx",
    }
    missing = sorted(required.difference(services))
    if missing:
        print(f"[FAIL] recipe sequence missing required service gates: {missing}")
        return 1

    command = panel.command_for(
        step,
        {
            "service_prefix": "dsr01",
            "recipe_dispenser_ids": "1,3,2",
            "dispenser_tcp_name": "GripperDA_v1_jarvis",
            "armed": True,
        },
    )
    checks = [
        "run_measured_dispenser_recipe_sequence.py",
        "--dispenser-ids 1,3,2",
        "--dispenser-tcp-name GripperDA_v1_jarvis",
        "ENABLE_MEASURED_DISPENSER_RECIPE_SEQUENCE",
    ]
    for expected in checks:
        if expected not in command:
            print(f"[FAIL] recipe command missing: {expected}")
            print(command)
            return 1

    dry = subprocess.run(
        [sys.executable, str(RECIPE_SCRIPT), "--dispenser-ids", "1,3,2", "--service-prefix", "dsr01"],
        cwd=str(ROOT),
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=10,
    )
    if dry.returncode != 0:
        print("[FAIL] recipe sequence dry-run failed")
        print(dry.stdout)
        return 1
    for expected in [
        "[DRY-RUN]",
        "dispenser_ids=1,3,2",
        "source=existing measured front_hold poses and taught dispenser press poses",
        "move/release -> press -> re-grasp/lift",
    ]:
        if expected not in dry.stdout:
            print(f"[FAIL] recipe dry-run missing: {expected}")
            print(dry.stdout)
            return 1

    print("[PASS] measured dispenser recipe sequence panel step is wired")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
