#!/usr/bin/env python3
"""Static regression check for dispenser-front cup re-grasp panel steps."""

from __future__ import annotations

import importlib.util
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PANEL_PATH = ROOT / "tools" / "run" / "robot_pipeline_control_server.py"
REGRASP_SCRIPT = ROOT / "tools" / "run" / "pick_from_measured_dispenser_front_hold.py"


def load_panel_module():
    spec = importlib.util.spec_from_file_location("robot_pipeline_control_server", PANEL_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load {PANEL_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    if not REGRASP_SCRIPT.is_file():
        print(f"[FAIL] missing re-grasp script: {REGRASP_SCRIPT}")
        return 1

    panel = load_panel_module()
    steps = {step.key: step for step in panel.STEPS}
    payload = {"service_prefix": "dsr01", "armed": True}
    for dispenser_id in range(1, 5):
        key = f"pick_from_dispenser_{dispenser_id}"
        step = steps.get(key)
        if step is None or not step.implemented or not step.real_motion:
            print(f"[FAIL] panel step not real/implemented: {key}")
            return 1
        services = panel.required_services_for_step(step, "dsr01")
        required = {
            "/jarvis/rg2/set_width",
            "/dsr01/motion/move_joint",
            "/dsr01/motion/move_line",
            "/dsr01/motion/move_wait",
            "/dsr01/motion/ikin",
            "/dsr01/system/get_robot_state",
            "/dsr01/aux_control/get_current_posj",
            "/dsr01/aux_control/get_current_posx",
            "/dsr01/tcp/get_current_tcp",
        }
        missing = sorted(required.difference(services))
        if missing:
            print(f"[FAIL] {key} missing required service gates: {missing}")
            return 1
        command = panel.command_for(step, payload)
        if "pick_from_measured_dispenser_front_hold.py" not in command:
            print(f"[FAIL] {key} does not route to re-grasp script")
            return 1
        if f"--dispenser-id {dispenser_id}" not in command:
            print(f"[FAIL] {key} command does not preserve dispenser id")
            return 1
        if "ENABLE_PICK_FROM_MEASURED_DISPENSER_FRONT_HOLD" not in command:
            print(f"[FAIL] {key} command missing explicit real-motion confirm")
            return 1

    dry = subprocess.run(
        [sys.executable, str(REGRASP_SCRIPT), "--dispenser-id", "2", "--service-prefix", "dsr01"],
        cwd=str(ROOT),
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=10,
    )
    if dry.returncode != 0:
        print("[FAIL] re-grasp dry-run failed")
        print(dry.stdout)
        return 1
    if "source=front_hold_poses" not in dry.stdout:
        print("[FAIL] dry-run did not report measured front_hold source")
        return 1
    if "pre-grasp joint_1 clearance" not in dry.stdout:
        print("[FAIL] dry-run did not report joint_1 clearance before front-hold approach")
        return 1
    if "post-grasp lift would read current TCP" not in dry.stdout:
        print("[FAIL] dry-run did not report current-TCP-derived lift")
        return 1

    print("[PASS] dispenser front-hold re-grasp panel steps are wired")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
