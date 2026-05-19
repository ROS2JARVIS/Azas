#!/usr/bin/env python3
"""Regression check for panel service-discovery false blocks."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PANEL_PATH = ROOT / "tools" / "run" / "robot_pipeline_control_server.py"


def load_panel_module():
    spec = importlib.util.spec_from_file_location("robot_pipeline_control_server", PANEL_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load {PANEL_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    panel = load_panel_module()
    shake = next(step for step in panel.STEPS if step.key == "shake_closed_cup")
    required = panel.required_services_for_step(shake, "dsr01")
    calls = {"ros_service_names": 0}

    def wait_ready(required_services, *, timeout_sec=20.0, proc=None):
        if required_services != required:
            raise AssertionError("unexpected required service set")
        return True, "required services became ready after 1 check(s): " + ", ".join(required_services)

    def flaky_service_list(*, timeout_sec=2.0):
        calls["ros_service_names"] += 1
        return ["/jarvis/rg2/open", "/jarvis/rg2/close"], "/jarvis/rg2/open\n/jarvis/rg2/close\n"

    panel.wait_for_required_services = wait_ready
    panel.ros_service_names = flaky_service_list

    missing, output = panel.missing_required_services(shake, "dsr01")
    if missing:
        print("[FAIL] ready service wait was converted into a false missing list:", missing)
        return 1
    if calls["ros_service_names"] != 0:
        print("[FAIL] service list was called after a successful wait sample")
        return 1
    if "required services became ready" not in output:
        print("[FAIL] ready evidence was not preserved")
        return 1
    print("[PASS] panel trusts successful required-service wait sample")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
