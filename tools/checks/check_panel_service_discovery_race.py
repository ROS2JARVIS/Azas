#!/usr/bin/env python3
"""Regression check for panel service-discovery false blocks."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
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
    with tempfile.TemporaryDirectory(prefix="azas_panel_commands_") as temp_dir:
        panel.COMMAND_OVERRIDES_PATH = Path(temp_dir) / "panel_command_overrides.json"

        color_scan_pose = next(step for step in panel.STEPS if step.key == "move_to_color_scan_pose")
        color_scan_command = panel.command_for(color_scan_pose, {"service_prefix": "dsr01"})
        if "--service-prefix dsr01" not in color_scan_command:
            print("[FAIL] color scan pose command does not target namespaced Doosan MoveJoint service")
            print(color_scan_command)
            return 1
        for expected in ("--j1 0", "--j2 10", "--j3 32", "--j4 0", "--j5 100", "--j6 90"):
            if expected not in color_scan_command:
                print("[FAIL] color scan pose command does not use the saved camera-view joint target")
                print(color_scan_command)
                return 1
        color_scan_required = panel.required_services_for_step(color_scan_pose, "dsr01")
        if "/dsr01/motion/move_joint" not in color_scan_required:
            print("[FAIL] color scan pose preflight does not require namespaced MoveJoint service")
            print(color_scan_required)
            return 1
        color_scan_order = panel.with_collision_scene_prereq(["color_scan"])
        expected_color_scan_order = ["move_to_color_scan_pose", "start_camera", "color_scan"]
        if color_scan_order != expected_color_scan_order:
            print("[FAIL] color_scan does not auto-run camera pose and RealSense prerequisites")
            print(color_scan_order)
            return 1

        custom_command = "echo custom panel command"
        panel.save_command_override("move_to_color_scan_pose", custom_command)
        if panel.command_for(color_scan_pose, {"service_prefix": "dsr01"}) != custom_command:
            print("[FAIL] saved panel command override was not used by command_for")
            return 1
        panel.save_command_override("move_to_color_scan_pose", "")
        if panel.command_for(color_scan_pose, {"service_prefix": "dsr01"}) == custom_command:
            print("[FAIL] clearing panel command override did not restore generated command")
            return 1

        side_grip = next(step for step in panel.STEPS if step.key == "side_grip")
        side_grip_command = panel.command_for(side_grip, {"service_prefix": "dsr01"})
        expected_package_source = "install/dsr_practice/share/dsr_practice/package.bash"
        if expected_package_source not in side_grip_command:
            print("[FAIL] side_grip command does not force the Azas dsr_practice overlay")
            print(side_grip_command)
            return 1

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
