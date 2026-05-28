#!/usr/bin/env python3
"""Regression check for dispenser press joint_1 clearance before HOME moves."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NODE = ROOT / "src" / "azas_dispenser" / "azas_dispenser" / "dispenser_press_node.py"
PANEL = ROOT / "tools" / "run" / "robot_pipeline_control_server.py"
RECIPE = ROOT / "tools" / "run" / "run_measured_dispenser_recipe_sequence.py"


def require(path: Path, token: str) -> bool:
    text = path.read_text(encoding="utf-8")
    if token not in text:
        print(f"[FAIL] {path.relative_to(ROOT)} missing token: {token}")
        return False
    return True


def main() -> int:
    checks = [
        (NODE, 'get_param(self.node, "joint1_clearance_before_home", True)'),
        (NODE, 'get_param(self.node, "joint1_clearance_return_home", True)'),
        (NODE, 'get_param(self.node, "joint1_clearance_offset_deg", 12.0)'),
        (NODE, "GetCurrentPosj"),
        (NODE, 'service_name(self.service_prefix, "aux_control/get_current_posj")'),
        (NODE, "def current_joints_deg(self, label):"),
        (NODE, 'def movej_joint1_clearance(self, label):'),
        (NODE, "현재 관절 자세를 유지한 채 joint_1만"),
        (NODE, "j2~j6 유지"),
        (NODE, 'pre-HOME joint_1 + clearance'),
        (NODE, 'return pre-HOME joint_1 + clearance'),
        (PANEL, '-p joint1_clearance_before_home:=true '),
        (PANEL, '-p joint1_clearance_return_home:=true '),
        (PANEL, '-p joint1_clearance_offset_deg:=12.0 '),
        (RECIPE, '-p joint1_clearance_before_home:=true '),
        (RECIPE, '-p joint1_clearance_return_home:=true '),
    ]
    for path, token in checks:
        if not require(path, token):
            return 1
    print("[PASS] dispenser press joint_1 clearance is wired")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
