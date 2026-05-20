#!/usr/bin/env python3
"""Static regression for measured tumbler collision scene wiring."""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NODE = ROOT / "src" / "azas_motion" / "azas_motion" / "tumbler_collision_scene_node.py"
SETUP = ROOT / "src" / "azas_motion" / "setup.py"
PANEL = ROOT / "tools" / "run" / "robot_pipeline_control_server.py"
RECIPE = ROOT / "tools" / "run" / "run_measured_dispenser_recipe_sequence.py"
DOC = ROOT / "docs" / "tumbler_dispenser_models.md"
CAL = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"


def require(path: Path, token: str) -> None:
    text = path.read_text(encoding="utf-8")
    if token not in text:
        raise SystemExit(f"[FAIL] {path.relative_to(ROOT)} missing token: {token}")


def main() -> int:
    require(DOC, "Tumbler: 75 mm diameter, 170 mm lidded height, 140 mm lidless body height")
    require(CAL, "top_center_estimated_xyz_m")
    require(NODE, "TUMBLER_DIAMETER_M = 0.075")
    require(NODE, "TUMBLER_LIDDED_HEIGHT_M = 0.170")
    require(NODE, "TUMBLER_LIDLESS_HEIGHT_M = 0.140")
    require(NODE, "/jarvis/tumbler_dispenser/tumbler_pose")
    require(NODE, "front_hold_poses")
    require(NODE, "top_center_estimated_xyz_m")
    require(NODE, "AttachedCollisionObject")
    require(NODE, "/collision_object")
    require(NODE, "/attached_collision_object")
    require(NODE, "action must be one of publish_detected, add_dispenser")
    require(SETUP, "tumbler_collision_scene_node = azas_motion.tumbler_collision_scene_node:main")
    require(PANEL, "start_collision_scene")
    require(PANEL, "measured_dispenser_collision_scene_node")
    require(PANEL, "tumbler_collision_scene_node")
    require(PANEL, "add_dispenser")
    require(PANEL, "remove_world")
    require(PANEL, "attach")
    require(RECIPE, "mark tumbler world object at dispenser")
    require(RECIPE, "attach carried tumbler object")
    print("[PASS] measured tumbler collision scene is wired from documented dimensions and measured configs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
