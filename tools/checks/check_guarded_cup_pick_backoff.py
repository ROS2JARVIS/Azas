#!/usr/bin/env python3
"""Regression check for guarded cup side-pick stop before gripper close."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools" / "pick" / "run_supervised_real_single_cup_pick.py"
LEGACY = ROOT / "src" / "azas_perception" / "azas_perception" / "yolo_cup_pick_legacy_node.py"


def load_supervised_module():
    spec = importlib.util.spec_from_file_location("run_supervised_real_single_cup_pick", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load {SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    mod = load_supervised_module()
    candidate = {
        "approach_pose": {
            "position": {"x": 0.300, "y": 0.100, "z": 0.200},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "grasp_pose": {
            "position": {"x": 0.100, "y": 0.100, "z": 0.200},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "lift_pose": {
            "position": {"x": 0.100, "y": 0.100, "z": 0.320},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    }
    guarded = mod.guarded_grasp_pose(candidate, 0.02)
    if abs(guarded["position"]["x"] - 0.120) > 1e-9:
        print(f"[FAIL] guarded grasp x did not back off from cup center: {guarded}")
        return 1
    if abs(guarded["position"]["y"] - 0.100) > 1e-9:
        print(f"[FAIL] guarded grasp y drifted unexpectedly: {guarded}")
        return 1
    lift = mod.guarded_lift_pose(candidate, guarded)
    if lift["position"]["x"] != guarded["position"]["x"] or lift["position"]["z"] != 0.320:
        print(f"[FAIL] guarded lift does not lift from guarded XY: {lift}")
        return 1

    source = LEGACY.read_text(encoding="utf-8")
    required = [
        'declare_parameter("side_grasp_stop_backoff_m", 0.02)',
        'declare_parameter("gripper_open_settle_sec", 1.0)',
        'declare_parameter("pre_pick_joint1_clearance_deg", 12.0)',
        'move_joint1_clearance_before_side_grip',
        'guarded_grasp_xy = grasp_xy + side_vec * self.side_grasp_stop_backoff_m',
        'wait {self.gripper_open_settle_sec:.2f}s for RG2 full-open before low approach',
    ]
    for token in required:
        if token not in source:
            print(f"[FAIL] legacy YOLO pick missing guarded approach token: {token}")
            return 1

    print("[PASS] guarded cup pick backoff is wired")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
