#!/usr/bin/env python3
"""Reject real-motion configs that drift from the currently launched geometry.

The real robot runners still pass fixed launch parameters into the Jarvis motion
nodes. Until those launch files are generated directly from calibration data,
this gate prevents a dangerous state where calibration.yaml is measured but the
robot executes a different hard-coded outlet or press pose.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from typing import Iterable, Sequence

import yaml


CALIBRATION_FILE = Path(
    os.environ.get(
        "CALIBRATION_FILE",
        "/home/ssu/Azas/src/azas_bringup/config/calibration.yaml",
    )
)
TOLERANCE_M = float(os.environ.get("DISPENSER_GEOMETRY_TOLERANCE_M", "0.003"))

EXPECTED_OUTLETS = {
    "1": [0.43, 0.18, 0.392],
    "2": [0.43, 0.08, 0.392],
    "3": [0.43, -0.02, 0.392],
    "4": [0.43, -0.12, 0.392],
}

# Press stage runs at outlet + press_x_extension and outlet_z - press_depth.
EXPECTED_PRESS_DOWN = {
    "1": [0.51, 0.18, 0.367],
    "2": [0.51, 0.08, 0.367],
    "3": [0.51, -0.02, 0.367],
    "4": [0.51, -0.12, 0.367],
}


def fail(message: str) -> int:
    print(f"[FAIL] {message}")
    return 1


def close_enough(actual: Sequence[float], expected: Sequence[float]) -> bool:
    return all(math.isclose(float(a), float(e), abs_tol=TOLERANCE_M) for a, e in zip(actual, expected))


def is_xyz(value) -> bool:
    return isinstance(value, list) and len(value) == 3 and all(isinstance(item, (int, float)) for item in value)


def format_xyz(value: Iterable[float]) -> str:
    return "[" + ", ".join(f"{float(item):.3f}" for item in value) + "]"


def main() -> int:
    if not CALIBRATION_FILE.is_file():
        return fail(f"calibration file missing: {CALIBRATION_FILE}")

    data = yaml.safe_load(CALIBRATION_FILE.read_text(encoding="utf-8")) or {}
    outlets = data.get("dispenser_outlets")
    if not isinstance(outlets, dict):
        return fail("dispenser_outlets mapping is missing")

    failures = 0
    print("[Azas] Measured dispenser geometry consistency gate")
    print(f"[Azas] calibration={CALIBRATION_FILE}")
    print(f"[Azas] tolerance_m={TOLERANCE_M}")
    for dispenser_id, expected_outlet in EXPECTED_OUTLETS.items():
        block = outlets.get(dispenser_id)
        if not isinstance(block, dict):
            failures += fail(f"dispenser {dispenser_id} block missing")
            continue

        outlet_xyz = block.get("outlet_pose_xyz_m")
        if not is_xyz(outlet_xyz):
            failures += fail(f"dispenser {dispenser_id} outlet_pose_xyz_m is not a measured XYZ list")
        elif close_enough(outlet_xyz, expected_outlet):
            print(f"[OK] dispenser {dispenser_id} outlet matches launched geometry {format_xyz(expected_outlet)}")
        else:
            failures += fail(
                "dispenser "
                f"{dispenser_id} measured outlet {format_xyz(outlet_xyz)} does not match "
                f"launched geometry {format_xyz(expected_outlet)}"
            )

        expected_press = EXPECTED_PRESS_DOWN[dispenser_id]
        press_xyz = block.get("press_pose_xyz_m")
        if not is_xyz(press_xyz):
            failures += fail(f"dispenser {dispenser_id} press_pose_xyz_m is not a measured XYZ list")
        elif close_enough(press_xyz, expected_press):
            print(f"[OK] dispenser {dispenser_id} press pose matches launched geometry {format_xyz(expected_press)}")
        else:
            failures += fail(
                "dispenser "
                f"{dispenser_id} measured press {format_xyz(press_xyz)} does not match "
                f"launched geometry {format_xyz(expected_press)}"
            )

    if failures:
        print(
            "[Azas] Fix by updating the Jarvis launch geometry from measured calibration, "
            "or by remeasuring calibration to match the current cell geometry."
        )
        return 1
    print("[PASS] measured dispenser geometry matches the currently launched real-motion geometry")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
