#!/usr/bin/env python3
"""Unit-style check for the YOLO bbox cup-orientation heuristic.

This script does not start ROS, camera, robot motion, MoveIt execution, or RG2.
It only checks the bbox height/width thresholds used before side-grasp pose
publication.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = REPO_ROOT / "src/azas_perception/azas_perception/yolo_tumbler_detector_node.py"


def _load_detector_module():
    spec = importlib.util.spec_from_file_location("yolo_tumbler_detector_node", NODE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load {NODE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    module = _load_detector_module()
    classify = module.YoloTumblerDetectorNode._classify_cup_orientation
    cases = [
        ("upright", 100, 130, "upright"),
        ("upright_boundary", 100, 120, "upright"),
        ("lying", 100, 70, "lying"),
        ("lying_boundary", 100, 79, "lying"),
        ("unknown", 100, 100, "unknown"),
    ]
    failures = []
    for label, width, height, expected in cases:
        actual = classify(width, height)
        print(f"{label}: bbox={width}x{height} expected={expected} actual={actual}")
        if actual != expected:
            failures.append((label, expected, actual))
    if failures:
        print(f"[FAIL] cup orientation heuristic mismatches: {failures}")
        return 1
    print("[OK] cup orientation heuristic cases passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
