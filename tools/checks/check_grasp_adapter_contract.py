#!/usr/bin/env python3
"""Validate the offline grasp detector input/output contract."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


REQUIRED_FRAME_FILES = ("rgb.png", "depth_m.npy", "camera_info.json")


def finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def validate_frame_dir(frame_dir: Path) -> list[str]:
    failures: list[str] = []
    for name in REQUIRED_FRAME_FILES:
        require((frame_dir / name).is_file(), f"missing {frame_dir / name}", failures)
    if failures:
        return failures

    depth = np.load(frame_dir / "depth_m.npy")
    require(depth.ndim == 2, "depth_m.npy must be HxW", failures)
    require(np.issubdtype(depth.dtype, np.floating), "depth_m.npy must be floating-point meters", failures)
    require(bool(np.isfinite(depth).any()), "depth_m.npy has no finite depth samples", failures)

    camera_info = json.loads((frame_dir / "camera_info.json").read_text(encoding="utf-8"))
    k = camera_info.get("k") or [item for row in camera_info.get("K", []) for item in row]
    require(len(k) == 9, "camera_info.json must contain 9-value k or 3x3 K", failures)
    if len(k) == 9:
        require(finite_number(k[0]) and float(k[0]) > 0, "camera_info fx must be positive", failures)
        require(finite_number(k[4]) and float(k[4]) > 0, "camera_info fy must be positive", failures)
        require(finite_number(k[2]), "camera_info cx must be finite", failures)
        require(finite_number(k[5]), "camera_info cy must be finite", failures)
    return failures


def validate_candidates(path: Path) -> list[str]:
    failures: list[str] = []
    payload = json.loads(path.read_text(encoding="utf-8"))
    candidates = payload.get("candidates")
    require(isinstance(candidates, list), "grasp_candidates.json must contain candidates[]", failures)
    if not isinstance(candidates, list):
        return failures
    require(payload.get("source"), "grasp_candidates.json must contain source", failures)
    for index, candidate in enumerate(candidates):
        prefix = f"candidate[{index}]"
        require(candidate.get("candidate_id") is not None, f"{prefix} missing candidate_id", failures)
        require(candidate.get("source"), f"{prefix} missing source", failures)
        require(finite_number(candidate.get("score")), f"{prefix} score must be finite", failures)
        require(finite_number(candidate.get("width_m")) and float(candidate.get("width_m")) > 0.0, f"{prefix} width_m must be positive", failures)
        pose = candidate.get("pose")
        require(isinstance(pose, dict), f"{prefix} missing pose", failures)
        if not isinstance(pose, dict):
            continue
        position = pose.get("position", {})
        orientation = pose.get("orientation", {})
        for key in ("x", "y", "z"):
            require(finite_number(position.get(key)), f"{prefix} position.{key} must be finite", failures)
        quat = [orientation.get(key) for key in ("x", "y", "z", "w")]
        require(all(finite_number(value) for value in quat), f"{prefix} quaternion must be finite", failures)
        if all(finite_number(value) for value in quat):
            norm = math.sqrt(sum(float(value) * float(value) for value in quat))
            require(norm > 1e-6, f"{prefix} quaternion norm must be nonzero", failures)
        frame_id = candidate.get("frame_id") or payload.get("frame_id") or payload.get("header", {}).get("frame_id")
        require(frame_id is not None, f"{prefix} must resolve a camera-frame frame_id", failures)
    return failures


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame-dir", default="/tmp/azas_grasp_frame")
    parser.add_argument("--candidates-json", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    failures = validate_frame_dir(Path(args.frame_dir))
    if args.candidates_json:
        failures.extend(validate_candidates(Path(args.candidates_json)))
    if failures:
        print("[FAIL] grasp adapter contract check failed")
        for failure in failures:
            print(f"- {failure}")
        return 1
    print("[OK] grasp adapter contract check passed")
    print("[Azas] contract check is offline only; no robot motion, MoveIt execute, or RG2 command was used")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
