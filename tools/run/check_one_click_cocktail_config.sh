#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RECIPE_DISPENSER_IDS="${RECIPE_DISPENSER_IDS:-${DISPENSER_IDS:-1x1}}"
MEASURED_CONFIG="${MEASURED_CONFIG:-${ROOT_DIR}/src/azas_bringup/config/measured_dispenser_collision.yaml}"
CALIBRATION_CONFIG="${CALIBRATION_CONFIG:-${ROOT_DIR}/src/azas_bringup/config/calibration.yaml}"

python3 - "${RECIPE_DISPENSER_IDS}" "${MEASURED_CONFIG}" "${CALIBRATION_CONFIG}" <<'PY'
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any

import yaml

raw_ids, measured_path_raw, calibration_path_raw = sys.argv[1:4]
measured_path = Path(measured_path_raw)
calibration_path = Path(calibration_path_raw)
allowed = {"1", "2", "3", "4"}


def parse_ids(raw: str) -> list[str]:
    values: list[str] = []
    for part in raw.replace(";", ",").split(","):
        item = part.strip().lower()
        if not item:
            continue
        if "x" in item:
            dispenser_id, count_raw = item.split("x", 1)
        elif ":" in item:
            dispenser_id, count_raw = item.split(":", 1)
        else:
            dispenser_id, count_raw = item, "1"
        dispenser_id = dispenser_id.strip()
        try:
            count = int(count_raw.strip())
        except ValueError as exc:
            raise ValueError(f"invalid count for dispenser {dispenser_id}: {count_raw!r}") from exc
        if count < 1:
            raise ValueError(f"count must be >= 1 for dispenser {dispenser_id}")
        if dispenser_id not in allowed:
            raise ValueError(f"unsupported dispenser id {dispenser_id}; allowed: 1,2,3,4")
        values.extend([dispenser_id] * count)
    if not values:
        raise ValueError("at least one dispenser id is required")
    return values


def require_list(block: dict[str, Any], key: str, count: int, label: str) -> list[float]:
    value = block.get(key)
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{label}.{key} must be a {count}-number list")
    try:
        numbers = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}.{key} must contain only numbers") from exc
    if not all(math.isfinite(item) for item in numbers):
        raise ValueError(f"{label}.{key} contains non-finite values")
    return numbers

try:
    dispenser_ids = parse_ids(raw_ids)
    unique_ids = []
    for dispenser_id in dispenser_ids:
        if dispenser_id not in unique_ids:
            unique_ids.append(dispenser_id)
    if not measured_path.is_file():
        raise FileNotFoundError(f"measured dispenser config not found: {measured_path}")
    if not calibration_path.is_file():
        raise FileNotFoundError(f"calibration config not found: {calibration_path}")
    measured = yaml.safe_load(measured_path.read_text(encoding="utf-8")) or {}
    calibration = yaml.safe_load(calibration_path.read_text(encoding="utf-8")) or {}
    front_hold_poses = measured.get("front_hold_poses") or {}
    outlets = calibration.get("dispenser_outlets") or {}

    for dispenser_id in unique_ids:
        front_key = f"dispenser_{dispenser_id}"
        front = front_hold_poses.get(front_key)
        if not isinstance(front, dict):
            raise ValueError(f"front_hold_poses.{front_key} missing in {measured_path}")
        require_list(front, "position_xyz_m", 3, f"front_hold_poses.{front_key}")
        require_list(front, "quaternion_xyzw", 4, f"front_hold_poses.{front_key}")

        outlet = outlets.get(dispenser_id)
        if not isinstance(outlet, dict):
            raise ValueError(f"dispenser_outlets.{dispenser_id} missing in {calibration_path}")
        require_list(outlet, "press_pose_xyz_m", 3, f"dispenser_outlets.{dispenser_id}")
        require_list(outlet, "press_pose_rpy_deg", 3, f"dispenser_outlets.{dispenser_id}")
        require_list(outlet, "press_contact_joints_deg", 6, f"dispenser_outlets.{dispenser_id}")

    grouped: list[tuple[str, int]] = []
    for dispenser_id in dispenser_ids:
        if grouped and grouped[-1][0] == dispenser_id:
            grouped[-1] = (grouped[-1][0], grouped[-1][1] + 1)
        else:
            grouped.append((dispenser_id, 1))
    print("[PASS] one-click cocktail config preflight OK")
    print(f"[Azas] dispenser_ids={','.join(dispenser_ids)}")
    print("[Azas] grouped_press_counts=" + ",".join(f"{dispenser_id}x{count}" for dispenser_id, count in grouped))
    print(f"[Azas] checked_front_hold_and_press_joints={','.join(unique_ids)}")
except Exception as exc:
    print(f"[FAIL] one-click cocktail config preflight failed: {exc}", file=sys.stderr)
    raise SystemExit(1)
PY
