#!/usr/bin/env python3
"""Move to a measured fixed dispenser front-hold pose.

The target is read from measured_dispenser_collision.yaml front_hold_poses.
This is for fixed dispenser/link_6 teaching poses only; cup pose still comes
from vision in the full pipeline.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from pathlib import Path
from typing import Any

import yaml


ROOT = Path("/home/ssu/Azas")
DEFAULT_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "measured_dispenser_collision.yaml"
DIRECT_MOVEL = ROOT / "tools" / "run" / "direct_movel_xyz.py"
CONFIRM_PHRASE = "ENABLE_MEASURED_DISPENSER_FRONT_HOLD"
DIRECT_CONFIRM_PHRASE = "ENABLE_DIRECT_MOVEL"


def numeric_list(value: Any, label: str, count: int) -> list[float]:
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{label} must be a {count}-number list")
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain only numbers") from exc


def xyz(value: Any, label: str) -> list[float]:
    return numeric_list(value, label, 3)


def quaternion_to_matrix_xyzw(quaternion: list[float]) -> list[list[float]]:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("quaternion norm must be non-zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def quaternion_to_doosan_zyz_deg(quaternion: list[float]) -> list[float]:
    """Convert ROS quaternion XYZW to the Doosan posx ZYZ Euler convention."""
    matrix = quaternion_to_matrix_xyzw(quaternion)
    beta = math.acos(max(-1.0, min(1.0, matrix[2][2])))
    sin_beta = math.sin(beta)
    if abs(sin_beta) > 1e-8:
        alpha = math.atan2(matrix[1][2], matrix[0][2])
        gamma = math.atan2(matrix[2][1], -matrix[2][0])
    else:
        alpha = 0.0
        gamma = math.atan2(-matrix[0][1], matrix[0][0])
    return [math.degrees(value) for value in (alpha, beta, gamma)]


def load_pose(config_path: Path, dispenser_id: int) -> tuple[list[float], list[float], list[float], list[float], str]:
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    metadata = data.get("metadata") or {}
    target_frame = str(metadata.get("measured_target_frame") or "")
    poses = data.get("front_hold_poses") or {}
    key = f"dispenser_{dispenser_id}"
    block = poses.get(key)
    if not isinstance(block, dict):
        raise ValueError(f"front_hold_poses.{key} is missing in {config_path}")
    position = xyz(block.get("position_xyz_m"), f"front_hold_poses.{key}.position_xyz_m")
    ros_rpy = xyz(block.get("rpy_deg"), f"front_hold_poses.{key}.rpy_deg")
    quaternion = numeric_list(
        block.get("quaternion_xyzw"), f"front_hold_poses.{key}.quaternion_xyzw", 4
    )
    doosan_zyz = quaternion_to_doosan_zyz_deg(quaternion)
    return position, doosan_zyz, ros_rpy, quaternion, target_frame


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move to measured dispenser_N front_hold pose from measured_dispenser_collision.yaml."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--dispenser-id", type=int, default=2, choices=(1, 2, 3, 4))
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--velocity", type=float, default=8.0)
    parser.add_argument("--acceleration", type=float, default=8.0)
    parser.add_argument("--timeout-sec", type=float, default=180.0)
    parser.add_argument("--wait-service-sec", type=float, default=8.0)
    parser.add_argument("--target-tolerance-mm", type=float, default=15.0)
    parser.add_argument("--verify-timeout-sec", type=float, default=70.0)
    parser.add_argument("--precheck-ikin", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--verify-target", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--confirm",
        default="",
        help=f"must equal {CONFIRM_PHRASE} when --execute is used",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.config.is_file():
        print(f"[FAIL] measured dispenser config not found: {args.config}")
        return 2

    try:
        position, doosan_zyz, ros_rpy, quaternion, target_frame = load_pose(
            args.config, args.dispenser_id
        )
    except ValueError as exc:
        print(f"[FAIL] {exc}")
        return 2

    print("[Azas] Measured dispenser front-hold target")
    print(f"[Azas] config={args.config}")
    print(f"[Azas] dispenser_id={args.dispenser_id}")
    print(f"[Azas] measured_target_frame={target_frame or '<unspecified>'}")
    print(
        "[Azas] xyz_m="
        f"[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] "
        f"quaternion_xyzw=[{quaternion[0]:.3f}, {quaternion[1]:.3f}, {quaternion[2]:.3f}, {quaternion[3]:.3f}]"
    )
    print(
        "[Azas] yaml_ros_rpy_deg(reference)="
        f"[{ros_rpy[0]:.3f}, {ros_rpy[1]:.3f}, {ros_rpy[2]:.3f}]"
    )
    print(
        "[Azas] direct_movel_doosan_zyz_deg="
        f"[{doosan_zyz[0]:.3f}, {doosan_zyz[1]:.3f}, {doosan_zyz[2]:.3f}]"
    )
    print("[Azas] source=front_hold_poses; not the old temporary direct XYZ candidate")

    if args.execute and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
        return 2

    cmd = [
        sys.executable,
        str(DIRECT_MOVEL),
        "--service-prefix",
        args.service_prefix,
        "--x",
        f"{position[0]:.6f}",
        "--y",
        f"{position[1]:.6f}",
        "--z",
        f"{position[2]:.6f}",
        "--rx",
        f"{doosan_zyz[0]:.6f}",
        "--ry",
        f"{doosan_zyz[1]:.6f}",
        "--rz",
        f"{doosan_zyz[2]:.6f}",
        "--velocity",
        f"{args.velocity:.6f}",
        "--acceleration",
        f"{args.acceleration:.6f}",
        "--timeout-sec",
        f"{args.timeout_sec:.6f}",
        "--wait-service-sec",
        f"{args.wait_service_sec:.6f}",
        "--target-tolerance-mm",
        f"{args.target_tolerance_mm:.6f}",
        "--verify-timeout-sec",
        f"{args.verify_timeout_sec:.6f}",
    ]
    if args.precheck_ikin:
        cmd.append("--precheck-ikin")
    if args.verify_target:
        cmd.append("--verify-target")
    if args.execute:
        cmd.extend(["--execute", "--confirm", DIRECT_CONFIRM_PHRASE])

    sys.stdout.flush()
    return subprocess.run(cmd, cwd=str(ROOT), check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
