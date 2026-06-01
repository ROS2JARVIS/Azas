#!/usr/bin/env python3
"""Move to the measured dispenser color-scan observation joint pose.

The motion command is a joint move, but the taught target must carry measured
TCP metadata.  Before executing, this script fail-closes if the current or
target TCP is outside the configured/repo safety envelope.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from dsr_msgs2.srv import GetCurrentPosx

ROOT = Path("/home/ssu/Azas")
DEFAULT_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "dispenser_color_scan.yaml"
DEFAULT_SAFETY_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "safety.yaml"
CONFIRM_PHRASE = "ENABLE_DISPENSER_COLOR_SCAN_MOVE"

# Mirrors the existing dispenser press / legacy workspace guard.  This is not
# a new calibration value; it is a conservative software safety envelope used
# when safety.yaml still has null measured bounds.
DEFAULT_SAFE_X_MIN_M = 0.00
DEFAULT_SAFE_X_MAX_M = 0.80
DEFAULT_SAFE_Y_MIN_M = -0.30
DEFAULT_SAFE_Y_MAX_M = 0.30
DEFAULT_SAFE_Z_MIN_M = 0.27
DEFAULT_SAFE_Z_MAX_M = 0.75


@dataclass(frozen=True)
class SafetyBounds:
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    z_min_m: float
    z_max_m: float
    source: str

    def validate_xyz_m(self, xyz_m: list[float], label: str) -> list[str]:
        x, y, z = xyz_m[:3]
        failures: list[str] = []
        if not self.x_min_m <= x <= self.x_max_m:
            failures.append(f"{label} x={x:.3f}m outside [{self.x_min_m:.3f}, {self.x_max_m:.3f}]m")
        if not self.y_min_m <= y <= self.y_max_m:
            failures.append(f"{label} y={y:.3f}m outside [{self.y_min_m:.3f}, {self.y_max_m:.3f}]m")
        if not self.z_min_m <= z <= self.z_max_m:
            failures.append(f"{label} z={z:.3f}m outside [{self.z_min_m:.3f}, {self.z_max_m:.3f}]m")
        return failures


def service_name(prefix: str, suffix: str) -> str:
    clean_prefix = prefix.strip("/")
    clean_suffix = suffix.strip("/")
    return f"/{clean_prefix}/{clean_suffix}" if clean_prefix else f"/{clean_suffix}"


def parse_axis_bounds(value: Any, axis: str) -> tuple[float, float] | None:
    if not isinstance(value, dict):
        return None
    candidates = [
        value.get(axis),
        value.get(f"{axis}_m"),
        value.get(f"{axis}_bounds_m"),
        value.get(f"{axis}_range_m"),
    ]
    min_key = value.get(f"{axis}_min_m")
    max_key = value.get(f"{axis}_max_m")
    if min_key is not None and max_key is not None:
        return float(min_key), float(max_key)
    for candidate in candidates:
        if isinstance(candidate, (list, tuple)) and len(candidate) == 2 and all(v is not None for v in candidate):
            return float(candidate[0]), float(candidate[1])
        if isinstance(candidate, dict):
            lo = candidate.get("min", candidate.get("min_m"))
            hi = candidate.get("max", candidate.get("max_m"))
            if lo is not None and hi is not None:
                return float(lo), float(hi)
    return None


def load_safety_bounds(path: Path, args: argparse.Namespace) -> SafetyBounds:
    bounds = SafetyBounds(
        float(args.x_min_m),
        float(args.x_max_m),
        float(args.y_min_m),
        float(args.y_max_m),
        float(args.z_min_m),
        float(args.z_max_m),
        "built-in legacy dispenser safety envelope",
    )
    if not path.exists():
        print(f"[WARN] safety config not found: {path}; using {bounds.source}")
        return bounds
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    motion = data.get("motion") or {}
    workspace = motion.get("workspace_bounds_m")
    min_z = motion.get("min_z_m")
    parsed = {
        "x": parse_axis_bounds(workspace, "x"),
        "y": parse_axis_bounds(workspace, "y"),
        "z": parse_axis_bounds(workspace, "z"),
    }
    if any(value is not None for value in parsed.values()) or min_z is not None:
        x_min, x_max = parsed["x"] or (bounds.x_min_m, bounds.x_max_m)
        y_min, y_max = parsed["y"] or (bounds.y_min_m, bounds.y_max_m)
        z_min, z_max = parsed["z"] or (bounds.z_min_m, bounds.z_max_m)
        if min_z is not None:
            z_min = float(min_z)
        return SafetyBounds(x_min, x_max, y_min, y_max, z_min, z_max, f"safety config {path}")
    print(f"[WARN] {path} has null/unmeasured motion.workspace_bounds_m/min_z_m; using {bounds.source}")
    return bounds


def current_posx_mm_deg(service_prefix: str, timeout_sec: float) -> list[float]:
    rclpy.init(args=None)
    node = rclpy.create_node("azas_dispenser_color_scan_safety_check")
    try:
        client = node.create_client(GetCurrentPosx, service_name(service_prefix, "aux_control/get_current_posx"))
        timeout_sec = max(timeout_sec, 0.1)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("get_current_posx service not available")
        req = GetCurrentPosx.Request()
        req.ref = 0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError("get_current_posx timeout")
        response = future.result()
        if response is None or not response.success or not response.task_pos_info:
            raise RuntimeError("GetCurrentPosx returned success=false or empty task_pos_info")
        values = list(response.task_pos_info[0].data)
        if len(values) < 6:
            raise RuntimeError(f"GetCurrentPosx returned too few values: {values}")
        return [float(value) for value in values[:6]]
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def posx_xyz_m(posx_mm_deg: list[float]) -> list[float]:
    return [float(posx_mm_deg[index]) / 1000.0 for index in range(3)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move to measured color-scan joint pose before RGB color mapping.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--safety-config", type=Path, default=DEFAULT_SAFETY_CONFIG)
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--velocity", type=float, default=20.0)
    parser.add_argument("--acceleration", type=float, default=25.0)
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument("--x-min-m", type=float, default=DEFAULT_SAFE_X_MIN_M)
    parser.add_argument("--x-max-m", type=float, default=DEFAULT_SAFE_X_MAX_M)
    parser.add_argument("--y-min-m", type=float, default=DEFAULT_SAFE_Y_MIN_M)
    parser.add_argument("--y-max-m", type=float, default=DEFAULT_SAFE_Y_MAX_M)
    parser.add_argument("--z-min-m", type=float, default=DEFAULT_SAFE_Z_MIN_M)
    parser.add_argument("--z-max-m", type=float, default=DEFAULT_SAFE_Z_MAX_M)
    parser.add_argument("--check-current-tcp", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} with --execute")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.config.exists():
        print(f"[FAIL] config not found: {args.config}")
        return 2
    data = yaml.safe_load(args.config.read_text(encoding="utf-8")) or {}
    pose = data.get("color_scan_pose") or {}
    joints = pose.get("joints_deg")
    tcp_posx = pose.get("tcp_posx_mm_deg")
    if not isinstance(joints, list) or len(joints) != 6 or any(value is None for value in joints):
        print("[BLOCKED] color_scan_pose.joints_deg is not measured yet; run teach_dispenser_color_scan_pose first")
        return 2
    has_target_tcp = isinstance(tcp_posx, list) and len(tcp_posx) >= 6 and not any(value is None for value in tcp_posx[:6])
    if not has_target_tcp:
        print(
            "[WARN] color_scan_pose.tcp_posx_mm_deg is missing; "
            "target TCP workspace precheck is skipped. Joint bounds and current TCP safety checks still apply."
        )

    bounds = load_safety_bounds(args.safety_config, args)
    print(f"[Azas] safety bounds source={bounds.source}")
    print(
        "[Azas] safety bounds "
        f"x=[{bounds.x_min_m:.3f},{bounds.x_max_m:.3f}]m "
        f"y=[{bounds.y_min_m:.3f},{bounds.y_max_m:.3f}]m "
        f"z=[{bounds.z_min_m:.3f},{bounds.z_max_m:.3f}]m"
    )
    if has_target_tcp:
        failures = bounds.validate_xyz_m(posx_xyz_m([float(v) for v in tcp_posx[:6]]), "target color-scan TCP")
        if failures:
            print("[BLOCKED] taught color-scan target is outside safety area")
            for failure in failures:
                print(f"  - {failure}")
            return 2

    if args.execute and args.check_current_tcp:
        try:
            current = current_posx_mm_deg(args.service_prefix, args.timeout_sec)
        except RuntimeError as exc:
            print(f"[BLOCKED] cannot verify current TCP safety before motion: {exc}")
            return 2
        current_failures = bounds.validate_xyz_m(posx_xyz_m(current), "current TCP")
        if current_failures:
            print("[BLOCKED] current TCP is outside safety area; refusing to start joint move")
            for failure in current_failures:
                print(f"  - {failure}")
            return 2
        print(
            "[Azas] current TCP safety OK xyz_m="
            f"[{current[0] / 1000.0:.3f}, {current[1] / 1000.0:.3f}, {current[2] / 1000.0:.3f}]"
        )

    cmd = [
        "python3",
        str(ROOT / "tools" / "run" / "direct_movej_joints.py"),
        "--service-prefix",
        args.service_prefix,
        "--velocity",
        str(args.velocity),
        "--acceleration",
        str(args.acceleration),
    ]
    for index, value in enumerate(joints, start=1):
        cmd.extend([f"--j{index}", str(float(value))])
    if args.execute:
        if args.confirm != CONFIRM_PHRASE:
            print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
            return 2
        cmd.extend(["--execute", "--confirm", "ENABLE_DIRECT_MOVEJ"])
    print("[Azas] Move to measured dispenser color-scan pose")
    print("[Azas] joints_deg=[" + ", ".join(f"{float(value):.3f}" for value in joints) + "]")
    if has_target_tcp:
        print("[Azas] target_tcp_xyz_m=[" + ", ".join(f"{v:.3f}" for v in posx_xyz_m([float(x) for x in tcp_posx[:6]])) + "]")
    else:
        print("[Azas] target_tcp_xyz_m=<not recorded; joint target only>")
    sys.stdout.flush()
    return subprocess.run(cmd).returncode


if __name__ == "__main__":
    raise SystemExit(main())
