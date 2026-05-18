#!/usr/bin/env python3
"""Send one explicit base-frame MoveLine target.

This bypasses perception/dispenser calibration on purpose: it only moves to
coordinates supplied by the operator. Defaults are fail-closed and dry-run.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass

import rclpy
from dsr_msgs2.srv import MoveLine


DR_BASE = 0
MOVE_MODE_ABSOLUTE = 0
SYNC = 0
BLENDING_SPEED_TYPE_DUPLICATE = 0
CONFIRM_PHRASE = "ENABLE_DIRECT_MOVEL"


@dataclass(frozen=True)
class Bounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def validate(self, x: float, y: float, z: float) -> list[str]:
        failures: list[str] = []
        if not self.x_min <= x <= self.x_max:
            failures.append(f"x={x:.3f} outside [{self.x_min:.3f}, {self.x_max:.3f}]")
        if not self.y_min <= y <= self.y_max:
            failures.append(f"y={y:.3f} outside [{self.y_min:.3f}, {self.y_max:.3f}]")
        if not self.z_min <= z <= self.z_max:
            failures.append(f"z={z:.3f} outside [{self.z_min:.3f}, {self.z_max:.3f}]")
        return failures


def service_name(prefix: str) -> str:
    clean = prefix.strip("/")
    return f"/{clean}/motion/move_line" if clean else "/motion/move_line"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move directly to one supplied XYZ/RPY target via Doosan MoveLine."
    )
    parser.add_argument("--x", type=float, required=True, help="target x in meters, base frame")
    parser.add_argument("--y", type=float, required=True, help="target y in meters, base frame")
    parser.add_argument("--z", type=float, required=True, help="target z in meters, base frame")
    parser.add_argument("--rx", type=float, default=180.0, help="Doosan rx in degrees")
    parser.add_argument("--ry", type=float, default=0.0, help="Doosan ry in degrees")
    parser.add_argument("--rz", type=float, default=180.0, help="Doosan rz in degrees")
    parser.add_argument("--service-prefix", default="", help="optional namespace before /motion/move_line")
    parser.add_argument("--velocity", type=float, default=20.0, help="line velocity")
    parser.add_argument("--acceleration", type=float, default=20.0, help="line acceleration")
    parser.add_argument("--timeout-sec", type=float, default=10.0, help="service response timeout")
    parser.add_argument("--wait-service-sec", type=float, default=5.0, help="service availability timeout")
    parser.add_argument("--x-min", type=float, default=0.10)
    parser.add_argument("--x-max", type=float, default=0.70)
    parser.add_argument("--y-min", type=float, default=-0.45)
    parser.add_argument("--y-max", type=float, default=0.45)
    parser.add_argument("--z-min", type=float, default=0.05)
    parser.add_argument("--z-max", type=float, default=0.80)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="actually call MoveLine; without this, only prints the request",
    )
    parser.add_argument(
        "--confirm",
        default="",
        help=f"must equal {CONFIRM_PHRASE} when --execute is used",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bounds = Bounds(args.x_min, args.x_max, args.y_min, args.y_max, args.z_min, args.z_max)
    failures = bounds.validate(args.x, args.y, args.z)
    if failures:
        print("[BLOCKED] target outside direct-move bounds")
        for failure in failures:
            print(f"  - {failure}")
        return 2

    move_service = service_name(args.service_prefix)
    pos_mm_deg = [
        args.x * 1000.0,
        args.y * 1000.0,
        args.z * 1000.0,
        args.rx,
        args.ry,
        args.rz,
    ]
    print("[Azas] Direct MoveLine target")
    print(f"[Azas] service={move_service}")
    print(
        "[Azas] pos_mm_deg="
        f"[{pos_mm_deg[0]:.1f}, {pos_mm_deg[1]:.1f}, {pos_mm_deg[2]:.1f}, "
        f"{pos_mm_deg[3]:.1f}, {pos_mm_deg[4]:.1f}, {pos_mm_deg[5]:.1f}]"
    )
    print(f"[Azas] vel=[{args.velocity:.1f}, {args.velocity:.1f}] acc=[{args.acceleration:.1f}, {args.acceleration:.1f}]")

    if not args.execute:
        print("[DRY-RUN] --execute not set; no robot command sent.")
        return 0

    if args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
        return 2

    rclpy.init(args=None)
    node = rclpy.create_node("azas_direct_movel_xyz")
    try:
        client = node.create_client(MoveLine, move_service)
        if not client.wait_for_service(timeout_sec=max(args.wait_service_sec, 0.1)):
            print(f"[FAIL] service not available: {move_service}")
            return 1

        req = MoveLine.Request()
        req.pos = pos_mm_deg
        req.vel = [args.velocity, args.velocity]
        req.acc = [args.acceleration, args.acceleration]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(args.timeout_sec, 0.1))
        if not future.done():
            print(f"[FAIL] MoveLine response timeout after {args.timeout_sec:.1f}s")
            return 1
        if future.exception() is not None:
            print(f"[FAIL] MoveLine exception: {future.exception()}")
            return 1
        response = future.result()
        if response is None or not response.success:
            print("[FAIL] MoveLine returned success=false")
            return 1
        print("[PASS] MoveLine accepted by service")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
