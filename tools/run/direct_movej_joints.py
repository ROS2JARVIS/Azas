#!/usr/bin/env python3
"""Send one explicit Doosan MoveJoint target.

This is for fixed-coordinate field tests where the operator intentionally
chooses the joint target. Defaults are fail-closed and dry-run.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass

import rclpy
from dsr_msgs2.srv import MoveJoint


MOVE_MODE_ABSOLUTE = 0
SYNC = 0
BLENDING_SPEED_TYPE_DUPLICATE = 0
CONFIRM_PHRASE = "ENABLE_DIRECT_MOVEJ"


@dataclass(frozen=True)
class JointBounds:
    lower_deg: float = -180.0
    upper_deg: float = 180.0
    joint5_lower_deg: float = -135.0
    joint5_upper_deg: float = 135.0

    def validate(self, values: list[float]) -> list[str]:
        failures: list[str] = []
        if len(values) != 6:
            failures.append(f"expected 6 joints, got {len(values)}")
            return failures
        for index, value in enumerate(values, start=1):
            if not self.lower_deg <= value <= self.upper_deg:
                failures.append(
                    f"j{index}={value:.3f} outside "
                    f"[{self.lower_deg:.3f}, {self.upper_deg:.3f}]"
                )
        joint5 = values[4]
        if not self.joint5_lower_deg <= joint5 <= self.joint5_upper_deg:
            failures.append(
                f"j5={joint5:.3f} outside safe wrist range "
                f"[{self.joint5_lower_deg:.3f}, {self.joint5_upper_deg:.3f}]"
            )
        return failures


def service_name(prefix: str) -> str:
    clean = prefix.strip("/")
    return f"/{clean}/motion/move_joint" if clean else "/motion/move_joint"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move directly to one supplied joint target via Doosan MoveJoint."
    )
    for index in range(1, 7):
        parser.add_argument(f"--j{index}", type=float, required=True, help=f"joint {index} in degrees")
    parser.add_argument("--service-prefix", default="", help="optional namespace before /motion/move_joint")
    parser.add_argument("--velocity", type=float, default=10.0, help="joint velocity")
    parser.add_argument("--acceleration", type=float, default=10.0, help="joint acceleration")
    parser.add_argument("--j5-min-deg", type=float, default=-135.0, help="safe lower limit for joint 5")
    parser.add_argument("--j5-max-deg", type=float, default=135.0, help="safe upper limit for joint 5")
    parser.add_argument("--timeout-sec", type=float, default=20.0, help="service response timeout")
    parser.add_argument("--wait-service-sec", type=float, default=5.0, help="service availability timeout")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="actually call MoveJoint; without this, only prints the request",
    )
    parser.add_argument(
        "--confirm",
        default="",
        help=f"must equal {CONFIRM_PHRASE} when --execute is used",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    joints_deg = [float(getattr(args, f"j{index}")) for index in range(1, 7)]
    failures = JointBounds(
        joint5_lower_deg=float(args.j5_min_deg),
        joint5_upper_deg=float(args.j5_max_deg),
    ).validate(joints_deg)
    if failures:
        print("[BLOCKED] joint target outside direct-move bounds")
        for failure in failures:
            print(f"  - {failure}")
        return 2

    move_service = service_name(args.service_prefix)
    print("[Azas] Direct MoveJoint target")
    print(f"[Azas] service={move_service}")
    print("[Azas] joints_deg=[" + ", ".join(f"{value:.1f}" for value in joints_deg) + "]")
    print(f"[Azas] vel={args.velocity:.1f} acc={args.acceleration:.1f}")

    if not args.execute:
        print("[DRY-RUN] --execute not set; no robot command sent.")
        return 0

    if args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
        return 2

    rclpy.init(args=None)
    node = rclpy.create_node("azas_direct_movej_joints")
    try:
        client = node.create_client(MoveJoint, move_service)
        if not client.wait_for_service(timeout_sec=max(args.wait_service_sec, 0.1)):
            print(f"[FAIL] service not available: {move_service}")
            return 1

        req = MoveJoint.Request()
        req.pos = joints_deg
        req.vel = float(args.velocity)
        req.acc = float(args.acceleration)
        req.time = 0.0
        req.radius = 0.0
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(args.timeout_sec, 0.1))
        if not future.done():
            print(f"[FAIL] MoveJoint response timeout after {args.timeout_sec:.1f}s")
            return 1
        if future.exception() is not None:
            print(f"[FAIL] MoveJoint exception: {future.exception()}")
            return 1
        response = future.result()
        if response is None or not response.success:
            print("[FAIL] MoveJoint returned success=false")
            return 1
        print("[PASS] MoveJoint accepted by service")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
