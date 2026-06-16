#!/usr/bin/env python3
"""Send one explicit base-frame MoveLine target.

This bypasses perception/dispenser calibration on purpose: it only moves to
coordinates supplied by the operator. Defaults are fail-closed and dry-run.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from dsr_msgs2.srv import GetCurrentPosx, GetLastAlarm, Ikin, MoveLine


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


def prefixed_service(prefix: str, suffix: str) -> str:
    clean = prefix.strip("/")
    return f"/{clean}/{suffix}" if clean else f"/{suffix}"


def call_service(
    node: Any,
    srv_type: Any,
    name: str,
    request: Any,
    *,
    timeout_sec: float,
    label: str,
) -> Any:
    client = node.create_client(srv_type, name)
    timeout_sec = max(timeout_sec, 0.1)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"{label} service not available: {name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        raise RuntimeError(f"{label} response timeout after {timeout_sec:.1f}s")
    if future.exception() is not None:
        raise RuntimeError(f"{label} exception: {future.exception()}")
    response = future.result()
    if response is None:
        raise RuntimeError(f"{label} returned no response")
    return response


def current_posx(node: Any, prefix: str, timeout_sec: float) -> list[float]:
    req = GetCurrentPosx.Request()
    req.ref = DR_BASE
    response = call_service(
        node,
        GetCurrentPosx,
        prefixed_service(prefix, "aux_control/get_current_posx"),
        req,
        timeout_sec=timeout_sec,
        label="GetCurrentPosx",
    )
    if not response.success or not response.task_pos_info:
        raise RuntimeError("GetCurrentPosx returned success=false or empty task_pos_info")
    values = list(response.task_pos_info[0].data)
    if len(values) < 6:
        raise RuntimeError(f"GetCurrentPosx returned too few values: {values}")
    return [float(value) for value in values[:6]]


def last_alarm_text(node: Any, prefix: str, timeout_sec: float) -> str:
    req = GetLastAlarm.Request()
    try:
        response = call_service(
            node,
            GetLastAlarm,
            prefixed_service(prefix, "system/get_last_alarm"),
            req,
            timeout_sec=timeout_sec,
            label="GetLastAlarm",
        )
    except RuntimeError as exc:
        return f"[Azas] GetLastAlarm failed: {exc}"
    return str(response)


def alarm_indicates_unreachable(text: str) -> bool:
    normalized = text.lower()
    return "not reachable" in normalized or "unreachable" in normalized


def xyz_distance_mm(actual: list[float], target: list[float]) -> float:
    return sum((actual[index] - target[index]) ** 2 for index in range(3)) ** 0.5


def wait_for_target(
    node: Any,
    prefix: str,
    target_pos_mm_deg: list[float],
    *,
    tolerance_mm: float,
    timeout_sec: float,
    abort_on_unreachable_alarm: bool = False,
) -> bool:
    started_at = time.monotonic()
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    last_line = ""
    next_alarm_check = 0.0
    initial_distance: float | None = None
    best_distance: float | None = None
    while time.monotonic() < deadline:
        actual = current_posx(node, prefix, timeout_sec=5.0)
        distance = xyz_distance_mm(actual, target_pos_mm_deg)
        if initial_distance is None:
            initial_distance = distance
        best_distance = distance if best_distance is None else min(best_distance, distance)
        last_line = (
            "[Azas] verify xyz="
            f"[{actual[0]:.1f}, {actual[1]:.1f}, {actual[2]:.1f}] "
            f"target=[{target_pos_mm_deg[0]:.1f}, {target_pos_mm_deg[1]:.1f}, {target_pos_mm_deg[2]:.1f}] "
            f"distance={distance:.1f}mm tolerance={tolerance_mm:.1f}mm"
        )
        print(last_line)
        if distance <= tolerance_mm:
            return True
        now = time.monotonic()
        if abort_on_unreachable_alarm and now >= next_alarm_check:
            next_alarm_check = now + 1.0
            alarm = last_alarm_text(node, prefix, timeout_sec=5.0)
            if alarm_indicates_unreachable(alarm):
                progress = max(0.0, (initial_distance or distance) - (best_distance or distance))
                if now - started_at >= 2.0 and progress < 3.0:
                    print("[WARN] target verification aborted early: controller reported unreachable pose")
                    print(alarm)
                    return False
        time.sleep(1.0)
    print("[FAIL] target verification timeout")
    if last_line:
        print(last_line)
    print(last_alarm_text(node, prefix, timeout_sec=5.0))
    return False


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
    parser.add_argument(
        "--use-current-rpy",
        action="store_true",
        help="read current TCP rx/ry/rz and preserve it while moving XYZ",
    )
    parser.add_argument(
        "--precheck-ikin",
        action="store_true",
        help="call /motion/ikin before MoveLine and fail closed if the pose is not solvable",
    )
    parser.add_argument(
        "--ikin-timeout-sec",
        type=float,
        default=20.0,
        help="service response timeout for each /motion/ikin precheck attempt",
    )
    parser.add_argument(
        "--ikin-retries",
        type=int,
        default=2,
        help="number of /motion/ikin precheck attempts before failing closed",
    )
    parser.add_argument("--ikin-sol-space", type=int, default=2, help="solution space used by --precheck-ikin")
    parser.add_argument(
        "--ikin-sol-spaces",
        default="",
        help="comma-separated solution spaces to try in order before failing the IK precheck",
    )
    parser.add_argument("--j5-min-deg", type=float, default=-135.0, help="safe lower limit for joint 5")
    parser.add_argument("--j5-max-deg", type=float, default=135.0, help="safe upper limit for joint 5")
    parser.add_argument("--service-prefix", default="", help="optional namespace before /motion/move_line")
    parser.add_argument("--velocity", type=float, default=20.0, help="line velocity")
    parser.add_argument("--acceleration", type=float, default=20.0, help="line acceleration")
    parser.add_argument("--timeout-sec", type=float, default=10.0, help="service response timeout")
    parser.add_argument(
        "--motion-timeout-sec",
        type=float,
        default=None,
        help="compatibility alias accepted from sequenced motion wrappers",
    )
    parser.add_argument("--wait-service-sec", type=float, default=5.0, help="service availability timeout")
    parser.add_argument("--x-min", type=float, default=0.10)
    parser.add_argument("--x-max", type=float, default=0.70)
    parser.add_argument("--y-min", type=float, default=-0.45)
    parser.add_argument("--y-max", type=float, default=0.45)
    parser.add_argument("--z-min", type=float, default=0.05)
    parser.add_argument("--z-max", type=float, default=0.80)
    parser.add_argument(
        "--verify-target",
        action="store_true",
        help="after MoveLine accepts, poll current TCP XYZ until it reaches the target",
    )
    parser.add_argument("--verify-timeout-sec", type=float, default=70.0)
    parser.add_argument("--target-tolerance-mm", type=float, default=15.0)
    parser.add_argument(
        "--movel-retries",
        type=int,
        default=1,
        help="number of MoveLine execution attempts when target verification fails",
    )
    parser.add_argument(
        "--movel-retry-sleep-sec",
        type=float,
        default=1.0,
        help="delay before retrying a failed MoveLine execution attempt",
    )
    parser.add_argument(
        "--abort-verify-on-unreachable-alarm",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="stop target verification early when the controller reports a NOT REACHABLE alarm",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="actually call MoveLine; without this, only prints the request",
    )
    parser.add_argument(
        "--fallback-movej-on-verify-fail",
        action="store_true",
        help="compatibility flag only; this tool keeps failing closed on MoveLine verification failure",
    )
    parser.add_argument("--fallback-movej-velocity", type=float, default=20.0)
    parser.add_argument("--fallback-movej-acceleration", type=float, default=20.0)
    parser.add_argument(
        "--confirm",
        default="",
        help=f"must equal {CONFIRM_PHRASE} when --execute is used",
    )
    return parser.parse_args()


def parse_ikin_sol_spaces(args: argparse.Namespace) -> list[int]:
    raw_value = str(args.ikin_sol_spaces).strip()
    if not raw_value:
        return [int(args.ikin_sol_space)]
    values = [int(part.strip()) for part in raw_value.split(",") if part.strip()]
    if not values:
        raise ValueError("--ikin-sol-spaces did not contain any solution spaces")
    return values


def main() -> int:
    args = parse_args()
    bounds = Bounds(args.x_min, args.x_max, args.y_min, args.y_max, args.z_min, args.z_max)
    failures = bounds.validate(args.x, args.y, args.z)
    if failures:
        print("[BLOCKED] target outside direct-move bounds")
        for failure in failures:
            print(f"  - {failure}")
        return 2

    needs_ros = args.execute or args.use_current_rpy or args.precheck_ikin or args.verify_target
    node = None
    if needs_ros:
        rclpy.init(args=None)
        node = rclpy.create_node("azas_direct_movel_xyz")

    try:
        rx = args.rx
        ry = args.ry
        rz = args.rz
        if args.use_current_rpy:
            assert node is not None
            pose = current_posx(node, args.service_prefix, timeout_sec=max(args.wait_service_sec, 0.1))
            rx, ry, rz = pose[3], pose[4], pose[5]
            print(f"[Azas] preserving current TCP RPY rx={rx:.3f} ry={ry:.3f} rz={rz:.3f}")

        move_service = service_name(args.service_prefix)
        pos_mm_deg = [
            args.x * 1000.0,
            args.y * 1000.0,
            args.z * 1000.0,
            rx,
            ry,
            rz,
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

        assert node is not None

        if args.precheck_ikin:
            try:
                sol_spaces = parse_ikin_sol_spaces(args)
            except ValueError as exc:
                print(f"[BLOCKED] {exc}")
                return 2

            response = None
            selected_sol_space = None
            last_failure = ""
            attempts = max(int(args.ikin_retries), 1)
            for attempt in range(1, attempts + 1):
                for sol_space in sol_spaces:
                    req = Ikin.Request()
                    req.pos = pos_mm_deg
                    req.sol_space = int(sol_space)
                    req.ref = DR_BASE
                    try:
                        candidate = call_service(
                            node,
                            Ikin,
                            prefixed_service(args.service_prefix, "motion/ikin"),
                            req,
                            timeout_sec=max(args.ikin_timeout_sec, 0.1),
                            label=f"Ikin sol_space={sol_space}",
                        )
                    except RuntimeError as exc:
                        last_failure = str(exc)
                        print(
                            f"[WARN] Ikin attempt {attempt}/{attempts} "
                            f"sol_space={sol_space} failed: {exc}"
                        )
                        continue
                    if not candidate.success:
                        last_failure = f"Ikin sol_space={sol_space} returned success=false"
                        print(f"[WARN] {last_failure}")
                        continue
                    if len(candidate.conv_posj) >= 5:
                        joint5 = float(candidate.conv_posj[4])
                        if not float(args.j5_min_deg) <= joint5 <= float(args.j5_max_deg):
                            last_failure = (
                                f"Ikin sol_space={sol_space} predicted joint_5={joint5:.3f} deg outside "
                                f"[{float(args.j5_min_deg):.3f}, {float(args.j5_max_deg):.3f}] deg"
                            )
                            print(f"[WARN] {last_failure}")
                            continue
                    response = candidate
                    selected_sol_space = int(sol_space)
                    break
                if response is not None:
                    break
                if attempt < attempts:
                    time.sleep(1.0)
            if response is None:
                print("[FAIL] Ikin precheck failed for all configured solution spaces")
                if last_failure:
                    print(f"[FAIL] last failure: {last_failure}")
                return 1
            print(
                f"[Azas] Ikin precheck success: sol_space={selected_sol_space} joints_deg=["
                + ", ".join(f"{value:.1f}" for value in response.conv_posj)
                + "]"
            )

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

        move_attempts = max(int(args.movel_retries), 1)
        last_failure = ""
        for move_attempt in range(1, move_attempts + 1):
            if move_attempts > 1:
                print(f"[Azas] MoveLine execution attempt {move_attempt}/{move_attempts}")
            if move_attempt > 1:
                time.sleep(max(float(args.movel_retry_sleep_sec), 0.0))

            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=max(args.timeout_sec, 0.1))
            if not future.done():
                last_failure = f"MoveLine response timeout after {args.timeout_sec:.1f}s"
                print(f"[WARN] {last_failure}")
                if args.verify_target:
                    print("[Azas] MoveLine request may still be executing; verifying target before retry/fail.")
                    if wait_for_target(
                        node,
                        args.service_prefix,
                        pos_mm_deg,
                        tolerance_mm=max(args.target_tolerance_mm, 0.1),
                        timeout_sec=max(args.verify_timeout_sec, 0.1),
                        abort_on_unreachable_alarm=bool(args.abort_verify_on_unreachable_alarm),
                    ):
                        print("[PASS] target reached after MoveLine response timeout")
                        return 0
                    last_failure = "MoveLine response timeout and target verification did not pass"
                if move_attempt < move_attempts:
                    print("[WARN] MoveLine attempt failed; retrying.")
                    continue
                print(f"[FAIL] {last_failure}")
                return 1
            if future.exception() is not None:
                last_failure = f"MoveLine exception: {future.exception()}"
                if move_attempt < move_attempts:
                    print(f"[WARN] {last_failure}; retrying.")
                    continue
                print(f"[FAIL] {last_failure}")
                return 1
            response = future.result()
            if response is None or not response.success:
                last_failure = "MoveLine returned success=false"
                if move_attempt < move_attempts:
                    print(f"[WARN] {last_failure}; retrying.")
                    continue
                print(f"[FAIL] {last_failure}")
                return 1
            print("[PASS] MoveLine accepted by service")
            if args.verify_target and not wait_for_target(
                node,
                args.service_prefix,
                pos_mm_deg,
                tolerance_mm=max(args.target_tolerance_mm, 0.1),
                timeout_sec=max(args.verify_timeout_sec, 0.1),
                abort_on_unreachable_alarm=bool(args.abort_verify_on_unreachable_alarm),
            ):
                last_failure = "MoveLine target verification failed"
                if move_attempt < move_attempts:
                    print("[WARN] MoveLine target verification failed; retrying.")
                    continue
                print(f"[FAIL] {last_failure} after {move_attempts} attempt(s)")
                return 1
            return 0
        print(f"[FAIL] {last_failure or 'MoveLine failed'}")
        return 1
    except RuntimeError as exc:
        print(f"[FAIL] {exc}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
