#!/usr/bin/env python3
"""Move only to measured dispenser press contact check poses.

This is a hardware check tool, not a recipe runner.  It reads measured
press_contact_joints_deg from calibration.yaml, uses Doosan FK to derive the
contact TCP pose, then generates PRE and optional PRESS poses by changing only
Z.  It never calls gripper, TCP setup, cup placement, or re-grasp services.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from dsr_msgs2.srv import Fkin, GetCurrentPosx, MoveLine, MoveWait


ROOT = Path("/home/ssu/Azas")
CALIBRATION_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"
CONFIRM_PHRASE = "ENABLE_CHECK_PRESS_CONTACT"

DR_BASE = 0
MOVE_MODE_ABSOLUTE = 0
SYNC = 0
BLENDING_SPEED_TYPE_DUPLICATE = 0
INVALID_PRESS_CONTACT_STATUSES = {
    "invalid",
    "invalid_reteach_required",
    "needs_reteach",
    "reteach_required",
    "확인 필요",
}


def service_name(prefix: str, suffix: str) -> str:
    clean = prefix.strip("/")
    return f"/{clean}/{suffix}" if clean else f"/{suffix}"


def numeric_list(raw: object, label: str, size: int) -> list[float]:
    if not isinstance(raw, list) or len(raw) != size:
        raise ValueError(f"{label} must be a {size}-item list")
    return [float(value) for value in raw]


def parse_dispenser_ids(raw: str) -> list[str]:
    ids: list[str] = []
    for token in raw.replace(",", " ").split():
        token = token.strip()
        if not token:
            continue
        if token not in {"1", "2", "3", "4"}:
            raise ValueError(f"unsupported dispenser id {token!r}; expected 1..4")
        ids.append(token)
    if not ids:
        raise ValueError("no dispenser ids provided")
    return ids


def load_press_contact_joints(calibration: Path, dispenser_id: str) -> list[float]:
    data = yaml.safe_load(calibration.read_text(encoding="utf-8")) or {}
    outlet = (data.get("dispenser_outlets") or {}).get(str(dispenser_id))
    if not isinstance(outlet, dict):
        raise ValueError(f"dispenser_outlets.{dispenser_id} is missing in {calibration}")
    status = str(outlet.get("press_contact_status", "")).strip()
    if status.lower() in INVALID_PRESS_CONTACT_STATUSES:
        raise ValueError(
            f"dispenser_outlets.{dispenser_id}.press_contact_joints_deg is marked "
            f"{status!r}; refusing press-contact motion until PRESS{dispenser_id}_CONTACT is re-taught"
        )
    return numeric_list(
        outlet.get("press_contact_joints_deg"),
        f"dispenser_outlets.{dispenser_id}.press_contact_joints_deg",
        6,
    )


def call_service(
    node: Any,
    client: Any,
    request: Any,
    *,
    timeout_sec: float,
    label: str,
) -> Any:
    timeout_sec = max(float(timeout_sec), 0.1)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"{label} service not available: {client.srv_name}")
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


def xyz_distance_mm(a: list[float], b: list[float]) -> float:
    return sum((a[index] - b[index]) ** 2 for index in range(3)) ** 0.5


class PressContactChecker:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        rclpy.init(args=None)
        self.node = rclpy.create_node("azas_check_measured_dispenser_press_contact")
        self.fkin = self.node.create_client(Fkin, service_name(args.service_prefix, "motion/fkin"))
        self.move_line = self.node.create_client(MoveLine, service_name(args.service_prefix, "motion/move_line"))
        self.move_wait = self.node.create_client(MoveWait, service_name(args.service_prefix, "motion/move_wait"))
        self.get_posx = self.node.create_client(
            GetCurrentPosx,
            service_name(args.service_prefix, "aux_control/get_current_posx"),
        )

    def close(self) -> None:
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def fkin_posx(self, joints_deg: list[float], label: str) -> list[float]:
        req = Fkin.Request()
        req.pos = [float(value) for value in joints_deg]
        req.ref = DR_BASE
        response = call_service(
            self.node,
            self.fkin,
            req,
            timeout_sec=self.args.wait_service_sec,
            label=f"Fkin {label}",
        )
        if not response.success:
            raise RuntimeError(f"Fkin returned success=false for {label}")
        posx = [float(value) for value in response.conv_posx[:6]]
        if len(posx) < 6:
            raise RuntimeError(f"Fkin returned too few values for {label}: {posx}")
        print(
            f"[Azas] {label}: contact_fk_posx=[{posx[0]:.1f}, {posx[1]:.1f}, {posx[2]:.1f}, "
            f"{posx[3]:.1f}, {posx[4]:.1f}, {posx[5]:.1f}]"
        )
        return posx

    def current_posx(self) -> list[float]:
        req = GetCurrentPosx.Request()
        req.ref = DR_BASE
        response = call_service(
            self.node,
            self.get_posx,
            req,
            timeout_sec=self.args.wait_service_sec,
            label="GetCurrentPosx",
        )
        if not response.success or not response.task_pos_info:
            raise RuntimeError("GetCurrentPosx returned success=false or empty task_pos_info")
        values = [float(value) for value in response.task_pos_info[0].data[:6]]
        if len(values) < 6:
            raise RuntimeError(f"GetCurrentPosx returned too few values: {values}")
        return values

    def move_posx(self, target: list[float], label: str, *, velocity: float, acceleration: float) -> None:
        print(
            f"[Azas] {label}: target_posx=[{target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}, "
            f"{target[3]:.1f}, {target[4]:.1f}, {target[5]:.1f}] vel={velocity:.1f} acc={acceleration:.1f}"
        )
        if not self.args.execute:
            return
        req = MoveLine.Request()
        req.pos = [float(value) for value in target]
        req.vel = [float(velocity), float(velocity)]
        req.acc = [float(acceleration), float(acceleration)]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC
        response = call_service(
            self.node,
            self.move_line,
            req,
            timeout_sec=self.args.motion_timeout_sec,
            label=f"MoveLine {label}",
        )
        if not response.success:
            raise RuntimeError(f"MoveLine returned success=false for {label}")
        self.wait_motion_done(label)
        if self.args.verify_target:
            self.wait_for_target(target, label)

    def wait_motion_done(self, label: str) -> None:
        response = call_service(
            self.node,
            self.move_wait,
            MoveWait.Request(),
            timeout_sec=self.args.motion_timeout_sec,
            label=f"MoveWait {label}",
        )
        if not bool(getattr(response, "success", True)):
            raise RuntimeError(f"MoveWait returned success=false for {label}: {response}")
        print(f"[Azas] {label}: MoveWait completed")

    def wait_for_target(self, target: list[float], label: str) -> None:
        deadline = time.monotonic() + max(float(self.args.verify_timeout_sec), 0.1)
        last_distance = 999999.0
        while time.monotonic() < deadline:
            actual = self.current_posx()
            last_distance = xyz_distance_mm(actual, target)
            print(
                f"[Azas] verify {label}: actual_xyz=[{actual[0]:.1f}, {actual[1]:.1f}, {actual[2]:.1f}] "
                f"distance={last_distance:.1f}mm tolerance={self.args.target_tolerance_mm:.1f}mm"
            )
            if last_distance <= max(float(self.args.target_tolerance_mm), 0.1):
                return
            time.sleep(max(float(self.args.verify_poll_seconds), 0.05))
        raise RuntimeError(f"target verification timeout for {label}; distance={last_distance:.1f}mm")

    def run_dispenser(self, dispenser_id: str) -> None:
        contact_joints = load_press_contact_joints(self.args.calibration, dispenser_id)
        print(
            f"[Azas] dispenser {dispenser_id}: press_contact_joints_deg=["
            + ", ".join(f"{value:.2f}" for value in contact_joints)
            + "]"
        )
        contact = self.fkin_posx(contact_joints, f"dispenser {dispenser_id} PRESS_CONTACT")
        pre = list(contact)
        pre[2] += max(float(self.args.pre_lift_m), 0.0) * 1000.0
        pressed = list(contact)
        pressed[2] -= max(float(self.args.press_depth_m), 0.0) * 1000.0
        print(
            f"[Azas] dispenser {dispenser_id}: generated PRE=CONTACT+Z{self.args.pre_lift_m * 1000.0:.1f}mm; "
            f"optional PRESS=CONTACT-Z{self.args.press_depth_m * 1000.0:.1f}mm"
        )
        self.move_posx(pre, f"D{dispenser_id} generated PRESS_PRE", velocity=self.args.travel_velocity, acceleration=self.args.travel_acceleration)
        if self.args.stage in {"contact", "press"}:
            self.move_posx(contact, f"D{dispenser_id} measured PRESS_CONTACT", velocity=self.args.line_velocity, acceleration=self.args.line_acceleration)
        if self.args.stage == "press" and self.args.press_depth_m > 0.0:
            self.move_posx(pressed, f"D{dispenser_id} optional Z press", velocity=self.args.line_velocity, acceleration=self.args.line_acceleration)
            self.move_posx(contact, f"D{dispenser_id} return to PRESS_CONTACT", velocity=self.args.line_velocity, acceleration=self.args.line_acceleration)
        if self.args.return_pre and self.args.stage in {"contact", "press"}:
            self.move_posx(pre, f"D{dispenser_id} return to generated PRESS_PRE", velocity=self.args.line_velocity, acceleration=self.args.line_acceleration)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check measured dispenser press contact positions only.")
    parser.add_argument("--dispenser-ids", default="1", help="comma/space separated dispenser ids, e.g. 1 or 1,2,3,4")
    parser.add_argument("--calibration", type=Path, default=CALIBRATION_CONFIG)
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--stage", choices=["pre", "contact", "press"], default="pre")
    parser.add_argument("--pre-lift-m", type=float, default=0.050)
    parser.add_argument("--press-depth-m", type=float, default=0.0)
    parser.add_argument("--return-pre", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--travel-velocity", type=float, default=12.0)
    parser.add_argument("--travel-acceleration", type=float, default=16.0)
    parser.add_argument("--line-velocity", type=float, default=6.0)
    parser.add_argument("--line-acceleration", type=float, default=10.0)
    parser.add_argument("--wait-service-sec", type=float, default=5.0)
    parser.add_argument("--motion-timeout-sec", type=float, default=80.0)
    parser.add_argument("--verify-target", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--verify-timeout-sec", type=float, default=30.0)
    parser.add_argument("--verify-poll-seconds", type=float, default=0.2)
    parser.add_argument("--target-tolerance-mm", type=float, default=20.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} with --execute")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        dispenser_ids = parse_dispenser_ids(args.dispenser_ids)
    except ValueError as exc:
        print(f"[BLOCKED] {exc}")
        return 2
    print("[Azas] Measured dispenser press contact check")
    print(f"[Azas] dispenser_ids={','.join(dispenser_ids)} stage={args.stage}")
    print("[Azas] no cup/gripper/TCP/re-grasp services are called")
    if not args.execute:
        print("[BLOCKED] --execute is required for this real press-motion check tool")
        return 2
    if args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
        return 2

    checker = PressContactChecker(args)
    try:
        for dispenser_id in dispenser_ids:
            checker.run_dispenser(dispenser_id)
        print("[PASS] measured dispenser press contact check completed")
        return 0
    except Exception as exc:
        print(f"[FAIL] {exc}")
        return 1
    finally:
        checker.close()


if __name__ == "__main__":
    raise SystemExit(main())
