#!/usr/bin/env python3
"""Record the current robot pose as a measured dispenser press teach point.

This helper commands no motion.  The operator must jog/teach the real robot to
the intended dispenser pre/contact pose first; this script only records the
current Doosan services into calibration.yaml.
"""

from __future__ import annotations

import argparse
import math
import re
import shutil
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from dsr_msgs2.srv import GetCurrentPosj, GetCurrentPosx

ROOT = Path("/home/ssu/Azas")
DEFAULT_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"
CONFIRM = "ENABLE_RECORD_DISPENSER_PRESS_POSE"


def service_name(prefix: str, suffix: str) -> str:
    clean_prefix = prefix.strip("/")
    clean_suffix = suffix.strip("/")
    return f"/{clean_prefix}/{clean_suffix}" if clean_prefix else f"/{clean_suffix}"


def call_service(node: Any, client: Any, request: Any, *, timeout_sec: float, label: str) -> Any:
    if not client.wait_for_service(timeout_sec=max(timeout_sec, 0.1)):
        raise RuntimeError(f"{label} service unavailable: {client.srv_name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=max(timeout_sec, 0.1))
    if not future.done():
        raise RuntimeError(f"{label} timed out after {timeout_sec:.1f}s")
    if future.exception() is not None:
        raise RuntimeError(f"{label} exception: {future.exception()}")
    result = future.result()
    if result is None:
        raise RuntimeError(f"{label} returned no result")
    return result


def read_current_pose(service_prefix: str, timeout_sec: float) -> tuple[list[float], list[float]]:
    rclpy.init(args=None)
    node = rclpy.create_node("azas_record_dispenser_press_pose")
    try:
        posx_client = node.create_client(GetCurrentPosx, service_name(service_prefix, "aux_control/get_current_posx"))
        posj_client = node.create_client(GetCurrentPosj, service_name(service_prefix, "aux_control/get_current_posj"))

        posx_req = GetCurrentPosx.Request()
        posx_req.ref = 0
        posx_res = call_service(node, posx_client, posx_req, timeout_sec=timeout_sec, label="GetCurrentPosx")
        if not posx_res.success or not posx_res.task_pos_info:
            raise RuntimeError("GetCurrentPosx returned success=false or empty task_pos_info")
        posx = [float(value) for value in list(posx_res.task_pos_info[0].data)[:6]]
        if len(posx) != 6:
            raise RuntimeError(f"GetCurrentPosx returned too few values: {posx}")

        posj_res = call_service(node, posj_client, GetCurrentPosj.Request(), timeout_sec=timeout_sec, label="GetCurrentPosj")
        posj = [float(value) for value in list(posj_res.pos)[:6]]
        if not posj_res.success or len(posj) != 6:
            raise RuntimeError(f"GetCurrentPosj returned success=false or too few values: {posj}")
        return posx, posj
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def format_list(values: list[float], precision: int) -> str:
    return "[" + ", ".join(f"{value:.{precision}f}" for value in values) + "]"


def replace_block_value(text: str, dispenser_id: str, key: str, value: str, comment: str = "") -> str:
    header_pattern = re.compile(rf'^  "{re.escape(dispenser_id)}":\n', re.M)
    match = header_pattern.search(text)
    if not match:
        raise RuntimeError(f'dispenser_outlets."{dispenser_id}" block not found')
    next_match = re.search(r'^(?:  "\d+":|[A-Za-z_][A-Za-z0-9_]*:)\n', text[match.end():], re.M)
    block_end = match.end() + next_match.start() if next_match else len(text)
    block = text[match.end():block_end]
    line_pattern = re.compile(rf"^    {re.escape(key)}: .*$", re.M)
    replacement = f"    {key}: {value}{comment}"
    if line_pattern.search(block):
        block = line_pattern.sub(replacement, block, count=1)
    else:
        block = block.rstrip() + "\n" + replacement + "\n"
    return text[:match.end()] + block + text[block_end:]


def update_config(config_path: Path, dispenser_id: str, kind: str, posx: list[float], posj: list[float]) -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    backup_path = config_path.with_suffix(config_path.suffix + f".bak-{stamp}")
    shutil.copy2(config_path, backup_path)

    text = config_path.read_text(encoding="utf-8")
    measured_comment = f"  # operator measured, {datetime.now().date().isoformat()}"
    if kind == "contact":
        xyz_m = [value / 1000.0 for value in posx[:3]]
        rpy_deg = posx[3:6]
        rpy_rad = [math.radians(value) for value in rpy_deg]
        text = replace_block_value(text, dispenser_id, "press_pose_xyz_m", format_list(xyz_m, 6))
        text = replace_block_value(text, dispenser_id, "press_pose_rpy_deg", format_list(rpy_deg, 3))
        text = replace_block_value(text, dispenser_id, "press_pose_rpy_rad", format_list(rpy_rad, 6))
        text = replace_block_value(
            text,
            dispenser_id,
            "press_contact_joints_deg",
            format_list(posj, 2),
            measured_comment,
        )
        text = replace_block_value(
            text,
            dispenser_id,
            "press_contact_status",
            "measured_confirmed",
            measured_comment,
        )
    elif kind == "pre":
        text = replace_block_value(
            text,
            dispenser_id,
            "press_pre_joints_deg",
            format_list(posj, 2),
            measured_comment,
        )
    elif kind == "common_pre":
        text = replace_block_value(
            text,
            dispenser_id,
            "press_common_pre_joints_deg",
            format_list(posj, 2),
            measured_comment,
        )
    elif kind == "cup_common_pre":
        text = replace_block_value(
            text,
            dispenser_id,
            "cup_common_pre_joints_deg",
            format_list(posj, 2),
            measured_comment,
        )
    elif kind == "cup_pre":
        text = replace_block_value(
            text,
            dispenser_id,
            "cup_pre_place_joints_deg",
            format_list(posj, 2),
            measured_comment,
        )
    elif kind == "cup_place":
        text = replace_block_value(
            text,
            dispenser_id,
            "cup_place_joints_deg",
            format_list(posj, 2),
            measured_comment,
        )
        text = replace_block_value(
            text,
            dispenser_id,
            "cup_place_status",
            "measured_confirmed",
            measured_comment,
        )
    else:
        raise RuntimeError(f"unsupported kind: {kind}")

    config_path.write_text(text, encoding="utf-8")
    return backup_path


def warn_lane_mismatch(dispenser_id: str, kind: str, posx: list[float], tolerance_mm: float = 25.0) -> None:
    """Warn when the recorded pose sits laterally over a different dispenser lane.

    Guards against the slot mix-ups observed on 2026-06-10 where press teach
    poses were saved under neighboring dispenser ids. Warning only; the
    operator decides.
    """
    collision_config = ROOT / "src" / "azas_bringup" / "config" / "measured_dispenser_collision.yaml"
    try:
        import yaml

        data = yaml.safe_load(collision_config.read_text(encoding="utf-8")) or {}
        front_holds = data.get("front_hold_poses") or {}
        lane_y_mm = {
            key.split("_")[-1]: float(value["position_xyz_m"][1]) * 1000.0
            for key, value in front_holds.items()
        }
    except Exception:
        return
    expected = lane_y_mm.get(dispenser_id)
    if expected is None:
        return
    actual = posx[1]
    delta = actual - expected
    nearest = min(lane_y_mm.items(), key=lambda kv: abs(kv[1] - actual))
    if abs(delta) > tolerance_mm:
        print(
            f"[WARN] {kind} y={actual:.1f}mm is {abs(delta):.0f}mm away from dispenser "
            f"{dispenser_id} lane (expected y~{expected:.1f}mm); nearest lane is "
            f"dispenser {nearest[0]}. Check the dispenser id before trusting this record."
        )
    else:
        print(f"[Azas] lane check OK: y={actual:.1f}mm matches dispenser {dispenser_id} (expected ~{expected:.1f}mm)")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dispenser-id", required=True, choices=["1", "2", "3", "4"])
    parser.add_argument(
        "--kind",
        required=True,
        choices=["pre", "common_pre", "contact", "cup_common_pre", "cup_pre", "cup_place"],
    )
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if args.write and args.confirm != CONFIRM:
        print(f"[BLOCKED] --write requires --confirm {CONFIRM}", file=sys.stderr)
        return 2
    if not args.config.is_file():
        print(f"[FAIL] config not found: {args.config}", file=sys.stderr)
        return 2

    if not args.write:
        print(f"[BLOCKED] --write --confirm {CONFIRM} is required; no dry-run recording mode is allowed.", file=sys.stderr)
        return 2

    posx, posj = read_current_pose(args.service_prefix, args.timeout_sec)
    print(f"[Azas] dispenser={args.dispenser_id} kind={args.kind}")
    print(f"[Azas] current_posx_mm_deg={format_list(posx, 3)}")
    print(f"[Azas] current_posj_deg={format_list(posj, 2)}")
    if args.kind == "contact":
        print(f"[Azas] calibration press_pose_xyz_m={format_list([value / 1000.0 for value in posx[:3]], 6)}")
        print(f"[Azas] calibration press_pose_rpy_deg={format_list(posx[3:6], 3)}")
        print(f"[Azas] calibration press_contact_joints_deg={format_list(posj, 2)}")
    elif args.kind == "pre":
        print(f"[Azas] calibration press_pre_joints_deg={format_list(posj, 2)}")
    elif args.kind == "common_pre":
        print(f"[Azas] calibration press_common_pre_joints_deg={format_list(posj, 2)}")
    elif args.kind == "cup_common_pre":
        print(f"[Azas] calibration cup_common_pre_joints_deg={format_list(posj, 2)}")
    elif args.kind == "cup_pre":
        print(f"[Azas] calibration cup_pre_place_joints_deg={format_list(posj, 2)}")
    else:
        print(f"[Azas] calibration cup_place_joints_deg={format_list(posj, 2)}")
    warn_lane_mismatch(args.dispenser_id, args.kind, posx)

    backup_path = update_config(args.config, args.dispenser_id, args.kind, posx, posj)
    print(f"[PASS] updated {args.config}")
    print(f"[Azas] backup={backup_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
