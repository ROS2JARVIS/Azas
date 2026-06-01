#!/usr/bin/env python3
"""Record current Doosan joints as the dispenser color-scan observation pose."""

from __future__ import annotations

import argparse
import shutil
import time
from pathlib import Path

import rclpy
import yaml
from dsr_msgs2.srv import GetCurrentPosj, GetCurrentPosx

ROOT = Path("/home/ssu/Azas")
DEFAULT_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "dispenser_color_scan.yaml"
CONFIRM_PHRASE = "ENABLE_TEACH_DISPENSER_COLOR_SCAN_POSE"


def service_name(prefix: str, suffix: str) -> str:
    clean_prefix = prefix.strip("/")
    clean_suffix = suffix.strip("/")
    return f"/{clean_prefix}/{clean_suffix}" if clean_prefix else f"/{clean_suffix}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="No-motion helper: save current joint pose for color scan.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--backup", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sync-perception-config", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} with --write")
    return parser.parse_args()


def read_current_pose(service_prefix: str, timeout_sec: float) -> tuple[list[float], list[float]]:
    rclpy.init(args=None)
    node = rclpy.create_node("azas_teach_dispenser_color_scan_pose")
    try:
        timeout_sec = max(timeout_sec, 0.1)
        posj_client = node.create_client(GetCurrentPosj, service_name(service_prefix, "aux_control/get_current_posj"))
        if not posj_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("get_current_posj service not available")
        posj_future = posj_client.call_async(GetCurrentPosj.Request())
        rclpy.spin_until_future_complete(node, posj_future, timeout_sec=timeout_sec)
        if not posj_future.done():
            raise RuntimeError("get_current_posj timeout")
        posj_response = posj_future.result()
        if posj_response is None or not posj_response.success or len(posj_response.pos) < 6:
            raise RuntimeError("GetCurrentPosj returned success=false or too few joints")

        posx_client = node.create_client(GetCurrentPosx, service_name(service_prefix, "aux_control/get_current_posx"))
        if not posx_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("get_current_posx service not available")
        posx_req = GetCurrentPosx.Request()
        posx_req.ref = 0
        posx_future = posx_client.call_async(posx_req)
        rclpy.spin_until_future_complete(node, posx_future, timeout_sec=timeout_sec)
        if not posx_future.done():
            raise RuntimeError("get_current_posx timeout")
        posx_response = posx_future.result()
        if posx_response is None or not posx_response.success or not posx_response.task_pos_info:
            raise RuntimeError("GetCurrentPosx returned success=false or empty task_pos_info")
        posx_values = list(posx_response.task_pos_info[0].data)
        if len(posx_values) < 6:
            raise RuntimeError(f"GetCurrentPosx returned too few values: {posx_values}")

        return (
            [float(value) for value in posj_response.pos[:6]],
            [float(value) for value in posx_values[:6]],
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    args = parse_args()
    if args.write and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
        return 2
    if not args.config.exists():
        print(f"[FAIL] config not found: {args.config}")
        return 2
    joints, tcp_posx = read_current_pose(args.service_prefix, args.timeout_sec)
    print("[Azas] Current dispenser color-scan joint candidate")
    print("[Azas] joints_deg=[" + ", ".join(f"{value:.6f}" for value in joints) + "]")
    print("[Azas] tcp_posx_mm_deg=[" + ", ".join(f"{value:.6f}" for value in tcp_posx) + "]")
    print("[Azas] No motion was commanded; this is measured direct-teaching data.")
    data = yaml.safe_load(args.config.read_text(encoding="utf-8")) or {}
    data.setdefault("metadata", {})["status"] = "measured_draft"
    data["metadata"]["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    data["color_scan_pose"] = {
        "joints_deg": [round(value, 6) for value in joints],
        "tcp_posx_mm_deg": [round(value, 6) for value in tcp_posx],
        "note": "operator taught pose for RealSense dispenser color detection; tcp_posx is used for safety-zone precheck",
    }
    if not args.write:
        print("[DRY-RUN] --write not set; config was not modified.")
        print(f"[Azas] To write: --write --confirm {CONFIRM_PHRASE}")
        return 0
    if args.backup:
        backup = args.config.with_suffix(args.config.suffix + f".bak-{time.strftime('%Y%m%d-%H%M%S')}")
        shutil.copy2(args.config, backup)
        print(f"[Azas] backup={backup}")
    args.config.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print(f"[PASS] updated color_scan_pose in {args.config}")
    if args.sync_perception_config and args.config == DEFAULT_CONFIG:
        perception_config = ROOT / "src" / "azas_perception" / "config" / args.config.name
        if perception_config.parent.is_dir():
            shutil.copy2(args.config, perception_config)
            print(f"[Azas] synced perception config={perception_config}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
