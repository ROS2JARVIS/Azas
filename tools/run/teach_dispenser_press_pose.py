#!/usr/bin/env python3
"""Record current Doosan TCP posx as a measured dispenser press pose.

No motion is commanded. Move/teach the real robot TCP to the desired pump-top
pose for a physical dispenser slot, then run this helper to copy the measured
GetCurrentPosx value into measured_dispenser_press.yaml.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path("/home/ssu/Azas")
DEFAULT_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "measured_dispenser_press.yaml"
CONFIRM_PHRASE = "ENABLE_TEACH_DISPENSER_PRESS_POSE"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="No-motion teaching helper: record current GetCurrentPosx into press_poses.dispenser_N."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--dispenser-id", type=int, choices=(1, 2, 3, 4), required=True)
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--backup", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sync-perception-config", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} with --write")
    return parser.parse_args()


def service_name(prefix: str, suffix: str) -> str:
    clean_prefix = prefix.strip("/")
    clean_suffix = suffix.strip("/")
    return f"/{clean_prefix}/{clean_suffix}" if clean_prefix else f"/{clean_suffix}"


def parse_numeric_array(text: str) -> list[float]:
    match = re.search(r"(?:data|pos)[:=]\s*(?:array\()?\[([^\]]+)\]", text, re.S)
    if match:
        values = re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", match.group(1))
        return [float(value) for value in values]
    values: list[str] = []
    in_data = False
    for line in text.splitlines():
        stripped = line.strip()
        if re.match(r"(?:data|pos):\s*$", stripped):
            in_data = True
            continue
        if in_data:
            item = re.match(r"-\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)", stripped)
            if item:
                values.append(item.group(1))
                if len(values) >= 6:
                    break
                continue
            if stripped and not stripped.startswith("-"):
                break
    return [float(value) for value in values]


def read_current_posx(service_prefix: str) -> list[float]:
    service = service_name(service_prefix, "aux_control/get_current_posx")
    cmd = [
        "bash",
        "-lc",
        "source /opt/ros/humble/setup.bash && "
        f"ros2 service call {service} dsr_msgs2/srv/GetCurrentPosx '{{ref: 0}}'",
    ]
    proc = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
    if proc.returncode != 0:
        raise RuntimeError(proc.stdout.strip() or f"get_current_posx failed rc={proc.returncode}")
    values = parse_numeric_array(proc.stdout)
    if len(values) < 6:
        raise RuntimeError("GetCurrentPosx response did not contain 6 posx values:\n" + proc.stdout)
    return values[:6]


def main() -> int:
    args = parse_args()
    if args.write and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
        return 2
    if not args.config.exists():
        print(f"[FAIL] config not found: {args.config}")
        return 2

    posx = read_current_posx(args.service_prefix)
    print("[Azas] Current taught dispenser press candidate")
    print(f"[Azas] dispenser_id={args.dispenser_id}")
    print("[Azas] source=/aux_control/get_current_posx active TCP")
    print("[Azas] top_posx_mm_deg=[" + ", ".join(f"{value:.6f}" for value in posx) + "]")
    print("[Azas] No motion was commanded; this is measured direct-teaching data.")

    data = yaml.safe_load(args.config.read_text(encoding="utf-8")) or {}
    data.setdefault("press_poses", {})[f"dispenser_{args.dispenser_id}"] = {
        "top_posx_mm_deg": [round(value, 6) for value in posx],
        "source": "operator_teaching_get_current_posx",
        "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
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
    print(f"[PASS] updated press_poses.dispenser_{args.dispenser_id} in {args.config}")
    if args.sync_perception_config and args.config == DEFAULT_CONFIG:
        perception_config = ROOT / "src" / "azas_perception" / "config" / args.config.name
        if perception_config.parent.is_dir():
            shutil.copy2(args.config, perception_config)
            print(f"[Azas] synced perception config={perception_config}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
