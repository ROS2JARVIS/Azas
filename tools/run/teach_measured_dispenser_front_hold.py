#!/usr/bin/env python3
"""Record the current taught link_6 pose as a dispenser front-hold pose.

This is a no-motion teaching helper.  Put the real robot in the desired
side-grip dispenser-front pose by direct teaching, then run this script to copy
the measured base_link -> link_6 TF into measured_dispenser_collision.yaml.
It does not ask for, invent, or hardcode cup coordinates.
"""

from __future__ import annotations

import argparse
import math
import re
import shutil
import sys
import time
from pathlib import Path

import yaml
import rclpy
from tf2_ros import Buffer, TransformException, TransformListener


ROOT = Path("/home/ssu/Azas")
DEFAULT_CONFIG = ROOT / "src" / "azas_bringup" / "config" / "measured_dispenser_collision.yaml"
CONFIRM_PHRASE = "ENABLE_TEACH_MEASURED_DISPENSER_FRONT_HOLD"
# Measured relative offsets from taught front-hold link_6 poses to conservative
# dispenser body/nozzle collision boxes. Reapplied when the front line is re-taught.
# Kept for explicit maintenance use only.  Normal color recognition must not
# re-anchor cup/front-hold/collision coordinates; it only maps color names to
# existing physical dispenser slots.
COLLISION_OFFSETS_BY_SLOT = {
    1: {"body": [0.1396, -0.0393, 0.0380], "nozzle": [0.0090, -0.0025, 0.3390]},
    2: {"body": [0.1396, -0.0393, 0.0340], "nozzle": [0.0090, -0.0025, 0.3350]},
    3: {"body": [0.1396, -0.0393, 0.0240], "nozzle": [0.0090, -0.0025, 0.3250]},
    4: {"body": [0.1396, -0.0393, 0.0220], "nozzle": [0.0090, -0.0025, 0.3230]},
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "No-motion teaching helper: record current base_link->link_6 TF into "
            "front_hold_poses.dispenser_N."
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--dispenser-id", type=int, choices=(1, 2, 3, 4), required=True)
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--target-frame", default="link_6")
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--write", action="store_true", help="write the taught pose into the YAML file")
    parser.add_argument("--backup", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sync-perception-config", action=argparse.BooleanOptionalAction, default=True, help="also copy the updated YAML to src/azas_perception/config for package consistency")
    parser.add_argument("--update-collision-from-front", action=argparse.BooleanOptionalAction, default=False, help="explicit maintenance only: re-anchor dispenser body/nozzle collision boxes from measured front_hold poses")
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} with --write")
    return parser.parse_args()


def quat_to_rpy_deg(x: float, y: float, z: float, w: float) -> list[float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def read_tf(base_frame: str, target_frame: str, timeout_sec: float):
    rclpy.init(args=None)
    node = rclpy.create_node("azas_teach_measured_dispenser_front_hold")
    buffer = Buffer()
    listener = TransformListener(buffer, node)  # noqa: F841 - keeps subscription alive
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    try:
        last_error: Exception | None = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                return buffer.lookup_transform(base_frame, target_frame, rclpy.time.Time())
            except TransformException as exc:
                last_error = exc
        raise RuntimeError(
            f"TF lookup {base_frame}->{target_frame} timed out after {timeout_sec:.1f}s: {last_error}"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def replace_front_hold_block(
    text: str,
    *,
    dispenser_id: int,
    position: list[float],
    quaternion: list[float],
    rpy_deg: list[float],
) -> str:
    key = f"dispenser_{dispenser_id}"
    pattern = re.compile(
        rf"(?P<prefix>^  {re.escape(key)}:\n)"
        r"(?P<body>(?:^    .*\n){3})",
        re.MULTILINE,
    )
    replacement = (
        rf"\g<prefix>"
        f"    position_xyz_m: [{position[0]:.6f}, {position[1]:.6f}, {position[2]:.6f}]\n"
        f"    quaternion_xyzw: [{quaternion[0]:.6f}, {quaternion[1]:.6f}, {quaternion[2]:.6f}, {quaternion[3]:.6f}]\n"
        f"    rpy_deg: [{rpy_deg[0]:.3f}, {rpy_deg[1]:.3f}, {rpy_deg[2]:.3f}]\n"
    )
    new_text, count = pattern.subn(replacement, text, count=1)
    if count != 1:
        raise RuntimeError(f"could not find front_hold_poses.{key} block in config")
    return new_text


def update_collision_objects_from_front_holds(text: str) -> str:
    data = yaml.safe_load(text) or {}
    poses = data.get("front_hold_poses") or {}
    objects = data.setdefault("estimated_collision_objects", {})
    for slot, offsets in COLLISION_OFFSETS_BY_SLOT.items():
        front = poses.get(f"dispenser_{slot}") or {}
        position = front.get("position_xyz_m")
        if not isinstance(position, list) or len(position) < 3:
            continue
        body = objects.get(f"dispenser_{slot}_body_box_v2")
        nozzle = objects.get(f"dispenser_{slot}_head_nozzle_box")
        if isinstance(body, dict):
            body["center_xyz_m"] = [round(float(position[i]) + offsets["body"][i], 4) for i in range(3)]
        if isinstance(nozzle, dict):
            nozzle["center_xyz_m"] = [round(float(position[i]) + offsets["nozzle"][i], 4) for i in range(3)]
    return yaml.safe_dump(data, sort_keys=False, allow_unicode=True)


def main() -> int:
    args = parse_args()
    if not args.config.is_file():
        print(f"[FAIL] config not found: {args.config}")
        return 2
    if args.write and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
        return 2

    transform = read_tf(args.base_frame, args.target_frame, args.timeout_sec)
    t = transform.transform.translation
    q = transform.transform.rotation
    position = [float(t.x), float(t.y), float(t.z)]
    quaternion = [float(q.x), float(q.y), float(q.z), float(q.w)]
    rpy_deg = quat_to_rpy_deg(*quaternion)

    print("[Azas] Current taught front-hold candidate")
    print(f"[Azas] dispenser_id={args.dispenser_id}")
    print(f"[Azas] source_tf={args.base_frame}->{args.target_frame}")
    print(
        "[Azas] position_xyz_m="
        f"[{position[0]:.6f}, {position[1]:.6f}, {position[2]:.6f}]"
    )
    print(
        "[Azas] quaternion_xyzw="
        f"[{quaternion[0]:.6f}, {quaternion[1]:.6f}, {quaternion[2]:.6f}, {quaternion[3]:.6f}]"
    )
    print(f"[Azas] rpy_deg=[{rpy_deg[0]:.3f}, {rpy_deg[1]:.3f}, {rpy_deg[2]:.3f}]")
    print("[Azas] No motion was commanded; this is measured direct-teaching data.")

    text = args.config.read_text(encoding="utf-8")
    updated = replace_front_hold_block(
        text,
        dispenser_id=args.dispenser_id,
        position=position,
        quaternion=quaternion,
        rpy_deg=rpy_deg,
    )

    if args.update_collision_from_front:
        updated = update_collision_objects_from_front_holds(updated)
        print("[Azas] Re-anchored dispenser body/nozzle collision boxes from measured front_hold poses.")
    else:
        print("[Azas] Collision boxes were NOT changed; color/slot mapping must not alter cup/front-hold/collision coordinates.")

    if not args.write:
        print("[DRY-RUN] --write not set; config was not modified.")
        print(f"[Azas] To write: --write --confirm {CONFIRM_PHRASE}")
        return 0

    if args.backup:
        backup = args.config.with_suffix(args.config.suffix + f".bak-{time.strftime('%Y%m%d-%H%M%S')}")
        shutil.copy2(args.config, backup)
        print(f"[Azas] backup={backup}")
    args.config.write_text(updated, encoding="utf-8")
    print(f"[PASS] updated front_hold_poses.dispenser_{args.dispenser_id} in {args.config}")
    if args.sync_perception_config and args.config == DEFAULT_CONFIG:
        perception_config = ROOT / "src" / "azas_perception" / "config" / args.config.name
        if perception_config.parent.is_dir():
            shutil.copy2(args.config, perception_config)
            print(f"[Azas] synced perception config={perception_config}")
    print("[Azas] Restart start_collision_scene/RViz publishers if you need refreshed markers.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
