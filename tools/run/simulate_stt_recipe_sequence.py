#!/usr/bin/env python3
"""Simulate a dispenser recipe sequence from an STT/topic message.

This tool does not create robot coordinates.  It only converts a symbolic STT
message (colors or dispenser IDs) into the existing measured dispenser sequence
planner.  By default it is a dry-run and sends no robot motion commands.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path("/home/ssu/Azas")
DEFAULT_COLOR_MAP_FILE = Path("/tmp/azas_dispenser_color_map.env")
CONFIRM_PHRASE = "ENABLE_STT_RECIPE_SEQUENCE_EXECUTE"
COLOR_ALIASES = {
    "red": "red",
    "빨강": "red",
    "빨간": "red",
    "빨간색": "red",
    "yellow": "yellow",
    "노랑": "yellow",
    "노란": "yellow",
    "노란색": "yellow",
    "blue": "blue",
    "파랑": "blue",
    "파란": "blue",
    "파란색": "blue",
    "green": "green",
    "초록": "green",
    "초록색": "green",
    "녹색": "green",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dry-run/execute measured dispenser sequence from STT text or topic.")
    parser.add_argument("--message", default="", help="STT/decision message to simulate, e.g. red,yellow,blue")
    parser.add_argument("--topic", default="", help="subscribe once to a std_msgs/String topic instead of --message")
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    parser.add_argument("--color-map", default="auto", help="red=4,yellow=1,blue=3,green=2 or auto")
    parser.add_argument("--color-map-file", type=Path, default=DEFAULT_COLOR_MAP_FILE)
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--execute", action="store_true", help="forward to real motion sequence")
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} with --execute")
    return parser.parse_args()


def read_topic_once(topic: str, timeout_sec: float) -> str:
    import rclpy
    from std_msgs.msg import String

    rclpy.init(args=None)
    node = rclpy.create_node("azas_stt_recipe_sequence_simulator")
    received: list[str] = []

    def cb(msg: String) -> None:
        received.append(msg.data)

    node.create_subscription(String, topic, cb, 10)
    deadline = node.get_clock().now().nanoseconds + int(max(timeout_sec, 0.1) * 1e9)
    try:
        while rclpy.ok() and not received and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not received:
        raise TimeoutError(f"no std_msgs/String received on {topic} within {timeout_sec:.1f}s")
    return received[0]


def flatten_json_text(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    if isinstance(value, (int, float)):
        return [str(value)]
    if isinstance(value, list):
        out: list[str] = []
        for item in value:
            out.extend(flatten_json_text(item))
        return out
    if isinstance(value, dict):
        for key in ("dispenser_ids", "colors", "recipe_dispenser_ids", "sequence", "order"):
            if key in value:
                return flatten_json_text(value[key])
        return flatten_json_text(list(value.values()))
    return [str(value)]


def extract_dispenser_order(message: str) -> list[str]:
    raw = message.strip()
    if not raw:
        raise ValueError("empty STT message")
    parts: list[str]
    try:
        parsed = json.loads(raw)
        parts = flatten_json_text(parsed)
    except json.JSONDecodeError:
        parts = re.split(r"[^0-9A-Za-z가-힣]+", raw)
    order: list[str] = []
    for part in parts:
        token = str(part).strip().lower()
        if not token:
            continue
        if token in {"1", "2", "3", "4"}:
            order.append(token)
            continue
        if token in COLOR_ALIASES:
            order.append(COLOR_ALIASES[token])
            continue
        # Accept compact English tokens accidentally preserved by simple STT strings.
        for alias, color in COLOR_ALIASES.items():
            if re.fullmatch(alias, token):
                order.append(color)
                break
    if not order:
        raise ValueError(f"no dispenser colors/IDs found in message: {message!r}")
    return order


def resolve_color_map(raw: str, color_map_file: Path) -> str:
    if raw.strip().lower() != "auto":
        return raw.strip()
    if color_map_file.exists():
        for line in color_map_file.read_text(encoding="utf-8").splitlines():
            if line.startswith("DISPENSER_COLOR_MAP="):
                value = line.split("=", 1)[1].strip()
                if value:
                    return value
    return "auto"


def main() -> int:
    args = parse_args()
    if args.topic:
        try:
            message = read_topic_once(args.topic, args.timeout_sec)
        except Exception as exc:
            print(f"[FAIL] {exc}")
            return 1
    else:
        message = args.message
    try:
        order = extract_dispenser_order(message)
    except ValueError as exc:
        print(f"[FAIL] {exc}")
        return 2
    dispenser_ids = ",".join(order)
    color_map = resolve_color_map(args.color_map, args.color_map_file)
    print("[Azas] STT recipe sequence simulation")
    print(f"[Azas] message={message}")
    print(f"[Azas] parsed_order={dispenser_ids}")
    print(f"[Azas] color_map={color_map}")

    cmd = [
        "python3",
        str(ROOT / "tools" / "run" / "run_measured_dispenser_recipe_sequence.py"),
        "--service-prefix",
        args.service_prefix,
        "--dispenser-ids",
        dispenser_ids,
        "--color-map",
        color_map,
        "--no-precheck-ikin",
    ]
    if args.execute:
        if args.confirm != CONFIRM_PHRASE:
            print(f"[BLOCKED] --confirm must be exactly {CONFIRM_PHRASE}")
            return 2
        cmd.extend(["--execute", "--confirm", "ENABLE_MEASURED_DISPENSER_RECIPE_SEQUENCE"])
    else:
        print("[DRY-RUN] not executing robot motion")
    return subprocess.run(cmd).returncode


if __name__ == "__main__":
    raise SystemExit(main())
