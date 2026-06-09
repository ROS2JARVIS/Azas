#!/usr/bin/env python3
"""색상 레시피 시퀀스 실행.

outputs/latest_recipe.json (색깔 목록) + outputs/dispenser_color_map.json (위치→색깔)
를 읽어 색깔→디스펜서 ID를 매핑한 뒤 run_measured_dispenser_recipe_sequence.py 실행.

사용법:
  python3 tools/run/run_color_recipe_sequence.py
  python3 tools/run/run_color_recipe_sequence.py --colors red:2,blue:1  # 직접 지정
  python3 tools/run/run_color_recipe_sequence.py --dispenser-ids 1x1,2x2,3x1
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
COLOR_MAP_PATH  = ROOT / "outputs" / "dispenser_color_map.json"
RECIPE_PATH     = ROOT / "outputs" / "latest_recipe.json"
SEQUENCE_SCRIPT = ROOT / "tools" / "run" / "run_measured_dispenser_recipe_sequence.py"
CONFIRM_PHRASE  = "ENABLE_MEASURED_DISPENSER_RECIPE_SEQUENCE"
FALLBACK_DISPENSER_SEQUENCE = ["1", "2", "3", "4"]


def load_color_map() -> dict[str, str]:
    """dispenser_id → color_name 매핑 로드. 정상 맵이 없으면 빈 dict로 fallback."""
    if not COLOR_MAP_PATH.exists():
        print(f"[run_color_recipe] 색상 맵 없음: {COLOR_MAP_PATH}; fallback 1,2,3,4 사용", file=sys.stderr)
        return {}
    data = json.loads(COLOR_MAP_PATH.read_text(encoding="utf-8"))
    mapped = {str(k): str(v).lower().strip() for k, v in data.items()}
    if not mapped or all(v == "unknown" for v in mapped.values()):
        print("[run_color_recipe] 색상 맵이 비어 있거나 전부 unknown; fallback 1,2,3,4 사용", file=sys.stderr)
        return {}
    return mapped


def color_to_dispenser_id(color: str, color_map: dict[str, str]) -> str | None:
    """색깔 이름 → 디스펜서 ID (없으면 None)."""
    color = color.lower().strip()
    for did, c in color_map.items():
        if c == color:
            return did
    return None


def parse_colors_arg(raw: str) -> list[tuple[str, int]]:
    """'red:2,blue:1' → [('red', 2), ('blue', 1)]."""
    result = []
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        if ":" in part:
            c, n = part.split(":", 1)
            result.append((c.strip().lower(), int(n.strip())))
        else:
            result.append((part.lower(), 1))
    return result


def parse_direct_dispenser_sequence(raw: str) -> list[str]:
    """Parse physical dispenser input.

    Accepted forms:
      1,2,2,3
      1x1,2x2,3x1
      1:1,2:2,3:1
    """
    result: list[str] = []
    for part in raw.replace(";", ",").split(","):
        item = part.strip().lower()
        if not item:
            continue
        if "x" in item:
            dispenser_id, count_raw = item.split("x", 1)
        elif ":" in item:
            dispenser_id, count_raw = item.split(":", 1)
        else:
            dispenser_id, count_raw = item, "1"
        dispenser_id = dispenser_id.strip()
        if dispenser_id not in {"1", "2", "3", "4"}:
            raise ValueError(f"unsupported dispenser id: {dispenser_id!r}")
        try:
            count = int(count_raw.strip())
        except ValueError as exc:
            raise ValueError(f"invalid count for dispenser {dispenser_id}: {count_raw!r}") from exc
        if count < 1:
            raise ValueError(f"count must be >= 1 for dispenser {dispenser_id}")
        result.extend([dispenser_id] * count)
    if not result:
        raise ValueError("direct dispenser input is empty")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--colors", default="",
                        help="직접 색깔 지정: 'red:2,blue:1' (생략 시 latest_recipe.json 사용)")
    parser.add_argument("--dispenser-ids", default="",
                        help="직접 물리 디스펜서 지정: '1,2,2,3' 또는 '1x1,2x2,3x1'")
    parser.add_argument("--confirm", action="store_true",
                        help=f"확인 구문({CONFIRM_PHRASE}) 자동 전달")
    parser.add_argument("--execute", action="store_true",
                        help="실제 measured dispenser sequence를 실행")
    parser.add_argument("--press-min-transit-z-m", default="0.720")
    parser.add_argument("--press-line-velocity", default="18.0")
    parser.add_argument("--press-line-acceleration", default="25.0")
    parser.add_argument("--press-travel-velocity", default="45.0")
    parser.add_argument("--press-travel-acceleration", default="60.0")
    parser.add_argument("--press-contact-joint-velocity", default="22.0")
    parser.add_argument("--press-contact-joint-acceleration", default="30.0")
    parser.add_argument("--gripper-open-settle-seconds", default="1.5")
    parser.add_argument("--gripper-settle-seconds", default="0.8")
    parser.add_argument("--wait-service-sec", default="15.0")
    parser.add_argument("--pose-read-retries", default="3")
    parser.add_argument("--pose-read-retry-sleep-sec", default="0.5")
    args = parser.parse_args()
    if args.execute and not args.confirm:
        print(f"[BLOCKED] --execute requires --confirm ({CONFIRM_PHRASE})", file=sys.stderr)
        return 2

    sequence_extra_args = [
        "--press-min-transit-z-m", str(args.press_min_transit_z_m),
        "--press-line-velocity", str(args.press_line_velocity),
        "--press-line-acceleration", str(args.press_line_acceleration),
        "--press-travel-velocity", str(args.press_travel_velocity),
        "--press-travel-acceleration", str(args.press_travel_acceleration),
        "--press-contact-joint-velocity", str(args.press_contact_joint_velocity),
        "--press-contact-joint-acceleration", str(args.press_contact_joint_acceleration),
        "--gripper-open-settle-seconds", str(args.gripper_open_settle_seconds),
        "--gripper-settle-seconds", str(args.gripper_settle_seconds),
        "--wait-service-sec", str(args.wait_service_sec),
        "--pose-read-retries", str(args.pose_read_retries),
        "--pose-read-retry-sleep-sec", str(args.pose_read_retry_sleep_sec),
    ]

    direct_dispenser_ids = args.dispenser_ids.strip()
    if direct_dispenser_ids:
        try:
            sequence = parse_direct_dispenser_sequence(direct_dispenser_ids)
        except ValueError as exc:
            print(f"[run_color_recipe] 잘못된 직접 입력: {exc}", file=sys.stderr)
            return 1
        dispenser_ids_str = ",".join(sequence)
        print(f"[run_color_recipe] 직접 디스펜서 실행 순서: {dispenser_ids_str}")
        cmd = [
            sys.executable, str(SEQUENCE_SCRIPT),
            "--dispenser-ids", dispenser_ids_str,
            *sequence_extra_args,
        ]
        if args.execute:
            cmd += ["--execute"]
        if args.confirm:
            cmd += ["--confirm", CONFIRM_PHRASE]
        print(f"[run_color_recipe] 실행: {' '.join(cmd)}")
        result = subprocess.run(cmd, check=False)
        return result.returncode

    color_map = load_color_map()
    print(f"[run_color_recipe] 색상 맵: {color_map if color_map else 'fallback 1,2,3,4'}")

    if not color_map:
        dispenser_ids_str = ",".join(FALLBACK_DISPENSER_SEQUENCE)
        print(f"[run_color_recipe] fallback 직접 디스펜서 실행 순서: {dispenser_ids_str}")
        cmd = [sys.executable, str(SEQUENCE_SCRIPT), "--dispenser-ids", dispenser_ids_str, *sequence_extra_args]
        if args.execute:
            cmd += ["--execute"]
        if args.confirm:
            cmd += ["--confirm", CONFIRM_PHRASE]
        print(f"[run_color_recipe] 실행: {' '.join(cmd)}")
        result = subprocess.run(cmd, check=False)
        return result.returncode

    # 색깔+펌프 수 결정
    if args.colors:
        color_pumps = parse_colors_arg(args.colors)
    else:
        if not RECIPE_PATH.exists():
            print(f"[run_color_recipe] 레시피 없음: {RECIPE_PATH}", file=sys.stderr)
            print("[run_color_recipe] listen_stt_recipe 스텝을 먼저 실행하세요.", file=sys.stderr)
            return 1
        recipe = json.loads(RECIPE_PATH.read_text(encoding="utf-8"))
        colors = recipe.get("colors", [])
        pumps  = recipe.get("pumps", {})
        color_pumps = [(c, int(pumps.get(c, 1))) for c in colors]

    print(f"[run_color_recipe] 레시피 색깔+펌프: {color_pumps}")

    # 색깔 → 디스펜서 ID 매핑
    sequence: list[str] = []
    for color, pumps in color_pumps:
        did = color_to_dispenser_id(color, color_map)
        if did is None:
            print(f"[run_color_recipe] '{color}' 색깔이 색상 맵에 없음 → 건너뜀", file=sys.stderr)
            continue
        for _ in range(pumps):
            sequence.append(did)

    if not sequence:
        print("[run_color_recipe] 실행할 디스펜서 없음 (색상 맵과 레시피 색깔이 불일치)", file=sys.stderr)
        return 1

    dispenser_ids_str = ",".join(sequence)
    print(f"[run_color_recipe] 실행 순서: {dispenser_ids_str}")

    cmd = [
        sys.executable, str(SEQUENCE_SCRIPT),
        "--dispenser-ids", dispenser_ids_str,
        *sequence_extra_args,
    ]
    if args.execute:
        cmd += ["--execute"]
    if args.confirm:
        cmd += ["--confirm", CONFIRM_PHRASE]

    print(f"[run_color_recipe] 실행: {' '.join(cmd)}")
    result = subprocess.run(cmd, check=False)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
