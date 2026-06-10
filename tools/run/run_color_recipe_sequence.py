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
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
COLOR_MAP_PATH  = ROOT / "outputs" / "dispenser_color_map.json"
RECIPE_PATH     = ROOT / "outputs" / "latest_recipe.json"
SEQUENCE_SCRIPT = ROOT / "tools" / "run" / "run_measured_dispenser_recipe_sequence.py"
CONFIRM_PHRASE  = "ENABLE_MEASURED_DISPENSER_RECIPE_SEQUENCE"

def normalize_color_map(data: object) -> dict[str, str]:
    if not isinstance(data, dict):
        return {}
    return {str(k): str(v).lower().strip() for k, v in data.items()}


def load_color_map(*, override_json: str = "") -> dict[str, str]:
    """Load dispenser_id → color_name mapping from the latest scan JSON."""
    if override_json.strip():
        try:
            mapped = normalize_color_map(json.loads(override_json))
        except json.JSONDecodeError as exc:
            print(f"[run_color_recipe] 직접 색상맵 JSON 파싱 실패: {exc}", file=sys.stderr)
            return {}
        if mapped and not all(v == "unknown" for v in mapped.values()):
            print("[run_color_recipe] 패널 직접 색상맵 사용")
            return mapped
        print("[run_color_recipe] 패널 직접 색상맵이 비어 있거나 전부 unknown", file=sys.stderr)
        return {}
    if not COLOR_MAP_PATH.exists():
        print(f"[run_color_recipe] 색상 맵 없음: {COLOR_MAP_PATH}", file=sys.stderr)
        return {}
    data = json.loads(COLOR_MAP_PATH.read_text(encoding="utf-8"))
    mapped = normalize_color_map(data)
    if not mapped or all(v == "unknown" for v in mapped.values()):
        print("[run_color_recipe] 색상 맵이 비어 있거나 전부 unknown", file=sys.stderr)
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
    """Parse color pump input.

    Accepted forms:
      red:2,blue:1
      redx2,bluex1
      red2,blue1
      red,blue
    """
    result: list[tuple[str, int]] = []
    for part in raw.replace(";", ",").split(","):
        item = part.strip().lower()
        if not item:
            continue
        if ":" in item:
            color, count_raw = item.split(":", 1)
        elif "x" in item:
            color, count_raw = item.split("x", 1)
        else:
            match = re.match(r"^([a-zA-Z가-힣_ -]+?)(\d+)?$", item)
            if not match:
                raise ValueError(f"invalid color token: {part!r}")
            color, count_raw = match.group(1), match.group(2) or "1"
        color = color.strip().lower()
        if not color:
            raise ValueError(f"empty color in token: {part!r}")
        try:
            count = int(str(count_raw).strip())
        except ValueError as exc:
            raise ValueError(f"invalid count for color {color}: {count_raw!r}") from exc
        if count < 1:
            raise ValueError(f"count must be >= 1 for color {color}")
        result.append((color, count))
    if not result:
        raise ValueError("color input is empty")
    return result


def parse_recipe_data(recipe: object) -> list[tuple[str, int]]:
    """Return color pump counts from supported recipe JSON shapes.

    Supported examples:
      {"colors": ["red", "green"], "pumps": {"red": 3, "green": 3}}
      {"pumps": {"red": 3, "green": 3}}
      {"red": 3, "green": 3}
      [{"color": "red", "count": 3}, {"color": "green", "pumps": 3}]
    """
    color_pumps: list[tuple[str, int]] = []

    def add(color: object, count: object = 1) -> None:
        color_name = str(color).lower().strip()
        if not color_name:
            return
        try:
            pump_count = int(count)
        except (TypeError, ValueError):
            raise ValueError(f"invalid pump count for color {color_name}: {count!r}")
        if pump_count < 1:
            raise ValueError(f"pump count must be >= 1 for color {color_name}")
        color_pumps.append((color_name, pump_count))

    if isinstance(recipe, list):
        for item in recipe:
            if isinstance(item, dict):
                color = item.get("color") or item.get("name")
                count = item.get("pumps", item.get("count", item.get("presses", 1)))
                if color:
                    add(color, count)
            else:
                add(item, 1)
        return color_pumps

    if not isinstance(recipe, dict):
        raise ValueError("recipe JSON must be an object or list")

    colors = recipe.get("colors")
    pumps = None
    for key in ("pumps", "presses", "counts"):
        if key in recipe:
            pumps = recipe.get(key)
            break

    if isinstance(colors, list):
        if not isinstance(pumps, dict):
            pumps = {}
        for color in colors:
            key = str(color).lower().strip()
            add(key, pumps.get(key, pumps.get(str(color), 1)))
        return color_pumps

    if isinstance(pumps, dict):
        for color, count in pumps.items():
            add(color, count)
        return color_pumps

    # Compact operator JSON: {"red": 3, "green": 3}
    metadata_keys = {"source", "note", "notes", "created_at", "updated_at"}
    for color, count in recipe.items():
        if str(color).lower().strip() in metadata_keys:
            continue
        if isinstance(count, (int, float, str)):
            add(color, count)

    return color_pumps


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
                        help="직접 색깔 지정: 'red:2,blue:1', 'redx2,bluex1', 'red2,blue1' (생략 시 latest_recipe.json 사용)")
    parser.add_argument("--dispenser-ids", default="",
                        help="직접 물리 디스펜서 지정: '1,2,2,3' 또는 '1x1,2x2,3x1'")
    parser.add_argument("--color-map-json", default="",
                        help="패널이 현재 알고 있는 dispenser_id→color JSON. --colors 직접 입력 시 우선 사용")
    parser.add_argument(
        "--confirm",
        nargs="?",
        const=CONFIRM_PHRASE,
        default="",
        help=(
            f"실행 확인. 값 없이 --confirm만 써도 되고, 기존 습관대로 확인 문구를 붙여도 됩니다. "
            f"내부 measured sequence에는 {CONFIRM_PHRASE}를 전달합니다."
        ),
    )
    parser.add_argument("--execute", action="store_true",
                        help="실제 measured dispenser sequence를 실행")
    parser.add_argument("--press-min-transit-z-m", default="0.500")
    parser.add_argument("--press-line-velocity", default="18.0")
    parser.add_argument("--press-line-acceleration", default="25.0")
    parser.add_argument("--press-travel-velocity", default="45.0")
    parser.add_argument("--press-travel-acceleration", default="60.0")
    parser.add_argument("--press-contact-joint-velocity", default="22.0")
    parser.add_argument("--press-contact-joint-acceleration", default="30.0")
    parser.add_argument("--press-contact-entry-lift-m", default="0.050")
    parser.add_argument("--press-depth-m", default="0.020")
    parser.add_argument(
        "--press-extra-depth-m",
        default="0.010",
        help="--press-depth-m에 추가할 Z-only 프레스 하강량. 기본 0.010m.",
    )
    parser.add_argument(
        "--press-lock-contact-joints",
        default="6",
        help=(
            "measured sequence로 전달할 contact 조인트 잠금 축. 기본 6: "
            "프레스 contact에서 J6을 pre 자세 값으로 유지해 손목 회전을 막습니다."
        ),
    )
    parser.add_argument("--press-pre-lift-m", default="0.080")
    parser.add_argument("--press-transit-height-m", default="0.080")
    # Base frame in the measured dispenser setup:
    #   +X points from the robot toward the dispenser body, so backing away
    #   from the dispenser toward the robot is negative X.
    #   Y separates dispenser slots left/right.  Do not use Y as a safety
    #   retreat; on dispenser 4 it pushes the cup farther to the robot-view
    #   right side and can move outside the measured dispenser footprint.
    parser.add_argument("--press-pre-lift-retreat-x-m", default="-0.050")
    parser.add_argument("--press-pre-lift-retreat-y-m", default="0.0")
    parser.add_argument("--move-release-offset-x-m", default="-0.020")
    parser.add_argument("--move-release-offset-y-m", default="0.0")
    parser.add_argument("--move-release-offset-z-m", default="0.0")
    parser.add_argument("--regrasp-retreat-x-m", default="-0.080")
    parser.add_argument("--regrasp-retreat-y-m", default="0.0")
    parser.add_argument("--post-press-safe-lift-z-m", default="0.470")
    parser.add_argument("--regrasp-rear-entry-offset-x-m", default="-0.080")
    parser.add_argument("--regrasp-rear-entry-offset-y-m", default="0.0")
    parser.add_argument(
        "--allow-tcp-set-failure",
        action="store_true",
        help="Doosan TCP 설정 서비스가 success=false를 반환해도 현재 TCP로 measured sequence를 계속 실행",
    )
    parser.add_argument("--force-cartesian-press", action="store_true")
    parser.add_argument("--gripper-open-settle-seconds", default="1.5")
    parser.add_argument("--gripper-settle-seconds", default="0.8")
    parser.add_argument("--wait-service-sec", default="15.0")
    parser.add_argument("--pose-read-retries", default="3")
    parser.add_argument("--pose-read-retry-sleep-sec", default="0.5")
    parser.add_argument("--safe-lift-target-tolerance-mm", default="30.0")
    parser.add_argument("--post-press-safe-lift-target-tolerance-mm", default="60.0")
    parser.add_argument(
        "--allow-missing-color-map-fallback",
        action="store_true",
        help="debug only: if no color map exists, run physical dispensers 1,2,3,4",
    )
    args = parser.parse_args()
    if args.execute and not args.confirm:
        print(f"[BLOCKED] --execute requires --confirm ({CONFIRM_PHRASE})", file=sys.stderr)
        return 2

    sequence_extra_args = [
        "--press-min-transit-z-m", str(args.press_min_transit_z_m),
        "--press-pre-lift-m", str(args.press_pre_lift_m),
        "--press-transit-height-m", str(args.press_transit_height_m),
        "--press-pre-lift-retreat-x-m", str(args.press_pre_lift_retreat_x_m),
        "--press-pre-lift-retreat-y-m", str(args.press_pre_lift_retreat_y_m),
        "--move-release-offset-x-m", str(args.move_release_offset_x_m),
        "--move-release-offset-y-m", str(args.move_release_offset_y_m),
        "--move-release-offset-z-m", str(args.move_release_offset_z_m),
        "--regrasp-retreat-x-m", str(args.regrasp_retreat_x_m),
        "--regrasp-retreat-y-m", str(args.regrasp_retreat_y_m),
        "--post-press-safe-lift-z-m", str(args.post_press_safe_lift_z_m),
        "--regrasp-rear-entry-offset-x-m", str(args.regrasp_rear_entry_offset_x_m),
        "--regrasp-rear-entry-offset-y-m", str(args.regrasp_rear_entry_offset_y_m),
        "--press-line-velocity", str(args.press_line_velocity),
        "--press-line-acceleration", str(args.press_line_acceleration),
        "--press-travel-velocity", str(args.press_travel_velocity),
        "--press-travel-acceleration", str(args.press_travel_acceleration),
        "--press-contact-joint-velocity", str(args.press_contact_joint_velocity),
        "--press-contact-joint-acceleration", str(args.press_contact_joint_acceleration),
        "--press-contact-entry-lift-m", str(args.press_contact_entry_lift_m),
        "--press-depth-m", str(args.press_depth_m),
        "--press-extra-depth-m", str(args.press_extra_depth_m),
        "--press-lock-contact-joints", str(args.press_lock_contact_joints),
        "--gripper-open-settle-seconds", str(args.gripper_open_settle_seconds),
        "--gripper-settle-seconds", str(args.gripper_settle_seconds),
        "--wait-service-sec", str(args.wait_service_sec),
        "--pose-read-retries", str(args.pose_read_retries),
        "--pose-read-retry-sleep-sec", str(args.pose_read_retry_sleep_sec),
        "--safe-lift-target-tolerance-mm", str(args.safe_lift_target_tolerance_mm),
        "--post-press-safe-lift-target-tolerance-mm", str(args.post_press_safe_lift_target_tolerance_mm),
        "--safe-lift-joint-fallback",
        "--no-integrated-regrasp-fallback-subprocess",
    ]
    if args.allow_tcp_set_failure:
        sequence_extra_args.append("--allow-tcp-set-failure")
    if args.force_cartesian_press:
        sequence_extra_args.append("--force-cartesian-press")

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

    color_map = load_color_map(override_json=args.color_map_json)
    print(f"[run_color_recipe] 색상 맵: {color_map if color_map else 'missing/invalid'}")

    if not color_map:
        if not args.allow_missing_color_map_fallback:
            print(
                "[BLOCKED] 색상 기반 레시피는 outputs/dispenser_color_map.json이 필요합니다. "
                "색상 스캔을 성공시키거나, 진단용으로 물리 디스펜서 번호를 직접 입력하세요.",
                file=sys.stderr,
            )
            return 2
        dispenser_ids_str = "1,2,3,4"
        print(f"[run_color_recipe] debug fallback 직접 디스펜서 실행 순서: {dispenser_ids_str}")
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
        try:
            color_pumps = parse_colors_arg(args.colors)
        except ValueError as exc:
            print(f"[run_color_recipe] 잘못된 색상 입력: {exc}", file=sys.stderr)
            return 1
    else:
        if not RECIPE_PATH.exists():
            print(f"[run_color_recipe] 레시피 없음: {RECIPE_PATH}", file=sys.stderr)
            print("[run_color_recipe] listen_stt_recipe 스텝을 먼저 실행하세요.", file=sys.stderr)
            return 1
        recipe = json.loads(RECIPE_PATH.read_text(encoding="utf-8"))
        try:
            color_pumps = parse_recipe_data(recipe)
        except ValueError as exc:
            print(f"[run_color_recipe] 레시피 JSON 파싱 실패: {exc}", file=sys.stderr)
            return 1
        if not color_pumps:
            print(f"[run_color_recipe] 레시피에 실행할 색상/펌프 수가 없습니다: {RECIPE_PATH}", file=sys.stderr)
            return 1

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
