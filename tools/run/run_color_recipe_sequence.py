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
import os
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
        "--service-prefix",
        default=os.environ.get("SERVICE_PREFIX", ""),
        help="Doosan direct service namespace. 현재 스택이 /motion/* 루트 서비스를 쓰면 빈 값",
    )
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
    parser.add_argument(
        "--recipe-speed-scale",
        type=float,
        default=4.0,
        help="디스펜서 레시피 사이클의 속도/가속도 배율. 기본 4.0배.",
    )
    parser.add_argument("--move-velocity", default="80.0")
    parser.add_argument("--move-acceleration", default="25.0")
    parser.add_argument("--move-prehold-velocity", default="80.0")
    parser.add_argument("--move-prehold-acceleration", default="22.0")
    parser.add_argument("--pick-approach-velocity", default="80.0")
    parser.add_argument("--pick-approach-acceleration", default="14.0")
    parser.add_argument("--pick-lift-velocity", default="80.0")
    parser.add_argument("--pick-lift-acceleration", default="25.0")
    parser.add_argument("--regrasp-approach-velocity", default="80.0")
    parser.add_argument("--regrasp-approach-acceleration", default="18.0")
    parser.add_argument(
        "--regrasp-reset-before-cup",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="프레스 후 컵 재접근 전에 HOME joint waypoint를 경유",
    )
    parser.add_argument("--regrasp-reset-joints-deg", default="0,0,90,0,90,180")
    parser.add_argument("--regrasp-reset-joint-velocity", default="80.0")
    parser.add_argument("--regrasp-reset-joint-acceleration", default="35.0")
    parser.add_argument("--press-min-transit-z-m", default="0.500")
    parser.add_argument("--press-line-velocity", default="25.0")
    parser.add_argument("--press-line-acceleration", default="10.0")
    parser.add_argument("--press-travel-velocity", default="40.0")
    parser.add_argument("--press-travel-acceleration", default="20.0")
    parser.add_argument("--press-contact-joint-velocity", default="35.0")
    parser.add_argument("--press-contact-joint-acceleration", default="15.0")
    parser.add_argument("--press-contact-entry-lift-m", default="0.050")
    parser.add_argument(
        "--dispenser-1-press-y-offset-m",
        default="0.002",
        help="1번 디스펜서 press target에만 적용할 Y 보정값(m). 기본 +0.002m.",
    )
    parser.add_argument(
        "--press-reset-before-press",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="컵을 놓은 뒤 CONTACT_ENTRY_LIFT 전에 PRESS_COMMON_PRE/HOME joint waypoint를 경유. 기본 false",
    )
    parser.add_argument("--press-reset-joints-deg", default="0,0,90,0,90,0")
    parser.add_argument("--press-reset-joint-velocity", default="26.6666667")
    parser.add_argument("--press-reset-joint-acceleration", default="8.33333333")
    parser.add_argument("--press-depth-m", default="0.070")
    parser.add_argument(
        "--press-extra-depth-m",
        default="0.0",
        help="--press-depth-m에 추가할 Z-only 프레스 하강량. 기본 0.",
    )
    parser.add_argument(
        "--press-lock-contact-joints",
        default="",
        help=(
            "measured sequence로 전달할 contact 조인트 잠금 축. 기본 빈 값: "
            "측정된 PRESS_CONTACT joint를 그대로 사용합니다."
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
    parser.add_argument("--move-release-offset-z-m", default="0.010")
    parser.add_argument("--cup-pre-from-place-x-offset-m", default="-0.090")
    parser.add_argument("--cup-pre-from-place-z-offset-m", default="0.030")
    parser.add_argument("--dispenser-3-cup-pre-extra-x-offset-m", default="-0.010")
    parser.add_argument("--generated-cup-pre-max-joint-delta-deg", default="190.0")
    parser.add_argument(
        "--press-contact-use-joint-move",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="measured PRESS_CONTACT movej 사용. 기본 false: PRESS_CONTACT FK까지 Cartesian Z-only 하강",
    )
    parser.add_argument(
        "--use-cup-common-pre",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="저장된 cup_common_pre_joints_deg를 사용. 기본 false: cup_place 기준 X/Z offset pre 생성",
    )
    parser.add_argument("--regrasp-retreat-x-m", default="-0.080")
    parser.add_argument("--regrasp-retreat-y-m", default="0.0")
    parser.add_argument("--post-press-safe-lift-z-m", default="0.350")
    parser.add_argument(
        "--start-safe-lift-z-m",
        default="0.15",
        help="시퀀스 시작 시 현재 TCP pose에서 Z-only로 먼저 올라갈 최소 절대 TCP Z (m)",
    )
    parser.add_argument(
        "--min-allowed-tcp-z-m",
        default="0.02",
        help="모든 cartesian target pose의 최소 허용 TCP Z (m). 미달 target은 로봇 명령 전송 전에 차단",
    )
    parser.add_argument(
        "--force-start-safe-lift",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="시작 시 항상 현재 위치에서 safe Z로 lift한 뒤 다음 waypoint로 이동 (기본 켜짐)",
    )
    parser.add_argument(
        "--skip-release-pre",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="cup_place 기준 X/Z offset release-pre 대신 자세 무관 안전 release 구조 사용",
    )
    parser.add_argument("--release-approach-lift-m", default="0.100",
                        help="release final 위에 생성할 release_above pose의 Z lift (m)")
    parser.add_argument("--release-start-safe-lift-m", default="0.120",
                        help="release 시작 시 현재 TCP에서 Z-only로 올릴 상대 높이 (m)")
    parser.add_argument("--release-min-transit-z-m", default="0.300",
                        help="release XY-transit pose의 최소 절대 TCP Z (m)")
    parser.add_argument("--post-release-safe-lift-m", default="0.100",
                        help="gripper open 후 release final에서 수직 상승할 높이 (m)")
    parser.add_argument("--release-staging-x-m", default="",
                        help="release staging pose 절대 X (m). 비우면 release final X 사용")
    parser.add_argument("--release-staging-y-m", default="",
                        help="release staging pose 절대 Y (m). 비우면 release final Y 사용")
    parser.add_argument("--release-staging-z-m", default="",
                        help="release staging pose 절대 Z (m). 비우면 transit 높이 사용")
    parser.add_argument(
        "--use-release-staging",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="release_above 진입 전에 staging pose를 경유 (기본 켜짐)",
    )
    parser.add_argument("--regrasp-rear-entry-offset-x-m", default="-0.090")
    parser.add_argument("--regrasp-rear-entry-offset-y-m", default="0.0")
    parser.add_argument("--final-regrasp-extra-x-offset-m", default="0.000")
    parser.add_argument("--skip-initial-move-release", action="store_true",
                        help="복구 모드: 컵이 이미 첫 디스펜서 front-hold에 놓여 있다고 가정하고 press부터 시작")
    parser.add_argument(
        "--resume",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="명시 복구 모드에서만 디스펜서 resume_state를 읽음. 기본 false.",
    )
    parser.add_argument(
        "--resume-state-file",
        default="",
        help="run_measured_dispenser_recipe_sequence.py에 전달할 디스펜서 resume JSON 경로",
    )
    parser.add_argument(
        "--clear-resume-state",
        action="store_true",
        help="이번 실행 시작 전에 디스펜서 resume JSON을 삭제",
    )
    parser.add_argument("--final-regrasp-extra-y-offset-m", default="0.0")
    parser.add_argument("--final-regrasp-extra-z-offset-m", default="0.0")
    parser.add_argument("--final-regrasp-grasp-width-m", default="0.068")
    parser.add_argument("--final-regrasp-force-n", default="25.0")
    parser.add_argument(
        "--allow-tcp-set-failure",
        action="store_true",
        help="Doosan TCP 설정 서비스가 success=false를 반환해도 현재 TCP로 measured sequence를 계속 실행",
    )
    parser.add_argument("--force-cartesian-press", action="store_true")
    parser.add_argument("--gripper-open-settle-seconds", default="1.5")
    parser.add_argument("--gripper-settle-seconds", default="0.8")
    parser.add_argument(
        "--place-cup-holder-after-sequence",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="마지막 디스펜서 처리 후 컵홀더에 컵을 놓음",
    )
    parser.add_argument("--cup-holder-place-final-z-offset-m", default="-0.040")
    parser.add_argument("--cup-holder-place-final-y-offset-m", default="-0.010")
    parser.add_argument(
        "--cup-holder-rz-offset-deg",
        default="-1.0",
        help="컵홀더 이동 전 구간의 RZ 자세 보정값. calibration.yaml은 수정하지 않음.",
    )
    parser.add_argument("--cup-holder-z-min-m", default="0.06",
                        help="컵홀더 place 목표 z 안전 하한. place z offset을 크게 낮출 때 함께 내려야 함")
    parser.add_argument("--cup-holder-approach-velocity", default="80.0")
    parser.add_argument("--cup-holder-approach-acceleration", default="20.0")
    parser.add_argument("--cup-holder-place-velocity", default="80.0")
    parser.add_argument("--cup-holder-place-acceleration", default="10.0")
    parser.add_argument("--cup-holder-retreat-velocity", default="80.0")
    parser.add_argument("--cup-holder-retreat-acceleration", default="16.0")
    parser.add_argument("--cup-holder-timeout-sec", default="90.0")
    parser.add_argument("--cup-holder-target-tolerance-mm", default="12.0")
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
    if args.recipe_speed_scale <= 0.0:
        parser.error("--recipe-speed-scale must be > 0")

    def scaled_motion(value: str) -> str:
        return f"{float(value) * args.recipe_speed_scale:.6g}"

    def scaled_motion_capped(value: str, cap: float) -> str:
        return f"{min(float(value) * args.recipe_speed_scale, cap):.6g}"

    sequence_extra_args = [
        "--service-prefix", str(args.service_prefix),
        "--move-velocity", scaled_motion(args.move_velocity),
        "--move-acceleration", scaled_motion(args.move_acceleration),
        "--move-prehold-velocity", scaled_motion(args.move_prehold_velocity),
        "--move-prehold-acceleration", scaled_motion(args.move_prehold_acceleration),
        "--pick-approach-velocity", scaled_motion(args.pick_approach_velocity),
        "--pick-approach-acceleration", scaled_motion(args.pick_approach_acceleration),
        "--pick-lift-velocity", scaled_motion(args.pick_lift_velocity),
        "--pick-lift-acceleration", scaled_motion(args.pick_lift_acceleration),
        "--regrasp-approach-velocity", scaled_motion(args.regrasp_approach_velocity),
        "--regrasp-approach-acceleration", scaled_motion(args.regrasp_approach_acceleration),
        "--regrasp-reset-joints-deg", str(args.regrasp_reset_joints_deg),
        "--regrasp-reset-joint-velocity", scaled_motion(args.regrasp_reset_joint_velocity),
        "--regrasp-reset-joint-acceleration", scaled_motion(args.regrasp_reset_joint_acceleration),
        "--press-min-transit-z-m", str(args.press_min_transit_z_m),
        "--press-pre-lift-m", str(args.press_pre_lift_m),
        "--press-transit-height-m", str(args.press_transit_height_m),
        "--press-pre-lift-retreat-x-m", str(args.press_pre_lift_retreat_x_m),
        "--press-pre-lift-retreat-y-m", str(args.press_pre_lift_retreat_y_m),
        "--move-release-offset-x-m", str(args.move_release_offset_x_m),
        "--move-release-offset-y-m", str(args.move_release_offset_y_m),
        "--move-release-offset-z-m", str(args.move_release_offset_z_m),
        "--cup-pre-from-place-x-offset-m", str(args.cup_pre_from_place_x_offset_m),
        "--cup-pre-from-place-z-offset-m", str(args.cup_pre_from_place_z_offset_m),
        "--dispenser-3-cup-pre-extra-x-offset-m", str(args.dispenser_3_cup_pre_extra_x_offset_m),
        "--generated-cup-pre-max-joint-delta-deg", str(args.generated_cup_pre_max_joint_delta_deg),
        "--regrasp-retreat-x-m", str(args.regrasp_retreat_x_m),
        "--regrasp-retreat-y-m", str(args.regrasp_retreat_y_m),
        "--post-press-safe-lift-z-m", str(args.post_press_safe_lift_z_m),
        "--start-safe-lift-z-m", str(args.start_safe_lift_z_m),
        "--min-allowed-tcp-z-m", str(args.min_allowed_tcp_z_m),
        "--release-approach-lift-m", str(args.release_approach_lift_m),
        "--release-start-safe-lift-m", str(args.release_start_safe_lift_m),
        "--release-min-transit-z-m", str(args.release_min_transit_z_m),
        "--post-release-safe-lift-m", str(args.post_release_safe_lift_m),
        "--regrasp-rear-entry-offset-x-m", str(args.regrasp_rear_entry_offset_x_m),
        "--regrasp-rear-entry-offset-y-m", str(args.regrasp_rear_entry_offset_y_m),
        "--final-regrasp-extra-x-offset-m", str(args.final_regrasp_extra_x_offset_m),
        "--final-regrasp-extra-y-offset-m", str(args.final_regrasp_extra_y_offset_m),
        "--final-regrasp-extra-z-offset-m", str(args.final_regrasp_extra_z_offset_m),
        "--final-regrasp-grasp-width-m", str(args.final_regrasp_grasp_width_m),
        "--final-regrasp-force-n", str(args.final_regrasp_force_n),
        "--press-line-velocity", scaled_motion(args.press_line_velocity),
        "--press-line-acceleration", scaled_motion(args.press_line_acceleration),
        "--press-travel-velocity", scaled_motion(args.press_travel_velocity),
        "--press-travel-acceleration", scaled_motion(args.press_travel_acceleration),
        "--press-contact-joint-velocity", scaled_motion(args.press_contact_joint_velocity),
        "--press-contact-joint-acceleration", scaled_motion(args.press_contact_joint_acceleration),
        "--press-contact-entry-lift-m", str(args.press_contact_entry_lift_m),
        "--dispenser-1-press-y-offset-m", str(args.dispenser_1_press_y_offset_m),
        "--press-reset-joints-deg", str(args.press_reset_joints_deg),
        "--press-reset-joint-velocity", scaled_motion_capped(args.press_reset_joint_velocity, 80.0),
        "--press-reset-joint-acceleration", scaled_motion_capped(args.press_reset_joint_acceleration, 25.0),
        "--press-depth-m", str(args.press_depth_m),
        "--press-extra-depth-m", str(args.press_extra_depth_m),
        "--press-lock-contact-joints", str(args.press_lock_contact_joints),
        "--gripper-open-settle-seconds", str(args.gripper_open_settle_seconds),
        "--gripper-settle-seconds", str(args.gripper_settle_seconds),
        "--cup-holder-place-final-z-offset-m", str(args.cup_holder_place_final_z_offset_m),
        "--cup-holder-place-final-y-offset-m", str(args.cup_holder_place_final_y_offset_m),
        "--cup-holder-rz-offset-deg", str(args.cup_holder_rz_offset_deg),
        "--cup-holder-z-min-m", str(args.cup_holder_z_min_m),
        "--cup-holder-approach-velocity", scaled_motion(args.cup_holder_approach_velocity),
        "--cup-holder-approach-acceleration", scaled_motion(args.cup_holder_approach_acceleration),
        "--cup-holder-place-velocity", scaled_motion(args.cup_holder_place_velocity),
        "--cup-holder-place-acceleration", scaled_motion(args.cup_holder_place_acceleration),
        "--cup-holder-retreat-velocity", scaled_motion(args.cup_holder_retreat_velocity),
        "--cup-holder-retreat-acceleration", scaled_motion(args.cup_holder_retreat_acceleration),
        "--cup-holder-timeout-sec", str(args.cup_holder_timeout_sec),
        "--cup-holder-target-tolerance-mm", str(args.cup_holder_target_tolerance_mm),
        "--wait-service-sec", str(args.wait_service_sec),
        "--pose-read-retries", str(args.pose_read_retries),
        "--pose-read-retry-sleep-sec", str(args.pose_read_retry_sleep_sec),
        "--safe-lift-target-tolerance-mm", str(args.safe_lift_target_tolerance_mm),
        "--post-press-safe-lift-target-tolerance-mm", str(args.post_press_safe_lift_target_tolerance_mm),
        "--safe-lift-joint-fallback",
        "--no-integrated-regrasp-fallback-subprocess",
    ]
    sequence_extra_args.append(
        "--force-start-safe-lift" if args.force_start_safe_lift else "--no-force-start-safe-lift"
    )
    sequence_extra_args.append(
        "--skip-release-pre" if args.skip_release_pre else "--no-skip-release-pre"
    )
    if args.skip_initial_move_release:
        sequence_extra_args.append("--skip-initial-move-release")
    sequence_extra_args.append(
        "--use-cup-common-pre" if args.use_cup_common_pre else "--no-use-cup-common-pre"
    )
    sequence_extra_args.append(
        "--use-release-staging" if args.use_release_staging else "--no-use-release-staging"
    )
    sequence_extra_args.append(
        "--place-cup-holder-after-sequence"
        if args.place_cup_holder_after_sequence
        else "--no-place-cup-holder-after-sequence"
    )
    for flag, value in (
        ("--release-staging-x-m", args.release_staging_x_m),
        ("--release-staging-y-m", args.release_staging_y_m),
        ("--release-staging-z-m", args.release_staging_z_m),
    ):
        if str(value).strip():
            sequence_extra_args += [flag, str(value).strip()]
    sequence_extra_args.append(
        "--press-reset-before-press" if args.press_reset_before_press else "--no-press-reset-before-press"
    )
    sequence_extra_args.append(
        "--press-contact-use-joint-move"
        if args.press_contact_use_joint_move
        else "--no-press-contact-use-joint-move"
    )
    sequence_extra_args.append(
        "--regrasp-reset-before-cup" if args.regrasp_reset_before_cup else "--no-regrasp-reset-before-cup"
    )
    if args.allow_tcp_set_failure:
        sequence_extra_args.append("--allow-tcp-set-failure")
    if args.force_cartesian_press:
        sequence_extra_args.append("--force-cartesian-press")
    sequence_extra_args.append("--resume" if args.resume else "--no-resume")
    if str(args.resume_state_file).strip():
        sequence_extra_args += ["--resume-state-file", str(args.resume_state_file).strip()]
    if args.clear_resume_state:
        sequence_extra_args.append("--clear-resume-state")

    print(f"[run_color_recipe] 속도 배율: {args.recipe_speed_scale:.2f}x")

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
    mapped_steps: list[str] = []
    for color, pumps in color_pumps:
        did = color_to_dispenser_id(color, color_map)
        if did is None:
            print(f"[run_color_recipe] '{color}' 색깔이 색상 맵에 없음 → 건너뜀", file=sys.stderr)
            continue
        mapped_steps.append(f"{color}->{did}x{pumps}")
        for _ in range(pumps):
            sequence.append(did)

    if not sequence:
        print("[run_color_recipe] 실행할 디스펜서 없음 (색상 맵과 레시피 색깔이 불일치)", file=sys.stderr)
        return 1

    dispenser_ids_str = ",".join(sequence)
    print(f"[run_color_recipe] 색상→디스펜서 상세: {', '.join(mapped_steps)}")
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
