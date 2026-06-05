#!/usr/bin/env python3
"""색상 레시피 시퀀스 실행.

outputs/latest_recipe.json (색깔 목록) + outputs/dispenser_color_map.json (위치→색깔)
를 읽어 색깔→디스펜서 ID를 매핑한 뒤 run_measured_dispenser_recipe_sequence.py 실행.

사용법:
  python3 tools/run/run_color_recipe_sequence.py
  python3 tools/run/run_color_recipe_sequence.py --colors red:2,blue:1  # 직접 지정
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


def load_color_map() -> dict[str, str]:
    """dispenser_id → color_name 매핑 로드."""
    if not COLOR_MAP_PATH.exists():
        print(f"[run_color_recipe] 색상 맵 없음: {COLOR_MAP_PATH}", file=sys.stderr)
        print("[run_color_recipe] color_scan 스텝을 먼저 실행하세요.", file=sys.stderr)
        sys.exit(1)
    data = json.loads(COLOR_MAP_PATH.read_text(encoding="utf-8"))
    return {str(k): str(v).lower().strip() for k, v in data.items()}


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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--colors", default="",
                        help="직접 색깔 지정: 'red:2,blue:1' (생략 시 latest_recipe.json 사용)")
    parser.add_argument("--confirm", action="store_true",
                        help=f"확인 구문({CONFIRM_PHRASE}) 자동 전달")
    args = parser.parse_args()

    color_map = load_color_map()
    print(f"[run_color_recipe] 색상 맵: {color_map}")

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
    ]
    if args.confirm:
        cmd += ["--confirm", CONFIRM_PHRASE]

    print(f"[run_color_recipe] 실행: {' '.join(cmd)}")
    result = subprocess.run(cmd, check=False)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
