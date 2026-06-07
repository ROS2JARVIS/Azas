#!/usr/bin/env python3
"""STT 레시피 대기: /azas/voice/recipe_decision 토픽을 수신해 outputs/latest_recipe.json 저장.

voice_input 스텝이 먼저 실행(STT+LLM 노드 기동)된 상태에서 호출.
최대 --timeout 초 동안 대기하며 "make_cocktail" 인텐트를 수신하면 저장 후 종료.

출력 포맷:
  {"colors": ["red", "blue"], "pumps": {"red": 2, "blue": 1}, "recipe_id": "..."}
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUTPUT_PATH = ROOT / "outputs" / "latest_recipe.json"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=60.0, help="레시피 대기 최대 초")
    args = parser.parse_args()

    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
    except ImportError as e:
        print(f"[listen_stt_recipe] rclpy 없음: {e}", file=sys.stderr)
        return 1

    import time

    received: dict | None = None

    def on_msg(msg) -> None:
        nonlocal received
        if received is not None:
            return
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        intent = str(data.get("intent", "")).strip().lower()
        if intent != "make_cocktail":
            print(f"[listen_stt_recipe] intent={intent} 무시 (make_cocktail 아님)")
            return

        recipe_id = str(data.get("recipe_id", "custom")).strip()

        # pump 수: dispenser_amounts(신규) 또는 pump_counts(구형) 중 있는 쪽 사용, 없으면 1
        pumps_raw = data.get("dispenser_amounts") or data.get("pump_counts") or {}

        # 색상 목록: dispenser_ids 우선, dispenser_amounts 키로 보완
        ids_from_field = [str(c).strip().lower() for c in data.get("dispenser_ids", []) if c]
        ids_from_amounts = list(pumps_raw.keys()) if pumps_raw else []
        colors = ids_from_field or ids_from_amounts

        pumps = {c: int(pumps_raw.get(c, 1)) for c in colors}

        received = {"colors": colors, "pumps": pumps, "recipe_id": recipe_id}
        print(f"[listen_stt_recipe] 수신: {received}")

    rclpy.init()
    node = rclpy.create_node("listen_stt_recipe_node")

    # azas_voice가 퍼블리시하는 토픽 - 메시지 타입은 std_msgs/String (JSON payload)
    from std_msgs.msg import String
    node.create_subscription(String, "/azas/voice/confirmed_recipe_decision", on_msg, qos_profile_sensor_data)

    print(f"[listen_stt_recipe] 레시피 대기 중... (최대 {args.timeout:.0f}초)")
    deadline = time.time() + args.timeout
    try:
        while rclpy.ok() and received is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if received is None:
        print(f"[listen_stt_recipe] {args.timeout:.0f}초 내 레시피 없음", file=sys.stderr)
        return 1

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(json.dumps(received, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[listen_stt_recipe] 저장: {OUTPUT_PATH}")
    print(json.dumps(received, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
