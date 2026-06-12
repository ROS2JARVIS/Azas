#!/usr/bin/env python3
"""Panel check: listen to /azas/human_hand_detection/status and summarize.

Perception-only (no motion command). Listens for a fixed window and reports
how many frames detected a hand and how many passed the stability gate that
actually publishes coordinates for the palm handover.

Exit codes:
  0  stable open-hand detections seen (handover can consume coordinates)
  1  status is flowing but no stable open hand in the window
  2  no status messages at all (detection node or camera is not running)
"""
from __future__ import annotations

import argparse
import json
import time

import rclpy
from std_msgs.msg import String

STATUS_TOPIC = "/azas/human_hand_detection/status"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen-sec", type=float, default=10.0)
    args = parser.parse_args()

    messages: list[dict] = []
    rclpy.init()
    node = rclpy.create_node("azas_check_hand_detection_status")
    node.create_subscription(
        String, STATUS_TOPIC, lambda m: messages.append(json.loads(m.data)), 10
    )
    print(f"[Azas] 손 검출 상태를 {args.listen_sec:.0f}초간 측정합니다 (로봇 모션 없음).")
    deadline = time.monotonic() + args.listen_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    detected = [m for m in messages if m.get("detected")]
    stable = [m for m in messages if m.get("stable")]
    print(f"[Azas] 상태 메시지 {len(messages)}개 / 손 검출 {len(detected)}개 / STABLE {len(stable)}개")

    if not messages:
        print("[FAIL] 상태 메시지가 없습니다. 카메라와 '손 검출 시작' 버튼이 켜져 있는지 확인하세요.")
        return 2
    if not stable:
        reasons = [str(m.get("reason", "")) for m in messages if m.get("reason")]
        if reasons:
            print(f"[Azas] 최근 사유: {reasons[-1]}")
        print(
            "[FAIL] STABLE 검출이 없습니다. 손바닥을 펴고(손가락 4개 이상), "
            "카메라에서 0.3m 이상 떨어져 1초간 정지하세요. 텀블러가 가리는 화면 "
            "오른쪽 아래를 피해 왼쪽/위쪽 영역에 손을 두세요."
        )
        return 1

    last = stable[-1]
    xyz = last.get("camera_xyz_m")
    depth = last.get("depth_m")
    print(f"[Azas] 마지막 STABLE: depth={depth}m camera_xyz_m={xyz} palm_px={last.get('palm_px')}")
    print("[PASS] 안정적인 손바닥 검출이 발행되고 있습니다. 핸드오버 진행 가능.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
