#!/usr/bin/env python3
"""One-shot auto handover: wait for a stable open palm, then hand the cup over.

Panel flow "손 보이면 자동 핸드오버": this watcher holds NO motion of its own.
It only listens to /azas/human_hand_detection (published by
run_human_hand_detection.sh ONLY while an open palm stays spatially stable)
and, once the palm has been continuously stable for the trigger window, runs
the existing gated handover script tools/run/handover_cup_to_palm.py exactly
once and exits with its return code.

Layered safety (kept from the manual flow):
  - trigger needs N stable detections inside a sliding window (person must
    hold the palm open and still BEFORE the robot starts at all)
  - handover_cup_to_palm.py then re-samples the palm itself, checks workspace
    bounds, re-checks the palm before descent, and aborts on any force spike
  - one-shot: after one attempt (success or abort) this watcher exits, so the
    robot never re-launches at a hand by itself

Usage:
  python3 tools/run/auto_handover_on_palm.py                      # dry-run
  python3 tools/run/auto_handover_on_palm.py --execute --confirm AUTO_HANDOVER_ON_PALM
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HANDOVER_SCRIPT = ROOT / "tools" / "run" / "handover_cup_to_palm.py"
HAND_TOPIC = "/azas/human_hand_detection"
CONFIRM_PHRASE = "AUTO_HANDOVER_ON_PALM"


def wait_for_stable_palm(args: argparse.Namespace) -> bool:
    """Spin a perception-only node until the palm trigger fires or we time out."""
    import rclpy
    from geometry_msgs.msg import PointStamped

    stamps: list[float] = []
    rclpy.init()
    node = rclpy.create_node("azas_auto_handover_watch")
    node.create_subscription(
        PointStamped, HAND_TOPIC, lambda _msg: stamps.append(time.monotonic()), 10
    )
    print(
        f"[Azas] 손 대기 시작: {args.trigger_window_sec:.1f}초 안에 안정 검출 "
        f"{args.trigger_stable_count}개가 쌓이면 핸드오버를 1회 실행합니다 "
        f"(최대 {args.wait_timeout_sec:.0f}초 대기, 대기 중 로봇 모션 없음)."
    )
    deadline = time.monotonic() + args.wait_timeout_sec
    last_report = 0.0
    triggered = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            stamps[:] = [t for t in stamps if now - t <= args.trigger_window_sec]
            if len(stamps) >= args.trigger_stable_count:
                triggered = True
                break
            if now - last_report >= 5.0:
                last_report = now
                remain = deadline - now
                print(
                    f"[Azas] 대기 중... 최근 {args.trigger_window_sec:.1f}초 안정 검출 "
                    f"{len(stamps)}/{args.trigger_stable_count}개 (남은 시간 {remain:.0f}초). "
                    "손바닥을 펴고 정지해 주세요."
                )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return triggered


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--trigger-stable-count", type=int, default=12,
                        help="trigger when this many stable detections land inside the window")
    parser.add_argument("--trigger-window-sec", type=float, default=3.0)
    parser.add_argument("--wait-timeout-sec", type=float, default=180.0,
                        help="give up (exit 3, no motion) when no stable palm appears in time")
    parser.add_argument("--release-tcp-above-palm-m", default="0.08")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} with --execute")
    args = parser.parse_args()

    if args.execute and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --execute requires --confirm {CONFIRM_PHRASE}")
        return 2
    if not args.execute:
        print("[DRY-RUN] --execute 미지정: 손 트리거 후 핸드오버도 dry-run(인식+계획만)으로 실행합니다.")

    if not wait_for_stable_palm(args):
        print(
            f"[FAIL] {args.wait_timeout_sec:.0f}초 안에 안정적인 손바닥이 없어 종료합니다 (로봇 모션 없음). "
            "손 검출 화면에서 STABLE이 뜨는 위치를 확인한 뒤 다시 실행하세요."
        )
        return 3

    print("[Azas] 손 트리거 충족. 핸드오버를 1회 실행합니다 (이후 자동 재시도 없음).")
    cmd = [
        sys.executable, str(HANDOVER_SCRIPT),
        "--service-prefix", args.service_prefix,
        "--release-tcp-above-palm-m", str(args.release_tcp_above_palm_m),
        "--transit-velocity", "10.0", "--transit-acceleration", "14.0",
        "--descent-velocity", "4.0", "--descent-acceleration", "6.0",
        "--force-abort-delta-n", "10.0",
    ]
    if args.execute:
        cmd += [
            "--execute", "--confirm", "ENABLE_HUMAN_PALM_HANDOVER",
            "--approve-motion", "ENABLE_HUMAN_PALM_HANDOVER_MOTION",
            "--approve-release", "RELEASE_CUP_NOW",
        ]
    rc = subprocess.run(cmd, cwd=str(ROOT), check=False).returncode
    if rc == 0:
        print("[PASS] 자동 핸드오버 완료.")
    else:
        print(f"[FAIL] 핸드오버가 비정상 종료했습니다 (rc={rc}); 위 로그를 확인하세요.")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
