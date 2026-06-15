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
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HANDOVER_SCRIPT = ROOT / "tools" / "run" / "handover_cup_to_palm.py"
DIRECT_MOVEJ = ROOT / "tools" / "run" / "direct_movej_joints.py"
HAND_TOPIC = "/azas/human_hand_detection"
STATUS_TOPIC = "/azas/human_hand_detection/status"
CONFIRM_PHRASE = "AUTO_HANDOVER_ON_PALM"
MOVEJ_CONFIRM_PHRASE = "ENABLE_DIRECT_MOVEJ"


def wait_for_stable_palm(args: argparse.Namespace) -> bool:
    """Spin a perception-only node until the palm trigger fires or we time out."""
    import rclpy
    from geometry_msgs.msg import PointStamped
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    stamps: list[float] = []
    latest_status = {"detected": False, "reason": "no status yet"}
    point_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    def status_ready(status: dict[str, object]) -> bool:
        depth_m = status.get("depth_m")
        if depth_m is None:
            return False
        depth = float(depth_m)
        return (
            bool(status.get("detected"))
            and bool(status.get("hand_open"))
            and bool(status.get("stable"))
            and args.trigger_min_depth_m <= depth <= args.trigger_max_depth_m
            and int(status.get("open_fingers", 0)) >= args.min_trigger_open_fingers
        )

    def on_status(msg: String) -> None:
        nonlocal latest_status
        try:
            latest_status = json.loads(msg.data)
        except json.JSONDecodeError:
            latest_status = {"raw": msg.data}
        if not status_ready(latest_status):
            stamps.clear()

    def on_hand(_msg: PointStamped) -> None:
        if status_ready(latest_status):
            stamps.append(time.monotonic())

    rclpy.init()
    node = rclpy.create_node("azas_auto_handover_watch")
    node.create_subscription(PointStamped, HAND_TOPIC, on_hand, point_qos)
    node.create_subscription(String, STATUS_TOPIC, on_status, 10)
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
            stable_duration = (now - stamps[0]) if stamps else 0.0
            if len(stamps) >= args.trigger_stable_count and stable_duration >= args.trigger_min_stable_sec:
                triggered = True
                break
            if now - last_report >= 5.0:
                last_report = now
                remain = deadline - now
                print(
                    f"[Azas] 대기 중... 최근 {args.trigger_window_sec:.1f}초 안정 검출 "
                    f"{len(stamps)}/{args.trigger_stable_count}개, 지속 {stable_duration:.1f}/"
                    f"{args.trigger_min_stable_sec:.1f}초 (남은 시간 {remain:.0f}초). "
                    f"status={latest_status}. 손바닥을 펴고 정지해 주세요."
                )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return triggered


def parse_joint_csv(value: str) -> list[float]:
    joints = [float(part.strip()) for part in str(value).split(",") if part.strip()]
    if len(joints) != 6:
        raise ValueError(f"--observe-joints must contain 6 comma-separated values, got {len(joints)}")
    return joints


def move_to_observe(args: argparse.Namespace) -> int:
    joints = parse_joint_csv(args.observe_joints)
    cmd = [
        sys.executable, str(DIRECT_MOVEJ),
        "--service-prefix", args.service_prefix,
        "--j1", f"{joints[0]:.6f}",
        "--j2", f"{joints[1]:.6f}",
        "--j3", f"{joints[2]:.6f}",
        "--j4", f"{joints[3]:.6f}",
        "--j5", f"{joints[4]:.6f}",
        "--j6", f"{joints[5]:.6f}",
        "--velocity", f"{args.observe_velocity:.3f}",
        "--acceleration", f"{args.observe_acceleration:.3f}",
        "--timeout-sec", f"{args.observe_timeout_sec:.1f}",
        "--motion-timeout-sec", f"{args.observe_motion_timeout_sec:.1f}",
        "--j5-min-deg", f"{args.observe_j5_min_deg:.3f}",
        "--j5-max-deg", f"{args.observe_j5_max_deg:.3f}",
    ]
    if args.execute:
        cmd += ["--execute", "--confirm", MOVEJ_CONFIRM_PHRASE]
    print(
        "[Azas] observe 위치로 먼저 이동합니다: joints_deg=["
        + ", ".join(f"{value:.1f}" for value in joints)
        + f"] vel={args.observe_velocity:.1f}"
    )
    rc = subprocess.run(cmd, cwd=str(ROOT), check=False).returncode
    if rc == 0:
        print("[PASS] observe 위치 이동 완료.")
    else:
        print(f"[FAIL] observe 위치 이동 실패(rc={rc}); 손 대기/핸드오버를 시작하지 않습니다.")
    return rc


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--service-prefix", default=os.environ.get("SERVICE_PREFIX", ""))
    parser.add_argument("--no-service-prefix-fallback", action="store_true",
                        help="use exactly --service-prefix in the handover script; do not fall back to dsr01")
    parser.add_argument("--trigger-stable-count", type=int, default=12,
                        help="trigger when this many stable detections land inside the window")
    parser.add_argument("--trigger-window-sec", type=float, default=3.0)
    parser.add_argument("--trigger-min-stable-sec", type=float, default=0.0,
                        help="also require the stable detections to span at least this many seconds")
    parser.add_argument("--min-trigger-open-fingers", type=int, default=3,
                        help="auto trigger also requires latest status.open_fingers >= this value")
    parser.add_argument("--trigger-min-depth-m", type=float, default=0.30,
                        help="auto trigger requires latest palm depth to be at least this close/far range")
    parser.add_argument("--trigger-max-depth-m", type=float, default=0.75,
                        help="auto trigger rejects hands farther than this camera depth")
    parser.add_argument("--wait-timeout-sec", type=float, default=180.0,
                        help="give up (exit 3, no motion) when no stable palm appears in time")
    parser.add_argument("--release-tcp-above-palm-m", default="0.08")
    parser.add_argument("--skip-observe", action="store_true",
                        help="do not move to the observe/camera-home joint pose before waiting for a palm")
    parser.add_argument("--observe-joints", default="3.0,-12.7,44.0,-9.0,133.0,90.0",
                        help="comma-separated J1..J6 degrees for the initial observe/camera-home pose")
    parser.add_argument("--observe-velocity", type=float, default=45.0)
    parser.add_argument("--observe-acceleration", type=float, default=60.0)
    parser.add_argument("--observe-timeout-sec", type=float, default=20.0)
    parser.add_argument("--observe-motion-timeout-sec", type=float, default=120.0)
    parser.add_argument("--observe-j5-min-deg", type=float, default=-150.0)
    parser.add_argument("--observe-j5-max-deg", type=float, default=150.0)
    parser.add_argument("--hand-sample-count", type=int, default=None)
    parser.add_argument("--hand-sample-timeout-sec", type=float, default=None)
    parser.add_argument("--hand-sample-spread-max-m", type=float, default=None)
    parser.add_argument("--hand-recheck-tolerance-m", type=float, default=None)
    parser.add_argument("--skip-hand-recheck", action="store_true",
                        help="handover to the initially sampled palm without the pre-descent re-check")
    parser.add_argument("--transit-velocity", type=float, default=75.0)
    parser.add_argument("--transit-acceleration", type=float, default=95.0)
    parser.add_argument("--descent-velocity", type=float, default=22.0)
    parser.add_argument("--descent-acceleration", type=float, default=32.0)
    parser.add_argument("--descent-step-m", type=float, default=0.03)
    parser.add_argument("--max-descent-steps", type=int, default=0,
                        help="maximum staged descent steps; 0 means use the Z floor only")
    parser.add_argument("--force-search-start-above-palm-m", type=float, default=0.16,
                        help="with contact release, start force-only descent this far above the detected palm")
    parser.add_argument("--force-search-below-palm-m", type=float, default=0.10,
                        help="with contact release, search down to this far below the detected palm before aborting")
    parser.add_argument("--move-timeout-sec", type=float, default=None)
    parser.add_argument("--verify-timeout-sec", type=float, default=None)
    parser.add_argument("--target-tolerance-mm", type=float, default=None)
    parser.add_argument("--ikin-timeout-sec", type=float, default=None)
    parser.add_argument("--ikin-retries", type=int, default=None)
    parser.add_argument("--ikin-sol-spaces", default=None)
    parser.add_argument("--j5-min-deg", type=float, default=None)
    parser.add_argument("--j5-max-deg", type=float, default=None)
    parser.add_argument("--skip-force-monitor", action="store_true",
                        help="pass through to handover script; staged descent remains but force abort is disabled")
    parser.add_argument("--force-abort-delta-n", type=float, default=2.0,
                        help="force rise over baseline that counts as palm contact during descent")
    parser.add_argument("--force-axis-delta-n", type=float, default=1.0,
                        help="also count contact when any single force axis changes by this much")
    parser.add_argument("--contact-axis", choices=("z", "xy", "all"), default="z",
                        help="force axes used for contact release; z is safest for vertical handover")
    parser.add_argument("--contact-z-direction", choices=("positive", "negative", "any"), default="positive",
                        help="when --contact-axis z, require this signed Z force delta for contact")
    parser.add_argument("--contact-step-delta-n", type=float, default=2.0,
                        help="contact candidate also requires this force jump from the previous descent step")
    parser.add_argument("--require-force-magnitude-delta", action=argparse.BooleanOptionalAction, default=True,
                        help="also require total force magnitude to rise before contact release")
    parser.add_argument("--force-magnitude-delta-n", type=float, default=1.5,
                        help="minimum total force magnitude rise required with --require-force-magnitude-delta")
    parser.add_argument("--force-baseline-samples", type=int, default=5,
                        help="average this many GetToolForce samples before descent")
    parser.add_argument("--force-baseline-interval-sec", type=float, default=0.05,
                        help="delay between baseline force samples")
    parser.add_argument("--force-read-settle-sec", type=float, default=0.15,
                        help="wait after each descent step before reading force")
    parser.add_argument("--release-on-contact", action=argparse.BooleanOptionalAction, default=True,
                        help="open the gripper at the first force/contact trigger during descent")
    parser.add_argument("--require-contact-for-release", action=argparse.BooleanOptionalAction, default=True,
                        help="with --release-on-contact, retreat with the cup if contact is never detected")
    parser.add_argument("--contact-confirm-samples", type=int, default=5,
                        help="consecutive above-threshold force samples required before opening RG2")
    parser.add_argument("--contact-confirm-interval-sec", type=float, default=0.12,
                        help="delay between force confirmation samples")
    parser.add_argument("--contact-relief-lift-m", type=float, default=0.0,
                        help="deprecated/ignored: contact release now opens RG2 at the confirmed contact pose")
    parser.add_argument("--contact-search-below-release-m", type=float, default=0.20,
                        help="with --release-on-contact, keep descending this far below release height while seeking contact")
    parser.add_argument("--gripper-open-retries", type=int, default=None)
    parser.add_argument("--gripper-open-retry-sleep-sec", type=float, default=None)
    parser.add_argument("--x-min", type=float, default=None)
    parser.add_argument("--x-max", type=float, default=None)
    parser.add_argument("--y-min", type=float, default=None)
    parser.add_argument("--y-max", type=float, default=None)
    parser.add_argument("--z-min", type=float, default=None)
    parser.add_argument("--z-max", type=float, default=None)
    parser.add_argument("--palm-z-max-m", type=float, default=None)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} with --execute")
    args = parser.parse_args()

    if args.execute and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --execute requires --confirm {CONFIRM_PHRASE}")
        return 2
    if args.release_on_contact and args.skip_force_monitor:
        print("[BLOCKED] contact-release mode requires force monitoring; remove --skip-force-monitor")
        return 2
    if not args.execute:
        print("[DRY-RUN] --execute 미지정: 손 트리거 후 핸드오버도 dry-run(인식+계획만)으로 실행합니다.")

    if not args.skip_observe:
        rc = move_to_observe(args)
        if rc != 0:
            return rc

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
        "--transit-velocity", f"{args.transit_velocity:.3f}",
        "--transit-acceleration", f"{args.transit_acceleration:.3f}",
        "--descent-velocity", f"{args.descent_velocity:.3f}",
        "--descent-acceleration", f"{args.descent_acceleration:.3f}",
        "--descent-step-m", f"{args.descent_step_m:.3f}",
        "--max-descent-steps", str(args.max_descent_steps),
        "--force-search-start-above-palm-m", f"{args.force_search_start_above_palm_m:.3f}",
        "--force-search-below-palm-m", f"{args.force_search_below_palm_m:.3f}",
        "--force-abort-delta-n", f"{args.force_abort_delta_n:.3f}",
        "--force-axis-delta-n", f"{args.force_axis_delta_n:.3f}",
        "--contact-axis", args.contact_axis,
        "--contact-z-direction", args.contact_z_direction,
        "--contact-step-delta-n", f"{args.contact_step_delta_n:.3f}",
        "--force-magnitude-delta-n", f"{args.force_magnitude_delta_n:.3f}",
        "--force-baseline-samples", str(args.force_baseline_samples),
        "--force-baseline-interval-sec", f"{args.force_baseline_interval_sec:.3f}",
        "--force-read-settle-sec", f"{args.force_read_settle_sec:.3f}",
        "--contact-confirm-samples", str(args.contact_confirm_samples),
        "--contact-confirm-interval-sec", f"{args.contact_confirm_interval_sec:.3f}",
        "--contact-relief-lift-m", f"{args.contact_relief_lift_m:.3f}",
        "--contact-search-below-release-m", f"{args.contact_search_below_release_m:.3f}",
    ]
    if args.hand_sample_count is not None:
        cmd += ["--hand-sample-count", str(args.hand_sample_count)]
    if args.hand_sample_timeout_sec is not None:
        cmd += ["--hand-sample-timeout-sec", f"{args.hand_sample_timeout_sec:.1f}"]
    if args.hand_sample_spread_max_m is not None:
        cmd += ["--hand-sample-spread-max-m", f"{args.hand_sample_spread_max_m:.3f}"]
    if args.hand_recheck_tolerance_m is not None:
        cmd += ["--hand-recheck-tolerance-m", f"{args.hand_recheck_tolerance_m:.3f}"]
    if args.skip_hand_recheck:
        cmd += ["--skip-hand-recheck"]
    if args.move_timeout_sec is not None:
        cmd += ["--move-timeout-sec", f"{args.move_timeout_sec:.1f}"]
    if args.verify_timeout_sec is not None:
        cmd += ["--verify-timeout-sec", f"{args.verify_timeout_sec:.1f}"]
    if args.target_tolerance_mm is not None:
        cmd += ["--target-tolerance-mm", f"{args.target_tolerance_mm:.1f}"]
    if args.ikin_timeout_sec is not None:
        cmd += ["--ikin-timeout-sec", f"{args.ikin_timeout_sec:.1f}"]
    if args.ikin_retries is not None:
        cmd += ["--ikin-retries", str(args.ikin_retries)]
    if args.ikin_sol_spaces:
        cmd += ["--ikin-sol-spaces", args.ikin_sol_spaces]
    if args.j5_min_deg is not None:
        cmd += ["--j5-min-deg", f"{args.j5_min_deg:.3f}"]
    if args.j5_max_deg is not None:
        cmd += ["--j5-max-deg", f"{args.j5_max_deg:.3f}"]
    if args.skip_force_monitor:
        cmd += ["--skip-force-monitor"]
    if args.release_on_contact:
        cmd += ["--release-on-contact"]
    if args.require_contact_for_release:
        cmd += ["--require-contact-for-release"]
    else:
        cmd += ["--no-require-contact-for-release"]
    if args.require_force_magnitude_delta:
        cmd += ["--require-force-magnitude-delta"]
    else:
        cmd += ["--no-require-force-magnitude-delta"]
    if args.gripper_open_retries is not None:
        cmd += ["--gripper-open-retries", str(args.gripper_open_retries)]
    if args.gripper_open_retry_sleep_sec is not None:
        cmd += ["--gripper-open-retry-sleep-sec", f"{args.gripper_open_retry_sleep_sec:.1f}"]
    for name in ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max"):
        value = getattr(args, name)
        if value is not None:
            cmd += [f"--{name.replace('_', '-')}", f"{value:.3f}"]
    if args.palm_z_max_m is not None:
        cmd += ["--palm-z-max-m", f"{args.palm_z_max_m:.3f}"]
    if args.execute:
        cmd += [
            "--execute", "--confirm", "ENABLE_HUMAN_PALM_HANDOVER",
            "--approve-motion", "ENABLE_HUMAN_PALM_HANDOVER_MOTION",
            "--approve-release", "RELEASE_CUP_NOW",
        ]
    if args.no_service_prefix_fallback:
        cmd += ["--no-service-prefix-fallback"]
    rc = subprocess.run(cmd, cwd=str(ROOT), check=False).returncode
    if rc == 0:
        print("[PASS] 자동 핸드오버 완료.")
    else:
        print(f"[FAIL] 핸드오버가 비정상 종료했습니다 (rc={rc}); 위 로그를 확인하세요.")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
