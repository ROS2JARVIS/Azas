#!/usr/bin/env python3
"""Replay registered dispenser press teach poses one by one for visual lane checks.

Reads press_pre_joints_deg / press_contact_joints_deg for each dispenser from
calibration.yaml, prints an offline URDF-FK prediction of which pump lane each
pose is over, then (with --execute) drives the robot slowly to each pose with a
safe joint-home transit between poses so a mislabeled contact pose cannot drag
across pump heads.

Usage:
  python3 tools/run/replay_registered_press_poses.py                # plan only
  python3 tools/run/replay_registered_press_poses.py --rviz         # RViz preview
  python3 tools/run/replay_registered_press_poses.py --execute \
      --confirm ENABLE_PRESS_POSE_REPLAY                            # real motion

RViz preview mode publishes /joint_states (joint_1..joint_6) plus markers for
the four configured press_pose_xyz_m lanes. Bring up the hardware-free scene
first:
  ros2 launch azas_bringup hardware_free_demo.launch.py

The operator watches which pump each pose actually hovers over and notes the
true lane; afterwards the YAML slots are rearranged to match reality.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
CALIBRATION = ROOT / "src" / "azas_bringup" / "config" / "calibration.yaml"
URDF = ROOT / "install" / "dsr_description2" / "share" / "dsr_description2" / "urdf" / "m0609.urdf"
MOVEJ_TOOL = ROOT / "tools" / "run" / "direct_movej_joints.py"
CONFIRM_PHRASE = "ENABLE_PRESS_POSE_REPLAY"
INVALID_PRESS_CONTACT_STATUSES = {
    "invalid",
    "invalid_reteach_required",
    "needs_reteach",
    "reteach_required",
    "확인 필요",
}
SAFE_HOME_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


def load_chain():
    tree = ET.parse(URDF)
    joints = {}
    for j in tree.getroot().findall("joint"):
        if j.get("type") not in ("revolute", "fixed"):
            continue
        origin = j.find("origin")
        xyz = [float(v) for v in (origin.get("xyz") or "0 0 0").split()] if origin is not None else [0.0, 0.0, 0.0]
        rpy = [float(v) for v in (origin.get("rpy") or "0 0 0").split()] if origin is not None else [0.0, 0.0, 0.0]
        axis_el = j.find("axis")
        axis = [float(v) for v in (axis_el.get("xyz") or "0 0 1").split()] if axis_el is not None else [0.0, 0.0, 1.0]
        joints[j.get("name")] = dict(
            parent=j.find("parent").get("link"),
            child=j.find("child").get("link"),
            xyz=xyz,
            rpy=rpy,
            axis=axis,
            type=j.get("type"),
        )
    chain = []
    link = "link_6"
    while True:
        entry = next(((n, j) for n, j in joints.items() if j["child"] == link), None)
        if entry is None:
            break
        chain.append(entry)
        link = entry[1]["parent"]
        if link in ("base_link", "base", "world", "base_0"):
            break
    chain.reverse()
    return chain


def rpy_mat(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def axis_rot(axis: list[float], theta: float) -> np.ndarray:
    a = np.array(axis) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(theta) * k + (1 - math.cos(theta)) * (k @ k)


def flange_fk_mm(chain, q_deg: list[float]) -> np.ndarray:
    t = np.eye(4)
    qi = 0
    for _, j in chain:
        a = np.eye(4)
        a[:3, :3] = rpy_mat(*j["rpy"])
        a[:3, 3] = j["xyz"]
        t = t @ a
        if j["type"] == "revolute":
            r = np.eye(4)
            r[:3, :3] = axis_rot(j["axis"], math.radians(q_deg[qi]))
            qi += 1
            t = t @ r
    return t[:3, 3] * 1000.0


def lane_guess(flange_y_mm: float) -> str:
    # Flange-frame lane centers observed for this setup (outlet TCP y 84/43/-2/-50).
    lanes = {"1": 71.0, "2": 27.0, "3": -33.0, "4": -68.0}
    best = min(lanes.items(), key=lambda kv: abs(kv[1] - flange_y_mm))
    return f"{best[0]}번 레인 부근 (오차 {abs(best[1] - flange_y_mm):.0f}mm)"


def movej(joints_deg: list[float], *, service_prefix: str, velocity: float, label: str) -> None:
    cmd = [
        sys.executable,
        str(MOVEJ_TOOL),
        "--service-prefix", service_prefix,
        "--j1", str(joints_deg[0]), "--j2", str(joints_deg[1]), "--j3", str(joints_deg[2]),
        "--j4", str(joints_deg[3]), "--j5", str(joints_deg[4]), "--j6", str(joints_deg[5]),
        "--velocity", str(velocity), "--acceleration", str(velocity),
        "--j5-min-deg", "-150", "--j5-max-deg", "150",
        "--timeout-sec", "60", "--motion-timeout-sec", "120",
        "--execute", "--confirm", "ENABLE_DIRECT_MOVEJ",
    ]
    print(f"[replay] movej: {label}")
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"movej failed for {label} (rc={result.returncode}); aborting replay")


def run_rviz_preview(poses: list[tuple[str, list[float]]], chain, outlets: dict) -> int:
    import threading
    import time

    import rclpy
    from rclpy.node import Node as RclpyNode
    from sensor_msgs.msg import JointState
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init()
    node = RclpyNode("press_pose_replay_rviz_preview")
    joint_pub = node.create_publisher(JointState, "/joint_states", 10)
    marker_pub = node.create_publisher(MarkerArray, "/azas/press_pose_replay/markers", 10)

    current = {"q": list(poses[0][1]) if poses else [0.0] * 6, "target": None, "label": ""}
    lock = threading.Lock()

    def make_markers() -> MarkerArray:
        markers = MarkerArray()
        for index, did in enumerate(("1", "2", "3", "4")):
            block = outlets.get(did) or {}
            xyz = block.get("press_pose_xyz_m")
            if not xyz:
                continue
            sphere = Marker()
            sphere.header.frame_id = "base_link"
            sphere.ns = "press_pose"
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(xyz[0])
            sphere.pose.position.y = float(xyz[1])
            sphere.pose.position.z = float(xyz[2])
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.03
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.3, 0.1, 0.9
            markers.markers.append(sphere)
            text = Marker()
            text.header.frame_id = "base_link"
            text.ns = "press_pose_label"
            text.id = index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(xyz[0])
            text.pose.position.y = float(xyz[1])
            text.pose.position.z = float(xyz[2]) + 0.05
            text.pose.orientation.w = 1.0
            text.scale.z = 0.04
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f"press {did}"
            markers.markers.append(text)
        return markers

    def spin_loop() -> None:
        rate_sec = 1.0 / 30.0
        step_per_tick = math.radians(20.0) * rate_sec  # 20 deg/s preview speed
        while rclpy.ok():
            with lock:
                target = current["target"]
                if target is not None:
                    done = True
                    for i in range(6):
                        delta = math.radians(target[i]) - current["q"][i]
                        if abs(delta) > step_per_tick:
                            current["q"][i] += math.copysign(step_per_tick, delta)
                            done = False
                        else:
                            current["q"][i] = math.radians(target[i])
                    if done:
                        current["target"] = None
                msg = JointState()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.name = [f"joint_{i}" for i in range(1, 7)]
                msg.position = list(current["q"])
            joint_pub.publish(msg)
            marker_pub.publish(make_markers())
            time.sleep(rate_sec)

    with lock:
        current["q"] = [math.radians(v) for v in (poses[0][1] if poses else [0.0] * 6)]
    thread = threading.Thread(target=spin_loop, daemon=True)
    thread.start()

    print("\n[RViz] /joint_states 퍼블리시 중. hardware_free_demo.launch.py RViz에서 로봇이 보여야 합니다.")
    print("[RViz] 빨간 구슬 = calibration의 press_pose_xyz_m (목표 레인). 로봇 그리퍼가 어느 구슬 위인지 비교하세요.")
    for index, (name, joints) in enumerate(poses, start=1):
        xyz = flange_fk_mm(chain, joints)
        print(f"\n[RViz] ({index}/{len(poses)}) 슬롯 {name} → FK 예상 {lane_guess(xyz[1])}")
        answer = input("  Enter=이 포즈로 미리보기 이동 / s=건너뜀 / q=종료: ").strip().lower()
        if answer == "q":
            break
        if answer == "s":
            continue
        with lock:
            current["target"] = list(joints)
        input(f"  >> RViz에서 {name} 자세 확인 후 Enter: ")
    rclpy.shutdown()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="실제 로봇 모션 실행")
    parser.add_argument("--rviz", action="store_true", help="실로봇 대신 RViz 조인트 프리뷰로 재생")
    parser.add_argument("--confirm", default="", help=f"실행 시 {CONFIRM_PHRASE} 필요")
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--velocity", type=float, default=10.0)
    parser.add_argument("--skip-home-between", action="store_true",
                        help="포즈 사이 조인트 홈 경유 생략 (권장하지 않음)")
    parser.add_argument("--only", default="", help="예: 1_pre,3_contact 처럼 일부만 재생")
    args = parser.parse_args()

    if args.execute and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --execute에는 --confirm {CONFIRM_PHRASE} 가 필요합니다.", file=sys.stderr)
        return 2

    data = yaml.safe_load(CALIBRATION.read_text(encoding="utf-8")) or {}
    outlets = data.get("dispenser_outlets") or {}
    chain = load_chain()

    poses: list[tuple[str, list[float]]] = []
    for did in ("1", "2", "3", "4"):
        block = outlets.get(did) or {}
        for kind, key in (("pre", "press_pre_joints_deg"), ("contact", "press_contact_joints_deg")):
            joints = block.get(key)
            if joints is None:
                continue
            if kind == "contact":
                status = str(block.get("press_contact_status", "")).strip()
                if status.lower() in INVALID_PRESS_CONTACT_STATUSES:
                    print(
                        f"[BLOCKED] {did}_contact is marked {status!r}; "
                        f"skipping stale PRESS{did}_CONTACT until re-taught"
                    )
                    continue
            poses.append((f"{did}_{kind}", [float(v) for v in joints]))

    selected = {token.strip() for token in args.only.split(",") if token.strip()}
    if selected:
        poses = [p for p in poses if p[0] in selected]

    print(f"[replay] calibration: {CALIBRATION}")
    print(f"[replay] {len(poses)}개 포즈 재생 예정 (홈 경유 {'생략' if args.skip_home_between else '포함'})")
    print(f"{'슬롯':12s} {'FK flange xyz(mm)':28s} 예상 레인")
    for name, joints in poses:
        xyz = flange_fk_mm(chain, joints)
        print(f"{name:12s} [{xyz[0]:7.1f}, {xyz[1]:7.1f}, {xyz[2]:7.1f}]  {lane_guess(xyz[1])}")

    if args.rviz:
        return run_rviz_preview(poses, chain, outlets)

    if not args.execute:
        print("\n[DRY-RUN] --execute 미지정: 모션 없음. 위 표로 예상 레인만 확인하세요. (--rviz로 시각 확인 가능)")
        return 0

    for index, (name, joints) in enumerate(poses, start=1):
        xyz = flange_fk_mm(chain, joints)
        print(f"\n[replay] ({index}/{len(poses)}) 슬롯 {name} → 예상 {lane_guess(xyz[1])}")
        answer = input("  Enter=이동 / s=건너뜀 / q=종료: ").strip().lower()
        if answer == "q":
            break
        if answer == "s":
            continue
        if not args.skip_home_between:
            movej(SAFE_HOME_DEG, service_prefix=args.service_prefix,
                  velocity=args.velocity, label="safe joint home transit")
        movej(joints, service_prefix=args.service_prefix,
              velocity=args.velocity, label=f"registered pose {name}")
        input(f"  >> 지금 로봇이 실제로 몇 번 펌프 위에 있는지 기록하세요 ({name}). Enter=다음: ")

    if not args.skip_home_between:
        movej(SAFE_HOME_DEG, service_prefix=args.service_prefix,
              velocity=args.velocity, label="final safe joint home")
    print("[replay] 완료. 기록한 실제 레인에 맞게 calibration.yaml 슬롯을 재배치하세요.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
