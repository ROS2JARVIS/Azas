#!/usr/bin/env python3
"""Pattern-A human handover: place the side-gripped cup onto an open palm.

This is an HRI motion (the robot moves toward a person). It follows
docs/post_shake_human_handover_plan.md with every gate kept explicit:

  1. PERCEPTION   sample /azas/human_hand_detection (run the detector first:
                  bash tools/run/run_human_hand_detection.sh) and transform the
                  palm into base frame via live TF base_link->link_6 and the
                  measured T_gripper2camera hand-eye calibration.
  2. PLAN         compute LIFT -> ABOVE_HIGH -> ABOVE_PALM -> staged descent
                  -> RELEASE -> RETREAT, all with the CURRENT side-grip
                  orientation preserved (--use-current-rpy on every MoveLine).
  3. GATES        default is dry-run. --execute needs --confirm, a typed
                  operator approval before any motion, a hand re-check right
                  before the descent, force-monitored descent steps, and a
                  second typed approval before the gripper opens.

Every Cartesian move is delegated to tools/run/direct_movel_xyz.py, which
enforces workspace bounds, IK precheck, and target verification on its own.

First-run advice: validate with a foam block or an empty palm-height surface
before any person, and tune --release-tcp-above-palm-m from that test.

Usage:
  python3 tools/run/handover_cup_to_palm.py                      # dry-run plan
  python3 tools/run/handover_cup_to_palm.py --execute --confirm ENABLE_HUMAN_PALM_HANDOVER
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DIRECT_MOVEL = ROOT / "tools" / "run" / "direct_movel_xyz.py"
RG2_OPEN = ROOT / "tools" / "run" / "rg2_full_open_verify.sh"
DEFAULT_HAND_EYE = ROOT / "src" / "azas_perception" / "config" / "T_gripper2camera.npy"
HAND_TOPIC = "/azas/human_hand_detection"
CONFIRM_PHRASE = "ENABLE_HUMAN_PALM_HANDOVER"
MOTION_APPROVAL_PHRASE = "ENABLE_HUMAN_PALM_HANDOVER_MOTION"
RELEASE_APPROVAL_PHRASE = "RELEASE_CUP_NOW"
DIRECT_CONFIRM_PHRASE = "ENABLE_DIRECT_MOVEL"


class HandoverPerception:
    """rclpy helpers: palm sampling, live TCP pose, tool force. No motion."""

    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy
        import tf2_ros
        from dsr_msgs2.srv import GetCurrentPosx, GetToolForce
        from geometry_msgs.msg import PointStamped

        self.args = args
        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = rclpy.create_node("azas_handover_cup_to_palm")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        prefix = args.service_prefix
        self.get_posx = self.node.create_client(GetCurrentPosx, f"/{prefix}/aux_control/get_current_posx")
        self.get_tool_force = self.node.create_client(GetToolForce, f"/{prefix}/aux_control/get_tool_force")
        self.hand_points: list[tuple[float, list[float]]] = []
        self.node.create_subscription(PointStamped, HAND_TOPIC, self._on_hand, 10)
        self.gripper2cam = np.load(str(args.hand_eye_npy)).astype(float)
        if abs(self.gripper2cam[:3, 3]).max() > 10.0:
            self.gripper2cam[:3, 3] /= 1000.0

    def close(self) -> None:
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()

    def _on_hand(self, msg) -> None:
        self.hand_points.append((time.monotonic(), [msg.point.x, msg.point.y, msg.point.z]))

    def _call(self, client, request, *, label: str, retries: int = 2):
        if not client.wait_for_service(timeout_sec=self.args.wait_service_sec):
            raise RuntimeError(f"{label} service unavailable")
        # Doosan aux services can time out on the first cold call; retry once.
        for attempt in range(1, retries + 1):
            future = client.call_async(request)
            self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.args.wait_service_sec)
            response = future.result()
            if response is not None:
                return response
            print(f"[Azas] {label} attempt {attempt}/{retries} timed out; retrying", file=sys.stderr)
        raise RuntimeError(f"{label} timed out after {retries} attempts")

    def current_posx(self) -> list[float]:
        from dsr_msgs2.srv import GetCurrentPosx

        req = GetCurrentPosx.Request()
        req.ref = 0  # DR_BASE
        response = self._call(self.get_posx, req, label="GetCurrentPosx")
        if not response.success or not response.task_pos_info:
            raise RuntimeError("GetCurrentPosx returned success=false")
        return [float(v) for v in list(response.task_pos_info[0].data)[:6]]

    def tool_force_n(self) -> list[float]:
        from dsr_msgs2.srv import GetToolForce

        req = GetToolForce.Request()
        req.ref = 0
        response = self._call(self.get_tool_force, req, label="GetToolForce")
        if not response.success:
            raise RuntimeError("GetToolForce returned success=false")
        return [float(v) for v in list(response.tool_force)[:3]]

    def base_to_camera(self) -> np.ndarray:
        import rclpy.time

        deadline = time.monotonic() + self.args.wait_service_sec
        last_error = ""
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            try:
                t = self.tf_buffer.lookup_transform("base_link", "link_6", rclpy.time.Time())
                break
            except Exception as exc:  # tf2 exception types vary by install
                last_error = str(exc)
        else:
            raise RuntimeError(f"TF base_link->link_6 unavailable: {last_error}")
        q = t.transform.rotation
        tr = t.transform.translation
        xx, yy, zz, ww = q.x, q.y, q.z, q.w
        rot = np.array(
            [
                [1 - 2 * (yy * yy + zz * zz), 2 * (xx * yy - zz * ww), 2 * (xx * zz + yy * ww)],
                [2 * (xx * yy + zz * ww), 1 - 2 * (xx * xx + zz * zz), 2 * (yy * zz - xx * ww)],
                [2 * (xx * zz - yy * ww), 2 * (yy * zz + xx * ww), 1 - 2 * (xx * xx + yy * yy)],
            ]
        )
        base2ee = np.eye(4)
        base2ee[:3, :3] = rot
        base2ee[:3, 3] = [tr.x, tr.y, tr.z]
        return base2ee @ self.gripper2cam

    def sample_palm_base(self, *, label: str) -> list[float]:
        """Collect stable hand detections and return the palm in base frame (m)."""
        if self.args.test_hand_xyz_m:
            xyz = [float(v) for v in self.args.test_hand_xyz_m.split(",")]
            print(f"[Azas] {label}: TEST palm injected at base xyz={xyz} (no camera sample)")
            return xyz
        self.hand_points.clear()
        deadline = time.monotonic() + self.args.hand_sample_timeout_sec
        while time.monotonic() < deadline and len(self.hand_points) < self.args.hand_sample_count:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        if len(self.hand_points) < self.args.hand_sample_count:
            raise RuntimeError(
                f"{label}: only {len(self.hand_points)}/{self.args.hand_sample_count} stable hand "
                f"detections within {self.args.hand_sample_timeout_sec:.1f}s; is "
                "run_human_hand_detection.sh running and the palm open and steady?"
            )
        base2cam = self.base_to_camera()
        base_points = []
        for _, cam_xyz in self.hand_points[-self.args.hand_sample_count:]:
            base_points.append((base2cam @ np.array([*cam_xyz, 1.0]))[:3])
        base_points = np.array(base_points)
        spread = float(np.max(np.linalg.norm(base_points - base_points.mean(axis=0), axis=1)))
        palm = base_points.mean(axis=0).tolist()
        print(
            f"[Azas] {label}: palm_base_m=[{palm[0]:.3f}, {palm[1]:.3f}, {palm[2]:.3f}] "
            f"samples={len(base_points)} spread={spread * 1000.0:.1f}mm"
        )
        if spread > self.args.hand_sample_spread_max_m:
            raise RuntimeError(
                f"{label}: palm samples spread {spread * 1000.0:.1f}mm exceeds "
                f"{self.args.hand_sample_spread_max_m * 1000.0:.1f}mm; hand or robot is moving"
            )
        return palm


def run_movel(args: argparse.Namespace, xyz_m: list[float], *, label: str, velocity: float, acceleration: float) -> None:
    cmd = [
        sys.executable, str(DIRECT_MOVEL),
        "--service-prefix", args.service_prefix,
        "--x", f"{xyz_m[0]:.6f}", "--y", f"{xyz_m[1]:.6f}", "--z", f"{xyz_m[2]:.6f}",
        "--use-current-rpy",
        "--velocity", f"{velocity:.3f}",
        "--acceleration", f"{acceleration:.3f}",
        "--timeout-sec", f"{args.move_timeout_sec:.1f}",
        "--wait-service-sec", f"{args.wait_service_sec:.1f}",
        "--x-min", f"{args.x_min:.3f}", "--x-max", f"{args.x_max:.3f}",
        "--y-min", f"{args.y_min:.3f}", "--y-max", f"{args.y_max:.3f}",
        "--z-min", f"{args.z_min:.3f}", "--z-max", f"{args.z_max:.3f}",
    ]
    if args.execute:
        cmd += ["--precheck-ikin", "--verify-target", "--execute", "--confirm", DIRECT_CONFIRM_PHRASE]
    print(f"[Azas] MOVE {label}: xyz_m=[{xyz_m[0]:.3f}, {xyz_m[1]:.3f}, {xyz_m[2]:.3f}] vel={velocity:.1f}")
    rc = subprocess.run(cmd, cwd=str(ROOT), check=False).returncode
    if rc != 0:
        raise RuntimeError(f"MoveLine step failed: {label} (rc={rc})")


def open_gripper(args: argparse.Namespace) -> None:
    env = os.environ.copy()
    env.setdefault("RG2_OPEN_TIMEOUT_SEC", "20.0")
    rc = subprocess.run([str(RG2_OPEN)], cwd=str(ROOT), env=env, check=False).returncode
    if rc != 0:
        raise RuntimeError(f"RG2 open failed (rc={rc})")


def require_typed_approval(phrase: str, *, prompt: str, preapproved: str = "") -> None:
    print(prompt)
    if preapproved.strip() == phrase:
        print(f"[Azas] approval {phrase} supplied non-interactively (panel/wrapper mode)")
        return
    entered = input(f"Type {phrase} to continue: ").strip()
    if entered != phrase:
        raise RuntimeError(f"operator approval mismatch; expected {phrase}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--service-prefix", default="dsr01")
    parser.add_argument("--hand-eye-npy", type=Path, default=DEFAULT_HAND_EYE)
    parser.add_argument("--hand-sample-count", type=int, default=10)
    parser.add_argument("--hand-sample-timeout-sec", type=float, default=20.0)
    parser.add_argument("--hand-sample-spread-max-m", type=float, default=0.03)
    parser.add_argument("--hand-recheck-tolerance-m", type=float, default=0.05,
                        help="abort if the palm moved more than this between plan and descent")
    parser.add_argument("--transit-z-m", type=float, default=0.45)
    parser.add_argument("--above-palm-m", type=float, default=0.12,
                        help="TCP height above the palm before the staged descent")
    parser.add_argument("--release-tcp-above-palm-m", type=float, default=0.08,
                        help="TCP height above the palm at release. TUNE WITH A FOAM-BLOCK "
                             "DRY TEST FIRST: depends on where the side grip holds the cup")
    parser.add_argument("--descent-step-m", type=float, default=0.02)
    parser.add_argument("--force-abort-delta-n", type=float, default=10.0,
                        help="abort descent when |tool force| rises this much over the pre-descent baseline")
    parser.add_argument("--retreat-lift-m", type=float, default=0.20)
    parser.add_argument("--transit-velocity", type=float, default=10.0)
    parser.add_argument("--transit-acceleration", type=float, default=14.0)
    parser.add_argument("--descent-velocity", type=float, default=4.0)
    parser.add_argument("--descent-acceleration", type=float, default=6.0)
    # Palm workspace bounds (base frame). The palm itself must be inside these.
    parser.add_argument("--x-min", type=float, default=0.25)
    parser.add_argument("--x-max", type=float, default=0.75)
    parser.add_argument("--y-min", type=float, default=-0.45)
    parser.add_argument("--y-max", type=float, default=0.45)
    parser.add_argument("--z-min", type=float, default=0.05)
    parser.add_argument("--z-max", type=float, default=0.60)
    parser.add_argument("--palm-z-max-m", type=float, default=0.40,
                        help="reject palms higher than this (likely a mis-detection)")
    parser.add_argument("--move-timeout-sec", type=float, default=60.0)
    parser.add_argument("--wait-service-sec", type=float, default=10.0)
    parser.add_argument("--auto-release", action="store_true",
                        help="skip the final typed release approval (NOT recommended)")
    parser.add_argument("--test-hand-xyz-m", default="",
                        help="debug: skip camera sampling and use this base-frame palm 'x,y,z' (meters)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="", help=f"must equal {CONFIRM_PHRASE} with --execute")
    parser.add_argument("--approve-motion", default="",
                        help=f"non-interactive operator approval; must equal {MOTION_APPROVAL_PHRASE} "
                             "(for panel/wrapper use where stdin is unavailable)")
    parser.add_argument("--approve-release", default="",
                        help=f"non-interactive release approval; must equal {RELEASE_APPROVAL_PHRASE}")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.execute and args.confirm != CONFIRM_PHRASE:
        print(f"[BLOCKED] --execute requires --confirm {CONFIRM_PHRASE}")
        return 2
    if not args.hand_eye_npy.is_file():
        print(f"[FAIL] hand-eye calibration not found: {args.hand_eye_npy}")
        return 2
    if args.release_tcp_above_palm_m >= args.above_palm_m:
        print("[BLOCKED] --release-tcp-above-palm-m must be below --above-palm-m")
        return 2
    if not args.execute:
        print("[DRY-RUN] --execute not set; perception + plan only, no robot command sent.")

    perception = HandoverPerception(args)
    try:
        # --- PERCEPTION + PLAN (no motion) ---
        current = perception.current_posx()
        current_m = [v / 1000.0 for v in current[:3]]
        print(
            f"[Azas] current TCP: xyz_m=[{current_m[0]:.3f}, {current_m[1]:.3f}, {current_m[2]:.3f}] "
            f"rpy_deg=[{current[3]:.1f}, {current[4]:.1f}, {current[5]:.1f}] (orientation is preserved)"
        )
        palm = perception.sample_palm_base(label="palm plan sample")
        if not (args.x_min <= palm[0] <= args.x_max and args.y_min <= palm[1] <= args.y_max
                and args.z_min <= palm[2] <= min(args.z_max, args.palm_z_max_m)):
            print(f"[BLOCKED] palm outside handover workspace bounds; refusing: palm={palm}")
            return 1

        lift = [current_m[0], current_m[1], max(current_m[2], args.transit_z_m)]
        above_high = [palm[0], palm[1], max(args.transit_z_m, palm[2] + args.above_palm_m)]
        above_palm = [palm[0], palm[1], palm[2] + args.above_palm_m]
        release = [palm[0], palm[1], palm[2] + args.release_tcp_above_palm_m]
        retreat = [palm[0], palm[1], palm[2] + args.retreat_lift_m]
        for name, pose in (("LIFT", lift), ("ABOVE_HIGH", above_high), ("ABOVE_PALM", above_palm),
                           ("RELEASE", release), ("RETREAT", retreat)):
            print(f"[PLAN] {name}: xyz_m=[{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}]")
        print(
            "[PLAN] descent ABOVE_PALM -> RELEASE in "
            f"{math.ceil((above_palm[2] - release[2]) / max(args.descent_step_m, 0.005))} steps of "
            f"{args.descent_step_m * 1000.0:.0f}mm with force abort delta {args.force_abort_delta_n:.1f}N"
        )
        if not args.execute:
            return 0

        # --- GATED EXECUTION ---
        require_typed_approval(
            MOTION_APPROVAL_PHRASE,
            prompt=(
                "[Azas] HRI MOTION APPROVAL REQUIRED. Confirm ALL:\n"
                "  - e-stop within reach\n"
                "  - only the receiving person is near the robot, arm steady, palm open\n"
                "  - first run was validated on a foam block, not a person\n"
                "  - speeds/bounds above were reviewed"
            ),
            preapproved=args.approve_motion,
        )
        run_movel(args, lift, label="LIFT to transit height (Z-only)",
                  velocity=args.transit_velocity, acceleration=args.transit_acceleration)
        run_movel(args, above_high, label="ABOVE_HIGH over palm at transit height",
                  velocity=args.transit_velocity, acceleration=args.transit_acceleration)
        run_movel(args, above_palm, label="ABOVE_PALM vertical pre-descent",
                  velocity=args.descent_velocity, acceleration=args.descent_acceleration)

        # Hand must still be where we planned; people move.
        recheck = perception.sample_palm_base(label="palm re-check before descent")
        moved = math.dist(recheck, palm)
        if moved > args.hand_recheck_tolerance_m:
            print(f"[ABORT] palm moved {moved * 1000.0:.0f}mm since planning; retreating without descent")
            run_movel(args, retreat, label="RETREAT after palm moved",
                      velocity=args.transit_velocity, acceleration=args.transit_acceleration)
            return 1

        baseline = perception.tool_force_n()
        baseline_mag = math.sqrt(sum(v * v for v in baseline))
        z = above_palm[2]
        while z > release[2] + 1e-6:
            z = max(z - max(args.descent_step_m, 0.005), release[2])
            run_movel(args, [palm[0], palm[1], z], label=f"descent step to z={z:.3f}m",
                      velocity=args.descent_velocity, acceleration=args.descent_acceleration)
            force = perception.tool_force_n()
            force_mag = math.sqrt(sum(v * v for v in force))
            print(f"[Azas] tool force {force_mag:.1f}N (baseline {baseline_mag:.1f}N)")
            if force_mag - baseline_mag > args.force_abort_delta_n:
                print("[ABORT] force spike during descent (palm contact or obstruction); retreating with cup")
                run_movel(args, retreat, label="RETREAT after force abort",
                          velocity=args.transit_velocity, acceleration=args.transit_acceleration)
                return 1

        if not args.auto_release:
            require_typed_approval(
                RELEASE_APPROVAL_PHRASE,
                prompt="[Azas] Cup is at release height. Confirm the palm is directly under the cup.",
                preapproved=args.approve_release,
            )
        open_gripper(args)
        time.sleep(1.0)
        run_movel(args, retreat, label="RETREAT vertical after release",
                  velocity=args.transit_velocity, acceleration=args.transit_acceleration)
        print("[PASS] palm handover sequence completed")
        return 0
    except RuntimeError as exc:
        print(f"[FAIL] {exc}")
        return 1
    finally:
        perception.close()


if __name__ == "__main__":
    raise SystemExit(main())
