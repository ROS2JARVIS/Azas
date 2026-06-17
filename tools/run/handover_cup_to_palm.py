#!/usr/bin/env python3
"""Pattern-A human handover: place the side-gripped cup onto an open palm.

This is an HRI motion (the robot moves toward a person). It follows
docs/post_shake_human_handover_plan.md with every gate kept explicit:

  1. PERCEPTION   sample /azas/human_hand_detection (run the detector first:
                  bash tools/run/run_human_hand_detection.sh) and transform the
                  palm into base frame via live TF base_link->link_6 and the
                  measured T_gripper2camera hand-eye calibration.
  2. PLAN         compute LIFT -> APPROACH -> ABOVE_PALM -> staged descent
                  -> RELEASE/CONTACT_RELEASE -> RETREAT, all with the CURRENT side-grip
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


def prefixed_service(prefix: str, suffix: str) -> str:
    clean = prefix.strip("/")
    return f"/{clean}/{suffix}" if clean else f"/{suffix}"


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def parse_ikin_sol_spaces(value: str) -> list[int]:
    values = [int(part.strip()) for part in str(value).split(",") if part.strip()]
    if not values:
        raise ValueError("--ikin-sol-spaces did not contain any solution spaces")
    return values


def resolve_service_prefix(node, srv_type, requested_prefix: str, wait_sec: float, *, allow_fallback: bool) -> str:
    requested = requested_prefix.strip("/")
    if requested or not allow_fallback:
        return requested
    for candidate in ("", "dsr01"):
        name = prefixed_service(candidate, "aux_control/get_current_posx")
        client = node.create_client(srv_type, name)
        if client.wait_for_service(timeout_sec=max(0.1, wait_sec)):
            return candidate
    return requested


class HandoverPerception:
    """rclpy helpers: palm sampling, live TCP pose, tool force. No motion."""

    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy
        import tf2_ros
        from dsr_msgs2.srv import GetCurrentPosx, GetToolForce, Ikin
        from geometry_msgs.msg import PointStamped
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        self.args = args
        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = rclpy.create_node("azas_handover_cup_to_palm")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        prefix = resolve_service_prefix(
            self.node,
            GetCurrentPosx,
            args.service_prefix,
            min(self.args.wait_service_sec, 1.0),
            allow_fallback=not args.no_service_prefix_fallback,
        )
        self.args.service_prefix = prefix
        print(f"[Azas] Doosan service prefix: {prefix or '<none>'}")
        self.get_posx = self.node.create_client(
            GetCurrentPosx, prefixed_service(prefix, "aux_control/get_current_posx")
        )
        self.get_tool_force = self.node.create_client(
            GetToolForce, prefixed_service(prefix, "aux_control/get_tool_force")
        )
        self.ikin = self.node.create_client(Ikin, prefixed_service(prefix, "motion/ikin"))
        self.hand_points: list[tuple[float, list[float]]] = []
        hand_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.node.create_subscription(PointStamped, HAND_TOPIC, self._on_hand, hand_qos)
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

    def averaged_tool_force_n(self, *, samples: int, interval_sec: float) -> list[float]:
        count = max(int(samples), 1)
        total = [0.0, 0.0, 0.0]
        for index in range(count):
            force = self.tool_force_n()
            for axis in range(3):
                total[axis] += force[axis]
            if index + 1 < count:
                time.sleep(max(interval_sec, 0.0))
        return [value / count for value in total]

    def ikin_pose_ok(
        self,
        xyz_m: list[float],
        rpy_deg: list[float],
        *,
        args: argparse.Namespace,
        label: str,
    ) -> tuple[bool, str]:
        from dsr_msgs2.srv import Ikin

        timeout_sec = max(float(args.ikin_timeout_sec), 0.1)
        if not self.ikin.wait_for_service(timeout_sec=timeout_sec):
            return False, f"{label}: Ikin service unavailable"
        try:
            sol_spaces = parse_ikin_sol_spaces(args.ikin_sol_spaces)
        except ValueError as exc:
            return False, str(exc)

        pos_mm_deg = [
            xyz_m[0] * 1000.0,
            xyz_m[1] * 1000.0,
            xyz_m[2] * 1000.0,
            rpy_deg[0],
            rpy_deg[1],
            rpy_deg[2],
        ]
        last_failure = ""
        for sol_space in sol_spaces:
            req = Ikin.Request()
            req.pos = pos_mm_deg
            req.sol_space = int(sol_space)
            req.ref = 0  # DR_BASE
            future = self.ikin.call_async(req)
            self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
            if not future.done():
                last_failure = f"sol_space={sol_space} timeout after {timeout_sec:.1f}s"
                continue
            if future.exception() is not None:
                last_failure = f"sol_space={sol_space} exception: {future.exception()}"
                continue
            response = future.result()
            if response is None or not bool(response.success):
                last_failure = f"sol_space={sol_space} success=false"
                continue
            joints = [float(value) for value in list(response.conv_posj)]
            if len(joints) >= 5 and not float(args.j5_min_deg) <= joints[4] <= float(args.j5_max_deg):
                last_failure = (
                    f"sol_space={sol_space} joint_5={joints[4]:.1f} outside "
                    f"[{float(args.j5_min_deg):.1f}, {float(args.j5_max_deg):.1f}]"
                )
                continue
            return True, f"sol_space={sol_space}"
        return False, last_failure or "no IK solution"

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


def run_movel(
    args: argparse.Namespace,
    xyz_m: list[float],
    *,
    label: str,
    velocity: float,
    acceleration: float,
    rpy_deg: list[float],
) -> None:
    cmd = [
        sys.executable, str(DIRECT_MOVEL),
        "--service-prefix", args.service_prefix,
        "--x", f"{xyz_m[0]:.6f}", "--y", f"{xyz_m[1]:.6f}", "--z", f"{xyz_m[2]:.6f}",
        "--rx", f"{rpy_deg[0]:.6f}", "--ry", f"{rpy_deg[1]:.6f}", "--rz", f"{rpy_deg[2]:.6f}",
        "--velocity", f"{velocity:.3f}",
        "--acceleration", f"{acceleration:.3f}",
        "--timeout-sec", f"{args.move_timeout_sec:.1f}",
        "--motion-timeout-sec", f"{args.move_timeout_sec:.1f}",
        "--wait-service-sec", f"{args.wait_service_sec:.1f}",
        "--verify-timeout-sec", f"{args.verify_timeout_sec:.1f}",
        "--target-tolerance-mm", f"{args.target_tolerance_mm:.1f}",
        "--ikin-timeout-sec", f"{args.ikin_timeout_sec:.1f}",
        "--ikin-retries", str(args.ikin_retries),
        "--ikin-sol-spaces", args.ikin_sol_spaces,
        "--j5-min-deg", f"{args.j5_min_deg:.3f}",
        "--j5-max-deg", f"{args.j5_max_deg:.3f}",
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


def monitored_axis_indices(mode: str) -> list[int]:
    if mode == "all":
        return [0, 1, 2]
    if mode == "xy":
        return [0, 1]
    if mode == "z":
        return [2]
    raise ValueError(f"unsupported contact axis mode: {mode}")


def force_contact_metrics(
    force: list[float],
    baseline: list[float],
    baseline_mag: float,
    *,
    contact_axis: str,
) -> tuple[float, list[float], float]:
    force_mag = math.sqrt(sum(v * v for v in force))
    axis_delta = [force[i] - baseline[i] for i in range(3)]
    max_axis_delta = max(abs(axis_delta[i]) for i in monitored_axis_indices(contact_axis))
    mag_delta = force_mag - baseline_mag
    return mag_delta, axis_delta, max_axis_delta


def contact_axis_hit(axis_delta: list[float], mag_delta: float, *, args: argparse.Namespace) -> bool:
    if args.require_force_magnitude_delta and mag_delta < args.force_magnitude_delta_n:
        return False
    if args.contact_axis == "z":
        z_delta = axis_delta[2]
        if args.contact_z_direction == "positive":
            return z_delta > args.force_axis_delta_n
        if args.contact_z_direction == "negative":
            return z_delta < -args.force_axis_delta_n
        return abs(z_delta) > args.force_axis_delta_n
    if args.contact_axis == "xy":
        return max(abs(axis_delta[0]), abs(axis_delta[1])) > args.force_axis_delta_n
    return (
        max(abs(axis_delta[0]), abs(axis_delta[1]), abs(axis_delta[2])) > args.force_axis_delta_n
        or mag_delta > args.force_abort_delta_n
    )


def contact_step_hit(force: list[float], previous_force: list[float], *, args: argparse.Namespace) -> bool:
    step_delta = [force[i] - previous_force[i] for i in range(3)]
    if args.contact_axis == "z":
        z_delta = step_delta[2]
        if args.contact_z_direction == "positive":
            return z_delta > args.contact_step_delta_n
        if args.contact_z_direction == "negative":
            return z_delta < -args.contact_step_delta_n
        return abs(z_delta) > args.contact_step_delta_n
    if args.contact_axis == "xy":
        return max(abs(step_delta[0]), abs(step_delta[1])) > args.contact_step_delta_n
    return max(abs(step_delta[0]), abs(step_delta[1]), abs(step_delta[2])) > args.contact_step_delta_n


def force_contact_confirmed(
    perception: HandoverPerception,
    reference_force: list[float],
    reference_mag: float,
    *,
    force_delta_n: float,
    axis_delta_n: float,
    contact_axis: str,
    samples: int,
    min_hits: int,
    interval_sec: float,
) -> bool:
    needed = max(int(samples), 1)
    required_hits = min(max(int(min_hits), 1), needed)
    hits = 0
    for index in range(needed):
        force = perception.tool_force_n()
        mag_delta, axis_delta, max_axis_delta = force_contact_metrics(
            force,
            reference_force,
            reference_mag,
            contact_axis=contact_axis,
        )
        confirm_args = argparse.Namespace(
            contact_axis=contact_axis,
            contact_z_direction=perception.args.contact_z_direction,
            force_axis_delta_n=axis_delta_n,
            force_abort_delta_n=force_delta_n,
            require_force_magnitude_delta=perception.args.require_force_magnitude_delta,
            force_magnitude_delta_n=perception.args.force_magnitude_delta_n,
        )
        hit = contact_axis_hit(axis_delta, mag_delta, args=confirm_args)
        hits += 1 if hit else 0
        print(
            "[Azas] contact confirm "
            f"{index + 1}/{needed}: hit={hit} hits={hits}/{required_hits} "
            f"fx={force[0]:.2f} fy={force[1]:.2f} fz={force[2]:.2f} "
            f"delta_mag={mag_delta:.2f}N "
            f"delta_axis=[{axis_delta[0]:.2f}, {axis_delta[1]:.2f}, {axis_delta[2]:.2f}] "
            f"max_{contact_axis}_axis={max_axis_delta:.2f}N"
        )
        if hits >= required_hits:
            return True
        remaining = needed - (index + 1)
        if hits + remaining < required_hits:
            return False
        if index + 1 < needed:
            time.sleep(max(interval_sec, 0.0))
    return hits >= required_hits


def open_gripper(args: argparse.Namespace) -> None:
    env = os.environ.copy()
    env.setdefault("RG2_OPEN_TIMEOUT_SEC", "20.0")
    env.setdefault("RG2_OPEN_RETRIES", str(args.gripper_open_retries))
    env.setdefault("RG2_OPEN_RETRY_SLEEP_SEC", f"{args.gripper_open_retry_sleep_sec:.1f}")
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


def bound_handover_xy(x: float, y: float, args: argparse.Namespace) -> list[float]:
    x = clamp(x, args.handover_target_x_min_m, args.handover_target_x_max_m)
    y = clamp(y, args.handover_target_y_min_m, args.handover_target_y_max_m)
    radius = math.hypot(x, y)
    min_radius = max(float(args.handover_target_xy_radius_min_m), 0.0)
    max_radius = max(float(args.handover_target_xy_radius_max_m), min_radius + 1e-6)
    if radius > max_radius:
        scale = max_radius / radius
        x *= scale
        y *= scale
    elif 1e-6 < radius < min_radius:
        scale = min_radius / radius
        x *= scale
        y *= scale
    return [
        clamp(x, args.handover_target_x_min_m, args.handover_target_x_max_m),
        clamp(y, args.handover_target_y_min_m, args.handover_target_y_max_m),
    ]


def handover_descent_z_limits(palm_z: float, args: argparse.Namespace) -> tuple[float, float]:
    start_z = palm_z + max(float(args.force_search_start_above_palm_m), 0.0)
    if args.release_on_contact:
        floor_z = max(float(args.z_min), palm_z - max(float(args.force_search_below_palm_m), 0.0))
    else:
        floor_z = palm_z + float(args.release_tcp_above_palm_m)
    return start_z, floor_z


def ik_probe_poses_for_handover_target(
    xy: list[float],
    palm_z: float,
    args: argparse.Namespace,
) -> list[list[float]]:
    start_z, floor_z = handover_descent_z_limits(palm_z, args)
    z_values = [start_z, floor_z]
    if abs(start_z - floor_z) > 0.04:
        z_values.insert(1, (start_z + floor_z) * 0.5)
    return [[xy[0], xy[1], clamp(z, args.z_min, args.z_max)] for z in z_values]


def select_ik_reachable_handover_target(
    perception: HandoverPerception,
    palm: list[float],
    current_m: list[float],
    rpy_deg: list[float],
    args: argparse.Namespace,
) -> list[float]:
    nearest_xy = bound_handover_xy(palm[0], palm[1], args)
    current_xy = bound_handover_xy(current_m[0], current_m[1], args)
    max_adjust = max(float(args.max_handover_target_adjust_m), 0.0)
    last_failure = ""
    seen: set[tuple[float, float]] = set()

    # Start at the closest bounded point to the detected palm. If that is still
    # outside the robot's IK envelope, pull the target toward the current robot
    # side-grip pose in small increments and use the first fully IK-valid point.
    for blend in (0.0, 0.10, 0.20, 0.35, 0.50, 0.65, 0.80, 1.0):
        xy = [
            nearest_xy[0] + (current_xy[0] - nearest_xy[0]) * blend,
            nearest_xy[1] + (current_xy[1] - nearest_xy[1]) * blend,
        ]
        xy = bound_handover_xy(xy[0], xy[1], args)
        key = (round(xy[0], 4), round(xy[1], 4))
        if key in seen:
            continue
        seen.add(key)
        adjust_m = math.dist([palm[0], palm[1]], xy)
        if max_adjust > 0.0 and adjust_m > max_adjust:
            last_failure = (
                f"candidate x={xy[0]:.3f} y={xy[1]:.3f} is "
                f"{adjust_m * 1000.0:.0f}mm from detected palm, above "
                f"max_handover_target_adjust={max_adjust * 1000.0:.0f}mm"
            )
            continue

        probe_poses = ik_probe_poses_for_handover_target(xy, palm[2], args)
        failures = []
        for index, pose in enumerate(probe_poses, start=1):
            ok, detail = perception.ikin_pose_ok(pose, rpy_deg, args=args, label=f"handover_target_probe_{index}")
            if not ok:
                failures.append(f"z={pose[2]:.3f}: {detail}")
        if not failures:
            if adjust_m > 0.001:
                print(
                    "[Azas] handover target adjusted toward IK-valid workspace: "
                    f"palm_xy=[{palm[0]:.3f}, {palm[1]:.3f}] "
                    f"target_xy=[{xy[0]:.3f}, {xy[1]:.3f}] "
                    f"offset={adjust_m * 1000.0:.0f}mm"
                )
            else:
                print("[Azas] detected palm XY is inside the IK-checked handover target envelope")
            return [xy[0], xy[1], palm[2]]
        last_failure = "; ".join(failures)
        print(
            "[Azas] IK rejected handover target candidate: "
            f"x={xy[0]:.3f} y={xy[1]:.3f} offset={adjust_m * 1000.0:.0f}mm; "
            f"{last_failure}"
        )

    raise RuntimeError(
        "No IK-valid handover target found near the detected palm. "
        f"Last failure: {last_failure}. Move the palm closer to the robot's front-center handover area."
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--service-prefix", default=os.environ.get("SERVICE_PREFIX", ""))
    parser.add_argument("--no-service-prefix-fallback", action="store_true",
                        help="use exactly --service-prefix, including empty prefix; do not fall back to dsr01")
    parser.add_argument("--hand-eye-npy", type=Path, default=DEFAULT_HAND_EYE)
    parser.add_argument("--hand-sample-count", type=int, default=10)
    parser.add_argument("--hand-sample-timeout-sec", type=float, default=20.0)
    parser.add_argument("--hand-sample-spread-max-m", type=float, default=0.03)
    parser.add_argument("--hand-recheck-tolerance-m", type=float, default=0.05,
                        help="abort if the palm moved more than this between plan and descent")
    parser.add_argument("--skip-hand-recheck", action="store_true",
                        help="use the initially sampled palm for descent without the pre-descent palm re-check")
    parser.add_argument("--transit-z-m", type=float, default=0.45)
    parser.add_argument("--diagonal-approach", action=argparse.BooleanOptionalAction, default=True,
                        help="move toward the palm with XYZ blended to the descent-start height; "
                             "--no-diagonal-approach keeps the older XY-at-transit-height behavior")
    parser.add_argument("--above-palm-m", type=float, default=0.12,
                        help="TCP height above the palm before the staged descent")
    parser.add_argument("--release-tcp-above-palm-m", type=float, default=0.08,
                        help="TCP height above the palm at release. TUNE WITH A FOAM-BLOCK "
                             "DRY TEST FIRST: depends on where the side grip holds the cup")
    parser.add_argument("--force-search-start-above-palm-m", type=float, default=0.16,
                        help="with --release-on-contact, start force-only descent this far above the detected palm")
    parser.add_argument("--force-search-below-palm-m", type=float, default=0.10,
                        help="with --release-on-contact, search down to this far below the detected palm before aborting")
    parser.add_argument("--descent-step-m", type=float, default=0.03)
    parser.add_argument("--first-descent-step-m", type=float, default=0.08,
                        help="first Z descent step before force/contact checks; subsequent steps use --descent-step-m")
    parser.add_argument("--max-descent-steps", type=int, default=0,
                        help="maximum staged descent steps; 0 means use the Z floor only")
    parser.add_argument("--force-abort-delta-n", type=float, default=0.6,
                        help="abort descent when |tool force| rises this much over the pre-descent baseline")
    parser.add_argument("--force-axis-delta-n", type=float, default=0.5,
                        help="trigger contact when the monitored force axis changes by this much")
    parser.add_argument("--contact-axis", choices=("z", "xy", "all"), default="all",
                        help="force axes used for contact release; all is most sensitive for handover release")
    parser.add_argument("--contact-z-direction", choices=("positive", "negative", "any"), default="any",
                        help="when --contact-axis z, require this signed Z force delta for contact")
    parser.add_argument("--contact-step-delta-n", type=float, default=0.3,
                        help="contact candidate also requires this force jump from the previous descent step")
    parser.add_argument("--require-force-magnitude-delta", action=argparse.BooleanOptionalAction, default=False,
                        help="also require total force magnitude to rise before contact release")
    parser.add_argument("--force-magnitude-delta-n", type=float, default=0.6,
                        help="minimum total force magnitude rise required with --require-force-magnitude-delta")
    parser.add_argument("--force-baseline-samples", type=int, default=5,
                        help="average this many GetToolForce samples before descent")
    parser.add_argument("--force-baseline-interval-sec", type=float, default=0.05,
                        help="delay between baseline force samples")
    parser.add_argument("--force-read-settle-sec", type=float, default=0.05,
                        help="wait after each descent step before reading force")
    parser.add_argument("--release-on-contact", action="store_true",
                        help="during staged descent, treat a force rise as palm contact: stop, open RG2, then retreat")
    parser.add_argument("--require-contact-for-release", action=argparse.BooleanOptionalAction, default=True,
                        help="with --release-on-contact, only open RG2 after contact is detected")
    parser.add_argument("--contact-confirm-samples", type=int, default=2,
                        help="consecutive above-threshold force samples required before opening RG2")
    parser.add_argument("--contact-confirm-min-hits", type=int, default=1,
                        help="minimum hit samples needed within --contact-confirm-samples; "
                             "0 means all samples, preserving the strict default")
    parser.add_argument("--contact-confirm-interval-sec", type=float, default=0.05,
                        help="delay between force confirmation samples")
    parser.add_argument("--contact-relief-lift-m", type=float, default=0.0,
                        help="deprecated/ignored: contact release now opens RG2 at the confirmed contact pose")
    parser.add_argument("--contact-search-below-release-m", type=float, default=0.20,
                        help="with --release-on-contact, keep descending this far below release height while seeking contact")
    parser.add_argument("--retreat-lift-m", type=float, default=0.20)
    parser.add_argument("--transit-velocity", type=float, default=75.0)
    parser.add_argument("--transit-acceleration", type=float, default=95.0)
    parser.add_argument("--descent-velocity", type=float, default=22.0)
    parser.add_argument("--descent-acceleration", type=float, default=32.0)
    # Palm workspace bounds (base frame). The palm itself must be inside these.
    parser.add_argument("--x-min", type=float, default=0.15)
    parser.add_argument("--x-max", type=float, default=1.50)
    parser.add_argument("--y-min", type=float, default=-0.65)
    parser.add_argument("--y-max", type=float, default=0.75)
    parser.add_argument("--z-min", type=float, default=0.04)
    parser.add_argument("--z-max", type=float, default=0.75)
    parser.add_argument("--palm-z-max-m", type=float, default=0.50,
                        help="reject palms higher than this (likely a mis-detection)")
    parser.add_argument("--handover-target-x-min-m", type=float, default=0.18,
                        help="minimum IK-biased TCP target x for handover; detected palm is not rewritten")
    parser.add_argument("--handover-target-x-max-m", type=float, default=0.65,
                        help="maximum IK-biased TCP target x for handover")
    parser.add_argument("--handover-target-y-min-m", type=float, default=-0.55,
                        help="minimum IK-biased TCP target y for handover")
    parser.add_argument("--handover-target-y-max-m", type=float, default=0.55,
                        help="maximum IK-biased TCP target y for handover")
    parser.add_argument("--handover-target-xy-radius-min-m", type=float, default=0.25,
                        help="minimum base XY radius for the IK-biased handover target")
    parser.add_argument("--handover-target-xy-radius-max-m", type=float, default=0.62,
                        help="maximum base XY radius for the IK-biased handover target")
    parser.add_argument("--max-handover-target-adjust-m", type=float, default=0.20,
                        help="fail closed if the IK-valid handover target would be farther from the detected palm")
    parser.add_argument("--move-timeout-sec", type=float, default=60.0)
    parser.add_argument("--verify-timeout-sec", type=float, default=90.0)
    parser.add_argument("--target-tolerance-mm", type=float, default=25.0)
    parser.add_argument("--ikin-timeout-sec", type=float, default=20.0)
    parser.add_argument("--ikin-retries", type=int, default=2)
    parser.add_argument("--ikin-sol-spaces", default="2,0,1,3,4,5,6,7",
                        help="solution spaces to try for every MoveLine IK precheck")
    parser.add_argument("--j5-min-deg", type=float, default=-160.0)
    parser.add_argument("--j5-max-deg", type=float, default=160.0)
    parser.add_argument("--wait-service-sec", type=float, default=10.0)
    parser.add_argument("--skip-force-monitor", action="store_true",
                        help="skip GetToolForce monitoring during descent; keeps staged descent and release approval")
    parser.add_argument("--gripper-open-retries", type=int, default=3)
    parser.add_argument("--gripper-open-retry-sleep-sec", type=float, default=1.0)
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
    if args.release_on_contact and args.skip_force_monitor:
        print("[BLOCKED] --release-on-contact requires force monitoring; remove --skip-force-monitor")
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
        preserved_rpy = current[3:6]
        handover_target = select_ik_reachable_handover_target(
            perception,
            palm,
            current_m,
            preserved_rpy,
            args,
        )

        lift = [current_m[0], current_m[1], max(current_m[2], args.transit_z_m)]
        contact_start_z = handover_target[2] + max(args.force_search_start_above_palm_m, 0.0)
        descent_start_z = (
            contact_start_z
            if args.release_on_contact
            else handover_target[2] + args.above_palm_m
        )
        if args.diagonal_approach:
            approach_z = max(args.z_min, min(args.z_max, descent_start_z))
            approach_label = "APPROACH"
        else:
            approach_z = max(args.transit_z_m, descent_start_z)
            approach_label = "ABOVE_HIGH"
        approach = [handover_target[0], handover_target[1], approach_z]
        above_palm = [
            handover_target[0],
            handover_target[1],
            descent_start_z,
        ]
        release = [
            handover_target[0],
            handover_target[1],
            handover_target[2] + args.release_tcp_above_palm_m,
        ]
        contact_floor_z = max(args.z_min, handover_target[2] - max(args.force_search_below_palm_m, 0.0))
        retreat = [
            handover_target[0],
            handover_target[1],
            handover_target[2] + args.retreat_lift_m,
        ]
        plan_items = [("LIFT", lift), (approach_label, approach), ("ABOVE_PALM", above_palm)]
        if not args.release_on_contact:
            plan_items.append(("RELEASE", release))
        plan_items.append(("RETREAT", retreat))
        for name, pose in plan_items:
            print(f"[PLAN] {name}: xyz_m=[{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}]")
        if args.release_on_contact:
            print(f"[PLAN] CONTACT_SEARCH_FLOOR: z_m={contact_floor_z:.3f}")
            print(
                "[PLAN] force-only Z search: "
                f"start_z={above_palm[2]:.3f} palm_z={palm[2]:.3f} floor_z={contact_floor_z:.3f}; "
                "gripper opens only after confirmed contact"
            )
        print(
            "[PLAN] descent ABOVE_PALM -> "
            f"{'CONTACT_SEARCH_FLOOR' if args.release_on_contact else 'RELEASE'} in "
            f"{math.ceil((above_palm[2] - (contact_floor_z if args.release_on_contact else release[2])) / max(args.descent_step_m, 0.005))} steps of "
            f"{args.first_descent_step_m * 1000.0:.0f}mm first / "
            f"{args.descent_step_m * 1000.0:.0f}mm subsequent with force abort delta {args.force_abort_delta_n:.1f}N"
        )
        if args.max_descent_steps > 0:
            no_contact_action = "retreat with cup" if args.require_contact_for_release else "open at final descent pose"
            print(f"[PLAN] max descent steps: {args.max_descent_steps} (no confirmed contact => {no_contact_action})")
        if args.release_on_contact:
            print("[PLAN] contact-release mode: keep descending until force/contact trigger, then open RG2")
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
                  velocity=args.transit_velocity, acceleration=args.transit_acceleration,
                  rpy_deg=preserved_rpy)
        approach_motion_label = (
            "APPROACH to palm descent start (XYZ blended)"
            if args.diagonal_approach else
            "ABOVE_HIGH over palm at transit height"
        )
        run_movel(args, approach, label=approach_motion_label,
                  velocity=args.transit_velocity, acceleration=args.transit_acceleration,
                  rpy_deg=preserved_rpy)
        if math.dist(approach, above_palm) > 0.001:
            run_movel(args, above_palm, label="ABOVE_PALM vertical pre-descent",
                      velocity=args.descent_velocity, acceleration=args.descent_acceleration,
                      rpy_deg=preserved_rpy)
        else:
            print("[Azas] ABOVE_PALM equals approach target; skipping duplicate pre-descent move")

        # Hand must still be where we planned; people move. This can be skipped
        # when the camera re-check is known to jump after arm motion.
        if args.skip_hand_recheck:
            print("[Azas] palm re-check skipped; descending to the initially sampled palm target")
        else:
            recheck = perception.sample_palm_base(label="palm re-check before descent")
            moved = math.dist(recheck, palm)
            if moved > args.hand_recheck_tolerance_m:
                print(f"[ABORT] palm moved {moved * 1000.0:.0f}mm since planning; retreating without descent")
                run_movel(args, retreat, label="RETREAT after palm moved",
                          velocity=args.transit_velocity, acceleration=args.transit_acceleration,
                          rpy_deg=preserved_rpy)
                return 1

        baseline = [0.0, 0.0, 0.0]
        baseline_mag = 0.0
        if args.skip_force_monitor:
            print("[Azas] force monitor skipped by operator option")
        else:
            baseline = perception.averaged_tool_force_n(
                samples=args.force_baseline_samples,
                interval_sec=args.force_baseline_interval_sec,
            )
            baseline_mag = math.sqrt(sum(v * v for v in baseline))
            print(
                "[Azas] force baseline: "
                f"fx={baseline[0]:.2f} fy={baseline[1]:.2f} fz={baseline[2]:.2f} "
                f"|f|={baseline_mag:.2f}N contact_axis={args.contact_axis} "
                f"contact_z_direction={args.contact_z_direction}"
            )
        z = above_palm[2]
        contact_release = False
        descent_floor_z = contact_floor_z if args.release_on_contact else release[2]
        previous_force = list(baseline)
        descent_step_index = 0
        while z > descent_floor_z + 1e-6:
            if args.max_descent_steps > 0 and descent_step_index >= args.max_descent_steps:
                print(f"[Azas] max descent steps reached ({args.max_descent_steps}) without confirmed contact")
                break
            descent_step_index += 1
            descent_step_m = args.first_descent_step_m if descent_step_index == 1 else args.descent_step_m
            z = max(z - max(descent_step_m, 0.005), descent_floor_z)
            run_movel(args, [handover_target[0], handover_target[1], z], label=f"descent step to z={z:.3f}m",
                      velocity=args.descent_velocity, acceleration=args.descent_acceleration,
                      rpy_deg=preserved_rpy)
            if not args.skip_force_monitor:
                time.sleep(max(args.force_read_settle_sec, 0.0))
                force = perception.tool_force_n()
                force_mag = math.sqrt(sum(v * v for v in force))
                mag_delta, axis_delta, max_axis_delta = force_contact_metrics(
                    force,
                    baseline,
                    baseline_mag,
                    contact_axis=args.contact_axis,
                )
                print(
                    "[Azas] tool force "
                    f"fx={force[0]:.2f} fy={force[1]:.2f} fz={force[2]:.2f} |f|={force_mag:.2f}N "
                    f"delta_mag={mag_delta:.2f}N "
                    f"delta_axis=[{axis_delta[0]:.2f}, {axis_delta[1]:.2f}, {axis_delta[2]:.2f}] "
                    f"step_delta=[{force[0] - previous_force[0]:.2f}, "
                    f"{force[1] - previous_force[1]:.2f}, {force[2] - previous_force[2]:.2f}] "
                    f"max_{args.contact_axis}_axis={max_axis_delta:.2f}N "
                    f"z_direction={args.contact_z_direction}"
                )
                contact_candidate = (
                    contact_axis_hit(axis_delta, mag_delta, args=args)
                    and contact_step_hit(force, previous_force, args=args)
                )
                if contact_candidate:
                    if args.release_on_contact:
                        print("[Azas] contact candidate detected; checking confirmation samples before RG2 open")
                        candidate_reference_force = list(previous_force)
                        candidate_reference_mag = math.sqrt(sum(v * v for v in candidate_reference_force))
                        if not force_contact_confirmed(
                            perception,
                            candidate_reference_force,
                            candidate_reference_mag,
                            force_delta_n=args.force_abort_delta_n,
                            axis_delta_n=args.force_axis_delta_n,
                            contact_axis=args.contact_axis,
                            samples=args.contact_confirm_samples,
                            min_hits=args.contact_confirm_min_hits or args.contact_confirm_samples,
                            interval_sec=args.contact_confirm_interval_sec,
                        ):
                            print(
                                "[Azas] contact candidate was not confirmed; "
                                "treating it as force noise and continuing descent"
                            )
                            previous_force = force
                            continue
                        contact_release = True
                        print(
                            "[Azas] contact trigger during descent: "
                            f"delta_mag={mag_delta:.2f}N(limit {args.force_abort_delta_n:.2f}), "
                            f"max_{args.contact_axis}_axis={max_axis_delta:.2f}N(limit {args.force_axis_delta_n:.2f}); "
                            "opening RG2 at the contact candidate pose"
                        )
                        break
                    print("[ABORT] force spike during descent (palm contact or obstruction); retreating with cup")
                    run_movel(args, retreat, label="RETREAT after force abort",
                              velocity=args.transit_velocity, acceleration=args.transit_acceleration,
                              rpy_deg=preserved_rpy)
                    return 1
                previous_force = force

        if args.release_on_contact and not contact_release:
            print("[Azas] contact search floor reached without contact trigger")
            if args.require_contact_for_release:
                print("[ABORT] contact was not detected; retreating with cup")
                run_movel(args, retreat, label="RETREAT after no contact",
                          velocity=args.transit_velocity, acceleration=args.transit_acceleration,
                          rpy_deg=preserved_rpy)
                return 1

        if args.release_on_contact and args.require_contact_for_release and not contact_release:
            print("[ABORT] fail-closed: contact release was not confirmed; gripper will stay closed")
            run_movel(args, retreat, label="RETREAT after unconfirmed contact release",
                      velocity=args.transit_velocity, acceleration=args.transit_acceleration,
                      rpy_deg=preserved_rpy)
            return 1

        if not args.auto_release:
            require_typed_approval(
                RELEASE_APPROVAL_PHRASE,
                prompt=(
                    "[Azas] Contact detected; confirm the palm is supporting the cup."
                    if contact_release else
                    "[Azas] Cup is at release height. Confirm the palm is directly under the cup."
                ),
                preapproved=args.approve_release,
            )
        open_gripper(args)
        time.sleep(1.0)
        run_movel(args, retreat, label="RETREAT vertical after release",
                  velocity=args.transit_velocity, acceleration=args.transit_acceleration,
                  rpy_deg=preserved_rpy)
        print("[PASS] palm handover sequence completed")
        return 0
    except RuntimeError as exc:
        print(f"[FAIL] {exc}")
        return 1
    finally:
        perception.close()


if __name__ == "__main__":
    raise SystemExit(main())
