#!/usr/bin/env python3
"""Send a SetGripper command with RG2 bridge readiness recovery.

The Azas RG2 wrapper accepts Modbus write requests but does not expose actual
finger-position feedback.  This script therefore verifies the strongest software
evidence available: the ROS service exists, returns success=True, and the command
payload is logged.  If the service is absent, it can start the RG2 bridge once and
retry instead of letting `ros2 service call` time out with an invalid rcl context.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from azas_interfaces.srv import SetGripper

ROOT = Path("/home/ssu/Azas")
LOG_DIR = ROOT / "log" / "panel"
ROS_SETUP = (
    "source /opt/ros/humble/setup.bash && "
    "source /home/ssu/ros2_ws/install/setup.bash && "
    "source /home/ssu/Azas/install/setup.bash"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Robust RG2 SetGripper command wrapper")
    parser.add_argument("--service", default=os.environ.get("RG2_SET_WIDTH_SERVICE", "/jarvis/rg2/set_width"))
    parser.add_argument("--command", required=True, choices=["open", "close", "set_width", "preopen", "grasp"])
    parser.add_argument("--width-m", type=float, required=True)
    parser.add_argument("--force-n", type=float, required=True)
    parser.add_argument("--timeout-sec", type=float, default=12.0)
    parser.add_argument("--ready-timeout-sec", type=float, default=18.0)
    parser.add_argument("--settle-sec", type=float, default=0.6)
    parser.add_argument("--auto-start", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rg2-ip", default=os.environ.get("RG2_IP", "192.168.1.1"))
    parser.add_argument("--rg2-port", default=os.environ.get("RG2_PORT", "502"))
    return parser.parse_args()


def wait_for_service(node: Any, service: str, timeout_sec: float) -> bool:
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    client = node.create_client(SetGripper, service)
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.5):
            node.destroy_client(client)
            return True
    node.destroy_client(client)
    return False


def start_rg2_bridge(args: argparse.Namespace) -> Path:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    log_path = LOG_DIR / f"connect_gripper-autostart-{stamp}.log"
    cmd = (
        f"cd {ROOT} && {ROS_SETUP} && "
        "ros2 launch azas_gripper rg2_trigger.launch.py "
        f"ip:={args.rg2_ip} port:={args.rg2_port} connect:=true "
        "open_width:=1100 close_width:=0 force:=300 settle_seconds:=0.6"
    )
    handle = log_path.open("w", encoding="utf-8", buffering=1)
    handle.write(f"[Azas] RG2 auto-start command: {cmd}\n\n")
    subprocess.Popen(
        ["bash", "-lc", cmd],
        cwd=str(ROOT),
        stdout=handle,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    handle.close()
    return log_path


def call_set_gripper(args: argparse.Namespace) -> tuple[bool, str]:
    node = rclpy.create_node("azas_rg2_set_width_verify")
    try:
        client = node.create_client(SetGripper, args.service)
        if not client.wait_for_service(timeout_sec=max(args.ready_timeout_sec, 0.1)):
            return False, f"service not available after {args.ready_timeout_sec:.1f}s: {args.service}"
        req = SetGripper.Request()
        req.command = args.command
        req.width_m = float(args.width_m)
        req.force_n = float(args.force_n)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(args.timeout_sec, 0.1))
        if not future.done():
            return False, f"response timeout after {args.timeout_sec:.1f}s"
        if future.exception() is not None:
            return False, f"service exception: {future.exception()}"
        response = future.result()
        if response is None:
            return False, "service returned no response"
        return bool(response.success), str(response.message)
    finally:
        node.destroy_node()


def main() -> int:
    args = parse_args()
    print("[Azas] RG2 set-width verified command")
    print(f"[Azas] service={args.service}")
    print(f"[Azas] command={args.command} width_m={args.width_m:.3f} force_n={args.force_n:.1f}")

    rclpy.init(args=None)
    node = rclpy.create_node("azas_rg2_set_width_ready_probe")
    try:
        ready = wait_for_service(node, args.service, min(args.ready_timeout_sec, 3.0))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not ready and args.auto_start:
        log_path = start_rg2_bridge(args)
        print(f"[Azas] RG2 service absent; auto-started bridge log={log_path}")
        time.sleep(max(args.settle_sec, 0.0))
    elif not ready:
        print(f"[FAIL] RG2 service absent and auto-start disabled: {args.service}")
        return 1

    rclpy.init(args=None)
    try:
        ok, message = call_set_gripper(args)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    print(f"[Azas] response: success={ok} message='{message}'")
    if not ok:
        print("[FAIL] RG2 command was not accepted by the ROS wrapper")
        return 1

    print(
        "[PASS] RG2 command accepted "
        f"(target_width_m={args.width_m:.3f}; actual finger feedback is not exposed)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
