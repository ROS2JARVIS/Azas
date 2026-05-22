#!/usr/bin/env python3
"""Call an empty-request ROS 2 service using rclpy.

This intentionally avoids `ros2 service call` because ros2cli parses the .srv
text before sending the request. Some installed Doosan dsr_msgs2 service files
can be imported by Python but fail that ros2cli parser path.
"""

from __future__ import annotations

import argparse
import importlib
import os
import sys
from pathlib import Path
from typing import Any

os.environ.setdefault("ROS_LOG_DIR", "/tmp/azas_ros_logs")
Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)

import rclpy


def load_service_class(type_name: str) -> Any:
    parts = type_name.split("/")
    if len(parts) != 3 or parts[1] != "srv":
        raise ValueError(f"expected service type like pkg/srv/Name, got {type_name!r}")
    module = importlib.import_module(f"{parts[0]}.srv")
    return getattr(module, parts[2])


def scalar_fields(message: Any) -> list[str]:
    fields = []
    for name in getattr(message, "__slots__", []):
        display_name = name.lstrip("_")
        value = getattr(message, name)
        if isinstance(value, (bool, int, float, str)):
            fields.append(f"{display_name}={value}")
        else:
            fields.append(f"{display_name}={value!r}")
    return fields


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("service_name")
    parser.add_argument("service_type")
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    service_class = load_service_class(args.service_type)
    rclpy.init(args=None)
    node = rclpy.create_node("azas_ros_call_empty_service")
    try:
        client = node.create_client(service_class, args.service_name)
        if not client.wait_for_service(timeout_sec=args.timeout):
            print(f"service not available: {args.service_name}", file=sys.stderr)
            return 1
        future = client.call_async(service_class.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        response = future.result()
        if response is None:
            print(f"service call timed out: {args.service_name}", file=sys.stderr)
            return 1
        for line in scalar_fields(response):
            print(line)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
