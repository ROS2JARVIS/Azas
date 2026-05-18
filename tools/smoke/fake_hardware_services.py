#!/usr/bin/env python3
"""Fake Doosan motion and RG2 services for Azas hardware-gated smoke tests.

This node never talks to hardware. It only records service requests and returns
success=True so the hardware-armed control path can be verified safely.
"""

from __future__ import annotations

import rclpy
from azas_interfaces.srv import SetGripper
from dsr_msgs2.srv import MoveJoint, MoveLine
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


class FakeHardwareServices(Node):
    def __init__(self) -> None:
        super().__init__("azas_fake_hardware_services")
        self.declare_parameter("service_prefix", "")

        prefix = str(self.get_parameter("service_prefix").value).strip("/")
        motion_prefix = f"/{prefix}/motion" if prefix else "/motion"
        self.create_service(MoveJoint, f"{motion_prefix}/move_joint", self.on_move_joint)
        self.create_service(MoveLine, f"{motion_prefix}/move_line", self.on_move_line)
        self.create_service(Trigger, "/jarvis/rg2/open", self.on_open)
        self.create_service(Trigger, "/jarvis/rg2/close", self.on_close)
        self.create_service(SetGripper, "/jarvis/rg2/set_width", self.on_set_width)
        self.get_logger().info(
            "Fake/no-motion hardware services ready; does not command real RG2 or Doosan: "
            f"{motion_prefix}/move_joint, {motion_prefix}/move_line, "
            "/jarvis/rg2/open, /jarvis/rg2/close, /jarvis/rg2/set_width"
        )

    def on_move_joint(self, request, response):
        self.get_logger().info(
            "fake move_joint: "
            f"pos={list(request.pos)} vel={request.vel} acc={request.acc}"
        )
        response.success = True
        return response

    def on_move_line(self, request, response):
        self.get_logger().info(
            "fake move_line: "
            f"pos={list(request.pos)} vel={list(request.vel)} acc={list(request.acc)} "
            f"ref={request.ref} mode={request.mode}"
        )
        response.success = True
        return response

    def on_open(self, request, response):
        response.success = True
        response.message = "fake RG2 open; does not command real RG2"
        self.get_logger().info(response.message)
        return response

    def on_close(self, request, response):
        response.success = True
        response.message = "fake RG2 close; does not command real RG2"
        self.get_logger().info(response.message)
        return response

    def on_set_width(self, request, response):
        response.success = True
        response.message = "fake RG2 set_width; does not command real RG2"
        self.get_logger().info(
            "fake RG2 set_width: "
            f"command={request.command} width_m={request.width_m:.3f} force_n={request.force_n:.1f}"
        )
        return response


def main() -> None:
    rclpy.init()
    node = FakeHardwareServices()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
