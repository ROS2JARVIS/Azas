#!/usr/bin/env python3
"""Fake Doosan motion and RG2 services for Azas hardware-gated smoke tests.

This node never talks to hardware. It only records service requests and returns
success=True so the hardware-armed control path can be verified safely.
"""

from __future__ import annotations

import rclpy
from azas_interfaces.srv import SetGripper
from dsr_msgs2.srv import (
    GetCurrentPosj,
    GetCurrentPosx,
    GetCurrentTcp,
    MoveJoint,
    MoveLine,
    MoveWait,
    SetCurrentTcp,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


class FakeHardwareServices(Node):
    def __init__(self) -> None:
        super().__init__("azas_fake_hardware_services")
        self.declare_parameter("service_prefix", "")

        prefix = str(self.get_parameter("service_prefix").value).strip("/")
        motion_prefix = f"/{prefix}/motion" if prefix else "/motion"
        aux_prefix = f"/{prefix}/aux_control" if prefix else "/aux_control"
        tcp_prefix = f"/{prefix}/tcp" if prefix else "/tcp"
        self.current_posx = [224.819, 4.105, 453.241, 89.8, 179.9, 120.3]
        self.current_posj = [0.0, -35.0, 50.0, 0.0, 70.0, 0.0]
        self.current_tcp = "GripperDA_v1_jarvis"
        self.create_service(MoveJoint, f"{motion_prefix}/move_joint", self.on_move_joint)
        self.create_service(MoveLine, f"{motion_prefix}/move_line", self.on_move_line)
        self.create_service(MoveWait, f"{motion_prefix}/move_wait", self.on_move_wait)
        self.create_service(
            GetCurrentPosx,
            f"{aux_prefix}/get_current_posx",
            self.on_get_current_posx,
        )
        self.create_service(
            GetCurrentPosj,
            f"{aux_prefix}/get_current_posj",
            self.on_get_current_posj,
        )
        self.create_service(GetCurrentTcp, f"{tcp_prefix}/get_current_tcp", self.on_get_current_tcp)
        self.create_service(SetCurrentTcp, f"{tcp_prefix}/set_current_tcp", self.on_set_current_tcp)
        self.create_service(Trigger, "/jarvis/rg2/open", self.on_open)
        self.create_service(Trigger, "/jarvis/rg2/close", self.on_close)
        self.create_service(SetGripper, "/jarvis/rg2/set_width", self.on_set_width)
        self.get_logger().info(
            "Fake/no-motion hardware services ready; does not command real RG2 or Doosan: "
            f"{motion_prefix}/move_joint, {motion_prefix}/move_line, "
            f"{motion_prefix}/move_wait, {aux_prefix}/get_current_posx, "
            f"{aux_prefix}/get_current_posj, "
            f"{tcp_prefix}/get_current_tcp, {tcp_prefix}/set_current_tcp, "
            "/jarvis/rg2/open, /jarvis/rg2/close, /jarvis/rg2/set_width"
        )

    def on_move_joint(self, request, response):
        self.get_logger().info(
            "fake move_joint: "
            f"pos={list(request.pos)} vel={request.vel} acc={request.acc}"
        )
        self.current_posj = [float(value) for value in request.pos[:6]]
        self.current_posx = [224.819, 4.105, 453.241, 89.8, 179.9, 120.3]
        response.success = True
        return response

    def on_move_line(self, request, response):
        self.current_posx = [float(value) for value in request.pos[:6]]
        self.get_logger().info(
            "fake move_line: "
            f"pos={list(request.pos)} vel={list(request.vel)} acc={list(request.acc)} "
            f"ref={request.ref} mode={request.mode}"
        )
        response.success = True
        return response

    def on_move_wait(self, request, response):
        response.success = True
        self.get_logger().info("fake move_wait")
        return response

    def on_get_current_posx(self, request, response):
        response.task_pos_info = [Float64MultiArray(data=list(self.current_posx))]
        response.success = True
        self.get_logger().info(f"fake get_current_posx: pos={self.current_posx}")
        return response

    def on_get_current_posj(self, request, response):
        response.pos = list(self.current_posj)
        response.success = True
        self.get_logger().info(f"fake get_current_posj: pos={self.current_posj}")
        return response

    def on_get_current_tcp(self, request, response):
        response.info = self.current_tcp
        response.success = True
        self.get_logger().info(f"fake get_current_tcp: {self.current_tcp}")
        return response

    def on_set_current_tcp(self, request, response):
        self.current_tcp = str(request.name).strip()
        response.success = True
        self.get_logger().info(f"fake set_current_tcp: {self.current_tcp}")
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
