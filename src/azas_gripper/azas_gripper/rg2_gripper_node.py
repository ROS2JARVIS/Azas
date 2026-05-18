import rclpy
from azas_interfaces.srv import SetGripper
from rclpy.node import Node


class RG2GripperNode(Node):
    """Azas-internal placeholder boundary; it does not command a real RG2."""

    def __init__(self):
        super().__init__("rg2_gripper_node")
        self.create_service(SetGripper, "/azas/gripper/open_close", self.on_set_gripper)
        self.get_logger().warn(
            "Azas internal SetGripper placeholder ready on /azas/gripper/open_close; "
            "does not command real RG2 and does not provide /jarvis/rg2/* services"
        )

    def on_set_gripper(self, request, response):
        command = request.command.lower()
        if command not in {"open", "close", "set_width"}:
            response.success = False
            response.message = f"unsupported command: {request.command}"
            return response

        self.get_logger().info(
            f"gripper command={command} width_m={request.width_m:.3f} "
            f"force_n={request.force_n:.1f}"
        )
        response.success = True
        response.message = (
            "accepted no-motion placeholder command; does not command real RG2; "
            "RG2 hardware binding and units 확인 필요"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RG2GripperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
