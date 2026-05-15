import rclpy
from azas_interfaces.srv import SetGripper
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class RG2GripperNode(Node):
    """ROS service boundary for dry-run or real OnRobot RG2 commands."""

    def __init__(self):
        super().__init__("rg2_gripper_node")
        self.declare_parameter("use_real_hardware", False)
        self.declare_parameter("gripper", "rg2")
        self.declare_parameter("host", "192.168.1.1")
        self.declare_parameter("port", 502)
        self.declare_parameter("default_open_width_m", 0.110)
        self.declare_parameter("default_close_width_m", 0.0)
        self.declare_parameter("default_force_n", 40.0)

        self.use_real_hardware = (
            self.get_parameter("use_real_hardware").get_parameter_value().bool_value
        )
        self.gripper = None
        if self.use_real_hardware:
            self.gripper = self._connect_real_gripper()

        self.create_service(SetGripper, "/azas/gripper/open_close", self.on_set_gripper)
        if self.use_real_hardware:
            self.get_logger().info(
                "RG2 hardware service ready on /azas/gripper/open_close"
            )
        else:
            self.get_logger().warn(
                "Dry-run gripper service ready on /azas/gripper/open_close; "
                "set use_real_hardware:=true to command the real RG2"
            )

    def _connect_real_gripper(self):
        from azas_gripper.onrobot import RG

        gripper = self.get_parameter("gripper").get_parameter_value().string_value
        host = self.get_parameter("host").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        self.get_logger().info(f"Connecting to OnRobot {gripper} at {host}:{port}")
        return RG(gripper, host, port)

    def _width_m_to_register_units(self, width_m):
        width_units = int(round(width_m * 10000.0))
        return max(0, min(width_units, self.gripper.max_width))

    def _force_n_to_register_units(self, force_n):
        force_units = int(round(force_n * 10.0))
        return max(0, min(force_units, self.gripper.max_force))

    def on_set_gripper(self, request, response):
        command = request.command.lower()
        if command not in {"open", "close", "set_width"}:
            response.success = False
            response.message = f"unsupported command: {request.command}"
            return response

        width_m = float(request.width_m)
        force_n = float(request.force_n)
        if force_n <= 0.0:
            force_n = (
                self.get_parameter("default_force_n").get_parameter_value().double_value
            )
        if command == "open" and width_m <= 0.0:
            width_m = (
                self.get_parameter("default_open_width_m")
                .get_parameter_value()
                .double_value
            )
        elif command == "close" and width_m <= 0.0:
            width_m = (
                self.get_parameter("default_close_width_m")
                .get_parameter_value()
                .double_value
            )

        self.get_logger().info(
            f"gripper command={command} width_m={width_m:.3f} force_n={force_n:.1f}"
        )
        if self.use_real_hardware:
            try:
                width_units = self._width_m_to_register_units(width_m)
                force_units = self._force_n_to_register_units(force_n)
                self.gripper.move_gripper(width_units, force_units)
            except Exception as exc:
                response.success = False
                response.message = f"RG2 command failed: {exc}"
                self.get_logger().error(response.message)
                return response

            response.success = True
            response.message = (
                f"sent RG2 {command} command "
                f"width_units={width_units} force_units={force_units}"
            )
            return response

        response.success = True
        response.message = (
            "accepted dry-run command; real RG2 was not commanded"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RG2GripperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
