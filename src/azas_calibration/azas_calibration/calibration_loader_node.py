import rclpy
from azas_interfaces.srv import CalibrateOutlet, SaveCupOffset
from rclpy.node import Node


class CalibrationLoaderNode(Node):
    """Boundary for measured calibration data; never invents hardware transforms."""

    def __init__(self):
        super().__init__("calibration_loader_node")
        self.create_service(
            CalibrateOutlet,
            "/azas/calibration/set_dispenser_outlet",
            self.on_calibrate_outlet,
        )
        self.create_service(
            SaveCupOffset,
            "/azas/calibration/save_cup_offset",
            self.on_save_cup_offset,
        )
        self.get_logger().warn(
            "Calibration values are placeholders until measured: camera_frame, "
            "EE_LINK/GROUP_NAME, dispenser_outlet, and TCP-to-cup offset all require confirmation."
        )

    def on_calibrate_outlet(self, request, response):
        if not request.outlet_id:
            response.success = False
            response.message = "outlet_id is required; measured pose not saved"
            return response
        response.success = False
        response.message = (
            "service boundary only: persist measured dispenser_outlet after calibration workflow is implemented"
        )
        return response

    def on_save_cup_offset(self, request, response):
        if not request.cup_type:
            response.success = False
            response.message = "cup_type is required; measured offset not saved"
            return response
        response.success = False
        response.message = (
            "service boundary only: persist measured tcp_to_cup_mouth after jig/dry-run calibration"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationLoaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
