import rclpy
from azas_interfaces.msg import CupDetection
from rclpy.node import Node


class SimulatedCupDetectionNode(Node):
    """Publish a deterministic CupDetection for code-only grasp pipeline tests.

    This node never touches a camera, robot, gripper, or hardware service. It is
    only a synthetic perception source for verifying launch wiring and dry-run
    grasp planning before real camera/robot integration.
    """

    def __init__(self):
        super().__init__("simulated_cup_detection_node")
        self.declare_parameter("output_topic", "/azas/sim/cup_detection")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("publish_once", True)
        self.declare_parameter("confidence", 0.95)
        self.declare_parameter("status", "detected:upright class=simulated_cup")
        self.declare_parameter("source", "simulated_cup_detection_node")
        self.declare_parameter("grasp_x", 0.32)
        self.declare_parameter("grasp_y", -0.22)
        self.declare_parameter("grasp_z", 0.05)
        self.declare_parameter("mouth_x", 0.32)
        self.declare_parameter("mouth_y", -0.22)
        self.declare_parameter("mouth_z", 0.22)

        self._published = False
        self._publisher = self.create_publisher(
            CupDetection,
            str(self.get_parameter("output_topic").value),
            10,
        )
        period = max(0.05, float(self.get_parameter("publish_period_sec").value))
        self._timer = self.create_timer(period, self._publish_detection)
        self.get_logger().info(
            "Simulated cup detection ready: "
            f"topic={self.get_parameter('output_topic').value} "
            f"frame={self.get_parameter('frame_id').value} "
            "hardware=disabled camera=disabled"
        )

    def _publish_detection(self) -> None:
        if bool(self.get_parameter("publish_once").value) and self._published:
            return

        msg = CupDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.grasp_pose.position.x = float(self.get_parameter("grasp_x").value)
        msg.grasp_pose.position.y = float(self.get_parameter("grasp_y").value)
        msg.grasp_pose.position.z = float(self.get_parameter("grasp_z").value)
        msg.grasp_pose.orientation.w = 1.0
        msg.cup_mouth_center.position.x = float(self.get_parameter("mouth_x").value)
        msg.cup_mouth_center.position.y = float(self.get_parameter("mouth_y").value)
        msg.cup_mouth_center.position.z = float(self.get_parameter("mouth_z").value)
        msg.cup_mouth_center.orientation.w = 1.0
        msg.confidence = float(self.get_parameter("confidence").value)
        msg.status = str(self.get_parameter("status").value)
        msg.source = str(self.get_parameter("source").value)

        self._publisher.publish(msg)
        self._published = True
        self.get_logger().info(
            "Published simulated CupDetection: "
            f"grasp=({msg.grasp_pose.position.x:.3f}, {msg.grasp_pose.position.y:.3f}, {msg.grasp_pose.position.z:.3f}) "
            f"frame={msg.header.frame_id} conf={msg.confidence:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedCupDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
