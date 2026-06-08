import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from azas_voice.command_parser import parse_recipe_command


class RecipeMapperNode(Node):
    """Map STT text to symbolic recipe decisions only.

    This node intentionally does not generate robot coordinates, trajectories,
    collision decisions, or safety approvals.
    """

    def __init__(self):
        super().__init__("recipe_mapper_node")
        self.declare_parameter("stt_topic", "/stt_result")
        self.declare_parameter("decision_topic", "/azas/voice/recipe_decision")
        self.declare_parameter("confirmation_topic", "/azas/voice/confirmation")

        stt_topic = self.get_parameter("stt_topic").value
        decision_topic = self.get_parameter("decision_topic").value
        confirmation_topic = self.get_parameter("confirmation_topic").value

        self._decision_pub = self.create_publisher(String, decision_topic, 10)
        self._confirmation_pub = self.create_publisher(String, confirmation_topic, 10)
        self._sub = self.create_subscription(String, stt_topic, self._on_stt, 10)

        self.get_logger().info(
            f"Recipe mapper ready: {stt_topic} -> {decision_topic}, {confirmation_topic}"
        )

    def _on_stt(self, msg: String) -> None:
        decision = parse_recipe_command(msg.data)
        payload = String()
        payload.data = json.dumps(decision.to_dict(), ensure_ascii=False)
        self._decision_pub.publish(payload)

        if decision.confirmation:
            confirmation = String()
            confirmation.data = decision.confirmation
            self._confirmation_pub.publish(confirmation)

        if decision.valid:
            self.get_logger().info(payload.data)
        else:
            self.get_logger().warn(payload.data)


def main(args=None):
    rclpy.init(args=args)
    node = RecipeMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
