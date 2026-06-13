import json
import time

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
        self.declare_parameter("publish_confirmation", True)
        self.declare_parameter("duplicate_utterance_window_s", 1.2)

        stt_topic = self.get_parameter("stt_topic").value
        decision_topic = self.get_parameter("decision_topic").value
        confirmation_topic = self.get_parameter("confirmation_topic").value
        self._publish_confirmation = bool(self.get_parameter("publish_confirmation").value)
        self._duplicate_window_s = max(
            0.0,
            float(self.get_parameter("duplicate_utterance_window_s").value),
        )
        self._last_normalized = ""
        self._last_intent = ""
        self._last_decision_at = 0.0

        self._decision_pub = self.create_publisher(String, decision_topic, 10)
        self._confirmation_pub = self.create_publisher(String, confirmation_topic, 10)
        self._sub = self.create_subscription(String, stt_topic, self._on_stt, 10)

        self.get_logger().info(
            f"Recipe mapper ready: {stt_topic} -> {decision_topic}, {confirmation_topic}"
        )

    def _on_stt(self, msg: String) -> None:
        decision = parse_recipe_command(msg.data)
        now = time.monotonic()
        if (
            self._duplicate_window_s > 0.0
            and decision.normalized
            and decision.normalized == self._last_normalized
            and decision.intent == self._last_intent
            and now - self._last_decision_at <= self._duplicate_window_s
        ):
            self.get_logger().info(
                "ignored duplicate STT utterance within "
                f"{self._duplicate_window_s:.1f}s: {decision.utterance}"
            )
            return
        self._last_normalized = decision.normalized
        self._last_intent = decision.intent
        self._last_decision_at = now

        payload = String()
        payload.data = json.dumps(decision.to_dict(), ensure_ascii=False)
        self._decision_pub.publish(payload)

        if self._publish_confirmation and decision.confirmation:
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
