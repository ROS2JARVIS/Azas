from __future__ import annotations

import copy
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ConversationManagerNode(Node):
    """Keep pending recipe state and emit confirmed execution requests."""

    def __init__(self):
        super().__init__("conversation_manager_node")

        self.declare_parameter("decision_topic", "/azas/voice/recipe_decision")
        self.declare_parameter("confirmation_topic", "/azas/voice/confirmation")
        self.declare_parameter("confirmed_decision_topic", "/azas/voice/confirmed_recipe_decision")
        self.declare_parameter("pending_timeout_s", 30.0)

        self._pending: dict[str, object] | None = None
        self._pending_at = 0.0
        self._timeout_s = float(self.get_parameter("pending_timeout_s").value)

        self._confirmation_pub = self.create_publisher(
            String,
            str(self.get_parameter("confirmation_topic").value),
            10,
        )
        self._confirmed_pub = self.create_publisher(
            String,
            str(self.get_parameter("confirmed_decision_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("decision_topic").value),
            self._on_decision,
            10,
        )

        self.get_logger().info(
            "Conversation manager ready: "
            f"{self.get_parameter('decision_topic').value} -> "
            f"{self.get_parameter('confirmed_decision_topic').value}"
        )

    def _on_decision(self, msg: String) -> None:
        try:
            decision = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._publish_confirmation(f"음성 인식 결과를 처리하지 못했습니다: {exc}")
            return

        intent = str(decision.get("intent", "unknown"))
        if intent == "make_cocktail":
            self._handle_make_cocktail(decision)
        elif intent == "confirm":
            self._handle_confirm(decision)
        elif intent == "cancel":
            self._pending = None
            self._publish_confirmation(str(decision.get("confirmation") or "취소했습니다."))
        elif decision.get("valid"):
            self._publish_confirmation(str(decision.get("confirmation") or "명령을 확인했습니다."))
        else:
            self._publish_confirmation("메뉴를 다시 말씀해주세요.")

    def _handle_make_cocktail(self, decision: dict[str, object]) -> None:
        if not decision.get("valid"):
            self._pending = None
            self._publish_confirmation("메뉴를 다시 말씀해주세요.")
            return

        if not decision.get("recipe_id") and not decision.get("dispenser_ids"):
            self._pending = None
            self._publish_confirmation("선택할 메뉴를 찾지 못했습니다.")
            return

        self._pending = copy.deepcopy(decision)
        self._pending_at = time.monotonic()
        self._publish_confirmation(str(decision.get("confirmation") or "진행할까요?"))

    def _handle_confirm(self, decision: dict[str, object]) -> None:
        if self._pending is None:
            self._publish_confirmation("먼저 메뉴를 선택해주세요.")
            return

        if time.monotonic() - self._pending_at > self._timeout_s:
            self._pending = None
            self._publish_confirmation("이전 주문이 만료되었습니다. 메뉴를 다시 말씀해주세요.")
            return

        confirmed = copy.deepcopy(self._pending)
        confirmed["confirmed"] = True
        confirmed["confirmed_by"] = "voice"
        confirmed["confirm_utterance"] = decision.get("utterance", "")
        confirmed["confirmation"] = "제조를 시작합니다."
        self._pending = None

        msg = String()
        msg.data = json.dumps(confirmed, ensure_ascii=False)
        self._confirmed_pub.publish(msg)
        self.get_logger().info(msg.data)
        self._publish_confirmation("제조를 시작합니다.")

    def _publish_confirmation(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._confirmation_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConversationManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
