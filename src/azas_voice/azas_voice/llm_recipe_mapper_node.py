import json
import os
from urllib import request

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - lets pure sanitizer tests run without ROS sourced.
    rclpy = None
    String = None
    Node = object

from azas_voice.command_parser import RecipeDecision, parse_recipe_command
from azas_voice.recipe_catalog import COLOR_ALIASES, RECIPE_DISPENSERS, RECIPE_DISPLAY_NAMES


ALLOWED_INTENTS = {"make_cocktail", "confirm", "cancel", "unknown"}
ALLOWED_DISPENSERS = {"1", "2", "3", "4"}


def _normalize_dispenser_id(value: object) -> str:
    raw = str(value).strip()
    if raw in ALLOWED_DISPENSERS:
        return raw
    normalized = "".join(raw.lower().split())
    for dispenser_id, aliases in COLOR_ALIASES.items():
        if any("".join(alias.lower().split()) == normalized for alias in aliases):
            return dispenser_id
    return ""


def _fallback_decision(text: str, reason: str = "") -> RecipeDecision:
    decision = parse_recipe_command(text)
    if decision.valid or not reason:
        return decision
    return RecipeDecision(
        decision.valid,
        decision.utterance,
        decision.normalized,
        decision.intent,
        decision.recipe_id,
        decision.dispenser_ids,
        decision.confirmation,
        f"{decision.error}; llm_fallback_reason={reason}",
    )


def _sanitize_llm_decision(text: str, payload: dict) -> RecipeDecision:
    intent = str(payload.get("intent", "unknown")).strip()
    if intent not in ALLOWED_INTENTS:
        return _fallback_decision(text, f"invalid_intent:{intent}")

    dispenser_ids = tuple(
        dispenser_id
        for dispenser_id in (_normalize_dispenser_id(item) for item in payload.get("dispenser_ids", []))
        if dispenser_id
    )
    recipe_id = payload.get("recipe_id")
    recipe_id = str(recipe_id).strip() if recipe_id else None
    if recipe_id and not recipe_id.startswith("recipe_") and recipe_id != "custom_color_selection":
        recipe_id = None
    if recipe_id and recipe_id != "custom_color_selection" and not dispenser_ids:
        dispenser_ids = RECIPE_DISPENSERS.get(recipe_id, ())

    if intent == "make_cocktail" and recipe_id is None and not dispenser_ids:
        return _fallback_decision(text, "missing_recipe_or_dispenser")

    valid = intent in {"make_cocktail", "confirm", "cancel"}
    if intent == "make_cocktail" and recipe_id is None:
        recipe_id = "custom_color_selection"

    confirmation = str(payload.get("confirmation", "")).strip()
    if valid and not confirmation:
        if intent == "cancel":
            confirmation = "칵테일 제조 요청을 취소합니다."
        elif intent == "confirm":
            confirmation = "선택한 칵테일 제조를 확인했습니다."
        else:
            color_text = ", ".join(dispenser_ids) if dispenser_ids else "configured recipe dispensers"
            recipe_name = RECIPE_DISPLAY_NAMES.get(str(recipe_id), str(recipe_id))
            confirmation = f"{recipe_name} 요청을 인식했습니다. 사용 디스펜서: {color_text}. 진행할까요?"

    fallback = parse_recipe_command(text)
    return RecipeDecision(
        valid,
        text.strip(),
        fallback.normalized,
        intent,
        recipe_id,
        dispenser_ids,
        confirmation,
        None if valid else "llm returned unknown intent",
    )


class LlmRecipeMapperNode(Node):
    """Map STT text to symbolic recipe decisions with an optional LLM.

    The LLM is constrained to recipe intent and dispenser IDs only. Robot poses,
    trajectories, calibration values, and collision decisions are never accepted
    from the model.
    """

    def __init__(self):
        super().__init__("llm_recipe_mapper_node")
        self.declare_parameter("stt_topic", "/stt_result")
        self.declare_parameter("decision_topic", "/azas/voice/recipe_decision")
        self.declare_parameter("confirmation_topic", "/azas/voice/confirmation")
        self.declare_parameter("enable_llm", False)
        self.declare_parameter("api_key_env", "OPENAI_API_KEY")
        self.declare_parameter("base_url", "https://api.openai.com/v1")
        self.declare_parameter("model", "gpt-4o-mini")
        self.declare_parameter("request_timeout_sec", 8.0)

        self._decision_pub = self.create_publisher(
            String,
            str(self.get_parameter("decision_topic").value),
            10,
        )
        self._confirmation_pub = self.create_publisher(
            String,
            str(self.get_parameter("confirmation_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("stt_topic").value),
            self._on_stt,
            10,
        )
        self.get_logger().info(
            "LLM recipe mapper ready: "
            f"enable_llm={bool(self.get_parameter('enable_llm').value)} "
            f"stt_topic={self.get_parameter('stt_topic').value}"
        )

    def _on_stt(self, msg: String) -> None:
        decision = self._map_text(msg.data)
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

    def _map_text(self, text: str) -> RecipeDecision:
        if not bool(self.get_parameter("enable_llm").value):
            return parse_recipe_command(text)

        api_key = os.environ.get(str(self.get_parameter("api_key_env").value), "").strip()
        if not api_key:
            return _fallback_decision(text, "missing_api_key")

        try:
            payload = self._call_chat_api(text, api_key)
            content = payload["choices"][0]["message"]["content"]
            return _sanitize_llm_decision(text, json.loads(content))
        except Exception as exc:
            self.get_logger().warn(f"LLM mapping failed; using deterministic parser: {exc}")
            return _fallback_decision(text, exc.__class__.__name__)

    def _call_chat_api(self, text: str, api_key: str) -> dict:
        base_url = str(self.get_parameter("base_url").value).rstrip("/")
        body = {
            "model": str(self.get_parameter("model").value),
            "messages": [
                {
                    "role": "system",
                    "content": (
                        "Return only JSON for Azas cocktail intent parsing. "
                        "Allowed fields: valid, intent, recipe_id, dispenser_ids, confirmation. "
                        "Allowed intents: make_cocktail, confirm, cancel, unknown. "
                        "The user does not know dispenser numbers; infer them internally. "
                        "If the user describes mood or asks for a recommendation, choose one recipe_01..recipe_16. "
                        "Allowed dispenser_ids: 1, 2, 3, 4 only. "
                        "Never output robot coordinates, calibration values, trajectories, or safety approvals."
                    ),
                },
                {"role": "user", "content": text},
            ],
            "temperature": 0.0,
        }
        data = json.dumps(body).encode("utf-8")
        req = request.Request(
            f"{base_url}/chat/completions",
            data=data,
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        timeout = float(self.get_parameter("request_timeout_sec").value)
        with request.urlopen(req, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))


def main(args=None):
    if rclpy is None:
        raise RuntimeError("rclpy is required to run llm_recipe_mapper_node")
    rclpy.init(args=args)
    node = LlmRecipeMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
