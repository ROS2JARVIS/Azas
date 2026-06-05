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

from azas_voice.command_parser import RecipeDecision, amounts_from_traits, parse_recipe_command, profile_from_amounts
from azas_voice.recipe_catalog import (
    COLOR_ALIASES,
    DISPENSER_TRAITS,
    RECIPE_DESCRIPTIONS,
    RECIPE_DISPENSERS,
    RECIPE_DISPLAY_NAMES,
)


ALLOWED_INTENTS = {"make_cocktail", "confirm", "cancel", "unknown"}
ALLOWED_CUSTOM_RECIPE_IDS = {"custom_color_selection", "custom_preference_mix"}
DISPENSER_NUMBER_TO_COLOR = {
    "1": "red",
    "2": "yellow",
    "3": "green",
    "4": "blue",
}
ALLOWED_TRAITS = set().union(*DISPENSER_TRAITS.values())


def _normalize_dispenser_id(value: object) -> str:
    raw = str(value).strip()
    if raw in DISPENSER_NUMBER_TO_COLOR:
        return DISPENSER_NUMBER_TO_COLOR[raw]
    normalized = "".join(raw.lower().split())
    for dispenser_id, aliases in COLOR_ALIASES.items():
        if dispenser_id == normalized:
            return dispenser_id
        if any("".join(alias.lower().split()) == normalized for alias in aliases):
            return dispenser_id
    return ""


def _normalize_traits(value: object) -> tuple[str, ...]:
    if not isinstance(value, list):
        return ()
    traits: list[str] = []
    for item in value:
        trait = str(item).strip().lower()
        if trait in ALLOWED_TRAITS and trait not in traits:
            traits.append(trait)
    return tuple(traits)


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

    fallback = parse_recipe_command(text)
    if fallback.valid and fallback.intent in {"confirm", "cancel"}:
        return fallback
    if fallback.valid and fallback.intent == "make_cocktail" and fallback.recipe_id in RECIPE_DISPENSERS and "추천" in fallback.confirmation:
        return fallback

    dispenser_ids = tuple(
        dispenser_id
        for dispenser_id in (_normalize_dispenser_id(item) for item in payload.get("dispenser_ids", []))
        if dispenser_id
    )
    amounts_payload = payload.get("dispenser_amounts", {})
    dispenser_amounts: dict[str, int] = {}
    if isinstance(amounts_payload, dict):
        for color in ("red", "yellow", "green", "blue"):
            try:
                amount = int(amounts_payload.get(color, 0))
            except (TypeError, ValueError):
                amount = 0
            dispenser_amounts[color] = max(0, min(amount, 3))

    wanted_traits = _normalize_traits(payload.get("wanted_traits", []))
    avoided_traits = _normalize_traits(payload.get("avoided_traits", []))
    if intent == "make_cocktail" and (wanted_traits or avoided_traits):
        dispenser_amounts = amounts_from_traits(wanted_traits, avoided_traits, fallback.normalized)
        dispenser_ids = tuple(color for color in ("red", "yellow", "green", "blue") if dispenser_amounts[color] > 0)

    if not dispenser_ids and any(dispenser_amounts.values()):
        dispenser_ids = tuple(color for color in ("red", "yellow", "green", "blue") if dispenser_amounts[color] > 0)

    recipe_id = payload.get("recipe_id")
    recipe_id = str(recipe_id).strip() if recipe_id else None
    if intent == "make_cocktail" and (wanted_traits or avoided_traits):
        recipe_id = "custom_preference_mix"
    if recipe_id and not recipe_id.startswith("recipe_") and recipe_id not in ALLOWED_CUSTOM_RECIPE_IDS:
        recipe_id = None
    if recipe_id and recipe_id not in ALLOWED_CUSTOM_RECIPE_IDS:
        dispenser_ids = RECIPE_DISPENSERS.get(recipe_id, ())

    if intent == "make_cocktail" and recipe_id is None and not dispenser_ids:
        return _fallback_decision(text, "missing_recipe_or_dispenser")

    valid = intent in {"make_cocktail", "confirm", "cancel"}
    if intent == "make_cocktail" and recipe_id is None:
        recipe_id = "custom_preference_mix" if dispenser_amounts else "custom_color_selection"

    if intent == "make_cocktail" and fallback.recipe_id == "custom_preference_mix":
        recipe_id = "custom_preference_mix"
        if not any(dispenser_amounts.values()) and fallback.dispenser_amounts:
            dispenser_amounts = dict(fallback.dispenser_amounts)
        if any(dispenser_amounts.values()):
            dispenser_ids = tuple(
                color for color in ("red", "yellow", "green", "blue") if dispenser_amounts[color] > 0
            )
        elif not dispenser_ids and fallback.dispenser_ids:
            dispenser_ids = fallback.dispenser_ids

    confirmation = str(payload.get("confirmation", "")).strip()
    if confirmation.lower() in {"false", "none", "null"}:
        confirmation = ""
    if valid and not confirmation:
        if intent == "cancel":
            confirmation = "칵테일 제조 요청을 취소합니다."
        elif intent == "confirm":
            confirmation = "선택한 칵테일 제조를 확인했습니다."
        elif recipe_id == "custom_preference_mix" and fallback.confirmation:
            confirmation = fallback.confirmation
        elif fallback.confirmation and "추천" in fallback.confirmation:
            recipe_name = RECIPE_DISPLAY_NAMES.get(str(recipe_id), str(recipe_id))
            description = RECIPE_DESCRIPTIONS.get(str(recipe_id), "")
            confirmation = f"{recipe_name}를 추천드릴게요. {description} 진행할까요?"
        else:
            recipe_name = RECIPE_DISPLAY_NAMES.get(str(recipe_id), str(recipe_id))
            confirmation = f"{recipe_name} 요청을 인식했습니다. 진행할까요?"

    profile = payload.get("profile")
    if recipe_id == "custom_preference_mix" and any(dispenser_amounts.values()):
        profile = profile_from_amounts(dispenser_amounts)
    if recipe_id != "custom_preference_mix":
        profile = None
    if not isinstance(profile, dict):
        profile = fallback.profile if recipe_id == "custom_preference_mix" else None
    if recipe_id == "custom_preference_mix" and fallback.profile:
        expected_profile_keys = {"rum", "syrup", "liqueur", "juice"}
        if not profile or set(profile.keys()) != expected_profile_keys:
            profile = fallback.profile
    return RecipeDecision(
        valid,
        text.strip(),
        fallback.normalized,
        intent,
        recipe_id,
        dispenser_ids,
        confirmation,
        None if valid else "llm returned unknown intent",
        profile={str(k): str(v) for k, v in profile.items()} if profile else None,
        dispenser_amounts=dispenser_amounts if any(dispenser_amounts.values()) else None,
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
        self.declare_parameter("publish_confirmation", True)

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
        self._publish_confirmation = bool(self.get_parameter("publish_confirmation").value)
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

        if self._publish_confirmation and decision.confirmation:
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
                        "Allowed fields: valid, intent, recipe_id, dispenser_ids, confirmation, wanted_traits, avoided_traits. "
                        "Allowed intents: make_cocktail, confirm, cancel, unknown. "
                        "For descriptive preference or recommendation requests, extract wanted_traits and avoided_traits instead of calculating amounts. "
                        "Allowed traits: sweetness, fruitiness, freshness, aroma, alcohol, bitterness, softness, light, depth, herbal. "
                        "Examples of preferences: not too strong, light, easy to drink, rich aroma, sweet, not sweet, fruity. "
                        "For a plain recommendation with no preferences, choose one recipe_01..recipe_04. "
                        "Only choose recipe_01..recipe_04 when the user explicitly asks for a numbered/color menu or gives no preferences. "
                        "Allowed dispenser_ids values: red, yellow, green, blue only. "
                        "Do not output dispenser_amounts; the application calculates amounts from traits. "
                        "Never output robot coordinates, calibration values, trajectories, or safety approvals."
                    ),
                },
                {"role": "user", "content": text},
            ],
            "temperature": 0.0,
            "response_format": {"type": "json_object"},
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
