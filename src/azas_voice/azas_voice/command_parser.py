from __future__ import annotations

from dataclasses import dataclass
import random

from azas_voice.recipe_catalog import (
    CANCEL_WORDS,
    COLOR_ALIASES,
    CONFIRM_WORDS,
    MOOD_WORDS,
    RANDOM_RECIPE_WORDS,
    RECIPE_ALIASES,
    RECIPE_DESCRIPTIONS,
    RECIPE_DISPENSERS,
    RECIPE_DISPLAY_NAMES,
)


@dataclass(frozen=True)
class RecipeDecision:
    valid: bool
    utterance: str
    normalized: str
    intent: str
    recipe_id: str | None
    dispenser_ids: tuple[str, ...]
    confirmation: str
    error: str | None = None
    # extra: LLM이 추가 필드(profile, dispenser_amounts 등)를 리턴할 때 pass-through
    extra: dict | None = None

    def to_dict(self) -> dict[str, object]:
        d = {
            "valid": self.valid,
            "utterance": self.utterance,
            "normalized": self.normalized,
            "intent": self.intent,
            "recipe_id": self.recipe_id,
            "dispenser_ids": list(self.dispenser_ids),
            "confirmation": self.confirmation,
            "error": self.error,
        }
        if self.extra:
            d.update(self.extra)
        return d


def normalize_text(text: str) -> str:
    return "".join(text.lower().split())


def _contains_any(normalized: str, words: tuple[str, ...]) -> bool:
    return any(normalize_text(word) in normalized for word in words)


def _match_recipe(normalized: str) -> str | None:
    if "디스펜서" in normalized:
        return None
    for recipe_id, aliases in RECIPE_ALIASES.items():
        if _contains_any(normalized, aliases):
            return recipe_id
    return None


def _match_colors(normalized: str) -> tuple[str, ...]:
    matched: list[str] = []
    for dispenser_id, aliases in COLOR_ALIASES.items():
        if _contains_any(normalized, aliases):
            matched.append(dispenser_id)
    return tuple(matched)


def _is_random_recipe_request(normalized: str) -> bool:
    has_mood = _contains_any(normalized, MOOD_WORDS)
    has_random = _contains_any(normalized, RANDOM_RECIPE_WORDS)
    return has_mood or has_random


def _recipe_name(recipe_id: str) -> str:
    return RECIPE_DISPLAY_NAMES.get(recipe_id, recipe_id)


def _recipe_description(recipe_id: str) -> str:
    return RECIPE_DESCRIPTIONS.get(recipe_id, "")


def _random_recipe_decision(utterance: str, normalized: str) -> RecipeDecision:
    recipe_id = random.choice(tuple(RECIPE_DISPENSERS))
    dispenser_ids = RECIPE_DISPENSERS[recipe_id]
    description = _recipe_description(recipe_id)
    confirmation = (
        f"{_recipe_name(recipe_id)}를 추천드릴게요. "
        f"{description} "
        f"진행할까요?"
    )
    return RecipeDecision(True, utterance, normalized, "make_cocktail", recipe_id, dispenser_ids, confirmation)


def parse_recipe_command(text: str) -> RecipeDecision:
    utterance = text.strip()
    normalized = normalize_text(utterance)

    if not normalized:
        return RecipeDecision(False, utterance, normalized, "unknown", None, (), "", "empty utterance")

    if _contains_any(normalized, CANCEL_WORDS):
        return RecipeDecision(True, utterance, normalized, "cancel", None, (), "칵테일 제조 요청을 취소합니다.")

    if _contains_any(normalized, CONFIRM_WORDS):
        return RecipeDecision(True, utterance, normalized, "confirm", None, (), "선택한 칵테일 제조를 확인했습니다.")

    recipe_id = _match_recipe(normalized)
    dispenser_ids = _match_colors(normalized)

    if recipe_id is None and not dispenser_ids and _is_random_recipe_request(normalized):
        return _random_recipe_decision(utterance, normalized)

    if recipe_id is None and not dispenser_ids:
        return RecipeDecision(
            False,
            utterance,
            normalized,
            "unknown",
            None,
            (),
            "",
            "no recipe or dispenser color matched",
        )

    if recipe_id is None:
        recipe_id = "custom_color_selection"
    elif not dispenser_ids:
        dispenser_ids = RECIPE_DISPENSERS.get(recipe_id, ())

    dispenser_text = ", ".join(dispenser_ids) if dispenser_ids else "configured recipe dispensers"
    confirmation = f"{_recipe_name(recipe_id)} 요청을 인식했습니다. 진행할까요?"
    return RecipeDecision(True, utterance, normalized, "make_cocktail", recipe_id, dispenser_ids, confirmation)
