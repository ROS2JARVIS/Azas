from __future__ import annotations

from dataclasses import dataclass
import random

from azas_voice.recipe_catalog import (
    AVOID_TRAIT_KEYWORDS,
    CANCEL_WORDS,
    COLOR_ALIASES,
    CONFIRM_WORDS,
    DISPENSER_TRAITS,
    MOOD_WORDS,
    PREFERENCE_WORDS,
    RANDOM_RECIPE_WORDS,
    REROLL_RECOMMENDATION_WORDS,
    RECIPE_ALIASES,
    RECIPE_DESCRIPTIONS,
    RECIPE_DISPENSERS,
    RECIPE_DISPLAY_NAMES,
    TRAIT_KEYWORDS,
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
    profile: dict[str, str] | None = None
    dispenser_amounts: dict[str, int] | None = None

    def to_dict(self) -> dict[str, object]:
        payload: dict[str, object] = {
            "valid": self.valid,
            "utterance": self.utterance,
            "normalized": self.normalized,
            "intent": self.intent,
            "recipe_id": self.recipe_id,
            "dispenser_ids": list(self.dispenser_ids),
            "confirmation": self.confirmation,
            "error": self.error,
        }
        if self.profile is not None:
            payload["profile"] = self.profile
        if self.dispenser_amounts is not None:
            payload["dispenser_amounts"] = self.dispenser_amounts
        return payload


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
        f"{description} 진행할까요?"
    )
    return RecipeDecision(True, utterance, normalized, "make_cocktail", recipe_id, dispenser_ids, confirmation)


def _level_text(amount: int, zero: str, low: str, normal: str, high: str) -> str:
    if amount <= 0:
        return zero
    if amount == 1:
        return low
    if amount == 2:
        return normal
    return high


def _extract_traits(normalized: str) -> tuple[tuple[str, ...], tuple[str, ...]]:
    wanted = {
        trait
        for trait, keywords in TRAIT_KEYWORDS.items()
        if _contains_any(normalized, keywords)
    }
    avoided = {
        trait
        for trait, keywords in AVOID_TRAIT_KEYWORDS.items()
        if _contains_any(normalized, keywords)
    }
    wanted -= avoided
    if "bitterness" in avoided:
        wanted.update({"sweetness", "fruitiness"})
        wanted -= avoided
    return tuple(sorted(wanted)), tuple(sorted(avoided))


def _amount_from_score(score: float) -> int:
    if score <= 0.0:
        return 0
    if score <= 1.0:
        return 1
    if score <= 2.0:
        return 2
    return 3


def amounts_from_traits(
    wanted_traits: tuple[str, ...],
    avoided_traits: tuple[str, ...],
    normalized: str = "",
) -> dict[str, int]:
    wanted = set(wanted_traits)
    avoided = set(avoided_traits)
    scores = {color: 1.0 for color in ("red", "yellow", "green", "blue")}

    for color, traits in DISPENSER_TRAITS.items():
        trait_set = set(traits)
        scores[color] += 1.25 * len(wanted & trait_set)
        scores[color] -= 0.75 * len(avoided & trait_set)

    amounts = {
        color: max(1, _amount_from_score(score))
        for color, score in scores.items()
    }

    if _contains_any(normalized, ("무알콜", "논알콜", "알코올없이", "술없이", "럼없이")):
        amounts["blue"] = 0

    return amounts


def profile_from_amounts(amounts: dict[str, int]) -> dict[str, str]:
    return {
        "rum": _level_text(amounts["blue"], "없음", "약하게", "보통", "강하게"),
        "syrup": _level_text(amounts["yellow"], "없음", "적게", "보통", "많게"),
        "liqueur": _level_text(amounts["green"], "없음", "적게", "보통", "많게"),
        "juice": _level_text(amounts["red"], "없음", "적게", "보통", "많게"),
    }


def _custom_preference_decision(utterance: str, normalized: str) -> RecipeDecision:
    wanted_traits, avoided_traits = _extract_traits(normalized)
    amounts = amounts_from_traits(wanted_traits, avoided_traits, normalized)

    dispenser_ids = tuple(color for color in ("red", "yellow", "green", "blue") if amounts[color] > 0)
    profile = profile_from_amounts(amounts)
    summary = (
        f"말씀하신 취향에는 럼 {profile['rum']}, 시럽 {profile['syrup']}, "
        f"리큐르 {profile['liqueur']}, 주스 {profile['juice']} 조합을 추천드릴게요. 진행할까요?"
    )
    return RecipeDecision(
        True,
        utterance,
        normalized,
        "make_cocktail",
        "custom_preference_mix",
        dispenser_ids,
        summary,
        profile=profile,
        dispenser_amounts=amounts,
    )


def parse_recipe_command(text: str) -> RecipeDecision:
    utterance = text.strip()
    normalized = normalize_text(utterance)

    if not normalized:
        return RecipeDecision(False, utterance, normalized, "unknown", None, (), "", "empty utterance")

    if _contains_any(normalized, REROLL_RECOMMENDATION_WORDS):
        return _random_recipe_decision(utterance, normalized)

    if _contains_any(normalized, CANCEL_WORDS):
        return RecipeDecision(True, utterance, normalized, "cancel", None, (), "칵테일 제조 요청을 취소합니다.")

    recipe_id = _match_recipe(normalized)
    dispenser_ids = _match_colors(normalized)

    if recipe_id is None and not dispenser_ids and _is_random_recipe_request(normalized):
        if _contains_any(normalized, PREFERENCE_WORDS):
            return _custom_preference_decision(utterance, normalized)
        return _random_recipe_decision(utterance, normalized)

    if recipe_id is None and not dispenser_ids and _contains_any(normalized, PREFERENCE_WORDS):
        return _custom_preference_decision(utterance, normalized)

    if recipe_id is None and not dispenser_ids and _contains_any(normalized, CONFIRM_WORDS):
        return RecipeDecision(True, utterance, normalized, "confirm", None, (), "선택한 칵테일 제조를 확인했습니다.")

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
