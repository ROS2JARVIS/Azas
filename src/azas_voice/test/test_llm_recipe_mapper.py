from azas_voice.llm_recipe_mapper_node import _sanitize_llm_decision
from azas_voice.recipe_catalog import RECIPE_DISPENSERS


def test_sanitize_llm_decision_converts_dispenser_numbers_to_colors():
    decision = _sanitize_llm_decision(
        "2번 4번으로 만들어줘",
        {
            "intent": "make_cocktail",
            "recipe_id": "custom_color_selection",
            "dispenser_ids": ["2", "4"],
            "confirmation": "진행할까요?",
        },
    )

    assert decision.valid
    assert decision.intent == "make_cocktail"
    assert decision.dispenser_ids == ("yellow", "blue")


def test_sanitize_llm_decision_accepts_color_aliases():
    decision = _sanitize_llm_decision(
        "노란색 파란색으로 만들어줘",
        {
            "intent": "make_cocktail",
            "recipe_id": "custom_color_selection",
            "dispenser_ids": ["yellow", "blue"],
            "confirmation": "진행할까요?",
        },
    )

    assert decision.valid
    assert decision.dispenser_ids == ("yellow", "blue")


def test_sanitize_llm_decision_rejects_coordinate_like_output():
    decision = _sanitize_llm_decision(
        "컵 잡아줘",
        {
            "intent": "make_cocktail",
            "recipe_id": "",
            "dispenser_ids": [],
            "x": 0.42,
            "y": -0.1,
        },
    )

    assert not decision.valid
    assert "no recipe or dispenser color matched" in str(decision.error)


def test_sanitize_llm_decision_fills_recipe_dispenser_ids():
    decision = _sanitize_llm_decision(
        "3번 메뉴 만들어줘",
        {
            "intent": "make_cocktail",
            "recipe_id": "recipe_03",
            "dispenser_ids": [],
            "confirmation": "",
        },
    )

    assert decision.valid
    assert decision.recipe_id == "recipe_03"
    assert decision.dispenser_ids
    assert "진행할까요" in decision.confirmation


def test_sanitize_llm_decision_accepts_expanded_catalog_recipe_amounts():
    decision = _sanitize_llm_decision(
        "딥 럼 펀치 만들어줘",
        {
            "intent": "make_cocktail",
            "recipe_id": "recipe_12",
            "dispenser_ids": [],
            "confirmation": "",
        },
    )

    assert decision.valid
    assert decision.recipe_id == "recipe_12"
    assert decision.dispenser_ids == ("red", "yellow", "blue")
    assert decision.dispenser_amounts == {
        "red": 1,
        "yellow": 1,
        "green": 0,
        "blue": 3,
    }


def test_sanitize_llm_decision_preserves_recommendation_wording():
    decision = _sanitize_llm_decision(
        "추천해줘",
        {
            "intent": "make_cocktail",
            "recipe_id": "recipe_01",
            "dispenser_ids": ["red", "yellow"],
            "profile": {"preference_order": "['not too strong', 'light']"},
            "confirmation": "",
        },
    )

    assert decision.valid
    assert decision.recipe_id in RECIPE_DISPENSERS
    assert decision.dispenser_ids == RECIPE_DISPENSERS[decision.recipe_id]
    assert "추천" in decision.confirmation
    assert "진행할까요" in decision.confirmation
    assert decision.profile is None
    if decision.dispenser_amounts is not None:
        assert all(color in {"red", "yellow", "green", "blue"} for color in decision.dispenser_amounts)


def test_sanitize_llm_decision_prefers_local_preference_recommendation():
    decision = _sanitize_llm_decision(
        "너무 세지 않고 향 좋은 걸로 추천해줘",
        {
            "intent": "make_cocktail",
            "recipe_id": "recipe_01",
            "dispenser_ids": ["red"],
            "confirmation": "",
        },
    )

    assert decision.valid
    assert decision.recipe_id == "custom_preference_mix"
    assert decision.dispenser_amounts
    assert decision.dispenser_amounts["blue"] == 1
    assert decision.dispenser_amounts["green"] >= 2


def test_sanitize_llm_decision_prefers_local_mood_recommendation():
    decision = _sanitize_llm_decision(
        "기분 안좋은데 메뉴 추천해줘",
        {
            "intent": "make_cocktail",
            "recipe_id": "recipe_02",
            "dispenser_ids": ["yellow"],
            "confirmation": "",
        },
    )

    assert decision.valid
    assert decision.recipe_id == "custom_preference_mix"
    assert decision.dispenser_amounts == {
        "blue": 1,
        "yellow": 3,
        "green": 1,
        "red": 3,
    }


def test_sanitize_llm_decision_maps_traits_to_amounts():
    decision = _sanitize_llm_decision(
        "쓴맛 나는 술은 싫고 달달한 걸로 추천해줘",
        {
            "intent": "make_cocktail",
            "wanted_traits": ["sweetness"],
            "avoided_traits": ["bitterness", "alcohol"],
            "confirmation": "",
        },
    )

    assert decision.valid
    assert decision.recipe_id == "custom_preference_mix"
    assert decision.dispenser_amounts == {
        "red": 3,
        "yellow": 3,
        "green": 1,
        "blue": 1,
    }
    assert decision.profile
    assert decision.profile["rum"] == "약하게"
    assert decision.profile["syrup"] == "많게"


def test_sanitize_llm_decision_prefers_explicit_stronger_followup():
    decision = _sanitize_llm_decision(
        "더 쎈거는 없어?",
        {
            "intent": "make_cocktail",
            "wanted_traits": ["sweetness", "fruitiness"],
            "avoided_traits": ["bitterness"],
            "confirmation": "",
        },
    )

    assert decision.valid
    assert decision.recipe_id == "custom_preference_mix"
    assert decision.dispenser_amounts
    assert decision.dispenser_amounts["blue"] == 3
    assert decision.profile
    assert decision.profile["rum"] == "강하게"


def test_sanitize_llm_decision_prefers_local_confirm_intent():
    decision = _sanitize_llm_decision(
        "진행해줘",
        {
            "intent": "make_cocktail",
            "recipe_id": "custom_color_selection",
            "dispenser_ids": ["red", "yellow"],
            "confirmation": "custom_color_selection 요청을 인식했습니다. 진행할까요?",
        },
    )

    assert decision.valid
    assert decision.intent == "confirm"
    assert decision.recipe_id is None
    assert decision.dispenser_ids == ()
    assert decision.confirmation == "선택한 칵테일 제조를 확인했습니다."


def test_sanitize_llm_decision_prefers_local_reroll_recommendation():
    decision = _sanitize_llm_decision(
        "아니 다른거",
        {
            "intent": "cancel",
            "recipe_id": None,
            "dispenser_ids": [],
            "confirmation": "칵테일 제조 요청을 취소합니다.",
        },
    )

    assert decision.valid
    assert decision.intent == "make_cocktail"
    assert decision.recipe_id in RECIPE_DISPENSERS
    assert decision.dispenser_ids == RECIPE_DISPENSERS[decision.recipe_id]
    assert "추천" in decision.confirmation


def test_sanitize_llm_decision_accepts_preference_amounts():
    decision = _sanitize_llm_decision(
        "술 약하게 덜 달고 상큼하게",
        {
            "intent": "make_cocktail",
            "recipe_id": "custom_preference_mix",
            "dispenser_amounts": {
                "red": 1,
                "yellow": 1,
                "green": 3,
                "blue": 1,
            },
            "profile": {
                "rum": "약하게",
                "syrup": "적게",
                "liqueur": "많게",
                "juice": "적게",
            },
            "confirmation": "취향에 맞춰 제조할까요?",
        },
    )

    assert decision.valid
    assert decision.recipe_id == "custom_preference_mix"
    assert decision.dispenser_ids == ("red", "yellow", "green", "blue")
    assert decision.dispenser_amounts
    assert decision.dispenser_amounts["green"] == 3
    assert decision.profile
    assert decision.profile["syrup"] == "적게"


def test_sanitize_llm_decision_repairs_incomplete_preference_mix():
    decision = _sanitize_llm_decision(
        "시럽 적게 리큐르 많이 럼 약하게 주스 많이 넣어줘",
        {
            "intent": "make_cocktail",
            "recipe_id": "custom_preference_mix",
            "dispenser_ids": ["yellow", "green", "blue"],
            "confirmation": "False",
        },
    )

    assert decision.valid
    assert decision.recipe_id == "custom_preference_mix"
    assert decision.dispenser_ids == ("red", "yellow", "green", "blue")
    assert decision.dispenser_amounts
    assert decision.dispenser_amounts["blue"] == 1
    assert decision.dispenser_amounts["yellow"] == 1
    assert decision.dispenser_amounts["green"] == 3
    assert decision.dispenser_amounts["red"] >= 2
    assert decision.profile
    assert decision.profile["rum"] == "약하게"
    assert "진행할까요" in decision.confirmation


def test_sanitize_llm_decision_repairs_nonstandard_preference_profile():
    decision = _sanitize_llm_decision(
        "오늘은 너무 세지 않고 향은 좀 풍부한 느낌으로 만들어줘",
        {
            "intent": "make_cocktail",
            "recipe_id": "custom_preference_mix",
            "dispenser_amounts": {
                "blue": 1,
                "yellow": 2,
                "green": 3,
                "red": 1,
            },
            "profile": {"preference_order": "['not too strong', 'rich aroma']"},
            "confirmation": "취향대로 맞출게요. 진행할까요?",
        },
    )

    assert decision.valid
    assert decision.recipe_id == "custom_preference_mix"
    assert decision.profile == {
        "rum": "약하게",
        "syrup": "보통",
        "liqueur": "많게",
        "juice": "적게",
    }
