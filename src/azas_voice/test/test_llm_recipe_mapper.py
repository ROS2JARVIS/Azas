from azas_voice.llm_recipe_mapper_node import _sanitize_llm_decision


def test_sanitize_llm_decision_accepts_fixed_dispenser_numbers():
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
    assert decision.dispenser_ids == ("2", "4")


def test_sanitize_llm_decision_converts_color_aliases_to_numbers():
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
    assert decision.dispenser_ids == ("1", "3")


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
        "기분에 맞는 칵테일 추천해줘",
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
