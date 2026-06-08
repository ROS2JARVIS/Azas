from azas_voice.command_parser import normalize_text, parse_recipe_command


def assert_dispenser_colors(dispenser_ids):
    assert dispenser_ids
    assert all(item in {"red", "yellow", "green", "blue"} for item in dispenser_ids)


def test_normalize_text_removes_spaces_and_lowercases():
    assert normalize_text(" Recipe 1 ") == "recipe1"


def test_color_selection_maps_to_dispenser_colors():
    decision = parse_recipe_command("노란색 파란색으로 만들어줘")
    assert decision.valid
    assert decision.intent == "make_cocktail"
    assert decision.recipe_id == "custom_color_selection"
    assert decision.dispenser_ids == ("yellow", "blue")


def test_number_selection_maps_to_dispenser_colors():
    decision = parse_recipe_command("디스펜서 2번 4번으로 만들어줘")
    assert decision.valid
    assert decision.intent == "make_cocktail"
    assert decision.recipe_id == "custom_color_selection"
    assert decision.dispenser_ids == ("yellow", "blue")


def test_recipe_alias_maps_to_recipe_id():
    decision = parse_recipe_command("3번 칵테일 만들어줘")
    assert decision.valid
    assert decision.recipe_id == "recipe_03"
    assert_dispenser_colors(decision.dispenser_ids)


def test_four_menu_recipes_map_to_one_dispenser_each():
    expected = {
        "1번 메뉴 만들어줘": ("recipe_01", ("red",)),
        "2번 메뉴 만들어줘": ("recipe_02", ("yellow",)),
        "3번 메뉴 만들어줘": ("recipe_03", ("green",)),
        "4번 메뉴 만들어줘": ("recipe_04", ("blue",)),
        "파란색 메뉴 만들어줘": ("recipe_04", ("blue",)),
    }

    for utterance, (recipe_id, dispenser_ids) in expected.items():
        decision = parse_recipe_command(utterance)
        assert decision.valid
        assert decision.recipe_id == recipe_id
        assert decision.dispenser_ids == dispenser_ids


def test_mood_request_maps_to_custom_recommendation():
    decision = parse_recipe_command("오늘 기분이 우울한데 칵테일 추천해줘")
    assert decision.valid
    assert decision.intent == "make_cocktail"
    assert decision.recipe_id == "custom_preference_mix"
    assert_dispenser_colors(decision.dispenser_ids)
    assert decision.dispenser_amounts == {
        "blue": 1,
        "yellow": 3,
        "green": 1,
        "red": 3,
    }
    assert "추천" in decision.confirmation
    assert "진행할까요" in decision.confirmation


def test_reroll_recommendation_words_take_priority_over_cancel():
    for utterance in ("아니 다른거", "다른거", "말고 다른 메뉴 추천해줘"):
        decision = parse_recipe_command(utterance)
        assert decision.valid
        assert decision.intent == "make_cocktail"
        assert decision.recipe_id is not None
        assert decision.recipe_id.startswith("recipe_")
        assert_dispenser_colors(decision.dispenser_ids)
        assert "추천" in decision.confirmation


def test_preference_recommendation_maps_to_custom_mix():
    decision = parse_recipe_command("너무 세지 않고 향 좋은 걸로 추천해줘")
    assert decision.valid
    assert decision.intent == "make_cocktail"
    assert decision.recipe_id == "custom_preference_mix"
    assert decision.dispenser_amounts
    assert decision.dispenser_amounts["blue"] == 1
    assert decision.dispenser_amounts["green"] >= 2
    assert "추천" in decision.confirmation


def test_mood_recommendations_map_to_custom_mix():
    sad_decision = parse_recipe_command("기분 안좋은데 메뉴 추천해줘")
    assert sad_decision.valid
    assert sad_decision.recipe_id == "custom_preference_mix"
    assert sad_decision.dispenser_amounts
    assert sad_decision.dispenser_amounts["blue"] == 1
    assert sad_decision.dispenser_amounts["yellow"] == 3
    assert sad_decision.dispenser_amounts["red"] == 3

    happy_decision = parse_recipe_command("기분 좋은데 메뉴 추천해줘")
    assert happy_decision.valid
    assert happy_decision.recipe_id == "custom_preference_mix"
    assert happy_decision.dispenser_amounts
    assert happy_decision.dispenser_amounts["green"] == 3
    assert happy_decision.dispenser_amounts["red"] == 3


def test_bitter_alcohol_dislike_maps_to_sweeter_custom_mix():
    decision = parse_recipe_command("쓴맛 나는 술은 싫은데 메뉴 추천해줘")
    assert decision.valid
    assert decision.recipe_id == "custom_preference_mix"
    assert decision.dispenser_amounts
    assert decision.dispenser_amounts["blue"] == 1
    assert decision.dispenser_amounts["yellow"] == 3
    assert decision.dispenser_amounts["red"] == 3


def test_stronger_followup_maps_to_high_alcohol_custom_mix():
    decision = parse_recipe_command("더 쎈거는 없어?")
    assert decision.valid
    assert decision.recipe_id == "custom_preference_mix"
    assert decision.dispenser_amounts
    assert decision.dispenser_amounts["blue"] == 3
    assert decision.profile
    assert decision.profile["rum"] == "강하게"


def test_preference_request_maps_to_ingredient_amounts():
    decision = parse_recipe_command("술 약하게 하고 덜 달고 상큼하게 과일맛 진하게 만들어줘")
    assert decision.valid
    assert decision.intent == "make_cocktail"
    assert decision.recipe_id == "custom_preference_mix"
    assert decision.dispenser_amounts
    assert decision.dispenser_amounts["red"] == 3
    assert decision.dispenser_amounts["yellow"] == 1
    assert decision.dispenser_amounts["blue"] == 1
    assert decision.profile == {
        "rum": "약하게",
        "syrup": "적게",
        "liqueur": "적게",
        "juice": "많게",
    }
    assert decision.dispenser_ids == ("red", "yellow", "green", "blue")


def test_non_alcohol_preference_omits_blue_dispenser():
    decision = parse_recipe_command("무알콜로 달달하고 과일맛 나게")
    assert decision.valid
    assert decision.recipe_id == "custom_preference_mix"
    assert decision.dispenser_amounts
    assert decision.dispenser_amounts["blue"] == 0
    assert "blue" not in decision.dispenser_ids


def test_unknown_text_is_invalid():
    decision = parse_recipe_command("무슨 말인지 모르겠어")
    assert not decision.valid
    assert decision.error == "no recipe or dispenser color matched"


def test_cancel_intent():
    decision = parse_recipe_command("취소해줘")
    assert decision.valid
    assert decision.intent == "cancel"


def test_proceed_phrase_maps_to_confirm_intent():
    decision = parse_recipe_command("진행해줘")
    assert decision.valid
    assert decision.intent == "confirm"


def test_common_acknowledgements_map_to_confirm_intent():
    for utterance in (
        "알겠어",
        "알겠습니다",
        "오케이",
        "그래 그렇게 해줘",
        "좋아요",
        "괜찮아",
        "콜",
        "가자",
        "계속해줘",
    ):
        decision = parse_recipe_command(utterance)
        assert decision.valid
        assert decision.intent == "confirm"


def test_order_phrase_with_make_word_still_maps_to_preference_order():
    decision = parse_recipe_command("술 약하게 해서 만들어줘")
    assert decision.valid
    assert decision.intent == "make_cocktail"
    assert decision.recipe_id == "custom_preference_mix"
