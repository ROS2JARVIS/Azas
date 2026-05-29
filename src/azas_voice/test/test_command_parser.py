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


def test_mood_request_randomly_recommends_executable_recipe():
    decision = parse_recipe_command("오늘 기분이 우울한데 칵테일 추천해줘")
    assert decision.valid
    assert decision.intent == "make_cocktail"
    assert decision.recipe_id is not None
    assert decision.recipe_id.startswith("recipe_")
    assert_dispenser_colors(decision.dispenser_ids)
    assert "추천" in decision.confirmation


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
