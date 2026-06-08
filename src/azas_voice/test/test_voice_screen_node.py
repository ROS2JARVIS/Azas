from azas_voice.voice_screen_node import _json_or_text, build_initial_state


def test_voice_screen_initial_state_has_dialogue_fields():
    state = build_initial_state()

    assert state["last_stt"] == ""
    assert state["last_confirmation"] == ""
    assert state["ui_state"]["state"] == "idle"
    assert state["events"] == []


def test_json_or_text_parses_decision_payload():
    payload = _json_or_text('{"intent": "make_cocktail", "recipe_id": "recipe_01"}')

    assert payload == {"intent": "make_cocktail", "recipe_id": "recipe_01"}


def test_json_or_text_wraps_plain_text():
    payload = _json_or_text("진행할까요?")

    assert payload == {"text": "진행할까요?"}
