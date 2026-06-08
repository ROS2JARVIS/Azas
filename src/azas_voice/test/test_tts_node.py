import json

from azas_voice.tts_node import build_ui_state


def test_build_ui_state_preserves_korean_text():
    payload = json.loads(build_ui_state("speaking", "레드 메뉴를 만들까요?", "friendly"))

    assert payload == {
        "state": "speaking",
        "emotion": "friendly",
        "text": "레드 메뉴를 만들까요?",
    }
