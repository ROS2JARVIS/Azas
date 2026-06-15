from types import SimpleNamespace

import pytest

from azas_voice.voice_screen_node import (
    _image_msg_to_bgr,
    _json_or_text,
    _parse_detection_overlay,
    build_initial_state,
)


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


def test_parse_cup_detection_overlay_extracts_orientation_center_and_bbox():
    payload = _parse_detection_overlay(
        "detected:upright class=tumbler bbox=120x220 orientation=upright center=(321,240)",
        kind="cup",
    )

    assert payload["detected"] is True
    assert payload["orientation"] == "upright"
    assert payload["center"] == (321, 240)
    assert payload["bbox"] == (120, 220)


def test_parse_lid_detection_overlay_prefers_lid_center():
    payload = _parse_detection_overlay(
        "detected:lid class=lid bbox=80x64 lid_center=(410,220) aruco_center=(398,218)",
        kind="lid",
    )

    assert payload["detected"] is True
    assert payload["center"] == (410, 220)
    assert payload["bbox"] == (80, 64)


def test_image_msg_to_bgr_converts_rgb8_with_row_step():
    pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")

    msg = SimpleNamespace(
        height=1,
        width=2,
        encoding="rgb8",
        step=8,
        data=bytes([255, 0, 0, 0, 255, 0, 99, 99]),
    )

    frame = _image_msg_to_bgr(msg)

    assert frame.shape == (1, 2, 3)
    np.testing.assert_array_equal(frame[0, 0], np.array([0, 0, 255], dtype=np.uint8))
    np.testing.assert_array_equal(frame[0, 1], np.array([0, 255, 0], dtype=np.uint8))
