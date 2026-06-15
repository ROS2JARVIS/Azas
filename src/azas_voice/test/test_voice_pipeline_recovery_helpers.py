from azas_voice.voice_pipeline_executor_node import (
    load_resume_snapshot,
    recipe_colors_from_resume_snapshot,
    stage_from_line,
)


def test_recipe_colors_from_resume_snapshot_uses_stored_recipe_only():
    snapshot = {
        "recipe": {
            "recipe_id": "recipe_05",
            "recipe_colors": "red:2,yellow:1,blue:1",
        },
        "stage": "recipe",
    }

    assert recipe_colors_from_resume_snapshot(snapshot) == "red:2,yellow:1,blue:1"
    assert recipe_colors_from_resume_snapshot({"recipe": {}}) == ""
    assert recipe_colors_from_resume_snapshot(None) == ""


def test_load_resume_snapshot_rejects_missing_and_invalid_files(tmp_path):
    assert load_resume_snapshot(tmp_path / "missing.json") is None

    invalid = tmp_path / "invalid.json"
    invalid.write_text("{not json", encoding="utf-8")
    assert load_resume_snapshot(invalid) is None

    valid = tmp_path / "valid.json"
    valid.write_text('{"status": "stopped"}\n', encoding="utf-8")
    assert load_resume_snapshot(valid) == {"status": "stopped"}


def test_stage_from_line_tracks_lid_shake_and_hand_detection_boundaries():
    assert stage_from_line("starting perception with cup classifier: ros2 launch azas_bringup yolo_perception.launch.py") == "컵 자세 구분"
    assert stage_from_line("waiting for stable route: samples=5, min_sec=0.80, view_hold=3.50s") == "컵 자세 구분"
    assert stage_from_line("[Azas] ArUco lid_grip_close 성공 status 확인 -> 컵홀더 컵 다시 잡기 후 쉐이킹") == "쉐이킹"
    assert stage_from_line("[Azas] SHAKE START: 컵홀더에 놓인 닫힌 컵을 측정 pose로 다시 side-grip 픽업") == "쉐이킹"
    assert stage_from_line("shake succeeded; starting MediaPipe palm handover") == "손 검출 / 핸드오버"
