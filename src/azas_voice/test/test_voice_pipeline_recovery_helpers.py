from azas_voice.voice_pipeline_executor_node import (
    load_resume_snapshot,
    recipe_colors_from_resume_snapshot,
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
