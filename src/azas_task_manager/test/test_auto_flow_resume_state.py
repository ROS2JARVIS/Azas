from azas_task_manager.auto_flow_resume_state import (
    AutoFlowResumeStore,
    FLOW_STAGES,
    load_resume_snapshot,
    safe_recipe_colors_from_snapshot,
)


def make_store(tmp_path, *, mode="normal", recipe_colors="red:1,blue:1"):
    return AutoFlowResumeStore(
        state_path=tmp_path / "resume.json",
        events_path=tmp_path / "events.jsonl",
        mode=mode,
        recipe_colors=recipe_colors,
        recipe_id="recipe_test",
    )


def test_normal_run_records_stage_progress_and_resume_skips_completed_stages(tmp_path):
    store = make_store(tmp_path)
    assert store.prepare()
    assert store.next_stage() == "color_scan"

    store.start_stage("color_scan")
    store.complete_stage("color_scan", verified={"color_map": True})
    store.start_stage("observe")
    store.complete_stage("observe")

    snapshot = load_resume_snapshot(tmp_path / "resume.json")
    assert snapshot is not None
    assert snapshot["completed_stages"] == ["color_scan", "observe"]
    assert safe_recipe_colors_from_snapshot(snapshot) == "red:1,blue:1"

    resumed = make_store(tmp_path, mode="resume", recipe_colors="")
    assert resumed.prepare()
    assert resumed.should_skip("color_scan")
    assert resumed.should_skip("observe")
    assert not resumed.should_skip("open_gripper")
    assert resumed.next_stage() == "open_gripper"


def test_failed_stage_records_next_stage_and_recovery_instruction(tmp_path):
    store = make_store(tmp_path)
    assert store.prepare()
    store.start_stage("cup_pick")
    store.fail_stage("cup_pick", "side_grasp_joint_state_stale")

    snapshot = load_resume_snapshot(tmp_path / "resume.json")
    assert snapshot is not None
    assert snapshot["status"] == "stopped"
    assert snapshot["stage"] == "cup_pick"
    assert snapshot["next_stage"] == "cup_pick"
    assert snapshot["blocker"] == "side_grasp_joint_state_stale"
    assert snapshot["auto_recoverable"] is True
    assert "이어서" in snapshot["required_user_action"]


def test_progress_updates_verified_facts_without_recording_coordinates(tmp_path):
    store = make_store(tmp_path)
    assert store.prepare()
    store.start_stage("lid_shake")
    store.update_progress(
        "lid_shake",
        "lid_closed",
        verified={"lid_grasped": True, "lid_closed": True},
        held_objects={"cup": "in_holder", "lid": "on_cup"},
    )

    snapshot = load_resume_snapshot(tmp_path / "resume.json")
    assert snapshot is not None
    assert snapshot["step"] == "lid_closed"
    assert snapshot["verified"]["lid_grasped"] is True
    assert snapshot["verified"]["lid_closed"] is True
    assert snapshot["held_objects"] == {"cup": "in_holder", "lid": "on_cup"}
    assert "pose" not in snapshot
    assert snapshot["next_stage"] == "lid_shake"


def test_complete_run_marks_every_stage_completed(tmp_path):
    store = make_store(tmp_path)
    assert store.prepare()
    store.complete_run()

    snapshot = load_resume_snapshot(tmp_path / "resume.json")
    assert snapshot is not None
    assert snapshot["status"] == "completed"
    assert snapshot["stage"] == "complete"
    assert snapshot["next_stage"] == "complete"
    assert tuple(snapshot["completed_stages"]) == FLOW_STAGES
