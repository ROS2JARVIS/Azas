from azas_voice.conversation_manager_node import BUSY_ORDER_MESSAGE, pipeline_status_is_busy


def test_pipeline_status_is_busy_for_active_voice_pipeline_states():
    assert pipeline_status_is_busy({"status": "starting"})
    assert pipeline_status_is_busy({"status": "running"})
    assert pipeline_status_is_busy({"status": "busy"})
    assert not pipeline_status_is_busy({"status": "completed"})
    assert "제조 중" in BUSY_ORDER_MESSAGE
