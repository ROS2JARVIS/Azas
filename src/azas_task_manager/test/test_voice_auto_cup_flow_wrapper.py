from pathlib import Path


def test_voice_auto_cup_flow_uses_meter_offset_for_three_millimeters():
    repo_root = Path(__file__).resolve().parents[3]
    script = repo_root / "tools" / "run" / "run_voice_auto_cup_flow.sh"
    text = script.read_text(encoding="utf-8")

    assert 'CUP_HOLDER_PLACE_FINAL_X_OFFSET_M="${CUP_HOLDER_PLACE_FINAL_X_OFFSET_M:-0.003}"' in text
    assert "cup_holder_place_x_offset_m:=3.0" not in text
    assert 'cup_holder_place_x_offset_m:="${CUP_HOLDER_PLACE_FINAL_X_OFFSET_M}"' in text


def test_voice_auto_cup_flow_blocks_meter_scale_holder_offsets():
    repo_root = Path(__file__).resolve().parents[3]
    script = repo_root / "tools" / "run" / "run_voice_auto_cup_flow.sh"
    text = script.read_text(encoding="utf-8")

    assert "abs(offset_m) > 0.05" in text
    assert "must be in meters" in text


def test_voice_auto_cup_flow_passes_dsr01_motion_namespace_by_default():
    repo_root = Path(__file__).resolve().parents[3]
    script = repo_root / "tools" / "run" / "run_voice_auto_cup_flow.sh"
    text = script.read_text(encoding="utf-8")

    assert 'SERVICE_PREFIX="${SERVICE_PREFIX:-dsr01}"' in text
    assert 'MOTION_SERVICE_PREFIX="${MOTION_SERVICE_PREFIX:-${SERVICE_PREFIX}}"' in text
    assert 'service_prefix:="${SERVICE_PREFIX}"' in text
    assert 'motion_service_prefix:="${MOTION_SERVICE_PREFIX}"' in text
    assert "moveit_controller_name:=/${SERVICE_PREFIX}/dsr_moveit_controller" in text
    assert (
        "controller_action_name:=/${SERVICE_PREFIX}/dsr_moveit_controller/follow_joint_trajectory"
        in text
    )


def test_auto_cup_flow_router_launch_defaults_to_dsr01_motion_namespace():
    repo_root = Path(__file__).resolve().parents[3]
    launch = repo_root / "src" / "azas_bringup" / "launch" / "auto_cup_flow_router.launch.py"
    text = launch.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("service_prefix", default_value="dsr01")' in text
    assert 'DeclareLaunchArgument("motion_service_prefix", default_value="dsr01")' in text
    assert (
        'DeclareLaunchArgument("moveit_controller_name", default_value="/dsr01/dsr_moveit_controller")'
        in text
    )
    assert 'default_value="/dsr01/dsr_moveit_controller/follow_joint_trajectory"' in text


def test_auto_cup_flow_router_node_defaults_to_dsr01_motion_namespace():
    repo_root = Path(__file__).resolve().parents[3]
    router = (
        repo_root
        / "src"
        / "azas_task_manager"
        / "azas_task_manager"
        / "auto_cup_flow_router.py"
    )
    text = router.read_text(encoding="utf-8")

    assert 'self.declare_parameter("service_prefix", "dsr01")' in text
    assert 'self.declare_parameter("motion_service_prefix", "dsr01")' in text
    assert 'self.declare_parameter("moveit_controller_name", "/dsr01/dsr_moveit_controller")' in text
    assert '"/dsr01/dsr_moveit_controller/follow_joint_trajectory"' in text
    assert 'base = f"/{prefix}/motion" if prefix else "/motion"' in text


def test_human_handover_detection_default_command_matches_current_cli():
    repo_root = Path(__file__).resolve().parents[3]
    launch = repo_root / "src" / "azas_bringup" / "launch" / "auto_cup_flow_router.launch.py"
    router = (
        repo_root
        / "src"
        / "azas_task_manager"
        / "azas_task_manager"
        / "auto_cup_flow_router.py"
    )
    text = launch.read_text(encoding="utf-8") + "\n" + router.read_text(encoding="utf-8")

    assert "--process-width-px" not in text
    assert "--overlay-width-px" not in text
    assert "--max-rate-hz 20" in text
    assert "--stable-window-seconds 1.0" in text


def test_auto_cup_flow_router_does_not_use_rcutils_logger_exception():
    repo_root = Path(__file__).resolve().parents[3]
    router = (
        repo_root
        / "src"
        / "azas_task_manager"
        / "azas_task_manager"
        / "auto_cup_flow_router.py"
    )
    text = router.read_text(encoding="utf-8")

    assert "traceback.format_exc()" in text
    assert "get_logger().exception" not in text
