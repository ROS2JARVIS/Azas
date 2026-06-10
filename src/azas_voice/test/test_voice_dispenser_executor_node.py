from azas_voice.voice_dispenser_executor_node import (
    build_dispenser_launch_command,
    requests_from_decision,
)


def test_requests_from_confirmed_decision_repeats_amounts():
    decision = {
        "confirmed": True,
        "intent": "make_cocktail",
        "recipe_id": "custom_preference_mix",
        "dispenser_ids": ["red", "yellow", "blue"],
        "dispenser_amounts": {"red": 2, "yellow": 0, "blue": 3},
    }

    requests = requests_from_decision(decision)

    assert [request.target_dispenser for request in requests] == [
        "red",
        "red",
        "blue",
        "blue",
        "blue",
    ]
    assert requests[0].repeat_index == 1
    assert requests[1].repeat_index == 2
    assert requests[1].repeat_total == 2


def test_requests_from_plain_color_selection_defaults_to_one_press_each():
    decision = {
        "confirmed": True,
        "intent": "make_cocktail",
        "recipe_id": "custom_color_selection",
        "dispenser_ids": ["yellow", "blue"],
    }

    requests = requests_from_decision(decision)

    assert [request.target_dispenser for request in requests] == ["yellow", "blue"]
    assert all(request.repeat_total == 1 for request in requests)


def test_build_dispenser_launch_command_passes_target_and_safety_args():
    request = requests_from_decision(
        {
            "intent": "make_cocktail",
            "recipe_id": "recipe_01",
            "dispenser_ids": ["red"],
        }
    )[0]

    command = build_dispenser_launch_command(
        request,
        launch_file="dispenser_press.launch.py",
        service_prefix="/",
        tcp_name="rg2_tcp",
        restore_tcp_after_run=True,
        require_tcp_for_taught_posx=True,
        allow_tcp_set_failure=False,
        joint_velocity=10.0,
        joint_acceleration=10.0,
        line_velocity=15.0,
        line_acceleration=25.0,
    )

    assert command[:4] == ["ros2", "launch", "azas_dispenser", "dispenser_press.launch.py"]
    assert "target_dispenser:=red" in command
    assert "service_prefix:=/" in command
    assert "tcp_name:=rg2_tcp" in command
    assert "line_velocity:=15.0" in command


def test_build_dispenser_launch_command_omits_empty_tcp_name():
    request = requests_from_decision(
        {
            "intent": "make_cocktail",
            "recipe_id": "recipe_01",
            "dispenser_ids": ["red"],
        }
    )[0]

    command = build_dispenser_launch_command(
        request,
        launch_file="dispenser_press.launch.py",
        service_prefix="/",
        tcp_name="",
        restore_tcp_after_run=True,
        require_tcp_for_taught_posx=False,
        allow_tcp_set_failure=False,
        joint_velocity=10.0,
        joint_acceleration=10.0,
        line_velocity=15.0,
        line_acceleration=25.0,
    )

    # An empty tcp_name must not produce a malformed "tcp_name:=" launch arg.
    assert not any(arg.startswith("tcp_name:=") for arg in command)
    assert "require_tcp_for_taught_posx:=false" in command
