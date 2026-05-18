import pytest

from azas_motion.dispenser_targets import (
    Position,
    dispenser_front_targets,
    nearest_dispenser_order,
    parse_outlet_positions,
    safe_dispenser_transfer_targets,
    selected_outlet,
)


def test_dispenser_front_target_matches_logged_dispenser_two_hold():
    outlets = parse_outlet_positions(
        [0.60, 0.08, 0.392, 0.60, 0.02, 0.392, 0.60, -0.04, 0.392]
    )

    target = dispenser_front_targets(
        2,
        selected_outlet(outlets, 2),
        front_approach_offset_x=0.12,
        outlet_front_offset_x=0.02,
        transfer_z_override=0.20,
        detour_y=-0.24,
        enable_obstacle_detour=False,
    )[0]

    assert target.label == "dispenser_2_outlet_front_hold"
    assert target.position.x == pytest.approx(0.580)
    assert target.position.y == pytest.approx(0.020)
    assert target.position.z == pytest.approx(0.200)


def test_dispenser_front_target_uses_outlet_z_without_override():
    outlet = Position(0.60, 0.02, 0.392)

    target = dispenser_front_targets(
        2,
        outlet,
        front_approach_offset_x=0.12,
        outlet_front_offset_x=0.02,
        transfer_z_override=0.0,
        detour_y=-0.24,
        enable_obstacle_detour=False,
    )[0]

    assert target.position.z == pytest.approx(0.392)


def test_parse_outlets_rejects_non_xyz_flat_list():
    with pytest.raises(ValueError, match="flat XYZ"):
        parse_outlet_positions([0.60, 0.08])


def test_nearest_order_keeps_motion_sequence_distance_based():
    outlets = parse_outlet_positions([0.60, 0.08, 0.392, 0.60, 0.02, 0.392])

    ordered = nearest_dispenser_order(
        [1, 2],
        outlets,
        Position(0.58, 0.01, 0.20),
        outlet_front_offset_x=0.02,
        transfer_z_override=0.20,
    )

    assert ordered == [2, 1]


def test_safe_dispenser_transfer_lifts_before_front_hold():
    outlet = Position(0.60, 0.02, 0.392)

    targets = safe_dispenser_transfer_targets(
        2,
        outlet,
        Position(0.55, -0.10, 0.10),
        outlet_front_offset_x=0.02,
        transfer_z_override=0.20,
        safe_lift_min_z=0.40,
        safe_lift_delta_z=0.15,
        safe_lift_max_z=0.55,
        dispenser_above_z=0.40,
        include_initial_lift=True,
    )

    assert [target.label for target in targets] == [
        "dispenser_2_safe_lift",
        "dispenser_2_above",
        "dispenser_2_outlet_front_hold",
        "dispenser_2_retreat_above",
    ]
    assert targets[0].position == Position(0.55, -0.10, 0.40)
    assert targets[1].position == Position(0.58, 0.02, 0.40)
    assert targets[2].position == Position(0.58, 0.02, 0.20)
    assert targets[3].position == Position(0.58, 0.02, 0.40)


def test_safe_dispenser_transfer_sequence_can_skip_repeated_initial_lift():
    outlet = Position(0.60, 0.08, 0.392)

    targets = safe_dispenser_transfer_targets(
        1,
        outlet,
        Position(0.58, 0.02, 0.40),
        outlet_front_offset_x=0.02,
        transfer_z_override=0.20,
        safe_lift_min_z=0.40,
        safe_lift_delta_z=0.15,
        safe_lift_max_z=0.55,
        dispenser_above_z=0.40,
        include_initial_lift=False,
        prefix="seq_2_",
    )

    assert [target.label for target in targets] == [
        "seq_2_dispenser_1_above",
        "seq_2_dispenser_1_outlet_front_hold",
        "seq_2_dispenser_1_retreat_above",
    ]
