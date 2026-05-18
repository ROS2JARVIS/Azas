import pytest

from azas_motion.dispenser_targets import (
    Position,
    dispenser_front_targets,
    nearest_dispenser_order,
    parse_outlet_positions,
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
