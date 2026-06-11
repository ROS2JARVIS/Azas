import math

import pytest
from geometry_msgs.msg import Pose

from azas_motion.alignment import (
    ObservePoseConfig,
    SideGraspConfig,
    compute_observe_pose,
    compute_side_grasp_plan,
)
from azas_motion.lid_grip import LidGripConfig, compute_lid_grip_plan


def _pose(x=0.30, y=-0.10, z=0.20):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def test_side_grasp_plan_offsets_approach_without_motion():
    plan = compute_side_grasp_plan(
        _pose(),
        SideGraspConfig(
            side_approach_axis="-x",
            side_approach_offset_m=0.12,
            cup_radius_m=0.035,
            side_clearance_m=0.02,
            grasp_height_offset_m=0.06,
        ),
    )

    assert plan.grasp_pose.position.z == pytest.approx(0.26)
    assert plan.approach_pose.position.x == pytest.approx(0.475)
    assert plan.lift_pose.position.z == pytest.approx(0.38)
    assert "placeholder" in plan.warning


def test_side_grasp_plan_rejects_out_of_bounds_z():
    with pytest.raises(ValueError, match="SIDE_GRASP_Z_OUT_OF_BOUNDS"):
        compute_side_grasp_plan(
            _pose(z=0.50),
            SideGraspConfig(grasp_height_offset_m=0.06, max_grasp_z_m=0.40),
        )


def test_observe_pose_normalizes_quaternion():
    pose = compute_observe_pose(ObservePoseConfig(qx=0.0, qy=0.0, qz=0.0, qw=2.0))

    assert pose.orientation.w == pytest.approx(1.0)
    norm = math.sqrt(
        pose.orientation.x**2
        + pose.orientation.y**2
        + pose.orientation.z**2
        + pose.orientation.w**2
    )
    assert norm == pytest.approx(1.0)


def test_lid_grip_plan_offsets_along_detected_local_z():
    pose = _pose(z=0.20)
    plan = compute_lid_grip_plan(
        pose,
        LidGripConfig(approach_offset_m=0.08, lift_offset_m=0.10),
    )

    assert plan.grasp_pose.position.z == pytest.approx(0.20)
    assert plan.approach_pose.position.z == pytest.approx(0.28)
    assert plan.lift_pose.position.z == pytest.approx(0.30)


def test_lid_grip_plan_applies_measured_tcp_grasp_offset_in_base_frame():
    pose = _pose(x=0.30, y=-0.10, z=0.20)
    plan = compute_lid_grip_plan(
        pose,
        LidGripConfig(
            approach_offset_m=0.08,
            lift_offset_m=0.10,
            surface_offset_m=0.02,
            offset_axis="base_z",
            tcp_grasp_offset_x_m=-0.006,
            tcp_grasp_offset_y_m=-0.045,
            tcp_grasp_offset_z_m=-0.064,
            min_grasp_z_m=0.0,
        ),
    )

    assert plan.grasp_pose.position.x == pytest.approx(0.294)
    assert plan.grasp_pose.position.y == pytest.approx(-0.145)
    assert plan.grasp_pose.position.z == pytest.approx(0.156)
    assert plan.approach_pose.position.z == pytest.approx(0.236)
    assert plan.lift_pose.position.z == pytest.approx(0.256)


def test_lid_grip_plan_clamps_approach_height_above_low_depth_detection():
    pose = _pose(x=0.41, y=-0.06, z=0.072)
    plan = compute_lid_grip_plan(
        pose,
        LidGripConfig(
            approach_offset_m=0.08,
            lift_offset_m=0.10,
            min_approach_z_m=0.26,
            offset_axis="base_z",
            tcp_grasp_offset_z_m=0.16,
            min_grasp_z_m=0.18,
        ),
    )

    assert plan.grasp_pose.position.z == pytest.approx(0.232)
    assert plan.approach_pose.position.z == pytest.approx(0.312)
    assert plan.lift_pose.position.z == pytest.approx(0.332)


def test_lid_grip_plan_refuses_low_grasp_after_tcp_compensation():
    with pytest.raises(ValueError, match="LID_GRASP_Z_OUT_OF_BOUNDS"):
        compute_lid_grip_plan(
            _pose(z=0.01),
            LidGripConfig(
                offset_axis="base_z",
                tcp_grasp_offset_z_m=0.16,
                min_grasp_z_m=0.18,
            ),
        )


def test_lid_grip_plan_rejects_unmeasured_z_bounds():
    with pytest.raises(ValueError, match="LID_GRASP_Z_OUT_OF_BOUNDS"):
        compute_lid_grip_plan(
            _pose(z=0.80),
            LidGripConfig(max_grasp_z_m=0.60),
        )
