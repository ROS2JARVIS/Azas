import math

import pytest
from geometry_msgs.msg import Pose

from azas_motion.alignment import (
    ObservePoseConfig,
    SideGraspConfig,
    compute_observe_pose,
    compute_side_grasp_plan,
)


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

