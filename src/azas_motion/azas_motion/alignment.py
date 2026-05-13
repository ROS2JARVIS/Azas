from dataclasses import dataclass

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3


@dataclass(frozen=True)
class AlignmentConfig:
    outlet_clearance_m: float


@dataclass(frozen=True)
class NoMotionPickPlan:
    pick_pose: Pose
    approach_pose: Pose
    lift_pose: Pose


def compute_alignment_tcp_pose(
    dispenser_outlet: Point,
    offset_tcp_to_cup_mouth: Vector3,
    orientation: Quaternion,
    config: AlignmentConfig,
) -> Pose:
    """Compute TCP pose so cup_mouth_center sits below dispenser_outlet.

    Inputs must be measured/calibrated. The function only applies the wiki equation;
    it does not define outlet coordinates, EE_LINK, GROUP_NAME, or TCP offset.
    """
    if config.outlet_clearance_m < 0.0:
        raise ValueError("outlet_clearance_m must be non-negative")

    pose = Pose()
    pose.position.x = dispenser_outlet.x - offset_tcp_to_cup_mouth.x
    pose.position.y = dispenser_outlet.y - offset_tcp_to_cup_mouth.y
    pose.position.z = (
        dispenser_outlet.z - config.outlet_clearance_m - offset_tcp_to_cup_mouth.z
    )
    pose.orientation = orientation
    return pose


def compute_no_motion_pick_plan(
    tumbler_pose: Pose,
    approach_z_offset_m: float = 0.10,
    lift_z_offset_m: float = 0.12,
) -> NoMotionPickPlan:
    """Compute pick/approach/lift poses for logging only; never commands motion."""
    if approach_z_offset_m < 0.0:
        raise ValueError("approach_z_offset_m must be non-negative")
    if lift_z_offset_m < 0.0:
        raise ValueError("lift_z_offset_m must be non-negative")

    pick_pose = Pose()
    pick_pose.position = tumbler_pose.position
    pick_pose.orientation = tumbler_pose.orientation

    approach_pose = Pose()
    approach_pose.position.x = tumbler_pose.position.x
    approach_pose.position.y = tumbler_pose.position.y
    approach_pose.position.z = tumbler_pose.position.z + approach_z_offset_m
    approach_pose.orientation = tumbler_pose.orientation

    lift_pose = Pose()
    lift_pose.position.x = tumbler_pose.position.x
    lift_pose.position.y = tumbler_pose.position.y
    lift_pose.position.z = tumbler_pose.position.z + lift_z_offset_m
    lift_pose.orientation = tumbler_pose.orientation

    return NoMotionPickPlan(
        pick_pose=pick_pose,
        approach_pose=approach_pose,
        lift_pose=lift_pose,
    )
