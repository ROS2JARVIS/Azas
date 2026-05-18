import math
from dataclasses import dataclass

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3


PICK_PLACE_STATES = (
    "HOME",
    "OBSERVE_CUP_POSE",
    "DETECT_CUP",
    "COMPUTE_SIDE_GRASP",
    "PLAN_SIDE_GRASP",
    "GRIPPER_OPEN",
    "MOVE_APPROACH",
    "MOVE_GRASP",
    "GRIPPER_CLOSE",
    "LIFT",
    "DONE",
)


@dataclass(frozen=True)
class AlignmentConfig:
    outlet_clearance_m: float


@dataclass(frozen=True)
class NoMotionPickPlan:
    pick_pose: Pose
    approach_pose: Pose
    lift_pose: Pose


@dataclass(frozen=True)
class SideGraspConfig:
    orientation_source: str = "input"
    side_grasp_qx: float = 0.0
    side_grasp_qy: float = 0.0
    side_grasp_qz: float = 0.0
    side_grasp_qw: float = 1.0
    side_approach_axis: str = "-x"
    side_approach_offset_m: float = 0.12
    side_clearance_m: float = 0.02
    cup_radius_m: float = 0.035
    grasp_height_offset_m: float = 0.06
    lift_offset_m: float = 0.12
    min_grasp_z_m: float = 0.03
    max_grasp_z_m: float = 0.40


@dataclass(frozen=True)
class SideGraspPlan:
    approach_pose: Pose
    grasp_pose: Pose
    lift_pose: Pose
    approach_axis: str
    approach_distance_m: float
    warning: str


@dataclass(frozen=True)
class ObservePoseConfig:
    x: float = 0.35
    y: float = -0.25
    z: float = 0.45
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


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

    pick_pose = _copy_pose(tumbler_pose)
    approach_pose = _pose_with_z_offset(tumbler_pose, approach_z_offset_m)
    lift_pose = _pose_with_z_offset(tumbler_pose, lift_z_offset_m)

    return NoMotionPickPlan(
        pick_pose=pick_pose,
        approach_pose=approach_pose,
        lift_pose=lift_pose,
    )


def compute_side_grasp_plan(
    cup_reference_pose: Pose,
    config: SideGraspConfig,
) -> SideGraspPlan:
    """Compute side-grasp poses for no-motion diagnostics only.

    The returned poses are geometric candidates, not robot-ready trajectories.
    They become executable only after measured TCP/cup offsets, collision scene,
    workspace bounds, gripper width/force, and MoveIt feasibility are verified.
    """
    _validate_side_grasp_config(config)

    orientation, orientation_warning = _side_grasp_orientation(cup_reference_pose, config)

    grasp_pose = _copy_pose(cup_reference_pose)
    grasp_pose.orientation = orientation
    grasp_pose.position.z = cup_reference_pose.position.z + config.grasp_height_offset_m
    if (
        grasp_pose.position.z < config.min_grasp_z_m
        or grasp_pose.position.z > config.max_grasp_z_m
    ):
        raise ValueError(
            "SIDE_GRASP_Z_OUT_OF_BOUNDS: "
            f"grasp_z={grasp_pose.position.z:.3f} "
            f"outside [{config.min_grasp_z_m:.3f}, {config.max_grasp_z_m:.3f}]"
        )

    approach_distance_m = (
        config.side_approach_offset_m + config.cup_radius_m + config.side_clearance_m
    )
    approach_pose = _copy_pose(grasp_pose)
    if config.side_approach_axis == "-x":
        approach_pose.position.x = grasp_pose.position.x + approach_distance_m
    elif config.side_approach_axis == "+x":
        approach_pose.position.x = grasp_pose.position.x - approach_distance_m
    elif config.side_approach_axis == "-y":
        approach_pose.position.y = grasp_pose.position.y + approach_distance_m
    elif config.side_approach_axis == "+y":
        approach_pose.position.y = grasp_pose.position.y - approach_distance_m

    lift_pose = _pose_with_z_offset(grasp_pose, config.lift_offset_m)

    warning = "orientation placeholder; no real robot side grasp allowed"
    if orientation_warning:
        warning = f"{warning}; {orientation_warning}"

    return SideGraspPlan(
        approach_pose=approach_pose,
        grasp_pose=grasp_pose,
        lift_pose=lift_pose,
        approach_axis=config.side_approach_axis,
        approach_distance_m=approach_distance_m,
        warning=warning,
    )


def compute_observe_pose(config: ObservePoseConfig) -> Pose:
    """Return a candidate high observation pose; planning-only until operator approval."""
    values = (config.x, config.y, config.z, config.qx, config.qy, config.qz, config.qw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("observe pose values must be finite")
    if config.z <= 0.0:
        raise ValueError("observe_pose_z must be positive")
    norm = math.sqrt(
        config.qx * config.qx
        + config.qy * config.qy
        + config.qz * config.qz
        + config.qw * config.qw
    )
    if norm == 0.0:
        raise ValueError("observe pose quaternion norm must be non-zero")

    pose = Pose()
    pose.position.x = config.x
    pose.position.y = config.y
    pose.position.z = config.z
    pose.orientation.x = config.qx / norm
    pose.orientation.y = config.qy / norm
    pose.orientation.z = config.qz / norm
    pose.orientation.w = config.qw / norm
    return pose


def _validate_side_grasp_config(config: SideGraspConfig) -> None:
    if config.side_approach_axis not in ("-x", "+x", "-y", "+y"):
        raise ValueError(
            "UNSUPPORTED_SIDE_APPROACH_AXIS: "
            "side_approach_axis must be one of -x, +x, -y, +y"
        )
    if config.orientation_source not in ("input", "parameter"):
        raise ValueError("orientation_source must be 'input' or 'parameter'")
    if config.cup_radius_m <= 0.0:
        raise ValueError("cup_radius_m must be positive")
    if config.side_approach_offset_m <= 0.0:
        raise ValueError("side_approach_offset_m must be positive")
    if config.side_clearance_m < 0.0:
        raise ValueError("side_clearance_m must be non-negative")
    if config.lift_offset_m <= 0.0:
        raise ValueError("lift_offset_m must be positive")
    if config.min_grasp_z_m > config.max_grasp_z_m:
        raise ValueError("min_grasp_z_m must be <= max_grasp_z_m")


def _side_grasp_orientation(
    cup_reference_pose: Pose,
    config: SideGraspConfig,
) -> tuple[Quaternion, str]:
    if config.orientation_source == "input":
        orientation = Quaternion()
        orientation.x = cup_reference_pose.orientation.x
        orientation.y = cup_reference_pose.orientation.y
        orientation.z = cup_reference_pose.orientation.z
        orientation.w = cup_reference_pose.orientation.w
        return orientation, "using input orientation candidate"

    values = (
        config.side_grasp_qx,
        config.side_grasp_qy,
        config.side_grasp_qz,
        config.side_grasp_qw,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("side_grasp quaternion values must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm == 0.0:
        raise ValueError("side_grasp quaternion norm must be non-zero")

    orientation = Quaternion()
    orientation.x = config.side_grasp_qx / norm
    orientation.y = config.side_grasp_qy / norm
    orientation.z = config.side_grasp_qz / norm
    orientation.w = config.side_grasp_qw / norm
    if abs(norm - 1.0) > 1e-6:
        return orientation, f"parameter quaternion normalized from norm={norm:.6f}"
    return orientation, "using parameter quaternion candidate"


def _copy_pose(source: Pose) -> Pose:
    pose = Pose()
    pose.position.x = source.position.x
    pose.position.y = source.position.y
    pose.position.z = source.position.z
    pose.orientation.x = source.orientation.x
    pose.orientation.y = source.orientation.y
    pose.orientation.z = source.orientation.z
    pose.orientation.w = source.orientation.w
    return pose


def _pose_with_z_offset(source: Pose, offset_m: float) -> Pose:
    pose = _copy_pose(source)
    pose.position.z = source.position.z + offset_m
    return pose
