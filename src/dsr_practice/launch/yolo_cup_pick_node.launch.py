from copy import deepcopy

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _as_bool(value):
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _runtime_nodes(context, moveit_params, moveit_py_params, side_prepose_params):
    controller_name = LaunchConfiguration("moveit_controller_name").perform(context)
    runtime_moveit_params = deepcopy(moveit_params)
    runtime_moveit_params["moveit_simple_controller_manager"] = {
        "controller_names": [controller_name],
        controller_name: {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
            ],
        },
    }

    nodes = []
    if _as_bool(LaunchConfiguration("start_joint_state_relay").perform(context)):
        nodes.append(
            Node(
                package="dsr_practice",
                executable="joint_state_relay",
                name="joint_state_relay",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/dsr01/joint_states",
                        "output_topic": "/joint_states",
                    }
                ],
            )
        )

    if _as_bool(LaunchConfiguration("workspace_collision_scene_enabled").perform(context)):
        nodes.append(
            Node(
                package="azas_motion",
                executable="workspace_collision_scene_node",
                name="workspace_collision_scene_node",
                output="screen",
                parameters=[
                    {
                        "safety_config_path": ParameterValue(
                            LaunchConfiguration("safety_config_path"),
                            value_type=str,
                        ),
                        "publish_period_sec": ParameterValue(
                            LaunchConfiguration(
                                "workspace_collision_publish_period_sec"
                            ),
                            value_type=float,
                        ),
                        "table_collision_enabled": ParameterValue(
                            LaunchConfiguration("table_collision_enabled"),
                            value_type=bool,
                        ),
                        "table_surface_z": ParameterValue(
                            LaunchConfiguration("table_surface_z"),
                            value_type=float,
                        ),
                        "table_thickness": ParameterValue(
                            LaunchConfiguration("table_thickness"),
                            value_type=float,
                        ),
                        "table_size_x": ParameterValue(
                            LaunchConfiguration("table_size_x"),
                            value_type=float,
                        ),
                        "table_size_y": ParameterValue(
                            LaunchConfiguration("table_size_y"),
                            value_type=float,
                        ),
                        "table_center_x": ParameterValue(
                            LaunchConfiguration("table_center_x"),
                            value_type=float,
                        ),
                        "table_center_y": ParameterValue(
                            LaunchConfiguration("table_center_y"),
                            value_type=float,
                        ),
                        "table_collision_expand_to_workspace_walls": ParameterValue(
                            LaunchConfiguration(
                                "table_collision_expand_to_workspace_walls"
                            ),
                            value_type=bool,
                        ),
                        "workspace_boundary_collision_enabled": ParameterValue(
                            LaunchConfiguration("workspace_boundary_collision_enabled"),
                            value_type=bool,
                        ),
                        "workspace_boundary_collision_prefix": ParameterValue(
                            LaunchConfiguration("workspace_boundary_collision_prefix"),
                            value_type=str,
                        ),
                        "workspace_boundary_wall_thickness": ParameterValue(
                            LaunchConfiguration("workspace_boundary_wall_thickness"),
                            value_type=float,
                        ),
                        "workspace_boundary_wall_clearance": ParameterValue(
                            LaunchConfiguration("workspace_boundary_wall_clearance"),
                            value_type=float,
                        ),
                    }
                ],
            )
        )

    if _as_bool(LaunchConfiguration("dispenser_collision_enabled").perform(context)):
        nodes.append(
            Node(
                package="azas_motion",
                executable="measured_dispenser_collision_scene_node",
                name="measured_dispenser_collision_scene_node",
                output="screen",
                parameters=[
                    {
                        "config_path": ParameterValue(
                            LaunchConfiguration("dispenser_collision_config_path"),
                            value_type=str,
                        ),
                        "publish_period_sec": LaunchConfiguration(
                            "dispenser_collision_publish_period_sec"
                        ),
                        "publish_collision_objects": LaunchConfiguration(
                            "dispenser_collision_publish_objects"
                        ),
                        "publish_markers": LaunchConfiguration(
                            "dispenser_collision_publish_markers"
                        ),
                    }
                ],
            )
        )

    yolo_node = Node(
        package="dsr_practice",
        executable="yolo_cup_pick_node",
        output="screen",
        parameters=[
                runtime_moveit_params,
                moveit_py_params,
                side_prepose_params,
                {
                    "model_path": ParameterValue(
                        LaunchConfiguration("model_path"),
                        value_type=str,
                    ),
                    "conf": LaunchConfiguration("conf"),
                    "imgsz": LaunchConfiguration("imgsz"),
                    "device": ParameterValue(
                        LaunchConfiguration("device"),
                        value_type=str,
                    ),
                    "target_class": ParameterValue(
                        LaunchConfiguration("target_class"),
                        value_type=str,
                    ),
                    "auto_pick_interval": LaunchConfiguration("auto_pick_interval"),
                    "pick_depth_ratio": LaunchConfiguration("pick_depth_ratio"),
                    "depth_patch_radius": LaunchConfiguration("depth_patch_radius"),
                    "min_depth_valid_ratio": LaunchConfiguration("min_depth_valid_ratio"),
                    "min_depth_m": LaunchConfiguration("min_depth_m"),
                    "max_depth_m": LaunchConfiguration("max_depth_m"),
                    "redetect_on_approach": LaunchConfiguration("redetect_on_approach"),
                    "redetect_settle_sec": LaunchConfiguration("redetect_settle_sec"),
                    "grasp_mode": ParameterValue(
                        LaunchConfiguration("grasp_mode"),
                        value_type=str,
                    ),
                    "side_grasp_axis": ParameterValue(
                        LaunchConfiguration("side_grasp_axis"),
                        value_type=str,
                    ),
                    "side_grasp_direction": LaunchConfiguration("side_grasp_direction"),
                    "side_approach_offset": LaunchConfiguration("side_approach_offset"),
                    "side_staging_offset": LaunchConfiguration("side_staging_offset"),
                    "side_far_stage_enabled": LaunchConfiguration(
                        "side_far_stage_enabled"
                    ),
                    "side_short_stage_backoff_m": LaunchConfiguration(
                        "side_short_stage_backoff_m"
                    ),
                    "side_stage_y_min": LaunchConfiguration("side_stage_y_min"),
                    "side_stage_y_max": LaunchConfiguration("side_stage_y_max"),
                    "side_grasp_offset": LaunchConfiguration("side_grasp_offset"),
                    "side_grasp_z_offset": LaunchConfiguration("side_grasp_z_offset"),
                    "side_grasp_stop_backoff_m": LaunchConfiguration(
                        "side_grasp_stop_backoff_m"
                    ),
                    "side_close_underreach_m": LaunchConfiguration(
                        "side_close_underreach_m"
                    ),
                    "side_low_retry_lift_m": LaunchConfiguration(
                        "side_low_retry_lift_m"
                    ),
                    "side_low_retry_attempts": LaunchConfiguration(
                        "side_low_retry_attempts"
                    ),
                    "side_auto_direction_by_cup_y": LaunchConfiguration(
                        "side_auto_direction_by_cup_y"
                    ),
                    "side_candidate_plan_check_enabled": LaunchConfiguration(
                        "side_candidate_plan_check_enabled"
                    ),
                    "side_linear_approach_enabled": LaunchConfiguration(
                        "side_linear_approach_enabled"
                    ),
                    "side_final_slide_enabled": LaunchConfiguration(
                        "side_final_slide_enabled"
                    ),
                    "side_lift_after_grasp": LaunchConfiguration(
                        "side_lift_after_grasp"
                    ),
                    "side_fixed_grasp_z_enabled": LaunchConfiguration(
                        "side_fixed_grasp_z_enabled"
                    ),
                    "side_fixed_grasp_z": LaunchConfiguration("side_fixed_grasp_z"),
                    "side_project_bbox_center_to_fixed_z": LaunchConfiguration(
                        "side_project_bbox_center_to_fixed_z"
                    ),
                    "table_collision_enabled": LaunchConfiguration(
                        "table_collision_enabled"
                    ),
                    "table_surface_z": LaunchConfiguration("table_surface_z"),
                    "table_thickness": LaunchConfiguration("table_thickness"),
                    "table_size_x": LaunchConfiguration("table_size_x"),
                    "table_size_y": LaunchConfiguration("table_size_y"),
                    "table_center_x": LaunchConfiguration("table_center_x"),
                    "table_center_y": LaunchConfiguration("table_center_y"),
                    "table_collision_expand_to_workspace_walls": LaunchConfiguration(
                        "table_collision_expand_to_workspace_walls"
                    ),
                    "safety_config_path": ParameterValue(
                        LaunchConfiguration("safety_config_path"),
                        value_type=str,
                    ),
                    "safety_workspace_enforced": LaunchConfiguration(
                        "safety_workspace_enforced"
                    ),
                    "workspace_boundary_collision_enabled": LaunchConfiguration(
                        "workspace_boundary_collision_enabled"
                    ),
                    "workspace_boundary_collision_prefix": ParameterValue(
                        LaunchConfiguration("workspace_boundary_collision_prefix"),
                        value_type=str,
                    ),
                    "workspace_boundary_wall_thickness": LaunchConfiguration(
                        "workspace_boundary_wall_thickness"
                    ),
                    "workspace_boundary_wall_clearance": LaunchConfiguration(
                        "workspace_boundary_wall_clearance"
                    ),
                    "side_orientation_mode": ParameterValue(
                        LaunchConfiguration("side_orientation_mode"),
                        value_type=str,
                    ),
                    "side_tool_roll_deg": LaunchConfiguration("side_tool_roll_deg"),
                    "side_roll_deg": LaunchConfiguration("side_roll_deg"),
                    "side_pitch_deg": LaunchConfiguration("side_pitch_deg"),
                    "side_yaw_deg": LaunchConfiguration("side_yaw_deg"),
                    "center_check_enabled": LaunchConfiguration("center_check_enabled"),
                    "center_check_settle_sec": LaunchConfiguration(
                        "center_check_settle_sec"
                    ),
                    "center_check_x": LaunchConfiguration("center_check_x"),
                    "center_check_y": LaunchConfiguration("center_check_y"),
                    "center_check_z": LaunchConfiguration("center_check_z"),
                    "side_prepose_enabled": LaunchConfiguration("side_prepose_enabled"),
                    "side_prepose_split_z": LaunchConfiguration("side_prepose_split_z"),
                    "side_move_to_initial_center_before_close": LaunchConfiguration(
                        "side_move_to_initial_center_before_close"
                    ),
                    "pre_pick_joint1_clearance_deg": LaunchConfiguration(
                        "pre_pick_joint1_clearance_deg"
                    ),
                    "verify_motion": LaunchConfiguration("verify_motion"),
                    "motion_verify_tolerance": LaunchConfiguration(
                        "motion_verify_tolerance"
                    ),
                    "joint_goal_tolerance_rad": LaunchConfiguration(
                        "joint_goal_tolerance_rad"
                    ),
                    "move_to_camera_home": LaunchConfiguration("move_to_camera_home"),
                    "move_joint_home_before_camera_home": LaunchConfiguration(
                        "move_joint_home_before_camera_home"
                    ),
                    "camera_home_mode": ParameterValue(
                        LaunchConfiguration("camera_home_mode"),
                        value_type=str,
                    ),
                    "camera_home_joint_1_deg": LaunchConfiguration(
                        "camera_home_joint_1_deg"
                    ),
                    "camera_home_joint_2_deg": LaunchConfiguration(
                        "camera_home_joint_2_deg"
                    ),
                    "camera_home_joint_3_deg": LaunchConfiguration(
                        "camera_home_joint_3_deg"
                    ),
                    "camera_home_joint_4_deg": LaunchConfiguration(
                        "camera_home_joint_4_deg"
                    ),
                    "camera_home_joint_5_deg": LaunchConfiguration(
                        "camera_home_joint_5_deg"
                    ),
                    "camera_home_joint_6_deg": LaunchConfiguration(
                        "camera_home_joint_6_deg"
                    ),
                    "camera_home_x": LaunchConfiguration("camera_home_x"),
                    "camera_home_y": LaunchConfiguration("camera_home_y"),
                    "camera_home_z": LaunchConfiguration("camera_home_z"),
                    "camera_home_search_max_z": LaunchConfiguration(
                        "camera_home_search_max_z"
                    ),
                    "camera_home_search_min_z": LaunchConfiguration(
                        "camera_home_search_min_z"
                    ),
                    "camera_home_search_step_z": LaunchConfiguration(
                        "camera_home_search_step_z"
                    ),
                    "min_motion_z": LaunchConfiguration("min_motion_z"),
                    "workspace_xy_clamp_enabled": LaunchConfiguration(
                        "workspace_xy_clamp_enabled"
                    ),
                    "return_home_after_task": LaunchConfiguration(
                        "return_home_after_task"
                    ),
                    "return_to_camera_home_after_attempt": LaunchConfiguration(
                        "return_to_camera_home_after_attempt"
                    ),
                    "exit_after_first_pick": LaunchConfiguration(
                        "exit_after_first_pick"
                    ),
                    "place_x": LaunchConfiguration("place_x"),
                    "place_y": LaunchConfiguration("place_y"),
                    "place_z": LaunchConfiguration("place_z"),
                    "auto_pick": LaunchConfiguration("auto_pick"),
                },
        ],
    )
    nodes.append(yolo_node)
    nodes.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=yolo_node,
                on_exit=[Shutdown(reason="yolo_cup_pick_node exited")],
            )
        )
    )
    return nodes


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="m0609",
            package_name="dsr_moveit_config_m0609",
        )
        .robot_description(file_path="config/m0609.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf")
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_scene_monitor()
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()
    moveit_params["moveit_controller_manager"] = (
        "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    )

    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("dsr_practice"), "config", "moveit_py.yaml"]
    )
    side_prepose_params = PathJoinSubstitution(
        [FindPackageShare("dsr_practice"), "config", "side_prepose.yaml"]
    )

    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/home/ssu/Azas/best.pt",
        description="Path to trained cup YOLO weights.",
    )
    conf_arg = DeclareLaunchArgument("conf", default_value="0.35")
    imgsz_arg = DeclareLaunchArgument("imgsz", default_value="640")
    device_arg = DeclareLaunchArgument("device", default_value="cpu")
    target_class_arg = DeclareLaunchArgument("target_class", default_value="cup")
    auto_pick_interval_arg = DeclareLaunchArgument(
        "auto_pick_interval", default_value="3.0"
    )
    pick_depth_ratio_arg = DeclareLaunchArgument(
        "pick_depth_ratio", default_value="0.55"
    )
    depth_patch_radius_arg = DeclareLaunchArgument(
        "depth_patch_radius", default_value="7"
    )
    min_depth_valid_ratio_arg = DeclareLaunchArgument(
        "min_depth_valid_ratio", default_value="0.03"
    )
    min_depth_m_arg = DeclareLaunchArgument("min_depth_m", default_value="0.15")
    max_depth_m_arg = DeclareLaunchArgument("max_depth_m", default_value="1.20")
    redetect_on_approach_arg = DeclareLaunchArgument(
        "redetect_on_approach", default_value="true"
    )
    redetect_settle_sec_arg = DeclareLaunchArgument(
        "redetect_settle_sec", default_value="0.5"
    )
    grasp_mode_arg = DeclareLaunchArgument("grasp_mode", default_value="side")
    side_grasp_axis_arg = DeclareLaunchArgument(
        "side_grasp_axis", default_value="y_axis"
    )
    side_grasp_direction_arg = DeclareLaunchArgument(
        "side_grasp_direction",
        default_value="1.0",
        description="Default +Y side approach keeps the low side-grip staging pose away from the measured dispenser row.",
    )
    side_approach_offset_arg = DeclareLaunchArgument(
        "side_approach_offset",
        default_value="0.16",
        description="Low side approach starts this far outside the cup center.",
    )
    side_staging_offset_arg = DeclareLaunchArgument(
        "side_staging_offset",
        default_value="0.30",
        description="Far outside offset where the wrist first turns horizontal.",
    )
    side_far_stage_enabled_arg = DeclareLaunchArgument(
        "side_far_stage_enabled",
        default_value="false",
        description="If false, skip the far side-staging waypoint and keep the side-grip posture while approaching from the closer pre-grasp offset.",
    )
    side_short_stage_backoff_m_arg = DeclareLaunchArgument(
        "side_short_stage_backoff_m",
        default_value="0.06",
        description="When far stage is disabled, start this much farther behind the side pre-grasp point before moving to the target.",
    )
    side_stage_y_min_arg = DeclareLaunchArgument(
        "side_stage_y_min",
        default_value="-0.35",
        description="Minimum preferred base_link Y for side staging; side direction is flipped if the configured direction leaves this range.",
    )
    side_stage_y_max_arg = DeclareLaunchArgument(
        "side_stage_y_max",
        default_value="0.35",
        description="Maximum preferred base_link Y for side staging; side direction is flipped if the configured direction leaves this range.",
    )
    side_grasp_offset_arg = DeclareLaunchArgument(
        "side_grasp_offset", default_value="0.035"
    )
    side_grasp_z_offset_arg = DeclareLaunchArgument(
        "side_grasp_z_offset",
        default_value="0.05",
        description="Side grasp height offset from detected base point.",
    )
    side_grasp_stop_backoff_m_arg = DeclareLaunchArgument(
        "side_grasp_stop_backoff_m",
        default_value="0.04",
        description="Keep the open gripper this far outside the computed side-grasp point before closing.",
    )
    side_close_underreach_m_arg = DeclareLaunchArgument(
        "side_close_underreach_m",
        default_value="0.03",
        description="Extra XY underreach along the side approach direction to stop short of the detected cup center.",
    )
    side_low_retry_lift_m_arg = DeclareLaunchArgument(
        "side_low_retry_lift_m",
        default_value="0.03",
        description="Raise the low side-grip Z by this amount per retry if table/dispenser collision or IK blocks the first low pose.",
    )
    side_low_retry_attempts_arg = DeclareLaunchArgument(
        "side_low_retry_attempts",
        default_value="5",
        description="Number of raised-Z retries for the low side-grip staging pose.",
    )
    side_auto_direction_by_cup_y_arg = DeclareLaunchArgument(
        "side_auto_direction_by_cup_y",
        default_value="false",
        description="If true, flip side direction by cup Y; disabled by default because the measured dispenser row is on the -Y side.",
    )
    side_candidate_plan_check_enabled_arg = DeclareLaunchArgument(
        "side_candidate_plan_check_enabled",
        default_value="true",
        description="Plan-check both side-grip approach candidates before executing the first feasible one.",
    )
    side_linear_approach_enabled_arg = DeclareLaunchArgument(
        "side_linear_approach_enabled",
        default_value="true",
        description="Use Pilz LIN for the final side approach and vertical lift.",
    )
    side_final_slide_enabled_arg = DeclareLaunchArgument(
        "side_final_slide_enabled",
        default_value="false",
        description="If true, move pre-grasp first and then perform an extra final slide into guarded close pose.",
    )
    side_lift_after_grasp_arg = DeclareLaunchArgument(
        "side_lift_after_grasp",
        default_value="false",
        description="After closing RG2 on a side grasp, lift the held cup before handing off to downstream tasks.",
    )
    side_fixed_grasp_z_enabled_arg = DeclareLaunchArgument(
        "side_fixed_grasp_z_enabled",
        default_value="false",
        description="Use a fixed base_link Z height for side grasp instead of detected depth Z plus offset.",
    )
    side_fixed_grasp_z_arg = DeclareLaunchArgument(
        "side_fixed_grasp_z",
        default_value="0.07",
        description="Fixed side grasp Z in base_link meters.",
    )
    side_project_bbox_center_to_fixed_z_arg = DeclareLaunchArgument(
        "side_project_bbox_center_to_fixed_z",
        default_value="true",
        description="Project the initial bbox center ray onto side_fixed_grasp_z for side target X/Y.",
    )
    dispenser_collision_enabled_arg = DeclareLaunchArgument(
        "dispenser_collision_enabled",
        default_value="true",
        description="Publish measured dispenser collision boxes to MoveIt's PlanningScene.",
    )
    dispenser_collision_config_path_arg = DeclareLaunchArgument(
        "dispenser_collision_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("azas_bringup"),
                "config",
                "measured_dispenser_collision.yaml",
            ]
        ),
        description="YAML with measured dispenser collision boxes in base_link.",
    )
    dispenser_collision_publish_period_sec_arg = DeclareLaunchArgument(
        "dispenser_collision_publish_period_sec",
        default_value="1.0",
        description="Republish period for measured dispenser collision objects.",
    )
    dispenser_collision_publish_objects_arg = DeclareLaunchArgument(
        "dispenser_collision_publish_objects",
        default_value="true",
        description=(
            "Publish measured dispenser boxes into MoveIt's PlanningScene. "
            "Set false only for geometry/RViz debugging, because disabling this "
            "removes dispenser collision avoidance from real motion planning."
        ),
    )
    dispenser_collision_publish_markers_arg = DeclareLaunchArgument(
        "dispenser_collision_publish_markers",
        default_value="true",
        description="Publish RViz markers for dispenser collision boxes.",
    )
    table_collision_enabled_arg = DeclareLaunchArgument(
        "table_collision_enabled",
        default_value="true",
        description="Publish a base_link table collision box so MoveIt avoids robot-link/table collisions.",
    )
    workspace_collision_scene_enabled_arg = DeclareLaunchArgument(
        "workspace_collision_scene_enabled",
        default_value="true",
        description="Start the shared workspace collision scene node for table and boundary walls.",
    )
    workspace_collision_publish_period_sec_arg = DeclareLaunchArgument(
        "workspace_collision_publish_period_sec",
        default_value="2.0",
        description="Republish period for shared workspace collision objects.",
    )
    table_surface_z_arg = DeclareLaunchArgument(
        "table_surface_z",
        default_value="0.0",
        description="Measured table top Z in base_link meters; only used when table_collision_enabled=true.",
    )
    table_thickness_arg = DeclareLaunchArgument(
        "table_thickness",
        default_value="0.04",
        description="Table collision box thickness below table_surface_z.",
    )
    table_size_x_arg = DeclareLaunchArgument(
        "table_size_x",
        default_value="1.20",
        description="Table collision box X size in base_link meters.",
    )
    table_size_y_arg = DeclareLaunchArgument(
        "table_size_y",
        default_value="1.00",
        description="Table collision box Y size in base_link meters.",
    )
    table_center_x_arg = DeclareLaunchArgument(
        "table_center_x",
        default_value="0.45",
        description="Table collision box center X in base_link meters.",
    )
    table_center_y_arg = DeclareLaunchArgument(
        "table_center_y",
        default_value="0.0",
        description="Table collision box center Y in base_link meters.",
    )
    table_collision_expand_to_workspace_walls_arg = DeclareLaunchArgument(
        "table_collision_expand_to_workspace_walls",
        default_value="true",
        description="Expand table collision XY footprint to the workspace wall inner faces.",
    )
    safety_config_path_arg = DeclareLaunchArgument(
        "safety_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("azas_bringup"),
                "config",
                "safety.yaml",
            ]
        ),
        description="YAML with enforced base_link workspace bounds.",
    )
    safety_workspace_enforced_arg = DeclareLaunchArgument(
        "safety_workspace_enforced",
        default_value="true",
        description="Fail closed before planning/execution when a pose goal is outside safety.yaml workspace bounds.",
    )
    workspace_boundary_collision_enabled_arg = DeclareLaunchArgument(
        "workspace_boundary_collision_enabled",
        default_value="true",
        description="Publish workspace wall collision objects from safety.yaml XY bounds.",
    )
    workspace_boundary_collision_prefix_arg = DeclareLaunchArgument(
        "workspace_boundary_collision_prefix",
        default_value="side_grip_workspace",
        description="Collision object ID prefix for workspace boundary walls.",
    )
    workspace_boundary_wall_thickness_arg = DeclareLaunchArgument(
        "workspace_boundary_wall_thickness",
        default_value="0.04",
        description="Thickness in meters for workspace boundary wall collision boxes.",
    )
    workspace_boundary_wall_clearance_arg = DeclareLaunchArgument(
        "workspace_boundary_wall_clearance",
        default_value="0.02",
        description="Extra XY clearance outside safety.yaml bounds for wall collision boxes; pose goals still use safety.yaml bounds.",
    )
    side_orientation_mode_arg = DeclareLaunchArgument(
        "side_orientation_mode",
        default_value="approach",
        description="Side grasp orientation: approach, euler, or home.",
    )
    side_tool_roll_deg_arg = DeclareLaunchArgument(
        "side_tool_roll_deg",
        default_value="0.0",
        description="Twist around the horizontal approach direction for RG2 finger alignment.",
    )
    side_roll_deg_arg = DeclareLaunchArgument(
        "side_roll_deg",
        default_value="0.0",
        description="Manual side grasp roll, used when side_orientation_mode:=euler.",
    )
    side_pitch_deg_arg = DeclareLaunchArgument(
        "side_pitch_deg",
        default_value="90.0",
        description="Manual side grasp pitch, used when side_orientation_mode:=euler.",
    )
    side_yaw_deg_arg = DeclareLaunchArgument(
        "side_yaw_deg",
        default_value="0.0",
        description="Manual side grasp yaw, used when side_orientation_mode:=euler.",
    )
    center_check_enabled_arg = DeclareLaunchArgument(
        "center_check_enabled",
        default_value="true",
        description="Move to a high observe pose and re-detect cup center before picking.",
    )
    center_check_settle_sec_arg = DeclareLaunchArgument(
        "center_check_settle_sec",
        default_value="0.6",
        description="Seconds to wait for camera frames after moving to center-check pose.",
    )
    center_check_x_arg = DeclareLaunchArgument("center_check_x", default_value="0.45")
    center_check_y_arg = DeclareLaunchArgument("center_check_y", default_value="0.0")
    center_check_z_arg = DeclareLaunchArgument("center_check_z", default_value="0.64")
    side_prepose_enabled_arg = DeclareLaunchArgument(
        "side_prepose_enabled",
        default_value="false",
        description="Enable rule-based joint-space pre-pose before side grasp.",
    )
    side_prepose_split_z_arg = DeclareLaunchArgument(
        "side_prepose_split_z",
        default_value="0.18",
        description="Cup base z threshold to select low/high prepose.",
    )
    side_move_to_initial_center_before_close_arg = DeclareLaunchArgument(
        "side_move_to_initial_center_before_close",
        default_value="false",
        description="Deprecated/no-op: center move before close is blocked to avoid pushing the cup.",
    )
    pre_pick_joint1_clearance_deg_arg = DeclareLaunchArgument(
        "pre_pick_joint1_clearance_deg",
        default_value="0.0",
        description="Optional joint_1 detour before side grip; disabled by default when side prepose is enabled.",
    )
    verify_motion_arg = DeclareLaunchArgument("verify_motion", default_value="true")
    motion_verify_tolerance_arg = DeclareLaunchArgument(
        "motion_verify_tolerance", default_value="0.01"
    )
    joint_goal_tolerance_rad_arg = DeclareLaunchArgument(
        "joint_goal_tolerance_rad", default_value="0.02"
    )
    move_to_camera_home_arg = DeclareLaunchArgument(
        "move_to_camera_home", default_value="true"
    )
    move_joint_home_before_camera_home_arg = DeclareLaunchArgument(
        "move_joint_home_before_camera_home",
        default_value="false",
        description="Move joint home before camera home. Disabled to start directly at high camera home.",
    )
    camera_home_mode_arg = DeclareLaunchArgument(
        "camera_home_mode",
        default_value="joint",
        description="Camera observe home mode: 'joint' uses taught joint angles, 'pose' uses Cartesian target.",
    )
    camera_home_joint_1_deg_arg = DeclareLaunchArgument(
        "camera_home_joint_1_deg", default_value="3.0"
    )
    camera_home_joint_2_deg_arg = DeclareLaunchArgument(
        "camera_home_joint_2_deg", default_value="-12.7"
    )
    camera_home_joint_3_deg_arg = DeclareLaunchArgument(
        "camera_home_joint_3_deg", default_value="44.0"
    )
    camera_home_joint_4_deg_arg = DeclareLaunchArgument(
        "camera_home_joint_4_deg", default_value="-9.0"
    )
    camera_home_joint_5_deg_arg = DeclareLaunchArgument(
        "camera_home_joint_5_deg", default_value="133.0"
    )
    camera_home_joint_6_deg_arg = DeclareLaunchArgument(
        "camera_home_joint_6_deg",
        default_value="90.0",
        description="Small wrist twist for the straight-up camera observe joint home; tune sign/angle from camera view.",
    )
    camera_home_x_arg = DeclareLaunchArgument("camera_home_x", default_value="0.45")
    camera_home_y_arg = DeclareLaunchArgument("camera_home_y", default_value="0.0")
    camera_home_z_arg = DeclareLaunchArgument("camera_home_z", default_value="0.64")
    camera_home_search_max_z_arg = DeclareLaunchArgument(
        "camera_home_search_max_z",
        default_value="0.64",
        description="Highest base_link Z to try for camera home; node descends until IK/planning succeeds.",
    )
    camera_home_search_min_z_arg = DeclareLaunchArgument(
        "camera_home_search_min_z",
        default_value="0.54",
        description="Lowest fallback base_link Z to try for camera home search.",
    )
    camera_home_search_step_z_arg = DeclareLaunchArgument(
        "camera_home_search_step_z",
        default_value="0.02",
        description="Camera home Z search step in meters.",
    )
    min_motion_z_arg = DeclareLaunchArgument(
        "min_motion_z",
        default_value="0.07",
        description="Minimum allowed commanded Z in base frame.",
    )
    workspace_xy_clamp_enabled_arg = DeclareLaunchArgument(
        "workspace_xy_clamp_enabled",
        default_value="false",
        description="If true, clamp commanded X/Y to legacy fixed workspace bounds; Z clamp still uses min_motion_z.",
    )
    return_home_after_task_arg = DeclareLaunchArgument(
        "return_home_after_task", default_value="true"
    )
    return_to_camera_home_after_attempt_arg = DeclareLaunchArgument(
        "return_to_camera_home_after_attempt",
        default_value="true",
        description="After each pick attempt, return to camera home even if the attempt failed.",
    )
    exit_after_first_pick_arg = DeclareLaunchArgument(
        "exit_after_first_pick",
        default_value="false",
        description="Exit this node after one successful pick so a wrapper can continue with the held cup.",
    )
    place_x_arg = DeclareLaunchArgument("place_x", default_value="0.45")
    place_y_arg = DeclareLaunchArgument("place_y", default_value="0.0")
    place_z_arg = DeclareLaunchArgument("place_z", default_value="0.30")
    auto_pick_arg = DeclareLaunchArgument("auto_pick", default_value="false")
    moveit_controller_name_arg = DeclareLaunchArgument(
        "moveit_controller_name",
        default_value="/dsr_moveit_controller",
        description="MoveIt FollowJointTrajectory controller name. Use /dsr01/dsr_moveit_controller for namespaced Doosan bringup.",
    )
    start_joint_state_relay_arg = DeclareLaunchArgument(
        "start_joint_state_relay",
        default_value="true",
        description="Start /dsr01/joint_states -> /joint_states relay. Disable if another relay already runs.",
    )

    return LaunchDescription(
        [
            model_path_arg,
            conf_arg,
            imgsz_arg,
            device_arg,
            target_class_arg,
            auto_pick_interval_arg,
            pick_depth_ratio_arg,
            depth_patch_radius_arg,
            min_depth_valid_ratio_arg,
            min_depth_m_arg,
            max_depth_m_arg,
            redetect_on_approach_arg,
            redetect_settle_sec_arg,
            grasp_mode_arg,
            side_grasp_axis_arg,
            side_grasp_direction_arg,
            side_approach_offset_arg,
            side_staging_offset_arg,
            side_far_stage_enabled_arg,
            side_short_stage_backoff_m_arg,
            side_stage_y_min_arg,
            side_stage_y_max_arg,
            side_grasp_offset_arg,
            side_grasp_z_offset_arg,
            side_grasp_stop_backoff_m_arg,
            side_close_underreach_m_arg,
            side_low_retry_lift_m_arg,
            side_low_retry_attempts_arg,
            side_auto_direction_by_cup_y_arg,
            side_candidate_plan_check_enabled_arg,
            side_linear_approach_enabled_arg,
            side_final_slide_enabled_arg,
            side_lift_after_grasp_arg,
            side_fixed_grasp_z_enabled_arg,
            side_fixed_grasp_z_arg,
            side_project_bbox_center_to_fixed_z_arg,
            dispenser_collision_enabled_arg,
            dispenser_collision_config_path_arg,
            dispenser_collision_publish_period_sec_arg,
            dispenser_collision_publish_objects_arg,
            dispenser_collision_publish_markers_arg,
            workspace_collision_scene_enabled_arg,
            workspace_collision_publish_period_sec_arg,
            table_collision_enabled_arg,
            table_surface_z_arg,
            table_thickness_arg,
            table_size_x_arg,
            table_size_y_arg,
            table_center_x_arg,
            table_center_y_arg,
            table_collision_expand_to_workspace_walls_arg,
            safety_config_path_arg,
            safety_workspace_enforced_arg,
            workspace_boundary_collision_enabled_arg,
            workspace_boundary_collision_prefix_arg,
            workspace_boundary_wall_thickness_arg,
            workspace_boundary_wall_clearance_arg,
            side_orientation_mode_arg,
            side_tool_roll_deg_arg,
            side_roll_deg_arg,
            side_pitch_deg_arg,
            side_yaw_deg_arg,
            center_check_enabled_arg,
            center_check_settle_sec_arg,
            center_check_x_arg,
            center_check_y_arg,
            center_check_z_arg,
            side_prepose_enabled_arg,
            side_prepose_split_z_arg,
            side_move_to_initial_center_before_close_arg,
            pre_pick_joint1_clearance_deg_arg,
            verify_motion_arg,
            motion_verify_tolerance_arg,
            joint_goal_tolerance_rad_arg,
            move_to_camera_home_arg,
            move_joint_home_before_camera_home_arg,
            camera_home_mode_arg,
            camera_home_joint_1_deg_arg,
            camera_home_joint_2_deg_arg,
            camera_home_joint_3_deg_arg,
            camera_home_joint_4_deg_arg,
            camera_home_joint_5_deg_arg,
            camera_home_joint_6_deg_arg,
            camera_home_x_arg,
            camera_home_y_arg,
            camera_home_z_arg,
            camera_home_search_max_z_arg,
            camera_home_search_min_z_arg,
            camera_home_search_step_z_arg,
            min_motion_z_arg,
            workspace_xy_clamp_enabled_arg,
            return_home_after_task_arg,
            return_to_camera_home_after_attempt_arg,
            exit_after_first_pick_arg,
            place_x_arg,
            place_y_arg,
            place_z_arg,
            auto_pick_arg,
            moveit_controller_name_arg,
            start_joint_state_relay_arg,
            OpaqueFunction(
                function=_runtime_nodes,
                args=[moveit_params, moveit_py_params, side_prepose_params],
            ),
        ]
    )
