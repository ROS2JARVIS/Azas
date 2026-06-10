from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


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
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="pilz_industrial_motion_planner",
        )
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()
    moveit_params["moveit_controller_manager"] = (
        "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    )
    moveit_params["moveit_simple_controller_manager"] = {
        "controller_names": ["/dsr01/dsr_moveit_controller"],
        "/dsr01/dsr_moveit_controller": {
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
    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("azas_bringup"), "config", "moveit_py.yaml"]
    )

    joint_state_relay = Node(
        package="azas_bringup",
        executable="joint_state_relay_legacy",
        name="joint_state_relay",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_joint_state_relay")),
        parameters=[
            {
                "input_topic": ParameterValue(
                    LaunchConfiguration("joint_state_relay_input_topic"),
                    value_type=str,
                ),
                "output_topic": ParameterValue(
                    LaunchConfiguration("joint_state_relay_output_topic"),
                    value_type=str,
                ),
            }
        ],
    )

    node = Node(
        package="azas_motion",
        executable="lid_screw_on_holder_node",
        name="lid_screw_on_holder_node",
        output="screen",
        parameters=[
            moveit_params,
            moveit_py_params,
            {
                "trigger_service": ParameterValue(
                    LaunchConfiguration("trigger_service"),
                    value_type=str,
                ),
                "status_topic": ParameterValue(
                    LaunchConfiguration("status_topic"),
                    value_type=str,
                ),
                "execute_motion": ParameterValue(
                    LaunchConfiguration("execute_motion"),
                    value_type=bool,
                ),
                "hardware_confirm": ParameterValue(
                    LaunchConfiguration("hardware_confirm"),
                    value_type=str,
                ),
                "shutdown_on_complete": ParameterValue(
                    LaunchConfiguration("shutdown_on_complete"),
                    value_type=bool,
                ),
                "service_prefix": ParameterValue(
                    LaunchConfiguration("service_prefix"),
                    value_type=str,
                ),
                "camera_info_topic": ParameterValue(
                    LaunchConfiguration("camera_info_topic"),
                    value_type=str,
                ),
                "color_topic": ParameterValue(
                    LaunchConfiguration("color_topic"),
                    value_type=str,
                ),
                "cup_pose_topic": ParameterValue(
                    LaunchConfiguration("cup_pose_topic"),
                    value_type=str,
                ),
                "raw_joint_state_topic": ParameterValue(
                    LaunchConfiguration("raw_joint_state_topic"),
                    value_type=str,
                ),
                "calibration_config_path": ParameterValue(
                    LaunchConfiguration("calibration_config_path"),
                    value_type=str,
                ),
                "safety_config_path": ParameterValue(
                    LaunchConfiguration("safety_config_path"),
                    value_type=str,
                ),
                "hand_eye_matrix_path": ParameterValue(
                    LaunchConfiguration("hand_eye_matrix_path"),
                    value_type=str,
                ),
                "safety_workspace_enforced": ParameterValue(
                    LaunchConfiguration("safety_workspace_enforced"),
                    value_type=bool,
                ),
                "move_to_observe_before_detect": ParameterValue(
                    LaunchConfiguration("move_to_observe_before_detect"),
                    value_type=bool,
                ),
                "observe_motion_backend": ParameterValue(
                    LaunchConfiguration("observe_motion_backend"),
                    value_type=str,
                ),
                "observe_joint_1_deg": ParameterValue(
                    LaunchConfiguration("observe_joint_1_deg"),
                    value_type=float,
                ),
                "observe_joint_2_deg": ParameterValue(
                    LaunchConfiguration("observe_joint_2_deg"),
                    value_type=float,
                ),
                "observe_joint_3_deg": ParameterValue(
                    LaunchConfiguration("observe_joint_3_deg"),
                    value_type=float,
                ),
                "observe_joint_4_deg": ParameterValue(
                    LaunchConfiguration("observe_joint_4_deg"),
                    value_type=float,
                ),
                "observe_joint_5_deg": ParameterValue(
                    LaunchConfiguration("observe_joint_5_deg"),
                    value_type=float,
                ),
                "observe_joint_6_deg": ParameterValue(
                    LaunchConfiguration("observe_joint_6_deg"),
                    value_type=float,
                ),
                "observe_joint_velocity_deg_s": ParameterValue(
                    LaunchConfiguration("observe_joint_velocity_deg_s"),
                    value_type=float,
                ),
                "observe_joint_acceleration_deg_s": ParameterValue(
                    LaunchConfiguration("observe_joint_acceleration_deg_s"),
                    value_type=float,
                ),
                "observe_joint_time_sec": ParameterValue(
                    LaunchConfiguration("observe_joint_time_sec"),
                    value_type=float,
                ),
                "observe_settle_sec": ParameterValue(
                    LaunchConfiguration("observe_settle_sec"),
                    value_type=float,
                ),
                "lid_aruco_dictionary": ParameterValue(
                    LaunchConfiguration("lid_aruco_dictionary"),
                    value_type=str,
                ),
                "lid_aruco_marker_id": ParameterValue(
                    LaunchConfiguration("lid_aruco_marker_id"),
                    value_type=int,
                ),
                "lid_aruco_marker_length_m": ParameterValue(
                    LaunchConfiguration("lid_aruco_marker_length_m"),
                    value_type=float,
                ),
                "lid_aruco_max_tilt_deg": ParameterValue(
                    LaunchConfiguration("lid_aruco_max_tilt_deg"),
                    value_type=float,
                ),
                "lid_aruco_detect_timeout_sec": ParameterValue(
                    LaunchConfiguration("lid_aruco_detect_timeout_sec"),
                    value_type=float,
                ),
                "lid_aruco_detect_poll_sec": ParameterValue(
                    LaunchConfiguration("lid_aruco_detect_poll_sec"),
                    value_type=float,
                ),
                "lid_pick_tcp_z_offset_m": ParameterValue(
                    LaunchConfiguration("lid_pick_tcp_z_offset_m"),
                    value_type=float,
                ),
                "lid_pick_approach_lift_m": ParameterValue(
                    LaunchConfiguration("lid_pick_approach_lift_m"),
                    value_type=float,
                ),
                "lid_pick_yaw_offset_deg": ParameterValue(
                    LaunchConfiguration("lid_pick_yaw_offset_deg"),
                    value_type=float,
                ),
                "lid_holder_tcp_z_m": ParameterValue(
                    LaunchConfiguration("lid_holder_tcp_z_m"),
                    value_type=float,
                ),
                "lid_holder_x_offset_m": ParameterValue(
                    LaunchConfiguration("lid_holder_x_offset_m"),
                    value_type=float,
                ),
                "lid_holder_y_offset_m": ParameterValue(
                    LaunchConfiguration("lid_holder_y_offset_m"),
                    value_type=float,
                ),
                "lid_holder_approach_lift_m": ParameterValue(
                    LaunchConfiguration("lid_holder_approach_lift_m"),
                    value_type=float,
                ),
                "lid_holder_yaw_offset_deg": ParameterValue(
                    LaunchConfiguration("lid_holder_yaw_offset_deg"),
                    value_type=float,
                ),
                "require_cup_pose_at_holder": ParameterValue(
                    LaunchConfiguration("require_cup_pose_at_holder"),
                    value_type=bool,
                ),
                "cup_pose_max_age_sec": ParameterValue(
                    LaunchConfiguration("cup_pose_max_age_sec"),
                    value_type=float,
                ),
                "cup_pose_wait_timeout_sec": ParameterValue(
                    LaunchConfiguration("cup_pose_wait_timeout_sec"),
                    value_type=float,
                ),
                "cup_pose_max_xy_error_m": ParameterValue(
                    LaunchConfiguration("cup_pose_max_xy_error_m"),
                    value_type=float,
                ),
                "aruco_redetect_settle_sec": ParameterValue(
                    LaunchConfiguration("aruco_redetect_settle_sec"),
                    value_type=float,
                ),
                "regrip_max_xy_error_m": ParameterValue(
                    LaunchConfiguration("regrip_max_xy_error_m"),
                    value_type=float,
                ),
                "screw_cycles": ParameterValue(
                    LaunchConfiguration("screw_cycles"),
                    value_type=int,
                ),
                "screw_turn_deg": ParameterValue(
                    LaunchConfiguration("screw_turn_deg"),
                    value_type=float,
                ),
                "screw_turn_direction": ParameterValue(
                    LaunchConfiguration("screw_turn_direction"),
                    value_type=float,
                ),
                "screw_motion_backend": ParameterValue(
                    LaunchConfiguration("screw_motion_backend"),
                    value_type=str,
                ),
                "screw_joint_velocity_scale": ParameterValue(
                    LaunchConfiguration("screw_joint_velocity_scale"),
                    value_type=float,
                ),
                "screw_joint_acceleration_scale": ParameterValue(
                    LaunchConfiguration("screw_joint_acceleration_scale"),
                    value_type=float,
                ),
                "screw_move_joint_velocity_deg_s": ParameterValue(
                    LaunchConfiguration("screw_move_joint_velocity_deg_s"),
                    value_type=float,
                ),
                "screw_move_joint_acceleration_deg_s": ParameterValue(
                    LaunchConfiguration("screw_move_joint_acceleration_deg_s"),
                    value_type=float,
                ),
                "screw_move_joint_time_sec": ParameterValue(
                    LaunchConfiguration("screw_move_joint_time_sec"),
                    value_type=float,
                ),
                "trajectory_action_name": ParameterValue(
                    LaunchConfiguration("trajectory_action_name"),
                    value_type=str,
                ),
                "trajectory_action_wait_sec": ParameterValue(
                    LaunchConfiguration("trajectory_action_wait_sec"),
                    value_type=float,
                ),
                "trajectory_execution_timeout_sec": ParameterValue(
                    LaunchConfiguration("trajectory_execution_timeout_sec"),
                    value_type=float,
                ),
                "max_single_segment_joint_motion_deg": ParameterValue(
                    LaunchConfiguration("max_single_segment_joint_motion_deg"),
                    value_type=float,
                ),
                "move_joint_service_timeout_sec": ParameterValue(
                    LaunchConfiguration("move_joint_service_timeout_sec"),
                    value_type=float,
                ),
                "move_joint_wait_timeout_sec": ParameterValue(
                    LaunchConfiguration("move_joint_wait_timeout_sec"),
                    value_type=float,
                ),
                "move_joint_verify_tolerance_deg": ParameterValue(
                    LaunchConfiguration("move_joint_verify_tolerance_deg"),
                    value_type=float,
                ),
                "joint6_raw_min_deg": ParameterValue(
                    LaunchConfiguration("joint6_raw_min_deg"),
                    value_type=float,
                ),
                "joint6_raw_max_deg": ParameterValue(
                    LaunchConfiguration("joint6_raw_max_deg"),
                    value_type=float,
                ),
                "gripper_set_service": ParameterValue(
                    LaunchConfiguration("gripper_set_service"),
                    value_type=str,
                ),
                "lid_gripper_close_width_m": ParameterValue(
                    LaunchConfiguration("lid_gripper_close_width_m"),
                    value_type=float,
                ),
                "lid_gripper_release_width_m": ParameterValue(
                    LaunchConfiguration("lid_gripper_release_width_m"),
                    value_type=float,
                ),
                "lid_gripper_force_n": ParameterValue(
                    LaunchConfiguration("lid_gripper_force_n"),
                    value_type=float,
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("trigger_service", default_value="/azas/lid_screw_on_holder/run"),
            DeclareLaunchArgument("status_topic", default_value="/azas/lid_screw_on_holder/status"),
            DeclareLaunchArgument("start_joint_state_relay", default_value="true"),
            DeclareLaunchArgument("joint_state_relay_input_topic", default_value="/dsr01/joint_states"),
            DeclareLaunchArgument("joint_state_relay_output_topic", default_value="/joint_states"),
            DeclareLaunchArgument("execute_motion", default_value="false"),
            DeclareLaunchArgument("hardware_confirm", default_value=""),
            DeclareLaunchArgument("shutdown_on_complete", default_value="false"),
            DeclareLaunchArgument("service_prefix", default_value="/dsr01"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
            DeclareLaunchArgument("color_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument("cup_pose_topic", default_value="/jarvis/tumbler_dispenser/tumbler_pose"),
            DeclareLaunchArgument("raw_joint_state_topic", default_value="/dsr01/joint_states"),
            DeclareLaunchArgument(
                "calibration_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "config", "calibration.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "safety_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "config", "safety.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "hand_eye_matrix_path",
                default_value="/home/ssu/Azas/src/azas_perception/config/T_gripper2camera.npy",
            ),
            DeclareLaunchArgument("safety_workspace_enforced", default_value="true"),
            DeclareLaunchArgument("move_to_observe_before_detect", default_value="true"),
            DeclareLaunchArgument("observe_motion_backend", default_value="move_joint"),
            DeclareLaunchArgument("observe_joint_1_deg", default_value="3.0"),
            DeclareLaunchArgument("observe_joint_2_deg", default_value="-12.7"),
            DeclareLaunchArgument("observe_joint_3_deg", default_value="44.0"),
            DeclareLaunchArgument("observe_joint_4_deg", default_value="-9.0"),
            DeclareLaunchArgument("observe_joint_5_deg", default_value="133.0"),
            DeclareLaunchArgument("observe_joint_6_deg", default_value="90.0"),
            DeclareLaunchArgument("observe_joint_velocity_deg_s", default_value="18.0"),
            DeclareLaunchArgument("observe_joint_acceleration_deg_s", default_value="22.0"),
            DeclareLaunchArgument("observe_joint_time_sec", default_value="0.0"),
            DeclareLaunchArgument("observe_settle_sec", default_value="0.5"),
            DeclareLaunchArgument("lid_aruco_dictionary", default_value="DICT_6X6_250"),
            DeclareLaunchArgument("lid_aruco_marker_id", default_value="-1"),
            DeclareLaunchArgument("lid_aruco_marker_length_m", default_value="0.0"),
            DeclareLaunchArgument("lid_aruco_max_tilt_deg", default_value="40.0"),
            DeclareLaunchArgument("lid_aruco_detect_timeout_sec", default_value="2.0"),
            DeclareLaunchArgument("lid_aruco_detect_poll_sec", default_value="0.10"),
            DeclareLaunchArgument("lid_pick_tcp_z_offset_m", default_value="0.0"),
            DeclareLaunchArgument("lid_pick_approach_lift_m", default_value="0.10"),
            DeclareLaunchArgument("lid_pick_yaw_offset_deg", default_value="0.0"),
            DeclareLaunchArgument("lid_holder_tcp_z_m", default_value="0.0"),
            DeclareLaunchArgument("lid_holder_x_offset_m", default_value="0.0"),
            DeclareLaunchArgument("lid_holder_y_offset_m", default_value="0.0"),
            DeclareLaunchArgument("lid_holder_approach_lift_m", default_value="0.10"),
            DeclareLaunchArgument("lid_holder_yaw_offset_deg", default_value="0.0"),
            DeclareLaunchArgument("require_cup_pose_at_holder", default_value="true"),
            DeclareLaunchArgument("cup_pose_max_age_sec", default_value="2.0"),
            DeclareLaunchArgument("cup_pose_wait_timeout_sec", default_value="6.0"),
            DeclareLaunchArgument("cup_pose_max_xy_error_m", default_value="0.060"),
            DeclareLaunchArgument("aruco_redetect_settle_sec", default_value="0.50"),
            DeclareLaunchArgument("regrip_max_xy_error_m", default_value="0.040"),
            DeclareLaunchArgument("screw_cycles", default_value="2"),
            DeclareLaunchArgument("screw_turn_deg", default_value="180.0"),
            DeclareLaunchArgument("screw_turn_direction", default_value="1.0"),
            DeclareLaunchArgument("screw_motion_backend", default_value="move_joint"),
            DeclareLaunchArgument("screw_joint_velocity_scale", default_value="0.04"),
            DeclareLaunchArgument("screw_joint_acceleration_scale", default_value="0.03"),
            DeclareLaunchArgument("screw_move_joint_velocity_deg_s", default_value="18.0"),
            DeclareLaunchArgument("screw_move_joint_acceleration_deg_s", default_value="22.0"),
            DeclareLaunchArgument("screw_move_joint_time_sec", default_value="0.0"),
            DeclareLaunchArgument(
                "trajectory_action_name",
                default_value="/dsr01/dsr_moveit_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument("trajectory_action_wait_sec", default_value="20.0"),
            DeclareLaunchArgument("trajectory_execution_timeout_sec", default_value="90.0"),
            DeclareLaunchArgument("max_single_segment_joint_motion_deg", default_value="180.0"),
            DeclareLaunchArgument("move_joint_service_timeout_sec", default_value="60.0"),
            DeclareLaunchArgument("move_joint_wait_timeout_sec", default_value="60.0"),
            DeclareLaunchArgument("move_joint_verify_tolerance_deg", default_value="2.0"),
            DeclareLaunchArgument("joint6_raw_min_deg", default_value="-360.0"),
            DeclareLaunchArgument("joint6_raw_max_deg", default_value="360.0"),
            DeclareLaunchArgument("gripper_set_service", default_value="/jarvis/rg2/set_width"),
            DeclareLaunchArgument("lid_gripper_close_width_m", default_value="0.012"),
            DeclareLaunchArgument("lid_gripper_release_width_m", default_value="0.080"),
            DeclareLaunchArgument("lid_gripper_force_n", default_value="40.0"),
            joint_state_relay,
            node,
        ]
    )
