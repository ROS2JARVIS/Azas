from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    model = LaunchConfiguration("model")
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    color = LaunchConfiguration("color")
    rt_host = LaunchConfiguration("rt_host")
    start_delay_sec = LaunchConfiguration("start_delay_sec")
    collision_config_path = LaunchConfiguration("collision_config_path")
    collision_publish_period_sec = LaunchConfiguration("collision_publish_period_sec")

    doosan_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dsr_bringup2"), "launch", "dsr_bringup2_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "mode": mode,
            "model": model,
            "host": host,
            "port": port,
            "color": color,
            "rt_host": rt_host,
        }.items(),
    )

    executor = Node(
        package="azas_motion",
        executable="doosan_moveit_grasped_tumbler_to_dispenser_node",
        name="doosan_moveit_grasped_tumbler_to_dispenser_node",
        output="screen",
        parameters=[
            {
                "start_delay_sec": ParameterValue(start_delay_sec, value_type=float),
                "execute_motion": ParameterValue(
                    LaunchConfiguration("execute_motion"), value_type=bool
                ),
                "selected_dispenser_id": ParameterValue(
                    LaunchConfiguration("selected_dispenser_id"), value_type=int
                ),
                "task_mode": LaunchConfiguration("task_mode"),
                "assume_already_at_side_grip": ParameterValue(
                    LaunchConfiguration("assume_already_at_side_grip"), value_type=bool
                ),
                "joint_1_deg": ParameterValue(LaunchConfiguration("joint_1_deg"), value_type=float),
                "joint_2_deg": ParameterValue(LaunchConfiguration("joint_2_deg"), value_type=float),
                "joint_3_deg": ParameterValue(LaunchConfiguration("joint_3_deg"), value_type=float),
                "joint_4_deg": ParameterValue(LaunchConfiguration("joint_4_deg"), value_type=float),
                "joint_5_deg": ParameterValue(LaunchConfiguration("joint_5_deg"), value_type=float),
                "joint_6_deg": ParameterValue(LaunchConfiguration("joint_6_deg"), value_type=float),
                "front_approach_offset_x": ParameterValue(
                    LaunchConfiguration("front_approach_offset_x"), value_type=float
                ),
                "outlet_front_offset_x": ParameterValue(
                    LaunchConfiguration("outlet_front_offset_x"), value_type=float
                ),
                "transfer_z_override": ParameterValue(
                    LaunchConfiguration("transfer_z_override"), value_type=float
                ),
                "enable_demo_obstacle": ParameterValue(
                    LaunchConfiguration("enable_demo_obstacle"), value_type=bool
                ),
                "enable_obstacle_detour": ParameterValue(
                    LaunchConfiguration("enable_obstacle_detour"), value_type=bool
                ),
                "detour_y": ParameterValue(LaunchConfiguration("detour_y"), value_type=float),
                "floor_target_x": ParameterValue(
                    LaunchConfiguration("floor_target_x"), value_type=float
                ),
                "floor_target_y": ParameterValue(
                    LaunchConfiguration("floor_target_y"), value_type=float
                ),
                "floor_target_z": ParameterValue(
                    LaunchConfiguration("floor_target_z"), value_type=float
                ),
                "floor_approach_z": ParameterValue(
                    LaunchConfiguration("floor_approach_z"), value_type=float
                ),
                "moveit_ready_wait_sec": ParameterValue(
                    LaunchConfiguration("moveit_ready_wait_sec"), value_type=float
                ),
                "controller_action_wait_sec": ParameterValue(
                    LaunchConfiguration("controller_action_wait_sec"), value_type=float
                ),
                "execution_backend": LaunchConfiguration("execution_backend"),
                "allow_dispenser_orientation_fallback": ParameterValue(
                    LaunchConfiguration("allow_dispenser_orientation_fallback"), value_type=bool
                ),
                "planning_pipeline": LaunchConfiguration("planning_pipeline"),
                "planner_id": LaunchConfiguration("planner_id"),
                "state_planner_id": LaunchConfiguration("state_planner_id"),
                "pose_planner_id": LaunchConfiguration("pose_planner_id"),
                "max_velocity_scaling_factor": ParameterValue(
                    LaunchConfiguration("max_velocity_scaling_factor"), value_type=float
                ),
                "max_acceleration_scaling_factor": ParameterValue(
                    LaunchConfiguration("max_acceleration_scaling_factor"), value_type=float
                ),
                "max_single_segment_joint_motion_deg": ParameterValue(
                    LaunchConfiguration("max_single_segment_joint_motion_deg"), value_type=float
                ),
            }
        ],
    )

    measured_collision_scene = Node(
        package="azas_motion",
        executable="measured_dispenser_collision_scene_node",
        name="measured_dispenser_collision_scene_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_measured_collision_scene")),
        parameters=[
            {
                "config_path": collision_config_path,
                "publish_period_sec": ParameterValue(
                    collision_publish_period_sec, value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="virtual"),
            DeclareLaunchArgument("model", default_value="m0609"),
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="12345"),
            DeclareLaunchArgument("color", default_value="white"),
            DeclareLaunchArgument("rt_host", default_value="192.168.137.50"),
            DeclareLaunchArgument("execute_motion", default_value="true"),
            DeclareLaunchArgument("start_delay_sec", default_value="14.0"),
            DeclareLaunchArgument("selected_dispenser_id", default_value="2"),
            DeclareLaunchArgument("task_mode", default_value="dispenser_front"),
            DeclareLaunchArgument("assume_already_at_side_grip", default_value="false"),
            DeclareLaunchArgument("joint_1_deg", default_value="159.0"),
            DeclareLaunchArgument("joint_2_deg", default_value="-43.0"),
            DeclareLaunchArgument("joint_3_deg", default_value="-105.0"),
            DeclareLaunchArgument("joint_4_deg", default_value="-81.0"),
            DeclareLaunchArgument("joint_5_deg", default_value="85.0"),
            DeclareLaunchArgument("joint_6_deg", default_value="31.0"),
            DeclareLaunchArgument("front_approach_offset_x", default_value="0.12"),
            DeclareLaunchArgument("outlet_front_offset_x", default_value="0.02"),
            DeclareLaunchArgument("transfer_z_override", default_value="0.20"),
            DeclareLaunchArgument("enable_demo_obstacle", default_value="true"),
            DeclareLaunchArgument("enable_obstacle_detour", default_value="true"),
            DeclareLaunchArgument("detour_y", default_value="-0.24"),
            DeclareLaunchArgument("floor_target_x", default_value="0.42"),
            DeclareLaunchArgument("floor_target_y", default_value="-0.22"),
            DeclareLaunchArgument("floor_target_z", default_value="0.20"),
            DeclareLaunchArgument("floor_approach_z", default_value="0.28"),
            DeclareLaunchArgument("execution_backend", default_value="controller_action"),
            DeclareLaunchArgument("moveit_ready_wait_sec", default_value="5.0"),
            DeclareLaunchArgument("controller_action_wait_sec", default_value="90.0"),
            DeclareLaunchArgument("allow_dispenser_orientation_fallback", default_value="true"),
            DeclareLaunchArgument("planning_pipeline", default_value="pilz_industrial_motion_planner"),
            DeclareLaunchArgument("planner_id", default_value="PTP"),
            DeclareLaunchArgument("state_planner_id", default_value="PTP"),
            DeclareLaunchArgument("pose_planner_id", default_value="LIN"),
            DeclareLaunchArgument("max_velocity_scaling_factor", default_value="0.10"),
            DeclareLaunchArgument("max_acceleration_scaling_factor", default_value="0.10"),
            DeclareLaunchArgument("max_single_segment_joint_motion_deg", default_value="170.0"),
            DeclareLaunchArgument("enable_measured_collision_scene", default_value="true"),
            DeclareLaunchArgument(
                "collision_config_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("azas_bringup"),
                        "config",
                        "measured_dispenser_collision.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("collision_publish_period_sec", default_value="1.0"),
            doosan_moveit,
            measured_collision_scene,
            TimerAction(period=start_delay_sec, actions=[executor]),
        ]
    )
