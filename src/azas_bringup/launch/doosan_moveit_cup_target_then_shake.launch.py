from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    transport_z = LaunchConfiguration("transport_z")
    shake_center_z = LaunchConfiguration("shake_center_z")
    shake_amplitude_x = LaunchConfiguration("shake_amplitude_x")
    shake_amplitude_y = LaunchConfiguration("shake_amplitude_y")
    shake_cycles = LaunchConfiguration("shake_cycles")
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
        executable="doosan_moveit_cup_target_then_shake_node",
        name="doosan_moveit_cup_target_then_shake_node",
        output="screen",
        parameters=[
            {
                "start_delay_sec": ParameterValue(start_delay_sec, value_type=float),
                "execute_motion": ParameterValue(
                    LaunchConfiguration("execute_motion"), value_type=bool
                ),
                "move_to_initial_side_grip": ParameterValue(
                    LaunchConfiguration("move_to_initial_side_grip"), value_type=bool
                ),
                "move_to_detected_cup": ParameterValue(
                    LaunchConfiguration("move_to_detected_cup"), value_type=bool
                ),
                "cup_pose_topic": LaunchConfiguration("cup_pose_topic"),
                "cup_approach_z_offset": ParameterValue(
                    LaunchConfiguration("cup_approach_z_offset"), value_type=float
                ),
                "shake_mode": LaunchConfiguration("shake_mode"),
                "lift_before_safe_shake_space": ParameterValue(
                    LaunchConfiguration("lift_before_safe_shake_space"), value_type=bool
                ),
                "move_to_safe_shake_space": ParameterValue(
                    LaunchConfiguration("move_to_safe_shake_space"), value_type=bool
                ),
                "joint_1_deg": ParameterValue(LaunchConfiguration("joint_1_deg"), value_type=float),
                "joint_2_deg": ParameterValue(LaunchConfiguration("joint_2_deg"), value_type=float),
                "joint_3_deg": ParameterValue(LaunchConfiguration("joint_3_deg"), value_type=float),
                "joint_4_deg": ParameterValue(LaunchConfiguration("joint_4_deg"), value_type=float),
                "joint_5_deg": ParameterValue(LaunchConfiguration("joint_5_deg"), value_type=float),
                "joint_6_deg": ParameterValue(LaunchConfiguration("joint_6_deg"), value_type=float),
                "lift_joint_1_deg": ParameterValue(
                    LaunchConfiguration("lift_joint_1_deg"), value_type=float
                ),
                "lift_joint_2_deg": ParameterValue(
                    LaunchConfiguration("lift_joint_2_deg"), value_type=float
                ),
                "lift_joint_3_deg": ParameterValue(
                    LaunchConfiguration("lift_joint_3_deg"), value_type=float
                ),
                "lift_joint_4_deg": ParameterValue(
                    LaunchConfiguration("lift_joint_4_deg"), value_type=float
                ),
                "lift_joint_5_deg": ParameterValue(
                    LaunchConfiguration("lift_joint_5_deg"), value_type=float
                ),
                "lift_joint_6_deg": ParameterValue(
                    LaunchConfiguration("lift_joint_6_deg"), value_type=float
                ),
                "safe_joint_1_deg": ParameterValue(
                    LaunchConfiguration("safe_joint_1_deg"), value_type=float
                ),
                "safe_joint_2_deg": ParameterValue(
                    LaunchConfiguration("safe_joint_2_deg"), value_type=float
                ),
                "safe_joint_3_deg": ParameterValue(
                    LaunchConfiguration("safe_joint_3_deg"), value_type=float
                ),
                "safe_joint_4_deg": ParameterValue(
                    LaunchConfiguration("safe_joint_4_deg"), value_type=float
                ),
                "safe_joint_5_deg": ParameterValue(
                    LaunchConfiguration("safe_joint_5_deg"), value_type=float
                ),
                "safe_joint_6_deg": ParameterValue(
                    LaunchConfiguration("safe_joint_6_deg"), value_type=float
                ),
                "target_x": ParameterValue(target_x, value_type=float),
                "target_y": ParameterValue(target_y, value_type=float),
                "transport_z": ParameterValue(transport_z, value_type=float),
                "shake_center_z": ParameterValue(shake_center_z, value_type=float),
                "shake_amplitude_x": ParameterValue(shake_amplitude_x, value_type=float),
                "shake_amplitude_y": ParameterValue(shake_amplitude_y, value_type=float),
                "shake_cycles": ParameterValue(shake_cycles, value_type=int),
                "relative_lift_z": ParameterValue(
                    LaunchConfiguration("relative_lift_z"), value_type=float
                ),
                "safe_min_z": ParameterValue(LaunchConfiguration("safe_min_z"), value_type=float),
                "safe_max_z": ParameterValue(LaunchConfiguration("safe_max_z"), value_type=float),
                "use_fixed_safe_xy": ParameterValue(
                    LaunchConfiguration("use_fixed_safe_xy"), value_type=bool
                ),
                "planning_pipeline": LaunchConfiguration("planning_pipeline"),
                "planner_id": LaunchConfiguration("planner_id"),
                "max_velocity_scaling_factor": ParameterValue(
                    LaunchConfiguration("max_velocity_scaling_factor"), value_type=float
                ),
                "max_acceleration_scaling_factor": ParameterValue(
                    LaunchConfiguration("max_acceleration_scaling_factor"), value_type=float
                ),
                "controller_action_wait_sec": ParameterValue(
                    LaunchConfiguration("controller_action_wait_sec"), value_type=float
                ),
            }
        ],
    )

    measured_collision_scene = Node(
        package="azas_motion",
        executable="measured_dispenser_collision_scene_node",
        name="measured_dispenser_collision_scene_node",
        output="screen",
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
            DeclareLaunchArgument("move_to_initial_side_grip", default_value="false"),
            DeclareLaunchArgument("move_to_detected_cup", default_value="false"),
            DeclareLaunchArgument(
                "cup_pose_topic", default_value="/jarvis/tumbler_dispenser/tumbler_pose"
            ),
            DeclareLaunchArgument("cup_approach_z_offset", default_value="0.12"),
            DeclareLaunchArgument("shake_mode", default_value="relative_pose"),
            DeclareLaunchArgument("lift_before_safe_shake_space", default_value="true"),
            DeclareLaunchArgument("move_to_safe_shake_space", default_value="true"),
            DeclareLaunchArgument("joint_1_deg", default_value="119.0"),
            DeclareLaunchArgument("joint_2_deg", default_value="-41.0"),
            DeclareLaunchArgument("joint_3_deg", default_value="-120.0"),
            DeclareLaunchArgument("joint_4_deg", default_value="32.0"),
            DeclareLaunchArgument("joint_5_deg", default_value="-103.0"),
            DeclareLaunchArgument("joint_6_deg", default_value="-137.0"),
            DeclareLaunchArgument("lift_joint_1_deg", default_value="105.0"),
            DeclareLaunchArgument("lift_joint_2_deg", default_value="-62.0"),
            DeclareLaunchArgument("lift_joint_3_deg", default_value="-92.0"),
            DeclareLaunchArgument("lift_joint_4_deg", default_value="20.0"),
            DeclareLaunchArgument("lift_joint_5_deg", default_value="-82.0"),
            DeclareLaunchArgument("lift_joint_6_deg", default_value="-125.0"),
            DeclareLaunchArgument("safe_joint_1_deg", default_value="90.0"),
            DeclareLaunchArgument("safe_joint_2_deg", default_value="-72.0"),
            DeclareLaunchArgument("safe_joint_3_deg", default_value="-78.0"),
            DeclareLaunchArgument("safe_joint_4_deg", default_value="0.0"),
            DeclareLaunchArgument("safe_joint_5_deg", default_value="-72.0"),
            DeclareLaunchArgument("safe_joint_6_deg", default_value="-115.0"),
            DeclareLaunchArgument("target_x", default_value="0.43"),
            DeclareLaunchArgument("target_y", default_value="0.08"),
            DeclareLaunchArgument("transport_z", default_value="0.50"),
            DeclareLaunchArgument("shake_center_z", default_value="0.62"),
            DeclareLaunchArgument("shake_amplitude_x", default_value="0.03"),
            DeclareLaunchArgument("shake_amplitude_y", default_value="0.02"),
            DeclareLaunchArgument("shake_cycles", default_value="2"),
            DeclareLaunchArgument("relative_lift_z", default_value="0.25"),
            DeclareLaunchArgument("safe_min_z", default_value="0.55"),
            DeclareLaunchArgument("safe_max_z", default_value="0.85"),
            DeclareLaunchArgument("use_fixed_safe_xy", default_value="false"),
            DeclareLaunchArgument("planning_pipeline", default_value="pilz_industrial_motion_planner"),
            DeclareLaunchArgument("planner_id", default_value="PTP"),
            DeclareLaunchArgument("max_velocity_scaling_factor", default_value="0.10"),
            DeclareLaunchArgument("max_acceleration_scaling_factor", default_value="0.10"),
            DeclareLaunchArgument("controller_action_wait_sec", default_value="20.0"),
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
            DeclareLaunchArgument("collision_publish_period_sec", default_value="2.0"),
            measured_collision_scene,
            doosan_moveit,
            TimerAction(period=start_delay_sec, actions=[executor]),
        ]
    )
