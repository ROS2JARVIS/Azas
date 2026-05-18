from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    frame_id = LaunchConfiguration("frame_id")
    robot_color = LaunchConfiguration("robot_color")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("dsr_description2"), "xacro", "m0609.urdf.xacro"]
                ),
                " color:=",
                robot_color,
                " simple:=true",
            ]
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("robot_color", default_value="white"),
            DeclareLaunchArgument("target_x", default_value="0.43"),
            DeclareLaunchArgument("target_y", default_value="0.08"),
            DeclareLaunchArgument("target_z", default_value="0.135"),
            DeclareLaunchArgument("seed_current_state", default_value="true"),
            DeclareLaunchArgument("planning_pipeline", default_value="pilz_industrial_motion_planner"),
            DeclareLaunchArgument("planner_id", default_value="PTP"),
            DeclareLaunchArgument("planning_timeout_sec", default_value="3.0"),
            DeclareLaunchArgument("planning_attempts", default_value="1"),
            DeclareLaunchArgument("max_velocity_scaling_factor", default_value="0.1"),
            DeclareLaunchArgument("max_acceleration_scaling_factor", default_value="0.1"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "rviz", "target_xyz_moveit.rviz"]
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="m0609_robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="azas_motion",
                executable="target_xyz_moveit_preview_node",
                name="target_xyz_moveit_preview_node",
                output="screen",
                parameters=[
                    {
                        "frame_id": frame_id,
                        "target_x": ParameterValue(LaunchConfiguration("target_x"), value_type=float),
                        "target_y": ParameterValue(LaunchConfiguration("target_y"), value_type=float),
                        "target_z": ParameterValue(LaunchConfiguration("target_z"), value_type=float),
                        "seed_current_state": ParameterValue(
                            LaunchConfiguration("seed_current_state"), value_type=bool
                        ),
                        "planning_pipeline": LaunchConfiguration("planning_pipeline"),
                        "planner_id": LaunchConfiguration("planner_id"),
                        "planning_timeout_sec": ParameterValue(
                            LaunchConfiguration("planning_timeout_sec"), value_type=float
                        ),
                        "planning_attempts": ParameterValue(
                            LaunchConfiguration("planning_attempts"), value_type=int
                        ),
                        "max_velocity_scaling_factor": ParameterValue(
                            LaunchConfiguration("max_velocity_scaling_factor"), value_type=float
                        ),
                        "max_acceleration_scaling_factor": ParameterValue(
                            LaunchConfiguration("max_acceleration_scaling_factor"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
