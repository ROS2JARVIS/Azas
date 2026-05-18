# Role: Launch vision, 3D estimation, and pre-grasp move test nodes together.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    debug = LaunchConfiguration("debug")

    default_params_file = PathJoinSubstitution(
        [FindPackageShare("cocktail_robot_system"), "config", "params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Path to the YAML parameter file.",
            ),
            DeclareLaunchArgument(
                "debug",
                default_value="true",
                description="Enable debug image publishing.",
            ),
            Node(
                package="cocktail_robot_system",
                executable="vision_node",
                name="vision_node",
                output="screen",
                parameters=[params_file, {"debug": debug}],
            ),
            Node(
                package="cocktail_robot_system",
                executable="detection_3d_node",
                name="detection_3d_node",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="cocktail_robot_system",
                executable="robot_move_test",
                name="robot_move_test",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
