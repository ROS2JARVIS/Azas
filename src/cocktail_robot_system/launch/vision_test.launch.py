# Role: Launch the YOLO vision node with package parameters.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


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
                parameters=[params_file, {"debug": ParameterValue(debug, value_type=bool)}],
            ),
        ]
    )
