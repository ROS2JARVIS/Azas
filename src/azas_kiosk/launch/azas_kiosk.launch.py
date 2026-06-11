from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8080"),
            DeclareLaunchArgument("stt_topic", default_value="/stt_result"),
            DeclareLaunchArgument(
                "cocktail_status_topic", default_value="/azas/cocktail/status"
            ),
            Node(
                package="azas_kiosk",
                executable="kiosk_node",
                name="azas_kiosk_node",
                output="screen",
                parameters=[
                    {
                        "host": LaunchConfiguration("host"),
                        "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                        "stt_topic": LaunchConfiguration("stt_topic"),
                        "cocktail_status_topic": LaunchConfiguration("cocktail_status_topic"),
                    }
                ],
            ),
        ]
    )
