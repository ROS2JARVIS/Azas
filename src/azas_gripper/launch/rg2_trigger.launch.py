from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("ip", default_value="192.168.1.1"),
            DeclareLaunchArgument("port", default_value="502"),
            DeclareLaunchArgument("connect", default_value="true"),
            DeclareLaunchArgument("gripper", default_value="rg2"),
            DeclareLaunchArgument("open_width", default_value="1100"),
            DeclareLaunchArgument("close_width", default_value="0"),
            DeclareLaunchArgument("force", default_value="300"),
            DeclareLaunchArgument("settle_seconds", default_value="0.6"),
            Node(
                package="azas_gripper",
                executable="rg2_gripper_node",
                name="rg2_trigger_node",
                output="screen",
                parameters=[
                    {
                        "use_real_hardware": LaunchConfiguration("connect"),
                        "gripper": LaunchConfiguration("gripper"),
                        "host": LaunchConfiguration("ip"),
                        "port": LaunchConfiguration("port"),
                        "default_open_width_m": 0.110,
                        "default_close_width_m": 0.0,
                        "default_force_n": 30.0,
                        "open_service": "/jarvis/rg2/open",
                        "close_service": "/jarvis/rg2/close",
                        "set_width_service": "/jarvis/rg2/set_width",
                    }
                ],
            ),
        ]
    )
