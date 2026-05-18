import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_file = os.path.join(
        get_package_share_directory("azas_dispenser"),
        "models",
        "dispenser",
        "model.sdf",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("name", default_value="azas_dispenser"),
            DeclareLaunchArgument("x", default_value="0.50"),
            DeclareLaunchArgument("y", default_value="0.00"),
            DeclareLaunchArgument("z", default_value="0.00"),
            DeclareLaunchArgument("yaw", default_value="0.00"),
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-file",
                    model_file,
                    "-name",
                    LaunchConfiguration("name"),
                    "-allow_renaming",
                    "true",
                    "-x",
                    LaunchConfiguration("x"),
                    "-y",
                    LaunchConfiguration("y"),
                    "-z",
                    LaunchConfiguration("z"),
                    "-Y",
                    LaunchConfiguration("yaw"),
                ],
            ),
        ]
    )
