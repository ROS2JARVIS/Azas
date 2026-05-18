from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/detect_grasps/grasps"),
            DeclareLaunchArgument("output_topic", default_value="/azas/gpd/grasp_pose"),
            DeclareLaunchArgument("min_score", default_value="0.0"),
            DeclareLaunchArgument("min_width_m", default_value="0.0"),
            DeclareLaunchArgument("max_width_m", default_value="0.12"),
            Node(
                package="azas_perception",
                executable="gpd_grasp_adapter_node",
                name="gpd_grasp_adapter_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "min_score": LaunchConfiguration("min_score"),
                        "min_width_m": LaunchConfiguration("min_width_m"),
                        "max_width_m": LaunchConfiguration("max_width_m"),
                    }
                ],
            ),
        ]
    )
