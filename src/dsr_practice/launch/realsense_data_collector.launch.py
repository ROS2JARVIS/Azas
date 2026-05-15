from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="/home/ssu/ros2_ws/realsense_dataset/raw",
        description="Directory where color, depth, and metadata files are saved.",
    )
    save_interval_arg = DeclareLaunchArgument(
        "save_interval_sec",
        default_value="1.0",
        description="Seconds between saved frames.",
    )
    max_frames_arg = DeclareLaunchArgument(
        "max_frames",
        default_value="0",
        description="Maximum number of frames to save. 0 means unlimited.",
    )
    save_depth_arg = DeclareLaunchArgument(
        "save_depth",
        default_value="true",
        description="Whether to save aligned depth images with RGB images.",
    )

    collector_node = Node(
        package="dsr_practice",
        executable="realsense_data_collector",
        name="realsense_data_collector",
        output="screen",
        parameters=[
            {
                "output_dir": LaunchConfiguration("output_dir"),
                "save_interval_sec": LaunchConfiguration("save_interval_sec"),
                "max_frames": LaunchConfiguration("max_frames"),
                "save_depth": LaunchConfiguration("save_depth"),
            }
        ],
    )

    return LaunchDescription(
        [
            output_dir_arg,
            save_interval_arg,
            max_frames_arg,
            save_depth_arg,
            collector_node,
        ]
    )
