from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    upstream_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("azas_cup_uprighting"),
                "launch",
                "yolo_cup_uprighting.launch.py",
            ])
        ),
        launch_arguments={
            "model_path": LaunchConfiguration("model_path"),
            "color_topic": LaunchConfiguration("color_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "debug_image_topic": LaunchConfiguration("debug_image_topic"),
            "preview_only": LaunchConfiguration("preview_only"),
            "show_window": LaunchConfiguration("show_window"),
            "use_mock_vision": LaunchConfiguration("use_mock_vision"),
            "max_velocity_scale": LaunchConfiguration("max_velocity_scale"),
            "max_acceleration_scale": LaunchConfiguration("max_acceleration_scale"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="/home/ssu/Azas/local_models/best.pt"),
        DeclareLaunchArgument("color_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("debug_image_topic", default_value="/yolo_cup_uprighting/debug_image"),
        DeclareLaunchArgument("preview_only", default_value="true"),
        DeclareLaunchArgument("show_window", default_value="false"),
        DeclareLaunchArgument("use_mock_vision", default_value="false"),
        DeclareLaunchArgument("max_velocity_scale", default_value="0.1"),
        DeclareLaunchArgument("max_acceleration_scale", default_value="0.1"),
        upstream_launch,
    ])
