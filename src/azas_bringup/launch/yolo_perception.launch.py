from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Defaults match the RealSense D435i namespace used by the current field
    # docs. Override these launch args if the camera driver is launched with a
    # different namespace; do not patch coordinates or frames in code.
    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="/home/ssu/Downloads/best.pt"),
        DeclareLaunchArgument("color_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.35"),
        DeclareLaunchArgument("target_class_names", default_value="cup,tumbler,bottle"),
        DeclareLaunchArgument("selection_policy", default_value="largest_bbox"),
        DeclareLaunchArgument("source_frame", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("depth_window_size", default_value="7"),
        DeclareLaunchArgument("min_depth_m", default_value="0.15"),
        DeclareLaunchArgument("max_depth_m", default_value="2.0"),
        DeclareLaunchArgument("device", default_value="cpu"),
        Node(
            package="azas_perception",
            executable="yolo_tumbler_detector_node",
            name="yolo_tumbler_detector_node",
            output="screen",
            parameters=[{
                "model_path": LaunchConfiguration("model_path"),
                "color_topic": LaunchConfiguration("color_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "confidence_threshold": ParameterValue(LaunchConfiguration("confidence_threshold"), value_type=float),
                "target_class_names": LaunchConfiguration("target_class_names"),
                "selection_policy": LaunchConfiguration("selection_policy"),
                "source_frame": LaunchConfiguration("source_frame"),
                "depth_window_size": ParameterValue(LaunchConfiguration("depth_window_size"), value_type=int),
                "min_depth_m": ParameterValue(LaunchConfiguration("min_depth_m"), value_type=float),
                "max_depth_m": ParameterValue(LaunchConfiguration("max_depth_m"), value_type=float),
                "device": LaunchConfiguration("device"),
            }],
        ),
    ])
