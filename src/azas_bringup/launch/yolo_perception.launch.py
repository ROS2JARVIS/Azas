from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Defaults match the RealSense D435i namespace used by the current field
    # docs. Override these launch args if the camera driver is launched with a
    # different namespace; do not patch coordinates or frames in code.
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("azas_perception"), "config", "yolo_cup_uprighting_best.pt"]
            ),
        ),
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
        DeclareLaunchArgument("capture_empty_table_baseline", default_value="false"),
        DeclareLaunchArgument("baseline_frame_count", default_value="30"),
        DeclareLaunchArgument("empty_table_baseline_path", default_value=""),
        DeclareLaunchArgument("cup_standing_height_threshold_m", default_value="0.0"),
        DeclareLaunchArgument("cup_side_lie_height_threshold_m", default_value="0.0"),
        DeclareLaunchArgument("cup_inverted_center_ratio_threshold", default_value="0.0"),
        DeclareLaunchArgument("cup_inverted_min_center_height_m", default_value="0.0"),
        DeclareLaunchArgument("enable_top_view_upright", default_value="false"),
        DeclareLaunchArgument("top_view_aspect_min", default_value="0.85"),
        DeclareLaunchArgument("top_view_aspect_max", default_value="1.15"),
        DeclareLaunchArgument("top_view_guard_aspect_min", default_value="0.70"),
        DeclareLaunchArgument("top_view_guard_aspect_max", default_value="1.35"),
        DeclareLaunchArgument("height_stat_for_orientation", default_value="p90"),
        DeclareLaunchArgument("min_height_valid_ratio", default_value="0.10"),
        DeclareLaunchArgument("orientation_classifier_path", default_value=""),
        DeclareLaunchArgument("orientation_classifier_arch", default_value="cnn"),
        DeclareLaunchArgument("orientation_classifier_min_confidence", default_value="0.70"),
        DeclareLaunchArgument("orientation_classifier_device", default_value="cpu"),
        DeclareLaunchArgument("orientation_classifier_pad", default_value="0.25"),
        DeclareLaunchArgument("orientation_classifier_tall_lie_aspect", default_value="1.35"),
        DeclareLaunchArgument("orientation_classifier_tall_lie_height_threshold_m", default_value="0.09"),
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
                "capture_empty_table_baseline": ParameterValue(LaunchConfiguration("capture_empty_table_baseline"), value_type=bool),
                "baseline_frame_count": ParameterValue(LaunchConfiguration("baseline_frame_count"), value_type=int),
                "empty_table_baseline_path": LaunchConfiguration("empty_table_baseline_path"),
                "cup_standing_height_threshold_m": ParameterValue(LaunchConfiguration("cup_standing_height_threshold_m"), value_type=float),
                "cup_side_lie_height_threshold_m": ParameterValue(LaunchConfiguration("cup_side_lie_height_threshold_m"), value_type=float),
                "cup_inverted_center_ratio_threshold": ParameterValue(LaunchConfiguration("cup_inverted_center_ratio_threshold"), value_type=float),
                "cup_inverted_min_center_height_m": ParameterValue(LaunchConfiguration("cup_inverted_min_center_height_m"), value_type=float),
                "enable_top_view_upright": ParameterValue(LaunchConfiguration("enable_top_view_upright"), value_type=bool),
                "top_view_aspect_min": ParameterValue(LaunchConfiguration("top_view_aspect_min"), value_type=float),
                "top_view_aspect_max": ParameterValue(LaunchConfiguration("top_view_aspect_max"), value_type=float),
                "top_view_guard_aspect_min": ParameterValue(LaunchConfiguration("top_view_guard_aspect_min"), value_type=float),
                "top_view_guard_aspect_max": ParameterValue(LaunchConfiguration("top_view_guard_aspect_max"), value_type=float),
                "height_stat_for_orientation": LaunchConfiguration("height_stat_for_orientation"),
                "min_height_valid_ratio": ParameterValue(LaunchConfiguration("min_height_valid_ratio"), value_type=float),
                "orientation_classifier_path": LaunchConfiguration("orientation_classifier_path"),
                "orientation_classifier_arch": LaunchConfiguration("orientation_classifier_arch"),
                "orientation_classifier_min_confidence": ParameterValue(LaunchConfiguration("orientation_classifier_min_confidence"), value_type=float),
                "orientation_classifier_device": LaunchConfiguration("orientation_classifier_device"),
                "orientation_classifier_pad": ParameterValue(LaunchConfiguration("orientation_classifier_pad"), value_type=float),
                "orientation_classifier_tall_lie_aspect": ParameterValue(LaunchConfiguration("orientation_classifier_tall_lie_aspect"), value_type=float),
                "orientation_classifier_tall_lie_height_threshold_m": ParameterValue(LaunchConfiguration("orientation_classifier_tall_lie_height_threshold_m"), value_type=float),
                "device": LaunchConfiguration("device"),
            }],
        ),
    ])
