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
            "workspace_collision_scene_enabled": LaunchConfiguration(
                "workspace_collision_scene_enabled"
            ),
            "workspace_collision_publish_period_sec": LaunchConfiguration(
                "workspace_collision_publish_period_sec"
            ),
            "table_collision_enabled": LaunchConfiguration("table_collision_enabled"),
            "table_surface_z": LaunchConfiguration("table_surface_z"),
            "table_thickness": LaunchConfiguration("table_thickness"),
            "table_size_x": LaunchConfiguration("table_size_x"),
            "table_size_y": LaunchConfiguration("table_size_y"),
            "table_center_x": LaunchConfiguration("table_center_x"),
            "table_center_y": LaunchConfiguration("table_center_y"),
            "table_collision_expand_to_workspace_walls": LaunchConfiguration(
                "table_collision_expand_to_workspace_walls"
            ),
            "safety_config_path": LaunchConfiguration("safety_config_path"),
            "workspace_boundary_collision_enabled": LaunchConfiguration(
                "workspace_boundary_collision_enabled"
            ),
            "workspace_boundary_collision_prefix": LaunchConfiguration(
                "workspace_boundary_collision_prefix"
            ),
            "workspace_boundary_wall_thickness": LaunchConfiguration(
                "workspace_boundary_wall_thickness"
            ),
            "workspace_boundary_wall_clearance": LaunchConfiguration(
                "workspace_boundary_wall_clearance"
            ),
            "dispenser_collision_enabled": LaunchConfiguration(
                "dispenser_collision_enabled"
            ),
            "dispenser_collision_config_path": LaunchConfiguration(
                "dispenser_collision_config_path"
            ),
            "dispenser_collision_publish_period_sec": LaunchConfiguration(
                "dispenser_collision_publish_period_sec"
            ),
            "dispenser_collision_publish_objects": LaunchConfiguration(
                "dispenser_collision_publish_objects"
            ),
            "dispenser_collision_publish_markers": LaunchConfiguration(
                "dispenser_collision_publish_markers"
            ),
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
        DeclareLaunchArgument("workspace_collision_scene_enabled", default_value="true"),
        DeclareLaunchArgument("workspace_collision_publish_period_sec", default_value="2.0"),
        DeclareLaunchArgument("table_collision_enabled", default_value="true"),
        DeclareLaunchArgument("table_surface_z", default_value="0.0"),
        DeclareLaunchArgument("table_thickness", default_value="0.04"),
        DeclareLaunchArgument("table_size_x", default_value="1.20"),
        DeclareLaunchArgument("table_size_y", default_value="1.00"),
        DeclareLaunchArgument("table_center_x", default_value="0.45"),
        DeclareLaunchArgument("table_center_y", default_value="0.0"),
        DeclareLaunchArgument("table_collision_expand_to_workspace_walls", default_value="true"),
        DeclareLaunchArgument(
            "safety_config_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("azas_bringup"), "config", "safety.yaml"]
            ),
        ),
        DeclareLaunchArgument("workspace_boundary_collision_enabled", default_value="true"),
        DeclareLaunchArgument(
            "workspace_boundary_collision_prefix",
            default_value="side_grip_workspace",
        ),
        DeclareLaunchArgument("workspace_boundary_wall_thickness", default_value="0.04"),
        DeclareLaunchArgument("workspace_boundary_wall_clearance", default_value="0.02"),
        DeclareLaunchArgument("dispenser_collision_enabled", default_value="true"),
        DeclareLaunchArgument(
            "dispenser_collision_config_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("azas_bringup"),
                    "config",
                    "measured_dispenser_collision.yaml",
                ]
            ),
        ),
        DeclareLaunchArgument("dispenser_collision_publish_period_sec", default_value="1.0"),
        DeclareLaunchArgument("dispenser_collision_publish_objects", default_value="true"),
        DeclareLaunchArgument("dispenser_collision_publish_markers", default_value="true"),
        upstream_launch,
    ])
