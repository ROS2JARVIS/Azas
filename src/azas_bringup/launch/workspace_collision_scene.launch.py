from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    safety_config_path = LaunchConfiguration("safety_config_path")
    publish_period_sec = LaunchConfiguration("publish_period_sec")
    dispenser_collision_config_path = LaunchConfiguration(
        "dispenser_collision_config_path"
    )
    dispenser_collision_publish_period_sec = LaunchConfiguration(
        "dispenser_collision_publish_period_sec"
    )

    workspace_collision_scene = Node(
        package="azas_motion",
        executable="workspace_collision_scene_node",
        name="workspace_collision_scene_node",
        output="screen",
        parameters=[
            {
                "safety_config_path": ParameterValue(
                    safety_config_path,
                    value_type=str,
                ),
                "publish_period_sec": ParameterValue(
                    publish_period_sec,
                    value_type=float,
                ),
                "publish_collision_objects": ParameterValue(
                    LaunchConfiguration("publish_collision_objects"),
                    value_type=bool,
                ),
                "frame_id": ParameterValue(LaunchConfiguration("frame_id"), value_type=str),
                "table_collision_enabled": ParameterValue(
                    LaunchConfiguration("table_collision_enabled"),
                    value_type=bool,
                ),
                "table_surface_z": ParameterValue(
                    LaunchConfiguration("table_surface_z"),
                    value_type=float,
                ),
                "table_thickness": ParameterValue(
                    LaunchConfiguration("table_thickness"),
                    value_type=float,
                ),
                "table_collision_expand_to_workspace_walls": ParameterValue(
                    LaunchConfiguration("table_collision_expand_to_workspace_walls"),
                    value_type=bool,
                ),
                "workspace_boundary_collision_enabled": ParameterValue(
                    LaunchConfiguration("workspace_boundary_collision_enabled"),
                    value_type=bool,
                ),
                "workspace_boundary_wall_thickness": ParameterValue(
                    LaunchConfiguration("workspace_boundary_wall_thickness"),
                    value_type=float,
                ),
                "workspace_boundary_wall_clearance": ParameterValue(
                    LaunchConfiguration("workspace_boundary_wall_clearance"),
                    value_type=float,
                ),
            }
        ],
    )

    dispenser_collision_scene = Node(
        package="azas_motion",
        executable="measured_dispenser_collision_scene_node",
        name="measured_dispenser_collision_scene_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("dispenser_collision_enabled")),
        parameters=[
            {
                "config_path": ParameterValue(
                    dispenser_collision_config_path,
                    value_type=str,
                ),
                "publish_period_sec": ParameterValue(
                    dispenser_collision_publish_period_sec,
                    value_type=float,
                ),
                "publish_collision_objects": ParameterValue(
                    LaunchConfiguration("dispenser_collision_publish_objects"),
                    value_type=bool,
                ),
                "publish_markers": ParameterValue(
                    LaunchConfiguration("dispenser_collision_publish_markers"),
                    value_type=bool,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "safety_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "config", "safety.yaml"]
                ),
            ),
            DeclareLaunchArgument("publish_period_sec", default_value="2.0"),
            DeclareLaunchArgument("publish_collision_objects", default_value="true"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("table_collision_enabled", default_value="true"),
            DeclareLaunchArgument("table_surface_z", default_value="0.0"),
            DeclareLaunchArgument("table_thickness", default_value="0.04"),
            DeclareLaunchArgument(
                "table_collision_expand_to_workspace_walls",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "workspace_boundary_collision_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "workspace_boundary_wall_thickness",
                default_value="0.04",
            ),
            DeclareLaunchArgument(
                "workspace_boundary_wall_clearance",
                default_value="0.02",
            ),
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
            DeclareLaunchArgument(
                "dispenser_collision_publish_period_sec",
                default_value="2.0",
            ),
            DeclareLaunchArgument(
                "dispenser_collision_publish_objects",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "dispenser_collision_publish_markers",
                default_value="true",
            ),
            workspace_collision_scene,
            dispenser_collision_scene,
        ]
    )
