from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    frame_id = LaunchConfiguration("frame_id")
    robot_color = LaunchConfiguration("robot_color")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    show_workspace_safety = LaunchConfiguration("show_workspace_safety")
    show_measured_dispenser_collision = LaunchConfiguration("show_measured_dispenser_collision")
    show_full_collision_scene = LaunchConfiguration("show_full_collision_scene")
    show_link6_gripper = LaunchConfiguration("show_link6_gripper")
    publish_workspace_collision_objects = LaunchConfiguration("publish_workspace_collision_objects")
    publish_dispenser_collision_objects = LaunchConfiguration("publish_dispenser_collision_objects")
    safety_config_path = LaunchConfiguration("safety_config_path")
    dispenser_collision_config_path = LaunchConfiguration("dispenser_collision_config_path")
    calibration_path = LaunchConfiguration("calibration_path")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare("dsr_description2"), "xacro", "m0609.urdf.xacro"]
                    ),
                    " color:=",
                    robot_color,
                    " simple:=true",
                ]
            ),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("robot_color", default_value="white"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("show_workspace_safety", default_value="true"),
            DeclareLaunchArgument("show_measured_dispenser_collision", default_value="true"),
            DeclareLaunchArgument("show_full_collision_scene", default_value="true"),
            DeclareLaunchArgument("show_link6_gripper", default_value="true"),
            DeclareLaunchArgument("publish_workspace_collision_objects", default_value="true"),
            DeclareLaunchArgument("publish_dispenser_collision_objects", default_value="true"),
            DeclareLaunchArgument(
                "safety_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "config", "safety.yaml"]
                ),
            ),
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
                "calibration_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "config", "calibration.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "rviz", "azas_dispenser_sequence_clean.rviz"]
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="m0609_robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="azas_motion",
                executable="workspace_collision_scene_node",
                name="workspace_collision_scene_node",
                output="screen",
                condition=IfCondition(show_workspace_safety),
                parameters=[
                    {
                        "safety_config_path": ParameterValue(safety_config_path, value_type=str),
                        "publish_collision_objects": ParameterValue(
                            publish_workspace_collision_objects,
                            value_type=bool,
                        ),
                        "publish_period_sec": 2.0,
                        "frame_id": frame_id,
                        "table_collision_enabled": True,
                        "table_collision_expand_to_workspace_walls": True,
                        "workspace_boundary_collision_enabled": True,
                    }
                ],
            ),
            Node(
                package="azas_motion",
                executable="measured_dispenser_collision_scene_node",
                name="measured_dispenser_collision_scene_node",
                output="screen",
                condition=IfCondition(show_measured_dispenser_collision),
                parameters=[
                    {
                        "config_path": ParameterValue(
                            dispenser_collision_config_path, value_type=str
                        ),
                        "calibration_path": ParameterValue(calibration_path, value_type=str),
                        "publish_collision_objects": ParameterValue(
                            publish_dispenser_collision_objects,
                            value_type=bool,
                        ),
                        "publish_markers": True,
                        "publish_rviz_visual_tools_compat": False,
                        "publish_debug_labels": True,
                        "publish_period_sec": 2.0,
                    }
                ],
            ),
            Node(
                package="azas_bringup",
                executable="collision_scene_rviz_publisher",
                name="collision_scene_rviz_publisher",
                output="screen",
                condition=IfCondition(show_full_collision_scene),
                parameters=[
                    {
                        "frame_id": frame_id,
                        "safety_config_path": ParameterValue(safety_config_path, value_type=str),
                        "dispenser_collision_config_path": ParameterValue(
                            dispenser_collision_config_path, value_type=str
                        ),
                        "calibration_path": ParameterValue(calibration_path, value_type=str),
                    }
                ],
            ),
            Node(
                package="azas_motion",
                executable="link6_gripper_collision_node",
                name="link6_gripper_collision_node",
                output="screen",
                condition=IfCondition(show_link6_gripper),
                parameters=[
                    {
                        "attached_link_name": "link_6",
                        "publish_markers": True,
                        "marker_topic": "/azas/link6_gripper/markers",
                        "remove_on_shutdown": False,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
