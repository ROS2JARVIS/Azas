import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


COLOR_SCAN_JOINTS_RAD = [
    0.0,
    math.radians(10.0),
    math.radians(32.0),
    0.0,
    math.radians(100.0),
    math.radians(90.0),
]


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    preview_mode = LaunchConfiguration("preview_mode")
    rviz_config = LaunchConfiguration("rviz_config")

    collision_scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("azas_bringup"), "launch", "workspace_collision_scene.launch.py"]
            )
        )
    )
    gripper_tcp_tree = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("azas_bringup"), "launch", "rg2_link6_tcp.launch.py"]
            )
        )
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare("dsr_description2"), "xacro", "m0609.urdf.xacro"]
                    ),
                    " color:=white simple:=true",
                ]
            ),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="m0609_color_scan_pose_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    color_scan_joint_state = Node(
        package="azas_motion",
        executable="m0609_shake_joint_state_node",
        name="m0609_color_scan_pose_joint_state_node",
        output="screen",
        parameters=[
            {
                "preview_mode": preview_mode,
                "loop_motion": False,
                "home_joints_rad": COLOR_SCAN_JOINTS_RAD,
            }
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("preview_mode", default_value="color_scan_pose_move"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "rviz", "color_scan_pose.rviz"]
                ),
            ),
            collision_scene,
            gripper_tcp_tree,
            robot_state_publisher,
            color_scan_joint_state,
            rviz_node,
        ]
    )
