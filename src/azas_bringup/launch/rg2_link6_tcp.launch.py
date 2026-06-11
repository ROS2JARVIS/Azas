from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "urdf", "rg2_link6_tcp.urdf.xacro"]
                ),
                " open_tcp_offset_m:=",
                LaunchConfiguration("open_tcp_offset_m"),
                " closed_tcp_offset_m:=",
                LaunchConfiguration("closed_tcp_offset_m"),
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("open_tcp_offset_m", default_value="0.15"),
            DeclareLaunchArgument("closed_tcp_offset_m", default_value="0.25"),
            DeclareLaunchArgument("publish_gripper_collision", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="azas_rg2_link6_tcp_state_publisher",
                output="screen",
                remappings=[
                    ("robot_description", "/azas/rg2_link6_tcp/robot_description"),
                ],
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="azas_motion",
                executable="link6_gripper_collision_node",
                name="link6_gripper_collision_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_gripper_collision")),
            ),
        ]
    )
