from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    model_path = PathJoinSubstitution(
        [FindPackageShare("azas_description"), "urdf", "m0609_rg2_parametric.urdf.xacro"]
    )

    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model_path,
                " ",
                "rg2_parent_link:=",
                LaunchConfiguration("rg2_parent_link"),
                " ",
                "rg2_mount_xyz:=",
                LaunchConfiguration("rg2_mount_xyz"),
                " ",
                "rg2_mount_rpy:=",
                LaunchConfiguration("rg2_mount_rpy"),
                " ",
                "rg2_tcp_z_offset:=",
                LaunchConfiguration("rg2_tcp_z_offset"),
                " ",
                "gripper_tcp_link_name:=",
                LaunchConfiguration("gripper_tcp_link_name"),
            ]
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("rg2_parent_link", default_value="tool0"),
            DeclareLaunchArgument("rg2_mount_xyz", default_value="0 0 0"),
            DeclareLaunchArgument("rg2_mount_rpy", default_value="1.570796327 0 1.570796327"),
            DeclareLaunchArgument("rg2_tcp_z_offset", default_value="0.213"),
            DeclareLaunchArgument("gripper_tcp_link_name", default_value="gripper_tcp"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
            ),
        ]
    )
