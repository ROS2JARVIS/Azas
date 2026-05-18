# Role: Launch vision, 3D detection, and keyboard-triggered real lid pick/place.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    debug = LaunchConfiguration("debug")
    execute_motion = LaunchConfiguration("execute_motion")
    use_real_robot = LaunchConfiguration("use_real_robot")

    default_params_file = PathJoinSubstitution(
        [FindPackageShare("cocktail_robot_system"), "config", "params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Path to the YAML parameter file.",
            ),
            DeclareLaunchArgument(
                "debug",
                default_value="true",
                description="Enable YOLO debug image publishing.",
            ),
            DeclareLaunchArgument(
                "use_real_robot",
                default_value="true",
                description="Connect to Doosan real robot ROS services.",
            ),
            DeclareLaunchArgument(
                "execute_motion",
                default_value="false",
                description="Actually send robot/gripper commands when 'p' is pressed.",
            ),
            Node(
                package="cocktail_robot_system",
                executable="vision_node",
                name="vision_node",
                output="screen",
                parameters=[params_file, {"debug": debug}],
            ),
            Node(
                package="cocktail_robot_system",
                executable="detection_3d_node",
                name="detection_3d_node",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="cocktail_robot_system",
                executable="lid_pick_place_keyboard",
                name="lid_pick_place_keyboard",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_real_robot": use_real_robot,
                        "execute_motion": execute_motion,
                    },
                ],
            ),
        ]
    )
