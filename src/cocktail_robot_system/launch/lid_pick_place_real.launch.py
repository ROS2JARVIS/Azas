# Role: Launch vision, 3D detection, and keyboard-triggered real lid pick/place.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    debug = LaunchConfiguration("debug")
    execute_motion = LaunchConfiguration("execute_motion")
    use_real_robot = LaunchConfiguration("use_real_robot")
    hand_eye_mode = LaunchConfiguration("hand_eye_mode")
    gripper_to_camera_matrix_path = LaunchConfiguration(
        "gripper_to_camera_matrix_path"
    )

    default_params_file = PathJoinSubstitution(
        [FindPackageShare("cocktail_robot_system"), "config", "params.yaml"]
    )
    default_gripper_to_camera_matrix_path = PathJoinSubstitution(
        [
            FindPackageShare("cocktail_robot_system"),
            "config",
            "calibration",
            "T_gripper2camera.npy",
        ]
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
            DeclareLaunchArgument(
                "hand_eye_mode",
                default_value="eye_in_hand_npy",
                description=(
                    "3D transform mode. Use eye_in_hand_npy for real M0609 "
                    "with T_gripper2camera.npy, or static_base_camera for simulation."
                ),
            ),
            DeclareLaunchArgument(
                "gripper_to_camera_matrix_path",
                default_value=default_gripper_to_camera_matrix_path,
                description=(
                    "Path to T_gripper2camera.npy from hand-eye calibration. "
                    "Default is the copy installed inside cocktail_robot_system."
                ),
            ),
            Node(
                package="cocktail_robot_system",
                executable="vision_node",
                name="vision_node",
                output="screen",
                parameters=[params_file, {"debug": ParameterValue(debug, value_type=bool)}],
            ),
            Node(
                package="cocktail_robot_system",
                executable="detection_3d_node",
                name="detection_3d_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "hand_eye_mode": hand_eye_mode,
                        "gripper_to_camera_matrix_path": gripper_to_camera_matrix_path,
                    },
                ],
            ),
            Node(
                package="cocktail_robot_system",
                executable="lid_pick_place_keyboard",
                name="lid_pick_place_keyboard",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_real_robot": ParameterValue(use_real_robot, value_type=bool),
                        "execute_motion": ParameterValue(execute_motion, value_type=bool),
                    },
                ],
            ),
        ]
    )
