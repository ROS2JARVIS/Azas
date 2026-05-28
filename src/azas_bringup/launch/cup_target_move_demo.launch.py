from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    cup_detection_topic = LaunchConfiguration("cup_detection_topic")
    tumbler_pose_topic = LaunchConfiguration("tumbler_pose_topic")
    frame_id = LaunchConfiguration("frame_id")
    robot_color = LaunchConfiguration("robot_color")
    rviz_config = LaunchConfiguration("rviz_config")
    use_robot_urdf = LaunchConfiguration("use_robot_urdf")
    animate_robot_joints = LaunchConfiguration("animate_robot_joints")

    robot_description = {
        "robot_description": Command(
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
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("cup_detection_topic", default_value="/azas/demo/cup_detection"),
            DeclareLaunchArgument("tumbler_pose_topic", default_value="/azas/demo/tumbler_pose"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("use_robot_urdf", default_value="true"),
            DeclareLaunchArgument("animate_robot_joints", default_value="true"),
            DeclareLaunchArgument("robot_color", default_value="white"),
            DeclareLaunchArgument("confidence", default_value="0.95"),
            DeclareLaunchArgument("grasp_x", default_value="0.32"),
            DeclareLaunchArgument("grasp_y", default_value="-0.22"),
            DeclareLaunchArgument("grasp_z", default_value="0.05"),
            DeclareLaunchArgument("mouth_x", default_value="0.32"),
            DeclareLaunchArgument("mouth_y", default_value="-0.22"),
            DeclareLaunchArgument("mouth_z", default_value="0.22"),
            DeclareLaunchArgument("target_x", default_value="0.43"),
            DeclareLaunchArgument("target_y", default_value="0.08"),
            DeclareLaunchArgument("target_z", default_value="0.135"),
            DeclareLaunchArgument("grasp_height_m", default_value="0.085"),
            DeclareLaunchArgument("lift_height_m", default_value="0.000"),
            DeclareLaunchArgument("side_pre_grasp_offset_m", default_value="0.100"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "rviz", "cup_target_move.rviz"]
                ),
            ),
            Node(
                package="azas_perception",
                executable="simulated_cup_detection_node",
                name="simulated_cup_detection_node",
                output="screen",
                parameters=[
                    {
                        "output_topic": cup_detection_topic,
                        "frame_id": frame_id,
                        "confidence": ParameterValue(LaunchConfiguration("confidence"), value_type=float),
                        "publish_once": True,
                        "grasp_x": ParameterValue(LaunchConfiguration("grasp_x"), value_type=float),
                        "grasp_y": ParameterValue(LaunchConfiguration("grasp_y"), value_type=float),
                        "grasp_z": ParameterValue(LaunchConfiguration("grasp_z"), value_type=float),
                        "mouth_x": ParameterValue(LaunchConfiguration("mouth_x"), value_type=float),
                        "mouth_y": ParameterValue(LaunchConfiguration("mouth_y"), value_type=float),
                        "mouth_z": ParameterValue(LaunchConfiguration("mouth_z"), value_type=float),
                    }
                ],
            ),
            Node(
                package="azas_perception",
                executable="cup_detection_pose_bridge_node",
                name="demo_cup_pose_bridge_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": cup_detection_topic,
                        "output_topic": tumbler_pose_topic,
                        "min_confidence": ParameterValue(LaunchConfiguration("confidence"), value_type=float),
                        "target_frame": frame_id,
                        "require_tf": False,
                    }
                ],
            ),
            Node(
                package="azas_motion",
                executable="cup_target_move_preview_node",
                name="cup_target_move_preview_node",
                output="screen",
                parameters=[
                    {
                        "cup_pose_topic": tumbler_pose_topic,
                        "frame_id": frame_id,
                        "target_x": ParameterValue(LaunchConfiguration("target_x"), value_type=float),
                        "target_y": ParameterValue(LaunchConfiguration("target_y"), value_type=float),
                        "target_z": ParameterValue(LaunchConfiguration("target_z"), value_type=float),
                        "grasp_height_m": ParameterValue(
                            LaunchConfiguration("grasp_height_m"), value_type=float
                        ),
                        "lift_height_m": ParameterValue(
                            LaunchConfiguration("lift_height_m"), value_type=float
                        ),
                        "side_pre_grasp_offset_m": ParameterValue(
                            LaunchConfiguration("side_pre_grasp_offset_m"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="m0609_robot_state_publisher",
                output="screen",
                parameters=[robot_description],
                condition=IfCondition(use_robot_urdf),
            ),
            Node(
                package="azas_motion",
                executable="m0609_shake_joint_state_node",
                name="m0609_cup_target_move_joint_state_node",
                output="screen",
                parameters=[{"preview_mode": "cup_target_move"}],
                condition=IfCondition(animate_robot_joints),
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
