from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cup_detection_topic = LaunchConfiguration("cup_detection_topic")
    tumbler_pose_topic = LaunchConfiguration("tumbler_pose_topic")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription([
        DeclareLaunchArgument("selected_dispenser_id", default_value="2"),
        DeclareLaunchArgument("cup_detection_topic", default_value="/azas/sim/cup_detection"),
        DeclareLaunchArgument("tumbler_pose_topic", default_value="/azas/sim/tumbler_pose"),
        DeclareLaunchArgument("frame_id", default_value="base_link"),
        DeclareLaunchArgument("confidence", default_value="0.95"),
        DeclareLaunchArgument("bridge_min_confidence", default_value="0.35"),
        DeclareLaunchArgument("planning_group", default_value=""),
        DeclareLaunchArgument("ee_link", default_value=""),
        DeclareLaunchArgument("planning_timeout_sec", default_value="5.0"),
        DeclareLaunchArgument("publish_once", default_value="true"),
        DeclareLaunchArgument("grasp_x", default_value="0.32"),
        DeclareLaunchArgument("grasp_y", default_value="-0.22"),
        DeclareLaunchArgument("grasp_z", default_value="0.05"),
        DeclareLaunchArgument("mouth_x", default_value="0.32"),
        DeclareLaunchArgument("mouth_y", default_value="-0.22"),
        DeclareLaunchArgument("mouth_z", default_value="0.22"),
        Node(
            package="azas_perception",
            executable="simulated_cup_detection_node",
            name="simulated_cup_detection_node",
            output="screen",
            parameters=[{
                "output_topic": cup_detection_topic,
                "frame_id": frame_id,
                "confidence": ParameterValue(LaunchConfiguration("confidence"), value_type=float),
                "publish_once": ParameterValue(LaunchConfiguration("publish_once"), value_type=bool),
                "grasp_x": ParameterValue(LaunchConfiguration("grasp_x"), value_type=float),
                "grasp_y": ParameterValue(LaunchConfiguration("grasp_y"), value_type=float),
                "grasp_z": ParameterValue(LaunchConfiguration("grasp_z"), value_type=float),
                "mouth_x": ParameterValue(LaunchConfiguration("mouth_x"), value_type=float),
                "mouth_y": ParameterValue(LaunchConfiguration("mouth_y"), value_type=float),
                "mouth_z": ParameterValue(LaunchConfiguration("mouth_z"), value_type=float),
            }],
        ),
        Node(
            package="azas_perception",
            executable="cup_detection_pose_bridge_node",
            name="simulated_cup_pose_bridge_node",
            output="screen",
            parameters=[{
                "input_topic": cup_detection_topic,
                "output_topic": tumbler_pose_topic,
                "min_confidence": ParameterValue(LaunchConfiguration("bridge_min_confidence"), value_type=float),
                "target_frame": frame_id,
                "require_tf": False,
            }],
        ),
        Node(
            package="azas_motion",
            executable="alignment_executor_node",
            name="side_grasp_planning_only_checker",
            output="screen",
            parameters=[{
                "enable_planning_only": True,
                "allow_execute": False,
                "planning_group": LaunchConfiguration("planning_group"),
                "ee_link": LaunchConfiguration("ee_link"),
                "base_frame": frame_id,
                "planning_timeout_sec": ParameterValue(LaunchConfiguration("planning_timeout_sec"), value_type=float),
                "use_fake_side_grasp_plan": True,
                "cup_reference_x": ParameterValue(LaunchConfiguration("grasp_x"), value_type=float),
                "cup_reference_y": ParameterValue(LaunchConfiguration("grasp_y"), value_type=float),
                "cup_reference_z": ParameterValue(LaunchConfiguration("grasp_z"), value_type=float),
            }],
        ),
        Node(
            package="azas_motion",
            executable="dispenser_sequence_preview_node",
            name="dispenser_sequence_preview_node",
            output="screen",
            parameters=[{
                "cup_pose_topic": tumbler_pose_topic,
                "frame_id": frame_id,
                "selected_dispenser_id": ParameterValue(
                    LaunchConfiguration("selected_dispenser_id"), value_type=int
                ),
            }],
        ),
    ])
