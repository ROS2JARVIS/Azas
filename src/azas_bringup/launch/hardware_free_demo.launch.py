from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    selected_dispenser_id = LaunchConfiguration("selected_dispenser_id")
    use_rviz = LaunchConfiguration("use_rviz")
    use_robot_urdf = LaunchConfiguration("use_robot_urdf")
    enable_ik_preview = LaunchConfiguration("enable_ik_preview")
    run_live_stt = LaunchConfiguration("run_live_stt")
    run_recipe_mapper = LaunchConfiguration("run_recipe_mapper")
    use_llm = LaunchConfiguration("use_llm")
    cup_detection_topic = LaunchConfiguration("cup_detection_topic")
    tumbler_pose_topic = LaunchConfiguration("tumbler_pose_topic")
    frame_id = LaunchConfiguration("frame_id")
    robot_color = LaunchConfiguration("robot_color")
    rviz_config = LaunchConfiguration("rviz_config")

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
            DeclareLaunchArgument("selected_dispenser_id", default_value="2"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_robot_urdf", default_value="true"),
            DeclareLaunchArgument("enable_ik_preview", default_value="true"),
            DeclareLaunchArgument("run_live_stt", default_value="false"),
            DeclareLaunchArgument("run_recipe_mapper", default_value="false"),
            DeclareLaunchArgument("use_llm", default_value="false"),
            DeclareLaunchArgument("enable_llm", default_value="false"),
            DeclareLaunchArgument("llm_model", default_value="gpt-4o-mini"),
            DeclareLaunchArgument("llm_base_url", default_value="https://api.openai.com/v1"),
            DeclareLaunchArgument("llm_api_key_env", default_value="OPENAI_API_KEY"),
            DeclareLaunchArgument("stt_topic", default_value="/stt_result"),
            DeclareLaunchArgument("cup_detection_topic", default_value="/azas/demo/cup_detection"),
            DeclareLaunchArgument("tumbler_pose_topic", default_value="/azas/demo/tumbler_pose"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("confidence", default_value="0.95"),
            DeclareLaunchArgument("grasp_x", default_value="0.42"),
            DeclareLaunchArgument("grasp_y", default_value="-0.24"),
            DeclareLaunchArgument("grasp_z", default_value="0.05"),
            DeclareLaunchArgument("mouth_x", default_value="0.42"),
            DeclareLaunchArgument("mouth_y", default_value="-0.24"),
            DeclareLaunchArgument("mouth_z", default_value="0.22"),
            DeclareLaunchArgument("robot_color", default_value="white"),
            DeclareLaunchArgument("planning_group", default_value="manipulator"),
            DeclareLaunchArgument("ee_link", default_value="tool0"),
            DeclareLaunchArgument("planning_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("loop_preview", default_value="true"),
            DeclareLaunchArgument("preview_publish_rate", default_value="30.0"),
            DeclareLaunchArgument("preview_frames_per_step", default_value="45"),
            DeclareLaunchArgument("preview_hold_frames", default_value="10"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "rviz", "azas_dispenser_sequence.rviz"]
                ),
            ),
            Node(
                package="azas_voice",
                executable="stt_node",
                name="stt_node",
                output="screen",
                parameters=[{"stt_topic": LaunchConfiguration("stt_topic")}],
                condition=IfCondition(run_live_stt),
            ),
            Node(
                package="azas_voice",
                executable="recipe_mapper_node",
                name="recipe_mapper_node",
                output="screen",
                parameters=[{"stt_topic": LaunchConfiguration("stt_topic")}],
                condition=IfCondition(run_recipe_mapper),
            ),
            Node(
                package="azas_voice",
                executable="llm_recipe_mapper_node",
                name="llm_recipe_mapper_node",
                output="screen",
                parameters=[
                    {
                        "stt_topic": LaunchConfiguration("stt_topic"),
                        "enable_llm": ParameterValue(LaunchConfiguration("enable_llm"), value_type=bool),
                        "model": LaunchConfiguration("llm_model"),
                        "base_url": LaunchConfiguration("llm_base_url"),
                        "api_key_env": LaunchConfiguration("llm_api_key_env"),
                    }
                ],
                condition=IfCondition(use_llm),
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
                        "publish_once": False,
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
                executable="dispenser_sequence_preview_node",
                name="dispenser_sequence_preview_node",
                output="screen",
                parameters=[
                    {
                        "cup_pose_topic": tumbler_pose_topic,
                        "frame_id": frame_id,
                        "selected_dispenser_id": ParameterValue(
                            selected_dispenser_id, value_type=int
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
                executable="side_grasp_ik_preview_node",
                name="side_grasp_ik_preview_node",
                output="screen",
                parameters=[
                    {
                        "plan_topic": "/azas/dispenser_sequence/plan",
                        "planning_group": LaunchConfiguration("planning_group"),
                        "ee_link": LaunchConfiguration("ee_link"),
                        "planning_timeout_sec": ParameterValue(
                            LaunchConfiguration("planning_timeout_sec"), value_type=float
                        ),
                        "publish_rate": ParameterValue(
                            LaunchConfiguration("preview_publish_rate"), value_type=float
                        ),
                        "frames_per_step": ParameterValue(
                            LaunchConfiguration("preview_frames_per_step"), value_type=int
                        ),
                        "hold_frames": ParameterValue(
                            LaunchConfiguration("preview_hold_frames"), value_type=int
                        ),
                        "loop_preview": ParameterValue(
                            LaunchConfiguration("loop_preview"), value_type=bool
                        ),
                    }
                ],
                condition=IfCondition(enable_ik_preview),
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
