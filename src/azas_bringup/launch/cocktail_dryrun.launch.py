from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    run_voice = LaunchConfiguration("run_voice")
    run_yolo = LaunchConfiguration("run_yolo")

    voice_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("azas_voice"), "launch", "azas_voice.launch.py"])
        ),
        launch_arguments={
            "use_live_stt": LaunchConfiguration("use_live_stt"),
            "use_llm": LaunchConfiguration("use_llm"),
            "enable_llm": LaunchConfiguration("enable_llm"),
            "llm_model": LaunchConfiguration("llm_model"),
            "llm_base_url": LaunchConfiguration("llm_base_url"),
            "llm_api_key_env": LaunchConfiguration("llm_api_key_env"),
            "stt_topic": LaunchConfiguration("stt_topic"),
        }.items(),
        condition=IfCondition(run_voice),
    )

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("azas_bringup"), "launch", "yolo_perception.launch.py"])
        ),
        launch_arguments={
            "model_path": LaunchConfiguration("model_path"),
            "color_topic": LaunchConfiguration("color_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "confidence_threshold": LaunchConfiguration("confidence_threshold"),
            "device": LaunchConfiguration("device"),
        }.items(),
        condition=IfCondition(run_yolo),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_voice", default_value="true"),
            DeclareLaunchArgument("use_live_stt", default_value="false"),
            DeclareLaunchArgument("use_llm", default_value="false"),
            DeclareLaunchArgument("enable_llm", default_value="false"),
            DeclareLaunchArgument("llm_model", default_value="gpt-4o-mini"),
            DeclareLaunchArgument("llm_base_url", default_value="https://api.openai.com/v1"),
            DeclareLaunchArgument("llm_api_key_env", default_value="OPENAI_API_KEY"),
            DeclareLaunchArgument("stt_topic", default_value="/stt_result"),
            DeclareLaunchArgument("run_yolo", default_value="true"),
            DeclareLaunchArgument("model_path", default_value="/home/ssu/Downloads/best.pt"),
            DeclareLaunchArgument("color_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
            DeclareLaunchArgument("confidence_threshold", default_value="0.35"),
            DeclareLaunchArgument("device", default_value="cpu"),
            DeclareLaunchArgument("max_detection_age_s", default_value="5.0"),
            DeclareLaunchArgument("require_cup", default_value="true"),
            DeclareLaunchArgument("require_lid", default_value="true"),
            voice_launch,
            yolo_launch,
            Node(
                package="azas_task_manager",
                executable="cocktail_dryrun_sequence_node",
                name="cocktail_dryrun_sequence_node",
                output="screen",
                parameters=[
                    {
                        "max_detection_age_s": ParameterValue(
                            LaunchConfiguration("max_detection_age_s"), value_type=float
                        ),
                        "require_cup": ParameterValue(LaunchConfiguration("require_cup"), value_type=bool),
                        "require_lid": ParameterValue(LaunchConfiguration("require_lid"), value_type=bool),
                    }
                ],
            ),
        ]
    )
