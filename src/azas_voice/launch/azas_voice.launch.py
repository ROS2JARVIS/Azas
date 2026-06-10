from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_live_stt = LaunchConfiguration("use_live_stt")
    use_llm = LaunchConfiguration("use_llm")
    use_conversation_manager = LaunchConfiguration("use_conversation_manager")
    run_voice_screen = LaunchConfiguration("run_voice_screen")
    use_dispenser_executor = LaunchConfiguration("use_dispenser_executor")
    use_tts = LaunchConfiguration("use_tts")
    enable_tts_audio = LaunchConfiguration("enable_tts_audio")
    tts_speech_rate = LaunchConfiguration("tts_speech_rate")
    tts_startup_prompt = LaunchConfiguration("tts_startup_prompt")
    stt_topic = LaunchConfiguration("stt_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_live_stt", default_value="false"),
            DeclareLaunchArgument("use_llm", default_value="false"),
            DeclareLaunchArgument("use_conversation_manager", default_value="true"),
            DeclareLaunchArgument("use_dispenser_executor", default_value="false"),
            DeclareLaunchArgument("enable_dispenser_hardware_execution", default_value="false"),
            DeclareLaunchArgument("dispenser_service_prefix", default_value="/"),
            DeclareLaunchArgument("dispenser_tcp_name", default_value=""),
            DeclareLaunchArgument(
                "dispenser_require_tcp_for_taught_posx", default_value="true"
            ),
            DeclareLaunchArgument("dispenser_joint_velocity", default_value="10.0"),
            DeclareLaunchArgument("dispenser_joint_acceleration", default_value="10.0"),
            DeclareLaunchArgument("dispenser_line_velocity", default_value="15.0"),
            DeclareLaunchArgument("dispenser_line_acceleration", default_value="25.0"),
            DeclareLaunchArgument("run_voice_screen", default_value="true"),
            DeclareLaunchArgument("voice_screen_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("voice_screen_port", default_value="8090"),
            DeclareLaunchArgument("use_tts", default_value="true"),
            DeclareLaunchArgument("enable_tts_audio", default_value="true"),
            DeclareLaunchArgument("tts_speech_rate", default_value="1.25"),
            DeclareLaunchArgument(
                "tts_startup_prompt",
                default_value="원하는 맛을 말씀해주시면 추천해드릴게요. 주문하시겠어요?",
            ),
            DeclareLaunchArgument("enable_llm", default_value="false"),
            DeclareLaunchArgument("llm_model", default_value="gpt-4o-mini"),
            DeclareLaunchArgument("llm_base_url", default_value="https://api.openai.com/v1"),
            DeclareLaunchArgument("llm_api_key_env", default_value="OPENAI_API_KEY"),
            DeclareLaunchArgument("llm_request_timeout_sec", default_value="20.0"),
            DeclareLaunchArgument("stt_topic", default_value="/stt_result"),
            Node(
                package="azas_voice",
                executable="recipe_mapper_node",
                name="recipe_mapper_node",
                output="screen",
                parameters=[
                    {
                        "stt_topic": stt_topic,
                        "publish_confirmation": False,
                    }
                ],
                condition=UnlessCondition(use_llm),
            ),
            Node(
                package="azas_voice",
                executable="llm_recipe_mapper_node",
                name="llm_recipe_mapper_node",
                output="screen",
                parameters=[
                    {
                        "stt_topic": stt_topic,
                        "enable_llm": ParameterValue(LaunchConfiguration("enable_llm"), value_type=bool),
                        "model": LaunchConfiguration("llm_model"),
                        "base_url": LaunchConfiguration("llm_base_url"),
                        "api_key_env": LaunchConfiguration("llm_api_key_env"),
                        "request_timeout_sec": ParameterValue(
                            LaunchConfiguration("llm_request_timeout_sec"), value_type=float
                        ),
                        "publish_confirmation": False,
                    }
                ],
                condition=IfCondition(use_llm),
            ),
            Node(
                package="azas_voice",
                executable="conversation_manager_node",
                name="conversation_manager_node",
                output="screen",
                condition=IfCondition(use_conversation_manager),
            ),
            Node(
                package="azas_voice",
                executable="voice_dispenser_executor_node",
                name="voice_dispenser_executor_node",
                output="screen",
                parameters=[
                    {
                        "enable_hardware_execution": ParameterValue(
                            LaunchConfiguration("enable_dispenser_hardware_execution"),
                            value_type=bool,
                        ),
                        "service_prefix": LaunchConfiguration("dispenser_service_prefix"),
                        "tcp_name": LaunchConfiguration("dispenser_tcp_name"),
                        "require_tcp_for_taught_posx": ParameterValue(
                            LaunchConfiguration("dispenser_require_tcp_for_taught_posx"),
                            value_type=bool,
                        ),
                        "joint_velocity": ParameterValue(
                            LaunchConfiguration("dispenser_joint_velocity"), value_type=float
                        ),
                        "joint_acceleration": ParameterValue(
                            LaunchConfiguration("dispenser_joint_acceleration"), value_type=float
                        ),
                        "line_velocity": ParameterValue(
                            LaunchConfiguration("dispenser_line_velocity"), value_type=float
                        ),
                        "line_acceleration": ParameterValue(
                            LaunchConfiguration("dispenser_line_acceleration"), value_type=float
                        ),
                    }
                ],
                condition=IfCondition(use_dispenser_executor),
            ),
            Node(
                package="azas_voice",
                executable="stt_node",
                name="stt_node",
                output="screen",
                parameters=[{"stt_topic": stt_topic}],
                condition=IfCondition(use_live_stt),
            ),
            Node(
                package="azas_voice",
                executable="tts_node",
                name="tts_node",
                output="screen",
                parameters=[
                    {
                        "enable_audio": ParameterValue(enable_tts_audio, value_type=bool),
                        "speech_rate": ParameterValue(tts_speech_rate, value_type=float),
                        "startup_prompt": tts_startup_prompt,
                    }
                ],
                condition=IfCondition(use_tts),
            ),
            Node(
                package="azas_voice",
                executable="voice_screen_node",
                name="azas_voice_screen_node",
                output="screen",
                parameters=[
                    {
                        "host": LaunchConfiguration("voice_screen_host"),
                        "port": ParameterValue(
                            LaunchConfiguration("voice_screen_port"), value_type=int
                        ),
                        "stt_topic": stt_topic,
                    }
                ],
                condition=IfCondition(run_voice_screen),
            ),
        ]
    )
