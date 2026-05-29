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
            DeclareLaunchArgument("use_tts", default_value="true"),
            DeclareLaunchArgument("enable_tts_audio", default_value="true"),
            DeclareLaunchArgument("tts_speech_rate", default_value="1.25"),
            DeclareLaunchArgument("tts_startup_prompt", default_value="주문하시겠어요?"),
            DeclareLaunchArgument("enable_llm", default_value="false"),
            DeclareLaunchArgument("llm_model", default_value="gpt-4o-mini"),
            DeclareLaunchArgument("llm_base_url", default_value="https://api.openai.com/v1"),
            DeclareLaunchArgument("llm_api_key_env", default_value="OPENAI_API_KEY"),
            DeclareLaunchArgument("stt_topic", default_value="/stt_result"),
            Node(
                package="azas_voice",
                executable="recipe_mapper_node",
                name="recipe_mapper_node",
                output="screen",
                parameters=[{"stt_topic": stt_topic}],
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
                    }
                ],
                condition=IfCondition(use_llm),
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
        ]
    )
