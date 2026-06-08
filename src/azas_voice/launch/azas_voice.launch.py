from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_live_stt = LaunchConfiguration("use_live_stt")
    use_llm = LaunchConfiguration("use_llm")
    stt_topic = LaunchConfiguration("stt_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_live_stt", default_value="false"),
            DeclareLaunchArgument("use_llm", default_value="false"),
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
        ]
    )
