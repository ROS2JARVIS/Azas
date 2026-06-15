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
    show_sequence_markers = LaunchConfiguration("show_sequence_markers")
    show_dispenser_markers = LaunchConfiguration("show_dispenser_markers")
    show_animated_cup = LaunchConfiguration("show_animated_cup")
    show_demo_arm = LaunchConfiguration("show_demo_arm")
    show_workspace_safety = LaunchConfiguration("show_workspace_safety")
    show_measured_dispenser_collision = LaunchConfiguration("show_measured_dispenser_collision")
    show_full_collision_scene = LaunchConfiguration("show_full_collision_scene")
    show_link6_gripper = LaunchConfiguration("show_link6_gripper")
    enable_rule_joint_preview = LaunchConfiguration("enable_rule_joint_preview")
    safety_config_path = LaunchConfiguration("safety_config_path")
    dispenser_collision_config_path = LaunchConfiguration("dispenser_collision_config_path")
    calibration_path = LaunchConfiguration("calibration_path")

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
            DeclareLaunchArgument("llm_provider", default_value="openai_chat"),
            DeclareLaunchArgument("llm_model", default_value="gpt-4o-mini"),
            DeclareLaunchArgument("llm_base_url", default_value="https://api.openai.com/v1"),
            DeclareLaunchArgument("llm_api_key_env", default_value="OPENAI_API_KEY"),
            DeclareLaunchArgument("elevenlabs_agent_id_env", default_value="ELEVENLABS_AGENT_ID"),
            DeclareLaunchArgument("elevenlabs_language", default_value="ko"),
            DeclareLaunchArgument("elevenlabs_new_turns_limit", default_value="2"),
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
            DeclareLaunchArgument("show_sequence_markers", default_value="true"),
            DeclareLaunchArgument("show_dispenser_markers", default_value="true"),
            DeclareLaunchArgument("show_animated_cup", default_value="true"),
            DeclareLaunchArgument("show_demo_arm", default_value="true"),
            DeclareLaunchArgument("show_workspace_safety", default_value="true"),
            DeclareLaunchArgument("show_measured_dispenser_collision", default_value="true"),
            DeclareLaunchArgument("show_full_collision_scene", default_value="true"),
            DeclareLaunchArgument("show_link6_gripper", default_value="true"),
            DeclareLaunchArgument("enable_rule_joint_preview", default_value="false"),
            DeclareLaunchArgument(
                "safety_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "config", "safety.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "dispenser_collision_config_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("azas_bringup"),
                        "config",
                        "measured_dispenser_collision.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "calibration_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "config", "calibration.yaml"]
                ),
            ),
            DeclareLaunchArgument("planning_group", default_value="manipulator"),
            DeclareLaunchArgument("ee_link", default_value="tool0"),
            DeclareLaunchArgument("planning_pipeline", default_value="pilz_industrial_motion_planner"),
            DeclareLaunchArgument("planner_id", default_value="PTP"),
            DeclareLaunchArgument("planning_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("max_velocity_scaling_factor", default_value="0.1"),
            DeclareLaunchArgument("max_acceleration_scaling_factor", default_value="0.1"),
            DeclareLaunchArgument("loop_preview", default_value="true"),
            DeclareLaunchArgument("preview_publish_rate", default_value="30.0"),
            DeclareLaunchArgument("preview_frames_per_step", default_value="45"),
            DeclareLaunchArgument("preview_hold_frames", default_value="10"),
            DeclareLaunchArgument("ik_preview_pose_limit", default_value="15"),
            DeclareLaunchArgument("ik_preview_planning_start_delay_sec", default_value="2.5"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("azas_bringup"), "rviz", "azas_dispenser_sequence_clean.rviz"]
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
                        "provider": LaunchConfiguration("llm_provider"),
                        "model": LaunchConfiguration("llm_model"),
                        "base_url": LaunchConfiguration("llm_base_url"),
                        "api_key_env": LaunchConfiguration("llm_api_key_env"),
                        "elevenlabs_agent_id_env": LaunchConfiguration("elevenlabs_agent_id_env"),
                        "elevenlabs_language": LaunchConfiguration("elevenlabs_language"),
                        "elevenlabs_new_turns_limit": ParameterValue(
                            LaunchConfiguration("elevenlabs_new_turns_limit"), value_type=int
                        ),
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
                        "calibration_path": ParameterValue(calibration_path, value_type=str),
                        "dispenser_collision_config_path": ParameterValue(
                            dispenser_collision_config_path, value_type=str
                        ),
                        "selected_dispenser_id": ParameterValue(
                            selected_dispenser_id, value_type=int
                        ),
                        "show_sequence_markers": ParameterValue(
                            show_sequence_markers, value_type=bool
                        ),
                        "show_dispenser_markers": ParameterValue(
                            show_dispenser_markers, value_type=bool
                        ),
                        "show_animated_cup": ParameterValue(show_animated_cup, value_type=bool),
                        "show_demo_arm": ParameterValue(show_demo_arm, value_type=bool),
                    }
                ],
            ),
            Node(
                package="azas_motion",
                executable="workspace_collision_scene_node",
                name="workspace_collision_scene_node",
                output="screen",
                condition=IfCondition(show_workspace_safety),
                parameters=[
                    {
                        "safety_config_path": ParameterValue(safety_config_path, value_type=str),
                        "publish_collision_objects": True,
                        "publish_period_sec": 2.0,
                        "frame_id": frame_id,
                        "table_collision_enabled": True,
                        "table_collision_expand_to_workspace_walls": True,
                        "workspace_boundary_collision_enabled": True,
                    }
                ],
            ),
            Node(
                package="azas_motion",
                executable="measured_dispenser_collision_scene_node",
                name="measured_dispenser_collision_scene_node",
                output="screen",
                condition=IfCondition(show_measured_dispenser_collision),
                parameters=[
                    {
                        "config_path": ParameterValue(
                            dispenser_collision_config_path, value_type=str
                        ),
                        "calibration_path": ParameterValue(calibration_path, value_type=str),
                        "publish_collision_objects": True,
                        "publish_markers": True,
                        "publish_rviz_visual_tools_compat": True,
                        "publish_debug_labels": True,
                        "publish_period_sec": 2.0,
                    }
                ],
            ),
            Node(
                package="azas_bringup",
                executable="collision_scene_rviz_publisher",
                name="collision_scene_rviz_publisher",
                output="screen",
                condition=IfCondition(show_full_collision_scene),
                parameters=[
                    {
                        "frame_id": frame_id,
                        "safety_config_path": ParameterValue(safety_config_path, value_type=str),
                        "dispenser_collision_config_path": ParameterValue(
                            dispenser_collision_config_path, value_type=str
                        ),
                        "calibration_path": ParameterValue(calibration_path, value_type=str),
                    }
                ],
            ),
            Node(
                package="azas_motion",
                executable="link6_gripper_collision_node",
                name="link6_gripper_collision_node",
                output="screen",
                condition=IfCondition(show_link6_gripper),
                parameters=[
                    {
                        "attached_link_name": "link_6",
                        "publish_markers": True,
                        "marker_topic": "/azas/link6_gripper/markers",
                        "remove_on_shutdown": False,
                    }
                ],
            ),
            Node(
                package="azas_bringup",
                executable="rule_motion_joint_preview_node",
                name="rule_motion_joint_preview_node",
                output="screen",
                condition=IfCondition(enable_rule_joint_preview),
                parameters=[
                    {
                        "path_topic": "/azas/dispenser_sequence/plan",
                        "joint_state_topic": "/joint_states",
                        "publish_rate_hz": 30.0,
                        "frames_per_pose": 12,
                        "loop_preview": True,
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
                        "planning_pipeline": LaunchConfiguration("planning_pipeline"),
                        "planner_id": LaunchConfiguration("planner_id"),
                        "planning_timeout_sec": ParameterValue(
                            LaunchConfiguration("planning_timeout_sec"), value_type=float
                        ),
                        "max_velocity_scaling_factor": ParameterValue(
                            LaunchConfiguration("max_velocity_scaling_factor"), value_type=float
                        ),
                        "max_acceleration_scaling_factor": ParameterValue(
                            LaunchConfiguration("max_acceleration_scaling_factor"), value_type=float
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
                        "max_preview_poses": ParameterValue(
                            LaunchConfiguration("ik_preview_pose_limit"), value_type=int
                        ),
                        "planning_start_delay_sec": ParameterValue(
                            LaunchConfiguration("ik_preview_planning_start_delay_sec"),
                            value_type=float,
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
