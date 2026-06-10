from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_realsense = LaunchConfiguration("enable_realsense")
    enable_rg2 = LaunchConfiguration("enable_rg2")
    enable_voice = LaunchConfiguration("enable_voice")
    enable_yolo_cup = LaunchConfiguration("enable_yolo_cup")
    enable_lid_pipeline = LaunchConfiguration("enable_lid_pipeline")
    enable_full_sequence = LaunchConfiguration("enable_full_sequence")
    publish_camera_base_tf = LaunchConfiguration("publish_camera_base_tf")
    publish_hand_eye_tf = LaunchConfiguration("publish_hand_eye_tf")

    # 1) RealSense camera
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                ]
            )
        ),
        launch_arguments={
            "camera_name": LaunchConfiguration("realsense_camera_name"),
            "camera_namespace": LaunchConfiguration("realsense_camera_namespace"),
            "enable_color": LaunchConfiguration("realsense_enable_color"),
            "enable_depth": LaunchConfiguration("realsense_enable_depth"),
            "align_depth.enable": LaunchConfiguration("realsense_align_depth"),
        }.items(),
        condition=IfCondition(enable_realsense),
    )

    # 2) RG2 gripper service node
    rg2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("azas_gripper"),
                    "launch",
                    "rg2_trigger.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ip": LaunchConfiguration("rg2_ip"),
            "port": LaunchConfiguration("rg2_port"),
            "connect": LaunchConfiguration("rg2_connect"),
        }.items(),
        condition=IfCondition(enable_rg2),
    )

    # 3) Cup YOLO perception
    yolo_cup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("azas_bringup"),
                    "launch",
                    "yolo_perception.launch.py",
                ]
            )
        ),
        launch_arguments={
            "model_path": LaunchConfiguration("cup_model_path"),
            "color_topic": LaunchConfiguration("color_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "confidence_threshold": LaunchConfiguration("cup_confidence_threshold"),
            "target_class_names": LaunchConfiguration("cup_target_class_names"),
            "selection_policy": LaunchConfiguration("cup_selection_policy"),
            "source_frame": LaunchConfiguration("camera_source_frame"),
            "depth_window_size": LaunchConfiguration("depth_window_size"),
            "min_depth_m": LaunchConfiguration("min_depth_m"),
            "max_depth_m": LaunchConfiguration("max_depth_m"),
            "device": LaunchConfiguration("device"),
        }.items(),
        condition=IfCondition(enable_yolo_cup),
    )

    # 4) Optional fixed camera TF
    # 외부 고정 카메라를 base_link 기준으로 직접 publish할 때 사용.
    # eye-in-hand 방식이면 보통 false로 두고 hand-eye TF를 사용.
    camera_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_base_static_tf",
        output="screen",
        arguments=[
            "--x",
            LaunchConfiguration("camera_base_tf_x"),
            "--y",
            LaunchConfiguration("camera_base_tf_y"),
            "--z",
            LaunchConfiguration("camera_base_tf_z"),
            "--roll",
            LaunchConfiguration("camera_base_tf_roll"),
            "--pitch",
            LaunchConfiguration("camera_base_tf_pitch"),
            "--yaw",
            LaunchConfiguration("camera_base_tf_yaw"),
            "--frame-id",
            LaunchConfiguration("camera_base_parent_frame"),
            "--child-frame-id",
            LaunchConfiguration("camera_base_child_frame"),
        ],
        condition=IfCondition(publish_camera_base_tf),
    )

    # 5) Hand-eye TF
    # link_6 -> camera_link 연결.
    hand_eye_tf = Node(
        package="azas_perception",
        executable="hand_eye_static_tf_node",
        name="hand_eye_static_tf_node",
        output="screen",
        parameters=[
            {
                "matrix_path": LaunchConfiguration("hand_eye_matrix_path"),
                "parent_frame": LaunchConfiguration("hand_eye_parent_frame"),
                "matrix_child_frame": LaunchConfiguration("hand_eye_matrix_child_frame"),
                "published_child_frame": LaunchConfiguration(
                    "hand_eye_published_child_frame"
                ),
                "translation_scale": ParameterValue(
                    LaunchConfiguration("hand_eye_translation_scale"),
                    value_type=float,
                ),
                "compose_with_existing_tf": ParameterValue(
                    LaunchConfiguration("hand_eye_compose_with_existing_tf"),
                    value_type=bool,
                ),
                "compose_timeout_sec": ParameterValue(
                    LaunchConfiguration("hand_eye_compose_timeout_sec"),
                    value_type=float,
                ),
            }
        ],
        condition=IfCondition(publish_hand_eye_tf),
    )

    # 6) CupDetection -> base_link PoseStamped bridge
    # /azas/cup_detection -> /jarvis/tumbler_dispenser/tumbler_pose
    cup_pose_bridge = Node(
        package="azas_perception",
        executable="cup_detection_pose_bridge_node",
        name="cup_detection_pose_bridge_node",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("cup_detection_topic"),
                "output_topic": LaunchConfiguration("tumbler_pose_topic"),
                "min_confidence": ParameterValue(
                    LaunchConfiguration("cup_confidence_threshold"),
                    value_type=float,
                ),
                "use_grasp_pose": True,
                "require_status_prefix": "detected",
                "require_upright_status": ParameterValue(
                    LaunchConfiguration("require_upright_cup"),
                    value_type=bool,
                ),
                "target_frame": LaunchConfiguration("target_frame"),
                "source_frame": LaunchConfiguration("camera_source_frame"),
                "require_tf": ParameterValue(
                    LaunchConfiguration("require_tf"),
                    value_type=bool,
                ),
                "transform_timeout_sec": ParameterValue(
                    LaunchConfiguration("transform_timeout_sec"),
                    value_type=float,
                ),
                "debug_pose_logging": ParameterValue(
                    LaunchConfiguration("debug_pose_logging"),
                    value_type=bool,
                ),
                "log_published_pose": ParameterValue(
                    LaunchConfiguration("log_published_pose"),
                    value_type=bool,
                ),
            }
        ],
        condition=IfCondition(enable_yolo_cup),
    )

    # 7) Voice pipeline
    voice_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("azas_voice"),
                    "launch",
                    "azas_voice.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_live_stt": LaunchConfiguration("use_live_stt"),
            "use_llm": LaunchConfiguration("use_llm"),
            "use_conversation_manager": LaunchConfiguration(
                "use_conversation_manager"
            ),
            "use_tts": LaunchConfiguration("use_tts"),
            "enable_tts_audio": LaunchConfiguration("enable_tts_audio"),
            "tts_speech_rate": LaunchConfiguration("tts_speech_rate"),
            "tts_startup_prompt": LaunchConfiguration("tts_startup_prompt"),
            "enable_llm": LaunchConfiguration("enable_llm"),
            "llm_model": LaunchConfiguration("llm_model"),
            "llm_base_url": LaunchConfiguration("llm_base_url"),
            "llm_api_key_env": LaunchConfiguration("llm_api_key_env"),
            "stt_topic": LaunchConfiguration("stt_topic"),
        }.items(),
        condition=IfCondition(enable_voice),
    )

    # 8) Lid detector
    # /azas/lid_detection 으로 CupDetection 타입 publish.
    lid_detector = Node(
        package="azas_perception",
        executable="lid_sticker_detector_node",
        name="lid_sticker_detector_node",
        output="screen",
        parameters=[
            {
                "model_path": LaunchConfiguration("lid_model_path"),
                "color_topic": LaunchConfiguration("color_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "output_topic": LaunchConfiguration("lid_detection_topic"),
                "grip_request_topic": LaunchConfiguration("lid_grip_request_topic"),
                "confidence_threshold": ParameterValue(
                    LaunchConfiguration("lid_confidence_threshold"),
                    value_type=float,
                ),
                "target_class_names": LaunchConfiguration("lid_target_class_names"),
                "selection_policy": LaunchConfiguration("lid_selection_policy"),
                "device": LaunchConfiguration("device"),
                "source_frame": LaunchConfiguration("camera_source_frame"),
                "depth_window_size": ParameterValue(
                    LaunchConfiguration("depth_window_size"),
                    value_type=int,
                ),
                "min_depth_m": ParameterValue(
                    LaunchConfiguration("min_depth_m"),
                    value_type=float,
                ),
                "max_depth_m": ParameterValue(
                    LaunchConfiguration("max_depth_m"),
                    value_type=float,
                ),
                "marker_type": LaunchConfiguration("lid_marker_type"),
                "require_lid_detection": ParameterValue(
                    LaunchConfiguration("require_lid_detection"),
                    value_type=bool,
                ),
                "aruco_dictionary": LaunchConfiguration("aruco_dictionary"),
                "aruco_marker_id": ParameterValue(
                    LaunchConfiguration("aruco_marker_id"),
                    value_type=int,
                ),
                "require_plane_normal": ParameterValue(
                    LaunchConfiguration("require_lid_plane_normal"),
                    value_type=bool,
                ),
                "show_preview": ParameterValue(
                    LaunchConfiguration("show_lid_preview"),
                    value_type=bool,
                ),
                "log_detections": ParameterValue(
                    LaunchConfiguration("log_lid_detections"),
                    value_type=bool,
                ),
            }
        ],
        condition=IfCondition(enable_lid_pipeline),
    )

    # 9) LidDetection -> base_link PoseStamped bridge
    # lid detector는 CupDetection 타입으로 내보내므로,
    # 기존 cup_detection_pose_bridge_node를 재사용해서 PoseStamped로 변환.
    lid_pose_bridge = Node(
        package="azas_perception",
        executable="cup_detection_pose_bridge_node",
        name="lid_detection_pose_bridge_node",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("lid_detection_topic"),
                "output_topic": LaunchConfiguration("lid_pose_topic"),
                "min_confidence": ParameterValue(
                    LaunchConfiguration("lid_confidence_threshold"),
                    value_type=float,
                ),
                "use_grasp_pose": True,
                "require_status_prefix": "detected",
                # lid status는 detected:lid 이므로 cup의 detected:upright 조건을 끈다.
                "require_upright_status": False,
                "target_frame": LaunchConfiguration("target_frame"),
                "source_frame": LaunchConfiguration("camera_source_frame"),
                "require_tf": ParameterValue(
                    LaunchConfiguration("require_tf"),
                    value_type=bool,
                ),
                "transform_timeout_sec": ParameterValue(
                    LaunchConfiguration("transform_timeout_sec"),
                    value_type=float,
                ),
                "debug_pose_logging": ParameterValue(
                    LaunchConfiguration("debug_pose_logging"),
                    value_type=bool,
                ),
                "log_published_pose": ParameterValue(
                    LaunchConfiguration("log_published_pose"),
                    value_type=bool,
                ),
            }
        ],
        condition=IfCondition(enable_lid_pipeline),
    )

    # 10) Lid grip planner
    # /jarvis/lid_gripper/lid_pose 를 받아 approach/grasp/lift 후보를 만들고,
    # hardware gate가 켜져 있으면 실제 뚜껑 집기/결합 동작까지 수행.
    lid_grip_planner = Node(
        package="azas_motion",
        executable="lid_grip_planner_node",
        name="lid_grip_planner_node",
        output="screen",
        parameters=[
            {
                "lid_pose_topic": LaunchConfiguration("lid_pose_topic"),
                "trigger_topic": LaunchConfiguration("lid_grip_request_topic"),
                "approach_pose_topic": "/jarvis/lid_gripper/approach_pose",
                "grasp_pose_topic": "/jarvis/lid_gripper/grasp_pose",
                "lift_pose_topic": "/jarvis/lid_gripper/lift_pose",
                "status_topic": "/jarvis/lid_gripper/status",
                "enable_hardware": ParameterValue(
                    LaunchConfiguration("enable_hardware"),
                    value_type=bool,
                ),
                "hardware_confirm": LaunchConfiguration("hardware_confirm"),
                "allow_service_control_without_moveit": ParameterValue(
                    LaunchConfiguration("allow_service_control_without_moveit"),
                    value_type=bool,
                ),
                "service_prefix": LaunchConfiguration("service_prefix"),
                "line_velocity": ParameterValue(
                    LaunchConfiguration("lid_line_velocity"),
                    value_type=float,
                ),
                "line_acceleration": ParameterValue(
                    LaunchConfiguration("lid_line_acceleration"),
                    value_type=float,
                ),
                "enable_gripper_service_calls": ParameterValue(
                    LaunchConfiguration("enable_lid_gripper_service_calls"),
                    value_type=bool,
                ),
                "execute_gripper_on_pose": ParameterValue(
                    LaunchConfiguration("execute_lid_gripper_on_pose"),
                    value_type=bool,
                ),
                "gripper_set_service": LaunchConfiguration("gripper_set_service"),
                "gripper_preopen_width_m": ParameterValue(
                    LaunchConfiguration("lid_gripper_preopen_width_m"),
                    value_type=float,
                ),
                "gripper_grasp_width_m": ParameterValue(
                    LaunchConfiguration("lid_gripper_grasp_width_m"),
                    value_type=float,
                ),
                "gripper_force_n": ParameterValue(
                    LaunchConfiguration("lid_gripper_force_n"),
                    value_type=float,
                ),
                "enable_lid_twist_after_grasp": ParameterValue(
                    LaunchConfiguration("enable_lid_twist_after_grasp"),
                    value_type=bool,
                ),
                "lid_twist_target_x_m": ParameterValue(
                    LaunchConfiguration("lid_twist_target_x_m"),
                    value_type=float,
                ),
                "lid_twist_target_y_m": ParameterValue(
                    LaunchConfiguration("lid_twist_target_y_m"),
                    value_type=float,
                ),
                "lid_twist_target_z_m": ParameterValue(
                    LaunchConfiguration("lid_twist_target_z_m"),
                    value_type=float,
                ),
                "lid_twist_rx": ParameterValue(
                    LaunchConfiguration("lid_twist_rx"),
                    value_type=float,
                ),
                "lid_twist_ry": ParameterValue(
                    LaunchConfiguration("lid_twist_ry"),
                    value_type=float,
                ),
                "lid_twist_rz": ParameterValue(
                    LaunchConfiguration("lid_twist_rz"),
                    value_type=float,
                ),
                "lid_twist_press_down_m": ParameterValue(
                    LaunchConfiguration("lid_twist_press_down_m"),
                    value_type=float,
                ),
                "lid_twist_rz_delta_deg": ParameterValue(
                    LaunchConfiguration("lid_twist_rz_delta_deg"),
                    value_type=float,
                ),
            }
        ],
        condition=IfCondition(enable_lid_pipeline),
    )

    # 11) Full cocktail orchestrator
    # 이 노드는 네가 추가할 full_cocktail_sequence_node.py가 있어야 실행된다.
    full_sequence_node = Node(
        package="azas_task_manager",
        executable="full_cocktail_sequence_node",
        name="full_cocktail_sequence_node",
        output="screen",
        parameters=[
            {
                "decision_topic": LaunchConfiguration("decision_topic"),
                "status_topic": LaunchConfiguration("full_status_topic"),
                "execute_real_motion": ParameterValue(
                    LaunchConfiguration("enable_hardware"),
                    value_type=bool,
                ),
                "service_prefix": LaunchConfiguration("service_prefix"),
            }
        ],
        condition=IfCondition(enable_full_sequence),
    )

    return LaunchDescription(
        [
            # Enable/disable groups
            DeclareLaunchArgument("enable_realsense", default_value="true"),
            DeclareLaunchArgument("enable_rg2", default_value="true"),
            DeclareLaunchArgument("enable_voice", default_value="true"),
            DeclareLaunchArgument("enable_yolo_cup", default_value="true"),
            DeclareLaunchArgument("enable_lid_pipeline", default_value="true"),
            DeclareLaunchArgument("enable_full_sequence", default_value="true"),

            # Hardware gate
            DeclareLaunchArgument("enable_hardware", default_value="false"),
            DeclareLaunchArgument("hardware_confirm", default_value=""),
            DeclareLaunchArgument(
                "allow_service_control_without_moveit",
                default_value="false",
            ),
            DeclareLaunchArgument("service_prefix", default_value="dsr01"),

            # RealSense
            DeclareLaunchArgument("realsense_camera_name", default_value="camera"),
            DeclareLaunchArgument("realsense_camera_namespace", default_value=""),
            DeclareLaunchArgument("realsense_enable_color", default_value="true"),
            DeclareLaunchArgument("realsense_enable_depth", default_value="true"),
            DeclareLaunchArgument("realsense_align_depth", default_value="true"),

            # RG2
            DeclareLaunchArgument("rg2_ip", default_value="192.168.1.1"),
            DeclareLaunchArgument("rg2_port", default_value="502"),
            DeclareLaunchArgument("rg2_connect", default_value="true"),
            DeclareLaunchArgument("gripper_set_service", default_value="/jarvis/rg2/set_width"),

            # Camera topics
            DeclareLaunchArgument(
                "color_topic",
                default_value="/camera/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
            ),
            DeclareLaunchArgument(
                "camera_source_frame",
                default_value="camera_color_optical_frame",
            ),
            DeclareLaunchArgument("target_frame", default_value="base_link"),

            # YOLO common
            DeclareLaunchArgument("device", default_value="cpu"),
            DeclareLaunchArgument("depth_window_size", default_value="7"),
            DeclareLaunchArgument("min_depth_m", default_value="0.15"),
            DeclareLaunchArgument("max_depth_m", default_value="2.0"),

            # Cup detection
            DeclareLaunchArgument("cup_model_path", default_value="/home/ssu/Azas/best.pt"),
            DeclareLaunchArgument("cup_confidence_threshold", default_value="0.35"),
            DeclareLaunchArgument("cup_target_class_names", default_value="cup,tumbler,bottle"),
            DeclareLaunchArgument("cup_selection_policy", default_value="largest_bbox"),
            DeclareLaunchArgument("cup_detection_topic", default_value="/azas/cup_detection"),
            DeclareLaunchArgument(
                "tumbler_pose_topic",
                default_value="/jarvis/tumbler_dispenser/tumbler_pose",
            ),
            DeclareLaunchArgument("require_upright_cup", default_value="true"),

            # TF
            DeclareLaunchArgument("require_tf", default_value="true"),
            DeclareLaunchArgument("transform_timeout_sec", default_value="0.2"),
            DeclareLaunchArgument("debug_pose_logging", default_value="false"),
            DeclareLaunchArgument("log_published_pose", default_value="false"),

            # Optional fixed camera TF
            DeclareLaunchArgument("publish_camera_base_tf", default_value="false"),
            DeclareLaunchArgument("camera_base_parent_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "camera_base_child_frame",
                default_value="camera_color_optical_frame",
            ),
            DeclareLaunchArgument("camera_base_tf_x", default_value="0.0"),
            DeclareLaunchArgument("camera_base_tf_y", default_value="0.0"),
            DeclareLaunchArgument("camera_base_tf_z", default_value="0.0"),
            DeclareLaunchArgument("camera_base_tf_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_base_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_base_tf_yaw", default_value="0.0"),

            # Hand-eye TF
            DeclareLaunchArgument("publish_hand_eye_tf", default_value="true"),
            DeclareLaunchArgument(
                "hand_eye_matrix_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("azas_perception"),
                        "config",
                        "T_gripper2camera.npy",
                    ]
                ),
            ),
            DeclareLaunchArgument("hand_eye_parent_frame", default_value="link_6"),
            DeclareLaunchArgument(
                "hand_eye_matrix_child_frame",
                default_value="camera_color_optical_frame",
            ),
            DeclareLaunchArgument(
                "hand_eye_published_child_frame",
                default_value="camera_link",
            ),
            DeclareLaunchArgument("hand_eye_translation_scale", default_value="0.001"),
            DeclareLaunchArgument("hand_eye_compose_with_existing_tf", default_value="true"),
            DeclareLaunchArgument("hand_eye_compose_timeout_sec", default_value="5.0"),

            # Voice
            DeclareLaunchArgument("use_live_stt", default_value="true"),
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
            DeclareLaunchArgument(
                "decision_topic",
                default_value="/azas/voice/confirmed_recipe_decision",
            ),

            # Lid detection
            DeclareLaunchArgument("lid_model_path", default_value="/home/ssu/Azas/best.pt"),
            DeclareLaunchArgument("lid_confidence_threshold", default_value="0.35"),
            DeclareLaunchArgument("lid_target_class_names", default_value="lid"),
            DeclareLaunchArgument("lid_selection_policy", default_value="highest_confidence"),
            DeclareLaunchArgument("lid_detection_topic", default_value="/azas/lid_detection"),
            DeclareLaunchArgument(
                "lid_pose_topic",
                default_value="/jarvis/lid_gripper/lid_pose",
            ),
            DeclareLaunchArgument(
                "lid_grip_request_topic",
                default_value="/jarvis/lid_gripper/grip_request",
            ),
            DeclareLaunchArgument("lid_marker_type", default_value="aruco"),
            DeclareLaunchArgument("require_lid_detection", default_value="true"),
            DeclareLaunchArgument("aruco_dictionary", default_value="DICT_4X4_50"),
            DeclareLaunchArgument("aruco_marker_id", default_value="-1"),
            DeclareLaunchArgument("require_lid_plane_normal", default_value="true"),
            DeclareLaunchArgument("show_lid_preview", default_value="false"),
            DeclareLaunchArgument("log_lid_detections", default_value="false"),

            # Lid grip / attach
            DeclareLaunchArgument("lid_line_velocity", default_value="15.0"),
            DeclareLaunchArgument("lid_line_acceleration", default_value="30.0"),
            DeclareLaunchArgument(
                "enable_lid_gripper_service_calls",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "execute_lid_gripper_on_pose",
                default_value="true",
            ),
            DeclareLaunchArgument("lid_gripper_preopen_width_m", default_value="0.075"),
            DeclareLaunchArgument("lid_gripper_grasp_width_m", default_value="0.030"),
            DeclareLaunchArgument("lid_gripper_force_n", default_value="25.0"),

            # 이 값들은 실제 컵홀더 위치 teach 후 반드시 수정해야 함.
            DeclareLaunchArgument("enable_lid_twist_after_grasp", default_value="false"),
            DeclareLaunchArgument("lid_twist_target_x_m", default_value="nan"),
            DeclareLaunchArgument("lid_twist_target_y_m", default_value="nan"),
            DeclareLaunchArgument("lid_twist_target_z_m", default_value="nan"),
            DeclareLaunchArgument("lid_twist_rx", default_value="nan"),
            DeclareLaunchArgument("lid_twist_ry", default_value="nan"),
            DeclareLaunchArgument("lid_twist_rz", default_value="nan"),
            DeclareLaunchArgument("lid_twist_press_down_m", default_value="0.0"),
            DeclareLaunchArgument("lid_twist_rz_delta_deg", default_value="-30.0"),

            # Full sequence status
            DeclareLaunchArgument(
                "full_status_topic",
                default_value="/azas/cocktail/full_status",
            ),

            realsense_launch,
            rg2_launch,
            yolo_cup_launch,
            camera_base_tf,
            hand_eye_tf,
            cup_pose_bridge,
            voice_launch,
            lid_detector,
            lid_pose_bridge,
            lid_grip_planner,
            full_sequence_node,
        ]
    )