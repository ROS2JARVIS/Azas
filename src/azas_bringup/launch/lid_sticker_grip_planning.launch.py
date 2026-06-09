import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def default_model_path():
    env_path = os.environ.get("AZAS_YOLO_MODEL_PATH") or os.environ.get("MODEL_PATH")
    candidates = [
        env_path,
        "/home/ssu/Downloads/best.pt",
        str(
            Path(__file__).resolve().parents[3]
            / "src"
            / "cocktail_robot_system"
            / "models"
            / "best.pt"
        ),
        "/home/ssu/ros2_ws/src/cocktail_robot_system/src/cocktail_robot_system/models/best.pt",
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return str(candidate)
    return "/home/ssu/Downloads/best.pt"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value=default_model_path()),
        DeclareLaunchArgument("color_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.35"),
        DeclareLaunchArgument("target_class_names", default_value="lid"),
        DeclareLaunchArgument("selection_policy", default_value="highest_confidence"),
        DeclareLaunchArgument("source_frame", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("require_tf", default_value="true"),
        DeclareLaunchArgument("transform_timeout_sec", default_value="0.2"),
        DeclareLaunchArgument("allow_latest_tf_fallback", default_value="true"),
        DeclareLaunchArgument("publish_hand_eye_tf", default_value="true"),
        DeclareLaunchArgument(
            "hand_eye_matrix_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("azas_perception"),
                "config",
                "T_gripper2camera.npy",
            ]),
        ),
        DeclareLaunchArgument("hand_eye_parent_frame", default_value="link_6"),
        DeclareLaunchArgument("hand_eye_matrix_child_frame", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("hand_eye_published_child_frame", default_value="camera_link"),
        DeclareLaunchArgument("hand_eye_translation_scale", default_value="0.001"),
        DeclareLaunchArgument("hand_eye_compose_with_existing_tf", default_value="true"),
        DeclareLaunchArgument("hand_eye_compose_timeout_sec", default_value="5.0"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("show_preview", default_value="true"),
        DeclareLaunchArgument("log_detections", default_value="false"),
        DeclareLaunchArgument("log_bridge_poses", default_value="false"),
        DeclareLaunchArgument("log_pose_plans", default_value="false"),
        DeclareLaunchArgument("log_korean_status", default_value="true"),
        DeclareLaunchArgument("log_json_status", default_value="false"),
        DeclareLaunchArgument("lid_detection_topic", default_value="/azas/lid_detection"),
        DeclareLaunchArgument("lid_pose_topic", default_value="/jarvis/lid_gripper/lid_pose"),
        DeclareLaunchArgument("grip_request_topic", default_value="/jarvis/lid_gripper/grip_request"),
        DeclareLaunchArgument("approach_pose_topic", default_value="/jarvis/lid_gripper/approach_pose"),
        DeclareLaunchArgument("grasp_pose_topic", default_value="/jarvis/lid_gripper/grasp_pose"),
        DeclareLaunchArgument("lift_pose_topic", default_value="/jarvis/lid_gripper/lift_pose"),
        DeclareLaunchArgument("status_topic", default_value="/jarvis/lid_gripper/status"),
        DeclareLaunchArgument("plan_on_pose", default_value="true"),
        DeclareLaunchArgument("depth_window_size", default_value="7"),
        DeclareLaunchArgument("min_depth_m", default_value="0.15"),
        DeclareLaunchArgument("max_depth_m", default_value="2.0"),
        DeclareLaunchArgument("marker_type", default_value="aruco"),
        DeclareLaunchArgument("require_lid_detection", default_value="true"),
        DeclareLaunchArgument("allow_aruco_only_after_grip_request", default_value="true"),
        DeclareLaunchArgument("aruco_only_after_grip_request_sec", default_value="20.0"),
        DeclareLaunchArgument("roi_padding_ratio", default_value="0.12"),
        DeclareLaunchArgument("red_min_area_px", default_value="80.0"),
        DeclareLaunchArgument("red_min_radius_px", default_value="4.0"),
        DeclareLaunchArgument("red_min_circularity", default_value="0.65"),
        DeclareLaunchArgument("aruco_dictionary", default_value="DICT_6X6_250"),
        DeclareLaunchArgument("aruco_marker_id", default_value="0"),
        DeclareLaunchArgument("aruco_fallback_markers", default_value="DICT_4X4_50:14"),
        DeclareLaunchArgument("aruco_marker_length_m", default_value="0.03"),
        DeclareLaunchArgument("use_aruco_axis_for_orientation", default_value="true"),
        DeclareLaunchArgument("aruco_finger_axis_quarter_turns", default_value="1"),
        DeclareLaunchArgument("plane_patch_radius_px", default_value="18"),
        DeclareLaunchArgument("min_plane_points", default_value="20"),
        DeclareLaunchArgument("max_plane_rmse_m", default_value="0.015"),
        DeclareLaunchArgument("approach_offset_m", default_value="0.08"),
        DeclareLaunchArgument("lift_offset_m", default_value="0.10"),
        DeclareLaunchArgument("surface_offset_m", default_value="0.0"),
        DeclareLaunchArgument("offset_axis", default_value="local_z"),
        DeclareLaunchArgument("tcp_grasp_offset_x_m", default_value="0.0"),
        DeclareLaunchArgument("tcp_grasp_offset_y_m", default_value="0.0"),
        DeclareLaunchArgument("tcp_grasp_offset_z_m", default_value="0.0"),
        DeclareLaunchArgument("min_grasp_z_m", default_value="0.02"),
        DeclareLaunchArgument("max_grasp_z_m", default_value="0.60"),
        DeclareLaunchArgument("enable_hardware", default_value="false"),
        DeclareLaunchArgument("hardware_confirm", default_value=""),
        DeclareLaunchArgument("allow_service_control_without_moveit", default_value="false"),
        DeclareLaunchArgument("service_prefix", default_value=""),
        DeclareLaunchArgument("rx", default_value="180.0"),
        DeclareLaunchArgument("ry", default_value="0.0"),
        DeclareLaunchArgument("rz", default_value="180.0"),
        DeclareLaunchArgument("use_lid_pose_yaw_for_pick", default_value="false"),
        DeclareLaunchArgument("lid_pose_yaw_axis", default_value="x"),
        DeclareLaunchArgument("lid_pose_yaw_offset_deg", default_value="0.0"),
        DeclareLaunchArgument("lid_pose_yaw_equivalence_deg", default_value="180.0"),
        DeclareLaunchArgument("line_velocity", default_value="15.0"),
        DeclareLaunchArgument("line_acceleration", default_value="30.0"),
        DeclareLaunchArgument("move_timeout_sec", default_value="10.0"),
        DeclareLaunchArgument("approach_lid_with_movej", default_value="false"),
        DeclareLaunchArgument("approach_movej_velocity", default_value="20.0"),
        DeclareLaunchArgument("approach_movej_acceleration", default_value="20.0"),
        DeclareLaunchArgument("lid_overhead_approach_enabled", default_value="false"),
        DeclareLaunchArgument("lid_overhead_min_z_m", default_value="0.22"),
        DeclareLaunchArgument("precheck_ikin", default_value="true"),
        DeclareLaunchArgument("ikin_sol_space", default_value="2"),
        DeclareLaunchArgument("ikin_timeout_sec", default_value="5.0"),
        DeclareLaunchArgument("verify_motion_reached", default_value="true"),
        DeclareLaunchArgument("motion_verify_timeout_sec", default_value="20.0"),
        DeclareLaunchArgument("motion_target_tolerance_m", default_value="0.015"),
        DeclareLaunchArgument("visual_refine_before_grasp", default_value="false"),
        DeclareLaunchArgument("visual_refine_sample_count", default_value="10"),
        DeclareLaunchArgument("visual_refine_timeout_sec", default_value="2.0"),
        DeclareLaunchArgument("visual_refine_min_sample_interval_sec", default_value="0.03"),
        DeclareLaunchArgument("visual_refine_max_yaw_std_deg", default_value="3.0"),
        DeclareLaunchArgument("visual_refine_max_position_std_m", default_value="0.005"),
        DeclareLaunchArgument("visual_refine_apply_xy", default_value="true"),
        DeclareLaunchArgument("visual_refine_apply_yaw", default_value="true"),
        DeclareLaunchArgument("visual_refine_fallback_to_initial_plan", default_value="true"),
        DeclareLaunchArgument("settle_seconds_before_grasp", default_value="0.5"),
        DeclareLaunchArgument("hold_seconds_after_grasp", default_value="0.2"),
        DeclareLaunchArgument("enable_lid_twist_after_grasp", default_value="false"),
        DeclareLaunchArgument("lid_twist_target_x_m", default_value="nan"),
        DeclareLaunchArgument("lid_twist_target_y_m", default_value="nan"),
        DeclareLaunchArgument("lid_twist_target_z_m", default_value="nan"),
        DeclareLaunchArgument("lid_twist_rx", default_value="nan"),
        DeclareLaunchArgument("lid_twist_ry", default_value="nan"),
        DeclareLaunchArgument("lid_twist_rz", default_value="nan"),
        DeclareLaunchArgument("lid_twist_use_force_control", default_value="false"),
        DeclareLaunchArgument("lid_twist_use_force_spiral", default_value="false"),
        DeclareLaunchArgument("lid_twist_press_down_m", default_value="0.0"),
        DeclareLaunchArgument("lid_twist_rz_delta_deg", default_value="-30.0"),
        DeclareLaunchArgument("lid_twist_turn_step_deg", default_value="90.0"),
        DeclareLaunchArgument("lid_twist_transfer_clearance_m", default_value="0.0"),
        DeclareLaunchArgument("lid_twist_release_lift_m", default_value="0.03"),
        DeclareLaunchArgument("lid_twist_min_z_m", default_value="0.02"),
        DeclareLaunchArgument("lid_twist_max_z_m", default_value="0.60"),
        DeclareLaunchArgument("lid_twist_transfer_max_z_m", default_value="0.60"),
        DeclareLaunchArgument("lid_twist_transfer_velocity", default_value="30.0"),
        DeclareLaunchArgument("lid_twist_press_velocity", default_value="8.0"),
        DeclareLaunchArgument("lid_twist_turn_velocity", default_value="8.0"),
        DeclareLaunchArgument("lid_twist_acceleration", default_value="10.0"),
        DeclareLaunchArgument("lid_twist_hold_seconds_before_turn", default_value="0.3"),
        DeclareLaunchArgument("lid_twist_hold_seconds_after_turn", default_value="0.5"),
        DeclareLaunchArgument("lid_twist_down_force_n", default_value="8.0"),
        DeclareLaunchArgument("lid_twist_force_ref", default_value="base"),
        DeclareLaunchArgument("lid_twist_force_rotation_mode", default_value="movel"),
        DeclareLaunchArgument("lid_twist_force_service_timeout_sec", default_value="5.0"),
        DeclareLaunchArgument("lid_twist_regrip_cycles", default_value="1"),
        DeclareLaunchArgument("lid_twist_regrip_turn_deg", default_value="nan"),
        DeclareLaunchArgument("lid_twist_regrip_reset_between_cycles", default_value="false"),
        DeclareLaunchArgument("lid_twist_regrip_gripper_wait_sec", default_value="0.5"),
        DeclareLaunchArgument("lid_twist_force_settle_seconds", default_value="0.4"),
        DeclareLaunchArgument("lid_twist_force_release_time", default_value="0.2"),
        DeclareLaunchArgument("lid_twist_preseat_periodic_before_turn", default_value="false"),
        DeclareLaunchArgument("lid_twist_preseat_periodic_x_amp_mm", default_value="0.0"),
        DeclareLaunchArgument("lid_twist_preseat_periodic_y_amp_mm", default_value="0.0"),
        DeclareLaunchArgument("lid_twist_preseat_periodic_z_amp_mm", default_value="1.0"),
        DeclareLaunchArgument("lid_twist_preseat_periodic_rx_amp_deg", default_value="0.0"),
        DeclareLaunchArgument("lid_twist_preseat_periodic_ry_amp_deg", default_value="3.0"),
        DeclareLaunchArgument("lid_twist_preseat_periodic_rz_amp_deg", default_value="5.0"),
        DeclareLaunchArgument("lid_twist_preseat_periodic_period_sec", default_value="1.0"),
        DeclareLaunchArgument("lid_twist_preseat_periodic_acc_time_sec", default_value="0.2"),
        DeclareLaunchArgument("lid_twist_preseat_periodic_repeat", default_value="2"),
        DeclareLaunchArgument("lid_twist_preseat_periodic_ref", default_value="tool"),
        DeclareLaunchArgument("lid_twist_compliance_x_stiffness", default_value="3000.0"),
        DeclareLaunchArgument("lid_twist_compliance_y_stiffness", default_value="3000.0"),
        DeclareLaunchArgument("lid_twist_compliance_z_stiffness", default_value="300.0"),
        DeclareLaunchArgument("lid_twist_compliance_rx_stiffness", default_value="200.0"),
        DeclareLaunchArgument("lid_twist_compliance_ry_stiffness", default_value="200.0"),
        DeclareLaunchArgument("lid_twist_compliance_rz_stiffness", default_value="200.0"),
        DeclareLaunchArgument("motion_target_orientation_tolerance_deg", default_value="3.0"),
        DeclareLaunchArgument("enable_gripper_service_calls", default_value="false"),
        DeclareLaunchArgument("execute_gripper_on_pose", default_value="false"),
        DeclareLaunchArgument("gripper_set_service", default_value="/jarvis/rg2/set_width"),
        DeclareLaunchArgument("gripper_preopen_width_m", default_value="nan"),
        DeclareLaunchArgument("gripper_grasp_width_m", default_value="nan"),
        DeclareLaunchArgument("gripper_force_n", default_value="nan"),
        DeclareLaunchArgument("gripper_wait_timeout_sec", default_value="10.0"),
        DeclareLaunchArgument("continue_after_gripper_grasp_failure", default_value="false"),
        DeclareLaunchArgument("gripper_grasp_failure_wait_sec", default_value="2.0"),
        Node(
            package="azas_perception",
            executable="hand_eye_static_tf_node",
            name="hand_eye_static_tf_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_hand_eye_tf")),
            parameters=[{
                "matrix_path": LaunchConfiguration("hand_eye_matrix_path"),
                "parent_frame": LaunchConfiguration("hand_eye_parent_frame"),
                "matrix_child_frame": LaunchConfiguration("hand_eye_matrix_child_frame"),
                "published_child_frame": LaunchConfiguration("hand_eye_published_child_frame"),
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
            }],
        ),
        Node(
            package="azas_perception",
            executable="lid_sticker_detector_node",
            name="lid_sticker_detector_node",
            output="screen",
            parameters=[{
                "model_path": LaunchConfiguration("model_path"),
                "color_topic": LaunchConfiguration("color_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "output_topic": LaunchConfiguration("lid_detection_topic"),
                "grip_request_topic": LaunchConfiguration("grip_request_topic"),
                "confidence_threshold": ParameterValue(
                    LaunchConfiguration("confidence_threshold"),
                    value_type=float,
                ),
                "target_class_names": LaunchConfiguration("target_class_names"),
                "selection_policy": LaunchConfiguration("selection_policy"),
                "source_frame": LaunchConfiguration("source_frame"),
                "depth_window_size": ParameterValue(
                    LaunchConfiguration("depth_window_size"),
                    value_type=int,
                ),
                "min_depth_m": ParameterValue(LaunchConfiguration("min_depth_m"), value_type=float),
                "max_depth_m": ParameterValue(LaunchConfiguration("max_depth_m"), value_type=float),
                "marker_type": LaunchConfiguration("marker_type"),
                "require_lid_detection": ParameterValue(
                    LaunchConfiguration("require_lid_detection"),
                    value_type=bool,
                ),
                "allow_aruco_only_after_grip_request": ParameterValue(
                    LaunchConfiguration("allow_aruco_only_after_grip_request"),
                    value_type=bool,
                ),
                "aruco_only_after_grip_request_sec": ParameterValue(
                    LaunchConfiguration("aruco_only_after_grip_request_sec"),
                    value_type=float,
                ),
                "roi_padding_ratio": ParameterValue(
                    LaunchConfiguration("roi_padding_ratio"),
                    value_type=float,
                ),
                "red_min_area_px": ParameterValue(
                    LaunchConfiguration("red_min_area_px"),
                    value_type=float,
                ),
                "red_min_radius_px": ParameterValue(
                    LaunchConfiguration("red_min_radius_px"),
                    value_type=float,
                ),
                "red_min_circularity": ParameterValue(
                    LaunchConfiguration("red_min_circularity"),
                    value_type=float,
                ),
                "aruco_dictionary": LaunchConfiguration("aruco_dictionary"),
                "aruco_marker_id": ParameterValue(
                    LaunchConfiguration("aruco_marker_id"),
                    value_type=int,
                ),
                "aruco_fallback_markers": LaunchConfiguration("aruco_fallback_markers"),
                "aruco_marker_length_m": ParameterValue(
                    LaunchConfiguration("aruco_marker_length_m"),
                    value_type=float,
                ),
                "use_aruco_axis_for_orientation": ParameterValue(
                    LaunchConfiguration("use_aruco_axis_for_orientation"),
                    value_type=bool,
                ),
                "aruco_finger_axis_quarter_turns": ParameterValue(
                    LaunchConfiguration("aruco_finger_axis_quarter_turns"),
                    value_type=int,
                ),
                "plane_patch_radius_px": ParameterValue(
                    LaunchConfiguration("plane_patch_radius_px"),
                    value_type=int,
                ),
                "min_plane_points": ParameterValue(
                    LaunchConfiguration("min_plane_points"),
                    value_type=int,
                ),
                "max_plane_rmse_m": ParameterValue(
                    LaunchConfiguration("max_plane_rmse_m"),
                    value_type=float,
                ),
                "device": LaunchConfiguration("device"),
                "log_detections": ParameterValue(
                    LaunchConfiguration("log_detections"),
                    value_type=bool,
                ),
                "show_preview": ParameterValue(LaunchConfiguration("show_preview"), value_type=bool),
            }],
        ),
        Node(
            package="azas_perception",
            executable="cup_detection_pose_bridge_node",
            name="lid_detection_pose_bridge_node",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("lid_detection_topic"),
                "output_topic": LaunchConfiguration("lid_pose_topic"),
                "min_confidence": ParameterValue(
                    LaunchConfiguration("confidence_threshold"),
                    value_type=float,
                ),
                "require_status_prefix": "detected:lid",
                "require_upright_status": False,
                "target_frame": LaunchConfiguration("target_frame"),
                "require_tf": ParameterValue(LaunchConfiguration("require_tf"), value_type=bool),
                "source_frame": LaunchConfiguration("source_frame"),
                "transform_timeout_sec": ParameterValue(
                    LaunchConfiguration("transform_timeout_sec"),
                    value_type=float,
                ),
                "tf_timeout_sec": ParameterValue(
                    LaunchConfiguration("transform_timeout_sec"),
                    value_type=float,
                ),
                "allow_latest_tf_fallback": ParameterValue(
                    LaunchConfiguration("allow_latest_tf_fallback"),
                    value_type=bool,
                ),
                "log_published_pose": ParameterValue(
                    LaunchConfiguration("log_bridge_poses"),
                    value_type=bool,
                ),
            }],
        ),
        Node(
            package="azas_motion",
            executable="lid_grip_planner_node",
            name="lid_grip_planner_node",
            output="screen",
            parameters=[{
                "lid_pose_topic": LaunchConfiguration("lid_pose_topic"),
                "trigger_topic": LaunchConfiguration("grip_request_topic"),
                "approach_pose_topic": LaunchConfiguration("approach_pose_topic"),
                "grasp_pose_topic": LaunchConfiguration("grasp_pose_topic"),
                "lift_pose_topic": LaunchConfiguration("lift_pose_topic"),
                "status_topic": LaunchConfiguration("status_topic"),
                "plan_on_pose": ParameterValue(LaunchConfiguration("plan_on_pose"), value_type=bool),
                "log_pose_plans": ParameterValue(
                    LaunchConfiguration("log_pose_plans"),
                    value_type=bool,
                ),
                "log_korean_status": ParameterValue(
                    LaunchConfiguration("log_korean_status"),
                    value_type=bool,
                ),
                "log_json_status": ParameterValue(
                    LaunchConfiguration("log_json_status"),
                    value_type=bool,
                ),
                "approach_offset_m": ParameterValue(
                    LaunchConfiguration("approach_offset_m"),
                    value_type=float,
                ),
                "lift_offset_m": ParameterValue(LaunchConfiguration("lift_offset_m"), value_type=float),
                "surface_offset_m": ParameterValue(
                    LaunchConfiguration("surface_offset_m"),
                    value_type=float,
                ),
                "offset_axis": ParameterValue(
                    LaunchConfiguration("offset_axis"),
                    value_type=str,
                ),
                "tcp_grasp_offset_x_m": ParameterValue(
                    LaunchConfiguration("tcp_grasp_offset_x_m"),
                    value_type=float,
                ),
                "tcp_grasp_offset_y_m": ParameterValue(
                    LaunchConfiguration("tcp_grasp_offset_y_m"),
                    value_type=float,
                ),
                "tcp_grasp_offset_z_m": ParameterValue(
                    LaunchConfiguration("tcp_grasp_offset_z_m"),
                    value_type=float,
                ),
                "min_grasp_z_m": ParameterValue(LaunchConfiguration("min_grasp_z_m"), value_type=float),
                "max_grasp_z_m": ParameterValue(LaunchConfiguration("max_grasp_z_m"), value_type=float),
                "enable_hardware": ParameterValue(
                    LaunchConfiguration("enable_hardware"),
                    value_type=bool,
                ),
                "hardware_confirm": ParameterValue(
                    LaunchConfiguration("hardware_confirm"),
                    value_type=str,
                ),
                "allow_service_control_without_moveit": ParameterValue(
                    LaunchConfiguration("allow_service_control_without_moveit"),
                    value_type=bool,
                ),
                "service_prefix": ParameterValue(
                    LaunchConfiguration("service_prefix"),
                    value_type=str,
                ),
                "rx": ParameterValue(LaunchConfiguration("rx"), value_type=float),
                "ry": ParameterValue(LaunchConfiguration("ry"), value_type=float),
                "rz": ParameterValue(LaunchConfiguration("rz"), value_type=float),
                "use_lid_pose_yaw_for_pick": ParameterValue(
                    LaunchConfiguration("use_lid_pose_yaw_for_pick"),
                    value_type=bool,
                ),
                "lid_pose_yaw_axis": ParameterValue(
                    LaunchConfiguration("lid_pose_yaw_axis"),
                    value_type=str,
                ),
                "lid_pose_yaw_offset_deg": ParameterValue(
                    LaunchConfiguration("lid_pose_yaw_offset_deg"),
                    value_type=float,
                ),
                "lid_pose_yaw_equivalence_deg": ParameterValue(
                    LaunchConfiguration("lid_pose_yaw_equivalence_deg"),
                    value_type=float,
                ),
                "line_velocity": ParameterValue(
                    LaunchConfiguration("line_velocity"),
                    value_type=float,
                ),
                "line_acceleration": ParameterValue(
                    LaunchConfiguration("line_acceleration"),
                    value_type=float,
                ),
                "move_timeout_sec": ParameterValue(
                    LaunchConfiguration("move_timeout_sec"),
                    value_type=float,
                ),
                "approach_lid_with_movej": ParameterValue(
                    LaunchConfiguration("approach_lid_with_movej"),
                    value_type=bool,
                ),
                "approach_movej_velocity": ParameterValue(
                    LaunchConfiguration("approach_movej_velocity"),
                    value_type=float,
                ),
                "approach_movej_acceleration": ParameterValue(
                    LaunchConfiguration("approach_movej_acceleration"),
                    value_type=float,
                ),
                "lid_overhead_approach_enabled": ParameterValue(
                    LaunchConfiguration("lid_overhead_approach_enabled"),
                    value_type=bool,
                ),
                "lid_overhead_min_z_m": ParameterValue(
                    LaunchConfiguration("lid_overhead_min_z_m"),
                    value_type=float,
                ),
                "precheck_ikin": ParameterValue(
                    LaunchConfiguration("precheck_ikin"),
                    value_type=bool,
                ),
                "ikin_sol_space": ParameterValue(
                    LaunchConfiguration("ikin_sol_space"),
                    value_type=int,
                ),
                "ikin_timeout_sec": ParameterValue(
                    LaunchConfiguration("ikin_timeout_sec"),
                    value_type=float,
                ),
                "verify_motion_reached": ParameterValue(
                    LaunchConfiguration("verify_motion_reached"),
                    value_type=bool,
                ),
                "motion_verify_timeout_sec": ParameterValue(
                    LaunchConfiguration("motion_verify_timeout_sec"),
                    value_type=float,
                ),
                "motion_target_tolerance_m": ParameterValue(
                    LaunchConfiguration("motion_target_tolerance_m"),
                    value_type=float,
                ),
                "visual_refine_before_grasp": ParameterValue(
                    LaunchConfiguration("visual_refine_before_grasp"),
                    value_type=bool,
                ),
                "visual_refine_sample_count": ParameterValue(
                    LaunchConfiguration("visual_refine_sample_count"),
                    value_type=int,
                ),
                "visual_refine_timeout_sec": ParameterValue(
                    LaunchConfiguration("visual_refine_timeout_sec"),
                    value_type=float,
                ),
                "visual_refine_min_sample_interval_sec": ParameterValue(
                    LaunchConfiguration("visual_refine_min_sample_interval_sec"),
                    value_type=float,
                ),
                "visual_refine_max_yaw_std_deg": ParameterValue(
                    LaunchConfiguration("visual_refine_max_yaw_std_deg"),
                    value_type=float,
                ),
                "visual_refine_max_position_std_m": ParameterValue(
                    LaunchConfiguration("visual_refine_max_position_std_m"),
                    value_type=float,
                ),
                "visual_refine_apply_xy": ParameterValue(
                    LaunchConfiguration("visual_refine_apply_xy"),
                    value_type=bool,
                ),
                "visual_refine_apply_yaw": ParameterValue(
                    LaunchConfiguration("visual_refine_apply_yaw"),
                    value_type=bool,
                ),
                "visual_refine_fallback_to_initial_plan": ParameterValue(
                    LaunchConfiguration("visual_refine_fallback_to_initial_plan"),
                    value_type=bool,
                ),
                "settle_seconds_before_grasp": ParameterValue(
                    LaunchConfiguration("settle_seconds_before_grasp"),
                    value_type=float,
                ),
                "hold_seconds_after_grasp": ParameterValue(
                    LaunchConfiguration("hold_seconds_after_grasp"),
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
                "lid_twist_use_force_control": ParameterValue(
                    LaunchConfiguration("lid_twist_use_force_control"),
                    value_type=bool,
                ),
                "lid_twist_use_force_spiral": ParameterValue(
                    LaunchConfiguration("lid_twist_use_force_spiral"),
                    value_type=bool,
                ),
                "lid_twist_press_down_m": ParameterValue(
                    LaunchConfiguration("lid_twist_press_down_m"),
                    value_type=float,
                ),
                "lid_twist_rz_delta_deg": ParameterValue(
                    LaunchConfiguration("lid_twist_rz_delta_deg"),
                    value_type=float,
                ),
                "lid_twist_turn_step_deg": ParameterValue(
                    LaunchConfiguration("lid_twist_turn_step_deg"),
                    value_type=float,
                ),
                "lid_twist_transfer_clearance_m": ParameterValue(
                    LaunchConfiguration("lid_twist_transfer_clearance_m"),
                    value_type=float,
                ),
                "lid_twist_release_lift_m": ParameterValue(
                    LaunchConfiguration("lid_twist_release_lift_m"),
                    value_type=float,
                ),
                "lid_twist_min_z_m": ParameterValue(
                    LaunchConfiguration("lid_twist_min_z_m"),
                    value_type=float,
                ),
                "lid_twist_max_z_m": ParameterValue(
                    LaunchConfiguration("lid_twist_max_z_m"),
                    value_type=float,
                ),
                "lid_twist_transfer_max_z_m": ParameterValue(
                    LaunchConfiguration("lid_twist_transfer_max_z_m"),
                    value_type=float,
                ),
                "lid_twist_transfer_velocity": ParameterValue(
                    LaunchConfiguration("lid_twist_transfer_velocity"),
                    value_type=float,
                ),
                "lid_twist_press_velocity": ParameterValue(
                    LaunchConfiguration("lid_twist_press_velocity"),
                    value_type=float,
                ),
                "lid_twist_turn_velocity": ParameterValue(
                    LaunchConfiguration("lid_twist_turn_velocity"),
                    value_type=float,
                ),
                "lid_twist_acceleration": ParameterValue(
                    LaunchConfiguration("lid_twist_acceleration"),
                    value_type=float,
                ),
                "lid_twist_hold_seconds_before_turn": ParameterValue(
                    LaunchConfiguration("lid_twist_hold_seconds_before_turn"),
                    value_type=float,
                ),
                "lid_twist_hold_seconds_after_turn": ParameterValue(
                    LaunchConfiguration("lid_twist_hold_seconds_after_turn"),
                    value_type=float,
                ),
                "lid_twist_down_force_n": ParameterValue(
                    LaunchConfiguration("lid_twist_down_force_n"),
                    value_type=float,
                ),
                "lid_twist_force_ref": ParameterValue(
                    LaunchConfiguration("lid_twist_force_ref"),
                    value_type=str,
                ),
                "lid_twist_force_rotation_mode": ParameterValue(
                    LaunchConfiguration("lid_twist_force_rotation_mode"),
                    value_type=str,
                ),
                "lid_twist_force_service_timeout_sec": ParameterValue(
                    LaunchConfiguration("lid_twist_force_service_timeout_sec"),
                    value_type=float,
                ),
                "lid_twist_regrip_cycles": ParameterValue(
                    LaunchConfiguration("lid_twist_regrip_cycles"),
                    value_type=int,
                ),
                "lid_twist_regrip_turn_deg": ParameterValue(
                    LaunchConfiguration("lid_twist_regrip_turn_deg"),
                    value_type=float,
                ),
                "lid_twist_regrip_reset_between_cycles": ParameterValue(
                    LaunchConfiguration("lid_twist_regrip_reset_between_cycles"),
                    value_type=bool,
                ),
                "lid_twist_regrip_gripper_wait_sec": ParameterValue(
                    LaunchConfiguration("lid_twist_regrip_gripper_wait_sec"),
                    value_type=float,
                ),
                "lid_twist_force_settle_seconds": ParameterValue(
                    LaunchConfiguration("lid_twist_force_settle_seconds"),
                    value_type=float,
                ),
                "lid_twist_force_release_time": ParameterValue(
                    LaunchConfiguration("lid_twist_force_release_time"),
                    value_type=float,
                ),
                "lid_twist_preseat_periodic_before_turn": ParameterValue(
                    LaunchConfiguration("lid_twist_preseat_periodic_before_turn"),
                    value_type=bool,
                ),
                "lid_twist_preseat_periodic_x_amp_mm": ParameterValue(
                    LaunchConfiguration("lid_twist_preseat_periodic_x_amp_mm"),
                    value_type=float,
                ),
                "lid_twist_preseat_periodic_y_amp_mm": ParameterValue(
                    LaunchConfiguration("lid_twist_preseat_periodic_y_amp_mm"),
                    value_type=float,
                ),
                "lid_twist_preseat_periodic_z_amp_mm": ParameterValue(
                    LaunchConfiguration("lid_twist_preseat_periodic_z_amp_mm"),
                    value_type=float,
                ),
                "lid_twist_preseat_periodic_rx_amp_deg": ParameterValue(
                    LaunchConfiguration("lid_twist_preseat_periodic_rx_amp_deg"),
                    value_type=float,
                ),
                "lid_twist_preseat_periodic_ry_amp_deg": ParameterValue(
                    LaunchConfiguration("lid_twist_preseat_periodic_ry_amp_deg"),
                    value_type=float,
                ),
                "lid_twist_preseat_periodic_rz_amp_deg": ParameterValue(
                    LaunchConfiguration("lid_twist_preseat_periodic_rz_amp_deg"),
                    value_type=float,
                ),
                "lid_twist_preseat_periodic_period_sec": ParameterValue(
                    LaunchConfiguration("lid_twist_preseat_periodic_period_sec"),
                    value_type=float,
                ),
                "lid_twist_preseat_periodic_acc_time_sec": ParameterValue(
                    LaunchConfiguration("lid_twist_preseat_periodic_acc_time_sec"),
                    value_type=float,
                ),
                "lid_twist_preseat_periodic_repeat": ParameterValue(
                    LaunchConfiguration("lid_twist_preseat_periodic_repeat"),
                    value_type=int,
                ),
                "lid_twist_preseat_periodic_ref": ParameterValue(
                    LaunchConfiguration("lid_twist_preseat_periodic_ref"),
                    value_type=str,
                ),
                "lid_twist_compliance_x_stiffness": ParameterValue(
                    LaunchConfiguration("lid_twist_compliance_x_stiffness"),
                    value_type=float,
                ),
                "lid_twist_compliance_y_stiffness": ParameterValue(
                    LaunchConfiguration("lid_twist_compliance_y_stiffness"),
                    value_type=float,
                ),
                "lid_twist_compliance_z_stiffness": ParameterValue(
                    LaunchConfiguration("lid_twist_compliance_z_stiffness"),
                    value_type=float,
                ),
                "lid_twist_compliance_rx_stiffness": ParameterValue(
                    LaunchConfiguration("lid_twist_compliance_rx_stiffness"),
                    value_type=float,
                ),
                "lid_twist_compliance_ry_stiffness": ParameterValue(
                    LaunchConfiguration("lid_twist_compliance_ry_stiffness"),
                    value_type=float,
                ),
                "lid_twist_compliance_rz_stiffness": ParameterValue(
                    LaunchConfiguration("lid_twist_compliance_rz_stiffness"),
                    value_type=float,
                ),
                "motion_target_orientation_tolerance_deg": ParameterValue(
                    LaunchConfiguration("motion_target_orientation_tolerance_deg"),
                    value_type=float,
                ),
                "enable_gripper_service_calls": ParameterValue(
                    LaunchConfiguration("enable_gripper_service_calls"),
                    value_type=bool,
                ),
                "execute_gripper_on_pose": ParameterValue(
                    LaunchConfiguration("execute_gripper_on_pose"),
                    value_type=bool,
                ),
                "gripper_set_service": LaunchConfiguration("gripper_set_service"),
                "gripper_preopen_width_m": ParameterValue(
                    LaunchConfiguration("gripper_preopen_width_m"),
                    value_type=float,
                ),
                "gripper_grasp_width_m": ParameterValue(
                    LaunchConfiguration("gripper_grasp_width_m"),
                    value_type=float,
                ),
                "gripper_force_n": ParameterValue(
                    LaunchConfiguration("gripper_force_n"),
                    value_type=float,
                ),
                "gripper_wait_timeout_sec": ParameterValue(
                    LaunchConfiguration("gripper_wait_timeout_sec"),
                    value_type=float,
                ),
                "continue_after_gripper_grasp_failure": ParameterValue(
                    LaunchConfiguration("continue_after_gripper_grasp_failure"),
                    value_type=bool,
                ),
                "gripper_grasp_failure_wait_sec": ParameterValue(
                    LaunchConfiguration("gripper_grasp_failure_wait_sec"),
                    value_type=float,
                ),
            }],
        ),
    ])
