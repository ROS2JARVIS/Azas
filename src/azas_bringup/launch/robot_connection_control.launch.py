from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_realsense = LaunchConfiguration("enable_realsense")
    enable_rg2 = LaunchConfiguration("enable_rg2")

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])
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

    rg2_trigger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("jarvis"), "launch", "rg2_trigger.launch.py"])
        ),
        launch_arguments={
            "ip": LaunchConfiguration("ip"),
            "port": LaunchConfiguration("port"),
            "connect": LaunchConfiguration("connect"),
        }.items(),
        condition=IfCondition(enable_rg2),
    )

    yolo_to_floor_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("azas_bringup"), "launch", "yolo_to_floor_place.launch.py"])
        ),
        launch_arguments={
            "selected_dispenser_id": LaunchConfiguration("selected_dispenser_id"),
            "model_path": LaunchConfiguration("model_path"),
            "color_topic": LaunchConfiguration("color_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "confidence_threshold": LaunchConfiguration("confidence_threshold"),
            "target_class_names": LaunchConfiguration("target_class_names"),
            "selection_policy": LaunchConfiguration("selection_policy"),
            "depth_window_size": LaunchConfiguration("depth_window_size"),
            "min_depth_m": LaunchConfiguration("min_depth_m"),
            "max_depth_m": LaunchConfiguration("max_depth_m"),
            "device": LaunchConfiguration("device"),
            "run_yolo": LaunchConfiguration("run_yolo"),
            "enable_hardware": LaunchConfiguration("enable_hardware"),
            "hardware_confirm": LaunchConfiguration("hardware_confirm"),
            "allow_service_control_without_moveit": LaunchConfiguration("allow_service_control_without_moveit"),
            "service_prefix": LaunchConfiguration("service_prefix"),
            "execution_stage": LaunchConfiguration("execution_stage"),
            "place_mouth_under_outlet": LaunchConfiguration("place_mouth_under_outlet"),
            "outlet_mouth_clearance": LaunchConfiguration("outlet_mouth_clearance"),
            "gripper_open_service": LaunchConfiguration("gripper_open_service"),
            "gripper_close_service": LaunchConfiguration("gripper_close_service"),
            "gripper_set_service": LaunchConfiguration("gripper_set_service"),
            "tumbler_pose_wait_timeout": LaunchConfiguration("tumbler_pose_wait_timeout"),
            "tumbler_pose_target_frame": LaunchConfiguration("tumbler_pose_target_frame"),
            "require_tumbler_pose_tf": LaunchConfiguration("require_tumbler_pose_tf"),
            "source_frame": LaunchConfiguration("source_frame"),
            "transform_timeout_sec": LaunchConfiguration("transform_timeout_sec"),
            "debug_pose_logging": LaunchConfiguration("debug_pose_logging"),
            "publish_camera_base_tf": LaunchConfiguration("publish_camera_base_tf"),
            "camera_base_parent_frame": LaunchConfiguration("camera_base_parent_frame"),
            "camera_base_child_frame": LaunchConfiguration("camera_base_child_frame"),
            "camera_base_tf_parent_frame": LaunchConfiguration("camera_base_tf_parent_frame"),
            "camera_base_tf_child_frame": LaunchConfiguration("camera_base_tf_child_frame"),
            "camera_base_tf_x": LaunchConfiguration("camera_base_tf_x"),
            "camera_base_tf_y": LaunchConfiguration("camera_base_tf_y"),
            "camera_base_tf_z": LaunchConfiguration("camera_base_tf_z"),
            "camera_base_tf_roll": LaunchConfiguration("camera_base_tf_roll"),
            "camera_base_tf_pitch": LaunchConfiguration("camera_base_tf_pitch"),
            "camera_base_tf_yaw": LaunchConfiguration("camera_base_tf_yaw"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("selected_dispenser_id", default_value="1"),
        DeclareLaunchArgument("model_path", default_value="/home/ssu/Downloads/best.pt"),
        DeclareLaunchArgument("color_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.35"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("run_yolo", default_value="true"),
        DeclareLaunchArgument("enable_realsense", default_value="false"),
        DeclareLaunchArgument("realsense_camera_name", default_value="camera"),
        DeclareLaunchArgument("realsense_camera_namespace", default_value=""),
        DeclareLaunchArgument("realsense_enable_color", default_value="true"),
        DeclareLaunchArgument("realsense_enable_depth", default_value="true"),
        DeclareLaunchArgument("realsense_align_depth", default_value="true"),
        DeclareLaunchArgument("ip", default_value="192.168.1.1"),
        DeclareLaunchArgument("port", default_value="502"),
        DeclareLaunchArgument("connect", default_value="true"),
        DeclareLaunchArgument("enable_rg2", default_value="true"),
        DeclareLaunchArgument("enable_hardware", default_value="false"),
        DeclareLaunchArgument("hardware_confirm", default_value=""),
        DeclareLaunchArgument("allow_service_control_without_moveit", default_value="false"),
        DeclareLaunchArgument("service_prefix", default_value=""),
        DeclareLaunchArgument("execution_stage", default_value="full"),
        DeclareLaunchArgument("place_mouth_under_outlet", default_value="false"),
        DeclareLaunchArgument("outlet_mouth_clearance", default_value="0.0"),
        DeclareLaunchArgument("gripper_open_service", default_value="/jarvis/rg2/open"),
        DeclareLaunchArgument("gripper_close_service", default_value="/jarvis/rg2/close"),
        DeclareLaunchArgument("gripper_set_service", default_value="/jarvis/rg2/set_width"),
        DeclareLaunchArgument("tumbler_pose_wait_timeout", default_value="30.0"),
        DeclareLaunchArgument("tumbler_pose_target_frame", default_value="base_link"),
        DeclareLaunchArgument("require_tumbler_pose_tf", default_value="true"),
        DeclareLaunchArgument("source_frame", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("transform_timeout_sec", default_value="0.2"),
        DeclareLaunchArgument("debug_pose_logging", default_value="false"),
        DeclareLaunchArgument("target_class_names", default_value="cup,tumbler,bottle"),
        DeclareLaunchArgument("selection_policy", default_value="largest_bbox"),
        DeclareLaunchArgument("depth_window_size", default_value="7"),
        DeclareLaunchArgument("min_depth_m", default_value="0.15"),
        DeclareLaunchArgument("max_depth_m", default_value="2.0"),
        DeclareLaunchArgument("publish_camera_base_tf", default_value="false"),
        DeclareLaunchArgument("camera_base_parent_frame", default_value="base_link"),
        DeclareLaunchArgument("camera_base_child_frame", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("camera_base_tf_parent_frame", default_value=LaunchConfiguration("camera_base_parent_frame")),
        DeclareLaunchArgument("camera_base_tf_child_frame", default_value=LaunchConfiguration("camera_base_child_frame")),
        DeclareLaunchArgument("camera_base_tf_x", default_value="0.0"),
        DeclareLaunchArgument("camera_base_tf_y", default_value="0.0"),
        DeclareLaunchArgument("camera_base_tf_z", default_value="0.0"),
        DeclareLaunchArgument("camera_base_tf_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_base_tf_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_base_tf_yaw", default_value="0.0"),
        realsense_launch,
        rg2_trigger_launch,
        yolo_to_floor_place_launch,
    ])
