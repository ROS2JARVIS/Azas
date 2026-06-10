from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("azas_cup_uprighting"), "config", "moveit_py.yaml"]
    )

    parameters = [moveit_py_params]
    if not _as_bool(LaunchConfiguration("preview_only").perform(context)):
        missing_packages = []
        for package_name in ("dsr_moveit_config_m0609", "moveit_py"):
            try:
                get_package_share_directory(package_name)
            except PackageNotFoundError:
                missing_packages.append(package_name)
        if missing_packages:
            raise RuntimeError(
                "Cannot start yolo_cup_uprighting with preview_only=false. "
                "Missing required motion package(s): "
                f"{', '.join(missing_packages)}. "
                "Run preview_only=true for camera/YOLO preview, or source/install the Doosan MoveIt stack before enabling motion."
            )
        moveit_config = (
            MoveItConfigsBuilder(
                robot_name="m0609",
                package_name="dsr_moveit_config_m0609",
            )
            .robot_description()
            .robot_description_semantic(file_path="config/dsr.srdf")
            .robot_description_kinematics()
            .joint_limits()
            .trajectory_execution()
            .planning_scene_monitor()
            .sensors_3d()
            .to_moveit_configs()
        )
        parameters.insert(0, moveit_config.to_dict())

    return [
        Node(
            package="azas_cup_uprighting",
            executable="yolo_cup_uprighting",
            name="yolo_cup_uprighting_py",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    preview_only = LaunchConfiguration("preview_only")
    show_window = LaunchConfiguration("show_window")
    use_mock_vision = LaunchConfiguration("use_mock_vision")
    max_velocity_scale = LaunchConfiguration("max_velocity_scale")
    max_acceleration_scale = LaunchConfiguration("max_acceleration_scale")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value="/home/ssu/Azas/local_models/best.pt",
            description="YOLO model path. Use tools/setup/link_yolo_model.sh to prepare this.",
        ),
        DeclareLaunchArgument("color_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("debug_image_topic", default_value="/yolo_cup_uprighting/debug_image"),
        DeclareLaunchArgument(
            "preview_only",
            default_value="true",
            description="When true, only YOLO/camera preview runs; MoveIt, gripper, and HOME motion are skipped.",
        ),
        DeclareLaunchArgument(
            "show_window",
            default_value="false",
            description="When true, show an OpenCV window for keyboard control. Headless debug images are always published.",
        ),
        DeclareLaunchArgument(
            "use_mock_vision",
            default_value="false",
            description="Use the internal mock gripper/vision path for bench debugging.",
        ),
        DeclareLaunchArgument("max_velocity_scale", default_value="0.1"),
        DeclareLaunchArgument("max_acceleration_scale", default_value="0.1"),
        SetEnvironmentVariable("YOLO_MODEL_PATH", model_path),
        SetEnvironmentVariable("YOLO_TOPIC_COLOR", color_topic),
        SetEnvironmentVariable("YOLO_TOPIC_DEPTH", depth_topic),
        SetEnvironmentVariable("YOLO_TOPIC_CAM_INFO", camera_info_topic),
        SetEnvironmentVariable("YOLO_TOPIC_DEBUG_IMAGE", debug_image_topic),
        SetEnvironmentVariable("YOLO_PREVIEW_ONLY", preview_only),
        SetEnvironmentVariable("YOLO_SHOW_WINDOW", show_window),
        SetEnvironmentVariable("YOLO_USE_MOCK_VISION", use_mock_vision),
        SetEnvironmentVariable("YOLO_MAX_VEL_SCALE", max_velocity_scale),
        SetEnvironmentVariable("YOLO_MAX_ACC_SCALE", max_acceleration_scale),
        OpaqueFunction(function=_launch_setup),
    ])
