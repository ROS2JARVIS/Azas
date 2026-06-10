from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("azas_cup_uprighting"), "config", "moveit_py.yaml"]
    )

    nodes = []
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

        if _as_bool(LaunchConfiguration("workspace_collision_scene_enabled").perform(context)):
            nodes.append(
                Node(
                    package="azas_motion",
                    executable="workspace_collision_scene_node",
                    name="workspace_collision_scene_node",
                    output="screen",
                    parameters=[
                        {
                            "safety_config_path": ParameterValue(
                                LaunchConfiguration("safety_config_path"),
                                value_type=str,
                            ),
                            "publish_period_sec": ParameterValue(
                                LaunchConfiguration(
                                    "workspace_collision_publish_period_sec"
                                ),
                                value_type=float,
                            ),
                            "table_collision_enabled": ParameterValue(
                                LaunchConfiguration("table_collision_enabled"),
                                value_type=bool,
                            ),
                            "table_surface_z": ParameterValue(
                                LaunchConfiguration("table_surface_z"),
                                value_type=float,
                            ),
                            "table_thickness": ParameterValue(
                                LaunchConfiguration("table_thickness"),
                                value_type=float,
                            ),
                            "table_size_x": ParameterValue(
                                LaunchConfiguration("table_size_x"),
                                value_type=float,
                            ),
                            "table_size_y": ParameterValue(
                                LaunchConfiguration("table_size_y"),
                                value_type=float,
                            ),
                            "table_center_x": ParameterValue(
                                LaunchConfiguration("table_center_x"),
                                value_type=float,
                            ),
                            "table_center_y": ParameterValue(
                                LaunchConfiguration("table_center_y"),
                                value_type=float,
                            ),
                            "table_collision_expand_to_workspace_walls": ParameterValue(
                                LaunchConfiguration(
                                    "table_collision_expand_to_workspace_walls"
                                ),
                                value_type=bool,
                            ),
                            "workspace_boundary_collision_enabled": ParameterValue(
                                LaunchConfiguration(
                                    "workspace_boundary_collision_enabled"
                                ),
                                value_type=bool,
                            ),
                            "workspace_boundary_collision_prefix": ParameterValue(
                                LaunchConfiguration(
                                    "workspace_boundary_collision_prefix"
                                ),
                                value_type=str,
                            ),
                            "workspace_boundary_wall_thickness": ParameterValue(
                                LaunchConfiguration(
                                    "workspace_boundary_wall_thickness"
                                ),
                                value_type=float,
                            ),
                            "workspace_boundary_wall_clearance": ParameterValue(
                                LaunchConfiguration(
                                    "workspace_boundary_wall_clearance"
                                ),
                                value_type=float,
                            ),
                        }
                    ],
                )
            )

        if _as_bool(LaunchConfiguration("dispenser_collision_enabled").perform(context)):
            nodes.append(
                Node(
                    package="azas_motion",
                    executable="measured_dispenser_collision_scene_node",
                    name="measured_dispenser_collision_scene_node",
                    output="screen",
                    parameters=[
                        {
                            "config_path": ParameterValue(
                                LaunchConfiguration(
                                    "dispenser_collision_config_path"
                                ),
                                value_type=str,
                            ),
                            "publish_period_sec": ParameterValue(
                                LaunchConfiguration(
                                    "dispenser_collision_publish_period_sec"
                                ),
                                value_type=float,
                            ),
                            "publish_collision_objects": ParameterValue(
                                LaunchConfiguration(
                                    "dispenser_collision_publish_objects"
                                ),
                                value_type=bool,
                            ),
                            "publish_markers": ParameterValue(
                                LaunchConfiguration(
                                    "dispenser_collision_publish_markers"
                                ),
                                value_type=bool,
                            ),
                        }
                    ],
                )
            )

    yolo_node = Node(
        package="azas_cup_uprighting",
        executable="yolo_cup_uprighting",
        name="yolo_cup_uprighting_py",
        output="screen",
        parameters=parameters,
    )
    nodes.append(yolo_node)
    nodes.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=yolo_node,
                on_exit=[Shutdown(reason="yolo_cup_uprighting exited")],
            )
        )
    )
    return nodes


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
        DeclareLaunchArgument(
            "workspace_collision_scene_enabled",
            default_value="true",
            description="Start the shared workspace collision scene node for table and boundary walls.",
        ),
        DeclareLaunchArgument(
            "workspace_collision_publish_period_sec",
            default_value="2.0",
            description="Republish period for shared workspace collision objects.",
        ),
        DeclareLaunchArgument(
            "table_collision_enabled",
            default_value="true",
            description="Publish a base_link table collision box so MoveIt avoids table collisions.",
        ),
        DeclareLaunchArgument("table_surface_z", default_value="0.0"),
        DeclareLaunchArgument("table_thickness", default_value="0.04"),
        DeclareLaunchArgument("table_size_x", default_value="1.20"),
        DeclareLaunchArgument("table_size_y", default_value="1.00"),
        DeclareLaunchArgument("table_center_x", default_value="0.45"),
        DeclareLaunchArgument("table_center_y", default_value="0.0"),
        DeclareLaunchArgument(
            "table_collision_expand_to_workspace_walls",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "safety_config_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("azas_bringup"), "config", "safety.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "workspace_boundary_collision_enabled",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "workspace_boundary_collision_prefix",
            default_value="side_grip_workspace",
        ),
        DeclareLaunchArgument(
            "workspace_boundary_wall_thickness",
            default_value="0.04",
        ),
        DeclareLaunchArgument(
            "workspace_boundary_wall_clearance",
            default_value="0.02",
        ),
        DeclareLaunchArgument(
            "dispenser_collision_enabled",
            default_value="true",
            description="Publish measured dispenser collision boxes to MoveIt's PlanningScene.",
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
            "dispenser_collision_publish_period_sec",
            default_value="1.0",
        ),
        DeclareLaunchArgument(
            "dispenser_collision_publish_objects",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "dispenser_collision_publish_markers",
            default_value="true",
        ),
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
