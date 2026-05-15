from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="m0609",
            package_name="dsr_moveit_config_m0609",
        )
        .robot_description(file_path="config/m0609.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf")
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_scene_monitor()
        .sensors_3d()
        .to_moveit_configs()
    )

    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("dsr_practice"), "config", "moveit_py.yaml"]
    )

    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/home/ssu/ros2_ws/yolo_runs/cup_yolov8n_ft1/weights/best.pt",
        description="Path to trained cup YOLO weights.",
    )
    conf_arg = DeclareLaunchArgument("conf", default_value="0.35")
    imgsz_arg = DeclareLaunchArgument("imgsz", default_value="640")
    device_arg = DeclareLaunchArgument("device", default_value="cpu")
    target_class_arg = DeclareLaunchArgument("target_class", default_value="cup")
    auto_pick_interval_arg = DeclareLaunchArgument(
        "auto_pick_interval", default_value="3.0"
    )
    pick_depth_ratio_arg = DeclareLaunchArgument(
        "pick_depth_ratio", default_value="0.55"
    )
    depth_patch_radius_arg = DeclareLaunchArgument(
        "depth_patch_radius", default_value="7"
    )
    min_depth_valid_ratio_arg = DeclareLaunchArgument(
        "min_depth_valid_ratio", default_value="0.03"
    )
    min_depth_m_arg = DeclareLaunchArgument("min_depth_m", default_value="0.15")
    max_depth_m_arg = DeclareLaunchArgument("max_depth_m", default_value="1.20")
    redetect_on_approach_arg = DeclareLaunchArgument(
        "redetect_on_approach", default_value="true"
    )
    redetect_settle_sec_arg = DeclareLaunchArgument(
        "redetect_settle_sec", default_value="0.5"
    )
    grasp_mode_arg = DeclareLaunchArgument("grasp_mode", default_value="side")
    side_grasp_axis_arg = DeclareLaunchArgument(
        "side_grasp_axis", default_value="y_axis"
    )
    side_grasp_direction_arg = DeclareLaunchArgument(
        "side_grasp_direction", default_value="-1.0"
    )
    side_approach_offset_arg = DeclareLaunchArgument(
        "side_approach_offset", default_value="0.12"
    )
    side_staging_offset_arg = DeclareLaunchArgument(
        "side_staging_offset",
        default_value="0.24",
        description="Far outside offset where the wrist first turns horizontal.",
    )
    side_grasp_offset_arg = DeclareLaunchArgument(
        "side_grasp_offset", default_value="0.035"
    )
    side_grasp_z_offset_arg = DeclareLaunchArgument(
        "side_grasp_z_offset",
        default_value="0.05",
        description="Side grasp height offset from detected base point.",
    )
    side_orientation_mode_arg = DeclareLaunchArgument(
        "side_orientation_mode",
        default_value="approach",
        description="Side grasp orientation: approach, euler, or home.",
    )
    side_tool_roll_deg_arg = DeclareLaunchArgument(
        "side_tool_roll_deg",
        default_value="0.0",
        description="Twist around the horizontal approach direction for RG2 finger alignment.",
    )
    side_roll_deg_arg = DeclareLaunchArgument(
        "side_roll_deg",
        default_value="0.0",
        description="Manual side grasp roll, used when side_orientation_mode:=euler.",
    )
    side_pitch_deg_arg = DeclareLaunchArgument(
        "side_pitch_deg",
        default_value="90.0",
        description="Manual side grasp pitch, used when side_orientation_mode:=euler.",
    )
    side_yaw_deg_arg = DeclareLaunchArgument(
        "side_yaw_deg",
        default_value="0.0",
        description="Manual side grasp yaw, used when side_orientation_mode:=euler.",
    )
    verify_motion_arg = DeclareLaunchArgument("verify_motion", default_value="true")
    motion_verify_tolerance_arg = DeclareLaunchArgument(
        "motion_verify_tolerance", default_value="0.01"
    )
    move_to_camera_home_arg = DeclareLaunchArgument(
        "move_to_camera_home", default_value="true"
    )
    camera_home_x_arg = DeclareLaunchArgument("camera_home_x", default_value="0.45")
    camera_home_y_arg = DeclareLaunchArgument("camera_home_y", default_value="0.0")
    camera_home_z_arg = DeclareLaunchArgument("camera_home_z", default_value="0.62")
    min_motion_z_arg = DeclareLaunchArgument(
        "min_motion_z",
        default_value="0.12",
        description="Minimum allowed commanded Z in base frame.",
    )
    return_home_after_task_arg = DeclareLaunchArgument(
        "return_home_after_task", default_value="true"
    )
    place_x_arg = DeclareLaunchArgument("place_x", default_value="0.45")
    place_y_arg = DeclareLaunchArgument("place_y", default_value="0.0")
    place_z_arg = DeclareLaunchArgument("place_z", default_value="0.30")
    auto_pick_arg = DeclareLaunchArgument("auto_pick", default_value="false")

    return LaunchDescription(
        [
            model_path_arg,
            conf_arg,
            imgsz_arg,
            device_arg,
            target_class_arg,
            auto_pick_interval_arg,
            pick_depth_ratio_arg,
            depth_patch_radius_arg,
            min_depth_valid_ratio_arg,
            min_depth_m_arg,
            max_depth_m_arg,
            redetect_on_approach_arg,
            redetect_settle_sec_arg,
            grasp_mode_arg,
            side_grasp_axis_arg,
            side_grasp_direction_arg,
            side_approach_offset_arg,
            side_staging_offset_arg,
            side_grasp_offset_arg,
            side_grasp_z_offset_arg,
            side_orientation_mode_arg,
            side_tool_roll_deg_arg,
            side_roll_deg_arg,
            side_pitch_deg_arg,
            side_yaw_deg_arg,
            verify_motion_arg,
            motion_verify_tolerance_arg,
            move_to_camera_home_arg,
            camera_home_x_arg,
            camera_home_y_arg,
            camera_home_z_arg,
            min_motion_z_arg,
            return_home_after_task_arg,
            place_x_arg,
            place_y_arg,
            place_z_arg,
            auto_pick_arg,
            Node(
                package="dsr_practice",
                executable="joint_state_relay",
                name="joint_state_relay",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/dsr01/joint_states",
                        "output_topic": "/joint_states",
                    }
                ],
            ),
            Node(
                package="dsr_practice",
                executable="yolo_cup_pick_node",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                    {
                        "model_path": ParameterValue(
                            LaunchConfiguration("model_path"),
                            value_type=str,
                        ),
                        "conf": LaunchConfiguration("conf"),
                        "imgsz": LaunchConfiguration("imgsz"),
                        "device": ParameterValue(
                            LaunchConfiguration("device"),
                            value_type=str,
                        ),
                        "target_class": ParameterValue(
                            LaunchConfiguration("target_class"),
                            value_type=str,
                        ),
                        "auto_pick_interval": LaunchConfiguration(
                            "auto_pick_interval"
                        ),
                        "pick_depth_ratio": LaunchConfiguration("pick_depth_ratio"),
                        "depth_patch_radius": LaunchConfiguration(
                            "depth_patch_radius"
                        ),
                        "min_depth_valid_ratio": LaunchConfiguration(
                            "min_depth_valid_ratio"
                        ),
                        "min_depth_m": LaunchConfiguration("min_depth_m"),
                        "max_depth_m": LaunchConfiguration("max_depth_m"),
                        "redetect_on_approach": LaunchConfiguration(
                            "redetect_on_approach"
                        ),
                        "redetect_settle_sec": LaunchConfiguration(
                            "redetect_settle_sec"
                        ),
                        "grasp_mode": ParameterValue(
                            LaunchConfiguration("grasp_mode"),
                            value_type=str,
                        ),
                        "side_grasp_axis": ParameterValue(
                            LaunchConfiguration("side_grasp_axis"),
                            value_type=str,
                        ),
                        "side_grasp_direction": LaunchConfiguration(
                            "side_grasp_direction"
                        ),
                        "side_approach_offset": LaunchConfiguration(
                            "side_approach_offset"
                        ),
                        "side_staging_offset": LaunchConfiguration(
                            "side_staging_offset"
                        ),
                        "side_grasp_offset": LaunchConfiguration("side_grasp_offset"),
                        "side_grasp_z_offset": LaunchConfiguration(
                            "side_grasp_z_offset"
                        ),
                        "side_orientation_mode": ParameterValue(
                            LaunchConfiguration("side_orientation_mode"),
                            value_type=str,
                        ),
                        "side_tool_roll_deg": LaunchConfiguration(
                            "side_tool_roll_deg"
                        ),
                        "side_roll_deg": LaunchConfiguration("side_roll_deg"),
                        "side_pitch_deg": LaunchConfiguration("side_pitch_deg"),
                        "side_yaw_deg": LaunchConfiguration("side_yaw_deg"),
                        "verify_motion": LaunchConfiguration("verify_motion"),
                        "motion_verify_tolerance": LaunchConfiguration(
                            "motion_verify_tolerance"
                        ),
                        "move_to_camera_home": LaunchConfiguration(
                            "move_to_camera_home"
                        ),
                        "camera_home_x": LaunchConfiguration("camera_home_x"),
                        "camera_home_y": LaunchConfiguration("camera_home_y"),
                        "camera_home_z": LaunchConfiguration("camera_home_z"),
                        "min_motion_z": LaunchConfiguration("min_motion_z"),
                        "return_home_after_task": LaunchConfiguration(
                            "return_home_after_task"
                        ),
                        "place_x": LaunchConfiguration("place_x"),
                        "place_y": LaunchConfiguration("place_y"),
                        "place_z": LaunchConfiguration("place_z"),
                        "auto_pick": LaunchConfiguration("auto_pick"),
                    },
                ],
            ),
        ]
    )
