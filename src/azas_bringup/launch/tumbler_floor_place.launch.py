from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    selected_dispenser_id = LaunchConfiguration("selected_dispenser_id")
    enable_hardware = LaunchConfiguration("enable_hardware")
    hardware_confirm = LaunchConfiguration("hardware_confirm")
    allow_service_control_without_moveit = LaunchConfiguration(
        "allow_service_control_without_moveit"
    )
    service_prefix = LaunchConfiguration("service_prefix")
    execution_stage = LaunchConfiguration("execution_stage")
    use_tumbler_pose_topic = LaunchConfiguration("use_tumbler_pose_topic")
    tumbler_pose_topic = LaunchConfiguration("tumbler_pose_topic")
    tumbler_pose_wait_timeout = LaunchConfiguration("tumbler_pose_wait_timeout")
    tumbler_pose_max_age_sec = LaunchConfiguration("tumbler_pose_max_age_sec")
    allow_demo_tumbler_position_fallback = LaunchConfiguration(
        "allow_demo_tumbler_position_fallback"
    )
    tumbler_position_x = LaunchConfiguration("tumbler_position_x")
    tumbler_position_y = LaunchConfiguration("tumbler_position_y")
    tumbler_position_z = LaunchConfiguration("tumbler_position_z")
    tumbler_bottom_diameter = LaunchConfiguration("tumbler_bottom_diameter")
    tumbler_top_diameter = LaunchConfiguration("tumbler_top_diameter")
    grasp_height = LaunchConfiguration("grasp_height")
    side_grasp_approach_offset = LaunchConfiguration("side_grasp_approach_offset")
    side_grasp_candidate_count = LaunchConfiguration("side_grasp_candidate_count")
    side_grasp_preferred_axes = LaunchConfiguration("side_grasp_preferred_axes")
    use_detected_grasp_yaw = LaunchConfiguration("use_detected_grasp_yaw")
    lift_height = LaunchConfiguration("lift_height")
    delivery_mode = LaunchConfiguration("delivery_mode")
    place_approach_height = LaunchConfiguration("place_approach_height")
    place_mouth_under_outlet = LaunchConfiguration("place_mouth_under_outlet")
    outlet_mouth_clearance = LaunchConfiguration("outlet_mouth_clearance")
    gripper_open_service = LaunchConfiguration("gripper_open_service")
    gripper_close_service = LaunchConfiguration("gripper_close_service")
    gripper_set_service = LaunchConfiguration("gripper_set_service")
    gripper_preopen_clearance = LaunchConfiguration("gripper_preopen_clearance")
    gripper_grasp_compression = LaunchConfiguration("gripper_grasp_compression")
    gripper_grasp_force_n = LaunchConfiguration("gripper_grasp_force_n")
    gripper_preopen_force_n = LaunchConfiguration("gripper_preopen_force_n")
    gripper_max_width_m = LaunchConfiguration("gripper_max_width_m")
    gripper_min_width_m = LaunchConfiguration("gripper_min_width_m")

    params = {
        "auto_start": True,
        "enable_hardware": ParameterValue(enable_hardware, value_type=bool),
        "hardware_confirm": hardware_confirm,
        "allow_service_control_without_moveit": ParameterValue(
            allow_service_control_without_moveit, value_type=bool
        ),
        # If dsr_bringup2 is launched with name:=dsr01, set this to "dsr01".
        "service_prefix": service_prefix,
        "execution_stage": execution_stage,
        "frame_id": "base_link",
        "use_tumbler_pose_topic": ParameterValue(use_tumbler_pose_topic, value_type=bool),
        "tumbler_pose_topic": tumbler_pose_topic,
        "tumbler_pose_wait_timeout": ParameterValue(tumbler_pose_wait_timeout, value_type=float),
        "tumbler_pose_max_age_sec": ParameterValue(tumbler_pose_max_age_sec, value_type=float),
        "allow_demo_tumbler_position_fallback": ParameterValue(
            allow_demo_tumbler_position_fallback,
            value_type=bool,
        ),
        "selected_dispenser_id": ParameterValue(selected_dispenser_id, value_type=int),
        "dispenser_count": 4,
        "tumbler_position": [0.32, -0.22, 0.05],
        "tumbler_position_x": ParameterValue(tumbler_position_x, value_type=float),
        "tumbler_position_y": ParameterValue(tumbler_position_y, value_type=float),
        "tumbler_position_z": ParameterValue(tumbler_position_z, value_type=float),
        "tumbler_height": 0.17,
        "tumbler_radius": 0.0375,
        "tumbler_bottom_diameter": ParameterValue(tumbler_bottom_diameter, value_type=float),
        "tumbler_top_diameter": ParameterValue(tumbler_top_diameter, value_type=float),
        "grasp_height": ParameterValue(grasp_height, value_type=float),
        "side_grasp_approach_offset": ParameterValue(side_grasp_approach_offset, value_type=float),
        "side_grasp_candidate_count": ParameterValue(side_grasp_candidate_count, value_type=int),
        "side_grasp_preferred_axes": side_grasp_preferred_axes,
        "use_detected_grasp_yaw": ParameterValue(use_detected_grasp_yaw, value_type=bool),
        "lift_height": ParameterValue(lift_height, value_type=float),
        "delivery_mode": delivery_mode,
        "place_approach_height": ParameterValue(place_approach_height, value_type=float),
        "placement_floor_z": 0.0,
        "place_mouth_under_outlet": ParameterValue(place_mouth_under_outlet, value_type=bool),
        "outlet_mouth_clearance": ParameterValue(outlet_mouth_clearance, value_type=float),
        "clearance": 0.05,
        "dispenser_bottle_positions": [
            0.55,
            0.18,
            0.1375,
            0.55,
            0.08,
            0.1375,
            0.55,
            -0.02,
            0.1375,
            0.55,
            -0.12,
            0.1375,
        ],
        "dispenser_outlet_positions": [
            0.43,
            0.18,
            0.392,
            0.43,
            0.08,
            0.392,
            0.43,
            -0.02,
            0.392,
            0.43,
            -0.12,
            0.392,
        ],
        "home_joints_deg": [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
        "move_home_first": False,
        "return_home": False,
        "rx": 180.0,
        "ry": 0.0,
        "rz": 180.0,
        "joint_velocity": 20.0,
        "joint_acceleration": 20.0,
        "line_velocity": 30.0,
        "line_acceleration": 50.0,
        "workspace_x_min": 0.0,
        "workspace_x_max": 0.80,
        "workspace_y_min": -0.35,
        "workspace_y_max": 0.35,
        "workspace_z_min": 0.0,
        "workspace_z_max": 0.80,
        # Optional std_srvs/Trigger services. Leave empty until RG2 wrapper is confirmed.
        "gripper_open_service": gripper_open_service,
        "gripper_close_service": gripper_close_service,
        "gripper_set_service": gripper_set_service,
        "gripper_preopen_clearance": ParameterValue(gripper_preopen_clearance, value_type=float),
        "gripper_grasp_compression": ParameterValue(gripper_grasp_compression, value_type=float),
        "gripper_grasp_force_n": ParameterValue(gripper_grasp_force_n, value_type=float),
        "gripper_preopen_force_n": ParameterValue(gripper_preopen_force_n, value_type=float),
        "gripper_max_width_m": ParameterValue(gripper_max_width_m, value_type=float),
        "gripper_min_width_m": ParameterValue(gripper_min_width_m, value_type=float),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("selected_dispenser_id", default_value="1"),
            DeclareLaunchArgument("enable_hardware", default_value="false"),
            DeclareLaunchArgument("hardware_confirm", default_value=""),
            DeclareLaunchArgument(
                "allow_service_control_without_moveit",
                default_value="false",
            ),
            DeclareLaunchArgument("service_prefix", default_value=""),
            DeclareLaunchArgument("execution_stage", default_value="full"),
            DeclareLaunchArgument("use_tumbler_pose_topic", default_value="true"),
            DeclareLaunchArgument(
                "tumbler_pose_topic",
                default_value="/jarvis/tumbler_dispenser/tumbler_pose",
            ),
            DeclareLaunchArgument("tumbler_pose_wait_timeout", default_value="3.0"),
            DeclareLaunchArgument("tumbler_pose_max_age_sec", default_value="1.0"),
            DeclareLaunchArgument(
                "allow_demo_tumbler_position_fallback",
                default_value="true",
            ),
            DeclareLaunchArgument("tumbler_position_x", default_value="0.32"),
            DeclareLaunchArgument("tumbler_position_y", default_value="-0.22"),
            DeclareLaunchArgument("tumbler_position_z", default_value="0.05"),
            DeclareLaunchArgument("tumbler_bottom_diameter", default_value="0.065"),
            DeclareLaunchArgument("tumbler_top_diameter", default_value="0.075"),
            DeclareLaunchArgument("grasp_height", default_value="0.085"),
            DeclareLaunchArgument("side_grasp_approach_offset", default_value="0.10"),
            DeclareLaunchArgument("side_grasp_candidate_count", default_value="16"),
            DeclareLaunchArgument("side_grasp_preferred_axes", default_value=""),
            DeclareLaunchArgument("use_detected_grasp_yaw", default_value="true"),
            DeclareLaunchArgument("lift_height", default_value="0.04"),
            DeclareLaunchArgument("delivery_mode", default_value="floor_place"),
            DeclareLaunchArgument("place_approach_height", default_value="0.06"),
            DeclareLaunchArgument("place_mouth_under_outlet", default_value="false"),
            DeclareLaunchArgument("outlet_mouth_clearance", default_value="0.0"),
            DeclareLaunchArgument("gripper_open_service", default_value=""),
            DeclareLaunchArgument("gripper_close_service", default_value=""),
            DeclareLaunchArgument("gripper_set_service", default_value="/jarvis/rg2/set_width"),
            DeclareLaunchArgument("gripper_preopen_clearance", default_value="0.025"),
            DeclareLaunchArgument("gripper_grasp_compression", default_value="0.006"),
            DeclareLaunchArgument("gripper_grasp_force_n", default_value="12.0"),
            DeclareLaunchArgument("gripper_preopen_force_n", default_value="8.0"),
            DeclareLaunchArgument("gripper_max_width_m", default_value="0.110"),
            DeclareLaunchArgument("gripper_min_width_m", default_value="0.0"),
            Node(
                package="azas_motion",
                executable="tumbler_floor_place_node",
                name="tumbler_floor_place_node",
                output="screen",
                parameters=[params],
            )
        ]
    )
