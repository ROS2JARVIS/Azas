from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_hardware = LaunchConfiguration("enable_hardware")
    hardware_confirm = LaunchConfiguration("hardware_confirm")
    allow_service_control_without_moveit = LaunchConfiguration(
        "allow_service_control_without_moveit"
    )
    service_prefix = LaunchConfiguration("service_prefix")
    execution_stage = LaunchConfiguration("execution_stage")
    shake_control_mode = LaunchConfiguration("shake_control_mode")
    shake_center_x = LaunchConfiguration("shake_center_x")
    shake_center_y = LaunchConfiguration("shake_center_y")
    shake_center_z = LaunchConfiguration("shake_center_z")
    shake_approach_height = LaunchConfiguration("shake_approach_height")
    shake_amplitude_x = LaunchConfiguration("shake_amplitude_x")
    shake_amplitude_y = LaunchConfiguration("shake_amplitude_y")
    shake_amplitude_z = LaunchConfiguration("shake_amplitude_z")
    shake_cycles = LaunchConfiguration("shake_cycles")
    shake_twist_rx_deg = LaunchConfiguration("shake_twist_rx_deg")
    shake_twist_ry_deg = LaunchConfiguration("shake_twist_ry_deg")
    shake_twist_rz_deg = LaunchConfiguration("shake_twist_rz_deg")
    line_time = LaunchConfiguration("line_time")
    approach_line_time = LaunchConfiguration("approach_line_time")
    shake_line_time = LaunchConfiguration("shake_line_time")
    approach_line_velocity = LaunchConfiguration("approach_line_velocity")
    approach_line_acceleration = LaunchConfiguration("approach_line_acceleration")
    shake_line_velocity = LaunchConfiguration("shake_line_velocity")
    shake_line_acceleration = LaunchConfiguration("shake_line_acceleration")
    service_wait_timeout_sec = LaunchConfiguration("service_wait_timeout_sec")
    motion_response_timeout_sec = LaunchConfiguration("motion_response_timeout_sec")
    precheck_ikin_joint5 = LaunchConfiguration("precheck_ikin_joint5")
    enforce_wrist_joint_limits = LaunchConfiguration("enforce_wrist_joint_limits")
    ikin_sol_space = LaunchConfiguration("ikin_sol_space")
    joint5_min_deg = LaunchConfiguration("joint5_min_deg")
    joint5_max_deg = LaunchConfiguration("joint5_max_deg")
    wrist_min_deg = LaunchConfiguration("wrist_min_deg")
    wrist_max_deg = LaunchConfiguration("wrist_max_deg")
    joint_shake_base_j1_deg = LaunchConfiguration("joint_shake_base_j1_deg")
    joint_shake_base_j2_deg = LaunchConfiguration("joint_shake_base_j2_deg")
    joint_shake_base_j3_deg = LaunchConfiguration("joint_shake_base_j3_deg")
    joint_shake_base_j4_deg = LaunchConfiguration("joint_shake_base_j4_deg")
    joint_shake_base_j5_deg = LaunchConfiguration("joint_shake_base_j5_deg")
    joint_shake_base_j6_deg = LaunchConfiguration("joint_shake_base_j6_deg")
    joint_shake_j3_amplitude_deg = LaunchConfiguration("joint_shake_j3_amplitude_deg")
    joint_shake_j4_amplitude_deg = LaunchConfiguration("joint_shake_j4_amplitude_deg")
    joint_shake_j5_amplitude_deg = LaunchConfiguration("joint_shake_j5_amplitude_deg")
    joint_shake_j6_amplitude_deg = LaunchConfiguration("joint_shake_j6_amplitude_deg")
    joint_shake_j1_min_deg = LaunchConfiguration("joint_shake_j1_min_deg")
    joint_shake_j1_max_deg = LaunchConfiguration("joint_shake_j1_max_deg")
    joint_shake_j2_min_deg = LaunchConfiguration("joint_shake_j2_min_deg")
    joint_shake_j2_max_deg = LaunchConfiguration("joint_shake_j2_max_deg")
    joint_shake_j3_min_deg = LaunchConfiguration("joint_shake_j3_min_deg")
    joint_shake_j3_max_deg = LaunchConfiguration("joint_shake_j3_max_deg")
    joint_shake_max_single_delta_deg = LaunchConfiguration("joint_shake_max_single_delta_deg")
    approach_joint_velocity = LaunchConfiguration("approach_joint_velocity")
    approach_joint_acceleration = LaunchConfiguration("approach_joint_acceleration")
    approach_joint_time = LaunchConfiguration("approach_joint_time")
    shake_joint_velocity = LaunchConfiguration("shake_joint_velocity")
    shake_joint_acceleration = LaunchConfiguration("shake_joint_acceleration")
    shake_joint_time = LaunchConfiguration("shake_joint_time")
    verify_joint_targets = LaunchConfiguration("verify_joint_targets")
    joint_target_tolerance_deg = LaunchConfiguration("joint_target_tolerance_deg")
    joint_target_wait_extra_sec = LaunchConfiguration("joint_target_wait_extra_sec")
    joint_target_poll_sec = LaunchConfiguration("joint_target_poll_sec")
    require_state_validity_for_joint_shake = LaunchConfiguration(
        "require_state_validity_for_joint_shake"
    )
    state_validity_service = LaunchConfiguration("state_validity_service")
    planning_group = LaunchConfiguration("planning_group")
    min_shake_z = LaunchConfiguration("min_shake_z")
    dispenser_keepout_radius = LaunchConfiguration("dispenser_keepout_radius")
    rx = LaunchConfiguration("rx")
    ry = LaunchConfiguration("ry")
    rz = LaunchConfiguration("rz")
    use_visualizer = LaunchConfiguration("use_visualizer")

    params = {
        "auto_start": True,
        "enable_hardware": ParameterValue(enable_hardware, value_type=bool),
        "hardware_confirm": hardware_confirm,
        "allow_service_control_without_moveit": ParameterValue(
            allow_service_control_without_moveit, value_type=bool
        ),
        "service_prefix": service_prefix,
        "execution_stage": execution_stage,
        "shake_control_mode": shake_control_mode,
        "frame_id": "base_link",
        "dispenser_count": 4,
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
        "shake_center_x": ParameterValue(shake_center_x, value_type=float),
        "shake_center_y": ParameterValue(shake_center_y, value_type=float),
        "shake_center_z": ParameterValue(shake_center_z, value_type=float),
        "shake_approach_height": ParameterValue(shake_approach_height, value_type=float),
        "shake_amplitude_x": ParameterValue(shake_amplitude_x, value_type=float),
        "shake_amplitude_y": ParameterValue(shake_amplitude_y, value_type=float),
        "shake_amplitude_z": ParameterValue(shake_amplitude_z, value_type=float),
        "shake_cycles": ParameterValue(shake_cycles, value_type=int),
        "shake_twist_rx_deg": ParameterValue(shake_twist_rx_deg, value_type=float),
        "shake_twist_ry_deg": ParameterValue(shake_twist_ry_deg, value_type=float),
        "shake_twist_rz_deg": ParameterValue(shake_twist_rz_deg, value_type=float),
        "shake_hold_seconds": 0.0,
        "workspace_min_x": 0.0,
        "workspace_max_x": 0.80,
        "workspace_min_y": -0.35,
        "workspace_max_y": 0.35,
        "workspace_min_z": 0.0,
        "workspace_max_z": 0.80,
        "min_shake_z": ParameterValue(min_shake_z, value_type=float),
        "dispenser_keepout_radius": ParameterValue(
            dispenser_keepout_radius,
            value_type=float,
        ),
        "rx": ParameterValue(rx, value_type=float),
        "ry": ParameterValue(ry, value_type=float),
        "rz": ParameterValue(rz, value_type=float),
        "line_velocity": 45.0,
        "line_acceleration": 80.0,
        "line_time": ParameterValue(line_time, value_type=float),
        "approach_line_velocity": ParameterValue(approach_line_velocity, value_type=float),
        "approach_line_acceleration": ParameterValue(approach_line_acceleration, value_type=float),
        "approach_line_time": ParameterValue(approach_line_time, value_type=float),
        "shake_line_velocity": ParameterValue(shake_line_velocity, value_type=float),
        "shake_line_acceleration": ParameterValue(shake_line_acceleration, value_type=float),
        "shake_line_time": ParameterValue(shake_line_time, value_type=float),
        "service_wait_timeout_sec": ParameterValue(service_wait_timeout_sec, value_type=float),
        "motion_response_timeout_sec": ParameterValue(
            motion_response_timeout_sec,
            value_type=float,
        ),
        "precheck_ikin_joint5": ParameterValue(precheck_ikin_joint5, value_type=bool),
        "enforce_wrist_joint_limits": ParameterValue(enforce_wrist_joint_limits, value_type=bool),
        "ikin_sol_space": ParameterValue(ikin_sol_space, value_type=int),
        "joint5_min_deg": ParameterValue(joint5_min_deg, value_type=float),
        "joint5_max_deg": ParameterValue(joint5_max_deg, value_type=float),
        "wrist_min_deg": ParameterValue(wrist_min_deg, value_type=float),
        "wrist_max_deg": ParameterValue(wrist_max_deg, value_type=float),
        "joint_shake_base_j1_deg": ParameterValue(joint_shake_base_j1_deg, value_type=float),
        "joint_shake_base_j2_deg": ParameterValue(joint_shake_base_j2_deg, value_type=float),
        "joint_shake_base_j3_deg": ParameterValue(joint_shake_base_j3_deg, value_type=float),
        "joint_shake_base_j4_deg": ParameterValue(joint_shake_base_j4_deg, value_type=float),
        "joint_shake_base_j5_deg": ParameterValue(joint_shake_base_j5_deg, value_type=float),
        "joint_shake_base_j6_deg": ParameterValue(joint_shake_base_j6_deg, value_type=float),
        "joint_shake_j3_amplitude_deg": ParameterValue(
            joint_shake_j3_amplitude_deg, value_type=float
        ),
        "joint_shake_j4_amplitude_deg": ParameterValue(
            joint_shake_j4_amplitude_deg, value_type=float
        ),
        "joint_shake_j5_amplitude_deg": ParameterValue(
            joint_shake_j5_amplitude_deg, value_type=float
        ),
        "joint_shake_j6_amplitude_deg": ParameterValue(
            joint_shake_j6_amplitude_deg, value_type=float
        ),
        "joint_shake_j1_min_deg": ParameterValue(joint_shake_j1_min_deg, value_type=float),
        "joint_shake_j1_max_deg": ParameterValue(joint_shake_j1_max_deg, value_type=float),
        "joint_shake_j2_min_deg": ParameterValue(joint_shake_j2_min_deg, value_type=float),
        "joint_shake_j2_max_deg": ParameterValue(joint_shake_j2_max_deg, value_type=float),
        "joint_shake_j3_min_deg": ParameterValue(joint_shake_j3_min_deg, value_type=float),
        "joint_shake_j3_max_deg": ParameterValue(joint_shake_j3_max_deg, value_type=float),
        "joint_shake_max_single_delta_deg": ParameterValue(
            joint_shake_max_single_delta_deg, value_type=float
        ),
        "approach_joint_velocity": ParameterValue(approach_joint_velocity, value_type=float),
        "approach_joint_acceleration": ParameterValue(
            approach_joint_acceleration, value_type=float
        ),
        "approach_joint_time": ParameterValue(approach_joint_time, value_type=float),
        "shake_joint_velocity": ParameterValue(shake_joint_velocity, value_type=float),
        "shake_joint_acceleration": ParameterValue(shake_joint_acceleration, value_type=float),
        "shake_joint_time": ParameterValue(shake_joint_time, value_type=float),
        "verify_joint_targets": ParameterValue(verify_joint_targets, value_type=bool),
        "joint_target_tolerance_deg": ParameterValue(
            joint_target_tolerance_deg,
            value_type=float,
        ),
        "joint_target_wait_extra_sec": ParameterValue(
            joint_target_wait_extra_sec,
            value_type=float,
        ),
        "joint_target_poll_sec": ParameterValue(joint_target_poll_sec, value_type=float),
        "require_state_validity_for_joint_shake": ParameterValue(
            require_state_validity_for_joint_shake,
            value_type=bool,
        ),
        "state_validity_service": state_validity_service,
        "planning_group": planning_group,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_hardware", default_value="false"),
            DeclareLaunchArgument("hardware_confirm", default_value=""),
            DeclareLaunchArgument(
                "allow_service_control_without_moveit",
                default_value="false",
            ),
            DeclareLaunchArgument("service_prefix", default_value=""),
            DeclareLaunchArgument("execution_stage", default_value="full"),
            DeclareLaunchArgument("shake_control_mode", default_value="cartesian"),
            DeclareLaunchArgument("shake_center_x", default_value="0.28"),
            DeclareLaunchArgument("shake_center_y", default_value="-0.30"),
            DeclareLaunchArgument("shake_center_z", default_value="0.62"),
            DeclareLaunchArgument("shake_approach_height", default_value="0.10"),
            DeclareLaunchArgument("shake_amplitude_x", default_value="0.100"),
            DeclareLaunchArgument("shake_amplitude_y", default_value="0.040"),
            DeclareLaunchArgument("shake_amplitude_z", default_value="0.055"),
            DeclareLaunchArgument("shake_cycles", default_value="4"),
            DeclareLaunchArgument("shake_twist_rx_deg", default_value="6.0"),
            DeclareLaunchArgument("shake_twist_ry_deg", default_value="3.0"),
            DeclareLaunchArgument("shake_twist_rz_deg", default_value="22.0"),
            DeclareLaunchArgument("line_time", default_value="0.0"),
            DeclareLaunchArgument("approach_line_velocity", default_value="20.0"),
            DeclareLaunchArgument("approach_line_acceleration", default_value="25.0"),
            DeclareLaunchArgument("approach_line_time", default_value="3.5"),
            DeclareLaunchArgument("shake_line_velocity", default_value="85.0"),
            DeclareLaunchArgument("shake_line_acceleration", default_value="130.0"),
            DeclareLaunchArgument("shake_line_time", default_value="0.40"),
            DeclareLaunchArgument("service_wait_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("motion_response_timeout_sec", default_value="10.0"),
            DeclareLaunchArgument("precheck_ikin_joint5", default_value="true"),
            DeclareLaunchArgument("enforce_wrist_joint_limits", default_value="false"),
            DeclareLaunchArgument("ikin_sol_space", default_value="2"),
            DeclareLaunchArgument("joint5_min_deg", default_value="40.0"),
            DeclareLaunchArgument("joint5_max_deg", default_value="100.0"),
            DeclareLaunchArgument("wrist_min_deg", default_value="-135.0"),
            DeclareLaunchArgument("wrist_max_deg", default_value="135.0"),
            DeclareLaunchArgument("joint_shake_base_j1_deg", default_value="0.0"),
            DeclareLaunchArgument("joint_shake_base_j2_deg", default_value="-35.0"),
            DeclareLaunchArgument("joint_shake_base_j3_deg", default_value="50.0"),
            DeclareLaunchArgument("joint_shake_base_j4_deg", default_value="0.0"),
            DeclareLaunchArgument("joint_shake_base_j5_deg", default_value="70.0"),
            DeclareLaunchArgument("joint_shake_base_j6_deg", default_value="0.0"),
            DeclareLaunchArgument("joint_shake_j3_amplitude_deg", default_value="0.0"),
            DeclareLaunchArgument("joint_shake_j4_amplitude_deg", default_value="25.0"),
            DeclareLaunchArgument("joint_shake_j5_amplitude_deg", default_value="30.0"),
            DeclareLaunchArgument("joint_shake_j6_amplitude_deg", default_value="37.0"),
            DeclareLaunchArgument("joint_shake_j1_min_deg", default_value="-20.0"),
            DeclareLaunchArgument("joint_shake_j1_max_deg", default_value="5.0"),
            DeclareLaunchArgument("joint_shake_j2_min_deg", default_value="-80.0"),
            DeclareLaunchArgument("joint_shake_j2_max_deg", default_value="5.0"),
            DeclareLaunchArgument("joint_shake_j3_min_deg", default_value="0.0"),
            DeclareLaunchArgument("joint_shake_j3_max_deg", default_value="135.0"),
            DeclareLaunchArgument("joint_shake_max_single_delta_deg", default_value="75.0"),
            DeclareLaunchArgument("approach_joint_velocity", default_value="18.0"),
            DeclareLaunchArgument("approach_joint_acceleration", default_value="22.0"),
            DeclareLaunchArgument("approach_joint_time", default_value="2.6"),
            DeclareLaunchArgument("shake_joint_velocity", default_value="125.0"),
            DeclareLaunchArgument("shake_joint_acceleration", default_value="190.0"),
            DeclareLaunchArgument("shake_joint_time", default_value="0.24"),
            DeclareLaunchArgument("verify_joint_targets", default_value="true"),
            DeclareLaunchArgument("joint_target_tolerance_deg", default_value="8.0"),
            DeclareLaunchArgument("joint_target_wait_extra_sec", default_value="3.0"),
            DeclareLaunchArgument("joint_target_poll_sec", default_value="0.05"),
            DeclareLaunchArgument("require_state_validity_for_joint_shake", default_value="false"),
            DeclareLaunchArgument("state_validity_service", default_value="/check_state_validity"),
            DeclareLaunchArgument("planning_group", default_value="manipulator"),
            DeclareLaunchArgument("min_shake_z", default_value="0.55"),
            DeclareLaunchArgument("dispenser_keepout_radius", default_value="0.20"),
            DeclareLaunchArgument("rx", default_value="180.0"),
            DeclareLaunchArgument("ry", default_value="0.0"),
            DeclareLaunchArgument("rz", default_value="180.0"),
            DeclareLaunchArgument("use_visualizer", default_value="true"),
            Node(
                package="azas_motion",
                executable="tumbler_shake_sequence_node",
                name="tumbler_shake_sequence_node",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="azas_motion",
                executable="shake_visualizer_node",
                name="shake_visualizer_node",
                output="screen",
                parameters=[
                    {
                        "shake_center_x": ParameterValue(shake_center_x, value_type=float),
                        "shake_center_y": ParameterValue(shake_center_y, value_type=float),
                        "shake_center_z": ParameterValue(shake_center_z, value_type=float),
                        "shake_amplitude_x": ParameterValue(shake_amplitude_x, value_type=float),
                        "shake_amplitude_y": ParameterValue(shake_amplitude_y, value_type=float),
                        "shake_amplitude_z": ParameterValue(shake_amplitude_z, value_type=float),
                        "publish_demo_arm": False,
                    }
                ],
                condition=IfCondition(use_visualizer),
            ),
        ]
    )
