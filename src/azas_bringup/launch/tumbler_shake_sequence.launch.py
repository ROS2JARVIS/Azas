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
    shake_center_x = LaunchConfiguration("shake_center_x")
    shake_center_y = LaunchConfiguration("shake_center_y")
    shake_center_z = LaunchConfiguration("shake_center_z")
    shake_amplitude_x = LaunchConfiguration("shake_amplitude_x")
    shake_amplitude_y = LaunchConfiguration("shake_amplitude_y")
    shake_amplitude_z = LaunchConfiguration("shake_amplitude_z")
    shake_cycles = LaunchConfiguration("shake_cycles")
    min_shake_z = LaunchConfiguration("min_shake_z")
    dispenser_keepout_radius = LaunchConfiguration("dispenser_keepout_radius")
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
        "shake_approach_height": 0.10,
        "shake_amplitude_x": ParameterValue(shake_amplitude_x, value_type=float),
        "shake_amplitude_y": ParameterValue(shake_amplitude_y, value_type=float),
        "shake_amplitude_z": ParameterValue(shake_amplitude_z, value_type=float),
        "shake_cycles": ParameterValue(shake_cycles, value_type=int),
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
        "rx": 180.0,
        "ry": 0.0,
        "rz": 180.0,
        "line_velocity": 45.0,
        "line_acceleration": 80.0,
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
            DeclareLaunchArgument("shake_center_x", default_value="0.28"),
            DeclareLaunchArgument("shake_center_y", default_value="-0.30"),
            DeclareLaunchArgument("shake_center_z", default_value="0.62"),
            DeclareLaunchArgument("shake_amplitude_x", default_value="0.100"),
            DeclareLaunchArgument("shake_amplitude_y", default_value="0.040"),
            DeclareLaunchArgument("shake_amplitude_z", default_value="0.055"),
            DeclareLaunchArgument("shake_cycles", default_value="4"),
            DeclareLaunchArgument("min_shake_z", default_value="0.55"),
            DeclareLaunchArgument("dispenser_keepout_radius", default_value="0.20"),
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
