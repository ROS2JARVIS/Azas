from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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

    pump_x_arg = DeclareLaunchArgument(
        "pump_x",
        default_value="0.45",
        description="Syrup pump x position in base_link frame [m].",
    )
    pump_y_arg = DeclareLaunchArgument(
        "pump_y",
        default_value="0.0",
        description="Syrup pump y position in base_link frame [m].",
    )
    start_z_arg = DeclareLaunchArgument(
        "start_z",
        default_value="0.50",
        description="Vertical approach start height [m].",
    )
    pump_top_z_arg = DeclareLaunchArgument(
        "pump_top_z",
        default_value="0.35",
        description="Syrup pump top height [m].",
    )
    press_depth_arg = DeclareLaunchArgument(
        "press_depth",
        default_value="0.05",
        description="Press depth from pump top [m].",
    )
    hold_sec_arg = DeclareLaunchArgument(
        "hold_sec",
        default_value="0.5",
        description="Holding time at pressed position [sec].",
    )

    return LaunchDescription(
        [
            pump_x_arg,
            pump_y_arg,
            start_z_arg,
            pump_top_z_arg,
            press_depth_arg,
            hold_sec_arg,
            Node(
                package="dsr_practice",
                executable="syrup_pump_press",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                    {
                        "pump_x": LaunchConfiguration("pump_x"),
                        "pump_y": LaunchConfiguration("pump_y"),
                        "start_z": LaunchConfiguration("start_z"),
                        "pump_top_z": LaunchConfiguration("pump_top_z"),
                        "press_depth": LaunchConfiguration("press_depth"),
                        "hold_sec": LaunchConfiguration("hold_sec"),
                    },
                ],
            ),
        ]
    )
