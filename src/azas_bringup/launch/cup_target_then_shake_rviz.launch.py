from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    cup_detection_topic = LaunchConfiguration("cup_detection_topic")
    tumbler_pose_topic = LaunchConfiguration("tumbler_pose_topic")
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    target_z = LaunchConfiguration("target_z")
    shake_delay_sec = LaunchConfiguration("shake_delay_sec")
    shake_center_x = LaunchConfiguration("shake_center_x")
    shake_center_y = LaunchConfiguration("shake_center_y")
    shake_center_z = LaunchConfiguration("shake_center_z")
    shake_amplitude_x = LaunchConfiguration("shake_amplitude_x")
    shake_amplitude_y = LaunchConfiguration("shake_amplitude_y")
    shake_amplitude_z = LaunchConfiguration("shake_amplitude_z")
    shake_cycles = LaunchConfiguration("shake_cycles")
    min_shake_z = LaunchConfiguration("min_shake_z")
    dispenser_keepout_radius = LaunchConfiguration("dispenser_keepout_radius")

    cup_target_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("azas_bringup"), "launch", "cup_target_move_demo.launch.py"]
            )
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "use_robot_urdf": "true",
            "animate_robot_joints": "false",
            "cup_detection_topic": cup_detection_topic,
            "tumbler_pose_topic": tumbler_pose_topic,
            "target_x": target_x,
            "target_y": target_y,
            "target_z": target_z,
            "grasp_x": LaunchConfiguration("grasp_x"),
            "grasp_y": LaunchConfiguration("grasp_y"),
            "grasp_z": LaunchConfiguration("grasp_z"),
            "mouth_x": LaunchConfiguration("mouth_x"),
            "mouth_y": LaunchConfiguration("mouth_y"),
            "mouth_z": LaunchConfiguration("mouth_z"),
        }.items(),
    )

    shake_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("azas_bringup"), "launch", "tumbler_shake_sequence.launch.py"]
            )
        ),
        launch_arguments={
            "enable_hardware": "false",
            "execution_stage": "full",
            "use_visualizer": "true",
            "shake_center_x": shake_center_x,
            "shake_center_y": shake_center_y,
            "shake_center_z": shake_center_z,
            "shake_amplitude_x": shake_amplitude_x,
            "shake_amplitude_y": shake_amplitude_y,
            "shake_amplitude_z": shake_amplitude_z,
            "shake_cycles": shake_cycles,
            "min_shake_z": min_shake_z,
            "dispenser_keepout_radius": dispenser_keepout_radius,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "cup_detection_topic",
                default_value="/azas/cup_target_then_shake/cup_detection",
            ),
            DeclareLaunchArgument(
                "tumbler_pose_topic",
                default_value="/azas/cup_target_then_shake/tumbler_pose",
            ),
            DeclareLaunchArgument("grasp_x", default_value="0.32"),
            DeclareLaunchArgument("grasp_y", default_value="-0.22"),
            DeclareLaunchArgument("grasp_z", default_value="0.05"),
            DeclareLaunchArgument("mouth_x", default_value="0.32"),
            DeclareLaunchArgument("mouth_y", default_value="-0.22"),
            DeclareLaunchArgument("mouth_z", default_value="0.22"),
            DeclareLaunchArgument("target_x", default_value="0.43"),
            DeclareLaunchArgument("target_y", default_value="0.08"),
            DeclareLaunchArgument("target_z", default_value="0.135"),
            DeclareLaunchArgument("shake_delay_sec", default_value="8.0"),
            DeclareLaunchArgument("shake_center_x", default_value="0.43"),
            DeclareLaunchArgument("shake_center_y", default_value="0.08"),
            DeclareLaunchArgument("shake_center_z", default_value="0.62"),
            DeclareLaunchArgument("shake_amplitude_x", default_value="0.100"),
            DeclareLaunchArgument("shake_amplitude_y", default_value="0.040"),
            DeclareLaunchArgument("shake_amplitude_z", default_value="0.055"),
            DeclareLaunchArgument("shake_cycles", default_value="4"),
            DeclareLaunchArgument("min_shake_z", default_value="0.55"),
            DeclareLaunchArgument("dispenser_keepout_radius", default_value="0.0"),
            cup_target_launch,
            Node(
                package="azas_motion",
                executable="m0609_shake_joint_state_node",
                name="m0609_cup_target_then_shake_joint_state_node",
                output="screen",
                parameters=[{"preview_mode": "side_grasp_move_then_shake"}],
            ),
            TimerAction(period=shake_delay_sec, actions=[shake_launch]),
        ]
    )
