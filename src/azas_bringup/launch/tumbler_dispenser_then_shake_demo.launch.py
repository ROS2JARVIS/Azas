from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    selected_dispenser_id = LaunchConfiguration("selected_dispenser_id")
    use_robot_urdf = LaunchConfiguration("use_robot_urdf")
    enable_ik_preview = LaunchConfiguration("enable_ik_preview")
    shake_delay_sec = LaunchConfiguration("shake_delay_sec")

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("azas_bringup"), "launch", "hardware_free_demo.launch.py"]
            )
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "selected_dispenser_id": selected_dispenser_id,
            "use_robot_urdf": use_robot_urdf,
            "enable_ik_preview": enable_ik_preview,
            "tumbler_pose_topic": "/azas/demo/tumbler_pose",
        }.items(),
    )

    pre_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("azas_bringup"), "launch", "tumbler_floor_place.launch.py"]
            )
        ),
        launch_arguments={
            "selected_dispenser_id": selected_dispenser_id,
            "execution_stage": "pre_place",
            "use_tumbler_pose_topic": "true",
            "tumbler_pose_topic": "/azas/demo/tumbler_pose",
            "enable_hardware": "false",
            "allow_demo_tumbler_position_fallback": "false",
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
            "shake_center_x": "0.28",
            "shake_center_y": "-0.30",
            "shake_center_z": "0.62",
            "shake_amplitude_x": "0.100",
            "shake_amplitude_y": "0.040",
            "shake_amplitude_z": "0.055",
            "shake_cycles": "4",
            "min_shake_z": "0.55",
            "dispenser_keepout_radius": "0.20",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("selected_dispenser_id", default_value="2"),
            DeclareLaunchArgument("use_robot_urdf", default_value="true"),
            DeclareLaunchArgument("enable_ik_preview", default_value="true"),
            DeclareLaunchArgument("shake_delay_sec", default_value="10.0"),
            demo_launch,
            TimerAction(period=2.0, actions=[pre_place_launch]),
            TimerAction(period=shake_delay_sec, actions=[shake_launch]),
        ]
    )
