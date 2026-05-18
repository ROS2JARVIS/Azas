from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    selected_dispenser_id = LaunchConfiguration("selected_dispenser_id")
    use_robot_urdf = LaunchConfiguration("use_robot_urdf")
    animate_robot_joints = LaunchConfiguration("animate_robot_joints")
    enable_ik_preview = LaunchConfiguration("enable_ik_preview")
    shake_delay_sec = LaunchConfiguration("shake_delay_sec")
    grasp_x = LaunchConfiguration("grasp_x")
    grasp_y = LaunchConfiguration("grasp_y")
    grasp_z = LaunchConfiguration("grasp_z")
    mouth_x = LaunchConfiguration("mouth_x")
    mouth_y = LaunchConfiguration("mouth_y")
    mouth_z = LaunchConfiguration("mouth_z")
    shake_center_x = LaunchConfiguration("shake_center_x")
    shake_center_y = LaunchConfiguration("shake_center_y")
    shake_center_z = LaunchConfiguration("shake_center_z")
    shake_amplitude_x = LaunchConfiguration("shake_amplitude_x")
    shake_amplitude_y = LaunchConfiguration("shake_amplitude_y")
    shake_amplitude_z = LaunchConfiguration("shake_amplitude_z")
    shake_cycles = LaunchConfiguration("shake_cycles")
    min_shake_z = LaunchConfiguration("min_shake_z")
    dispenser_keepout_radius = LaunchConfiguration("dispenser_keepout_radius")
    show_sequence_markers = LaunchConfiguration("show_sequence_markers")
    show_dispenser_markers = LaunchConfiguration("show_dispenser_markers")
    show_animated_cup = LaunchConfiguration("show_animated_cup")
    show_demo_arm = LaunchConfiguration("show_demo_arm")
    use_shake_visualizer = LaunchConfiguration("use_shake_visualizer")

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
            "grasp_x": grasp_x,
            "grasp_y": grasp_y,
            "grasp_z": grasp_z,
            "mouth_x": mouth_x,
            "mouth_y": mouth_y,
            "mouth_z": mouth_z,
            "show_sequence_markers": show_sequence_markers,
            "show_dispenser_markers": show_dispenser_markers,
            "show_animated_cup": show_animated_cup,
            "show_demo_arm": show_demo_arm,
        }.items(),
    )

    outlet_hold_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("azas_bringup"), "launch", "tumbler_floor_place.launch.py"]
            )
        ),
        launch_arguments={
            "selected_dispenser_id": selected_dispenser_id,
            "delivery_mode": "hold_under_outlet",
            "execution_stage": "full",
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
            "use_visualizer": use_shake_visualizer,
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
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("selected_dispenser_id", default_value="2"),
            DeclareLaunchArgument("use_robot_urdf", default_value="true"),
            DeclareLaunchArgument("animate_robot_joints", default_value="true"),
            DeclareLaunchArgument("enable_ik_preview", default_value="true"),
            DeclareLaunchArgument("shake_delay_sec", default_value="10.0"),
            DeclareLaunchArgument("grasp_x", default_value="0.42"),
            DeclareLaunchArgument("grasp_y", default_value="-0.24"),
            DeclareLaunchArgument("grasp_z", default_value="0.05"),
            DeclareLaunchArgument("mouth_x", default_value="0.42"),
            DeclareLaunchArgument("mouth_y", default_value="-0.24"),
            DeclareLaunchArgument("mouth_z", default_value="0.22"),
            DeclareLaunchArgument("shake_center_x", default_value="0.28"),
            DeclareLaunchArgument("shake_center_y", default_value="-0.30"),
            DeclareLaunchArgument("shake_center_z", default_value="0.62"),
            DeclareLaunchArgument("shake_amplitude_x", default_value="0.100"),
            DeclareLaunchArgument("shake_amplitude_y", default_value="0.040"),
            DeclareLaunchArgument("shake_amplitude_z", default_value="0.055"),
            DeclareLaunchArgument("shake_cycles", default_value="4"),
            DeclareLaunchArgument("min_shake_z", default_value="0.55"),
            DeclareLaunchArgument("dispenser_keepout_radius", default_value="0.20"),
            DeclareLaunchArgument("show_sequence_markers", default_value="false"),
            DeclareLaunchArgument("show_dispenser_markers", default_value="false"),
            DeclareLaunchArgument("show_animated_cup", default_value="false"),
            DeclareLaunchArgument("show_demo_arm", default_value="false"),
            DeclareLaunchArgument("use_shake_visualizer", default_value="false"),
            demo_launch,
            Node(
                package="azas_motion",
                executable="m0609_shake_joint_state_node",
                name="m0609_side_grasp_move_shake_joint_state_node",
                output="screen",
                parameters=[{"preview_mode": "side_grasp_move_then_shake"}],
                condition=IfCondition(animate_robot_joints),
            ),
            TimerAction(period=2.0, actions=[outlet_hold_launch]),
            TimerAction(period=shake_delay_sec, actions=[shake_launch]),
        ]
    )
