from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="m0609", package_name="dsr_moveit_config_m0609")
        .robot_description()
        .robot_description_semantic("config/dsr.srdf")
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("dispenser_id", default_value="1"),
            DeclareLaunchArgument("press_count", default_value="2"),
            DeclareLaunchArgument("press_only", default_value="false"),
            DeclareLaunchArgument("start_delay_sec", default_value="4.0"),
            DeclareLaunchArgument("dispenser_x", default_value="0.50"),
            DeclareLaunchArgument("dispenser_y", default_value="0.00"),
            DeclareLaunchArgument("cup_place_z", default_value=""),
            DeclareLaunchArgument("cup_lift_z", default_value="0.54"),
            DeclareLaunchArgument("cup_lift_m", default_value="0.08"),
            DeclareLaunchArgument("cup_pre_grasp_backoff_m", default_value="0.08"),
            DeclareLaunchArgument("cup_release_retract_m", default_value="0.05"),
            DeclareLaunchArgument("press_ready_z", default_value="0.54"),
            DeclareLaunchArgument("press_down_m", default_value="0.08"),
            DeclareLaunchArgument("press_up_m", default_value="0.05"),
            DeclareLaunchArgument("trajectory_time_scale", default_value="5.0"),
            DeclareLaunchArgument("planning_time_sec", default_value="5.0"),
            DeclareLaunchArgument("joint_states_topic", default_value="/dsr01/joint_states"),
            DeclareLaunchArgument(
                "moveit_controller_action",
                default_value="/dsr01/dsr_moveit_controller/follow_joint_trajectory",
            ),
            Node(
                package="azas_motion",
                executable="dispenser_press_cycle_moveit_node",
                name="dispenser_press_cycle_moveit_node",
                output="screen",
                remappings=[
                    ("/joint_states", LaunchConfiguration("joint_states_topic")),
                    (
                        "dsr_moveit_controller/follow_joint_trajectory",
                        LaunchConfiguration("moveit_controller_action"),
                    ),
                    (
                        "/dsr_moveit_controller/follow_joint_trajectory",
                        LaunchConfiguration("moveit_controller_action"),
                    ),
                ],
                additional_env={
                    "DISPENSER_ID": LaunchConfiguration("dispenser_id"),
                    "PRESS_COUNT": LaunchConfiguration("press_count"),
                    "PRESS_ONLY": LaunchConfiguration("press_only"),
                    "DISPENSER_X": LaunchConfiguration("dispenser_x"),
                    "DISPENSER_Y": LaunchConfiguration("dispenser_y"),
                    "CUP_PLACE_Z": LaunchConfiguration("cup_place_z"),
                    "CUP_LIFT_Z": LaunchConfiguration("cup_lift_z"),
                    "CUP_LIFT_M": LaunchConfiguration("cup_lift_m"),
                    "CUP_PRE_GRASP_BACKOFF_M": LaunchConfiguration("cup_pre_grasp_backoff_m"),
                    "CUP_RELEASE_RETRACT_M": LaunchConfiguration("cup_release_retract_m"),
                    "PRESS_READY_Z": LaunchConfiguration("press_ready_z"),
                    "PRESS_DOWN_M": LaunchConfiguration("press_down_m"),
                    "PRESS_UP_M": LaunchConfiguration("press_up_m"),
                    "TRAJECTORY_TIME_SCALE": LaunchConfiguration("trajectory_time_scale"),
                    "PLANNING_TIME_SEC": LaunchConfiguration("planning_time_sec"),
                },
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                    {
                        "planning_scene_monitor_options": {
                            "joint_state_topic": LaunchConfiguration("joint_states_topic"),
                        },
                        "moveit_simple_controller_manager": {
                            "controller_names": ["/dsr01/dsr_moveit_controller"],
                            "/dsr01/dsr_moveit_controller": {
                                "action_ns": "follow_joint_trajectory",
                                "type": "FollowJointTrajectory",
                                "default": True,
                                "joints": [
                                    "joint_1",
                                    "joint_2",
                                    "joint_3",
                                    "joint_4",
                                    "joint_5",
                                    "joint_6",
                                ],
                            },
                        },
                        "press_count": ParameterValue(LaunchConfiguration("press_count"), value_type=int),
                        "start_delay_sec": ParameterValue(LaunchConfiguration("start_delay_sec"), value_type=float),
                        "dispenser_x": ParameterValue(LaunchConfiguration("dispenser_x"), value_type=float),
                        "dispenser_y": ParameterValue(LaunchConfiguration("dispenser_y"), value_type=float),
                        "cup_lift_z": ParameterValue(LaunchConfiguration("cup_lift_z"), value_type=float),
                        "press_down_m": ParameterValue(LaunchConfiguration("press_down_m"), value_type=float),
                        "press_up_m": ParameterValue(LaunchConfiguration("press_up_m"), value_type=float),
                    },
                ],
            ),
        ]
    )
