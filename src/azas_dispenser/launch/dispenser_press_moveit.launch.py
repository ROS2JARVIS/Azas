from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    dispenser_press_moveit_params = {
        "group_name": "manipulator",
        "base_frame": "base_link",
        "ee_link": "rg2_tcp",
        "ik_link_name": "link_6",
        "tool_offset_xyz": [0.0, 0.0, 0.27],
        "service_prefix": LaunchConfiguration("service_prefix"),
        "keep_home_pose_from_controller": ParameterValue(
            LaunchConfiguration("keep_home_pose_from_controller"), value_type=bool
        ),
        "use_taught_posx": ParameterValue(
            LaunchConfiguration("use_taught_posx"), value_type=bool
        ),
        "use_taught_orientation": ParameterValue(
            LaunchConfiguration("use_taught_orientation"), value_type=bool
        ),
        "target_dispenser": LaunchConfiguration("target_dispenser"),
        "joint_names": [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ],
        "home_joints_deg": [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
        # FK of home_joints_deg for m0609 in the current Doosan MoveIt setup.
        # Position unit: meter, orientation unit: degree.
        "home_tcp": [0.368, 0.00625, 0.425],
        "home_rpy_deg": [45.0, 180.0, 45.0],
        "red_top_posx": [
            732.1023559570312,
            64.33094787597656,
            379.1507568359375,
            174.0473175048828,
            -118.16372680664062,
            -149.73670959472656,
        ],
        "green_top_posx": [
            733.4710083007812,
            3.988441228866577,
            379.1507568359375,
            168.5689239501953,
            -117.13253784179688,
            -149.81581115722656,
        ],
        "yellow_top_posx": [
            736.9231567382812,
            -54.69612121582031,
            379.1507568359375,
            164.23757934570312,
            -114.83785247802734,
            -150.598876953125,
        ],
        "blue_top_posx": [
            730.6580200195312,
            -109.8679428100586,
            379.1507568359375,
            158.76589965820312,
            -114.91173553466797,
            -156.96270751953125,
        ],
        # Dispenser pump target in base_link frame.
        "dispenser_x": 0.50,
        "dispenser_y": 0.00,
        "dispenser_y_offset": 0.05,
        "dispenser_top_z": 0.38,
        "approach_height": 0.05,
        "home_lift_height": 0.05,
        "press_depth": 0.03,
        "hold_seconds": 0.5,
        "approach_pause_seconds": 0.5,
        "allowed_planning_time": 5.0,
        "max_velocity_scaling": 0.15,
        "max_acceleration_scaling": 0.15,
        "goal_tolerance_rad": 0.01,
        "move_home_first": True,
        "return_home": True,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "service_prefix",
                default_value="dsr01",
                description="Doosan service namespace for optional controller pose reads.",
            ),
            DeclareLaunchArgument(
                "keep_home_pose_from_controller",
                default_value="false",
                description="Read HOME TCP pose from Doosan services before MoveIt execution.",
            ),
            DeclareLaunchArgument(
                "use_taught_posx",
                default_value="true",
                description="Use color-specific taught dispenser poses.",
            ),
            DeclareLaunchArgument(
                "use_taught_orientation",
                default_value="false",
                description="Use taught dispenser RPY values instead of the HOME RPY.",
            ),
            DeclareLaunchArgument(
                "target_dispenser",
                default_value="red",
                description="Dispenser color to press: red, green, yellow, or blue.",
            ),
            Node(
                package="azas_dispenser",
                executable="dispenser_press_moveit_node",
                output="screen",
                parameters=[dispenser_press_moveit_params],
            )
        ]
    )
