from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dispenser_press_moveit_params = {
        "group_name": "manipulator",
        "base_frame": "base_link",
        "ee_link": "rg2_tcp",
        "ik_link_name": "link_6",
        "tool_offset_xyz": [0.0, 0.0, 0.27],
        "service_prefix": "/",
        "keep_home_pose_from_controller": True,
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
            Node(
                package="azas_dispenser",
                executable="dispenser_press_moveit_node",
                output="screen",
                parameters=[dispenser_press_moveit_params],
            )
        ]
    )
