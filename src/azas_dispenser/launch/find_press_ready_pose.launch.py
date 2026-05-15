from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="azas_dispenser",
                executable="find_press_ready_pose_node",
                output="screen",
                parameters=[
                    {
                        "group_name": "manipulator",
                        "base_frame": "base_link",
                        "ee_link": "link_6",
                        "target_x": 0.50,
                        "target_y": 0.05,
                        "target_z": 0.70,
                        "max_solutions": 10,
                        "allowed_planning_time": 3.0,
                        "max_velocity_scaling": 0.15,
                        "max_acceleration_scaling": 0.15,
                    }
                ],
            )
        ]
    )
