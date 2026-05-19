import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def rviz_node_function(context):
    model = LaunchConfiguration("model").perform(context)
    package_name = f"dsr_moveit_config_{model}"
    FindPackageShare(package_name).perform(context)

    moveit_config = (
        MoveItConfigsBuilder(model, "robot_description", package_name)
        .robot_description(file_path=f"config/{model}.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
            load_all=False,
        )
        .to_moveit_configs()
    )

    rviz_config = os.path.join(
        get_package_share_directory(package_name), "launch", "moveit.rviz"
    )
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="m0609"),
            OpaqueFunction(function=rviz_node_function),
        ]
    )
