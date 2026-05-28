import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _robot_description(model: str, color: str, namespace: str) -> str:
    description_share = get_package_share_directory("dsr_description2")
    xacro_path = os.path.join(description_share, "xacro", f"{model}.urdf.xacro")

    with open(xacro_path, "r", encoding="utf-8") as f:
        xacro_text = f.read()

    # The installed M0609 Gazebo xacro path omits update_rate when calling the
    # gz ros2_control macro. Patch only the temporary launch-time copy.
    xacro_text = xacro_text.replace(
        '<xacro:m0609_gz_ros2_control namespace="$(arg namespace)"/>',
        '<xacro:m0609_gz_ros2_control namespace="$(arg namespace)" update_rate="${update_rate}"/>',
    )

    with tempfile.NamedTemporaryFile("w", suffix=".urdf.xacro", delete=False, encoding="utf-8") as f:
        f.write(xacro_text)
        patched_xacro_path = f.name

    try:
        xml = xacro.process_file(
            patched_xacro_path,
            mappings={
                "use_gazebo": "true",
                "color": color,
                "namespace": namespace,
                "model": model,
                "update_rate": "100",
            },
        ).toxml()
    finally:
        try:
            os.unlink(patched_xacro_path)
        except OSError:
            pass

    # The upstream Doosan Gazebo branch targets Gazebo Sim/Ignition. This launch
    # adapts the same robot description to the installed Gazebo Classic stack.
    xml = re.sub(
        r'<plugin\s+filename="ign_ros2_control-system"\s+name="ign_ros2_control::IgnitionROS2ControlPlugin">',
        '<plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">',
        xml,
    )
    xml = xml.replace("ign_ros2_control/IgnitionSystem", "gazebo_ros2_control/GazeboSystem")
    xml = re.sub(r"<!--.*?-->", "", xml, flags=re.DOTALL)
    return " ".join(xml.split())


def _launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("name").perform(context)
    model = LaunchConfiguration("model").perform(context)
    color = LaunchConfiguration("color").perform(context)
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    roll = LaunchConfiguration("R").perform(context)
    pitch = LaunchConfiguration("P").perform(context)
    yaw = LaunchConfiguration("Y").perform(context)
    namespace = f"/{name}/gz"
    robot_description = _robot_description(model, color, namespace)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            "m0609",
            "-topic",
            f"{namespace}/robot_description",
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-R",
            roll,
            "-P",
            pitch,
            "-Y",
            yaw,
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        output="screen",
        arguments=["dsr_position_controller", "--controller-manager", "controller_manager"],
    )

    controllers = TimerAction(period=4.0, actions=[joint_state_broadcaster, position_controller])

    return [gazebo, robot_state_publisher, spawn, controllers]


def generate_launch_description():
    name_arg = DeclareLaunchArgument("name", default_value="dsr01")
    model_arg = DeclareLaunchArgument("model", default_value="m0609")
    color_arg = DeclareLaunchArgument("color", default_value="white")
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.1525")
    roll_arg = DeclareLaunchArgument("R", default_value="0.0")
    pitch_arg = DeclareLaunchArgument("P", default_value="0.0")
    yaw_arg = DeclareLaunchArgument("Y", default_value="0.0")

    return LaunchDescription(
        [
            name_arg,
            model_arg,
            color_arg,
            x_arg,
            y_arg,
            z_arg,
            roll_arg,
            pitch_arg,
            yaw_arg,
            OpaqueFunction(function=_launch_setup),
        ]
    )
