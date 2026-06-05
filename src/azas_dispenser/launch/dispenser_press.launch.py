from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    dispenser_press_params = {
        # Use "/" when Doosan services are not namespaced.
        "service_prefix": LaunchConfiguration("service_prefix"),
        "tcp_name": LaunchConfiguration("tcp_name"),
        "restore_tcp_after_run": LaunchConfiguration("restore_tcp_after_run"),
        "require_tcp_for_taught_posx": LaunchConfiguration("require_tcp_for_taught_posx"),
        "allow_tcp_set_failure": LaunchConfiguration("allow_tcp_set_failure"),
        "close_gripper_at_home": LaunchConfiguration("close_gripper_at_home"),
        "gripper_service": LaunchConfiguration("gripper_service"),
        "gripper_close_width": LaunchConfiguration("gripper_close_width"),
        "gripper_close_force": LaunchConfiguration("gripper_close_force"),
        "gripper_wait_timeout": LaunchConfiguration("gripper_wait_timeout"),
        # True이면 실제로 찍어둔 색상별 펌프 상단 TCP 좌표를 사용합니다.
        "use_taught_posx": ParameterValue(
            LaunchConfiguration("use_taught_posx"), value_type=bool
        ),
        "target_dispenser": LaunchConfiguration("target_dispenser"),
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
        # False이면 아래 dispenser_x/y/top_z 좌표로 이동해서 누릅니다.
        "use_home_as_reference": False,
        # True이면 HOME 자세 고정 대신 미리 찾은 press-ready 관절 자세로 접근합니다.
        "use_press_ready_pose": False,
        # HOME에서 읽은 TCP 자세(rx, ry, rz)를 유지합니다.
        "keep_home_orientation": False,
        # base_link 기준 디스펜서 펌프 상단 중앙 위치입니다.
        "dispenser_x": 0.50,
        "dispenser_y": 0.00,
        # 디스펜서 중심보다 y 방향으로 5 cm 높은 위치에서 누릅니다.
        "dispenser_y_offset": 0.05,
        # 디스펜서 높이 (펌프를 누르기 시작하는 높이)
        "dispenser_top_z": 0.65,
        "approach_height": 0.05,
        "transit_height": 0.10,
        # 홈에서 얼마나 올릴 것인지
        "home_lift_height": 0.05,
        "press_depth": 0.04,
        "hold_seconds": 0.5,
        "approach_pause_seconds": 0.5,
        "move_home_first": True,
        "return_home": True,
        # Doosan posx orientation is rx, ry, rz in degrees.
        "rx": 90.0,
        "ry": 180.0,
        "rz": 90.0,
        "home_joints_deg": [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
        "press_ready_joints_deg": [6.58, 6.94, 57.71, -15.02, 26.12, -76.44],
        "joint_velocity": LaunchConfiguration("joint_velocity"),
        "joint_acceleration": LaunchConfiguration("joint_acceleration"),
        "line_velocity": LaunchConfiguration("line_velocity"),
        "line_acceleration": LaunchConfiguration("line_acceleration"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "service_prefix",
                default_value="dsr01",
                description='Doosan service namespace. Use "/" for no namespace.',
            ),
            DeclareLaunchArgument(
                "target_dispenser",
                default_value="red",
                description="Dispenser color to press: red, green, yellow, or blue.",
            ),
            DeclareLaunchArgument(
                "use_taught_posx",
                default_value="true",
                description="Use color-specific taught dispenser TCP poses.",
            ),
            DeclareLaunchArgument(
                "tcp_name",
                default_value="",
                description="Doosan controller TCP name to activate before taught-posx press.",
            ),
            DeclareLaunchArgument(
                "restore_tcp_after_run",
                default_value="true",
                description="Restore the previous Doosan TCP after the press sequence.",
            ),
            DeclareLaunchArgument(
                "require_tcp_for_taught_posx",
                default_value="true",
                description="Require a named TCP when using taught dispenser posx targets.",
            ),
            DeclareLaunchArgument(
                "allow_tcp_set_failure",
                default_value="false",
                description="Continue with the current controller TCP if tcp/set_current_tcp fails.",
            ),
            DeclareLaunchArgument(
                "close_gripper_at_home",
                default_value="true",
                description="Close the gripper after HOME motion before dispenser press.",
            ),
            DeclareLaunchArgument(
                "gripper_service",
                default_value="/azas/gripper/open_close",
                description="SetGripper service name used for gripper close.",
            ),
            DeclareLaunchArgument(
                "gripper_close_width",
                default_value="0.0",
                description="Target close width in meters for SetGripper.",
            ),
            DeclareLaunchArgument(
                "gripper_close_force",
                default_value="20.0",
                description="Target close force in newtons for SetGripper.",
            ),
            DeclareLaunchArgument(
                "gripper_wait_timeout",
                default_value="2.0",
                description="Seconds to wait for the gripper service before continuing.",
            ),
            DeclareLaunchArgument(
                "joint_velocity",
                default_value="30.0",
                description="Doosan movej velocity.",
            ),
            DeclareLaunchArgument(
                "joint_acceleration",
                default_value="30.0",
                description="Doosan movej acceleration.",
            ),
            DeclareLaunchArgument(
                "line_velocity",
                default_value="20.0",
                description="Doosan movel translational/rotational velocity.",
            ),
            DeclareLaunchArgument(
                "line_acceleration",
                default_value="30.0",
                description="Doosan movel translational/rotational acceleration.",
            ),
            Node(
                package="azas_dispenser",
                executable="dispenser_press_node",
                output="screen",
                parameters=[
                    dispenser_press_params,
                ],
            )
        ]
    )
