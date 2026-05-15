from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dispenser_press_params = {
        # Use "/" when Doosan services are not namespaced.
        "service_prefix": LaunchConfiguration("service_prefix"),
        # True이면 실제로 찍어둔 색상별 펌프 상단 TCP 좌표를 사용합니다.
        "use_taught_posx": True,
        "target_dispenser": LaunchConfiguration("target_dispenser"),
        "red_top_posx": [
            732.1023559570312,
            64.33094787597656,
            375.81304931640625,
            174.0473175048828,
            -118.16372680664062,
            -149.73670959472656,
        ],
        "green_top_posx": [
            733.4710083007812,
            3.988441228866577,
            379.2102966308594,
            168.5689239501953,
            -117.13253784179688,
            -149.81581115722656,
        ],
        "yellow_top_posx": [
            736.9231567382812,
            -54.69612121582031,
            398.9580078125,
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
        "press_depth": 0.015,
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
        "joint_velocity": 20.0,
        "joint_acceleration": 20.0,
        "line_velocity": 10.0,
        "line_acceleration": 15.0,
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
