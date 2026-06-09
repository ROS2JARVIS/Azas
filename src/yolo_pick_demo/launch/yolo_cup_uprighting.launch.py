from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 두산 M0609 로봇의 MoveIt 파라미터 빌드 (URDF, SRDF, Kinematics 등)
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="m0609",
            package_name="dsr_moveit_config_m0609",
        )
        .robot_description()
        .robot_description_semantic(file_path="config/dsr.srdf")
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_scene_monitor()
        .sensors_3d()
        .to_moveit_configs()
    )

    # 2. 패키지 내 config/moveit_py.yaml 경로 설정
    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("yolo_pick_demo"), "config", "moveit_py.yaml"]
    )

    # 3. 컵 직립화(Uprighting) 노드 실행 및 파라미터 주입
    yolo_cup_uprighting_node = Node(
        package="yolo_pick_demo",
        executable="yolo_cup_uprighting",
        name="yolo_cup_uprighting_py", # MoveItPy 초기화를 위해 소스 코드와 이름 일치
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_py_params,
        ],
    )

    return LaunchDescription([yolo_cup_uprighting_node])