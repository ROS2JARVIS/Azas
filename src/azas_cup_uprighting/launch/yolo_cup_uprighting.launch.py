from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        [FindPackageShare("azas_cup_uprighting"), "config", "moveit_py.yaml"]
    )

    # 3. 공통 안전/충돌 장면: side-grip, dispenser, cup-uprighting이 같은 바닥/벽/디스펜서 기준을 보도록 통일
    workspace_collision_scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("azas_bringup"),
                "launch",
                "workspace_collision_scene.launch.py",
            ])
        ),
        launch_arguments={
            "publish_collision_objects": "true",
            "table_collision_enabled": "true",
            "table_collision_expand_to_workspace_walls": "true",
            "workspace_boundary_collision_enabled": "true",
            "dispenser_collision_enabled": "true",
            "dispenser_collision_publish_objects": "true",
            "dispenser_collision_publish_markers": "true",
        }.items(),
    )

    # 4. 컵 직립화(Uprighting) 노드 실행 및 파라미터 주입
    yolo_cup_uprighting_node = Node(
        package="azas_cup_uprighting",
        executable="yolo_cup_uprighting",
        name="yolo_cup_uprighting_py", 
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_py_params,
        ],
    )

    return LaunchDescription([workspace_collision_scene, yolo_cup_uprighting_node])