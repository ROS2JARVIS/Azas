from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
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
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("azas_perception"),
            "config",
            "yolo_cup_uprighting_best.pt",
        ]),
        description="YOLO weights for cup uprighting.",
    )
    publish_hand_eye_tf_arg = DeclareLaunchArgument(
        "publish_hand_eye_tf",
        default_value="true",
        description="Publish measured base_link -> camera_color_optical_frame TF.",
    )
    auto_pick_arg = DeclareLaunchArgument(
        "auto_pick",
        default_value="false",
        description="Automatically run the first detected fallen-cup upright sequence. false keeps manual p-key confirmation.",
    )
    skip_initial_home_move_arg = DeclareLaunchArgument(
        "skip_initial_home_move",
        default_value="false",
        description="Use the current robot pose as the camera observation pose without commanding Home first.",
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

    world_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="cup_uprighting_world_base_tf",
        output="screen",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--yaw", "0",
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "world",
            "--child-frame-id", "base_link",
        ],
    )

    hand_eye_tf = Node(
        package="azas_perception",
        executable="hand_eye_static_tf_node",
        name="cup_uprighting_hand_eye_static_tf_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_hand_eye_tf")),
        parameters=[{
            "compose_timeout_sec": 30.0,
            "allow_direct_fallback": False,
        }],
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
            {
                "model_path": ParameterValue(
                    LaunchConfiguration("model_path"),
                    value_type=str,
                ),
                "auto_pick": LaunchConfiguration("auto_pick"),
                "skip_initial_home_move": LaunchConfiguration("skip_initial_home_move"),
            },
        ],
    )

    return LaunchDescription([
        model_path_arg,
        publish_hand_eye_tf_arg,
        auto_pick_arg,
        skip_initial_home_move_arg,
        workspace_collision_scene,
        world_base_tf,
        hand_eye_tf,
        yolo_cup_uprighting_node,
    ])
