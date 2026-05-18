from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """MVP-1 skeleton launch: perception, calibration, gripper, motion, task action.

    This launch is still no-motion oriented. It wires the pose-producing bridge
    so the action server can receive a base_link tumbler pose, but it does not
    bypass calibration/safety gates or enable real robot execution.
    """
    return LaunchDescription(
        [
            Node(
                package="azas_perception",
                executable="yolo_tumbler_detector_node",
                name="yolo_tumbler_detector_node",
                output="screen",
            ),
            # The action server consumes /jarvis/tumbler_dispenser/tumbler_pose.
            # Without this bridge, YOLO detections never become robot-frame poses.
            Node(
                package="azas_perception",
                executable="cup_detection_pose_bridge_node",
                name="cup_detection_pose_bridge_node",
                output="screen",
            ),
            Node(
                package="azas_calibration",
                executable="calibration_loader_node",
                name="calibration_loader_node",
                output="screen",
            ),
            Node(
                package="azas_gripper",
                executable="rg2_gripper_node",
                name="rg2_gripper_node",
                output="screen",
            ),
            Node(
                package="azas_motion",
                executable="alignment_executor_node",
                name="alignment_executor_node",
                output="screen",
            ),
            Node(
                package="azas_task_manager",
                executable="pick_and_align_action_server",
                name="pick_and_align_action_server",
                output="screen",
            ),
        ]
    )
