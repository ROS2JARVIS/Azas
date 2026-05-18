# Role: Subscribe to 3D detections and move the robot to a cup pre-grasp pose.

from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cocktail_robot_system.doosan_adapter import DoosanAdapter
from cocktail_robot_system.grasp_selector import HeuristicGraspSelector


class RobotMoveTest(Node):
    """Move above the detected cup center. This node does not grasp."""

    def __init__(self) -> None:
        super().__init__("robot_move_test")

        self.declare_parameter(
            "detections_3d_topic", "/cocktail/detection_3d/detections"
        )
        self.declare_parameter("target_class", "cup")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("pregrasp_z_offset", 0.10)
        self.declare_parameter("pregrasp_orientation_xyzw", [0.0, 1.0, 0.0, 0.0])
        self.declare_parameter("min_confidence", 0.40)
        self.declare_parameter("move_once", True)
        self.declare_parameter("move_home_on_start", False)
        self.declare_parameter("use_real_robot", False)
        self.declare_parameter("robot_id", "dsr01")
        self.declare_parameter("robot_model", "m0609")
        self.declare_parameter("robot_velocity", 50.0)
        self.declare_parameter("robot_acceleration", 50.0)

        self.detections_3d_topic = str(
            self.get_parameter("detections_3d_topic").value
        )
        self.target_class = str(self.get_parameter("target_class").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.pregrasp_z_offset = float(
            self.get_parameter("pregrasp_z_offset").value
        )
        self.pregrasp_orientation_xyzw = [
            float(v) for v in self.get_parameter("pregrasp_orientation_xyzw").value
        ]
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.move_once = bool(self.get_parameter("move_once").value)
        self.move_home_on_start = bool(
            self.get_parameter("move_home_on_start").value
        )

        self.selector = HeuristicGraspSelector()
        self.adapter = DoosanAdapter(
            node=self,
            robot_id=str(self.get_parameter("robot_id").value),
            robot_model=str(self.get_parameter("robot_model").value),
            use_real_robot=bool(self.get_parameter("use_real_robot").value),
            velocity=float(self.get_parameter("robot_velocity").value),
            acceleration=float(self.get_parameter("robot_acceleration").value),
        )
        self._already_moved = False

        self.sub = self.create_subscription(
            String, self.detections_3d_topic, self._detections_3d_callback, 10
        )

        if self.move_home_on_start:
            self.adapter.move_home()

        self.get_logger().info(
            "RobotMoveTest ready. Waiting for "
            f"{self.target_class} on {self.detections_3d_topic}"
        )

    def _detections_3d_callback(self, msg: String) -> None:
        if self.move_once and self._already_moved:
            return

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid 3D detection JSON: {exc}")
            return

        detections = payload.get("detections", [])
        if not detections:
            self.get_logger().debug("No 3D detections in the latest message.")
            return

        target = self.selector.select_target(
            detections, self.target_class, self.min_confidence
        )
        if target is None:
            self.get_logger().debug(
                f"No {self.target_class} detection above confidence "
                f"{self.min_confidence:.2f}."
            )
            return

        frame_id = str(payload.get("target_frame_id") or self.target_frame)
        try:
            result = self.selector.compute_pregrasp_pose(
                detection=target,
                z_offset=self.pregrasp_z_offset,
                orientation_xyzw=self.pregrasp_orientation_xyzw,
                frame_id=frame_id,
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to compute pre-grasp pose: {exc}")
            return

        result.pose.header.stamp = self.get_clock().now().to_msg()

        p = result.pose.pose.position
        q = result.pose.pose.orientation
        self.get_logger().info(
            f"Moving to {result.class_name} pre-grasp pose via {result.method}: "
            f"pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), "
            f"quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})"
        )

        success = self.adapter.move_linear(result.pose)
        if success:
            self._already_moved = True
            self.get_logger().info("Pre-grasp move command completed/accepted.")
        else:
            self.get_logger().warn("Pre-grasp move command was not completed.")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = RobotMoveTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
