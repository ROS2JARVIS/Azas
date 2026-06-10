#!/usr/bin/env python3
"""Publish a measured hand-eye matrix as a ROS static TF."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def quaternion_from_matrix(matrix: np.ndarray) -> tuple[float, float, float, float]:
    m00, m01, m02 = [float(value) for value in matrix[0]]
    m10, m11, m12 = [float(value) for value in matrix[1]]
    m20, m21, m22 = [float(value) for value in matrix[2]]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (m21 - m12) / scale
        qy = (m02 - m20) / scale
        qz = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / scale
        qx = 0.25 * scale
        qy = (m01 + m10) / scale
        qz = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / scale
        qx = (m01 + m10) / scale
        qy = 0.25 * scale
        qz = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / scale
        qx = (m02 + m20) / scale
        qy = (m12 + m21) / scale
        qz = 0.25 * scale
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        raise ValueError("rotation matrix produced a zero quaternion")
    return qx / norm, qy / norm, qz / norm, qw / norm


class HandEyeTfPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("hand_eye_tf_publisher_node")
        self.declare_parameter(
            "hand_eye_matrix_path",
            "/home/ssu/Azas/src/azas_perception/config/T_gripper2camera.npy",
        )
        self.declare_parameter("parent_frame", "link_6")
        self.declare_parameter("child_frame", "camera_color_optical_frame")
        self.declare_parameter("translation_scale", 0.001)

        matrix = self.load_matrix()
        transform = self.make_transform(matrix)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(transform)
        t = transform.transform.translation
        q = transform.transform.rotation
        self.get_logger().info(
            "Published measured hand-eye static TF "
            f"{transform.header.frame_id}->{transform.child_frame_id}: "
            f"xyz=({t.x:.6f},{t.y:.6f},{t.z:.6f}) "
            f"quat=({q.x:.6f},{q.y:.6f},{q.z:.6f},{q.w:.6f})"
        )

    def load_matrix(self) -> np.ndarray:
        path = Path(str(self.get_parameter("hand_eye_matrix_path").value)).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"hand-eye matrix does not exist: {path}")
        matrix = np.load(str(path)).astype(float)
        if matrix.shape != (4, 4):
            raise ValueError(f"hand-eye matrix must be 4x4: {path}")
        matrix = matrix.copy()
        matrix[:3, 3] *= float(self.get_parameter("translation_scale").value)
        return matrix

    def make_transform(self, matrix: np.ndarray) -> TransformStamped:
        qx, qy, qz, qw = quaternion_from_matrix(matrix[:3, :3])
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(self.get_parameter("parent_frame").value)
        transform.child_frame_id = str(self.get_parameter("child_frame").value)
        transform.transform.translation.x = float(matrix[0, 3])
        transform.transform.translation.y = float(matrix[1, 3])
        transform.transform.translation.z = float(matrix[2, 3])
        transform.transform.rotation.x = float(qx)
        transform.transform.rotation.y = float(qy)
        transform.transform.rotation.z = float(qz)
        transform.transform.rotation.w = float(qw)
        return transform


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandEyeTfPublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
