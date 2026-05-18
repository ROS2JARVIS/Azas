import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node

try:
    from gpd_ros.msg import GraspConfigList
except ImportError:  # pragma: no cover - depends on optional ROS package install.
    GraspConfigList = None


class GpdGraspAdapterNode(Node):
    """Publish the best optional GPD grasp candidate as a PoseStamped."""

    def __init__(self):
        super().__init__("gpd_grasp_adapter_node")
        if GraspConfigList is None:
            raise RuntimeError(
                "gpd_ros.msg.GraspConfigList is required for gpd_grasp_adapter_node"
            )

        self.declare_parameter("input_topic", "/detect_grasps/grasps")
        self.declare_parameter("output_topic", "/azas/gpd/grasp_pose")
        self.declare_parameter("min_score", 0.0)
        self.declare_parameter("min_width_m", 0.0)
        self.declare_parameter("max_width_m", 0.12)

        self._pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.create_subscription(
            GraspConfigList,
            str(self.get_parameter("input_topic").value),
            self._on_grasps,
            10,
        )
        self.get_logger().info(
            "GPD grasp adapter ready: "
            f"{self.get_parameter('input_topic').value} -> {self.get_parameter('output_topic').value}"
        )

    def _on_grasps(self, msg) -> None:
        candidates = list(getattr(msg, "grasps", []))
        best = self._select_best_candidate(candidates)
        if best is None:
            self.get_logger().warn(
                f"No GPD grasp candidate passed filters: count={len(candidates)} "
                f"min_score={float(self.get_parameter('min_score').value):.3f} "
                f"min_width_m={float(self.get_parameter('min_width_m').value):.3f} "
                f"max_width_m={float(self.get_parameter('max_width_m').value):.3f}"
            )
            return

        index, grasp, score, width = best
        pose = self._pose_from_grasp(grasp)
        if pose is None:
            self.get_logger().warn(
                f"Rejecting GPD grasp candidate[{index}]: missing position or orientation axes"
            )
            return

        output = PoseStamped()
        output.header = msg.header
        output.pose = pose
        self._pub.publish(output)
        self.get_logger().info(
            "Published GPD grasp PoseStamped: "
            f"candidate_index={index} score={score:.3f} width_m={width:.4f} "
            f"frame={output.header.frame_id}"
        )

    def _select_best_candidate(self, candidates: list) -> Optional[tuple[int, object, float, float]]:
        min_score = float(self.get_parameter("min_score").value)
        min_width = float(self.get_parameter("min_width_m").value)
        max_width = float(self.get_parameter("max_width_m").value)

        best: Optional[tuple[int, object, float, float]] = None
        for index, grasp in enumerate(candidates):
            score = self._finite_float(getattr(grasp, "score", 0.0))
            width = self._finite_float(getattr(grasp, "width", math.nan))
            if score is None or width is None:
                continue
            if score < min_score or width < min_width or width > max_width:
                continue
            if best is None or score > best[2]:
                best = (index, grasp, score, width)
        return best

    def _pose_from_grasp(self, grasp) -> Optional[Pose]:
        position = self._grasp_position(grasp)
        orientation = self._orientation_from_grasp(grasp)
        if position is None or orientation is None:
            return None
        pose = Pose()
        pose.position = position
        pose.orientation = orientation
        return pose

    def _grasp_position(self, grasp) -> Optional[Point]:
        bottom = self._point_tuple(getattr(grasp, "bottom", None))
        top = self._point_tuple(getattr(grasp, "top", None))
        if bottom is not None and top is not None:
            return Point(
                x=(bottom[0] + top[0]) * 0.5,
                y=(bottom[1] + top[1]) * 0.5,
                z=(bottom[2] + top[2]) * 0.5,
            )

        for field in ("sample", "surface"):
            point = self._point_tuple(getattr(grasp, field, None))
            if point is not None:
                return Point(x=point[0], y=point[1], z=point[2])
        return None

    def _orientation_from_grasp(self, grasp) -> Optional[Quaternion]:
        x_axis = self._normalized(getattr(grasp, "axis", None))
        y_axis = self._normalized(getattr(grasp, "binormal", None))
        z_axis = self._normalized(getattr(grasp, "approach", None))
        if x_axis is None or y_axis is None or z_axis is None:
            return None
        return self._quaternion_from_matrix(
            (
                (x_axis[0], y_axis[0], z_axis[0]),
                (x_axis[1], y_axis[1], z_axis[1]),
                (x_axis[2], y_axis[2], z_axis[2]),
            )
        )

    @staticmethod
    def _finite_float(value) -> Optional[float]:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number):
            return None
        return number

    @classmethod
    def _point_tuple(cls, value) -> Optional[tuple[float, float, float]]:
        if value is None:
            return None
        x = cls._finite_float(getattr(value, "x", math.nan))
        y = cls._finite_float(getattr(value, "y", math.nan))
        z = cls._finite_float(getattr(value, "z", math.nan))
        if x is None or y is None or z is None:
            return None
        return (x, y, z)

    @classmethod
    def _normalized(cls, value) -> Optional[tuple[float, float, float]]:
        vector = cls._point_tuple(value)
        if vector is None:
            return None
        norm = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
        if norm <= 1e-9:
            return None
        return (vector[0] / norm, vector[1] / norm, vector[2] / norm)

    @staticmethod
    def _quaternion_from_matrix(matrix: tuple[tuple[float, float, float], ...]) -> Quaternion:
        m00, m01, m02 = matrix[0]
        m10, m11, m12 = matrix[1]
        m20, m21, m22 = matrix[2]
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
        return Quaternion(x=qx / norm, y=qy / norm, z=qz / norm, w=qw / norm)


def main(args=None):
    rclpy.init(args=args)
    node = GpdGraspAdapterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
