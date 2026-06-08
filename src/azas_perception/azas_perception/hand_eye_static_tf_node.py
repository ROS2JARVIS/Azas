from __future__ import annotations

from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


def default_hand_eye_matrix_path() -> str:
    return str(Path(get_package_share_directory("azas_perception")) / "config" / "T_gripper2camera.npy")


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q))
    if norm <= 0.0 or not np.isfinite(norm):
        raise ValueError("rotation produced an invalid quaternion")
    return q / norm


def quaternion_from_matrix(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("rotation matrix must be 3x3")

    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (matrix[2, 1] - matrix[1, 2]) / s
        qy = (matrix[0, 2] - matrix[2, 0]) / s
        qz = (matrix[1, 0] - matrix[0, 1]) / s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (matrix[0, 1] + matrix[1, 0]) / s
        qz = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / s
        qx = (matrix[0, 1] + matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / s
        qx = (matrix[0, 2] + matrix[2, 0]) / s
        qy = (matrix[1, 2] + matrix[2, 1]) / s
        qz = 0.25 * s

    return normalize_quaternion(np.array([qx, qy, qz, qw], dtype=float))


def matrix_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    q = normalize_quaternion(np.array([qx, qy, qz, qw], dtype=float))
    x, y, z, w = q
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_from_transform(transform: TransformStamped) -> np.ndarray:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = matrix_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def transform_from_matrix(
    matrix: np.ndarray,
    parent_frame: str,
    child_frame: str,
    stamp,
) -> TransformStamped:
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = float(matrix[0, 3])
    transform.transform.translation.y = float(matrix[1, 3])
    transform.transform.translation.z = float(matrix[2, 3])
    qx, qy, qz, qw = quaternion_from_matrix(matrix[:3, :3])
    transform.transform.rotation.x = float(qx)
    transform.transform.rotation.y = float(qy)
    transform.transform.rotation.z = float(qz)
    transform.transform.rotation.w = float(qw)
    return transform


def load_hand_eye_matrix(path: str | Path, translation_scale: float) -> np.ndarray:
    matrix_path = Path(path)
    if not matrix_path.is_file():
        raise FileNotFoundError(f"hand-eye matrix file not found: {matrix_path}")

    matrix = np.asarray(np.load(str(matrix_path)), dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"hand-eye matrix must be 4x4, got {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("hand-eye matrix contains non-finite values")
    if not np.allclose(matrix[3, :], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-6):
        raise ValueError("hand-eye matrix bottom row must be [0, 0, 0, 1]")
    if not np.isfinite(translation_scale) or translation_scale <= 0.0:
        raise ValueError("translation_scale must be a positive finite value")

    scaled = matrix.copy()
    scaled[:3, 3] *= float(translation_scale)
    rotation = scaled[:3, :3]
    orthogonality_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
    determinant = float(np.linalg.det(rotation))
    if orthogonality_error > 1e-3:
        raise ValueError(f"hand-eye rotation is not orthonormal: error={orthogonality_error:.6g}")
    if abs(determinant - 1.0) > 1e-3:
        raise ValueError(f"hand-eye rotation determinant must be near +1, got {determinant:.6g}")
    return scaled


def compose_parent_to_published_child(
    parent_from_matrix_child: np.ndarray,
    published_child_from_matrix_child: np.ndarray,
) -> np.ndarray:
    return parent_from_matrix_child @ np.linalg.inv(published_child_from_matrix_child)


class HandEyeStaticTfNode(Node):
    """Publish measured hand-eye calibration as a TF tree connection.

    The bundled legacy calibration matrix is expressed in the camera optical
    frame. RealSense usually already owns camera_link -> camera_color_optical_frame,
    so this node composes the measured matrix with that existing camera TF and
    publishes link_6 -> camera_link. That connects the camera tree to the robot
    tree without assigning a second parent to camera_color_optical_frame.
    """

    def __init__(self):
        super().__init__("hand_eye_static_tf_node")
        self.declare_parameter("matrix_path", "")
        self.declare_parameter("parent_frame", "link_6")
        self.declare_parameter("matrix_child_frame", "camera_color_optical_frame")
        self.declare_parameter("published_child_frame", "camera_link")
        self.declare_parameter("translation_scale", 0.001)
        self.declare_parameter("compose_with_existing_tf", True)
        self.declare_parameter("compose_timeout_sec", 5.0)
        self.declare_parameter("allow_direct_fallback", False)

        self._broadcaster = StaticTransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._started_at = time.monotonic()
        self._published = False

        matrix_path = str(self.get_parameter("matrix_path").value).strip() or default_hand_eye_matrix_path()
        translation_scale = float(self.get_parameter("translation_scale").value)
        try:
            self._parent_from_matrix_child = load_hand_eye_matrix(matrix_path, translation_scale)
        except Exception as exc:
            self.get_logger().error(f"Cannot publish hand-eye TF: {exc}")
            self._timer = None
            return

        self._matrix_path = matrix_path
        self._timer = self.create_timer(0.1, self._try_publish)
        self.get_logger().info(
            "Loaded hand-eye matrix: "
            f"path={matrix_path} parent={self.get_parameter('parent_frame').value} "
            f"matrix_child={self.get_parameter('matrix_child_frame').value} "
            f"published_child={self.get_parameter('published_child_frame').value} "
            f"translation_scale={translation_scale}"
        )

    def _try_publish(self) -> None:
        if self._published:
            return

        parent_frame = str(self.get_parameter("parent_frame").value).strip()
        matrix_child_frame = str(self.get_parameter("matrix_child_frame").value).strip()
        published_child_frame = str(self.get_parameter("published_child_frame").value).strip()
        if not parent_frame or not matrix_child_frame or not published_child_frame:
            self.get_logger().error("parent_frame, matrix_child_frame, and published_child_frame are required")
            self._stop_timer()
            return

        transform_matrix = self._parent_from_matrix_child
        compose = (
            bool(self.get_parameter("compose_with_existing_tf").value)
            and published_child_frame != matrix_child_frame
        )
        if compose:
            try:
                camera_transform = self._tf_buffer.lookup_transform(
                    published_child_frame,
                    matrix_child_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
                transform_matrix = compose_parent_to_published_child(
                    self._parent_from_matrix_child,
                    matrix_from_transform(camera_transform),
                )
            except TransformException as exc:
                timeout_sec = float(self.get_parameter("compose_timeout_sec").value)
                if time.monotonic() - self._started_at < timeout_sec:
                    return
                if bool(self.get_parameter("allow_direct_fallback").value):
                    self.get_logger().warn(
                        "Falling back to direct hand-eye TF because camera frame composition failed: "
                        f"{published_child_frame}->{matrix_child_frame}: {exc}"
                    )
                    published_child_frame = matrix_child_frame
                    transform_matrix = self._parent_from_matrix_child
                else:
                    self.get_logger().error(
                        "Cannot compose hand-eye TF with existing camera tree; "
                        f"missing TF target={published_child_frame} source={matrix_child_frame}: {exc}"
                    )
                    self._stop_timer()
                    return

        transform = transform_from_matrix(
            transform_matrix,
            parent_frame,
            published_child_frame,
            self.get_clock().now().to_msg(),
        )
        self._broadcaster.sendTransform(transform)
        self._published = True
        self._stop_timer()
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.get_logger().info(
            "Published measured hand-eye static TF: "
            f"{parent_frame} -> {published_child_frame} "
            f"xyz=({translation.x:.6f}, {translation.y:.6f}, {translation.z:.6f}) "
            f"quat=({rotation.x:.6f}, {rotation.y:.6f}, {rotation.z:.6f}, {rotation.w:.6f}) "
            f"source_matrix={self._matrix_path}"
        )

    def _stop_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeStaticTfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
