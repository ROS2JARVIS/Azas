import rclpy
from azas_interfaces.msg import CupDetection
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


class CupDetectionPoseBridgeNode(Node):
    """Bridge verified cup detections into the base-frame tumbler pose topic."""

    def __init__(self):
        super().__init__("cup_detection_pose_bridge_node")
        self.declare_parameter("input_topic", "/azas/cup_detection")
        self.declare_parameter("output_topic", "/jarvis/tumbler_dispenser/tumbler_pose")
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("use_grasp_pose", True)
        self.declare_parameter("require_status_prefix", "detected")
        self.declare_parameter("require_upright_status", True)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("source_frame", "")
        self.declare_parameter("require_tf", True)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("transform_timeout_sec", -1.0)
        self.declare_parameter("use_latest_tf_when_stamp_zero", True)
        self.declare_parameter("allow_latest_tf_fallback", False)
        self.declare_parameter("debug_pose_logging", False)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.create_subscription(
            CupDetection,
            str(self.get_parameter("input_topic").value),
            self._on_detection,
            10,
        )
        self.get_logger().info(
            "CupDetection bridge ready: "
            f"{self.get_parameter('input_topic').value} -> {self.get_parameter('output_topic').value} "
            f"target_frame={self.get_parameter('target_frame').value} "
            f"source_frame={self.get_parameter('source_frame').value or '<msg.header.frame_id>'}"
        )

    def _on_detection(self, msg: CupDetection) -> None:
        # Only upright detections may be turned into robot-frame poses. A
        # rejected/ambiguous cup still publishes diagnostic status upstream, but
        # must not leak into the motion-facing pose topic.
        min_confidence = float(self.get_parameter("min_confidence").value)
        required = str(self.get_parameter("require_status_prefix").value)
        # CupDetection.confidence is float32 on the wire; allow a tiny epsilon so
        # values displayed as equal, e.g. 0.950, are not rejected by binary
        # representation noise.
        if msg.confidence + 1e-6 < min_confidence:
            self.get_logger().warn(
                f"Ignoring low confidence detection: {msg.confidence:.3f} < {min_confidence:.3f}"
            )
            return
        if required and not msg.status.startswith(required):
            self.get_logger().warn(
                f"Ignoring detection status={msg.status!r}; "
                f"error_code={self._orientation_error_code(msg.status)}"
            )
            return
        if bool(self.get_parameter("require_upright_status").value) and not msg.status.startswith(
            "detected:upright"
        ):
            self.get_logger().warn(
                "Ignoring non-upright cup detection; refusing to publish tumbler pose: "
                f"status={msg.status!r} error_code={self._orientation_error_code(msg.status)}"
            )
            return

        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        source_frame = str(self.get_parameter("source_frame").value).strip()
        if source_frame:
            pose_msg.header.frame_id = source_frame
        pose_msg.pose = (
            msg.grasp_pose
            if bool(self.get_parameter("use_grasp_pose").value)
            else msg.cup_mouth_center
        )
        pose_msg = self._to_target_frame(pose_msg)
        if pose_msg is None:
            return
        self._pub.publish(pose_msg)
        self.get_logger().info(
            "Published tumbler pose from detection: "
            f"x={pose_msg.pose.position.x:.3f} y={pose_msg.pose.position.y:.3f} "
            f"z={pose_msg.pose.position.z:.3f} frame={pose_msg.header.frame_id} conf={msg.confidence:.3f}"
        )

    def _to_target_frame(self, pose_msg: PoseStamped):
        # This bridge converts camera-frame detections into base_link. If TF is
        # absent or stale, fail closed instead of publishing a guessed pose.
        target_frame = str(self.get_parameter("target_frame").value).strip()
        require_tf = bool(self.get_parameter("require_tf").value)
        source_frame = pose_msg.header.frame_id
        debug_pose_logging = bool(self.get_parameter("debug_pose_logging").value)
        if debug_pose_logging:
            self._log_pose("Input cup pose", pose_msg)
        if not target_frame or pose_msg.header.frame_id == target_frame:
            return pose_msg

        timeout_sec = self._transform_timeout_sec()
        request_stamp = pose_msg.header.stamp
        use_latest_tf = (
            request_stamp.sec == 0
            and request_stamp.nanosec == 0
            and bool(self.get_parameter("use_latest_tf_when_stamp_zero").value)
        )
        try:
            timeout = rclpy.duration.Duration(seconds=timeout_sec)
            # TODO: Model eye-in-hand direct matrix multiplication as TF before publishing.
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time() if use_latest_tf else rclpy.time.Time.from_msg(request_stamp),
                timeout=timeout,
            )
        except TransformException as exc:
            if bool(self.get_parameter("allow_latest_tf_fallback").value) and not use_latest_tf:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=timeout_sec),
                    )
                    self.get_logger().warn(
                        "Using latest TF fallback after stamped lookup failed; "
                        "this is diagnostic-only and does not prove real-motion readiness: "
                        f"target={target_frame} source={source_frame} "
                        f"request_stamp={request_stamp.sec}.{request_stamp.nanosec:09d}: {exc}"
                    )
                except TransformException as fallback_exc:
                    return self._handle_tf_failure(
                        target_frame,
                        source_frame,
                        request_stamp,
                        timeout_sec,
                        require_tf,
                        fallback_exc,
                    )
            else:
                return self._handle_tf_failure(
                    target_frame,
                    source_frame,
                    request_stamp,
                    timeout_sec,
                    require_tf,
                    exc,
                )
        if use_latest_tf:
            self.get_logger().warn(
                "Using latest TF because detection stamp is zero; "
                "this is diagnostic-only and does not prove real-motion readiness"
            )

        transformed = PoseStamped()
        transformed.header.stamp = pose_msg.header.stamp
        transformed.header.frame_id = target_frame
        transformed.pose = do_transform_pose(pose_msg.pose, transform)
        if debug_pose_logging:
            self._log_pose("Transformed cup pose", transformed)
        return transformed

    def _handle_tf_failure(
        self,
        target_frame: str,
        source_frame: str,
        request_stamp,
        timeout_sec: float,
        require_tf: bool,
        exc: TransformException,
    ):
        message = (
            f"No TF target={target_frame} source={source_frame} "
            f"request_stamp={request_stamp.sec}.{request_stamp.nanosec:09d} "
            f"timeout_sec={timeout_sec:.3f}; "
            "refusing to publish robot-motion pose from camera-frame detection"
        )
        if require_tf:
            self.get_logger().error(f"{message}: {exc}")
            return None
        self.get_logger().warn(
            f"{message}; require_tf=false but output frame differs, fail-closed: {exc}"
        )
        return None

    def _transform_timeout_sec(self) -> float:
        transform_timeout_sec = float(self.get_parameter("transform_timeout_sec").value)
        if transform_timeout_sec >= 0.0:
            return transform_timeout_sec
        return float(self.get_parameter("tf_timeout_sec").value)

    def _log_pose(self, label: str, pose_msg: PoseStamped) -> None:
        position = pose_msg.pose.position
        orientation = pose_msg.pose.orientation
        stamp = pose_msg.header.stamp
        self.get_logger().info(
            f"{label}: frame={pose_msg.header.frame_id} "
            f"stamp={stamp.sec}.{stamp.nanosec:09d} "
            f"position=({position.x:.4f}, {position.y:.4f}, {position.z:.4f}) "
            f"orientation=({orientation.x:.4f}, {orientation.y:.4f}, "
            f"{orientation.z:.4f}, {orientation.w:.4f})"
        )

    @staticmethod
    def _orientation_error_code(status: str) -> str:
        if "unknown_orientation" in status or "orientation=unknown" in status:
            return "CUP_ORIENTATION_UNKNOWN"
        if status.startswith("rejected:") or not status.startswith("detected:upright"):
            return "CUP_ORIENTATION_NOT_UPRIGHT"
        return ""


def main(args=None):
    rclpy.init(args=args)
    node = CupDetectionPoseBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
