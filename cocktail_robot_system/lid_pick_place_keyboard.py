# Role: Show YOLO debug image and execute a real lid pick-and-place on 'p'.

from __future__ import annotations

import json
import threading
from typing import Any, Dict, List, Optional

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cocktail_robot_system.doosan_adapter import DoosanAdapter


class LidPickPlaceKeyboard(Node):
    """Keyboard-triggered real robot test for lid detection.

    The OpenCV window must be focused. Press:
      p: pick the latest detected lid and move it to the fixed place pose
      q or ESC: quit
    """

    def __init__(self) -> None:
        super().__init__("lid_pick_place_keyboard")

        self.declare_parameter("debug_image_topic", "/cocktail/vision/debug_image")
        self.declare_parameter(
            "detections_3d_topic", "/cocktail/detection_3d/detections"
        )
        self.declare_parameter("target_class", "lid")
        self.declare_parameter("min_confidence", 0.40)
        self.declare_parameter("execute_motion", False)
        self.declare_parameter("robot_id", "dsr01")
        self.declare_parameter("robot_model", "m0609")
        self.declare_parameter("use_real_robot", True)
        self.declare_parameter("linear_vel_mm_s", 30.0)
        self.declare_parameter("linear_acc_mm_s2", 60.0)
        self.declare_parameter("rot_vel_deg_s", 20.0)
        self.declare_parameter("rot_acc_deg_s2", 40.0)
        self.declare_parameter("service_timeout_sec", 10.0)
        self.declare_parameter("approach_offset_mm", 80.0)
        self.declare_parameter("grasp_z_offset_mm", 10.0)
        self.declare_parameter("lift_offset_mm", 120.0)
        self.declare_parameter("place_approach_offset_mm", 100.0)
        self.declare_parameter("release_retreat_offset_mm", 100.0)
        self.declare_parameter(
            "pick_orientation_rpy_deg",
            [108.40734100341797, -176.3208770751953, 175.9803924560547],
        )
        self.declare_parameter(
            "place_pose_mm_deg",
            [
                499.37957763671875,
                -15.573370933532715,
                155.22544860839844,
                108.40734100341797,
                -176.3208770751953,
                175.9803924560547,
            ],
        )
        self.declare_parameter("workspace_min_mm", [150.0, -450.0, 40.0])
        self.declare_parameter("workspace_max_mm", [750.0, 450.0, 650.0])
        self.declare_parameter("gripper_enabled", True)
        self.declare_parameter("gripper_name", "rg2")
        self.declare_parameter("gripper_ip", "192.168.1.1")
        self.declare_parameter("gripper_port", 502)
        self.declare_parameter("gripper_open_width", 500)
        self.declare_parameter("gripper_close_width", 200)
        self.declare_parameter("gripper_force", 200)

        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.detections_3d_topic = str(
            self.get_parameter("detections_3d_topic").value
        )
        self.target_class = str(self.get_parameter("target_class").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.execute_motion = bool(self.get_parameter("execute_motion").value)
        self.approach_offset_mm = float(
            self.get_parameter("approach_offset_mm").value
        )
        self.grasp_z_offset_mm = float(
            self.get_parameter("grasp_z_offset_mm").value
        )
        self.lift_offset_mm = float(self.get_parameter("lift_offset_mm").value)
        self.place_approach_offset_mm = float(
            self.get_parameter("place_approach_offset_mm").value
        )
        self.release_retreat_offset_mm = float(
            self.get_parameter("release_retreat_offset_mm").value
        )
        self.pick_orientation_rpy_deg = [
            float(v) for v in self.get_parameter("pick_orientation_rpy_deg").value
        ]
        self.place_pose_mm_deg = [
            float(v) for v in self.get_parameter("place_pose_mm_deg").value
        ]
        self.workspace_min_mm = [
            float(v) for v in self.get_parameter("workspace_min_mm").value
        ]
        self.workspace_max_mm = [
            float(v) for v in self.get_parameter("workspace_max_mm").value
        ]

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_detections: List[Dict[str, Any]] = []
        self.latest_target: Optional[Dict[str, Any]] = None
        self._sequence_running = False
        self._stop_requested = False
        self._lock = threading.Lock()

        self.adapter = DoosanAdapter(
            node=self,
            robot_id=str(self.get_parameter("robot_id").value),
            robot_model=str(self.get_parameter("robot_model").value),
            use_real_robot=bool(self.get_parameter("use_real_robot").value),
            velocity=float(self.get_parameter("linear_vel_mm_s").value),
            acceleration=float(self.get_parameter("linear_acc_mm_s2").value),
            rot_velocity=float(self.get_parameter("rot_vel_deg_s").value),
            rot_acceleration=float(self.get_parameter("rot_acc_deg_s2").value),
            service_timeout_sec=float(
                self.get_parameter("service_timeout_sec").value
            ),
            gripper_enabled=bool(self.get_parameter("gripper_enabled").value),
            gripper_name=str(self.get_parameter("gripper_name").value),
            gripper_ip=str(self.get_parameter("gripper_ip").value),
            gripper_port=int(self.get_parameter("gripper_port").value),
            gripper_open_width=int(self.get_parameter("gripper_open_width").value),
            gripper_close_width=int(self.get_parameter("gripper_close_width").value),
            gripper_force=int(self.get_parameter("gripper_force").value),
        )

        self.image_sub = self.create_subscription(
            Image, self.debug_image_topic, self._image_callback, 10
        )
        self.detection_sub = self.create_subscription(
            String, self.detections_3d_topic, self._detections_callback, 10
        )

        mode = "REAL MOTION ENABLED" if self.execute_motion else "DRY RUN ONLY"
        self.get_logger().warn(
            f"LidPickPlaceKeyboard ready: {mode}. Focus the OpenCV window and "
            "press 'p' to run the latest lid pick-and-place."
        )

    def _image_callback(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert debug image: {exc}")
            return

        with self._lock:
            self.latest_image = image

    def _detections_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid 3D detection JSON: {exc}")
            return

        detections = payload.get("detections", [])
        target = self._select_target(detections)
        with self._lock:
            self.latest_detections = detections
            self.latest_target = target

    def _select_target(
        self, detections: List[Dict[str, Any]]
    ) -> Optional[Dict[str, Any]]:
        candidates = [
            det
            for det in detections
            if det.get("class_name") == self.target_class
            and float(det.get("confidence", 0.0)) >= self.min_confidence
            and "robot_xyz" in det
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda det: float(det.get("confidence", 0.0)))

    def run(self) -> None:
        window = "Cocktail Lid Pick - press p"
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)

        while rclpy.ok() and not self._stop_requested:
            rclpy.spin_once(self, timeout_sec=0.02)

            frame = self._make_display_frame()
            if frame is not None:
                cv2.imshow(window, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                self._stop_requested = True
            elif key == ord("p"):
                self._handle_pick_key()

        cv2.destroyAllWindows()

    def _make_display_frame(self):
        with self._lock:
            frame = None if self.latest_image is None else self.latest_image.copy()
            target = None if self.latest_target is None else dict(self.latest_target)
            count = len(self.latest_detections)

        if frame is None:
            return None

        mode = "REAL" if self.execute_motion else "DRY-RUN"
        color = (0, 0, 255) if self.execute_motion else (0, 220, 255)
        cv2.putText(
            frame,
            f"{mode} | detections={count} | target={self.target_class} | p:pick q:quit",
            (20, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            color,
            2,
            cv2.LINE_AA,
        )

        if target is not None:
            x, y, z = [float(v) for v in target["robot_xyz"]]
            cv2.putText(
                frame,
                f"latest {self.target_class}: base=({x:.3f},{y:.3f},{z:.3f}) m "
                f"conf={float(target.get('confidence', 0.0)):.2f}",
                (20, 64),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (50, 255, 50),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                frame,
                "No valid lid target yet",
                (20, 64),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        return frame

    def _handle_pick_key(self) -> None:
        if self._sequence_running:
            self.get_logger().warn("Pick sequence is already running; ignoring 'p'.")
            return

        with self._lock:
            target = None if self.latest_target is None else dict(self.latest_target)

        if target is None:
            self.get_logger().warn("No valid lid detection. Move lid into view first.")
            return

        self._sequence_running = True
        try:
            self._run_pick_place_sequence(target)
        finally:
            self._sequence_running = False

    def _run_pick_place_sequence(self, target: Dict[str, Any]) -> None:
        pick_pose = self._target_to_pick_pose(target)
        if not self._pose_inside_workspace(pick_pose):
            self.get_logger().error(
                "Computed pick pose is outside workspace limits. "
                f"pose={self._fmt_pose(pick_pose)}"
            )
            return

        place_pose = list(self.place_pose_mm_deg)
        place_approach = list(place_pose)
        place_approach[2] += self.place_approach_offset_mm
        lift_pose = list(pick_pose)
        lift_pose[2] += self.lift_offset_mm
        pick_approach = list(pick_pose)
        pick_approach[2] += self.approach_offset_mm
        retreat_pose = list(place_pose)
        retreat_pose[2] += self.release_retreat_offset_mm

        sequence = [
            ("gripper_open", None),
            ("move_pick_approach", pick_approach),
            ("move_pick", pick_pose),
            ("gripper_close", None),
            ("move_lift", lift_pose),
            ("move_place_approach", place_approach),
            ("move_place", place_pose),
            ("gripper_open", None),
            ("move_retreat", retreat_pose),
        ]

        self.get_logger().warn("========== LID PICK PLACE SEQUENCE ==========")
        self.get_logger().warn(f"Target detection: {target}")
        for name, pose in sequence:
            if pose is None:
                self.get_logger().warn(f"{name}")
            else:
                self.get_logger().warn(f"{name}: {self._fmt_pose(pose)}")

        if not self.execute_motion:
            self.get_logger().warn(
                "execute_motion=False, so no robot command was sent. "
                "Relaunch with execute_motion:=true after checking the poses."
            )
            return

        for name, pose in sequence:
            self.get_logger().warn(f"Executing step: {name}")
            if name == "gripper_open":
                if not self.adapter.gripper_open():
                    self.get_logger().error("Failed at gripper_open.")
                    return
            elif name == "gripper_close":
                if not self.adapter.gripper_close():
                    self.get_logger().error("Failed at gripper_close.")
                    return
                self.adapter.gripper_check()
            elif pose is not None:
                if not self.adapter.move_linear_posx(pose):
                    self.get_logger().error(f"Failed at {name}.")
                    return

        self.get_logger().warn("========== LID PICK PLACE DONE ==========")

    def _target_to_pick_pose(self, target: Dict[str, Any]) -> List[float]:
        x_m, y_m, z_m = [float(v) for v in target["robot_xyz"]]
        return [
            x_m * 1000.0,
            y_m * 1000.0,
            z_m * 1000.0 + self.grasp_z_offset_mm,
            self.pick_orientation_rpy_deg[0],
            self.pick_orientation_rpy_deg[1],
            self.pick_orientation_rpy_deg[2],
        ]

    def _pose_inside_workspace(self, pose: List[float]) -> bool:
        x, y, z = pose[:3]
        min_x, min_y, min_z = self.workspace_min_mm
        max_x, max_y, max_z = self.workspace_max_mm
        return min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z

    def _fmt_pose(self, pose: List[float]) -> str:
        return (
            f"[{pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}, "
            f"{pose[3]:.2f}, {pose[4]:.2f}, {pose[5]:.2f}]"
        )


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LidPickPlaceKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
