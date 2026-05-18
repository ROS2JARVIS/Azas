# Role: Minimal state machine skeleton for the future cup/lid closing workflow.

from __future__ import annotations

import json
from enum import Enum, auto
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LidCloseState(Enum):
    WAIT_SENSOR = auto()
    DETECT_OBJECTS = auto()
    ESTIMATE_3D = auto()
    MOVE_PREGRASP = auto()
    DONE = auto()
    ERROR = auto()

    # Future extension states:
    # PICK_CUP = auto()
    # PLACE_CUP_IN_HOLDER = auto()
    # PICK_LID = auto()
    # ALIGN_LID = auto()
    # LOWER_TO_CONTACT = auto()
    # ROTATE_TIGHTEN = auto()
    # VERIFY_CLOSE = auto()


class LidCloseStateMachine(Node):
    """Small runnable skeleton that advances through the first workflow states."""

    def __init__(self) -> None:
        super().__init__("lid_close_state_machine")

        self.declare_parameter(
            "detections_3d_topic", "/cocktail/detection_3d/detections"
        )
        self.declare_parameter("timer_period_sec", 0.5)

        self.detections_3d_topic = str(
            self.get_parameter("detections_3d_topic").value
        )
        self.state = LidCloseState.WAIT_SENSOR
        self.latest_detection_count = 0
        self.latest_valid_3d_count = 0

        self.sub = self.create_subscription(
            String, self.detections_3d_topic, self._detections_3d_callback, 10
        )
        self.timer = self.create_timer(
            float(self.get_parameter("timer_period_sec").value), self._tick
        )

        self.get_logger().info(
            f"LidCloseStateMachine started in state {self.state.name}."
        )

    def _detections_3d_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid 3D detection JSON: {exc}")
            self._transition(LidCloseState.ERROR)
            return

        detections = payload.get("detections", [])
        self.latest_detection_count = len(detections)
        self.latest_valid_3d_count = len(
            [det for det in detections if "robot_xyz" in det]
        )

    def _tick(self) -> None:
        try:
            if self.state == LidCloseState.WAIT_SENSOR:
                if self.latest_detection_count > 0:
                    self._transition(LidCloseState.DETECT_OBJECTS)

            elif self.state == LidCloseState.DETECT_OBJECTS:
                self.get_logger().info(
                    f"Detected objects count={self.latest_detection_count}"
                )
                self._transition(LidCloseState.ESTIMATE_3D)

            elif self.state == LidCloseState.ESTIMATE_3D:
                if self.latest_valid_3d_count > 0:
                    self._transition(LidCloseState.MOVE_PREGRASP)
                else:
                    self.get_logger().warn("No valid 3D detections yet.")

            elif self.state == LidCloseState.MOVE_PREGRASP:
                self.get_logger().info(
                    "MOVE_PREGRASP skeleton reached. "
                    "Actual movement is handled by robot_move_test for now."
                )
                self._transition(LidCloseState.DONE)

            elif self.state == LidCloseState.DONE:
                return

            elif self.state == LidCloseState.ERROR:
                self.get_logger().error("State machine is in ERROR state.")

        except Exception as exc:
            self.get_logger().error(f"State machine tick failed: {exc}")
            self._transition(LidCloseState.ERROR)

    def _transition(self, next_state: LidCloseState) -> None:
        if self.state == next_state:
            return
        self.get_logger().info(f"State transition: {self.state.name} -> {next_state.name}")
        self.state = next_state


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LidCloseStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
