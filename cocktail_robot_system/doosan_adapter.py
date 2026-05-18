# Role: Adapter layer for Doosan M0609 robot and gripper commands.

from __future__ import annotations

import logging
import time
from typing import Any, List, Optional, Sequence, Union

from geometry_msgs.msg import Pose, PoseStamped
import rclpy
from rclpy.node import Node

PoseLike = Union[PoseStamped, Pose, Sequence[float]]


class DoosanAdapter:
    """Thin wrapper around Doosan APIs.

    The current implementation is intentionally safe: by default it only logs
    requested motions. Set use_real_robot=True and fill in the TODO sections
    after your Doosan ROS2 control stack is verified.
    """

    def __init__(
        self,
        node: Optional[Node] = None,
        robot_id: str = "dsr01",
        robot_model: str = "m0609",
        use_real_robot: bool = False,
        velocity: float = 50.0,
        acceleration: float = 50.0,
        rot_velocity: float = 30.0,
        rot_acceleration: float = 60.0,
        service_timeout_sec: float = 5.0,
        gripper_enabled: bool = False,
        gripper_name: str = "rg2",
        gripper_ip: str = "192.168.1.1",
        gripper_port: int = 502,
        gripper_open_width: int = 500,
        gripper_close_width: int = 200,
        gripper_force: int = 200,
    ) -> None:
        self.node = node
        self.robot_id = robot_id
        self.robot_model = robot_model
        self.use_real_robot = use_real_robot
        self.velocity = velocity
        self.acceleration = acceleration
        self.rot_velocity = rot_velocity
        self.rot_acceleration = rot_acceleration
        self.service_timeout_sec = service_timeout_sec
        self.gripper_enabled = gripper_enabled
        self.gripper_name = gripper_name
        self.gripper_ip = gripper_ip
        self.gripper_port = gripper_port
        self.gripper_open_width = gripper_open_width
        self.gripper_close_width = gripper_close_width
        self.gripper_force = gripper_force
        self._real_api_ready = False
        self._gripper: Optional[Any] = None

        self._move_line_client: Optional[Any] = None
        self._move_home_client: Optional[Any] = None
        self._move_wait_client: Optional[Any] = None
        self._get_current_posx_client: Optional[Any] = None
        self._MoveLine: Optional[Any] = None
        self._MoveHome: Optional[Any] = None
        self._MoveWait: Optional[Any] = None
        self._GetCurrentPosx: Optional[Any] = None

        if self.node is None:
            logging.basicConfig(level=logging.INFO)
            self._python_logger = logging.getLogger("DoosanAdapter")
        else:
            self._python_logger = None

        self._info(
            f"DoosanAdapter initialized. robot_id={robot_id}, "
            f"robot_model={robot_model}, use_real_robot={use_real_robot}"
        )

        if self.use_real_robot:
            self._connect_real_robot()

    def move_home(self) -> bool:
        self._info("Command requested: move_home()")

        if not self._can_use_real_robot():
            return True

        if self._MoveHome is None or self._move_home_client is None:
            self._warn("MoveHome service client is not ready.")
            return False

        req = self._MoveHome.Request()
        req.target = 1
        response = self._call_service(self._move_home_client, req)
        return bool(response is not None and getattr(response, "success", False))

    def move_linear(self, pose: PoseLike) -> bool:
        pose_list = self._pose_to_list(pose)

        if not self._can_use_real_robot():
            self._info(f"Command requested: move_linear({pose_list})")
            return True

        if len(pose_list) == 6:
            return self.move_linear_posx(pose_list)

        self._info(
            "Command requested: move_linear("
            f"x={pose_list[0]:.3f}, y={pose_list[1]:.3f}, z={pose_list[2]:.3f}, "
            f"qx={pose_list[3]:.3f}, qy={pose_list[4]:.3f}, "
            f"qz={pose_list[5]:.3f}, qw={pose_list[6]:.3f})"
        )

        self._warn(
            "move_linear(Pose/PoseStamped/quaternion) cannot be sent directly to "
            "Doosan MoveLine. Use move_linear_posx([x_mm,y_mm,z_mm,rx,ry,rz])."
        )
        return False

    def move_linear_posx(self, pose_mm_deg: Sequence[float]) -> bool:
        """Send a Doosan MoveLine command using posx format.

        pose_mm_deg is [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg].
        """
        pose = [float(v) for v in pose_mm_deg]
        if len(pose) != 6:
            raise ValueError("Doosan posx pose must have 6 values.")

        self._info(
            "Command requested: move_linear_posx("
            f"x={pose[0]:.1f}, y={pose[1]:.1f}, z={pose[2]:.1f}, "
            f"rx={pose[3]:.2f}, ry={pose[4]:.2f}, rz={pose[5]:.2f})"
        )

        if not self._can_use_real_robot():
            return True

        if self._MoveLine is None or self._move_line_client is None:
            self._warn("MoveLine service client is not ready.")
            return False

        req = self._MoveLine.Request()
        req.pos = pose
        req.vel = [float(self.velocity), float(self.rot_velocity)]
        req.acc = [float(self.acceleration), float(self.rot_acceleration)]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0

        response = self._call_service(self._move_line_client, req)
        success = bool(response is not None and getattr(response, "success", False))
        if not success:
            self._warn("MoveLine service returned failure.")
            return False

        return self.move_wait()

    def move_wait(self) -> bool:
        if not self._can_use_real_robot():
            return True

        if self._MoveWait is None or self._move_wait_client is None:
            self._warn("MoveWait service client is not ready; skipping wait.")
            return True

        req = self._MoveWait.Request()
        response = self._call_service(self._move_wait_client, req)
        return bool(response is not None and getattr(response, "success", False))

    def get_current_posx(self, ref: int = 0) -> Optional[List[float]]:
        if not self._can_use_real_robot():
            return None

        if self._GetCurrentPosx is None or self._get_current_posx_client is None:
            self._warn("GetCurrentPosx service client is not ready.")
            return None

        req = self._GetCurrentPosx.Request()
        req.ref = int(ref)
        response = self._call_service(self._get_current_posx_client, req)
        if response is None or not getattr(response, "success", False):
            self._warn("GetCurrentPosx service returned failure.")
            return None

        task_pos_info = getattr(response, "task_pos_info", [])
        if not task_pos_info:
            return None

        values = list(task_pos_info[0].data)
        return [float(v) for v in values]

    def gripper_open(self) -> bool:
        self._info("Command requested: gripper_open()")

        if not self._can_use_real_robot():
            return True

        if not self.gripper_enabled:
            self._warn("gripper_enabled=False; skipping real gripper_open().")
            return True

        if self._gripper is None:
            self._warn("Gripper is not connected.")
            return False

        try:
            self._gripper.move_gripper(
                width_val=self.gripper_open_width, force_val=self.gripper_force
            )
            time.sleep(0.8)
            return True
        except Exception as exc:
            self._warn(f"gripper_open() failed: {exc}")
            return False

    def gripper_close(self) -> bool:
        self._info("Command requested: gripper_close()")

        if not self._can_use_real_robot():
            return True

        if not self.gripper_enabled:
            self._warn("gripper_enabled=False; skipping real gripper_close().")
            return True

        if self._gripper is None:
            self._warn("Gripper is not connected.")
            return False

        try:
            self._gripper.move_gripper(
                width_val=self.gripper_close_width, force_val=self.gripper_force
            )
            time.sleep(1.0)
            return True
        except Exception as exc:
            self._warn(f"gripper_close() failed: {exc}")
            return False

    def gripper_check(self) -> bool:
        self._info("Command requested: gripper_check()")

        if not self._can_use_real_robot():
            self._info("Simulated gripper_check(): returning True")
            return True

        if not self.gripper_enabled:
            return True

        if self._gripper is None:
            self._warn("Gripper is not connected.")
            return False

        try:
            status = self._gripper.get_status()
            busy = bool(status[0])
            grip_detected = bool(status[1])
            self._info(f"Gripper status: busy={busy}, grip_detected={grip_detected}")
            return grip_detected
        except Exception as exc:
            self._warn(f"gripper_check() failed: {exc}")
            return False

    def rotate_tool_z(self, delta_deg: float) -> bool:
        self._info(f"Command requested: rotate_tool_z(delta_deg={delta_deg:.2f})")

        if not self._can_use_real_robot():
            return True

        # TODO: Implement tool-frame rotation around local Z.
        self._warn("Real rotate_tool_z() is not implemented yet.")
        return False

    def _connect_real_robot(self) -> None:
        try:
            if self.node is None:
                self._warn("Real robot mode requires an rclpy Node.")
                self._real_api_ready = False
                return

            from dsr_msgs2.srv import GetCurrentPosx, MoveHome, MoveLine, MoveWait

            namespace = f"/{self.robot_id}"
            self._MoveLine = MoveLine
            self._MoveHome = MoveHome
            self._MoveWait = MoveWait
            self._GetCurrentPosx = GetCurrentPosx
            self._move_line_client = self.node.create_client(
                MoveLine, f"{namespace}/motion/move_line"
            )
            self._move_home_client = self.node.create_client(
                MoveHome, f"{namespace}/motion/move_home"
            )
            self._move_wait_client = self.node.create_client(
                MoveWait, f"{namespace}/motion/move_wait"
            )
            self._get_current_posx_client = self.node.create_client(
                GetCurrentPosx, f"{namespace}/aux_control/get_current_posx"
            )

            required_clients = [
                (self._move_line_client, "motion/move_line"),
                (self._move_wait_client, "motion/move_wait"),
                (self._get_current_posx_client, "aux_control/get_current_posx"),
            ]
            for client, name in required_clients:
                if not client.wait_for_service(timeout_sec=self.service_timeout_sec):
                    self._warn(f"Doosan service not available: {namespace}/{name}")
                    self._real_api_ready = False
                    return

            self._real_api_ready = True
            self._info(f"Connected to Doosan services under namespace {namespace}.")

            if self.gripper_enabled:
                self._connect_gripper()
        except Exception as exc:
            self._real_api_ready = False
            self._warn(f"Failed to connect to Doosan API: {exc}")

    def _can_use_real_robot(self) -> bool:
        if not self.use_real_robot:
            self._info("Simulation/log-only mode: command accepted without hardware.")
            return False

        if not self._real_api_ready:
            self._warn("Real robot requested, but Doosan API is not ready.")
            return False

        return True

    def _connect_gripper(self) -> None:
        try:
            from dsr_practice.onrobot import RG

            self._gripper = RG(
                gripper=self.gripper_name,
                ip=self.gripper_ip,
                port=self.gripper_port,
            )
            time.sleep(0.5)
            self._info(
                f"Connected to {self.gripper_name} gripper at "
                f"{self.gripper_ip}:{self.gripper_port}."
            )
        except Exception as exc:
            self._gripper = None
            self._warn(f"Failed to connect gripper: {exc}")

    def _call_service(self, client: Any, request: Any) -> Optional[Any]:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=self.service_timeout_sec
        )
        if not future.done():
            self._warn("Service call timed out.")
            return None

        try:
            return future.result()
        except Exception as exc:
            self._warn(f"Service call failed: {exc}")
            return None

    def _pose_to_list(self, pose: PoseLike) -> List[float]:
        if isinstance(pose, PoseStamped):
            return self._pose_to_list(pose.pose)

        if isinstance(pose, Pose):
            return [
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ]

        values = [float(v) for v in pose]
        if len(values) == 3:
            return [values[0], values[1], values[2], 0.0, 0.0, 0.0, 1.0]
        if len(values) == 6:
            return values
        if len(values) == 7:
            return values

        raise ValueError(
            "Pose must be PoseStamped, Pose, [x,y,z], [x,y,z,r,p,y], "
            "or [x,y,z,qx,qy,qz,qw]."
        )

    def _info(self, message: str) -> None:
        if self.node is not None:
            self.node.get_logger().info(message)
        elif self._python_logger is not None:
            self._python_logger.info(message)

    def _warn(self, message: str) -> None:
        if self.node is not None:
            self.node.get_logger().warn(message)
        elif self._python_logger is not None:
            self._python_logger.warning(message)
