# src/azas_task_manager/azas_task_manager/full_cocktail_sequence_node.py

from __future__ import annotations

import json
import shlex
import subprocess
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


ROOT = Path("/home/ssu/Azas")

COLOR_TO_DISPENSER_ID = {
    "red": "1",
    "green": "2",
    "yellow": "3",
    "blue": "4",
}


class FullCocktailSequenceNode(Node):
    def __init__(self):
        super().__init__("full_cocktail_sequence_node")

        self.declare_parameter(
            "decision_topic",
            "/azas/voice/confirmed_recipe_decision",
        )
        self.declare_parameter("status_topic", "/azas/cocktail/full_status")
        self.declare_parameter("execute_real_motion", False)
        self.declare_parameter("service_prefix", "dsr01")

        self._running = False
        self._lock = threading.Lock()

        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )

        self.create_subscription(
            String,
            str(self.get_parameter("decision_topic").value),
            self._on_decision,
            10,
        )

        self.get_logger().warn(
            "Full cocktail sequence node ready. "
            "This node connects existing Azas task primitives into one flow."
        )

    def _publish_status(self, status: str, **kwargs):
        msg = String()
        msg.data = json.dumps(
            {"status": status, **kwargs},
            ensure_ascii=False,
        )
        self._status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def _on_decision(self, msg: String):
        try:
            decision = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._publish_status("blocked", reason="invalid_json", error=str(exc))
            return

        if not decision.get("confirmed"):
            self._publish_status("blocked", reason="not_confirmed")
            return

        if decision.get("intent") != "make_cocktail":
            self._publish_status("blocked", reason="not_make_cocktail", decision=decision)
            return

        with self._lock:
            if self._running:
                self._publish_status("blocked", reason="sequence_already_running")
                return
            self._running = True

        thread = threading.Thread(
            target=self._run_sequence,
            args=(decision,),
            daemon=True,
        )
        thread.start()

    def _run_sequence(self, decision: dict):
        try:
            self._publish_status("started", decision=decision)

            dispenser_ids = self._normalize_dispenser_ids(
                decision.get("dispenser_ids", [])
            )

            if not dispenser_ids:
                raise RuntimeError("no valid dispenser ids after normalization")

            self._move_observe_pose()
            self._pick_cup_side_grip()
            self._run_dispenser_sequence(dispenser_ids)
            self._place_cup_in_holder()

            self._move_observe_pose()
            self._pick_and_attach_lid()

            self._pick_cup_from_holder()
            self._shake_closed_cup()

            self._publish_status("completed")

        except Exception as exc:
            self._publish_status("failed", error=str(exc))
            self.get_logger().exception("full cocktail sequence failed")
        finally:
            with self._lock:
                self._running = False

    def _normalize_dispenser_ids(self, values) -> list[str]:
        result: list[str] = []

        for value in values:
            raw = str(value).strip().lower()

            if raw in {"1", "2", "3", "4"}:
                result.append(raw)
                continue

            if raw in COLOR_TO_DISPENSER_ID:
                result.append(COLOR_TO_DISPENSER_ID[raw])
                continue

            raise RuntimeError(f"unsupported dispenser id/color: {value}")

        return result

    def _run(self, command: str, *, timeout_sec: float | None = None):
        self._publish_status("running_command", command=command)

        completed = subprocess.run(
            ["bash", "-lc", command],
            cwd=str(ROOT),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_sec,
        )

        if completed.returncode != 0:
            raise RuntimeError(
                f"command failed rc={completed.returncode}\n"
                f"cmd={command}\n"
                f"output={completed.stdout[-3000:]}"
            )

        self._publish_status(
            "command_done",
            command=command,
            output_tail=completed.stdout[-1000:],
        )

    def _ros_env(self) -> str:
        return (
            "source /opt/ros/humble/setup.bash && "
            "source /home/ssu/ws_moveit/install/setup.bash && "
            "source /home/ssu/ros2_ws/install/setup.bash && "
            "source /home/ssu/Azas/install/setup.bash"
        )

    def _move_observe_pose(self):
        cmd = (
            f"{self._ros_env()} && "
            "python3 tools/run/direct_movej_joints.py "
            "--service-prefix dsr01 "
            "--j1 0 --j2 -5 --j3 50 --j4 0 --j5 135 --j6 0 "
            "--velocity 20 --acceleration 20 "
            "--execute --confirm ENABLE_DIRECT_MOVEJ"
        )
        self._run(cmd, timeout_sec=90)

    def _pick_cup_side_grip(self):
        cmd = (
            f"{self._ros_env()} && "
            "timeout 180s ros2 launch dsr_practice yolo_cup_pick_node.launch.py "
            "auto_pick:=true "
            "grasp_mode:=side "
            "move_to_camera_home:=true "
            "return_home_after_task:=false "
            "return_to_camera_home_after_attempt:=false "
            "dispenser_collision_enabled:=true "
            "workspace_collision_scene_enabled:=true "
            "table_collision_enabled:=true"
        )
        self._run(cmd, timeout_sec=190)

    def _run_dispenser_sequence(self, dispenser_ids: list[str]):
        joined = ",".join(dispenser_ids)
        cmd = (
            f"{self._ros_env()} && "
            "python3 tools/run/run_measured_dispenser_recipe_sequence.py "
            f"--dispenser-ids {shlex.quote(joined)} "
            "--service-prefix dsr01 "
            "--execute "
            "--confirm ENABLE_MEASURED_DISPENSER_RECIPE_SEQUENCE"
        )
        self._run(cmd, timeout_sec=900)

    def _place_cup_in_holder(self):
        cmd = (
            f"{self._ros_env()} && "
            "python3 tools/run/place_side_grip_cup_in_holder.py "
            "--service-prefix dsr01 "
            "--execute "
            "--confirm ENABLE_CUP_HOLDER_PLACE"
        )
        self._run(cmd, timeout_sec=180)

    def _pick_and_attach_lid(self):
        # 현재 develop 기준에서 가장 덜 완성된 구간.
        # lid_grip_planner_node는 존재하지만 컵홀더 위 결합 target pose를
        # calibration.yaml에 확정해 넣어야 완전 자동화가 가능함.
        raise RuntimeError(
            "lid attach is not fully wired yet. "
            "Add calibrated cup_holder.lid_attach pose and run lid_grip_planner_node "
            "with enable_lid_twist_after_grasp:=true."
        )

    def _pick_cup_from_holder(self):
        cmd = (
            f"{self._ros_env()} && "
            "python3 tools/run/pick_from_cup_holder_side_grip.py "
            "--service-prefix dsr01 "
            "--place-final-z-offset-m -0.020 "
            "--execute "
            "--confirm ENABLE_CUP_HOLDER_PICK"
        )
        self._run(cmd, timeout_sec=180)

    def _shake_closed_cup(self):
        cmd = (
            f"{self._ros_env()} && "
            "ros2 launch azas_bringup tumbler_shake_sequence.launch.py "
            "enable_hardware:=true "
            "hardware_confirm:=ENABLE_REAL_ROBOT_MOTION "
            "allow_service_control_without_moveit:=true "
            "service_prefix:=dsr01 "
            "use_visualizer:=false "
            "shake_control_mode:=joint "
            "shake_cycles:=3 "
            "joint_shake_base_j1_deg:=0.0 "
            "joint_shake_base_j2_deg:=-35.0 "
            "joint_shake_base_j3_deg:=50.0 "
            "joint_shake_base_j4_deg:=0.0 "
            "joint_shake_base_j5_deg:=70.0 "
            "joint_shake_base_j6_deg:=0.0 "
            "joint_shake_j4_amplitude_deg:=18.0 "
            "joint_shake_j5_amplitude_deg:=20.0 "
            "joint_shake_j6_amplitude_deg:=24.0"
        )
        self._run(cmd, timeout_sec=240)


def main(args=None):
    rclpy.init(args=args)
    node = FullCocktailSequenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()