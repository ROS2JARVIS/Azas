import json
import time

import rclpy
from azas_interfaces.msg import CupDetection
from azas_task_manager.cocktail_workflow_plan import build_cocktail_steps, detection_class
from rclpy.node import Node
from std_msgs.msg import String


class CocktailDryRunSequenceNode(Node):
    """Recipe-to-task sequence boundary for the full cocktail workflow.

    This node applies the DSR demo's stepwise action-server pattern to Azas,
    but it deliberately publishes dry-run status only. Robot poses, MoveIt
    execution, gripper commands, and dispenser actuation remain separate gates.
    """

    def __init__(self):
        super().__init__("cocktail_dryrun_sequence_node")
        self.declare_parameter("decision_topic", "/azas/voice/recipe_decision")
        self.declare_parameter("detection_topic", "/azas/cup_detection")
        self.declare_parameter("plan_topic", "/azas/cocktail/task_plan")
        self.declare_parameter("status_topic", "/azas/cocktail/status")
        self.declare_parameter("max_detection_age_s", 5.0)
        self.declare_parameter("require_cup", True)
        self.declare_parameter("require_lid", True)

        self._detections: dict[str, tuple[CupDetection, float]] = {}
        self._plan_pub = self.create_publisher(
            String, self.get_parameter("plan_topic").value, 10
        )
        self._status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10
        )
        self._decision_sub = self.create_subscription(
            String,
            self.get_parameter("decision_topic").value,
            self._on_decision,
            10,
        )
        self._detection_sub = self.create_subscription(
            CupDetection,
            self.get_parameter("detection_topic").value,
            self._on_detection,
            10,
        )
        self.get_logger().info(
            "Cocktail dry-run sequence ready: "
            f"{self.get_parameter('decision_topic').value} -> "
            f"{self.get_parameter('status_topic').value}"
        )

    def _on_detection(self, msg: CupDetection) -> None:
        cls = detection_class(msg.status)
        if cls in {"cup", "lid"}:
            self._detections[cls] = (msg, time.monotonic())

    def _fresh_detection_classes(self) -> set[str]:
        max_age = float(self.get_parameter("max_detection_age_s").value)
        now = time.monotonic()
        return {cls for cls, (_, stamp) in self._detections.items() if now - stamp <= max_age}

    def _publish_status(self, status: str, **fields: object) -> None:
        payload = {"status": status, **fields}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)
        if status == "blocked":
            self.get_logger().warn(msg.data)
        else:
            self.get_logger().info(msg.data)

    def _on_decision(self, msg: String) -> None:
        try:
            decision = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._publish_status("blocked", reason="invalid_recipe_decision_json", error=str(exc))
            return

        if decision.get("intent") == "cancel":
            self._publish_status("cancelled", reason="voice_cancel")
            return

        if not decision.get("valid") or decision.get("intent") != "make_cocktail":
            self._publish_status("blocked", reason="recipe_decision_not_executable", decision=decision)
            return

        dispenser_ids = [str(item) for item in decision.get("dispenser_ids", []) if str(item)]
        if not dispenser_ids:
            self._publish_status(
                "blocked",
                reason="recipe_has_no_dispenser_ids",
                recipe_id=decision.get("recipe_id"),
                detail="Populate recipe dispenser IDs or request explicit colors.",
            )
            return

        fresh = self._fresh_detection_classes()
        missing = []
        if bool(self.get_parameter("require_cup").value) and "cup" not in fresh:
            missing.append("cup")
        if bool(self.get_parameter("require_lid").value) and "lid" not in fresh:
            missing.append("lid")
        if missing:
            self._publish_status(
                "blocked",
                reason="missing_fresh_detection",
                missing=missing,
                fresh=sorted(fresh),
            )
            return

        steps = build_cocktail_steps(dispenser_ids)
        plan_msg = String()
        plan_msg.data = json.dumps(
            {
                "mode": "dry_run_no_motion",
                "recipe_id": decision.get("recipe_id"),
                "dispenser_ids": dispenser_ids,
                "stop_condition": (
                    "no robot/RG2/dispenser command is sent by this node; "
                    "hardware execution requires strict live gate and measured calibration"
                ),
                "steps": [step.to_dict() for step in steps],
            },
            ensure_ascii=False,
        )
        self._plan_pub.publish(plan_msg)

        total = len(steps)
        for index, step in enumerate(steps, start=1):
            self._publish_status(
                "dry_run_step",
                index=index,
                total=total,
                phase=step.phase,
                detail=step.detail,
                command=step.command,
                hardware_gate=step.hardware_gate,
            )
        self._publish_status("dry_run_complete", total=total, mode="no_robot_motion")


def main(args=None):
    rclpy.init(args=args)
    node = CocktailDryRunSequenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
