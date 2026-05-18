from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class TaskStep:
    phase: str
    detail: str
    required_inputs: tuple[str, ...] = ()
    produces: tuple[str, ...] = ()
    command: str = "none"
    hardware_gate: str = "no_motion"
    parameters: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "phase": self.phase,
            "detail": self.detail,
            "required_inputs": list(self.required_inputs),
            "produces": list(self.produces),
            "command": self.command,
            "hardware_gate": self.hardware_gate,
        }
        if self.parameters:
            payload["parameters"] = dict(self.parameters)
        return payload


def detection_class(status: str) -> str | None:
    prefix = "detected:"
    if not status.startswith(prefix):
        return None
    tail = status[len(prefix) :]
    return tail.split(maxsplit=1)[0].split(":", maxsplit=1)[0].strip().lower() or None


def build_cocktail_steps(dispenser_ids: list[str]) -> list[TaskStep]:
    steps = [
        TaskStep(
            "VERIFY_RECIPE",
            "accept symbolic recipe and ordered dispenser IDs",
            required_inputs=("/azas/voice/recipe_decision",),
            produces=("ordered_dispenser_ids",),
        ),
        TaskStep(
            "VERIFY_CUP_AND_LID_DETECTION",
            "require fresh cup and lid detections before any manipulation plan",
            required_inputs=("/azas/cup_detection:cup", "/azas/cup_detection:lid"),
            produces=("cup_detection", "lid_detection"),
        ),
        TaskStep(
            "VERIFY_CALIBRATION",
            "require measured camera frame, base frame, TCP, cup offset, dispenser ID pose mapping, and safety bounds",
            required_inputs=(
                "calibration.yaml",
                "safety.yaml",
                "base_link<-camera_frame TF",
                "tcp_to_cup_mouth_m",
                "dispenser_outlets.1..4 outlet_pose and press_pose",
            ),
            produces=("calibration_ready",),
            command="tools/checks/check_real_motion_config.sh",
            hardware_gate="measured_config_required",
        ),
        TaskStep(
            "TRANSFORM_CUP_TO_BASE",
            "convert detected cup pose from camera frame into base_link",
            required_inputs=("/azas/cup_detection", "base_link<-camera_frame TF"),
            produces=("/jarvis/tumbler_dispenser/tumbler_pose",),
            command="cup_detection_pose_bridge_node",
            hardware_gate="tf_required",
        ),
        TaskStep(
            "PICK_CUP",
            "approach, close RG2 on detected cup, and lift just enough for transfer",
            required_inputs=("/jarvis/tumbler_dispenser/tumbler_pose", "/jarvis/rg2/close"),
            produces=("cup_held",),
            command="MoveLine approach + /jarvis/rg2/close + slight lift",
            hardware_gate="strict_live_gate",
        ),
    ]

    for dispenser_id in dispenser_ids:
        steps.extend(
            [
                TaskStep(
                    "ALIGN_CUP_UNDER_DISPENSER",
                    f"move cup mouth below dispenser {dispenser_id} outlet",
                    required_inputs=("cup_held", f"dispenser:{dispenser_id}:outlet_pose"),
                    produces=(f"cup_aligned_under:{dispenser_id}",),
                    command="MoveLine",
                    hardware_gate="strict_live_gate",
                    parameters={"dispenser_id": dispenser_id},
                ),
                TaskStep(
                    "PRESS_DISPENSER",
                    f"press/squeeze dispenser {dispenser_id} so contents fall into the held cup",
                    required_inputs=(f"cup_aligned_under:{dispenser_id}", f"dispenser:{dispenser_id}:press_pose"),
                    produces=(f"dispensed:{dispenser_id}",),
                    command="dispenser_press_node",
                    hardware_gate="strict_live_gate",
                    parameters={"dispenser_id": dispenser_id},
                ),
            ]
        )

    steps.extend(
        [
            TaskStep(
                "PICK_LID",
                "pick detected lid after drink dispensing",
                required_inputs=("/azas/cup_detection:lid", "/jarvis/rg2/close"),
                produces=("lid_held",),
                command="MoveLine + /jarvis/rg2/close",
                hardware_gate="strict_live_gate",
            ),
            TaskStep(
                "PLACE_AND_PRESS_LID",
                "place lid onto cup and press within configured force/speed limits",
                required_inputs=("lid_held", "cup_pose_after_dispense"),
                produces=("cup_closed",),
                command="MoveLine",
                hardware_gate="strict_live_gate",
            ),
            TaskStep(
                "SHAKE_CUP",
                "shake closed cup with bounded amplitude, cycles, velocity, and acceleration",
                required_inputs=("cup_closed", "shake_safety_limits", "dispenser_keepout_zones"),
                produces=("cocktail_mixed",),
                command="tumbler_shake_sequence.launch.py",
                hardware_gate="strict_live_gate",
                parameters={
                    "default_cycles": 3,
                    "default_amplitude_x_m": 0.035,
                    "default_amplitude_y_m": 0.020,
                    "default_center_xyz_m": (0.30, -0.28, 0.32),
                    "min_shake_z_m": 0.25,
                    "dispenser_keepout_radius_m": 0.20,
                    "requires_lid_closed": True,
                },
            ),
            TaskStep(
                "OPEN_LID",
                "remove or open lid after shaking",
                required_inputs=("cocktail_mixed",),
                produces=("cup_opened",),
                command="MoveLine + /jarvis/rg2/open",
                hardware_gate="strict_live_gate",
            ),
            TaskStep(
                "POUR",
                "pour cocktail into target cup after target pose is detected and checked",
                required_inputs=("cup_opened", "target_cup_pose"),
                produces=("cocktail_served",),
                command="MoveLine pour primitive",
                hardware_gate="strict_live_gate",
            ),
        ]
    )
    return steps
