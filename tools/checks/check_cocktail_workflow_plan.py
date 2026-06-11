#!/usr/bin/env python3
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "azas_task_manager"))

from azas_task_manager.cocktail_workflow_plan import build_cocktail_steps  # noqa: E402


def main() -> int:
    steps = build_cocktail_steps(["1", "2"])
    phases = [step.phase for step in steps]
    required_order = [
        "VERIFY_RECIPE",
        "VERIFY_CUP_AND_LID_DETECTION",
        "VERIFY_CALIBRATION",
        "TRANSFORM_CUP_TO_BASE",
        "PICK_CUP",
        "ALIGN_CUP_UNDER_DISPENSER",
        "PRESS_DISPENSER",
        "ALIGN_CUP_UNDER_DISPENSER",
        "PRESS_DISPENSER",
        "PICK_LID",
        "PLACE_AND_PRESS_LID",
        "SHAKE_CUP",
        "OPEN_LID",
        "POUR",
        "VERIFY_HUMAN_HAND_TRACKING",
        "COMPUTE_HANDOVER_POSE",
        "WAIT_FOR_HANDOVER_APPROVAL",
        "HANDOVER_CUP_TO_HUMAN_DISABLED",
    ]
    if phases != required_order:
        print("[FAIL] unexpected workflow phase order")
        print(json.dumps(phases, ensure_ascii=False, indent=2))
        return 1

    strict_phases = {
        "PICK_CUP",
        "ALIGN_CUP_UNDER_DISPENSER",
        "PRESS_DISPENSER",
        "PICK_LID",
        "PLACE_AND_PRESS_LID",
        "SHAKE_CUP",
        "OPEN_LID",
        "POUR",
    }
    unsafe = [
        step.phase
        for step in steps
        if step.phase in strict_phases and step.hardware_gate != "strict_live_gate"
    ]
    if unsafe:
        print("[FAIL] hardware phases missing strict_live_gate: " + ", ".join(unsafe))
        return 1

    shake_steps = [step for step in steps if step.phase == "SHAKE_CUP"]
    if len(shake_steps) != 1 or shake_steps[0].command != "tumbler_shake_sequence.launch.py":
        print("[FAIL] SHAKE_CUP must route to tumbler_shake_sequence.launch.py")
        return 1
    shake_params = shake_steps[0].parameters
    if shake_params.get("min_shake_z_m", 0.0) < 0.25:
        print("[FAIL] SHAKE_CUP min_shake_z_m is too low")
        return 1
    if shake_params.get("dispenser_keepout_radius_m", 0.0) < 0.20:
        print("[FAIL] SHAKE_CUP dispenser_keepout_radius_m is too small")
        return 1

    handover = {step.phase: step for step in steps if step.phase.startswith(("VERIFY_HUMAN", "COMPUTE_HANDOVER", "WAIT_FOR_HANDOVER", "HANDOVER_CUP"))}
    expected_handover = {
        "VERIFY_HUMAN_HAND_TRACKING",
        "COMPUTE_HANDOVER_POSE",
        "WAIT_FOR_HANDOVER_APPROVAL",
        "HANDOVER_CUP_TO_HUMAN_DISABLED",
    }
    if set(handover) != expected_handover:
        print("[FAIL] post-shake human handover dry-run steps are missing")
        print(json.dumps(sorted(handover), ensure_ascii=False, indent=2))
        return 1
    if handover["VERIFY_HUMAN_HAND_TRACKING"].hardware_gate != "no_motion_hri_perception_only":
        print("[FAIL] hand tracking must remain perception-only")
        return 1
    if handover["HANDOVER_CUP_TO_HUMAN_DISABLED"].hardware_gate != "disabled_until_hri_safety_review":
        print("[FAIL] live handover must remain disabled until HRI safety review")
        return 1
    if handover["HANDOVER_CUP_TO_HUMAN_DISABLED"].command != "disabled_handover_motion_placeholder":
        print("[FAIL] handover placeholder must not route to a live motion command")
        return 1

    print("[PASS] full cocktail workflow plan includes calibration, dispenser press, shake gates, and disabled post-shake handover planning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
