#!/usr/bin/env python3
"""Verify RViz scene and floor-place launch share the same fixed geometry."""

from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path("/home/ssu/ros2_ws/src/Azas")
SCENE_LAUNCH = ROOT / "launch" / "tumbler_dispenser_scene.launch.py"
FLOOR_LAUNCH = ROOT / "launch" / "tumbler_floor_place.launch.py"
DISPENSE_LID_LAUNCH = ROOT / "launch" / "dispense_lid_sequence.launch.py"
SHAKE_LAUNCH = ROOT / "launch" / "tumbler_shake_sequence.launch.py"
FLOOR_NODE = ROOT / "jarvis" / "tumbler_floor_place_node.py"
SCENE_NODE = ROOT / "jarvis" / "tumbler_dispenser_scene_node.py"
SHAKE_NODE = ROOT / "jarvis" / "tumbler_shake_sequence_node.py"
DISPENSE_LID_NODE = ROOT / "jarvis" / "dispense_lid_sequence_node.py"

EXPECTED_BOTTLES = [
    0.55,
    0.18,
    0.1375,
    0.55,
    0.08,
    0.1375,
    0.55,
    -0.02,
    0.1375,
    0.55,
    -0.12,
    0.1375,
]
EXPECTED_OUTLETS = [
    0.43,
    0.18,
    0.392,
    0.43,
    0.08,
    0.392,
    0.43,
    -0.02,
    0.392,
    0.43,
    -0.12,
    0.392,
]


def literal_dict(path: Path, name: str) -> dict:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == name and isinstance(node.value, ast.Dict):
                    result = {}
                    for key, value in zip(node.value.keys, node.value.values):
                        if isinstance(key, ast.Constant) and isinstance(key.value, str):
                            try:
                                result[key.value] = ast.literal_eval(value)
                            except (ValueError, SyntaxError):
                                pass
                    return result
    raise RuntimeError(f"{name} dict not found in {path}")


def declared_parameter_literal(path: Path, parameter_name: str):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Attribute) or node.func.attr != "declare_parameter":
            continue
        if len(node.args) < 2:
            continue
        if not isinstance(node.args[0], ast.Constant) or node.args[0].value != parameter_name:
            continue
        return ast.literal_eval(node.args[1])
    raise RuntimeError(f"declare_parameter {parameter_name} not found in {path}")


def launch_default(path: Path, argument_name: str) -> str:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not isinstance(func, ast.Name) or func.id != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        if node.args[0].value != argument_name:
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(keyword.value, ast.Constant):
                return str(keyword.value.value)
    raise RuntimeError(f"DeclareLaunchArgument {argument_name} not found in {path}")


def check_equal(label: str, actual, expected) -> int:
    if actual == expected:
        print(f"[OK] {label}")
        return 0
    print(f"[FAIL] {label}")
    print(f"  actual  ={actual}")
    print(f"  expected={expected}")
    return 1


def main() -> int:
    failures = 0
    scene = literal_dict(SCENE_LAUNCH, "scene_params")
    floor = literal_dict(FLOOR_LAUNCH, "params")
    dispense_lid = literal_dict(DISPENSE_LID_LAUNCH, "params")
    shake = literal_dict(SHAKE_LAUNCH, "params")
    floor_node_bottles = declared_parameter_literal(FLOOR_NODE, "dispenser_bottle_positions")
    scene_node_bottles = declared_parameter_literal(SCENE_NODE, "dispenser_bottle_positions")
    shake_node_bottles = declared_parameter_literal(SHAKE_NODE, "dispenser_bottle_positions")
    floor_node_outlets = declared_parameter_literal(FLOOR_NODE, "dispenser_outlet_positions")
    scene_node_outlets = declared_parameter_literal(SCENE_NODE, "dispenser_outlet_positions")
    dispense_lid_node_outlets = declared_parameter_literal(DISPENSE_LID_NODE, "dispenser_outlet_positions")

    print("[Azas] Fixed dispenser geometry check")
    failures += check_equal("scene bottle positions", scene["dispenser_bottle_positions"], EXPECTED_BOTTLES)
    failures += check_equal("floor bottle positions", floor["dispenser_bottle_positions"], EXPECTED_BOTTLES)
    failures += check_equal("shake bottle positions", shake["dispenser_bottle_positions"], EXPECTED_BOTTLES)
    failures += check_equal("floor node default bottle positions", floor_node_bottles, EXPECTED_BOTTLES)
    failures += check_equal("scene node default bottle positions", scene_node_bottles, EXPECTED_BOTTLES)
    failures += check_equal("shake node default bottle positions", shake_node_bottles, EXPECTED_BOTTLES)
    failures += check_equal("scene outlet positions", scene["dispenser_outlet_positions"], EXPECTED_OUTLETS)
    failures += check_equal("floor outlet positions", floor["dispenser_outlet_positions"], EXPECTED_OUTLETS)
    failures += check_equal("floor node default outlet positions", floor_node_outlets, EXPECTED_OUTLETS)
    failures += check_equal("scene node default outlet positions", scene_node_outlets, EXPECTED_OUTLETS)
    failures += check_equal("dispense/lid node default outlet positions", dispense_lid_node_outlets, EXPECTED_OUTLETS)
    failures += check_equal(
        "dispense/lid outlet positions",
        dispense_lid["dispenser_outlet_positions"],
        EXPECTED_OUTLETS,
    )
    failures += check_equal("dispense/lid cup place x", dispense_lid["cup_place_x"], 0.43)
    # The scene node keeps a shorthand outlet marker for the visual midpoint
    # between the row entries; the floor-place node uses the indexed list above.
    failures += check_equal("scene selected outlet shorthand", scene["dispenser_outlet_position"], [0.43, 0.05, 0.392])
    failures += check_equal("scene side grasp height", scene["grasp_height"], 0.085)
    failures += check_equal("scene side grasp approach offset", scene["side_grasp_approach_offset"], 0.10)
    failures += check_equal("scene lift height", scene["lift_height"], 0.04)
    failures += check_equal("scene pre-outlet height", scene["pre_outlet_height"], 0.06)
    failures += check_equal("floor side grasp launch default", launch_default(FLOOR_LAUNCH, "grasp_height"), "0.085")
    failures += check_equal(
        "floor side grasp approach offset launch default",
        launch_default(FLOOR_LAUNCH, "side_grasp_approach_offset"),
        "0.10",
    )
    failures += check_equal(
        "floor side grasp candidate count launch default",
        launch_default(FLOOR_LAUNCH, "side_grasp_candidate_count"),
        "16",
    )
    failures += check_equal(
        "floor detected grasp yaw launch default",
        launch_default(FLOOR_LAUNCH, "use_detected_grasp_yaw"),
        "true",
    )
    failures += check_equal(
        "floor execution stage launch default",
        launch_default(FLOOR_LAUNCH, "execution_stage"),
        "full",
    )
    failures += check_equal("floor lift launch default", launch_default(FLOOR_LAUNCH, "lift_height"), "0.04")
    failures += check_equal(
        "floor place approach launch default",
        launch_default(FLOOR_LAUNCH, "place_approach_height"),
        "0.06",
    )
    failures += check_equal(
        "floor tumbler bottom diameter launch default",
        launch_default(FLOOR_LAUNCH, "tumbler_bottom_diameter"),
        "0.065",
    )
    failures += check_equal(
        "floor tumbler top diameter launch default",
        launch_default(FLOOR_LAUNCH, "tumbler_top_diameter"),
        "0.075",
    )
    failures += check_equal(
        "floor gripper set service launch default",
        launch_default(FLOOR_LAUNCH, "gripper_set_service"),
        "/jarvis/rg2/set_width",
    )
    failures += check_equal(
        "floor gripper preopen clearance launch default",
        launch_default(FLOOR_LAUNCH, "gripper_preopen_clearance"),
        "0.025",
    )
    failures += check_equal(
        "floor gripper grasp compression launch default",
        launch_default(FLOOR_LAUNCH, "gripper_grasp_compression"),
        "0.006",
    )
    failures += check_equal(
        "floor gripper grasp force launch default",
        launch_default(FLOOR_LAUNCH, "gripper_grasp_force_n"),
        "12.0",
    )
    failures += check_equal(
        "floor gripper preopen force launch default",
        launch_default(FLOOR_LAUNCH, "gripper_preopen_force_n"),
        "8.0",
    )

    if failures:
        return 1
    print("[PASS] RViz scene and floor-place fixed geometry are aligned.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
