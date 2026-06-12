#!/usr/bin/env python3
"""Verify that the M0609 MoveIt description includes the RG2 collision mesh.

This is a no-hardware check. It expands the MoveIt URDF xacro and verifies that
the robot model contains the vendored OnRobot RG2FT collision links instead of
planning with the bare robot flange only.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MOVEIT_XACRO = (
    ROOT
    / "third_party"
    / "ros2_src"
    / "doosan-robot2"
    / "dsr_moveit2"
    / "dsr_moveit_config_m0609"
    / "config"
    / "m0609.urdf.xacro"
)
MOVEIT_SRDF = MOVEIT_XACRO.with_name("dsr.srdf")

REQUIRED_LINKS = {
    "rg2_quick_changer",
    "rg2_gripper_body",
    "rg2_angle_bracket",
    "rg2_left_inner_finger",
    "rg2_right_inner_finger",
    "gripper_tcp",
}

REQUIRED_DISABLED_COLLISIONS = {
    frozenset(("link_6", "rg2_quick_changer")),
    frozenset(("rg2_quick_changer", "rg2_angle_bracket")),
    frozenset(("rg2_angle_bracket", "rg2_gripper_body")),
    frozenset(("rg2_gripper_body", "rg2_left_inner_knuckle")),
    frozenset(("rg2_gripper_body", "rg2_right_inner_knuckle")),
    frozenset(("rg2_left_inner_knuckle", "rg2_left_inner_finger")),
    frozenset(("rg2_right_inner_knuckle", "rg2_right_inner_finger")),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check that M0609 MoveIt robot_description contains RG2 mesh collisions."
    )
    parser.add_argument("--xacro", type=Path, default=MOVEIT_XACRO)
    parser.add_argument("--srdf", type=Path, default=MOVEIT_SRDF)
    return parser.parse_args()


def run_xacro(path: Path) -> str:
    env = os.environ.copy()
    env.setdefault("AMENT_PREFIX_PATH", str(ROOT / "install"))
    try:
        completed = subprocess.run(
            ["xacro", str(path)],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
        )
    except FileNotFoundError:
        print("[FAIL] xacro executable not found", file=sys.stderr)
        raise SystemExit(1)
    except subprocess.CalledProcessError as exc:
        print(exc.stderr, file=sys.stderr)
        print(f"[FAIL] failed to expand xacro: {path}", file=sys.stderr)
        raise SystemExit(exc.returncode)
    return completed.stdout


def main() -> int:
    args = parse_args()
    xacro_path = args.xacro.expanduser().resolve()
    if not xacro_path.is_file():
        print(f"[FAIL] missing MoveIt xacro: {xacro_path}")
        return 1

    source = xacro_path.read_text(encoding="utf-8")
    required_source = [
        "rg2_parametric.xacro",
        "xacro:azas_rg2_parametric",
        'name="rg2_parent_link" default="link_6"',
        'name="rg2_mount_rpy" default="0 0 0"',
    ]
    missing_source = [needle for needle in required_source if needle not in source]
    if missing_source:
        print("[FAIL] MoveIt xacro does not wire the RG2 description:")
        for needle in missing_source:
            print(f"missing={needle}")
        return 1

    root = ET.fromstring(run_xacro(xacro_path))
    links = {link.attrib["name"] for link in root.findall("link")}
    missing_links = sorted(REQUIRED_LINKS - links)
    if missing_links:
        print(f"[FAIL] expanded robot_description missing RG2 links: {missing_links}")
        return 1

    collision_meshes = [
        mesh.attrib.get("filename", "")
        for mesh in root.findall(".//collision/geometry/mesh")
    ]
    rg2_collision_meshes = [
        filename
        for filename in collision_meshes
        if filename.startswith("package://azas_description/meshes/onrobot_rg2ft/collision/")
    ]
    if len(rg2_collision_meshes) < 8:
        print(f"[FAIL] expected RG2 collision meshes, found {len(rg2_collision_meshes)}")
        return 1

    quick_changer_joint = root.find("./joint[@name='rg2_quick_changer_joint']")
    parent = quick_changer_joint.find("parent").attrib.get("link") if quick_changer_joint is not None else None
    if parent != "link_6":
        print(f"[FAIL] rg2_quick_changer_joint parent should be link_6, found {parent!r}")
        return 1
    origin = quick_changer_joint.find("origin") if quick_changer_joint is not None else None
    mount_rpy = origin.attrib.get("rpy") if origin is not None else None
    if mount_rpy != "0 0 0":
        print(f"[FAIL] rg2_quick_changer_joint mount rpy should be identity, found {mount_rpy!r}")
        return 1

    srdf_path = args.srdf.expanduser().resolve()
    if not srdf_path.is_file():
        print(f"[FAIL] missing MoveIt SRDF: {srdf_path}")
        return 1
    srdf = ET.parse(srdf_path).getroot()
    disabled_collisions = {
        frozenset((entry.attrib.get("link1", ""), entry.attrib.get("link2", "")))
        for entry in srdf.findall("disable_collisions")
    }
    missing_disabled = REQUIRED_DISABLED_COLLISIONS - disabled_collisions
    if missing_disabled:
        print("[FAIL] MoveIt SRDF does not allow required RG2 internal self-collisions:")
        for pair in sorted(tuple(sorted(pair)) for pair in missing_disabled):
            print(f"missing={pair[0]} <-> {pair[1]}")
        return 1

    print("[PASS] M0609 MoveIt robot_description includes RG2 mesh collision links.")
    print(f"rg2_collision_meshes={len(rg2_collision_meshes)}")
    print("rg2_parent=link_6")
    print("rg2_mount_rpy=0 0 0")
    print(f"rg2_required_disabled_collisions={len(REQUIRED_DISABLED_COLLISIONS)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
