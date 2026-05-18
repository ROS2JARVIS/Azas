#!/usr/bin/env python3
"""Supervised observe-pose entrypoint.

Default behavior is planning-only guidance. Real motion is intentionally not
implemented in this batch because the MoveIt execution action/service contract
has not been accepted for Azas supervised operation.
"""

import argparse
import sys


CONFIRM_PHRASE = "I_UNDERSTAND_THIS_WILL_MOVE_THE_ROBOT"


def main() -> int:
    parser = argparse.ArgumentParser(description="Azas supervised observe pose gate")
    parser.add_argument("--planning-group", default="manipulator")
    parser.add_argument("--ee-link", default="tool0")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--observe-frame", default="base_link")
    parser.add_argument("--observe-pose-x", type=float, default=0.35)
    parser.add_argument("--observe-pose-y", type=float, default=-0.25)
    parser.add_argument("--observe-pose-z", type=float, default=0.45)
    parser.add_argument("--observe-qx", type=float, default=0.0)
    parser.add_argument("--observe-qy", type=float, default=0.0)
    parser.add_argument("--observe-qz", type=float, default=0.0)
    parser.add_argument("--observe-qw", type=float, default=1.0)
    parser.add_argument("--enable-real-motion", action="store_true")
    parser.add_argument("--one-shot", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    print("Azas OBSERVE_CUP_POSE supervised gate")
    print(
        "observe pose: "
        f"frame={args.observe_frame} "
        f"x={args.observe_pose_x:.3f} y={args.observe_pose_y:.3f} z={args.observe_pose_z:.3f} "
        f"qx={args.observe_qx:.6f} qy={args.observe_qy:.6f} "
        f"qz={args.observe_qz:.6f} qw={args.observe_qw:.6f}"
    )
    print(f"planning_group={args.planning_group} ee_link={args.ee_link}")

    if not args.enable_real_motion:
        print("mode=planning_only")
        print("No real robot motion was commanded.")
        print("Run tools/checks/check_observe_pose_planning_only.sh to validate MoveIt plan().")
        return 0

    missing = []
    if args.confirm != CONFIRM_PHRASE:
        missing.append(f"--confirm {CONFIRM_PHRASE}")
    if not args.one_shot:
        missing.append("--one-shot")
    if missing:
        print("REFUSED: real observe motion requires all explicit gates:")
        for item in missing:
            print(f"  missing {item}")
        print("No real robot motion was commanded.")
        return 2

    print("REFUSED: real observe motion is not implemented in this batch.")
    print("Required before implementation: accepted MoveIt execution action/service contract,")
    print("operator clearance, e-stop check, workspace bounds, and measured calibration.")
    print("No real robot motion was commanded.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
