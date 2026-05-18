# Azas OSS Robot Control Stack

This document turns the open-source shortlist into a robot-control integration path for the cocktail project.

## Success Criteria

Azas is ready for open-source-assisted robot control when all of these are true:

1. ROS 2 Humble, Azas packages, and the `jarvis` bridge packages build locally.
2. Doosan `dsr_bringup2`, M0609 MoveIt config, and `moveit_py` are available.
3. Doosan virtual MoveIt launch and `robot_connection_control.launch.py --show-args` resolve without missing package errors.
4. RealSense topics are available or explicitly disabled for a non-camera dry-run.
5. YOLO publishes `/azas/cup_detection` from camera input or a tested replacement publishes the same contract.
6. `cup_detection_pose_bridge_node` forwards only confident detections to `/jarvis/tumbler_dispenser/tumbler_pose`.
7. Floor-place control remains `enable_hardware:=false` until e-stop, workspace, collision, RG2, and operator gates are verified.
8. Real robot motion uses `tools/run/run_robot_real.sh`, not an ad-hoc launch command, and requires a recent strict live-gate stamp.

## Chosen OSS Stack

| Layer | OSS | Azas role | Gate |
| --- | --- | --- | --- |
| Middleware | ROS 2 Humble | node/topic/service/action base | Required |
| Arm driver | Doosan `doosan-robot2` | M0609 hardware/simulation driver | Required before real arm control |
| Planning | MoveIt 2 / MoveItPy | reach, pick, alignment, collision checks | Required for generalized motion |
| Camera | RealSense ROS 2 wrapper | RGB-D and CameraInfo | Required for live vision |
| Fast detection | Ultralytics YOLO | MVP cup/tumbler detector | Use with license review |
| Fallback segmentation | LangSAM/GroundingDINO/Grounded-SAM2 | prompt-based mask when YOLO fails | Optional, slower path |
| Calibration | AprilTag ROS 2, easy_handeye2 | camera/base transform validation | Required before trusting 3D poses |
| Voice | SpeechRecognition now; Vosk/Whisper candidates | recipe intent only | Never outputs robot coordinates |

## Non-Hardware Readiness Check

Run after building Azas and `ros2_ws`:

```bash
/home/ssu/Azas/tools/checks/check_oss_stack.sh
```

Full non-hardware verifier:

```bash
/home/ssu/Azas/tools/checks/verify_control_readiness.sh
```

The verifier writes `/tmp/azas_control_readiness_report.txt` by default.

For CI-like strictness, make optional model/STT imports fail the check:

```bash
STRICT_OPTIONAL=true /home/ssu/Azas/tools/checks/check_oss_stack.sh
```

Run the end-to-end control-path smoke without camera, YOLO, RG2, or real robot motion:

```bash
/home/ssu/Azas/tools/smoke/smoke_control_path.sh
```

Expected result:

```text
[OK] smoke control path reached DONE
```

This proves the local message contract and launch wiring from fake `CupDetection` through the pose bridge into `tumbler_floor_place_node`. It does not prove camera calibration, real depth projection, MoveIt feasibility, RG2 actuation, or Doosan hardware motion.

Run the hardware-armed path against fake services:

```bash
/home/ssu/Azas/tools/smoke/smoke_fake_hardware_path.sh
```

Expected result:

```text
[OK] fake hardware path reached DONE
```

This proves the hardware-gated path can call Doosan `MoveLine` and RG2 Trigger service types without deadlocking, while still sending no real hardware commands.

After starting `tools/run/run_robot_dryrun.sh` or an equivalent live bringup in another terminal, run the field gate checker:

```bash
/home/ssu/Azas/tools/checks/check_live_hardware_gates.sh
```

This checks camera topics, `CameraInfo` sampling, detection/pose topics, Doosan motion services, and RG2 trigger service availability without sending motion or gripper commands.

For the real-motion gate, use strict mode so warnings become failures and the approved stamp is written:

```bash
STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed /home/ssu/Azas/tools/checks/check_live_hardware_gates.sh
```

`tools/run/run_robot_real.sh` refuses to launch unless that strict stamp exists, was produced by `STRICT=true`, and is recent.

For the exact terminal order, use `docs/field_control_runbook.md`.

For the decision of when to simulate, when to connect the camera, and when to connect the real robot/RG2, use `docs/simulation_and_connection_plan.md`.

To isolate the RGB-D camera gate:

```bash
/home/ssu/Azas/tools/checks/check_depth_projection_sample.sh
```

Expected result with a live aligned depth camera:

```text
[PASS] depth projection sample frame='...' pixel=(u,v) depth_raw=... point_camera_m=(x,y,z)
```

## Build Path

```bash
source /opt/ros/humble/setup.bash
cd /home/ssu/Azas
colcon build --symlink-install
source install/setup.bash

cd /home/ssu/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Install optional Python runtime dependencies only into the robot PC environment after license review:

```bash
python3 -m pip install -r /home/ssu/Azas/dependencies/python_optional_requirements.txt
```

## Safe Control Path

Default dry-run:

```bash
/home/ssu/Azas/tools/run/run_robot_dryrun.sh
```

Detection check:

```bash
/home/ssu/Azas/tools/checks/check_robot_detection.sh
```

Full live gate check:

```bash
/home/ssu/Azas/tools/checks/check_live_hardware_gates.sh
```

Strict live gate for real motion:

```bash
STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed /home/ssu/Azas/tools/checks/check_live_hardware_gates.sh
```

Real robot entrypoint, only after the safety checklist is complete:

```bash
/home/ssu/Azas/tools/run/run_robot_real.sh
```

## Hard Boundaries

- LLM, VLA, and STT may select a recipe or dispenser ID only.
- Robot coordinates must come from calibrated vision, fixed dispenser config, and TF.
- External source trees stay in review workspaces; do not vendor them into Azas.
- Grounded-SAM2/LangSAM is a fallback or labeling aid until runtime latency is measured.
