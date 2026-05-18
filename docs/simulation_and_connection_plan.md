# Simulation And Connection Plan

Purpose: decide when to use simulation, when to connect the camera, and when to connect the real Doosan/RG2 hardware for the Azas cocktail robot.

## Short Answer

Yes, run simulation and fake-hardware checks before connecting the robot for motion.

You do not need the real robot connected for the first validation pass. Connect hardware in this order:

1. No hardware: run OSS readiness and fake control-path smokes.
2. Camera only: verify RealSense topics, YOLO detection, depth projection, and frame IDs.
3. Robot/RG2 powered but no motion: verify Doosan/RG2 ROS services and calibration config gates.
4. Robot motion: only after strict live gate stamp plus operator confirmation.

At any point, run this no-motion stage report to see what should be connected next:

```bash
/home/ssu/Azas/tools/checks/check_connection_stage.sh
```

## Stage 0: Local Non-Hardware Readiness

Run:

```bash
/home/ssu/Azas/tools/checks/verify_control_readiness.sh
```

This should pass before touching hardware. It verifies package availability, launch arguments, fake detection-to-control wiring, and fake hardware service calls. It does not prove live camera detection, calibration, MoveIt feasibility, RG2 behavior, or real robot motion.

## Stage 1: Doosan Virtual / Fake Hardware

Use this when the robot is not connected:

```bash
/home/ssu/Azas/tools/run/run_doosan_virtual_m0609.sh
/home/ssu/Azas/tools/smoke/smoke_fake_hardware_path.sh
```

Goal: prove that the Azas control path can call the expected Doosan `MoveLine` and RG2 Trigger service shapes without deadlock or real hardware commands.

Stop condition: fake-hardware smoke reaches `DONE`.

RViz simulation evidence from this session is recorded in `docs/rviz_simulation_verification_2026-05-11.md`.

## Stage 2: Camera Only

Connect the RealSense before connecting real robot motion. Start the dry-run launch:

```bash
/home/ssu/Azas/tools/run/run_robot_dryrun.sh
```

Then verify:

```bash
/home/ssu/Azas/tools/checks/check_robot_detection.sh
/home/ssu/Azas/tools/checks/check_depth_projection_sample.sh
```

Goal: prove the live camera can produce color, aligned depth, `CameraInfo`, YOLO detection, and a camera-frame 3D point for the actual tumbler/lid.

Stop condition: `/azas/cup_detection` reports a cup status starting with `detected:upright`, depth projection passes with a known-distance target, and `/jarvis/tumbler_dispenser/tumbler_pose` comes from live camera detection in `base_link`.

Current live camera evidence is recorded in `docs/camera_connection_verification_2026-05-11.md`. Camera topics and depth projection pass, but stable live cup/lid detection is still missing.

## Stage 3: Robot/RG2 Connected, No Motion

Connect/power the Doosan and RG2, but do not run real motion yet.

Run the strict gate:

```bash
STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed /home/ssu/Azas/tools/checks/check_live_hardware_gates.sh
```

This sends no motion commands. It checks:

- camera topics and camera info sampling,
- depth projection,
- `/azas/cup_detection`,
- `/jarvis/tumbler_dispenser/tumbler_pose`,
- Doosan `move_line` and `move_joint` services,
- RG2 open/close Trigger services,
- real-motion calibration and safety config placeholders.

The strict gate must fail while `calibration.yaml` or `safety.yaml` still contains `null` or `확인 필요` placeholders. That is intentional.

## Stage 4: Real Motion

Only after Stage 3 passes and the safety checklist is complete:

```bash
/home/ssu/Azas/tools/run/run_robot_real.sh
```

`run_robot_real.sh` refuses to launch unless the strict gate stamp exists, was produced by `STRICT=true`, and is recent. It also requires typing `ENABLE_REAL_ROBOT_MOTION`.

## Current Recommendation

Do not connect the robot for motion yet. Connect the camera first if available, because the current missing evidence is live detection/depth/frames and measured calibration. The robot should be connected after camera evidence exists and before the strict no-motion service/config gate.
