# Field Control Runbook

Purpose: move from open-source stack readiness to controlled Azas dry-run evidence without accidentally commanding real robot motion.

For the higher-level decision of whether to simulate first or connect hardware, see `docs/simulation_and_connection_plan.md`.
For copy-paste field commands, see `docs/field_execution_commands.md`.

## Preconditions

- Azas and `ros2_ws` have been built.
- `tools/checks/check_oss_stack.sh` passes with `failures=0`.
- `tools/smoke/smoke_control_path.sh` reaches `DONE`.
- The operator has read `docs/safety_checklist.md`.
- Real robot motion remains disabled until this runbook explicitly reaches the real-run gate.

## Terminal 1: Doosan Virtual M0609

```bash
/home/ssu/Azas/tools/run/run_doosan_virtual_m0609.sh
```

Default values:

- `MODE=virtual`
- `MODEL=m0609`
- `ROBOT_NAME=` which produces `/motion/move_line` and `/motion/move_joint`

If the Doosan stack is launched with a namespace, keep it consistent:

```bash
ROBOT_NAME=dsr01 /home/ssu/Azas/tools/run/run_doosan_virtual_m0609.sh
```

Then use `SERVICE_PREFIX=dsr01` in the gate checker.

## Terminal 2: Azas Safe Dry-Run

```bash
/home/ssu/Azas/tools/run/run_robot_dryrun.sh
```

This launches camera/YOLO/RG2 bridge/floor-place wiring with `enable_hardware:=false`.

To avoid RG2 Modbus connection attempts while checking only camera and motion services:

```bash
ENABLE_RG2=false /home/ssu/Azas/tools/run/run_robot_dryrun.sh
```

## Terminal 3: Doosan Real No-Motion Bringup

When the physical Doosan controller is connected, start the driver/MoveIt stack
without sending Azas robot-motion or RG2 commands:

```bash
ROBOT_HOST=<robot-ip> DOOSAN_NO_MOTION_CONFIRM=CONNECT_DOOSAN_NO_MOTION \
  /home/ssu/Azas/tools/run/run_doosan_real_no_motion_m0609.sh
```

This stage is for service discovery and controller connection only. It does not
launch `enable_hardware:=true`, does not call `MoveLine`/`MoveJoint`, and does
not call RG2 open/close. If the robot remains red, stop here and fix the robot
controller/teach-pendant/e-stop/network state before any real-motion attempt.

If the Doosan stack uses a namespace, keep it consistent:

```bash
ROBOT_NAME=dsr01 ROBOT_HOST=<robot-ip> \
  DOOSAN_NO_MOTION_CONFIRM=CONNECT_DOOSAN_NO_MOTION \
  /home/ssu/Azas/tools/run/run_doosan_real_no_motion_m0609.sh
```

Then use `SERVICE_PREFIX=dsr01` in the gate checker.

## Terminal 4: Live Gates

Recommended field report, no motion or gripper commands:

```bash
/home/ssu/Azas/tools/run/field_no_motion_report.sh
```

When the tumbler body and lid are each placed clearly in view, run the same report with stability checks:

```bash
RUN_LID_STABILITY=true RUN_CUP_STABILITY=true /home/ssu/Azas/tools/run/field_no_motion_report.sh
```

When camera, Doosan, RG2, and measured config are all expected to be ready, run the strict report:

```bash
STRICT_LIVE_GATE=true RUN_LID_STABILITY=true RUN_CUP_STABILITY=true /home/ssu/Azas/tools/run/field_no_motion_report.sh
```

Single-command acceptance after connecting camera, Doosan, and RG2:

```bash
/home/ssu/Azas/tools/checks/robot_connection_acceptance.sh
```

This wraps the strict field report, hand-eye readiness check, and strict
completion audit. It still sends no Doosan motion command and no RG2
open/close request.

If ROS CLI discovery reports FastDDS or participant errors while the nodes are
visibly publishing, retry the same checks through the ROS daemon:

```bash
ROS2_DAEMON_FLAG="" /home/ssu/Azas/tools/checks/check_connection_stage.sh
ROS2_DAEMON_FLAG="" /home/ssu/Azas/tools/checks/robot_connection_acceptance.sh
```

This changes only ROS graph discovery/echo behavior. It does not enable robot
motion or RG2 actuation.

No motion or gripper commands:

```bash
/home/ssu/Azas/tools/checks/check_live_hardware_gates.sh
```

Hand-eye / camera-to-base readiness, no motion or gripper commands:

```bash
/home/ssu/Azas/tools/checks/check_hand_eye_readiness.sh
```

Use `CAMERA_FRAME=camera_color_optical_frame` if `CameraInfo` is not currently
publishing but a known frame should be checked against TF. A passing result only
proves topic availability and TF evidence between `base_link` and the camera
frame; it does not prove motion safety or measured calibration quality.

Strict gate:

```bash
STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed /home/ssu/Azas/tools/checks/check_live_hardware_gates.sh
```

Namespaced Doosan services:

```bash
SERVICE_PREFIX=dsr01 STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed /home/ssu/Azas/tools/checks/check_live_hardware_gates.sh
```

Expected before real motion:

- color/depth/camera_info topics are present,
- `CameraInfo` sample contains `frame_id` and intrinsics,
- `/azas/cup_detection` reports a cup status starting with `detected:upright`,
- `/jarvis/tumbler_dispenser/tumbler_pose` publishes a fresh `base_link` pose from real detection,
- Doosan `move_line` and `move_joint` services exist with `dsr_msgs2/srv/MoveLine` and `dsr_msgs2/srv/MoveJoint` types,
- RG2 open/close services exist with `std_srvs/srv/Trigger` type if gripper actuation is part of the run.
- `tools/run/field_no_motion_report.sh` writes a field summary to `/tmp/azas_field_no_motion_report.txt`.
- `tools/checks/robot_connection_acceptance.sh` writes a post-connection summary to `/tmp/azas_robot_connection_acceptance_report.txt`.

## Real Motion Gate

Only after the strict live gate passes and the safety checklist is complete:

Connected no-motion field path:

```bash
ROBOT_HOST=<robot-ip> RG2_IP=<rg2-ip> /home/ssu/Azas/tools/run/run_connected_robot_control.sh
```

This starts Doosan real no-motion bringup, starts the Azas safe dry-run, runs
`robot_connection_acceptance.sh`, and stops before real motion by default.
Use it to collect connected field evidence, not as a one-command motion path.

```bash
/home/ssu/Azas/tools/run/run_robot_real.sh
```

`run_robot_real.sh` refuses to launch unless `/tmp/azas_live_hardware_gates_passed` exists, was produced by `STRICT=true`, and is recent. The default max age is 600 seconds; override with `LIVE_GATE_MAX_AGE_SEC` only for a documented field reason.

The script also requires typing `ENABLE_REAL_ROBOT_MOTION`. Do not bypass these gates with ad-hoc `ros2 launch` commands. If a supervised field run intentionally wants the connected wrapper to hand off to this entrypoint after acceptance, set `RUN_REAL_AFTER_ACCEPTANCE=true` explicitly.
