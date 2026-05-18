# Recovery After Poweroff

Use this page if the PC shuts down before the robot-control work is finished.

## Current State

As of 2026-05-11:

- Non-hardware OSS robot-control path is wired.
- DSR_DeepTree demo patterns were applied as a no-motion cocktail task sequence.
- RealSense D435i was previously verified, but current topic state must be rechecked after reboot.
- Real robot motion is still blocked.
- Doosan/RG2 live services, hand-eye calibration, cup-body stability, and measured safety/calibration config remain unfinished.

Do not run ad-hoc real robot launch commands. Use the gated scripts below.

## Reboot Recovery Commands

From a new terminal:

```bash
source /opt/ros/humble/setup.bash
cd /home/ssu/Azas
colcon build --symlink-install
source install/setup.bash
```

If Jarvis/Doosan bridge packages are needed:

```bash
source /opt/ros/humble/setup.bash
cd /home/ssu/ros2_ws
colcon build --packages-select jarvis --symlink-install
source install/setup.bash
```

Run the non-hardware verifier:

```bash
/home/ssu/Azas/tools/checks/verify_control_readiness.sh
```

Run the current field no-motion report:

```bash
/home/ssu/Azas/tools/run/field_no_motion_report.sh
```

After camera, Doosan, and RG2 are connected, run the one-command acceptance
check:

```bash
/home/ssu/Azas/tools/checks/robot_connection_acceptance.sh
```

Review measured-value blockers in `docs/real_motion_measurement_worksheet.md`.

## Camera Restart

For camera-only dry-run after plugging in RealSense:

```bash
ENABLE_RG2=false /home/ssu/Azas/tools/run/run_robot_dryrun.sh
```

Then check:

```bash
/home/ssu/Azas/tools/checks/check_connection_stage.sh
/home/ssu/Azas/tools/checks/check_depth_projection_sample.sh
/home/ssu/Azas/tools/checks/check_robot_detection.sh
```

Cup/lid stability checks:

```bash
/home/ssu/Azas/tools/checks/check_cup_lid_sequence.sh
```

## Robot/RG2 No-Motion Gate

After Doosan/RG2 services are running, still before real motion:

```bash
ROBOT_HOST=<robot-ip> DOOSAN_NO_MOTION_CONFIRM=CONNECT_DOOSAN_NO_MOTION /home/ssu/Azas/tools/run/run_doosan_real_no_motion_m0609.sh
```

```bash
STRICT_LIVE_GATE=true RUN_LID_STABILITY=true RUN_CUP_STABILITY=true /home/ssu/Azas/tools/run/field_no_motion_report.sh
```

This must write:

```text
/tmp/azas_live_hardware_gates_passed
```

If that file is missing, stale, or not strict, `tools/run/run_robot_real.sh` refuses to launch.

## Main Files To Inspect

- `docs/control_readiness_audit.md`
- `docs/field_control_runbook.md`
- `docs/real_motion_measurement_worksheet.md`
- `docs/dsr_deeptree_integration.md`
- `docs/full_cocktail_workflow_plan.md`
- `tools/checks/verify_control_readiness.sh`
- `tools/run/field_no_motion_report.sh`
- `tools/checks/robot_connection_acceptance.sh`

## Safe Stop Point

If interrupted, the safe resume point is:

```bash
/home/ssu/Azas/tools/checks/verify_control_readiness.sh
/home/ssu/Azas/tools/run/field_no_motion_report.sh
```

Do not claim real robot-control readiness until `docs/control_readiness_audit.md` has no missing real-hardware items.
