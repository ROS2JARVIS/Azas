# Real Motion Measurement Worksheet

Purpose: collect the measured values required before `tools/run/run_robot_real.sh` can be allowed.

Do not guess these values. Fill `calibration.yaml` and `safety.yaml` only from robot-cell measurement, calibration output, or Doosan/RG2 configuration evidence.

## Required Measurements

| Field | File | Required evidence | Notes |
| --- | --- | --- | --- |
| `frames.camera_frame` | `src/azas_bringup/config/calibration.yaml` | Live `CameraInfo.header.frame_id` | Current RealSense evidence used `camera_color_optical_frame`, but verify in the final bringup. |
| `frames.ee_link` | `calibration.yaml` | MoveIt robot model / planning group output | Must match the actual Doosan M0609 MoveIt config. |
| `frames.planning_group` | `calibration.yaml` | MoveIt group name | Common examples use `manipulator`; verify locally. |
| `frames.gripper_tcp` | `calibration.yaml` | TF or tool frame setup | Must represent the RG2 tool center point used for planning. |
| `hand_eye.child_frame` | `calibration.yaml` | Camera frame used by hand-eye calibration | Should normally match `frames.camera_frame`. |
| `hand_eye.xyz_m` | `calibration.yaml` | Hand-eye calibration result | Transform from `base_link` to camera frame. |
| `hand_eye.rpy_rad` | `calibration.yaml` | Hand-eye calibration result | RPY in radians, same transform convention as documented. |
| `cup_offsets.default.tcp_to_cup_mouth_m` | `calibration.yaml` | Jig/dry-run measurement | Vector from gripper TCP to the cup mouth center after grasp. |
| `outlet.pose_xyz_m` | `calibration.yaml` | Teaching/calibration of fixed dispenser outlet | Base-frame outlet center. |
| `outlet.pose_rpy_rad` | `calibration.yaml` | Teaching/calibration of outlet frame | Use a documented orientation convention. |
| `outlet.clearance_m` | `calibration.yaml` | Measured rim/outlet clearance | Must be numeric and positive. |
| `dispenser_outlets."1".."4".outlet_pose_xyz_m` | `calibration.yaml` | Teaching/calibration per dispenser number | Base-frame outlet center for each fixed dispenser ID selected by STT/LLM. |
| `dispenser_outlets."1".."4".outlet_pose_rpy_rad` | `calibration.yaml` | Teaching/calibration per dispenser number | Orientation for aligning the held cup under each outlet. |
| `dispenser_outlets."1".."4".press_pose_xyz_m` | `calibration.yaml` | Teaching/calibration per dispenser number | Base-frame press/actuation pose for each fixed dispenser ID. |
| `dispenser_outlets."1".."4".press_pose_rpy_rad` | `calibration.yaml` | Teaching/calibration per dispenser number | Orientation for the dispenser press primitive. |
| `dispenser_outlets."1".."4".clearance_m` | `calibration.yaml` | Measured rim/outlet clearance per dispenser | Must be numeric and positive. |
| `motion.workspace_bounds_m` | `src/azas_bringup/config/safety.yaml` | Measured workcell bounds | Include only reachable safe robot cell volume. |
| `motion.min_z_m` | `safety.yaml` | Table/collision clearance measurement | Lower Z limit for planning. |
| `gripper.default_width_m` | `safety.yaml` | RG2 command/feedback units check | Confirm width unit conversion. |
| `gripper.default_force_n` | `safety.yaml` | RG2 force limit decision | Start low; document actual value. |

## Recommended Field Sequence

1. Start camera dry-run:

   ```bash
   ENABLE_RG2=false /home/ssu/Azas/tools/run/run_robot_dryrun.sh
   ```

2. Capture current no-motion status:

   ```bash
   /home/ssu/Azas/tools/run/field_no_motion_report.sh
   ```

3. Verify cup and lid stability with the actual tumbler body and lid:

   ```bash
   RUN_LID_STABILITY=true RUN_CUP_STABILITY=true /home/ssu/Azas/tools/run/field_no_motion_report.sh
   ```

4. Connect/start Doosan and RG2 services, still without commanding motion.

5. Fill measured `calibration.yaml` and `safety.yaml`.

6. Run strict gate:

   ```bash
   STRICT_LIVE_GATE=true RUN_LID_STABILITY=true RUN_CUP_STABILITY=true /home/ssu/Azas/tools/run/field_no_motion_report.sh
   ```

Only after the strict gate writes `/tmp/azas_live_hardware_gates_passed` should `tools/run/run_robot_real.sh` be considered.

## Current Known Blockers

- Cup/tumbler-body detection stability has not been captured in the current session.
- Real Doosan `MoveLine`/`MoveJoint` services are not currently present.
- Real RG2 open/close Trigger services are not currently present.
- Hand-eye/base-camera transform has not been measured.
- `calibration.yaml` and `safety.yaml` still contain placeholder values.
