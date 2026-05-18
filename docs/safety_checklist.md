# Safety Checklist

Use this before any real robot execution. MVP-1 must fail closed: no calibrated value, no robot motion.

- Connecting camera, Doosan, or RG2 must not be treated as permission to move the robot.
- `tools/run/run_robot_dryrun.sh` is allowed for connection checks because it launches with `enable_hardware:=false`.
- Real motion must only use `tools/run/run_robot_real.sh`; do not use ad-hoc `ros2 launch` commands for real motion.
- `tools/run/run_robot_real.sh` requires a fresh strict live-gate stamp and the exact typed confirmation `ENABLE_REAL_ROBOT_MOTION`.
- Confirm M0609 virtual/fake mode first; real mode only after dry-run evidence.
- Confirm emergency stop behavior and operator position.
- Confirm low speed/acceleration limits in `safety.yaml`.
- Confirm actual MoveIt `GROUP_NAME`, `EE_LINK`, controller names, and HOME pose.
- Confirm camera topics, frame IDs, depth units, and TF freshness.
- Confirm `Z = depth_raw / 1000.0` projection with a known-distance target.
- Confirm `RUN_LID_STABILITY=true RUN_CUP_STABILITY=true /home/ssu/Azas/tools/run/field_no_motion_report.sh` reports stable lid and cup/tumbler-body detections.
- Confirm hand-eye/static transform from `camera_frame` to `base_link`.
- Confirm `dispenser_outlet` pose by teaching/calibration; do not use guessed coordinates.
- Confirm `gripper_tcp` to `cup_mouth_center` offset with a jig/dry-run.
- Confirm RG2 open/close width, force units, timeout, and grip success signal.
- Confirm table/workspace collision geometry and minimum safe `z`.
- Confirm no person enters the robot workspace during motion.
- Stop on detection, TF, planning, gripper, or calibration uncertainty.
- Confirm `STRICT_LIVE_GATE=true /home/ssu/Azas/tools/run/field_no_motion_report.sh` passes before `tools/run/run_robot_real.sh`.
