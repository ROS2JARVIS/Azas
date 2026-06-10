# ArUco Lid Screw-On Holder Sequence

This path is implemented as `azas_motion/lid_screw_on_holder_node.py` and is
triggered through `std_srvs/Trigger` on `/azas/lid_screw_on_holder/run`.

## Coordinate Policy

- The node does not ask the operator for cup coordinates.
- Cup-holder X/Y comes from `src/azas_bringup/config/calibration.yaml`.
- A fresh base-link cup pose from `/jarvis/tumbler_dispenser/tumbler_pose` is
  required by default before screw motion.
- The lid pose comes from the ArUco marker attached to the lid.
- The node refuses to run unless measured values are supplied for:
  - `lid_aruco_marker_length_m`
  - `lid_pick_tcp_z_offset_m`
  - either `lid_holder_tcp_z_m` or `cup_holder.lid_screw.tcp_pose_xyz_m`

## Motion Summary

1. Detect the lid ArUco marker from the wrist camera.
2. Pick the lid with the gripper yaw aligned to the marker direction.
3. Move the held lid to the calibrated cup-holder target.
4. Tighten with joint 6:
   - rotate joint 6 by `screw_turn_deg` (default 180 degrees),
   - open the gripper,
   - rotate joint 6 back while open,
   - re-detect the ArUco marker and re-grip,
   - repeat for `screw_cycles` (default 2).
5. Release and retreat above the holder.

## Safety Assumptions

- The cup is already seated in the measured cup holder.
- The cup pose bridge is publishing the holder cup pose with
  `PoseStamped.header.frame_id == "base_link"`.
- The ArUco marker is visible from the configured observe/approach pose.
- The lid thread direction matches `screw_turn_direction`; use `-1.0` for the opposite thread direction.
- The gripper widths and TCP Z offsets are measured for the actual lid and RG2 fingertips.
- Table/workspace/dispenser collision publishers remain enabled for real motion.

## Speed Limits

The screw turn uses a separate low-speed joint planner:

- `screw_joint_velocity_scale` default: `0.04`
- `screw_joint_acceleration_scale` default: `0.03`

Increase these only after supervised low-speed validation.

## Failure Behavior

The sequence fails closed if:

- ArUco detection is missing or tilted beyond `lid_aruco_max_tilt_deg`,
- `/jarvis/tumbler_dispenser/tumbler_pose` is missing, stale, not in
  `base_link`, or not near the calibrated holder,
- required measured lid parameters are unset,
- the cup-holder target is outside `safety.yaml` workspace bounds,
- joint 6 planning/execution does not reach the target,
- re-grip ArUco center drifts beyond `regrip_max_xy_error_m`.

## Validation

Run first in plan-only mode:

```bash
ros2 launch azas_bringup lid_screw_on_holder.launch.py \
  execute_motion:=false \
  lid_aruco_marker_length_m:=<measured_marker_side_m> \
  lid_pick_tcp_z_offset_m:=<measured_pick_tcp_z_offset_m> \
  lid_holder_tcp_z_m:=<measured_holder_lid_tcp_z_m>
```

Trigger the sequence from another terminal:

```bash
ros2 service call /azas/lid_screw_on_holder/run std_srvs/srv/Trigger "{}"
```

For real motion, add `execute_motion:=true` and
`hardware_confirm:=ENABLE_REAL_ROBOT_MOTION` only after supervised validation.
