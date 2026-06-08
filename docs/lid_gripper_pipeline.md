# Lid Gripper Pipeline

This path is for detecting the cup lid center marker, showing a live RealSense preview,
and generating supervised lid grip requests. By default it is plan-only and does
not execute Doosan motion.

## Runtime Contract

1. `lid_sticker_detector_node`
   - subscribes to RealSense color, aligned depth, and camera info,
   - runs the trained YOLO model for class `lid`,
   - detects a 30 mm ArUco marker inside the lid bbox,
   - projects the ArUco center with aligned depth,
   - estimates the local lid plane normal from the surrounding depth patch,
   - publishes `/azas/lid_detection` as `azas_interfaces/msg/CupDetection` with status `detected:lid ...`.
   - opens an OpenCV preview window by default and overlays the ArUco marker, optional lid bbox, depth, and status.
   - pressing `p` in the preview publishes `/jarvis/lid_gripper/grip_request`; it is ignored unless the current frame is `detected:lid`.

2. `cup_detection_pose_bridge_node`
   - runs a second instance configured for lid detections,
   - accepts only status prefix `detected:lid`,
   - transforms the camera-frame pose into `base_link`,
   - publishes `/jarvis/lid_gripper/lid_pose`.

3. `lid_grip_planner_node`
   - subscribes to `/jarvis/lid_gripper/lid_pose`,
   - subscribes to `/jarvis/lid_gripper/grip_request` for supervised `p` key requests,
   - validates `frame_id == base_link`,
   - publishes:
     - `/jarvis/lid_gripper/approach_pose`
     - `/jarvis/lid_gripper/grasp_pose`
     - `/jarvis/lid_gripper/lift_pose`
     - `/jarvis/lid_gripper/status`
   - only sends Doosan `MoveLine` requests when `enable_hardware:=true`, `hardware_confirm:=ENABLE_REAL_ROBOT_MOTION`, and `allow_service_control_without_moveit:=true`.

## Safety Assumptions

- The ArUco marker defines the lid center target. Current motion still uses the configured Doosan `rx/ry/rz` for TCP orientation.
- The grip angle comes from the depth-estimated lid plane normal.
- The hand-eye matrix used by this launch is copied into `src/azas_perception/config/T_gripper2camera.npy`
  from `/home/ssu/ros2_ws/src/doosan-robot2/dsr_practice/dsr_practice/Calibration_Tutorial/T_gripper2camera.npy`.
  The copied matrix hash was verified to match the original:
  `c7fa0eb6aefc6afec2ed36b3672afb0515472142b653ba5e36cb57e114984d4c`.
- The original calibration capture used Doosan `set_tcp("2FG_TCP")` before `get_current_posx()`.
  The current RViz/MoveIt-compatible default publishes the matrix under `hand_eye_parent_frame:=link_6`,
  matching the existing legacy MoveIt nodes. Before real hardware use, confirm whether the live TF tree
  exposes the calibrated TCP separately and override `hand_eye_parent_frame` if needed.
- The default launch is planning-only: no MoveIt execution, no Doosan service call, and no RG2 command.
- `p` is a supervised trigger, not automatic motion. With default parameters it only confirms the latest plan.
- RG2 service calls remain disabled unless `enable_gripper_service_calls:=true` and measured gripper width/force parameters are provided.
- Real lid gripping additionally requires the explicit hardware gates above, verified TF, conservative speed/acceleration values, measured RG2 widths, and operator clearance. The direct `MoveLine` path uses the configured `rx/ry/rz`; validate those values before hardware use.
- `surface_offset_m` moves the grasp target along `offset_axis`, which defaults to the detected lid normal (`local_z`). Use `offset_axis:=base_z` when tuning only vertical clearance.
- `tcp_grasp_offset_x_m/y_m/z_m` is a measured base-frame offset from the detected lid point to the actual RG2 TCP grasp point. It is a gripper/TCP tuning value, not a cup coordinate.
- With hardware enabled, the planner fail-closes before gripper motion when `motion/ikin` is unavailable or fails, and verifies each `MoveLine` target with `aux_control/get_current_posx`.
- RG2 `preopen` is sent only after the approach target is verified. RG2 `grasp` is sent only after the grasp target is verified.
- If the Doosan controller accepts a `MoveLine` service request but the TCP does not reach the target within `motion_verify_timeout_sec`, the sequence stops and reports the last Doosan alarm in `/jarvis/lid_gripper/status`.

## Launch

```bash
ros2 launch azas_bringup lid_sticker_grip_planning.launch.py
```

If the model is not in `/home/ssu/Downloads/best.pt`, the launch file also checks
`src/cocktail_robot_system/models/best.pt`. You can override it explicitly:

```bash
ros2 launch azas_bringup lid_sticker_grip_planning.launch.py \
  model_path:=/absolute/path/to/best.pt
```

For the current 30 mm ArUco marker flow, the defaults are:

```bash
marker_type:=aruco
require_lid_detection:=true
aruco_dictionary:=DICT_4X4_50
aruco_marker_id:=-1
aruco_marker_length_m:=0.03
```

Expected live outputs:

```bash
ros2 topic echo --once /azas/lid_detection
ros2 topic echo --once /jarvis/lid_gripper/lid_pose
ros2 topic echo --once /jarvis/lid_gripper/status
```

If `/jarvis/lid_gripper/lid_pose` is missing while `/azas/lid_detection` is present,
check the TF from `base_link` to `camera_color_optical_frame`.

## RViz / No-Robot Check

Use this before connecting the real robot. It should prove only the visual
perception, TF conversion, and lid grip candidate topics.

1. Start the virtual Doosan/MoveIt or any RViz setup that publishes
   `base_link -> link_6`.
2. Start the RealSense driver so it publishes the color image, aligned depth,
   camera info, and camera TF tree.
3. Start:

```bash
ros2 launch azas_bringup lid_sticker_grip_planning.launch.py
```

4. Confirm the preview shows `detected:lid`.
5. Confirm these topics publish:

```bash
ros2 topic echo --once /jarvis/lid_gripper/lid_pose
ros2 topic echo --once /jarvis/lid_gripper/approach_pose
ros2 topic echo --once /jarvis/lid_gripper/grasp_pose
ros2 topic echo --once /jarvis/lid_gripper/lift_pose
```

6. In RViz, add `PoseStamped` displays for the three pose topics above.
   Seeing these poses does not prove collision-free robot motion; it only
   confirms the perception-to-plan geometry.

Hardware-gated supervised request example:

```bash
ros2 launch azas_bringup lid_sticker_grip_planning.launch.py \
  enable_hardware:=true \
  hardware_confirm:=ENABLE_REAL_ROBOT_MOTION \
  allow_service_control_without_moveit:=true \
  service_prefix:=/dsr01 \
  rx:=108.41 \
  ry:=-176.32 \
  rz:=175.98 \
  offset_axis:=base_z \
  surface_offset_m:=0.0 \
  tcp_grasp_offset_x_m:=-0.006 \
  tcp_grasp_offset_y_m:=-0.045 \
  tcp_grasp_offset_z_m:=-0.064 \
  min_grasp_z_m:=0.0 \
  precheck_ikin:=true \
  verify_motion_reached:=true \
  enable_gripper_service_calls:=true \
  gripper_preopen_width_m:=0.050 \
  gripper_grasp_width_m:=0.033 \
  gripper_force_n:=8.0
```

Press `p` only after the preview shows a stable `detected:lid` overlay and
`/jarvis/lid_gripper/lid_pose` is publishing in `base_link`.
