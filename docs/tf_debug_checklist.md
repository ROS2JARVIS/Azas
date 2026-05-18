# TF debug checklist

Use this checklist to debug camera-to-base TF and tumbler pose wiring without
commanding robot motion.

## ROS log directory workaround

Some field machines cannot write ROS logs under `/home/ssu/.ros/log`. Set a
writable log directory before starting ROS nodes. This is a temporary
workaround; investigate the read-only `/home/ssu/.ros/log` cause separately.

Use this order:

```bash
mkdir -p /tmp/ros2_logs
touch /tmp/ros2_logs/test_write
export ROS_LOG_DIR=/tmp/ros2_logs
source /opt/ros/humble/setup.bash
```

Confirm the local Humble `static_transform_publisher` accepts named arguments:

```bash
ros2 run tf2_ros static_transform_publisher --help
```

## Start virtual Doosan

Run the Doosan virtual bringup first:

```bash
mkdir -p /tmp/ros2_logs
touch /tmp/ros2_logs/test_write
export ROS_LOG_DIR=/tmp/ros2_logs
source /opt/ros/humble/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py model:=m0609 mode:=virtual host:=127.0.0.1 port:=12345
```

This is a dry-run environment. It does not prove real robot calibration,
workspace safety, RG2 behavior, or physical reachability.

## Optional dry-run static TF

If no measured hand-eye TF is available, publish a temporary static transform
only for TF graph debugging:

```bash
mkdir -p /tmp/ros2_logs
touch /tmp/ros2_logs/test_write
export ROS_LOG_DIR=/tmp/ros2_logs
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link \
  --child-frame-id camera_color_optical_frame
```

The placeholder static TF values in
`src/azas_bringup/config/calibration.yaml` must not be used on the real robot.
They are example values only, not measured calibration.

## Camera dry-run launch

In a separate terminal, launch the current YOLO-to-floor-place path with a
temporary static TF. The values below are placeholders for TF pipeline debugging
only and must not be used for real robot execution:

```bash
mkdir -p /tmp/ros2_logs
touch /tmp/ros2_logs/test_write
export ROS_LOG_DIR=/tmp/ros2_logs
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
ros2 launch azas_bringup yolo_to_floor_place.launch.py \
  publish_camera_base_tf:=true \
  source_frame:=camera_color_optical_frame \
  target_class_names:=cup,tumbler,bottle \
  selection_policy:=largest_bbox \
  depth_window_size:=7 \
  min_depth_m:=0.15 \
  max_depth_m:=2.0 \
  debug_pose_logging:=true \
  camera_base_tf_x:=0.0 \
  camera_base_tf_y:=0.0 \
  camera_base_tf_z:=0.0 \
  camera_base_tf_roll:=0.0 \
  camera_base_tf_pitch:=0.0 \
  camera_base_tf_yaw:=0.0 \
  allow_demo_tumbler_position_fallback:=false
```

Expected detector logs:

- selected class is one of `cup`, `tumbler`, or `bottle`
- if multiple detections pass confidence and class filters, the largest bbox
  area wins
- bbox center pixel is `((xmin+xmax)/2, (ymin+ymax)/2)`
- depth is the median of valid depth values in the `depth_window_size` window
- depth is rejected if it is zero, NaN, inf, below `min_depth_m`, or above
  `max_depth_m`
- depth scale is selected from the depth image encoding in `depth_scale_mode=auto`:
  `16UC1`/`mono16` use millimeter scale `0.001`, and `32FC1` uses meter scale
  `1.0`
- projected camera point is logged in `camera_color_optical_frame`
- transformed pose is logged and published only when TF to `base_link` succeeds

Confirm the live aligned-depth encoding before trusting projected camera points:

```bash
ros2 topic echo --once /camera/camera/aligned_depth_to_color/image_raw | grep encoding
```

The bridge may use latest TF when a detection stamp is zero, and can optionally
retry latest TF when `allow_latest_tf_fallback:=true`. That fallback is for
diagnosing timestamp problems only; it does not prove real-motion readiness.

## Inspect TF

Check whether `base_link` can transform to the camera optical frame:

```bash
mkdir -p /tmp/ros2_logs
touch /tmp/ros2_logs/test_write
export ROS_LOG_DIR=/tmp/ros2_logs
source /opt/ros/humble/setup.bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

Write a TF graph snapshot:

```bash
mkdir -p /tmp/ros2_logs
touch /tmp/ros2_logs/test_write
export ROS_LOG_DIR=/tmp/ros2_logs
source /opt/ros/humble/setup.bash
ros2 run tf2_tools view_frames
```

Expected dry-run result: `base_link` and `camera_color_optical_frame` appear in
one connected tree. If `tf2_echo` cannot resolve the transform, stop before
running any pose-to-motion path.

## Inspect tumbler pose topics

List TF, tumbler, camera, and YOLO topics:

```bash
mkdir -p /tmp/ros2_logs
touch /tmp/ros2_logs/test_write
export ROS_LOG_DIR=/tmp/ros2_logs
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E "tf|tumbler|camera|yolo"
```

Watch the pose consumed by the current floor-place bridge:

```bash
mkdir -p /tmp/ros2_logs
touch /tmp/ros2_logs/test_write
export ROS_LOG_DIR=/tmp/ros2_logs
source /opt/ros/humble/setup.bash
ros2 topic echo /jarvis/tumbler_dispenser/tumbler_pose
```

Expected dry-run result: the pose header frame is either directly in the
`base_link` tree or has a resolvable TF path to `base_link`; the published
`/jarvis/tumbler_dispenser/tumbler_pose` message must have
`header.frame_id: base_link`. If the pose is missing, stale, or in an
unconnected frame, keep the system in no-motion mode.

## Stop conditions

- Stop if `tf2_echo` cannot resolve `base_link -> camera_color_optical_frame`.
- Stop if `/jarvis/tumbler_dispenser/tumbler_pose` is absent or stale.
- Stop if the pose frame is not exactly `base_link`.
- Stop if `calibration.yaml` still contains null or unconfirmed real robot
  calibration values.
- Do not replace measured hand-eye calibration with example static TF values.
- Do not treat latest-TF fallback success as measured hand-eye calibration.
- Do not connect real robot execution, RG2 control, or `PickAndAlignActionServer`
  execution while using placeholder TF values.
