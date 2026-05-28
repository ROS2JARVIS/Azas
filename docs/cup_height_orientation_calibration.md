# Cup Height Orientation Calibration

This procedure tunes the depth-height orientation gate without asking for or
inventing cup coordinates. Cup pose for motion still comes only from
`/jarvis/tumbler_dispenser/tumbler_pose` after `/azas/cup_detection` reports an
upright detection.

## 1. Observation Pose

Use the existing supervised observation flow or legacy `move_camera_home()` path
to move the EE/camera to the measured worktable viewing pose. Confirm and record
only measured values:

- `camera_home_x`
- `camera_home_y`
- `camera_home_z`
- `home_ori`

If any of these values are `null` or marked `확인 필요` in calibration material,
stop and measure them first. Do not generate substitute values. Once this pose is
fixed, keep the robot and camera stationary during baseline capture and sample
recording.

## 2. Empty Table Baseline

Clear the worktable completely, then start the detector with baseline capture:

```bash
ros2 launch azas_bringup yolo_perception.launch.py \
  capture_empty_table_baseline:=true \
  baseline_frame_count:=30 \
  empty_table_baseline_path:=/tmp/azas_empty_table_depth_baseline.npy
```

Wait for:

```text
Empty-table depth baseline ready
```

The saved `table_depth_map` is the per-pixel median of 30 aligned depth frames
in meters.

## 3. Sample Recording

Place the cup at the requested positions without moving the camera. For each
sample, record the `/azas/cup_detection` `status` fields:

- `bbox`
- `orientation`
- `table_height_m`
- `height_median`
- `height_p90`
- `height_max`
- `height_valid_ratio`

Required upright samples: center, top, bottom, left, right.

Required lying samples: center, top, bottom, left, right.

Required diagonal or ambiguous samples: center, top, bottom.

## 4. Threshold Selection

Compute:

- `standing_min`: minimum selected height statistic from upright samples
- `lying_max`: maximum selected height statistic from lying samples

Set thresholds between them:

```bash
ros2 launch azas_bringup yolo_perception.launch.py \
  empty_table_baseline_path:=/tmp/azas_empty_table_depth_baseline.npy \
  cup_standing_height_threshold_m:=0.075 \
  cup_side_lie_height_threshold_m:=0.065 \
  height_stat_for_orientation:=p90
```

Or keep the same detector process alive and set the parameters dynamically:

```bash
ros2 param set /yolo_tumbler_detector_node cup_standing_height_threshold_m 0.075
ros2 param set /yolo_tumbler_detector_node cup_side_lie_height_threshold_m 0.065
```

Use measured thresholds only. The example values above are placeholders for the
shape of the command, not robot-cell calibration data. If the saved baseline file
is not supplied on restart, height statistics are unavailable and the detector
falls back to the bbox ratio heuristic.

## 5. Acceptance

Expected `/azas/cup_detection` behavior:

- standing cup: `detected:upright`
- lying cup: `rejected:lying_or_unknown orientation=lying`
- diagonal/ambiguous cup: `rejected:unknown_orientation orientation=unknown` or lying

Only `detected:upright` detections are converted by
`cup_detection_pose_bridge_node` into `/jarvis/tumbler_dispenser/tumbler_pose`.
