# External Grasp Detector Adapter

This note defines the offline boundary for testing RGB-D grasp detectors before
connecting them to Azas runtime topics. It is intentionally non-hardware:
exporting frames and checking JSON files must not command Doosan motion, MoveIt
execution, or RG2.

## Frame Export

Export one RealSense/YOLO frame:

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/setup.bash
python3 /home/ssu/Azas/tools/perception/export_grasp_frame.py \
  --output-dir /tmp/azas_grasp_frame \
  --rgb-topic /camera/camera/color/image_raw \
  --depth-topic /camera/camera/aligned_depth_to_color/image_raw \
  --camera-info-topic /camera/camera/color/camera_info \
  --detection-topic /azas/cup_detection
```

Expected output:

```text
/tmp/azas_grasp_frame/
  rgb.png
  depth_m.npy
  camera_info.json
  bbox.json          # optional, when /azas/cup_detection is received
  mask.png           # optional, when --mask-topic is provided
  manifest.json
```

`depth_m.npy` is always float depth in meters. Auto scale follows the existing
Azas depth rule: `16UC1` and `mono16` use `0.001`; `32FC1` uses `1.0`.

## Detector Input Contract

External detectors should consume:

- `rgb.png`: color image.
- `depth_m.npy`: aligned depth in meters.
- `camera_info.json`: camera matrix `k` / `K`.
- optional `bbox.json`: YOLO status-derived class, confidence, bbox size, and
  center when available.
- optional `mask.png`: binary object crop mask.

The detector may run in a separate conda environment or external workspace. Do
not copy detector source trees or licensed SDK binaries into Azas.

## Detector Output Contract

Write:

```text
/tmp/azas_grasp_frame/grasp_candidates.json
```

Schema:

```json
{
  "format": "azas_grasp_candidates_v1",
  "frame_id": "camera_color_optical_frame",
  "source": "graspnet_baseline",
  "candidates": [
    {
      "candidate_id": "0",
      "source": "graspnet_baseline",
      "score": 0.91,
      "width_m": 0.064,
      "pose": {
        "position": {"x": 0.0, "y": 0.0, "z": 0.5},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
      },
      "status": "raw_camera_frame; requires_tf_workspace_collision_moveit_filter"
    }
  ]
}
```

The pose is camera-frame 6D grasp pose. Azas must still transform it to
`base_link`, apply workspace and collision checks, run MoveIt planning-only, and
select a supervised candidate before any real pick trial.

## Optional GPD ROS Adapter

`gpd_grasp_adapter_node` is an optional boundary for external workspaces that
already provide `gpd_ros/msg/GraspConfigList`. Azas does not list `gpd_ros` as a
required ROS dependency because the known `gpd_ros` package is ROS1/catkin, not
a ROS 2 Humble package.

The adapter publishes only the selected grasp candidate pose:

```text
/azas/gpd/grasp_pose  geometry_msgs/PoseStamped
```

This topic is not `/azas/cup_detection` and must not be remapped into the cup
pose bridge. A GPD grasp pose is not a verified `cup_mouth_center`; it still
requires TF, workspace, collision, and planning-only gates before any motion
path can consume it.

Current `azas_interfaces/msg/GraspCandidate` has no `width_m` or
`collision_score` fields. Until the message is extended, bridge nodes should
preserve detector width/collision metadata in `status` and keep the raw JSON as
evidence.

Validate the exported frame and optional candidates:

```bash
python3 /home/ssu/Azas/tools/checks/check_grasp_adapter_contract.py \
  --frame-dir /tmp/azas_grasp_frame \
  --candidates-json /tmp/azas_grasp_frame/grasp_candidates.json
```

## Recommended Detector Order

1. `graspnet/graspnet-baseline`: first offline experiment. Use the RealSense
   pretrained checkpoint if available. It accepts RGB-D plus intrinsics and is a
   better fit for RTX 5080 than older TensorFlow/CUDA stacks.
2. `NVlabs/contact_graspnet`: second experiment if GraspNet baseline quality is
   poor. It accepts depth/K/segmap or point cloud, but the published environment
   is older Python/TensorFlow/CUDA.
3. `graspnet/anygrasp_sdk`: revisit after license registration. Do not include
   SDK binaries in Azas without approval.
4. `atenpas/gpd`: CPU/PCL fallback if CUDA/PyTorch custom ops block progress.

## Failure Handling

- If no frame can be exported, fix RealSense topics and CameraInfo first.
- If candidates are generated but all fail MoveIt planning-only, inspect TCP
  frame, approach axis, collision scene, and table height.
- If detector installation fails, record the exact Python/CUDA/torch error in
  `dependencies/README.md` and switch to the next detector. Do not weaken the
  real-motion gates.
