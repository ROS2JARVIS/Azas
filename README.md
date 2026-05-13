# Azas

ROS 2 Humble MVP-1 workspace for Doosan M0609 + OnRobot RG2 + depth vision.

The authoritative project knowledge base is `wiki/`, compiled from the local course PDFs under `/home/ssu/Downloads/로봇`. Current repo scaffold must stay subordinate to that wiki.
When implementation details are unclear, inspect `/home/ssu/Downloads/로봇/` directly before relying on summaries. STT work should follow `17차시(04.24)/31장 STT-로봇 연동.pdf` and `dsr_practice.zip` first.

Power-off recovery entrypoints: `docs/recovery_after_poweroff.md` and `docs/current_handoff_2026-05-11.md`.

## MVP-1 scope

```text
random tumbler detection
-> RG2 grasp
-> align cup_mouth_center below a fixed dispenser_outlet
```

STT/LLM/VLA cocktail selection is post-MVP. It may choose user intent or a recipe later, but it must never generate robot coordinates, trajectories, collision decisions, calibration values, or safety decisions.

## Workspace

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Packages

- `azas_interfaces` - shared MVP messages, services, and `PickAndAlign.action`.
- `azas_perception` - depth/CameraInfo ingestion, tumbler detection, pixel-to-3D projection.
- `azas_calibration` - camera/base, `dispenser_outlet`, and `cup_mouth_center` offset boundaries.
- `azas_motion` - MoveItPy motion coordination skeleton for M0609.
- `azas_gripper` - RG2 service boundary with fake/real separation pending hardware confirmation.
- `azas_task_manager` - `/azas/pick_and_align` action orchestration.
- `azas_bringup` - launch and YAML placeholders.
- `azas_voice` - 17차시 STT pattern adapted for `/stt_result` text input and symbolic cocktail recipe/dispenser color mapping.

`azas_voice` is present as a symbolic STT/recipe-mapping layer. It does not command robot motion; final `/azas/make_cocktail` orchestration remains gated by MVP-1 robot readiness.

## Hardware values policy

The following are deliberately placeholders until measured or confirmed: `EE_LINK`, `GROUP_NAME`, camera topics/frame, hand-eye transform, `dispenser_outlet` pose, RG2 command units/ranges, TCP offset, cup dimensions, table/workspace bounds.

`src/azas_bringup/config/calibration.yaml` separates real calibration fields from dry-run example static TF values. The real fields intentionally remain `null` / `확인 필요` until measured. Placeholder static TF values are for simulation and TF graph debugging only; they must not be used on the real robot.

Start with:

- `wiki/overview.md`
- `wiki/syntheses/MVP-1 Tumbler Pick And Dispenser Alignment Plan.md`
- `wiki/syntheses/GitHub Collaboration Task Breakdown.md`
- `wiki/syntheses/ROS 2 Package Architecture.md`

## PickAndAlign no-motion status

`PickAndAlignActionServer` defaults to `execution_mode=no_motion`. It waits for a
`PoseStamped` on `/jarvis/tumbler_dispenser/tumbler_pose`, requires
`header.frame_id: base_link` by default, and computes side-grasp
approach/grasp/lift poses for feedback/logging. The default `grasp_mode=side`
returns `NO_MOTION_SIDE_GRASP_OK` after `DONE_NO_MOTION`. It does not call
MoveIt, Doosan motion, or real RG2 hardware.
`execution_mode=skeleton` is still available for the original `SKELETON_ONLY`
contract, and `grasp_mode=vertical` keeps the earlier z-offset no-motion plan.

The intended pick order now starts with an observation stage:

```text
HOME
-> OBSERVE_CUP_POSE
-> DETECT_CUP
-> COMPUTE_SIDE_GRASP
-> PLAN_SIDE_GRASP
-> GRIPPER_OPEN
-> MOVE_APPROACH
-> MOVE_GRASP
-> GRIPPER_CLOSE
-> LIFT
-> DONE
```

`OBSERVE_CUP_POSE` is now handled by the supervised single-pick tool as a
HOME-based joint target, because Cartesian observe IK produced confusing wrist
solutions. The default sequence starts at HOME
`j1=0, j2=0, j3=90, j4=0, j5=90, j6=0`, then moves to observe
`j1=0, j2=25, j3=65, j4=0, j5=135, j6=0`. The positive `joint_2` observe
target lifts the arm forward; `joint_5=135` is an operator-tunable camera
direction candidate for seeing the cup/work area. This is not a calibrated
camera pose guarantee.

Planning-only observe check:

```bash
/home/ssu/Azas/tools/check_observe_pose_planning_only.sh
```

Supervised observe/pick entrypoint:

```bash
python3 /home/ssu/Azas/tools/run_supervised_real_single_cup_pick.py --help
```

Implementation files are organized under `tools/pick/`; root-level `tools/*.py`
files are compatibility wrappers while older docs and teammate commands are
migrated. See `tools/README.md` before adding new scripts.

After an operator-approved observe pose is reached in a future batch, capture
the cup scene with RealSense and export a detector frame:

```bash
ros2 launch realsense2_camera rs_align_depth_launch.py ...

python3 tools/export_grasp_frame.py \
  --output /tmp/azas_grasp_frame \
  --rgb-topic /camera/camera/color/image_raw \
  --depth-topic /camera/camera/aligned_depth_to_color/image_raw \
  --camera-info-topic /camera/camera/color/camera_info \
  --timeout-sec 10

python3 tools/export_grasp_frame.py \
  --output /tmp/azas_grasp_frame \
  --rgb-topic /camera/camera/color/image_raw \
  --depth-topic /camera/camera/aligned_depth_to_color/image_raw \
  --camera-info-topic /camera/camera/color/camera_info \
  --wait-for-bbox \
  --timeout-sec 10
```

Current side grasp is a no-motion approximation. The
`/jarvis/tumbler_dispenser/tumbler_pose` position is treated only as a cup
reference pose, `grasp_height_offset_m` is an offset from that reference, and
`side_grasp_qx/qy/qz/qw` is only a planning-only TCP quaternion candidate. The
quaternion is normalized before use and remains a placeholder until measured.
Side grasp is currently limited to upright cups only. The YOLO detector uses a
simple bbox aspect-ratio heuristic before publishing a graspable detection:
`bbox_height / bbox_width >= 1.2` publishes `detected:upright`,
`< 0.8` publishes `rejected:lying_or_unknown`, and the middle band publishes
`rejected:unknown_orientation`. This heuristic is only a fail-closed guard; it
does not prove the cup axis, mouth direction, table contact, or grasp surface.
The pose bridge refuses to publish `/jarvis/tumbler_dispenser/tumbler_pose` for
non-upright statuses, and `/azas/pick_and_align` reports
`CUP_ORIENTATION_NOT_UPRIGHT` or `CUP_ORIENTATION_UNKNOWN` if a rejected
detection is observed instead of an upright pose.
Before any real side grasp, measured hand-eye/base-camera TF, cup center/radius,
table height, TCP quaternion, gripper width/force, collision scene/clearance,
operator clearance, and e-stop readiness must be verified with hardware gates.
Lying, upside-down, or ambiguous cups must be handled in a later perception step
with mask/PCA/point-cloud pose estimation before real grasp planning.

Planning-only side grasp check:

```bash
/home/ssu/Azas/tools/check_side_grasp_planning_only.sh
```

Candidate sweep for planning-only side grasp feasibility:

```bash
python3 /home/ssu/Azas/tools/sweep_side_grasp_planning_candidates.py \
  --planning-group manipulator \
  --ee-link tool0 \
  --cup-reference-x 0.42 \
  --cup-reference-y -0.24 \
  --cup-reference-z 0.05 \
  --max-candidates 100
```

Planning-only means trajectory feasibility preparation/reporting only. When
verified `planning_group` and `ee_link` are provided, `alignment_executor_node`
constructs MoveItPy planning requests for approach, grasp, and lift poses and
calls `PlanningComponent.plan()`. It is not real readiness, and
`alignment_executor_node` keeps `allow_execute=false` by default.
Current evidence points to `tool0` as the more relevant TCP candidate than the
SRDF chain tip `link_6`, but the final TCP link/quaternion must still be
measured. Planning success for a swept quaternion/axis/height candidate is only a
feasibility signal; it is not proof of real cup contact, grip quality, or safe
robot execution.

No-motion action smoke:

```bash
/home/ssu/Azas/tools/smoke_pick_and_align_no_motion.sh
```

## TF debug dry-run

Use `docs/tf_debug_checklist.md` when checking camera-to-base TF and tumbler pose wiring without commanding robot motion.

The current detector selects one target cup-like object by:

- class filter: `cup`, `tumbler`, or `bottle`
- selection policy: largest bounding-box area
- representative pixel: bbox center
- orientation gate: upright-only bbox heuristic; non-upright or ambiguous
  boxes are rejected before the pose bridge publishes a robot-frame pose
- depth: median valid depth in a 7x7 center window
- depth scale: `depth_scale_mode=auto` maps `16UC1`/`mono16` to `0.001`
  meter-per-mm scale and `32FC1` to `1.0` meter scale
- reject: zero, NaN, inf, `<0.15 m`, or `>2.0 m` depth

The pose bridge publishes `/jarvis/tumbler_dispenser/tumbler_pose` only after TF
conversion succeeds. The published `PoseStamped.header.frame_id` must be
`base_link`; camera-frame poses must not be treated as robot-frame poses.
Latest-TF fallback is a diagnostic aid for timestamp problems only; it is not
real robot readiness evidence.

Start virtual Doosan:

```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py model:=m0609 mode:=virtual host:=127.0.0.1 port:=12345
```

Inspect TF and pose wiring:

```bash
mkdir -p /tmp/ros2_logs
touch /tmp/ros2_logs/test_write
export ROS_LOG_DIR=/tmp/ros2_logs
source /opt/ros/humble/setup.bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_tools view_frames
ros2 topic echo --once /camera/aligned_depth_to_color/image_raw | grep encoding
ros2 topic list | grep -E "tf|tumbler|camera|yolo"
ros2 topic echo /jarvis/tumbler_dispenser/tumbler_pose
```


## Connection-ready tumbler transfer

When camera and robot are connected, the intended path is:

```text
YOLO model /home/ssu/Downloads/best.pt
-> yolo_tumbler_detector_node
-> /azas/cup_detection
-> cup_detection_pose_bridge_node
-> /jarvis/tumbler_dispenser/tumbler_pose
-> tumbler_floor_place_node
-> RG2 open/close + Doosan move_line when hardware gates are explicitly enabled
```

Build both workspaces first:

```bash
source /opt/ros/humble/setup.bash
cd /home/ssu/Azas && colcon build --symlink-install
cd /home/ssu/ros2_ws && colcon build --packages-select jarvis --symlink-install
```

Start RG2 services:

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
ros2 launch jarvis rg2_trigger.launch.py ip:=192.168.1.1
```

Start camera-driven dry-run transfer:

```bash
/home/ssu/Azas/tools/run_robot_dryrun.sh
```

Check whether the camera detector sees a cup or lid:

```bash
/home/ssu/Azas/tools/check_robot_detection.sh
```

Only add real motion gates after camera detection, RG2 services, emergency stop, workspace bounds, and operator clearance are confirmed:

```bash
/home/ssu/Azas/tools/run_robot_real.sh
```

Live YOLO requires `ultralytics` and `torch` in the active Python environment.

## OSS robot-control stack

The open-source integration path is tracked in:

- `docs/oss_robot_control_stack.md` - chosen stack, gates, and control path.
- `docs/field_control_runbook.md` - terminal-by-terminal field procedure from virtual Doosan to dry-run gates.
- `docs/simulation_and_connection_plan.md` - when to use simulation, camera-only checks, robot/RG2 no-motion checks, and real motion.
- `docs/rviz_simulation_verification_2026-05-11.md` - current RViz simulation and dry-run controller evidence without robot hardware.
- `docs/camera_connection_verification_2026-05-11.md` - current RealSense D435i connection, topic, depth, and detection evidence.
- `docs/full_cocktail_workflow_plan.md` - full STT/recipe/cup/lid/dispenser/shake/pour workflow split into milestones.
- `docs/dsr_deeptree_integration.md` - project demo source review and the Azas adapter surface.
- `dependencies/ros2_sources.repos` - ROS 2 source candidates for disposable review workspaces.
- `dependencies/dsr_deeptree_sources.repos` - pinned project demo source for review-only import.
- `dependencies/python_optional_requirements.txt` - YOLO, Grounded-SAM/LangSAM, and STT Python candidates.
- `tools/check_oss_stack.sh` - non-hardware readiness check for packages, launch files, and optional imports.
- `docs/control_readiness_audit.md` - current completion audit and remaining hardware gates.

Run the non-hardware check after building Azas and `ros2_ws`:

```bash
/home/ssu/Azas/tools/check_oss_stack.sh
```

Or run the full non-hardware verifier:

```bash
/home/ssu/Azas/tools/verify_control_readiness.sh
```

Warnings mean an optional runtime path is unavailable; failures mean the robot-control stack is not ready for dry-run.

Run the end-to-end non-hardware control smoke:

```bash
/home/ssu/Azas/tools/smoke_control_path.sh
```

This injects a fake `CupDetection`, verifies the pose bridge, and waits for the floor-place controller to publish `DONE` with `enable_hardware:=false`.

Run the fake-hardware service call smoke:

```bash
/home/ssu/Azas/tools/smoke_fake_hardware_path.sh
```

This verifies `enable_hardware:=true` against fake Doosan `MoveLine` and fake RG2 Trigger services only.

Current gripper service contract:

| Service name | Service type | Provider | Fake/dry-run | Real hardware | Current status | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| `/jarvis/rg2/open` | `std_srvs/srv/Trigger` | `jarvis` RG2 trigger launch or `tools/fake_hardware_services.py` | Yes when provided by fake services | Not proven by Azas gates | Expected by floor-place launch | Existence/type does not prove RG2 actuation. |
| `/jarvis/rg2/close` | `std_srvs/srv/Trigger` | `jarvis` RG2 trigger launch or `tools/fake_hardware_services.py` | Yes when provided by fake services | Not proven by Azas gates | Expected by floor-place launch | Existence/type does not prove RG2 actuation. |
| `/jarvis/rg2/set_width` | `azas_interfaces/srv/SetGripper` | `tools/fake_hardware_services.py` in no-motion smoke; real adapter pending | Yes in fake smoke | Not connected | Explicit adapter required for real width/force control | Fake service logs requests and does not command real RG2. |
| `/azas/gripper/open_close` | `azas_interfaces/srv/SetGripper` | `azas_gripper/rg2_gripper_node.py` | Placeholder only | No | Azas internal boundary | Not wired into `/jarvis/rg2/*`; not a real RG2 driver. |

Fake service pass is not real RG2 readiness. Real RG2 use still requires separate
hardware confirmation, operator clearance, unit/range verification, and a real
adapter that is not implemented here.

After starting the live dry-run bringup on the robot PC, check the field gates without commanding motion:

```bash
/home/ssu/Azas/tools/check_live_hardware_gates.sh
```

To decide what should be connected next without commanding motion:

```bash
/home/ssu/Azas/tools/check_connection_stage.sh
```

For the full field no-motion report before any real robot run:

```bash
/home/ssu/Azas/tools/field_no_motion_report.sh
```

To see exactly which measured calibration/safety values still block real motion:

```bash
/home/ssu/Azas/tools/real_motion_measurement_report.sh
```

Use `STRICT=true` when every optional warning, including detection and RG2 service availability, should fail the gate.

Before real robot motion, strict mode must pass and write the gate stamp accepted by `run_robot_real.sh`:

```bash
STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed /home/ssu/Azas/tools/check_live_hardware_gates.sh
```

To isolate camera depth projection:

```bash
/home/ssu/Azas/tools/check_depth_projection_sample.sh
```

For a DSR-inspired cocktail task sequence without robot motion:

```bash
ros2 launch azas_bringup cocktail_dryrun.launch.py
```

For a non-hardware smoke test with fake cup/lid detections and a two-dispenser recipe:

```bash
/home/ssu/Azas/tools/smoke_cocktail_dryrun_sequence.sh
```

For field execution order, follow:

```bash
/home/ssu/Azas/tools/run_doosan_virtual_m0609.sh
/home/ssu/Azas/tools/run_robot_dryrun.sh
STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed /home/ssu/Azas/tools/check_live_hardware_gates.sh
/home/ssu/Azas/tools/run_robot_real.sh
```

For the hardware connection decision, read `docs/simulation_and_connection_plan.md`. In short: do simulation/fake-hardware first, connect the camera before robot motion, then connect Doosan/RG2 for no-motion strict gates, and only then run real motion.
