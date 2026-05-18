# Full Cocktail Workflow Plan

Purpose: map the intended cocktail task into perception, planning, gripper, dispenser, and safety gates.

## User-Level Workflow

1. Receive speech input.
2. Parse the requested cocktail and choose a recipe.
3. Recognize the cup/tumbler and lid.
4. Pick the cup.
5. Move the cup under a fixed dispenser outlet.
6. Press/squeeze the dispenser so liquid falls vertically into the cup.
7. Repeat for each dispenser in the recipe.
8. Pick the lid.
9. Place/press the lid onto the cup.
10. Pick the closed cup and shake it.
11. Open/remove the lid.
12. Pour the cocktail into another cup.

## Current Verified Pieces

| Piece | Current evidence | Status |
| --- | --- | --- |
| STT/recipe boundary | `azas_voice` maps text into symbolic recipe/dispenser decisions | Skeleton only |
| Cocktail task sequence | `cocktail_dryrun_sequence_node` uses `cocktail_workflow_plan.py` to publish a no-motion plan with calibration, cup pick, dispenser alignment/press, lid, shake, open, and pour phases | Dry-run adapter with full phase gates |
| Camera hardware | RealSense D435i recognized by USB/V4L2 and ROS | Verified |
| Depth projection | center projection passed in `camera_color_optical_frame` | Verified |
| Lid detection | `check_detection_stability.sh --expect-class lid` saw 140/140 lid samples, confidence 0.972-0.975 | Verified for current view |
| Cup detection | YOLO model has class `cup`, but body-only sample not yet tested | Missing |
| Cup pose bridge | `/jarvis/tumbler_dispenser/tumbler_pose` publishes camera-frame pose from detection | Verified in camera frame |
| RViz dispenser/cup scene | fixed four-dispenser layout, target pose, path visualization | Verified |
| Floor-place dry-run | controller reaches `DONE` with hardware disabled | Verified |
| Fake Doosan/RG2 service path | fake services reach `DONE` | Verified |
| Real robot/RG2 service path | no real service evidence yet | Missing |
| Hand-eye/base transform | no measured `camera_color_optical_frame -> base_link` transform yet | Missing |
| Lid placement/press | `dispense_lid_sequence_node` dry-run/fake-service sequence approaches the placed cup and presses the lid down | Offline/fake-service verified |
| Dispenser pressing/squeezing | `dispense_lid_sequence_node` presses the selected fixed outlet from an extended +X pose so the arm is more stretched than the cup-placement pose; real execution still waits for measured press poses | Offline/fake-service verified, needs field calibration |
| Shaking | `tumbler_shake_sequence_node` runs a high safe-space shake path with min-height, workspace, and dispenser keepout checks; fake `MoveLine` smoke verifies the planned motion | Offline/fake-service verified, needs field validation |
| Lid removal/opening | no controller yet | Missing |
| Pour into another cup | no perception/target/motion primitive yet | Missing |

## DSR_DeepTree Application

The project demo repo was reviewed at commit `22f5435086037a759e563047f535f2c3c418351e`.
Its directly reusable pattern is the stepwise ROS 2 task execution loop: accept a symbolic command, publish each phase, and keep motion/gripper execution behind explicit controllers.

Applied in Azas:

```bash
ros2 launch azas_bringup cocktail_dryrun.launch.py
/home/ssu/Azas/tools/smoke/smoke_cocktail_dryrun_sequence.sh
```

This currently verifies orchestration only. It does not move the robot or actuate RG2/dispenser hardware.

As of 2026-05-12, the dry-run plan is no longer only symbolic. Each step
declares required inputs, produced state, command boundary, and hardware gate:

- `VERIFY_CALIBRATION` blocks on measured `calibration.yaml`, `safety.yaml`, TF,
  TCP-to-cup offset, and dispenser outlet poses.
- `TRANSFORM_CUP_TO_BASE` uses `cup_detection_pose_bridge_node`.
- `PICK_CUP`, `ALIGN_CUP_UNDER_DISPENSER`, `PRESS_DISPENSER`, `PICK_LID`,
  `PLACE_AND_PRESS_LID`, `SHAKE_CUP`, `OPEN_LID`, and `POUR` all require the
  strict live gate before real hardware execution.
- `tools/checks/check_cocktail_workflow_plan.py` verifies the phase order and strict
  hardware gates without ROS hardware.

## Control Architecture

The full system should remain split by responsibility:

- STT/LLM: choose recipe and ordered dispenser IDs only.
- Vision: publish cup and lid detections with camera-frame 3D poses.
- Calibration/TF: transform camera-frame detections into `base_link`.
- Task manager: sequence recipe, dispenser visits, lid, shake, and pour phases.
- Motion: plan robot movements from measured TF/config only.
- Gripper: open/close/force/width primitives with success/failure feedback.
- Dispenser actuator: press/squeeze fixed dispenser at calibrated outlet/press point.
- Safety gates: abort on missing detection, TF, calibration, workspace, gripper, planning, or operator clearance.

## Minimal Next Milestones

### Milestone A: Cup/Lid Vision Gate

Required before robot connection:

```bash
/home/ssu/Azas/tools/checks/check_detection_stability.sh --expect-class lid
/home/ssu/Azas/tools/checks/check_detection_stability.sh --expect-class cup
```

Acceptance:

- `lid` ratio >= 0.7 with stable depth.
- `cup` ratio >= 0.7 with stable depth.
- Both produce poses in `camera_color_optical_frame`.

### Milestone B: No-Motion Robot Service Gate

Required before any real motion:

```bash
STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed /home/ssu/Azas/tools/checks/check_live_hardware_gates.sh
```

Acceptance:

- camera topics pass,
- `detected:upright` cup sample exists,
- Doosan `move_line` and `move_joint` services exist,
- RG2 open/close services exist,
- measured calibration/safety config replaces placeholders.

### Milestone C: Single-Dispenser Cup Placement

Scope:

- pick cup,
- move under one fixed dispenser outlet,
- dry-run then fake-service then real low-speed execution.

Acceptance:

- MoveIt/service path reaches target without collision/workspace violation,
- cup mouth is under outlet within measured tolerance,
- no dispenser actuation yet.

### Milestone D: Dispenser Press Integration

Scope:

- press/squeeze one fixed dispenser while cup is held under the outlet,
- then repeat across recipe-ordered dispenser IDs.

Acceptance:

- each fixed dispenser ID has calibrated `dispenser_outlets.<id>.outlet_pose_*`
  and `dispenser_outlets.<id>.press_pose_*`,
- press action has timeout and failure behavior,
- liquid falls into cup in dry-run/manual supervised test.

Code-prepared state:

- `cocktail_workflow_plan.py` emits `ALIGN_CUP_UNDER_DISPENSER` followed by
  `PRESS_DISPENSER` for every `dispenser_id` in the recipe decision.
- `dispense_lid_sequence_node` provides a concrete no-motion/fake-service path:
  `extended_press_approach -> extended_press_top -> extended_press_down ->
  extended_press_retreat -> lid_close_approach -> lid_close_press ->
  lid_close_retreat`.
- The default press point is outlet XYZ plus `press_x_extension=0.08m`,
  giving a more extended arm posture for dispenser pressing.
- Real execution remains blocked until every selected dispenser ID resolves to
  measured outlet and press poses in `calibration.yaml`, and the strict live
  gate passes.

### Milestone E: Lid, Shake, Open, Pour

This is partly code-prepared now, but real execution should still wait until
cup placement and dispenser pressing are reliable.

Required new capabilities:

- lid pick pose and orientation,
- lid-on-cup placement/press primitive against measured cup/lid geometry,
- closed-cup grasp verification,
- lid removal primitive,
- second cup detection/target pose,
- pouring trajectory and spill safety rules.

Code-prepared state:

- `tumbler_shake_sequence_node` plans `shake_safe_approach ->
  shake_center_start -> bounded x/y shake cycles -> shake_safe_retreat`.
- Default safe shake center is `x=0.30, y=-0.28, z=0.32m`, away from the
  current fixed dispenser row, with `min_shake_z=0.25m` and
  `dispenser_keepout_radius=0.20m`.
- Tumbler shake remains covered by the cocktail dry-run plan and should move
  into Python-level tests instead of adding another shell wrapper.

## Current Recommendation

Do not connect the real robot for motion yet. The next concrete test is cup-body detection:

```bash
/home/ssu/Azas/tools/checks/check_detection_stability.sh --expect-class cup
```

If it passes, proceed to Doosan/RG2 no-motion service checks and measured calibration. If it fails, collect more cup-body images or tune the YOLO target/class confidence before attempting robot motion.

Offline verifier before reconnecting hardware:

```bash
/home/ssu/Azas/tools/checks/check_cocktail_workflow_plan.py
/home/ssu/Azas/tools/smoke/smoke_cocktail_dryrun_sequence.sh
/home/ssu/Azas/tools/checks/verify_control_readiness.sh
```
