# Azas Project Gap Audit

Updated: 2026-05-14

This audit records developer- and project-level gaps that can be improved without inventing robot coordinates, calibration values, trajectories, or safety approvals.

## Critical Gaps

1. `mvp_bringup.launch.py` must launch the detection-to-pose bridge.
   - Evidence: `PickAndAlignActionServer` waits on `/jarvis/tumbler_dispenser/tumbler_pose`, and `cup_detection_pose_bridge_node` is the in-repo producer for that topic.
   - Current mitigation: `mvp_bringup.launch.py` now launches `cup_detection_pose_bridge_node` beside `yolo_tumbler_detector_node`.

2. RealSense topic defaults must stay consistent across docs, launch files, and detector code.
   - Evidence: agent/project docs describe `/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`, and `/camera/camera/color/camera_info`.
   - Current mitigation: detector and `yolo_perception.launch.py` defaults now match that documented RealSense namespace.

3. Core no-hardware logic needs regression tests beyond voice parsing.
   - Evidence: previous tests covered `azas_voice` only, while depth projection and side-grasp planning are control-critical no-hardware logic.
   - Current mitigation: added tests for depth projection, bbox orientation policy, detection selection tie-breaking, side-grasp offsets, z bounds, and observe-pose quaternion normalization.

4. Repository structure must be understandable without reverse-engineering every script.
   - Evidence: ROS node, launch, run, check, smoke, and experimental helper files are intentionally split, but the split was not documented file-by-file.
   - Current mitigation: `docs/repository_file_map.md` now describes each source/tool/documentation file group, its status, and whether it can affect real hardware.

5. Real-robot testing needs a smaller ladder than full cocktail execution.
   - Evidence: full workflow requires measured calibration, TF, RG2, MoveIt execution, and collision/workspace checks; jumping straight to full cocktail hides the first failing boundary.
   - Current mitigation: `tools/run/run_real_robot_test_ladder.sh` and `docs/real_robot_test_ladder.md` provide staged status, no-hardware, field, live-gate, observe-dry, pick-dry, and explicitly confirmed one-shot `pick-real` stages.

6. Cup placement and dispenser press must be wired as one gated field path, not two disconnected demos.
   - Evidence: floor-place could align to a selected outlet, and dispense/lid could press a dispenser, but the real entrypoint did not run them as one guarded sequence.
   - Current mitigation: `tools/run/run_cup_to_dispenser_press_real.sh` now runs live detection/side-grasp/outlet placement followed by press-only `dispense_lid_sequence`, and `smoke_cup_to_dispenser_press_path.sh` verifies the press primitive against fake MoveLine services.

7. Fixed dispenser geometry must not drift across launch files, node defaults, and measured config.
   - Evidence: older defaults existed in Jarvis node bodies and shake keepout while updated launch files used the newer outlet row.
   - Current mitigation: Jarvis node defaults, scene/floor/press/shake launch geometry, `check_fixed_dispenser_geometry.py`, and `check_measured_dispenser_geometry.py` now check the same geometry and fail closed if measured config disagrees.

## Remaining Gaps

1. `PickAndAlign.action` exposes `execute_motion`, but the action server still only supports `no_motion` and `skeleton` execution modes.
   - Do not enable real motion until measured calibration, workspace bounds, RG2 behavior, collision checks, and operator gates are complete.

2. Gripper service boundaries remain split.
   - `azas_gripper` provides `/azas/gripper/open_close` with `SetGripper`.
   - the task manager expects `/jarvis/rg2/open` and `/jarvis/rg2/close` with `Trigger` for the fake/field path.
   - No-motion checks may verify service presence/type, but they must not be treated as real RG2 actuation evidence.
   - Add an explicit adapter or choose one service contract before motion execution work.

3. Motion-facing cup detection now requires the upright contract.
   - `yolo_tumbler_detector_node` publishes `detected:upright ...` only after the bbox upright heuristic passes.
   - `cup_detection_pose_bridge_node` refuses non-upright statuses and does not publish `/jarvis/tumbler_dispenser/tumbler_pose` for `detected:lid`, `rejected:*`, or ambiguous cup states.

4. Calibration persistence is intentionally not implemented.
   - `calibration_loader_node` exposes service boundaries but does not save measured values.
   - Keep `calibration.yaml` fail-closed until real measured values exist.

5. CI is now present, but only as a baseline.
   - `.github/workflows/ci.yml` runs ROS 2 Humble dependency install, `colcon build`, `colcon test`, and `colcon test-result`.
   - It does not yet run hardware-free smoke scripts such as `verify_control_readiness.sh`; add those after making host-specific Doosan/Jarvis assumptions portable in CI.

6. Measured calibration is still not auto-loaded into motion launch parameters.
   - Current behavior is intentionally conservative: real-motion config now fails if measured outlet/press geometry does not match the currently launched geometry.
   - Better long-term fix: generate the Jarvis launch parameters directly from `calibration.yaml` instead of comparing against fixed values.
