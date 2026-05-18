# Control Readiness Audit

Objective: find and integrate open-source components enough for the Azas cocktail robot to reach a robot-control-ready state.

Status on 2026-05-12: **not complete for real hardware**, but the non-hardware OSS control path and full cocktail workflow plan are wired and smoke-tested.

Latest verifier evidence: `/home/ssu/Azas/tools/checks/verify_control_readiness.sh` passed at `2026-05-12T12:56:22+09:00`, including OSS stack checks, non-hardware control smoke, fake hardware-armed smoke, cocktail dry-run sequence smoke, full cocktail workflow plan gate, real-motion entrypoint fail-closed smoke, config-gate smoke, and Doosan virtual launch argument resolution.

Power-off recovery: `docs/recovery_after_poweroff.md` and `docs/current_handoff_2026-05-11.md`.

## Prompt-To-Artifact Checklist

| Requirement | Artifact / command | Evidence | Status |
| --- | --- | --- | --- |
| Open-source candidates selected for the cocktail robot | `wiki/syntheses/Open Source Dependency Candidates.md`, `docs/oss_robot_control_stack.md` | ROS 2 Humble, Doosan `doosan-robot2`, MoveIt 2/MoveItPy, RealSense ROS 2, YOLO, Grounded-SAM/LangSAM, AprilTag/easy_handeye2, SpeechRecognition/Vosk/Whisper mapped to Azas roles | Done |
| External sources tracked without vendoring | `dependencies/ros2_sources.repos`, `dependencies/experimental_sources.repos`, `dependencies/README.md` | Review workspace flow documented; external source trees explicitly not copied into Azas | Done |
| Runtime install candidates documented | `dependencies/system_apt_packages.txt`, `dependencies/python_optional_requirements.txt` | ROS/camera/STT system package candidates and optional Python runtime candidates listed | Done |
| Non-hardware verifier bundled | `tools/checks/verify_control_readiness.sh` | Runs syntax checks, OSS stack availability, control smoke, fake hardware smoke, DSR-inspired cocktail dry-run sequence smoke, full cocktail workflow plan gate, and Doosan launch arg check into `/tmp/azas_control_readiness_report.txt` | Done |
| Azas core packages build | `source /opt/ros/humble/setup.bash; cd /home/ssu/Azas; colcon build --symlink-install` | 8 packages finished | Done |
| Azas tests do not fail | `colcon test --packages-select azas_motion azas_perception azas_voice --event-handlers console_direct+; colcon test-result --verbose` | 2026-05-14: 18 tests, 0 errors, 0 failures, 0 skipped. Coverage now includes voice parser/sanitizer, depth projection, bbox orientation policy, detection selection, side-grasp offsets, z bounds, and observe quaternion normalization. | Done for no-hardware core logic; broader ROS launch/smoke coverage still needed |
| Doosan/MoveIt stack is locally available | `/home/ssu/Azas/tools/checks/check_oss_stack.sh` | OK for `dsr_bringup2`, `dsr_moveit_config_m0609`, `moveit_py` | Done |
| RealSense package available | `/home/ssu/Azas/tools/checks/check_oss_stack.sh` | OK for `realsense2_camera` | Done |
| YOLO MVP runtime available | `/home/ssu/Azas/tools/checks/check_oss_stack.sh` | OK for Ultralytics import and `/home/ssu/Downloads/best.pt` | Done, license review still required |
| Optional fallback/STT candidates checked | `/home/ssu/Azas/tools/checks/check_oss_stack.sh` | LangSAM, Vosk, Whisper warnings because not installed | Not complete, optional |
| Launch wiring resolves without hardware | `/home/ssu/Azas/tools/checks/check_oss_stack.sh` | OK for Doosan virtual MoveIt launch, `robot_connection_control.launch.py`, `yolo_to_floor_place.launch.py` | Done |
| Detection-to-control contract works without hardware | `/home/ssu/Azas/tools/smoke/smoke_control_path.sh` | Fake `CupDetection` reaches floor-place `DONE` with `enable_hardware:=false` | Done |
| Pick action side-grasp no-motion sequence works | `/home/ssu/Azas/tools/smoke/smoke_pick_and_align_no_motion.sh` | Fake base-link `PoseStamped` can drive `/azas/pick_and_align` through `WAIT_TUMBLER_POSE`, `COMPUTE_SIDE_GRASP`, side approach/pick/lift no-motion feedback, `DONE_NO_MOTION`, and `NO_MOTION_SIDE_GRASP_OK` without MoveIt, Doosan, or real RG2 commands. This is a no-motion approximation only: the TCP quaternion is a planning-only parameter candidate and is not real readiness. Live perception must publish a cup status starting with `detected:upright`; `detected:lid`, lying/ambiguous cup detections, and rejected statuses fail closed with `CUP_ORIENTATION_NOT_UPRIGHT` or `CUP_ORIENTATION_UNKNOWN`. | Done |
| Observe-cup supervised stage exists | `/home/ssu/Azas/tools/pick/run_supervised_real_single_cup_pick.py`, `/home/ssu/Azas/tools/pick/run_supervised_real_single_cup_pick.py`, `/home/ssu/Azas/tools/checks/check_observe_pose_planning_only.sh` | The supervised entrypoint now starts from HOME `j1=0, j2=0, j3=90, j4=0, j5=90, j6=0`, then uses a joint observe candidate `j1=0, j2=25, j3=65, j4=0, j5=135, j6=0`. Real observe motion is allowed only through the one-shot explicit confirmation gate and `/execute_trajectory`. Observe-only mode does not call RG2. The no-motion action still reports `PLAN_OBSERVE_CUP_POSE_NO_MOTION` and `DETECT_CUP_PENDING` before waiting for `/jarvis/tumbler_dispenser/tumbler_pose`. | Supervised observe implemented; pose still operator-tuned |
| Side-grasp planning-only MoveItPy request boundary exists | `/home/ssu/Azas/tools/checks/check_side_grasp_planning_only.sh`, `azas_motion/alignment_executor_node.py` | Static gate confirms `allow_execute=false` default and no execution call in Azas motion/task-manager code. Runtime node fails closed when `planning_group`/`ee_link` are empty or MoveItPy is unavailable. With verified planning params, it constructs MoveItPy planning requests for approach/grasp/lift and calls `PlanningComponent.plan()` only. Planning-only is trajectory feasibility reporting, not real side grasp readiness. Real side grasp still requires measured hand-eye/base-camera TF, verified cup center/radius, table height, TCP quaternion, gripper width/force, collision scene/clearance, operator clearance, and e-stop confirmation. | Planning-only request path connected, no execution |
| Side-grasp candidate sweep available | `/home/ssu/Azas/tools/pick/sweep_side_grasp_planning_candidates.py`, `/home/ssu/Azas/tools/pick/sweep_side_grasp_planning_candidates.py` | Sweeps axis, grasp-height offset, and TCP quaternion candidates with MoveItPy `PlanningComponent.plan()` only, writes `/tmp/azas_side_grasp_candidate_sweep.json` and `.csv`, and never calls trajectory execution. `tool0` is the preferred TCP candidate for this sweep, but final TCP remains a measured hardware value. Root-level script is a compatibility wrapper. | Planning-only diagnostic only |
| Hardware-gated service path works against fakes | `tools/smoke/fake_hardware_services.py`, `tools/smoke/smoke_fake_hardware_path.sh`, `/home/ssu/ros2_ws/src/Azas/jarvis/tumbler_floor_place_node.py` | Initial smoke exposed a service future deadlock; fixed by running `run_once` outside the timer callback and waiting while the executor remains free. Rebuilt `jarvis`; fake hardware path now reaches `DONE`. | Done |
| Live hardware field gates scripted | `/home/ssu/Azas/tools/checks/check_live_hardware_gates.sh` | Latest strict run fails safely: missing live camera topics, Doosan MoveLine/MoveJoint services, RG2 services, and measured calibration/safety config. `STRICT=true` writes the only stamp accepted by the real-motion entrypoint, but no stamp is written while failures remain. | Done |
| Field no-motion report scripted | `/home/ssu/Azas/tools/run/field_no_motion_report.sh` | Aggregates connection stage, depth projection, optional cup/lid stability, and live hardware gate into `/tmp/azas_field_no_motion_report.txt` without motion or gripper calls | Done |
| Robot connection acceptance wrapper scripted | `/home/ssu/Azas/tools/checks/robot_connection_acceptance.sh` | Runs the strict field no-motion report and hand-eye readiness check as one post-connection command without motion or gripper calls. | Done |
| Cup/lid operator sequence scripted | `/home/ssu/Azas/tools/checks/check_cup_lid_sequence.sh` | Prompts for lid then cup/tumbler-body placement and runs no-motion stability checks on `/azas/cup_detection` | Done, needs live rerun with objects |
| Hand-eye readiness helper scripted | `/home/ssu/Azas/tools/checks/check_hand_eye_readiness.sh` | Checks camera topics, CameraInfo frame, TF topics, and base-to-camera transform evidence without motion or gripper calls | Done, currently fails until camera/TF are running |
| Service contract type gates scripted | `/home/ssu/Azas/tools/checks/check_live_hardware_gates.sh`, `/home/ssu/Azas/tools/checks/check_connection_stage.sh` | Gate now checks Doosan `MoveLine`/`MoveJoint` service types and RG2 `Trigger` service types without calling them. Fake service contract check observed `dsr_msgs2/srv/MoveLine`, `dsr_msgs2/srv/MoveJoint`, and `std_srvs/srv/Trigger` when fake services were present. | Done |
| Real-motion measurement worksheet available | `docs/real_motion_measurement_worksheet.md` | Lists every measured value required before `calibration.yaml`/`safety.yaml` can pass the real-motion config gate | Done |
| Real-motion entrypoint fails closed | `tools/run/run_robot_real.sh`, `tools/smoke/smoke_real_motion_entrypoint_gates.sh` | `run_robot_real.sh` now re-runs the measured config gate after validating the strict stamp. Smoke verifies refusal for missing stamp, non-strict stamp, and placeholder config. | Done |
| Real-motion config gate regression tested | `tools/checks/check_real_motion_config.sh`, `tools/smoke/smoke_real_motion_config_gate.sh` | Smoke verifies placeholder config is blocked and measured-like temporary fixture config passes, without changing production calibration/safety files. | Done |
| Power-off recovery documented | `docs/recovery_after_poweroff.md` | Reboot commands, safe resume checks, and current blockers are available | Done |
| Real-motion calibration/safety config gate scripted | `/home/ssu/Azas/tools/checks/check_real_motion_config.sh` | Current placeholder configs fail closed on `null` and `확인 필요`, while preserving velocity/acceleration and failure-behavior checks | Done |
| Field execution order documented | `docs/field_control_runbook.md`, `tools/run/run_doosan_virtual_m0609.sh` | Virtual Doosan, Azas dry-run, and live gate terminal sequence documented | Done |
| Simulation vs hardware connection order documented | `docs/simulation_and_connection_plan.md` | Recommends non-hardware, camera-only, robot/RG2 no-motion, then real-motion stages | Done |
| Current connection-stage report available | `/home/ssu/Azas/tools/checks/check_connection_stage.sh` | No-motion report maps current ROS graph/config state to next connection step | Done |
| Project demo source reviewed and pinned | `dependencies/dsr_deeptree_sources.repos`, `docs/dsr_deeptree_integration.md` | `deeptree0819/DSR_DeepTree` pinned at commit `22f5435086037a759e563047f535f2c3c418351e`; reusable Task 1/2/3/4 patterns mapped without vendoring the repo | Done |
| DSR-inspired cocktail sequence applied | `azas_task_manager/cocktail_workflow_plan.py`, `azas_task_manager/cocktail_dryrun_sequence_node.py`, `azas_bringup/launch/cocktail_dryrun.launch.py`, `tools/smoke/smoke_cocktail_dryrun_sequence.sh`, `tools/checks/check_cocktail_workflow_plan.py` | Fake cup/lid detections plus a two-dispenser recipe publish a no-motion workflow with calibration, camera-to-base transform, cup pick, per-dispenser alignment/press, lid placement, shake, open, and pour phases. Hardware-capable phases are marked `strict_live_gate`. | Done |
| RViz simulation runs without robot | `docs/rviz_simulation_verification_2026-05-11.md`, `ros2 launch jarvis tumbler_dispenser_scene.launch.py selected_dispenser_id:=2 use_rviz:=true` | Scene node and RViz start; target/tumbler/path topics publish; floor-place dry-run consumes simulated pose and reaches `DONE` with `enable_hardware:=false` | Done |
| Real camera hardware recognized | `docs/camera_connection_verification_2026-05-11.md`, `lsusb`, `v4l2-ctl --list-devices` | Intel RealSense D435i detected on USB and `/dev/video0` through `/dev/video5` | Done |
| Real camera topics and depth projection verified | `tools/checks/check_depth_projection_sample.sh`, `docs/camera_connection_verification_2026-05-11.md` | RealSense publishes color, aligned depth, CameraInfo; center depth projection passed at `0.3100m` in `camera_color_optical_frame` | Done |
| Real camera lid detection verified | `/home/ssu/Azas/tools/checks/check_robot_detection.sh`, `/home/ssu/Azas/tools/checks/check_connection_stage.sh`, `docs/camera_connection_verification_2026-05-11.md` | Model classes are `{0: cup, 1: lid}`; repeated fresh samples report `detected:lid` with confidence around `0.972` and `depth_raw=264.0`. This is lid evidence only and does not satisfy the motion-facing cup pose contract. | Done |
| Lid detection stability verified | `/home/ssu/Azas/tools/checks/check_detection_stability.sh --expect-class lid` | 140/140 samples were `lid`; confidence `0.972-0.975`; `depth_raw=263.0-265.0` | Done |
| Real camera cup/tumbler-body detection verified | `/home/ssu/Azas/tools/checks/check_robot_detection.sh`, `/home/ssu/Azas/tools/checks/check_connection_stage.sh` | Not yet sampled with body-only/visible cup condition after lid test. Required evidence is a status starting with `detected:upright` plus a real-camera `/jarvis/tumbler_dispenser/tumbler_pose` after TF conversion. | Missing |
| Full cocktail workflow decomposed | `docs/full_cocktail_workflow_plan.md`, `azas_task_manager/cocktail_workflow_plan.py` | STT, recipe sequencing, cup/lid perception, measured calibration, fixed dispenser placement, dispenser pressing, lid, shaking, opening, and pouring are split into executable no-motion plan phases with explicit hardware gates | Done for offline plan, needs field calibration/execution |
| Hand-eye/base-camera transform verified | AprilTag/easy_handeye2 workflow | No current measurement evidence | Missing |
| MoveIt feasibility for detected poses verified | Doosan virtual/emulator plus Azas motion path | Doosan launch resolves; fake Doosan service path executes request shape, but no real MoveIt planning result for measured poses yet | Missing |
| RG2 actuation verified | `tools/run/run_robot_dryrun.sh`, `tools/run/run_robot_real.sh`, `jarvis` RG2 services | Fake RG2 Trigger service path works; real RG2 hardware/fake-driver feedback still not field-verified | Missing |
| Real robot hardware gate verified | `tools/run/run_robot_real.sh`, `docs/safety_checklist.md` | Script requires a recent strict live-gate stamp plus explicit phrase; e-stop/workspace/operator gates not field-verified | Missing |

## Gripper Service Contract

| Service name | Service type | Provider | Fake/dry-run 여부 | Real hardware 여부 | Current status | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| `/jarvis/rg2/open` | `std_srvs/srv/Trigger` | `jarvis` RG2 trigger launch or `tools/smoke/fake_hardware_services.py` | Yes when fake services are started | Not proven by gate | Floor-place default open service | Presence/type only; not real actuation evidence. |
| `/jarvis/rg2/close` | `std_srvs/srv/Trigger` | `jarvis` RG2 trigger launch or `tools/smoke/fake_hardware_services.py` | Yes when fake services are started | Not proven by gate | Floor-place default close service | Presence/type only; not real actuation evidence. |
| `/jarvis/rg2/set_width` | `azas_interfaces/srv/SetGripper` | `tools/smoke/fake_hardware_services.py`; real adapter pending | Yes in fake smoke | No real adapter in Azas | Checked as service shape only | Width/force command logging in fake mode does not prove RG2 units or hardware response. |
| `/azas/gripper/open_close` | `azas_interfaces/srv/SetGripper` | `src/azas_gripper/azas_gripper/rg2_gripper_node.py` | Placeholder boundary | No | Internal Azas service | Separate from `/jarvis/rg2/*`; not used as the floor-place gripper adapter. |

`tools/checks/check_live_hardware_gates.sh` checks service existence and type only. It
does not call RG2 services and does not prove open/close/set-width movement.
Fake smoke tests must not be interpreted as real RG2 readiness.

## Current Stop Condition

Do not claim real robot-control readiness until these are verified on the robot PC:

1. RealSense publishes color, aligned depth, and camera info with expected frame IDs.
2. YOLO `best.pt` publishes `detected:upright ...` on `/azas/cup_detection` for the actual tumbler body; lid-only evidence is not a cup pose.
3. Depth projection is validated with a known-distance target.
4. Camera-to-base or hand-eye TF is measured and stable.
5. `/jarvis/tumbler_dispenser/tumbler_pose` is produced from real camera detection, not the smoke publisher.
6. Doosan virtual/emulator motion accepts the selected M0609 plan without workspace/collision failure.
7. RG2 open/close service behavior, width/force, timeout, and failure handling are verified.
8. Real motion is attempted only through `tools/run/run_robot_real.sh` after the full safety checklist is signed off and a recent strict live-gate stamp exists.
9. `tools/checks/check_real_motion_config.sh` passes with measured, non-placeholder calibration and safety values.

Field command before real motion:

```bash
STRICT=true GATE_STAMP=/tmp/azas_live_hardware_gates_passed /home/ssu/Azas/tools/checks/check_live_hardware_gates.sh
```

Current no-bringup evidence: the gate checker fails closed because camera topics, Doosan motion services, and RG2 services are not running in this session. `run_robot_real.sh` also refuses to proceed when the strict gate stamp is missing.
