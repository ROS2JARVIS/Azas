# DSR_DeepTree Integration Note

Source reviewed: `https://github.com/deeptree0819/DSR_DeepTree`, commit `22f5435086037a759e563047f535f2c3c418351e`.

## What Is Reusable

The repo is useful as a Doosan M0609 + RG2 ROS 2 demonstration, not as a direct vendored dependency.

Applicable pieces:

- Task 1 `pick_place_server.py`: stepwise action execution with HOME, APPROACH, PICK, RETREAT, TRANSFER, PLACE feedback.
- Task 1 `robot_controller.py`: MoveItPy ownership, single-threaded motion serialization, RG2 service pattern, workspace bounds.
- Task 2 `_perception.py`: YOLO detection, depth projection, hand-eye transform, camera-to-base conversion pattern.
- Task 3 `nlp_node.py`: `/stt_result` input, keyword parsing, command publication, TTS confirmation pattern.
- Task 4 Isaac Sim assets: useful later for M0609/RG2 simulation review after Azas dry-run gates are stable.

## What Was Applied Now

Azas now has a no-motion cocktail sequence adapter:

- `azas_task_manager/cocktail_dryrun_sequence_node.py`
- launch: `ros2 launch azas_bringup cocktail_dryrun.launch.py`
- smoke test: `/home/ssu/Azas/tools/smoke/smoke_cocktail_dryrun_sequence.sh`

The node applies DSR_DeepTree's stepwise feedback idea to the Azas workflow:

```text
recipe decision
-> verify fresh cup/lid detections
-> pick cup
-> move/press each recipe dispenser
-> pick/place lid
-> shake
-> open lid
-> pour
```

It publishes JSON status only on `/azas/cocktail/status` and a JSON plan on `/azas/cocktail/task_plan`. It sends no robot, RG2, MoveIt, or dispenser commands.

## Runtime Contract

Input topics:

- `/azas/voice/recipe_decision` (`std_msgs/String`, JSON from `azas_voice`)
- `/azas/cup_detection` (`azas_interfaces/CupDetection`)

Output topics:

- `/azas/cocktail/task_plan` (`std_msgs/String`, JSON)
- `/azas/cocktail/status` (`std_msgs/String`, JSON)

Blocking behavior:

- invalid or non-executable recipe decision blocks,
- recipe without `dispenser_ids` blocks,
- missing fresh `cup` or `lid` detection blocks by default,
- hardware motion is intentionally impossible through this node.

## Review Import

Keep the demo repo outside the Azas source tree:

```bash
mkdir -p /tmp/azas_demo_review/src
cd /tmp/azas_demo_review
vcs import src < /home/ssu/Azas/dependencies/dsr_deeptree_sources.repos
```

Do not copy the full DSR_DeepTree tree into `/home/ssu/Azas`. If a pattern is reused, port only the smallest adapter and document safety behavior in this file.

## Next Application Points

1. Replace symbolic dispenser IDs with the confirmed 16-recipe catalog.
2. Add a hardware-gated executor behind the dry-run sequence after measured TF and service gates pass.
3. Port the DSR single-thread MoveIt execution ownership pattern into `azas_motion`.
4. Port camera-to-base hand-eye transform structure after calibration data exists.
5. Add a dispenser actuator primitive with timeout, force/speed limits, and fail-closed status.

## Safety Boundary

DSR_DeepTree contains real motion and RG2 patterns. Azas must not enable those paths until:

- cup and lid detection pass stability checks,
- `camera_color_optical_frame -> base_link` transform is measured,
- Doosan and RG2 services pass no-motion strict gates,
- workspace bounds and speed/force limits replace placeholders,
- an operator clearance and emergency-stop procedure are documented.
