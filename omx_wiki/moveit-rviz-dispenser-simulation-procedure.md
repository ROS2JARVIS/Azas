---
title: "MoveIt RViz dispenser simulation procedure"
tags: ["rviz", "moveit", "simulation", "dispenser", "front-hold", "press", "tcp"]
created: 2026-06-01T09:00:02.895Z
updated: 2026-06-01T09:00:02.895Z
sources: []
links: []
category: reference
confidence: medium
schemaVersion: 1
---

# MoveIt RViz dispenser simulation procedure

# MoveIt/RViz simulation procedure for dispenser front-hold, press, and color-scan

## Purpose
Use this procedure when the operator asks to “show the orange/MoveIt robot”, “Plan & Execute”, “cup in front of dispenser”, “press dispenser”, or “camera pose toward dispenser” in RViz.

This is a **simulation/visual validation procedure**. Do not invent real cup coordinates. Real cup pose must still come from `/jarvis/tumbler_dispenser/tumbler_pose`; measured dispenser poses come only from repository calibration/config files.

## Correct baseline stack
Do **not** start the shake preview or `rviz_only` when the user expects the teaching-material MoveIt UI.

Start the full virtual MoveIt stack:

```bash
set +u
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
source /home/ssu/Azas/install/local_setup.bash
set -u
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  mode:=virtual model:=m0609 host:=127.0.0.1 color:=white
```

Expected verification:

```bash
ros2 node list | grep -E 'move_group|robot_state_publisher|joint_state_broadcaster|rviz2|virtual_node'
ros2 action list | grep -E '/move_action|/execute_trajectory|follow_joint_trajectory'
ros2 topic list | grep rviz_moveit_motion_planning_display
```

Good signs:
- `/move_group` exists.
- `/move_action` and `/execute_trajectory` exist.
- `/dsr_moveit_controller/follow_joint_trajectory` exists.
- RViz log contains `Ready to take commands for planning group manipulator`.
- `/tf` contains `base_link -> ... -> link_6` from `robot_state_publisher`.

## RViz config pitfalls
The installed M0609 MoveIt RViz config may hide the interactive end-effector marker:

```yaml
Interactive Marker Size: 0
```

If the link_6/goal handle is not visible, patch or use a copy of:

```text
/home/ssu/ros2_ws/install/dsr_moveit_config_m0609/share/dsr_moveit_config_m0609/launch/moveit.rviz
```

Set:

```yaml
Interactive Marker Size: 0.25
```

The visible “orange robot” in operator language usually means the MoveIt MotionPlanning UI with the end-effector interactive marker and Plan/Execute controls, not literally an orange mesh.

## What actually moves in simulation
Doosan virtual services move the controller state. RViz follows only if the chain is intact:

```bash
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link link_6
```

If `/joint_states` changes but RViz does not visually move, inspect:
- `/tf` frame availability.
- RViz fixed frame (`base_link` or `world`).
- RobotModel/MotionPlanning display enabled.
- Scene Robot visual enabled.

A large temporary marker on `link_6` is useful for visibility, but it is diagnostic only; do not confuse it with real TCP calibration.

## Show camera/color-scan pose
The dispenser-facing camera/color scan pose is a measured joint pose in:

```text
src/azas_bringup/config/dispenser_color_scan.yaml
```

Dry-run:

```bash
python3 tools/run/move_to_dispenser_color_scan_pose.py --service-prefix ''
```

Visual/sim execution:

```bash
python3 tools/run/move_to_dispenser_color_scan_pose.py \
  --service-prefix '' \
  --no-check-current-tcp \
  --execute \
  --confirm ENABLE_DISPENSER_COLOR_SCAN_MOVE
```

`--no-check-current-tcp` is for virtual/RViz viewing when the simulated starting TCP is outside the real safety envelope. Do not use it as a real robot bypass.

## Show cup placement / front-hold pose
Measured front-hold poses are in:

```text
src/azas_bringup/config/measured_dispenser_collision.yaml
```

Dry-run dispenser 1:

```bash
python3 tools/run/move_to_measured_dispenser_front_hold.py \
  --service-prefix '' \
  --dispenser-id 1 \
  --config src/azas_bringup/config/measured_dispenser_collision.yaml \
  --no-verify-target
```

Visual/sim execution for a dispenser front-hold:

```bash
python3 tools/run/move_to_measured_dispenser_front_hold.py \
  --service-prefix '' \
  --dispenser-id 1 \
  --config src/azas_bringup/config/measured_dispenser_collision.yaml \
  --velocity 15 \
  --acceleration 20 \
  --timeout-sec 120 \
  --wait-service-sec 8 \
  --no-compensate-current-tcp \
  --no-moveit-planning-guard \
  --no-verify-target \
  --execute \
  --confirm ENABLE_MEASURED_DISPENSER_FRONT_HOLD
```

To show an approach above the placement pose, use a measured target with a staging offset:

```bash
--target-offset-z-m 0.120
```

Then run again with `--target-offset-z-m 0.000` for the actual front-hold/cup-place pose.

## Show dispenser press pose
Measured press top poses are in:

```text
src/azas_bringup/config/measured_dispenser_press.yaml
```

The existing production press primitive is:

```bash
ros2 run azas_dispenser dispenser_press_node --ros-args \
  -p use_taught_posx:=true \
  -p taught_posx_config_path:=/home/ssu/Azas/src/azas_bringup/config/measured_dispenser_press.yaml \
  -p target_slot:=1 \
  -p target_dispenser:=red \
  -p tcp_name:=GripperDA_v1_jarvis \
  -p require_tcp_for_taught_posx:=false \
  -p allow_tcp_set_failure:=true
```

For visual-only press demonstration, show this sequence per slot:
- approach above: `top_posx.z + 100mm`
- top: `top_posx.z`
- press down: `top_posx.z - 40mm`
- retreat above: `top_posx.z + 100mm`

Use `tools/run/direct_movel_xyz.py` with the measured `top_posx_mm_deg` values; do not synthesize new dispenser coordinates.

## Safety workspace visualization
The operator expects floor + four side walls, **not a ceiling**. If a visualization helper is used, do not add a z-max lid/ceiling. The valid conceptual objects are:
- `safety_floor`
- `safety_x_min_wall`
- `safety_x_max_wall`
- `safety_y_min_wall`
- `safety_y_max_wall`

Bounds come from:

```text
src/azas_bringup/config/safety.yaml
motion.workspace_bounds_m
```

## Known distinction: visualization vs production logic
- Production recipe: `tools/run/run_measured_dispenser_recipe_sequence.py`.
- Front-hold primitive: `tools/run/move_to_measured_dispenser_front_hold.py`.
- Press primitive: `src/azas_dispenser/azas_dispenser/dispenser_press_node.py`.
- Color scan primitive: `tools/run/move_to_dispenser_color_scan_pose.py`.

Temporary RViz marker scripts under `/tmp` are diagnostics only and should not be treated as product logic.

## Common mistakes from the June 2026 debugging session
- Starting `tumbler_dispenser_then_shake_demo` is wrong for front-hold/press visualization; it shows shake logic.
- `doosan_moveit_rviz_only.launch.py` is insufficient when the user expects Plan & Execute; use the full `dsr_bringup2_moveit` stack.
- A one-shot motion can be missed visually; use a repeated or staged sequence if the operator asks to see motion.
- If movement is accepted by service but not visible, verify `/joint_states`, `/tf`, and RViz display settings before claiming RViz is wrong.
- Do not ask for cup coordinates or invent them. Cup pose still belongs to the perception pipeline.
