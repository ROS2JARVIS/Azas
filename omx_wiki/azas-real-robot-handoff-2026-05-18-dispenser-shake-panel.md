---
title: "Azas Real Robot Handoff 2026-05-18 Dispenser Shake Panel"
tags: ["azas", "real-robot", "handoff", "dispenser", "shake", "panel", "safety"]
created: 2026-05-18T10:06:59.396Z
updated: 2026-05-19T13:54:12+09:00
sources: []
links: []
category: session-log
confidence: medium
schemaVersion: 1
---

# Azas Real Robot Handoff 2026-05-18 Dispenser Shake Panel

# Azas real robot handoff — dispenser / shake / panel (2026-05-18)

## Current repo / runtime state

- Workspace: `/home/ssu/Azas`
- Branch: `integration/dispenser-feature-trial-20260518`
- Purpose of branch: trial integration of GitHub `feature/dispenser` / PR #3 without merging into the user's main working branch.
- Panel URL: `http://127.0.0.1:8765/`
- Panel server process observed at handoff: PID `310448`, command `python3 tools/run/robot_pipeline_control_server.py`.
- Doosan bringup process observed at handoff: PID `321933`, command includes `dsr_bringup2_moveit.launch.py name:=dsr01 host:=192.168.1.100 rt_host:=192.168.1.101 mode:=real model:=m0609`.
- Latest commits on this branch:
  - `1da2078 Require TCP selection for dispenser press`
  - `b5314ba Use measured camera table view joints`
  - `29b2f14 Show panel step outcomes inline`
  - `6a18445 Keep live shake J5-gated while wiring dispenser trial`
  - `ed603af Merge remote-tracking branch 'origin/feature/dispenser' into integration/dispenser-feature-trial-20260518`

## Hard safety stop

Do **not** run real robot motion automatically from an agent. The user is testing on a real Doosan M0609 + RG2 system. Only edit/build/check code unless explicitly asked for a command preview. The latest user feedback says the shake behavior was dangerous: base/link motion went downward/unsafe and could damage the robot.

## User's latest explicit requirements

1. **Shake must be redesigned in code, not only by changing shell/panel command arguments.**
   - User specifically asked whether the shake code was changed or only commands were changed and said that command-only tuning is unacceptable.
2. **Shake must not use the bad base-link/Cartesian behavior that sends the robot down.**
   - User wants joint-space safe behavior.
   - Joint 1 and joint 2 should stay almost `0` or on the negative side to secure safe workspace.
   - After reaching a safe space, shake using the remaining joints.
   - Cup mouth should face upward / toward the sky during shake.
   - Joint 5 must never exceed the safe limit, especially `135°`.
3. **Collision handling matters.**
   - Collision should be treated seriously before allowing shake or dispenser-related motion.
   - Previous collision-on planning had failed due to start-state collision; do not blindly re-enable collision planning without checking the current planning scene and start state.
4. **Panel checkbox UX must be fixed.**
   - After the user presses “선택 실행” and the selected commands finish, the UI should uncheck the checkboxes that were just executed.
   - The user has asked for this repeatedly.
5. **Dispenser press HOME return should exist.**
   - The panel previously forced `return_home:=false`; it has since been changed to `return_home:=true` for `press_dispenser_*` commands.
   - Keep `move_home_first:=false` unless the user changes their mind: press should start from current pose, then return HOME after pressing.

## What was already fixed before this handoff

### Dispenser press TCP / HOME

Files changed in commit `1da2078`:

- `src/azas_dispenser/azas_dispenser/dispenser_press_node.py`
  - Added `tcp_name` parameter.
  - Requires TCP for taught POSX mode by default.
  - Calls `/{service_prefix}/tcp/set_current_tcp` then verifies with `/{service_prefix}/tcp/get_current_tcp`.
  - If TCP is missing/wrong, it fails closed before motion.
  - Added pose verification tolerance and now aborts when actual pose does not reach target.
  - This directly addresses the log where the robot reported a 100 mm z error but the step still claimed completed.

- `tools/run/robot_pipeline_control_server.py`
  - `DEFAULT_DISPENSER_TCP_NAME = "rg2_tcp"`.
  - `press_dispenser_1..4` commands now pass:
    - `-p tcp_name:=rg2_tcp` (or UI/env override)
    - `-p require_tcp_for_taught_posx:=true`
    - `-p return_home:=true`
    - pose tolerance params.
  - Required services for dispenser press include TCP set/get services.

- `docs/robot_pipeline_control.html`
  - Added `DISPENSER_TCP_NAME` field defaulting to `rg2_tcp`.

Validation already run for that patch:

```bash
python3 -m py_compile src/azas_dispenser/azas_dispenser/dispenser_press_node.py tools/run/robot_pipeline_control_server.py
node --check /tmp/robot_panel_inline.js
source /opt/ros/humble/setup.bash && colcon build --packages-select azas_dispenser --symlink-install
```

### Camera table view pose

Panel `lift_robot` / camera table-view command was changed to the user's measured joint command:

```bash
python3 tools/run/direct_movej_joints.py \
  --service-prefix dsr01 \
  --j1 0 --j2 -5 --j3 50 --j4 0 --j5 135 --j6 0 \
  --velocity 30 --acceleration 30 \
  --j5-min-deg -135 --j5-max-deg 135 \
  --timeout-sec 60 --execute --confirm ENABLE_DIRECT_MOVEJ
```

## Shake-specific diagnosis at handoff

Current real shake path is still unsafe in concept:

- Panel `shake_closed_cup` routes to `tools/run/run_rule_based_shake_real.sh`.
- That script launches `azas_bringup tumbler_shake_sequence.launch.py`.
- `tumbler_shake_sequence_node.py` currently builds Cartesian `MoveLine` waypoints around a base-frame center using `rx/ry/rz` plus offsets.
- The panel command had been tuned with environment variables such as `SHAKE_CENTER_X`, `SHAKE_CENTER_Y`, `SHAKE_CENTER_Z`, `SHAKE_TWIST_*`, `APPROACH_LINE_TIME`, `SHAKE_LINE_TIME`, but that is not enough; the user explicitly wants code-level change.
- The current node prechecks IK joint 5, but that does not guarantee a safe joint branch or cup-up posture. IK may choose a bad branch or a visually unsafe motion.

Files to change next:

- `src/azas_motion/azas_motion/tumbler_shake_sequence_node.py`
- `src/azas_bringup/launch/tumbler_shake_sequence.launch.py`
- `tools/run/run_rule_based_shake_real.sh`
- `tools/run/robot_pipeline_control_server.py`
- `tools/smoke/smoke_tumbler_shake_sequence.sh`
- Possibly `src/azas_motion/azas_motion/m0609_shake_joint_state_node.py` for RViz preview so it matches the real joint-space logic.
- `docs/robot_pipeline_control.html` for checkbox clearing.

Recommended implementation direction:

1. Add a **joint-space shake mode** to `tumbler_shake_sequence_node.py` using `MoveJoint`, not `MoveLine`.
2. Default real shake to this joint-space mode.
3. Use a named safe base joint pose with joint 1 and joint 2 near `0` or negative.
   - Do **not** invent final joint values as “safe” without RViz/operator verification.
   - Expose them as explicit parameters and mark them as field-tuned.
4. Build shake offsets only from safe base pose:
   - Keep joint 1 and joint 2 fixed or very small/negative-bounded.
   - Use joint 3/4/5/6 for shake motion.
   - Clamp joint 5 to `[-135, 135]` for every generated waypoint.
   - Reject any generated waypoint outside limits before calling MoveJoint.
5. Add current joint/state sanity gates before shake:
   - Read `/dsr01/aux_control/get_current_posj`.
   - If current pose is too far from the configured safe shake base, first move slowly to a safe approach joint pose, or fail closed if no verified transition exists.
6. Add collision gate before real shake if using MoveIt state validity:
   - Query `/check_state_validity` for every candidate joint waypoint when available.
   - If unavailable or invalid, fail closed for real mode.
   - Be careful because previous collision-on planning failed at start-state collision; report exact invalid state/contact instead of blindly commanding.
7. Update RViz preview so the robot visualization uses the same joint-space keyframes, not a separate misleading path.
8. Update smoke tests to assert fake hardware receives `move_joint` requests for shake, not only `move_line`.

## Panel UX bug to fix next

In `docs/robot_pipeline_control.html`, after `/api/run` returns, uncheck the selected checkboxes that were executed. Current JS already computes `selected` in the click handler:

```js
const selected = selectedStepKeys();
```

After processing `data.results`, add logic equivalent to:

```js
for (const key of selected) {
  const input = document.querySelector(`.step-check[value="${CSS.escape(key)}"]`);
  if (input) input.checked = false;
}
```

Need to be careful with CSS escaping inside a value selector; an easier robust implementation is to loop all `.step-check` and uncheck those whose value is in a Set.

## Do not forget project rules

- Do not commit `build/`, `install/`, `log/`, `.omx/`, `.agents/`, `.codex/`.
- Do not invent cup coordinates. Cup pose must come from `/jarvis/tumbler_dispenser/tumbler_pose` in production paths.
- Calibration fields marked null/unknown must not be fabricated.
- For hardware-impacting code changes, document safety assumptions, speed limits, failure behavior, and validation evidence.

## Suggested immediate next commit scope

One small safety/UX patch:

1. Disable/replace current `shake_closed_cup` real command until joint-space shake mode is implemented, or make it fail closed unless `SHAKE_MODE=joint` is explicitly selected.
2. Implement checkbox auto-uncheck after run in the HTML panel.
3. Add/adjust fake-hardware smoke tests for the new behavior.
4. Build and run static checks; do not execute real robot motion.

## Stop condition for next agent

Do not tell the user shake is safe until all are true:

- Real shake code path uses joint-space or a verified collision-safe path, not the current Cartesian MoveLine path.
- Joint 1/2 behavior matches user requirement: near zero or negative-side safe workspace, not swinging into bad base motion.
- Cup-up / sky-facing posture is represented by verified joint pose or documented TCP orientation, not assumed.
- Joint 5 is proven clamped/rejected at or below `135°` for all waypoints.
- Collision/state validity gate is either passing with evidence or the step remains blocked.
- RViz preview matches the actual real-motion command path.
- Panel unchecks selected steps after execution.

## 2026-05-19 update — press failure and shake J3 rollback

Latest branch remains `integration/dispenser-feature-trial-20260518`.

### Latest user evidence

- `press_dispenser_2 / green` failed after RG2 full-close because `dispenser_press_node` built the first taught-POSX step as:
  - current HOME-ish XYZ: `[224.8, 4.1, 579.2]`
  - but taught dispenser RPY: `[168.6, -117.1, -149.8]`
- Robot did not move to the mixed pose; strict verification correctly failed at `position=125.91 mm`.
- User also reported current shake is worse than the previous version:
  - `joint_3` must not go negative / backward.
  - Shake was not visibly happening.

### Fixes applied

- `src/azas_dispenser/azas_dispenser/dispenser_press_node.py`
  - Taught-POSX first lift now preserves **current TCP RPY** while lifting at current XY.
  - Dispenser alignment and press steps then use taught dispenser RPY.
  - Step verification now checks each step against its own per-step RPY instead of one global RPY.
  - This prevents the bad “current XYZ + taught RPY” first MoveLine.

- `tools/run/robot_pipeline_control_server.py`
  - `press_dispenser_*` panel command changed to:
    - `move_home_first:=true`
    - `close_gripper_at_home:=true`
    - `gripper_service:=/jarvis/rg2/set_width`
    - `gripper_close_width:=0.0`
    - `gripper_close_force:=12.0`
  - Meaning: HOME 이동 → RG2 full-close success 검증 → taught POSX transit/press/retreat → HOME 복귀.
  - External `tools/run/rg2_full_close_verify.sh && ...` was removed from the press command because closing must happen after the HOME move, not before it.

- Shake joint defaults restored to a safer previous-style J3-positive base:
  - `JOINT_SHAKE_BASE_J1_DEG=0.0`
  - `JOINT_SHAKE_BASE_J2_DEG=-35.0`
  - `JOINT_SHAKE_BASE_J3_DEG=50.0`
  - `JOINT_SHAKE_J3_AMPLITUDE_DEG=0.0`
  - `JOINT_SHAKE_J3_MIN_DEG=0.0`
  - `JOINT_SHAKE_J3_MAX_DEG=135.0`
  - `JOINT_SHAKE_BASE_J5_DEG=70.0`
  - `JOINT_SHAKE_J5_AMPLITUDE_DEG=30.0`, so J5 stays `40..100`.
  - `JOINT_SHAKE_J4_AMPLITUDE_DEG=18.0`, `JOINT_SHAKE_J6_AMPLITUDE_DEG=36.0`.
  - Shake speed restored to `SHAKE_JOINT_VELOCITY=95.0`, `SHAKE_JOINT_ACCELERATION=150.0`, `SHAKE_JOINT_TIME=0.32`.

- `src/azas_motion/azas_motion/tumbler_shake_sequence_node.py`
  - Added J3 safety range validation.
  - Restored non-cocktail labels/pattern: `j5_j6_plus`, `j5_j6_minus`, wrist snaps, J5-only pulses, center.
  - J3 remains positive and fixed by default.

- `src/azas_bringup/launch/tumbler_shake_sequence.launch.py`
- `tools/run/run_rule_based_shake_real.sh`
- `src/azas_motion/azas_motion/m0609_shake_joint_state_node.py`
- `tools/smoke/smoke_tumbler_shake_sequence.sh`
  - Updated to match the J3-positive shake path.

- `tools/smoke/fake_hardware_services.py`
  - Extended fake services to support `move_wait`, `get_current_posx`, and TCP set/get for no-motion dispenser press validation.

### Validation evidence

No real robot motion was executed by the agent.

Passed:

```bash
python3 -m py_compile \
  src/azas_dispenser/azas_dispenser/dispenser_press_node.py \
  src/azas_motion/azas_motion/tumbler_shake_sequence_node.py \
  src/azas_motion/azas_motion/m0609_shake_joint_state_node.py \
  src/azas_bringup/launch/tumbler_shake_sequence.launch.py \
  tools/run/robot_pipeline_control_server.py \
  tools/smoke/fake_hardware_services.py

source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
colcon build --packages-select azas_dispenser azas_motion azas_bringup --symlink-install

ROS_DOMAIN_ID=87 SERVICE_PREFIX=azas_fake_shake_<pid> \
  tools/smoke/smoke_tumbler_shake_sequence.sh
```

Shake smoke confirmed:

- `joint_shake_safe_ready` is `[0.0, -35.0, 50.0, 0.0, 70.0, 0.0]`.
- `j5_j6_plus` is `[0.0, -35.0, 50.0, -18.0, 100.0, 36.0]`.
- `j5_j6_minus` is `[0.0, -35.0, 50.0, 18.0, 40.0, -36.0]`.
- Unsafe joint_1 shake plan fails closed.

Additional no-motion dispenser press validation passed against fake services:

- `move to HOME` executed first.
- RG2 set_width close command accepted with `width_m=0.000`, `force_n=12.0`.
- `lift to transit height` preserved current RPY `[89.8, 179.9, 120.3]`.
- `align above dispenser` and press steps used taught green RPY `[168.569, -117.133, -149.816]`.
- `press dispenser pump` reached fake target.
- HOME return executed.

Panel server was restarted. `/api/steps` resolved commands now show:

- `press_dispenser_2`: HOME → full-close in node → taught POSX press → HOME.
- `shake_closed_cup`: J3 fixed positive at `50.0`, J3 min `0.0`, J5 range `40..100`.

## 2026-05-19 update — restore PR #3 press path, keep cup-place/HOME/close order

The user clarified that the merged `feature/dispenser` / PR #3 press path was already working and should not have been changed. The press path was therefore restored to the PR-compatible taught-POSX sequence:

1. `lift to transit height` at current XY.
2. `align above dispenser`.
3. `descend to approach`.
4. `move to dispenser top`.
5. `press dispenser pump`.
6. `retreat above dispenser`.
7. `lift to return transit height`.

The required higher-level flow is now:

- `move_to_dispenser_*`: move cup under dispenser and then RG2 full-open to release cup.
- `press_dispenser_*`: move HOME, full-close gripper, execute original PR #3 taught-POSX press path, then HOME return.

Important details:

- `strict_pose_verification` default is `false`, matching PR #3 behavior where pose error is logged but does not abort each step.
- TCP guard remains: if `GripperDA_v1_jarvis` is already active, `tcp/set_current_tcp` is skipped; otherwise it tries to set/verify TCP.
- Panel command no longer uses external pre-close and no longer sets `transit_height:=0.0`.

Validation, no real robot motion:

```bash
python3 -m py_compile src/azas_dispenser/azas_dispenser/dispenser_press_node.py tools/run/robot_pipeline_control_server.py
colcon build --packages-select azas_dispenser --symlink-install
```

Fake press validation confirmed:

- HOME `move_joint`.
- RG2 full-close via `/jarvis/rg2/set_width`.
- 7 PR-compatible MoveLine steps including `z=579.151` transit.
- press down at `z=339.151`.
- HOME return.

Panel server restarted and `/api/steps` confirmed:

- `move_to_dispenser_1`: measured front hold + `tools/run/rg2_full_open_verify.sh`.
- `press_dispenser_1`: PR-compatible path with `move_home_first:=true`, `close_gripper_at_home:=true`, `strict_pose_verification:=false`.

## 2026-05-19 update — panel Doosan service startup race

Latest user evidence showed `move_to_dispenser_1` blocked because motion/TCP/aux services were absent in the panel pre-check while `press_dispenser_1` in the same run later succeeded after waiting for motion services.

### Diagnosis

- This is a service graph readiness race, not a dispenser coordinate failure.
- `move_to_dispenser_*` did a one-shot `ros2 service list` pre-check and blocked immediately.
- `dispenser_press_node` has its own service wait loop, so it survived the same startup delay and then completed.

### Fix applied

- `tools/run/robot_pipeline_control_server.py`
  - Added a per-step required-service wait window before blocking:
    - `move_to_dispenser_*` and `press_dispenser_*`: 35 seconds.
    - HOME/lift/side/shake: 30 seconds.
    - gripper-only steps: 12 seconds.
  - Added `/{service_prefix}/system/get_robot_state` to Doosan real-motion service requirements so the later motion-ready gate is not called before the system service exists.
  - `missing_required_services()` now retries until the required Doosan/RG2 services settle, then only blocks with the final missing list if they never appear.

Validation:

- `python3 -m py_compile tools/run/robot_pipeline_control_server.py src/azas_dispenser/azas_dispenser/dispenser_press_node.py`
- Local import/monkeypatch smoke verified `move_to_dispenser_1` no longer fails on an early incomplete service list and waits until required services appear.
- Panel server restarted and `/api/steps` returned 27 steps.

No real robot motion was executed for this validation.

## 2026-05-19 update — more dynamic joint-space cocktail shake

User requested a more dynamic shake while preserving the safety rules already established for real robot testing.

### Safety constraints preserved

- Joint-space mode remains the panel default for `shake_closed_cup`.
- `joint_1`: `[-20, 5]` deg.
- `joint_2`: `[-80, 5]` deg.
- `joint_3`: fixed positive at `50 deg`, allowed range `[0, 135]`; no negative J3 shake.
- `joint_5`: constrained to `[40, 100]` deg.
- `joint_shake_max_single_delta_deg=75`.
- Panel still uses `REQUIRE_STATE_VALIDITY_FOR_JOINT_SHAKE=true`.

### Change applied

- `src/azas_motion/azas_motion/tumbler_shake_sequence_node.py`
  - Reworked joint shake cycle into a more cocktail-like wrist pattern:
    - `j5_j6_plus`
    - `wrist_snap_minus`
    - `j5_j6_minus`
    - `wrist_snap_plus`
    - `j5_only_plus`
    - `j5_only_minus`
    - `diagonal_snap_plus`
    - `diagonal_snap_minus`
    - `center`
  - This adds diagonal cross-snaps and alternating wrist recoil while keeping J3 fixed positive and J5 inside 40..100.
  - Default J4 amplitude increased from `18` to `24`.
  - Default shake joint speed increased:
    - velocity `95 → 125`
    - acceleration `150 → 190`
    - time `0.32 → 0.24`

- `src/azas_bringup/launch/tumbler_shake_sequence.launch.py`
- `tools/run/run_rule_based_shake_real.sh`
- `tools/run/robot_pipeline_control_server.py`
  - Updated default/panel values to match the more dynamic joint shake.

### Validation, no real robot motion

```bash
python3 -m py_compile src/azas_motion/azas_motion/tumbler_shake_sequence_node.py src/azas_bringup/launch/tumbler_shake_sequence.launch.py tools/run/robot_pipeline_control_server.py
bash -n tools/run/run_rule_based_shake_real.sh
colcon build --packages-select azas_motion azas_bringup --symlink-install
tools/smoke/smoke_tumbler_shake_sequence.sh
ros2 launch azas_bringup tumbler_shake_sequence.launch.py ... enable_hardware:=false shake_control_mode:=joint shake_cycles:=1
curl -s http://127.0.0.1:8765/api/steps
```

Evidence:

- Fake hardware smoke reached `DONE`.
- Unsafe joint_1 shake plan still fails closed.
- Dry-run plan showed the new dynamic waypoints, e.g.:
  - `j5_j6_plus`: `[0, -35, 50, -24, 100, 36]`
  - `wrist_snap_minus`: `[0, -35, 50, 24, 54, -36]`
  - `diagonal_snap_plus`: `[0, -35, 50, -18, 86, -27]`
- `/api/steps` confirmed panel `shake_closed_cup` command includes:
  - `JOINT_SHAKE_J4_AMPLITUDE_DEG=24.0`
  - `SHAKE_JOINT_VELOCITY=125.0`
  - `SHAKE_JOINT_ACCELERATION=190.0`
  - `SHAKE_JOINT_TIME=0.24`

No real robot motion was executed for this validation.

## 2026-05-19 update — press pre-HOME retreat to avoid cup sweep

User observed that `press_dispenser_*` starts by moving HOME, and that direct joint-space HOME can sweep through the cup after the cup has been released under the dispenser.

### Fix applied

- `src/azas_dispenser/azas_dispenser/dispenser_press_node.py`
  - Added optional `pre_home_retreat_before_home`.
  - When enabled, the node reads the current Doosan TCP pose before `move to HOME`.
  - If current TCP `x >= 450 mm`, it performs one checked MoveLine retreat before HOME:
    - default/panel direction: `dx=-140 mm`, `dy=0`.
    - preserves current RPY.
    - keeps current Z by default (`pre_home_retreat_min_z_mm=0.0`) instead of forcing a vertical lift.
    - uses checked/no-clamp bounds and aborts before HOME if the retreat target is outside allowed range.
  - Then the existing PR #3-compatible flow continues unchanged:
    - `move HOME` → RG2 full-close → taught POSX press path → HOME return.

- `tools/run/robot_pipeline_control_server.py`
  - Panel `press_dispenser_1..4` commands now pass:

```bash
-p pre_home_retreat_before_home:=true \
-p pre_home_retreat_dx_mm:=-140.0 \
-p pre_home_retreat_dy_mm:=0.0 \
-p pre_home_retreat_min_z_mm:=0.0 \
-p pre_home_retreat_min_current_x_mm:=450.0 \
-p pre_home_retreat_velocity:=25.0 \
-p pre_home_retreat_acceleration:=35.0
```

### Validation, no real robot motion

```bash
python3 -m py_compile src/azas_dispenser/azas_dispenser/dispenser_press_node.py tools/run/robot_pipeline_control_server.py
colcon build --packages-select azas_dispenser --symlink-install
curl -s http://127.0.0.1:8765/api/steps
```

Evidence:

- `py_compile` passed.
- `azas_dispenser` build passed.
- Panel server restarted.
- `/api/steps` confirmed all four `press_dispenser_*` resolved commands include `pre_home_retreat_before_home:=true`.

No real robot motion was executed for this validation.

## 2026-05-19T15:34:58+09:00 update — panel UI/UX compact three-pane layout

The user reported that the robot control panel required too much scrolling and was hard to read.

### Fix applied

- `docs/robot_pipeline_control.html`
  - Reworked the page into a fixed-height three-pane layout:
    - left: settings, arm checkbox, run/cleanup/stop, quick selection buttons.
    - center: searchable/filterable pipeline step list.
    - right: execution log, fixed independently from the step list.
  - Collapsed long shell commands behind per-step `명령 보기` accordions so each card stays compact.
  - Added quick filters/selection for connection, cup move, press, shake, real motion, and blocked steps.
  - Added selected/loaded counts in the header.
  - Preserved per-step result badges across filtering/re-rendering.
  - After execution, selected checkboxes are cleared from the internal selection state.
  - Added log copy/clear controls.

### Validation, no real robot motion

```bash
python3 -m py_compile tools/run/robot_pipeline_control_server.py
node --check /tmp/azas_panel_script.js
curl -fsS http://127.0.0.1:8765/
curl -fsS http://127.0.0.1:8765/api/steps
```

Evidence:

- HTML static checks passed for the new layout markers and collapsed command UI.
- JavaScript syntax check passed.
- Panel server restarted on `127.0.0.1:8765`.
- `/api/steps` returned 27 steps.

No real robot motion was executed for this validation.


## 2026-05-19T15:45:47+09:00 update — panel readability correction after three-pane feedback

The first three-pane UI reduced scrolling but made readability worse because the center column became cramped.

### Fix applied

- `docs/robot_pipeline_control.html`
  - Removed the narrow permanent 3-column layout.
  - Replaced it with a wide single-board layout and sticky top execution/search bar.
  - Moved settings into a collapsible `설정 보기 / 숨기기` section.
  - Kept commands hidden behind `명령 보기` accordions.
  - Changed the log to a bottom drawer that is collapsed by default and opens automatically for run/cleanup/stop.
  - Rendered steps as wider cards in an auto-fit grid, grouped by stage.

### Validation, no real robot motion

```bash
node --check /tmp/azas_panel_script.js
python3 -m py_compile tools/run/robot_pipeline_control_server.py
curl -fsS http://127.0.0.1:8765/
curl -fsS http://127.0.0.1:8765/api/steps
```

Evidence:

- HTML rewrite checks passed.
- JavaScript syntax check passed.
- Panel server restarted on `127.0.0.1:8765`.
- `/api/steps` returned 27 steps.

No real robot motion was executed for this validation.


## 2026-05-19T15:51:41+09:00 update — panel side-log layout per user request

The user rejected the collapsible bottom log and requested the log to stay on the side.

### Fix applied

- `docs/robot_pipeline_control.html`
  - Removed the bottom collapsible log drawer.
  - Added a two-column content layout:
    - left: wide grouped pipeline cards.
    - right: always-visible sticky execution log.
  - Kept settings collapsed in the top control bar to avoid returning to the cramped 3-column layout.
  - Kept command text collapsed under `명령 보기`.
  - Added `로그로 이동` button for narrow screens where the log stacks below the card board.

### Validation, no real robot motion

```bash
node --check /tmp/azas_panel_script.js
python3 -m py_compile tools/run/robot_pipeline_control_server.py
curl -fsS http://127.0.0.1:8765/
curl -fsS http://127.0.0.1:8765/api/steps
```

Evidence:

- HTML side-log checks passed.
- JavaScript syntax check passed.
- Panel server restarted on `127.0.0.1:8765`.
- `/api/steps` returned 27 steps.

No real robot motion was executed for this validation.
