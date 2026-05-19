# azas_dispenser

Azas dispenser press feature package copied from the initial `jarvis` dispenser prototype.

## Contents

- `dispenser_press_node`: direct Doosan service-based dispenser press sequence
- `dispenser_press_moveit_node`: MoveIt-based prototype sequence
- `find_press_ready_pose_node`: helper for finding candidate press-ready poses
- `spawn_dispenser.launch.py`: Gazebo dispenser model spawn helper

## Build

```bash
cd /home/ssu/Azas
colcon build --packages-select azas_dispenser
source install/setup.bash
```

## Example

```bash
ros2 launch azas_dispenser dispenser_press.launch.py target_dispenser:=green service_prefix:=/
```

Use `service_prefix:=dsr01` when Doosan services are namespaced under `/dsr01`, and `service_prefix:=/` when services are exposed as `/motion/...` and `/aux_control/...`.
