# Prepose tools

## Fill side prepose YAML from snapshots

1) Teach two safe joint-space poses on the robot (컵이 왼쪽일 때 / 오른쪽일 때), then capture JointState once each:

```bash
ros2 topic echo /joint_states --once > prepose_low.txt
ros2 topic echo /joint_states --once > prepose_high.txt
```

2) Fill `src/dsr_practice/config/side_prepose.yaml`:

```bash
python3 tools/prepose/fill_side_prepose_yaml.py \
  --low prepose_low.txt \
  --high prepose_high.txt \
  --mode y \
  --enable
```

Notes
- `ros2 topic echo` outputs joint positions in **radians** for `sensor_msgs/msg/JointState`. If your snapshot is degrees for some reason, add `--degrees`.
- `--mode y`: fills `side_prepose_joints_cup_left/right_rad` (cup y-based selection).
- `--mode z`: fills legacy `side_prepose_joints_low/high_rad` (z-based selection).
- This tool only writes joint arrays + optionally enables the feature; it does not generate or ask for any cup coordinates.
