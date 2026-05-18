# Tumbler And Dispenser 3D Models

Generated from `wiki/sources/User Supplied Tumbler And Dispenser Specs.md`.

## Asset Locations

Source copy:

```text
/home/ssu/Azas/models
```

ROS package copy for RViz:

```text
/home/ssu/ros2_ws/src/Azas/models
/home/ssu/ros2_ws/install/jarvis/share/jarvis/models
```

## RViz Mesh Resources

Use these in `visualization_msgs/Marker.mesh_resource` with `type=MESH_RESOURCE`:

```text
package://jarvis/models/azas_tumbler_shaker.obj
package://jarvis/models/azas_dispenser_single.obj
package://jarvis/models/azas_four_dispenser_row.obj
package://jarvis/models/azas_tumbler_dispenser_preview.obj
```

## Isaac Sim Assets

Open or import:

```text
/home/ssu/Azas/models/azas_tumbler_dispenser_preview.usda
/home/ssu/Azas/models/azas_tumbler_dispenser_preview.obj
```

The `.usda` stage references the OBJ files and lays out a four-dispenser row plus one tumbler preview.

## Included Dimensions

- Tumbler: 75 mm diameter, 170 mm lidded height, 140 mm lidless body height.
- Dispenser: 58 mm bottle width reference, 275 mm bottle height, 18/28 mm mouth inner/outer diameter, 205 mm tube length, 7/8.5 mm tube inner/outer diameter, 195 mm pump head length, 117 mm exposed pump portion.

## Regeneration

```bash
cd /home/ssu/Azas
./tools/generate_tumbler_dispenser_models.py
cp -f models/* /home/ssu/ros2_ws/src/Azas/models/
cd /home/ssu/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jarvis
```

## Safety Note

These are visualization and digital-twin approximation meshes. They are not calibrated robot-cell collision geometry until physically measured and validated in the real workcell.
