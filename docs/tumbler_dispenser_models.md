# Tumbler And Dispenser 3D Models

> 폐기 안내: 이 문서의 예전 외부 워크스페이스 복사 절차는 사용하지 않습니다.
> Azas 모델과 RViz/MoveIt 작업은 `/home/ssu/Azas` 아래에서만 관리합니다.

Generated from `wiki/sources/User Supplied Tumbler And Dispenser Specs.md`.

## Asset Locations

Source copy:

```text
/home/ssu/Azas/models
```

ROS package copy for RViz:

```text
/home/ssu/Azas/models
/home/ssu/Azas/install/azas_bringup/share/azas_bringup/models
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
source /opt/ros/humble/setup.bash
colcon build --packages-select azas_bringup azas_motion --symlink-install
source /home/ssu/Azas/install/local_setup.bash
```

## Safety Note

These are visualization and digital-twin approximation meshes. They are not calibrated robot-cell collision geometry until physically measured and validated in the real workcell.
