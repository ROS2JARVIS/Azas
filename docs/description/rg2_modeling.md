# RG2 Modeling Notes

The Azas RG2 model lives in `src/azas_description`.

## Current Model

`urdf/rg2_parametric.xacro` is a source-backed parametric approximation of the
OnRobot RG2. It uses documented gross properties only:

- total stroke: 110 mm
- external dimensions: 213 x 149 x 36 mm
- product weight: 0.78 kg

Sources:

- OnRobot RG2 product page: https://onrobot.com/us/products/rg2-gripper
- OnRobot RG2 datasheet: https://onrobot.com/storage/datasheets/datasheet_rg2.pdf

The model provides:

- `rg2_mount`
- `rg2_body`
- `rg2_palm`
- `rg2_left_finger`
- `rg2_right_finger`
- `rg2_tcp`
- `gripper_tcp` as a fixed alias of `rg2_tcp` by default

`urdf/m0609_rg2_parametric.urdf.xacro` includes the base Doosan
`dsr_description2/urdf/m0609.urdf` and then attaches this RG2 model under
`tool0`. It avoids wrapping the local `dsr_description2/xacro/m0609.urdf.xacro`
because that file may contain site-local tool edits.

The two finger links use prismatic joints. `rg2_right_finger_joint` mimics
`rg2_left_finger_joint`, so opening the left joint opens both fingers.

## Safety Boundary

`rg2_mount_xyz`, `rg2_mount_rpy`, `rg2_tcp_z_offset`, and the default
`gripper_tcp` alias are not measured Azas calibration values. They are
launch/xacro parameters so the model can be reviewed in RViz before official CAD
or measured adapter/TCP evidence is added.

The current review orientation mounts the parametric RG2 under `tool0` with
`rg2_mount_rpy="0 1.570796327 0"` so the gripper body axis is perpendicular to
the M0609 wrist flange view instead of lying sideways from the flange.

Do not copy these defaults into `calibration.yaml` as confirmed hardware values.

## Official CAD Upgrade Path

1. Put official visual meshes under `src/azas_description/meshes/rg2/visual`.
2. Keep MoveIt collision simple: boxes/cylinders or simplified collision meshes
   under `src/azas_description/meshes/rg2/collision`.
3. Replace only the relevant `<visual>` blocks first; keep collision primitives
   conservative until planning has been checked.
4. Update mount/TCP origins only from official CAD, adapter drawings, or measured
   hardware evidence.
5. Re-run xacro validation and RViz review before using the model in MoveIt.

## Review Command

```bash
ros2 launch azas_description view_m0609_rg2_parametric.launch.py
```

Move the `rg2_left_finger_joint` slider in the joint state publisher GUI. The
right finger should mimic it and open symmetrically.

## Doosan Bringup Integration

The local Doosan M0609 descriptions in `/home/ssu/ros2_ws/src/doosan-robot2`
now include this RG2 macro:

- `dsr_description2/xacro/m0609.urdf.xacro`
- `dsr_moveit2/dsr_moveit_config_m0609/config/m0609.urdf.xacro`

Those launch paths pass `finger_joints_fixed="true"` so real Doosan MoveIt
bringup does not wait for RG2 finger joint states that the robot controller does
not publish.

The gripper mount is not rotated to fake a wrist pose. For review in RViz, the
MoveIt SRDF includes a `wrist-90` named state where `joint_6` is
`1.570796327` rad.

After building, this command should show the RG2 overlay in RViz:

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  name:=dsr01 \
  mode:=real \
  model:=m0609 \
  host:=192.168.1.100 \
  port:=12345 \
  rt_host:=192.168.1.50
```
