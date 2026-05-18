# Coordinate Frames

Initial frame contract for MVP-1. Values marked `확인 필요` must not be used as hardware truth.

| Frame | Owner | Purpose | Status |
|---|---|---|---|
| `base_link` | Robot/MoveIt | Robot base frame for planning and calibrated targets. | 확인 필요: actual robot config name |
| `camera_frame` | Perception/calibration | Depth camera 3D point frame from `CameraInfo` + depth image. | 확인 필요 |
| `EE_LINK` / `gripper_tcp` | Robot/tool calibration | MoveIt end-effector link and physical RG2 TCP. | 확인 필요: do not hard-code |
| `cup_mouth_center` | Alignment | Virtual point at tumbler opening center after grasp. | Derived from measured TCP-to-cup offset |
| `dispenser_outlet` | Calibration | Fixed outlet center frame. | 확인 필요: teach/calibrate before motion |

## Projection Rule

PDF depth-camera rule:

```text
Z = depth_raw / 1000.0
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

## Alignment Rule

The robot does not ask an LLM/VLA for target coordinates. It computes a `gripper_tcp` target from calibrated transforms so `cup_mouth_center` is below `dispenser_outlet`:

```text
T_base_gripper_target = T_base_dispenser_outlet
                        * T_outlet_cup_target
                        * inverse(T_gripper_tcp_cup_mouth_center)
```
