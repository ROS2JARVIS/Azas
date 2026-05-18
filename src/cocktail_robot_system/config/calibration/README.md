# Calibration Files

`T_gripper2camera.npy` is the eye-in-hand calibration matrix used by
`detection_3d_node` when `hand_eye_mode` is `eye_in_hand_npy`.

This file is specific to one physical setup:

- Doosan M0609
- mounted RealSense camera
- tool/gripper mounting geometry

If the camera mount, tool flange, gripper, or robot setup changes, run hand-eye
calibration again and replace this file.

