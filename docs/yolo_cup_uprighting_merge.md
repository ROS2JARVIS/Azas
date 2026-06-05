# yolo_cup_uprighting merge note

This branch integrates the safe perception parts of
`https://github.com/ssarahstar/yolo_cup_uprighting` into Azas.

## Integrated

- `best.pt` is copied into `src/azas_perception/config/yolo_cup_uprighting_best.pt`.
  It is packaged by `azas_perception/setup.py` and tracked through Git LFS.
- YOLO launch defaults now resolve the packaged model through
  `FindPackageShare("azas_perception")`, removing the old local-only
  `/home/ssu/Downloads/best.pt` default.
- Image-only cup axis / red-marker helpers are adapted into
  `azas_perception.cup_uprighting_vision` with unit tests.

## Deliberately not integrated

The upstream `yolo_pick_demo` motion node directly computes base-frame points
and contains mock coordinates for local testing. That does not match the Azas
project rule that cup poses must flow through:

`/azas/cup_detection` -> `/jarvis/tumbler_dispenser/tumbler_pose`

No upstream motion sequence, mock coordinate, or generated robot trajectory was
merged. Real robot motion should continue to consume the validated
`PoseStamped` from the Azas TF bridge.

## Validation target

- Perception unit tests pass.
- Launch files keep the same override surface: operators can still pass
  `model_path:=...` when testing another model.
