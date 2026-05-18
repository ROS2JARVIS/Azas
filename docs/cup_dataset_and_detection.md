# Cup Dataset And Detection

This note records the current C/D path for cup perception.

## Dataset

Legacy cup-only dataset path:

```text
/home/ssu/Downloads/yolo_cup_dataset
```

Current split counts:

```text
train: 301 images
val: 86 images
test: 43 images
```

The dataset has `images/`, `labels/`, and `cup.yaml`, but the label folders are currently empty. YOLO training is blocked until `labels/{train,val,test}/*.txt` files exist.

Current cup/lid dataset path from the operator:

```text
/home/ssu/Downloads/로봇 데이터
```

Observed static-photo corpus:

```text
cup_only: 23 jpg
lid: 40 jpg
cup_lid_together: 30 jpg
demo_environment: 20 jpg
cup_noise: 64 jpg
model weights: 3 best.pt files
classes: cup, lid
```

The existing `test_predictions/centers.csv` sample has 18 predictions against 19 GT rows with average confidence `0.770`. It is usable as an offline perception regression gate, but it shows known weak cases:

- 3 predictions below confidence `0.60`.
- 1 cup/lid class mismatch sample.
- 1 missing cup match and 1 missing lid match against GT.

Run:

```bash
/home/ssu/Azas/tools/checks/check_static_cup_lid_dataset.py
```

This gate proves only that static images, labels, weights, and prior predictions are present and internally usable. It does not prove depth, hand-eye calibration, camera-to-base TF, or real robot coordinates.

## C. Auto Label With YOLO

Use `tools/perception/auto_label_yolo_cups.py` to bootstrap labels from a pretrained Ultralytics COCO detector. The script maps the COCO class named `cup` to local class id `0`.

Recommended flow:

```bash
python3 -m venv /tmp/azas-yolo
source /tmp/azas-yolo/bin/activate
pip install ultralytics
python3 tools/perception/auto_label_yolo_cups.py --dry-run
python3 tools/perception/auto_label_yolo_cups.py --overwrite
```

Then manually review the labels before training. The sample images include light or transparent cups on a light tabletop, so generic COCO detection may miss or under-box some cups.

## D. Depth Rule Detector MVP

Do not block MVP-1 on a trained detector. On the robot, use aligned depth and `CameraInfo` to produce a conservative candidate first:

```text
aligned depth + CameraInfo
-> nearest tabletop object blob in workspace ROI
-> median pixel and median depth
-> camera_frame 3D point
-> tf2 transform to base_link
-> MoveIt PlanningScene + pick_and_align
```

`tools/perception/depth_rule_cup_detector.py` contains a dependency-light depth utility for this path. It is meant to be wired into `azas_perception` after the actual camera topics, frame ids, depth scale, and workspace ROI are confirmed.

## Safety Boundary

LLM/STT must not generate cup coordinates. Coordinates must come from calibrated camera data, TF, and checked parameters. If depth evidence, TF, calibration, planning, or gripper state is uncertain, `/azas/pick_and_align` must fail closed.
