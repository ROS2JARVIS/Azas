# Offline dispenser color discrimination test

This branch adds a camera-free test path for dispenser/cocktail color classification.
It is meant for re-experimenting when the RealSense/camera is not available.

## What it tests

- HSV median color classification for:
  - red
  - orange
  - yellow
  - green
  - blue
  - purple
  - black
  - white
- Center-crop median HSV logic to avoid noisy borders or overlays.
- Optional saved-image crop evaluation from CSV.

This is perception-only. It does not run ROS camera subscribers, MoveIt, gripper, or robot motion commands.

## Quick synthetic regression

```bash
cd /home/ssu/Azas
python3 tools/checks/check_offline_color_discrimination.py
```

Expected result:

```text
[PASS] offline HSV color discrimination works without camera
```

Outputs:

```text
outputs/color_discrimination/color_discrimination_results.csv
outputs/color_discrimination/preview/*.png
```

## Test saved images without a camera

Create a CSV such as `outputs/color_discrimination/manual_boxes.csv`:

```csv
image_path,expected_color,x1,y1,x2,y2
/path/to/image.png,red,100,80,180,160
/path/to/image.png,blue,210,80,290,160
```

Run:

```bash
python3 tools/perception/offline_color_discrimination_test.py \
  --box-csv outputs/color_discrimination/manual_boxes.csv
```

The output CSV includes:

- expected color
- predicted color
- median HSV
- confidence
- preview crop path

## Why this helps

For the real robot project, live camera bringup and robot motion should be separate gates.
This offline test verifies the deterministic color classifier first, using synthetic
patches or saved images, before connecting any camera or robot pipeline.
