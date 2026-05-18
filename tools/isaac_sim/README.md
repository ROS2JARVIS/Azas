# Isaac Sim Lesson 21 Sensor Examples

These standalone scripts implement the 21차시 PDF exercises for Camera, OpenCV
calibration, SingleViewDepthSensor, RTX LiDAR, IMU, and Contact Sensor.

Run them with Isaac Sim's `python.sh`, not system Python:

```bash
/home/ssu/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh tools/isaac_sim/camera_sensor_demo.py --headless --test --output-dir /tmp/azas_camera_test
/home/ssu/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh tools/isaac_sim/camera_opencv_fisheye.py --output-dir /tmp/azas_fisheye_test
/home/ssu/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh tools/isaac_sim/camera_opencv_pinhole.py --output-dir /tmp/azas_pinhole_test
/path/to/isaac-sim/python.sh tools/isaac_sim/realsense_depth_asset_demo.py --test
/path/to/isaac-sim/python.sh tools/isaac_sim/rotating_lidar_rtx_demo.py --test
/path/to/isaac-sim/python.sh tools/isaac_sim/imu_contact_sensor_demo.py --test
```

`camera_sensor_demo.py` prints frame summaries by default so the terminal is not
flooded with full image/motion-vector arrays. Use `--verbose-frame-data` only
when raw frame dictionaries are needed. Do not pass `--disable-output` when you
want PNG files.

System Python can still validate the non-Isaac configuration layer:

```bash
python3 tools/checks/check_isaac_sensor_lesson21.py
```
