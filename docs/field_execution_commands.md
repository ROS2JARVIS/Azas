# Field Execution Commands

Purpose: exact commands to run when the physical Doosan M0609 and camera are
connected in class.

## 1. Doosan Real Connection

First confirm the PC is on the robot subnet. Current observed failure on
2026-05-11: the PC only had `203.246.36.217/24`, while the documented Doosan
defaults are `192.168.137.100` or `192.168.127.100`.

If `ping 192.168.137.100` fails and the wired interface is `enp128s31f6`, add a
temporary secondary robot-subnet IP:

```bash
sudo ip addr add 192.168.137.50/24 dev enp128s31f6
ping 192.168.137.100
```

If the lab controller uses the alternate subnet:

```bash
sudo ip addr add 192.168.127.50/24 dev enp128s31f6
ping 192.168.127.100
```

Try the Doosan official/default real MoveIt bringup first. This is the command
that should turn the robot connection state from red to white if the controller
IP and robot-side state are correct.

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=real model:=m0609 host:=192.168.137.100 port:=12345
```

If the lab controller uses the other documented default subnet:

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=real model:=m0609 host:=192.168.127.100 port:=12345
```

Confirm motion services are visible:

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
ros2 service list | grep motion
```

Expected services include:

```text
/motion/move_joint
/motion/move_line
```

## 2. Course Demo Commands

STT jog-control demo:

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
ros2 launch dsr_practice stt_robot_control.launch.py
```

Publish manual STT commands without using the microphone:

```bash
ros2 topic pub --once /stt_result std_msgs/msg/String "{data: 'home'}"
ros2 topic pub --once /stt_result std_msgs/msg/String "{data: '왼쪽'}"
ros2 topic pub --once /stt_result std_msgs/msg/String "{data: '오른쪽'}"
ros2 topic pub --once /stt_result std_msgs/msg/String "{data: '앞으로'}"
ros2 topic pub --once /stt_result std_msgs/msg/String "{data: '뒤로'}"
ros2 topic pub --once /stt_result std_msgs/msg/String "{data: '위로'}"
ros2 topic pub --once /stt_result std_msgs/msg/String "{data: '아래로'}"
```

STT pick/place demo:

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
ros2 launch dsr_practice stt_pick_and_place.launch.py
```

Manual pick/place commands:

```bash
ros2 topic pub --once /stt_result std_msgs/msg/String "{data: 'pick'}"
ros2 topic pub --once /stt_result std_msgs/msg/String "{data: 'place'}"
ros2 topic pub --once /stt_result std_msgs/msg/String "{data: 'pickplace'}"
```

## 3. Azas Connected Field Path

After the Doosan real bringup works, this project wrapper chains the Azas
camera/YOLO path and acceptance gates. It stops before real motion by default:

```bash
ROBOT_HOST=192.168.137.100 RG2_IP=192.168.1.1 /home/ssu/Azas/tools/run/run_connected_robot_control.sh
```

If the controller IP is on the other subnet:

```bash
ROBOT_HOST=192.168.127.100 RG2_IP=192.168.1.1 /home/ssu/Azas/tools/run/run_connected_robot_control.sh
```

This wrapper is not a replacement for the course demo. It is the Azas field path
for the cocktail task: Doosan connection, RealSense/YOLO detection, and
no-motion acceptance. Real motion still goes through `tools/run/run_robot_real.sh`;
set `RUN_REAL_AFTER_ACCEPTANCE=true` only for a supervised run that should
explicitly hand off after acceptance.

## 4. Why Azas May Still Refuse

`tools/run/run_robot_real.sh` intentionally refuses if any of these are true:

- `/tmp/azas_motion_hold` exists.
- `/tmp/azas_live_hardware_gates_passed` is missing or stale.
- measured `calibration.yaml` / `safety.yaml` gates fail.
- Doosan `move_line` / `move_joint` services are not visible.
- RG2 trigger services are not visible.
- camera topics or base-camera TF are missing.

For a no-motion explanation:

```bash
/home/ssu/Azas/tools/checks/explain_real_robot_blockers.sh
```
