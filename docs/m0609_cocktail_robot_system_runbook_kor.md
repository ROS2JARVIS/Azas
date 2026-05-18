# M0609 Cocktail Robot System Runbook

이 문서는 `cocktail_robot_system` 패키지 기준의 현장 실행 절차입니다.

패키지 위치:

```bash
/home/ssu/ros2_ws/src/cocktail_robot_system/src/cocktail_robot_system
```

Hand-eye calibration 산출물:

```bash
~/ros2_ws/src/cocktail_robot_system/src/cocktail_robot_system/config/calibration/T_gripper2camera.npy
```

원본 calibration script가 만든 파일은 보통
`~/ros2_ws/src/doosan-robot2/dsr_practice/dsr_practice/Calibration_Tutorial/T_gripper2camera.npy`
에 생긴다. fork/clone 후에도 동작하도록 현재 패키지 내부
`config/calibration/T_gripper2camera.npy`로 복사해서 사용한다.

## 1. 현재 구조에서 중요한 점

`T_gripper2camera.npy`는 `hand_eye_mode:=eye_in_hand_npy`일 때 자동으로 사용된다.

```text
T_base_camera = T_base_gripper(current from /dsr01/aux_control/get_current_posx)
                * T_gripper_camera(from T_gripper2camera.npy)
```

시뮬레이션 또는 고정 관측 자세에서는 `hand_eye_mode:=static_base_camera`를 사용한다. 이 경우 `params.yaml`의 `hand_eye_transform_matrix`가 직접 `T_base_camera`로 쓰이고, `.npy`는 사용하지 않는다.

## 2. Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cocktail_robot_system --symlink-install
source install/setup.bash
```

설치 확인:

```bash
ros2 pkg executables cocktail_robot_system
ros2 launch cocktail_robot_system lid_pick_place_real.launch.py --show-args
```

## 3. Calibration

RealSense ROS node는 끄고 진행한다. `data_recording.py`는 OpenCV로 카메라를 직접 잡는다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real \
  model:=m0609 \
  host:=192.168.1.100
```

다른 터미널:

```bash
cd ~/ros2_ws/src/doosan-robot2/dsr_practice/dsr_practice/Calibration_Tutorial
rm -rf data
mkdir -p data
python3 data_recording.py
```

로봇 자세를 15장 이상 다양하게 바꾸며 체커보드가 잘 보일 때 `q`를 눌러 저장한다.

계산:

```bash
cd ~/ros2_ws/src/doosan-robot2/dsr_practice/dsr_practice/Calibration_Tutorial
python3 handeye_calibration.py
ls -lh T_gripper2camera.npy
```

새 calibration 결과를 패키지 내부로 반영:

```bash
cp T_gripper2camera.npy \
  ~/ros2_ws/src/cocktail_robot_system/src/cocktail_robot_system/config/calibration/T_gripper2camera.npy
```

확인:

```bash
python3 - <<'PY'
import numpy as np
T = np.load("T_gripper2camera.npy")
np.set_printoptions(precision=6, suppress=True)
print(T)
print("translation:", T[:3, 3])
PY
```

`T_gripper2camera.npy`의 translation은 보통 mm 단위다. `detection_3d_node`는 `gripper_to_camera_translation_unit: auto`로 두면 큰 값은 자동으로 mm에서 m로 변환한다.

## 4. RealSense

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

필수 토픽:

```bash
ros2 topic list | grep camera
```

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

## 5. Real M0609 Dry Run

Doosan real bringup과 RealSense가 떠 있는 상태에서 실행한다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch cocktail_robot_system lid_pick_place_real.launch.py \
  execute_motion:=false \
  use_real_robot:=true \
  hand_eye_mode:=eye_in_hand_npy
```

OpenCV 창에서 뚜껑 bbox와 base 좌표를 확인한다. 창에 포커스를 두고 `p`를 누르면 실제 모션 없이 예정 sequence만 로그로 출력된다.

좌표 확인:

```bash
ros2 topic echo /cocktail/detection_3d/detections
ros2 topic echo /cocktail/detection_3d/target_pose
```

`robot_xyz`가 실제 작업대 위치와 다르면 즉시 중단하고 calibration 또는 camera frame/topic을 다시 확인한다.

## 6. Real M0609 Motion

dry-run pose가 안전하고 현실적인 경우에만 실행한다.

```bash
ros2 launch cocktail_robot_system lid_pick_place_real.launch.py \
  execute_motion:=true \
  use_real_robot:=true \
  hand_eye_mode:=eye_in_hand_npy
```

OpenCV 창에서 `p`:

```text
gripper open
move_pick_approach
move_pick
gripper close
move_lift
move_place_approach
move_place
gripper open
move_retreat
```

종료는 `q` 또는 `ESC`.

## 7. Virtual / No-Hardware Check

실제 로봇 없이 launch와 노드 wiring만 확인할 때:

```bash
ros2 launch cocktail_robot_system lid_pick_place_real.launch.py \
  execute_motion:=false \
  use_real_robot:=false \
  hand_eye_mode:=static_base_camera
```

Doosan service shape까지 fake로 확인할 때:

```bash
cd ~/ros2_ws/src/cocktail_robot_system
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

python3 tools/smoke/fake_hardware_services.py --ros-args -p service_prefix:=dsr01
```

fake services:

```text
/dsr01/motion/move_joint
/dsr01/motion/move_line
/dsr01/motion/move_wait
/dsr01/aux_control/get_current_posx
```

이 fake node는 실제 하드웨어를 움직이지 않는다.

## 8. NumPy / cv_bridge Note

현재 환경에 `numpy 2.x`가 설치되어 있으면 ROS Humble `cv_bridge`가 깨질 수 있다.

`cocktail_robot_system`의 vision/depth/debug image path는 `cv_bridge`를 쓰지 않도록 수정되어 있다. 하지만 다른 패키지나 튜토리얼에서 `cv_bridge`를 직접 import하면 같은 문제가 날 수 있다.

장기적으로는 ROS Humble 환경에서는 다음처럼 NumPy를 1.x로 맞추는 편이 안전하다.

```bash
python3 -m pip install --user "numpy<2"
```

시스템 전체 Python에 영향이 있으므로 현장에서는 팀과 합의 후 적용한다.
