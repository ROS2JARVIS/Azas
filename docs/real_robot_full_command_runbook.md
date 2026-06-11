# 실제 로봇 통합 명령어 총정리

이 문서는 실제 Doosan M0609, RG2, RealSense, 색상 구분, 디스펜서 투입, 컵홀더 재픽업, 쉐이킹까지 현장에서 쓰는 명령을 한 곳에 모은 런북입니다.

중요:

- 컵 좌표는 사람이 직접 넣지 않습니다.
- 컵 위치는 비전 파이프라인의 `/jarvis/tumbler_dispenser/tumbler_pose` 또는 측정된 `calibration.yaml` 값을 사용합니다.
- RViz 명령은 미리보기입니다. 실제 로봇 연결/모션 명령과 섞어 쓰지 마세요.
- 실제 모션 전에 비상정지, 주변 장애물, 컵 뚜껑, 디스펜서 위치, 그리퍼 상태를 확인하세요.

---

## 0. 기본 터미널 준비

새 터미널마다 기본으로 실행합니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash
```

기본 환경값입니다. 현장 IP가 다르면 값만 바꿉니다.

```bash
export ROBOT_HOST=192.168.1.100
export RT_HOST=192.168.1.101
export ROBOT_NAME=dsr01
export SERVICE_PREFIX=dsr01
export RG2_IP=192.168.1.1
```

---

## 1. 제어 패널 실행

브라우저 패널에서 단계별 실행/명령 편집을 하려면 이것을 먼저 켭니다.

```bash
cd /home/ssu/Azas
bash tools/run/run_robot_pipeline_control_panel.sh
```

패널이 보여주는 명령은 `tools/run/robot_pipeline_control_server.py`의 단계 정의와 저장된 명령 override를 기준으로 합니다.

---

## 2. 실제 로봇 연결

로봇 bringup 터미널입니다. 이 터미널은 계속 켜둡니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ROBOT_HOST=192.168.1.100 \
ROBOT_NAME=dsr01 \
RT_HOST=192.168.1.101 \
DOOSAN_REAL_MOTION_CONFIRM=ENABLE_DOOSAN_REAL_MOTION_BRINGUP \
bash tools/run/run_doosan_real_m0609.sh
```

연결 확인:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ros2 service list | grep /dsr01/motion
ros2 service type /dsr01/motion/move_line
ros2 service type /dsr01/motion/move_joint
python3 tools/run/ros_call_empty_service.py /dsr01/system/get_robot_state dsr_msgs2/srv/GetRobotState --timeout 8.0
```

---

## 3. RG2 그리퍼 연결

그리퍼 노드 터미널입니다. 이 터미널도 계속 켜둡니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ros2 launch azas_gripper rg2_trigger.launch.py \
  ip:=192.168.1.1 \
  port:=502 \
  connect:=true \
  open_width:=1100 \
  close_width:=0 \
  force:=300 \
  settle_seconds:=0.6
```

확인:

```bash
ros2 service list | grep /jarvis/rg2
timeout 12s ros2 service call /jarvis/rg2/set_width azas_interfaces/srv/SetGripper "{command: 'set_width', width_m: 0.075, force_n: 25.0}"
```

---

## 4. 카메라 연결과 컵 인식

RealSense 카메라 실행:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true
```

중요: 아래 YOLO launch는 화면을 띄우는 명령이 아닙니다. `/camera/camera/color/image_raw`를 구독해서 `/azas/cup_detection` 같은 인식 토픽을 내보내는 명령입니다.

카메라 화면 확인:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw
```

패널 화면에서 보려면 패널을 켠 뒤 `카메라 갱신`을 누릅니다.

```bash
cd /home/ssu/Azas
bash tools/run/run_robot_pipeline_control_panel.sh
```

브라우저:

```text
http://127.0.0.1:8765/
```

YOLO 컵/뚜껑 인식 실행:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ros2 launch azas_bringup yolo_perception.launch.py
```

토픽 확인:

```bash
ros2 topic echo /azas/cup_detection --once
ros2 topic echo /jarvis/tumbler_dispenser/tumbler_pose --once
```

정리:

- `realsense2_camera`: 카메라 토픽 생성
- `rqt_image_view`: 사람이 보는 화면
- `yolo_perception.launch.py`: 컵/뚜껑 인식 토픽 생성
- `dispenser_color_scan_ros.sh`: 디스펜서 색상 JSON 생성

---

## 5. 디스펜서 색깔 구분

먼저 로봇을 색상 스캔 자세로 보냅니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

python3 tools/run/direct_movej_joints.py \
  --service-prefix dsr01 \
  --j1 0 --j2 10 --j3 32 --j4 0 --j5 100 --j6 90 \
  --velocity 30 \
  --acceleration 30 \
  --timeout-sec 60 \
  --execute \
  --confirm ENABLE_DIRECT_MOVEJ
```

색상 스캔 실행:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

bash tools/run/dispenser_color_scan_ros.sh
```

결과 확인:

```bash
cat outputs/dispenser_color_map.json
```

실패 파일 확인:

```bash
cat outputs/dispenser_color_map.json.failed
```

`outputs/dispenser_color_map.json.failed`만 있고 `outputs/dispenser_color_map.json`이 없으면 색상 스캔이 실패한 상태입니다. 보통 원인은 디스펜서가 카메라 프레임 밖에 있거나, 색상 스캔 자세/TF가 맞지 않거나, 조명 때문에 분류가 `unknown`으로 나온 경우입니다.

수동으로 색상 맵을 확정해야 할 때는 패널 API로 저장합니다. 예시는 1번 red, 2번 blue, 3번 green, 4번 yellow입니다.

```bash
curl -fsS \
  -X POST http://127.0.0.1:8765/api/dispenser_color_map \
  -H 'Content-Type: application/json' \
  -d '{"map":{"1":"red","2":"blue","3":"green","4":"yellow"}}'
```

저장 후 확인:

```bash
cat outputs/dispenser_color_map.json
curl -fsS http://127.0.0.1:8765/api/dispenser_color_map
```

이 파일은 색상 레시피 실행에서 `빨강/파랑/초록...` 같은 색상 이름을 실제 디스펜서 번호로 매핑하는 데 사용됩니다.

---

## 6. 음성 레시피 입력

마이크/STT 실행:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

ros2 launch azas_voice azas_voice.launch.py
```

STT 레시피 수신:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

python3 tools/run/listen_stt_recipe.py --timeout 60
cat outputs/latest_recipe.json
```

---

## 7. 실행 전 준비도 점검

레시피를 디스펜서 번호로 직접 지정할 때는 `1x1,2x2,3x1` 형식을 씁니다. 예시는 1번 1회, 2번 2회, 3번 1회입니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

RECIPE_DISPENSER_IDS=1x1,2x2,3x1 \
ROBOT_HOST=192.168.1.100 \
ROBOT_NAME=dsr01 \
SERVICE_PREFIX=dsr01 \
bash tools/run/check_one_click_cocktail_ready.sh
```

설정만 점검:

```bash
RECIPE_DISPENSER_IDS=1x1,2x2,3x1 \
bash tools/run/check_one_click_cocktail_config.sh
```

---

## 8. 전체 디스펜서 통합 실행

실제 로봇으로 컵 픽업, 디스펜서 앞 배치, 그리퍼 열기, 디스펜서 프레스, 다시 잡기/리프트까지 실행하는 통합 명령입니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE \
ROBOT_HOST=192.168.1.100 \
ROBOT_NAME=dsr01 \
SERVICE_PREFIX=dsr01 \
bash tools/run/run_cocktail_now_real.sh 1x1,2x2,3x1
```

동일한 통합 실행을 환경변수로 지정할 수도 있습니다.

```bash
REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE \
RECIPE_DISPENSER_IDS=1x1,2x2,3x1 \
ROBOT_HOST=192.168.1.100 \
ROBOT_NAME=dsr01 \
SERVICE_PREFIX=dsr01 \
bash tools/run/run_cocktail_now_real.sh
```

---

## 9. 색상/음성 레시피 기반 디스펜서 실행

`outputs/latest_recipe.json`과 `outputs/dispenser_color_map.json`을 사용해서 색상 레시피를 디스펜서 번호로 바꿔 실행합니다.

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

python3 tools/run/run_color_recipe_sequence.py --execute --confirm
```

디스펜서 번호를 직접 지정해서 실행:

```bash
python3 tools/run/run_color_recipe_sequence.py \
  --dispenser-ids 1x1,2x2,3x1 \
  --execute \
  --confirm
```

---

## 10. 디스펜서 개별 단계 명령

통합 스크립트가 내부에서 하는 핵심 순서입니다.

1. 컵을 들고 선택 디스펜서 앞 측정 pose로 이동
2. 컵을 디스펜서 앞에 놓기
3. 그리퍼를 열고 컵 안쪽/전방에서 빠지기
4. 디스펜서 버튼을 1회 이상 프레스
5. 컵을 다시 side grip으로 잡기
6. 컵을 들어 올리기

1번 디스펜서 앞 이동:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

python3 tools/run/move_to_measured_dispenser_front_hold.py \
  --service-prefix dsr01 \
  --dispenser-id 1 \
  --timeout-sec 180 \
  --verify-target \
  --verify-timeout-sec 70 \
  --ikin-timeout-sec 20 \
  --ikin-retries 2 \
  --target-tolerance-mm 15 \
  --no-set-current-tcp-before-move \
  --compensate-current-tcp \
  --direct-x-max 0.95 \
  --verify-link6-target \
  --no-moveit-planning-guard \
  --velocity 35 \
  --acceleration 45 \
  --target-offset-x-m 0.0 \
  --target-offset-y-m 0.0 \
  --target-offset-z-m 0.0 \
  --execute \
  --confirm ENABLE_MEASURED_DISPENSER_FRONT_HOLD
```

1번 디스펜서 1회 전체 사이클:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

python3 tools/run/run_measured_dispenser_recipe_sequence.py \
  --service-prefix dsr01 \
  --dispenser-ids 1 \
  --execute \
  --confirm ENABLE_MEASURED_DISPENSER_RECIPE_SEQUENCE
```

이 러너가 `calibration.yaml`의 해당 디스펜서 측정 press pose를 읽어서 `dispenser_x/y/z`, `rx/ry/rz`를 자동으로 넣습니다. 프레스 pose 좌표를 문서에서 사람이 새로 만들거나 복사하지 않습니다.

1번 디스펜서 앞 컵 다시 잡기:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

python3 tools/run/pick_from_measured_dispenser_front_hold.py \
  --service-prefix dsr01 \
  --dispenser-id 1 \
  --approach-velocity 20.0 \
  --approach-acceleration 25.0 \
  --pregrasp-staging \
  --pregrasp-offset-x-m 0.0 \
  --pregrasp-offset-y-m 0.0 \
  --pregrasp-offset-z-m 0.060 \
  --pregrasp-staging-velocity 12.0 \
  --pregrasp-staging-acceleration 20.0 \
  --joint1-clearance-deg 0.0 \
  --lift-m 0.100 \
  --lift-velocity 18.0 \
  --lift-acceleration 24.0 \
  --timeout-sec 120 \
  --wait-service-sec 8 \
  --verify-timeout-sec 45 \
  --target-tolerance-mm 15 \
  --gripper-grasp-width-m 0.075 \
  --gripper-force-n 25.0 \
  --x-min 0.10 \
  --x-max 0.95 \
  --execute \
  --confirm ENABLE_PICK_FROM_MEASURED_DISPENSER_FRONT_HOLD
```

---

## 11. 컵홀더에 놓고 다시 잡아서 쉐이킹

컵을 컵홀더 측정 pose에 놓기:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

python3 tools/run/place_side_grip_cup_in_holder.py \
  --service-prefix dsr01 \
  --config /home/ssu/Azas/install/azas_bringup/share/azas_bringup/config/calibration.yaml \
  --approach-velocity 15.0 \
  --approach-acceleration 20.0 \
  --place-final-z-offset-m -0.020 \
  --place-velocity 6.0 \
  --place-acceleration 10.0 \
  --retreat-velocity 12.0 \
  --retreat-acceleration 16.0 \
  --timeout-sec 90.0 \
  --target-tolerance-mm 12.0 \
  --verify-timeout-sec 45.0 \
  --z-max 0.28 \
  --execute \
  --confirm ENABLE_CUP_HOLDER_PLACE
```

컵홀더에 놓인 닫힌 컵을 다시 잡고 실제 관절 쉐이킹:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

SERVICE_PREFIX=dsr01 \
bash tools/run/run_rule_based_shake_real.sh
```

이 스크립트는 실제 이동 전에 터미널에서 `ENABLE_REAL_ROBOT_MOTION` 입력을 요구합니다. 내부 순서는 다음과 같습니다.

1. 컵홀더 측정 pose 접근
2. RG2로 컵 다시 잡기
3. 컵홀더에서 리프트
4. 실제 로봇 관절 쉐이킹 실행

패널의 `전체: 컵홀더 재픽업->쉐이킹`은 디스펜서 통합 실행 후 `place_cup_holder`, `shake_closed_cup`을 이어서 실행하는 용도입니다.

---

## 12. RViz 미리보기 전용

아래 명령은 실제 로봇을 움직이지 않습니다.

디스펜서/컵 collision 미리보기:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

RECIPE_DISPENSER_IDS=1x1,2x2,3x1 \
DISPENSER_COLLISION_OBJECTS=1 \
bash tools/run/run_cocktail_collision_rviz_preview.sh
```

미리보기 정리:

```bash
cd /home/ssu/Azas
bash tools/run/stop_cocktail_motion_preview.sh
```

다음 명령은 쉐이킹 RViz 프리뷰입니다. 실제 로봇 연결 명령이 아닙니다.

```bash
cd /home/ssu/Azas
ROS_DOMAIN_ID=79 \
TARGET_X=0.430 TARGET_Y=0.080 TARGET_Z=0.135 \
SHAKE_DELAY_SEC=4.0 \
SHAKE_CENTER_X=0.430 SHAKE_CENTER_Y=0.080 SHAKE_CENTER_Z=0.620 \
SHAKE_AMPLITUDE_X=0.100 SHAKE_AMPLITUDE_Y=0.040 SHAKE_AMPLITUDE_Z=0.055 \
SHAKE_CYCLES=4 \
SHAKE_TWIST_RX_DEG=6.0 SHAKE_TWIST_RY_DEG=3.0 SHAKE_TWIST_RZ_DEG=22.0 \
APPROACH_LINE_TIME=3.5 \
SHAKE_LINE_TIME=0.40 \
MIN_SHAKE_Z=0.550 \
bash tools/run/run_cup_target_then_shake_rviz.sh
```

---

## 13. 결과 확인과 로그

통합 실행 후 결과 확인:

```bash
cd /home/ssu/Azas
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash

SERVICE_PREFIX=dsr01 \
bash tools/run/check_one_click_cocktail_result.sh
```

주요 로그:

```bash
ls -lt log/manual | head
tail -n 120 log/manual/one_click_real_integrated_recipe.log
tail -n 120 log/manual/one_click_real_readiness.log
tail -n 120 log/manual/one_click_real_result.log
```

---

## 14. 추천 실제 운영 순서

터미널별로 나누면 다음 순서가 가장 덜 헷갈립니다.

1. `bash tools/run/run_robot_pipeline_control_panel.sh`
2. 실제 로봇 연결: `bash tools/run/run_doosan_real_m0609.sh`
3. RG2 연결: `ros2 launch azas_gripper rg2_trigger.launch.py ...`
4. 카메라 연결: `ros2 launch realsense2_camera rs_launch.py ...`
5. YOLO 인식: `ros2 launch azas_bringup yolo_perception.launch.py`
6. 색상 스캔 자세 이동 후 `bash tools/run/dispenser_color_scan_ros.sh`
7. `bash tools/run/check_one_click_cocktail_ready.sh`
8. `REAL_COCKTAIL_CONFIRM=ENABLE_REAL_COCKTAIL_SEQUENCE bash tools/run/run_cocktail_now_real.sh 1x1,2x2,3x1`
9. 컵홀더 놓기: `python3 tools/run/place_side_grip_cup_in_holder.py ... --execute --confirm ENABLE_CUP_HOLDER_PLACE`
10. 컵홀더 재픽업 후 쉐이킹: `SERVICE_PREFIX=dsr01 bash tools/run/run_rule_based_shake_real.sh`
