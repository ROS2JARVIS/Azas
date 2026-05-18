# tools/run/

현장 실행 진입점입니다. 새 스크립트를 늘리기보다 아래 대표 명령에 옵션을 추가하세요.

## 규칙

- 실제 모션 스크립트는 `--enable-real-motion` 플래그와 확인 문구가 필수
- 실제 모션은 기본적으로 1회성 실행 (자동 반복 금지)
- MoveIt 플래닝 실패 시 Doosan 직접 명령으로 폴백 절대 금지
- 그리퍼 명령은 관측 모션과 분리하여 명시적으로 처리

## 현장 투입 순서

```
① run_doosan_virtual_m0609.sh   # 가상 로봇 시작
② run_robot_dryrun.sh            # 드라이런 검증
③ check_live_hardware_gates.sh   # 게이트 통과 (checks/ 참고)
④ run_real_robot_test_ladder.sh  # observe/pick 단계형 실로봇 테스트
⑤ run_connected_cup_pick_real.sh # live gate → dry pick → optional one-shot real pick
⑥ run_robot_real.sh              # full real entrypoint
⑦ run_cup_to_dispenser_press_real.sh # 컵을 출수구 아래에 놓고 디스펜서 프레스
```

## 두산 공식 MoveIt2 가상 실행

25장 교안의 시뮬레이션 MoveIt2 실행 명령을 이 스크립트로 감쌉니다. 기본값은 실제 로봇에 연결하지 않는 `mode:=virtual`, `host:=127.0.0.1`, `model:=m0609`입니다.

```bash
bash tools/run/run_doosan_virtual_m0609.sh
```

내부적으로 아래와 같은 launch를 실행합니다.

```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m0609 host:=127.0.0.1
```

이 경로는 MoveIt2/RViz 기반 가상 로봇 검증용이고, Gazebo 물리 시뮬레이터에서 관절 움직임을 보려면 아래의 Gazebo ros2_control 경로를 사용합니다.

## 스크립트 목록

| 스크립트 | 설명 |
|----------|------|
| `run_doosan_virtual_m0609.sh` | 가상 Doosan M0609 시작 |
| `run_doosan_real_no_motion_m0609.sh` | 실제 Doosan 비-모션 연결 |
| `run_robot_dryrun.sh` | 카메라 기반 드라이런 |
| `run_robot_real.sh` | 실제 로봇 모션 실행 |
| `run_connected_cup_pick_real.sh` | 로봇/카메라/RG2 연결 후 strict live gate, dry pick, optional one-shot real pick을 순서대로 실행 |
| `run_cup_to_dispenser_press_real.sh` | 카메라 감지 → 사이드그랩 → 선택 출수구 아래 컵 배치 → 디스펜서 프레스 |
| `run_real_robot_test_ladder.sh` | status → live-gate → dry-run → one-shot real pick 단계형 실로봇 테스트 |
| `run_rule_based_dispenser_then_shake_sim.sh` | RViz에서 side grasp 후 컵을 출수구 앞에 든 상태로 이동하고 high-shake 시뮬레이션 |
| `run_rule_based_dispenser_then_shake_real.sh` | 실제 로봇에서 side grasp 후 컵을 출수구 앞에 든 상태로 이동하고 high-shake 실행 |
| `run_rule_based_shake_real.sh` | 실제 로봇 high-shake 단독 실행 |
| `run_connected_robot_control.sh` | 연결 로봇 제어 통합 |
| `run_supervised_observe_pose.py` | 감독 하에 관측 포즈 이동 |
| `field_no_motion_report.sh` | 현장 비-모션 종합 보고서 |

## Gazebo에서 ros2_control 관절 움직임 확인

교안의 핵심은 Gazebo 안의 로봇 모델이 `ros2_control` 컨트롤러를 가지고 있어야 하고,
그 컨트롤러 토픽으로 목표 관절값을 발행해야 실제 시뮬레이터에서 움직임이 보인다는 점입니다.
기존 `tools/gazebo_models/tumbler_dispenser_m0609_preview.world`는 배치/시각화용 월드라 이 방식으로
관절 명령을 받을 수 없습니다.

터미널 1에서 Gazebo Classic ros2_control M0609 시뮬레이션을 시작합니다.

```bash
bash tools/run/run_m0609_gazebo_ros2_control.sh
```

터미널 2에서 위치 컨트롤러로 확인용 관절 움직임을 보냅니다. 각 구간은 cubic 보간으로 잘게 나눠 발행합니다.

```bash
CYCLES=3 PERIOD_SEC=3.0 RATE_HZ=50.0 bash tools/run/run_m0609_gazebo_demo_motion.sh
```

컨트롤러가 올라왔는지 확인할 때는 아래 명령을 사용합니다.

```bash
ros2 control list_controllers -c /dsr01/gz/controller_manager
```

정해진 한 좌표형 관절 목표만 보내고 싶으면 6축 라디안 값을 직접 보냅니다. 이 명령도 Gazebo 전용이며 실제 로봇에는 연결하지 않습니다.

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/Azas/install/local_setup.bash
tools/run/send_m0609_gazebo_demo_motion.py --once --period-sec 3.0 --rate-hz 50.0 --target 0.0 -0.45 1.25 0.0 1.15 0.0
```

## 로봇 없이 임시 좌표로 컵 이동 + 쉐이킹 실험

실제 로봇/카메라가 없을 때는 아래처럼 임시 컵 좌표와 쉐이킹 중심을 환경변수로 바꿔 RViz/드라이런 경로를 확인합니다.
이 좌표는 코드에 고정되지 않고 launch 인자로만 전달됩니다.

```bash
USE_RVIZ=false \
USE_ROBOT_URDF=false \
ENABLE_IK_PREVIEW=false \
SHAKE_DELAY_SEC=3.0 \
GRASP_X=0.36 \
GRASP_Y=-0.18 \
GRASP_Z=0.05 \
MOUTH_Z=0.22 \
SHAKE_CENTER_X=0.30 \
SHAKE_CENTER_Y=-0.26 \
SHAKE_CENTER_Z=0.62 \
SHAKE_AMPLITUDE_X=0.030 \
SHAKE_AMPLITUDE_Y=0.020 \
SHAKE_AMPLITUDE_Z=0.020 \
SHAKE_CYCLES=1 \
bash tools/run/run_rule_based_dispenser_then_shake_sim.sh
```

실제 카메라 연결 후에는 임시 `GRASP_*`, `MOUTH_*` 값 대신 `/jarvis/tumbler_dispenser/tumbler_pose` 파이프라인을 사용해야 합니다.

## 이미 컵을 잡은 상태에서 쉐이킹만 테스트

컵 검출/그리퍼/디스펜서 캘리브레이션을 건너뛰고 현재 TCP 주변에서 작은 쉐이킹만 테스트할 때 사용합니다.

```bash
GRASPED_CUP_TEST_MODE=true \
USE_CURRENT_TCP_AS_SHAKE_CENTER=true \
SHAKE_APPROACH_HEIGHT=0.010 \
SHAKE_AMPLITUDE_X=0.006 \
SHAKE_AMPLITUDE_Y=0.004 \
SHAKE_AMPLITUDE_Z=0.004 \
SHAKE_CYCLES=1 \
LINE_TIME=1.0 \
tools/run/run_rule_based_shake_real.sh
```

이 모드는 현재 `joint_5`가 `[-135, 135] deg` 범위 밖이면 모션 명령 전에 중단합니다.
또한 `/system/get_robot_state`가 응답하지 않거나 `STATE_STANDBY(1)`가 아니면 확인 문구를 받기 전에 중단합니다.
