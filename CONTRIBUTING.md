# Azas 기여 가이드

> **Doosan M0609 + OnRobot RG2 + RealSense 칵테일 로봇 프로젝트**  
> 7명이 역할별로 나눠 협업합니다. 이 문서를 먼저 읽고 개발을 시작하세요.

---

## 팀 역할 분담

| 번호 | 역할 | 담당 패키지 | 브랜치 접두어 |
|:----:|------|------------|-------------|
| 1 | 시스템 통합 · 런치 | `azas_bringup` | `feature/bringup-` |
| 2 | 태스크 오케스트레이션 | `azas_task_manager` | `feature/task-` |
| 3 | 인식 · 비전 | `azas_perception` | `feature/perception-` |
| 4 | 로봇 모션 | `azas_motion` | `feature/motion-` |
| 5 | 그리퍼 | `azas_gripper` | `feature/gripper-` |
| 6 | 캘리브레이션 | `azas_calibration` | `feature/calib-` |
| 7 | 음성 · AI / 인터페이스 | `azas_voice`, `azas_interfaces` | `feature/voice-`, `feature/interface-` |

---

## 브랜치 전략

### 구조

```
main          ─────────────────────────────  ← 항상 빌드 가능, 직접 push 금지
  │
  └─ develop  ─────────────────────────────  ← 통합 브랜치, PR 타깃
       │
       ├─ feature/perception-yolo-v8        ← 기능 개발
       ├─ feature/motion-side-grasp-tuning
       ├─ feature/gripper-rg2-width-ctrl
       ├─ hotfix/42-tf-timeout              ← 긴급 수정
       └─ chore/update-dependencies         ← 설정 · 문서
```

### 브랜치 명명 규칙

```
feature/{역할}-{짧은설명}     # 기능 개발 (소문자, 하이픈)
hotfix/{이슈번호}-{설명}      # 긴급 버그 수정
chore/{내용}                  # 빌드 · 설정 · 문서 변경
```

**예시**
```bash
git checkout develop
git checkout -b feature/perception-grounded-sam2
git checkout -b hotfix/55-camera-topic-missing
git checkout -b chore/add-pr-template
```

### 규칙 요약

| 규칙 | 내용 |
|------|------|
| `main` 직접 push | **절대 금지** |
| `develop` merge | PR + 최소 **1인** 리뷰 승인 필수 |
| 하드웨어 영향 PR | 최소 **2인** 리뷰 + `safety-checklist` 라벨 |
| 브랜치 수명 | 기능 완료 즉시 삭제 (develop merge 후) |
| main 릴리즈 | develop → main은 팀장 또는 2인 합의 후 진행 |

---

## 커밋 메시지 컨벤션

```
{타입}({패키지}): {짧은 설명}  ← 제목 50자 이내

{필요 시 본문: 무엇을, 왜 변경했는지}

Refs: #{이슈번호}
```

### 타입 목록

| 타입 | 사용 상황 |
|------|----------|
| `feat` | 새 기능 |
| `fix` | 버그 수정 |
| `docs` | 문서 변경 |
| `refactor` | 기능 변화 없는 코드 정리 |
| `test` | 테스트 추가 · 수정 |
| `chore` | 빌드 · 설정 · 의존성 |
| `safety` | 안전 관련 변경 (하드웨어 게이트 등) |

**예시**
```
feat(azas_perception): YOLO 텀블러 직립 방향 감지 추가

bbox 비율 기반 heuristic: height/width >= 1.2 → upright
비직립 감지 시 pose bridge 발행 차단

Refs: #23
```

---

## PR 작성 가이드

1. **제목 형식**: `[패키지명] 짧은 설명`
   - 예: `[azas_perception] Grounded-SAM2 마스크 노드 추가`
2. **체크리스트** (PR 본문에 포함)
   - `colcon build --symlink-install` 통과
   - `colcon test` 통과
   - 하드웨어 영향 코드라면 `docs/safety_checklist.md` 확인
3. **라벨** 붙이기: `perception`, `motion`, `gripper`, `hardware-affecting`, `docs`
4. **리뷰어** 지정: 담당 역할 + 통합 담당(번호 1)

---

## 안전 개발 수칙

```
하드웨어에 영향을 주는 모든 변경은 다음을 반드시 지킵니다.
```

- **실제 모션**은 `--enable-real-motion` 플래그 없이 절대 실행되어서는 안 됩니다.
- `src/azas_bringup/config/calibration.yaml`의 `null` / `확인 필요` 항목은  
  실측 완료 전 절대 수정하지 않습니다.
- **LLM/VLA**는 사용자 의도 · 레시피 선택만 담당합니다.  
  좌표, 궤적, 충돌 판단, 캘리브레이션 값 생성을 절대 수행하지 않습니다.
- 안전 체크 목록: `docs/safety_checklist.md`

---

## 로컬 개발 환경 세팅

```bash
# 1. 환경 소싱
source /opt/ros/humble/setup.bash

# 2. 빌드
cd /home/ssu/Azas
colcon build --symlink-install
source install/setup.bash

# 3. 테스트 (단위)
colcon test --packages-select azas_interfaces azas_voice
colcon test-result --verbose

# 4. 비-하드웨어 점검
bash tools/checks/check_oss_stack.sh
bash tools/checks/verify_control_readiness.sh
```

> 전체 명령어 목록은 **[COMMANDS.md](COMMANDS.md)** 를 참고하세요.

---

## 이슈 등록 방법

1. GitHub Issues → **New Issue** → `task` 템플릿 선택
2. 제목: `[패키지] 짧은 설명`
3. 라벨: 담당 역할 라벨 선택
4. 담당자 지정 후 브랜치 생성

---

## 폴더 구조 한눈에 보기

```
Azas/
├── src/                     # ROS 2 패키지 소스
│   ├── azas_interfaces/     # 공용 메시지 · 서비스 · 액션
│   ├── azas_perception/     # 비전 · 탐지 · 3D 투영
│   ├── azas_calibration/    # 카메라·베이스 · 디스펜서 캘리브레이션
│   ├── azas_motion/         # MoveIt 모션 실행
│   ├── azas_gripper/        # RG2 그리퍼 서비스
│   ├── azas_task_manager/   # 픽앤플레이스 액션 서버
│   ├── azas_bringup/        # 시스템 런치 · YAML 설정
│   └── azas_voice/          # STT · 레시피 매핑
├── tools/
│   ├── checks/              # 비-모션 상태 점검 스크립트
│   ├── smoke/               # 가짜 하드웨어 스모크 테스트
│   ├── run/                 # 현장 실행 스크립트
│   ├── pick/                # 컵 픽 · 그라스프 플래닝 도구
│   ├── perception/          # 인식 데이터 수집 도구
│   └── gazebo_models/       # Gazebo 프리뷰 모델
├── docs/                    # 기술 문서
│   ├── onboarding/          # 신규 협업자용 한국어 입문 문서
│   └── reference/           # 명령어 · 토픽 · 구조 참조 문서
├── models/                  # 3D 모델 파일
├── dependencies/            # 의존성 목록
├── config/                  # (루트 수준 설정)
├── .github/                 # GitHub 템플릿
├── CONTRIBUTING.md          # 이 파일 - 협업 가이드
├── COMMANDS.md              # 명령어 빠른 참조
└── README.md                # 프로젝트 개요
```
