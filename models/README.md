# models/

텀블러·디스펜서 3D 모델 파일 모음입니다.  
시각화 및 디지털 트윈용으로, 실제 충돌 기하로 사용하지 마세요.

---

## 파일 목록

| 파일 | 설명 |
|------|------|
| `azas_tumbler_shaker.obj` | 텀블러/셰이커 근사 모델 |
| `azas_dispenser_single.obj` | 단일 디스펜서 (투명 병 + 검정 펌프 헤드) |
| `azas_four_dispenser_row.obj` | 4구 디스펜서 배열 (85 mm 간격) |
| `azas_tumbler_dispenser_preview.obj` | 4구 디스펜서 + 텀블러 프리뷰 통합 |
| `azas_tumbler_dispenser_preview.usda` | Isaac Sim용 스테이지 (OBJ 에셋 참조) |
| `azas_models.mtl` | OBJ 공용 재질 파일 |

---

## 치수 참조값

**텀블러**
- 직경: 75 mm
- 뚜껑 포함 높이: 170 mm
- 뚜껑 제외 몸체 높이: 140 mm

**디스펜서**
- 병 폭: 58 mm
- 병 높이: 275 mm
- 입구 내경/외경: 18 mm / 28 mm
- 튜브 길이: 205 mm
- 튜브 내경/외경: 7 mm / 8.5 mm
- 펌프 헤드 총 길이: 195 mm
- 노출된 펌프 부분: 117 mm

---

## 사용 방법

**RViz** — `visualization_msgs/Marker` (`type=MESH_RESOURCE`)로 OBJ 메시를 로드할 수 있습니다.  
**Isaac Sim** — OBJ 파일 직접 임포트 또는 `azas_tumbler_dispenser_preview.usda` 파일 열기

---

> 이 모델들은 시각화/디지털 트윈 근사값입니다.  
> 실제 로봇 실행의 충돌 기하로 사용하려면 물리 환경에서 직접 실측 후 교체하세요.
