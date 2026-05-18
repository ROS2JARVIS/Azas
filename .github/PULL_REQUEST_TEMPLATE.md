## 변경 내용

<!-- 무엇을 왜 변경했는지 간단히 설명하세요 -->

## 변경 타입

- [ ] feat (새 기능)
- [ ] fix (버그 수정)
- [ ] refactor (리팩토링)
- [ ] docs (문서)
- [ ] chore (빌드 · 설정)
- [ ] safety (하드웨어 안전 관련)

## 관련 이슈

Refs: #

## 테스트 방법

```bash
# 빌드 확인
colcon build --symlink-install --packages-select <패키지명>

# 테스트 실행
colcon test --packages-select <패키지명>
colcon test-result --verbose
```

## 체크리스트

- [ ] `colcon build --symlink-install` 통과
- [ ] `colcon test` 통과
- [ ] 관련 문서(docs/) 업데이트
- [ ] 하드웨어 영향 코드라면 `docs/safety_checklist.md` 확인 완료
- [ ] 하드웨어 영향 코드라면 리뷰어 2인 이상 지정

## 하드웨어 영향 여부

- [ ] 이 PR은 실제 로봇·그리퍼·카메라에 영향을 줍니다 → `hardware-affecting` 라벨 추가
- [ ] 이 PR은 하드웨어에 영향을 주지 않습니다
