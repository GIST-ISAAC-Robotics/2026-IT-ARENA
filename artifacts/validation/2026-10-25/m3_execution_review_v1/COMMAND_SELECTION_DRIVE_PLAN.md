# 실제 명령 후보 재선택 — 후속 제한 주행 사전 등록

2026-10-25, run9 원인 분석과 run11 개루프 대조 후 등록. run9는 진행·간격·평가 모드 표식 누락 실패로 보존한다. 같은 실패의 단순 재실행이 아니라 실제 명령의 관측 거절 뒤 남은 후보도 검사하는 구현의 비교다.

## 조건과 질문

- run9와 동일 `slow_lead`, seed0, 왼쪽 도로 내 옆차, 경계 `corner_distance_candidate`, `per_ray_candidate`, 회랑 보완 ON, 최근 관측 OFF. 차량·센서·물성·속도·보호층·간격·재개 확인 횟수·잔디 규칙 모두 동일.
- 차이는 `--m3-command-fallback` 명시 ON과 평가 trace에 원래 출력된 M3 표식을 보존하는 수정이다. 기존 기본값은 OFF다.
- 계획 통과/실제 조향 거절 뒤 직진을 선택하면 중앙 복귀 거절에 의한 가속·정지 반복이 줄고 옆차 통과 뒤 전진하는가? 측방 간격 미달은 별도이며 재개 개선으로 해결됐다고 하지 않는다.
- 저장 입력 비교에서 ON 비용이 늘었으므로 MCU/보호 래치·입력 나이·판단 비용과 정상 종료를 확인한다. 보호 기준을 낮추거나 갱신 기한을 늘리지 않는다.

## 실행과 판정

관련 회귀·빌드·설치본 확인 후 왼쪽 옆차 **1회만** 실행한다. 실패 시 같은 설정 반복 금지. 원본 비교선과 별도 97 mm·최대 두 바퀴 잔디 판정은 [첫 주행 계획](LIMITED_DRIVE_PLAN.md) 그대로다. 명시 STOP·밀집 정지·소스 불변·잔존0·판단 동일 재생을 요구한다. 원시 판정은 덮어쓰지 않는다.

```bash
python3 -B scripts/validate_g1_traffic.py --case slow_lead --seed 0 --mode m3 --m3-command-fallback --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --actor-overlay config/tests/e3_onroad_encounter_v1.json --output artifacts/validation/2026-10-25/m3_execution_review_v1/run13_selection_closed_loop/run
```

종료 후 판단 재생·노면 평가 각 1회. 결과 파일 생성 후 종료 영수증이 불분명해도 중복 실행하지 않고 기존 세션·로그를 확인한다. 후방 사각·실물 제동·연속 외곽 보증은 이 시험 범위 밖이다.
