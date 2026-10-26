# R0+ 선택형 연결과 핵심 사례 비교

## 코드 선별

- 순수 후보 15개 통과. 후속 통합 225개 중 첫 실행은 224통과·1실패였다.
- 실패: `test_snapshot_covers_transitive_internal_imports_including_per_ray_timing`. 기록 재생기의 새 R0+ import를 G1 계측 도구의 소스 보존 목록에 추가하지 않았다.
- 수정: 보존 목록에 `r0_plus.py`를 추가했다. G1 전용 계측 도구는 R0+ schema 2 기록을 명시 거절하고, 공통 판단 재생기는 두 모드를 구분한다. 기존 schema 1 G1은 유지한다.
- 결과: 수정 후 관련 **225회귀 통과**(`checks_v2.log`). 기존 실패를 없었던 것으로 취급하지 않는다.

## 폐루프 사전 등록

- 후보 `r0plus`, 연결 옵션만 명시 활성화. 기본 R0/G1 및 센서·물성·독립 보호·STOP 기준 유지.
- 첫 사례: `stopped_center`, seed 0, `e3_two_actors_harness_v1.json`.
- 그 다음 사례: `stop_restart`, seed 0, `e3_stop_restart_v1.json`.
- 각 1회. 첫 사례에서 무결성 또는 행동 실패가 나면 두 번째를 자동 실행하지 않고 분석한다.
- 고정 G1과 같은 상한 1 m/s·점별 취득 팽창·간격 법칙. 옆차는 잔디 쪽이며 도로 내 피추월은 별도 미시험이다.
- 두 후보의 실제 경로/센서열은 달라질 수 있다. 저장 입력 대조를 폐루프 동등성으로 해석하지 않는다.

## 결과

두 사례 각 1회 모두 `run_valid/behavior_gates_passed/passed=true`, 입력·노면·독립 보호·STOP·정상 종료·소스 불변을 확인했다. 정지 앞차 진행 1.01989 m·앞차 swept 최소 간격 0.359712 m, 재출발 지연 1.766 s·재개 후 진행 1.51984 m다. 상세 비교는 [E3 보고서](../../../../docs/autonomy/MULTI_ACTOR_E3_2026_10_24.md#6-핵심-앞차-사례의-폐루프-비교)에 모았다.

