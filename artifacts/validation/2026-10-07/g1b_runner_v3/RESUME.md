# G1-b 재개 실행 상태 (최종, 2026-10-07)

- 제어기 `local_follow.py` `fddc808b…` 불변. 최종 평가판 v4 `g1b_eval_v4_fresh_stop_clock`, 소스 고정 [`g1b_unit_v8/source_freeze.json`](../g1b_unit_v8/source_freeze.json), 회귀 472 통과.
- 완료: v2 평가판 G1 9개(seed 1 등속 실패 보존), v3 4개(요청 시각 0.113~0.118 s 오래됨, 보존), v4 4개(G1 seed 1 등속 통과, R0 3개 실행 유효). 모든 배치 `BATCH_DONE`, 시험 잔존 프로세스 0.
- 감사: v4는 [`g1b_source_audit_v5`](../g1b_source_audit_v5/), v3는 [`g1b_source_audit_v4`](../g1b_source_audit_v4/)(현재 트리 비교 실패는 뒤 평가기 변경 때문이며 `registered_vs_freeze` 보조 검사는 통과). R0의 coverage 경계/HOLD 검사는 G1 전용이라 비적용.
- 남은 과제: R0 정지 앞차 접촉·실행 간 차이 조사(보호층 변경은 별도 결정). 진행 중인 실행·예약 없음.
