# 광선 원점 모호함의 보수적 계약 정정

`run_checks.sh` 1회: 전체 **3,176 pytest**, ON 3,001/OFF 2,837판단 불일치 0, 정상 종료 0. `source_hash_diff.txt`는 비어 있고 각 profile의 입력·소스 불변 검사가 통과했다. 각 profile의 source snapshot은 시험 당시 소스다.

- ON total p50/p95/max: 5.252607/8.607223/22.857397 ms, >20 ms 5건.
- OFF total p50/p95/max: 5.064025/7.744849/16.962266 ms, >20 ms 0건.
- 어느 광선 원점과든 1e-9 m 이내인 질의는 해당 장면의 깊이 지지를 거절한다. 방향이 정의되지 않는 수치 경계이며 장애물 존재 판정이 아니다.
- 기존 저장 판단 동등성 검사다. 실제 ROS/Jetson 마감, 새로운 코너 진행, 실물 안전 시험이 아니다.

[기준 보고서](../../../../docs/autonomy/G1_OBSERVATION_CONTRACT_2026_10_21.md)
