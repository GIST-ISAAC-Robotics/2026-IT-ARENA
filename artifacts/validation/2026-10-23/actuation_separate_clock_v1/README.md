# 별도 시계의 제한 프레임 교환 검사

- [실행 스크립트](run_checks.sh), [회귀 로그](tests.log): **299 passed·0 skipped·exit 0**. [소스 해시 차이](source_hash_diff.txt)는 0 B이며 당시 `source_snapshot.tar.gz`를 보존했다.
- [감사 결과](audit/report.json): **8/8 조건이 예상 결과와 일치**했다. 실제 portable C++ 수신/송신기를 PC 자식 프로세스로 호출했으며 물리 장치·운영 ROS에 연결하지 않았다.

| 조건 | 시험기가 DRIVE 생성 | STOP 직전 수신기 | 최종 |
|---|---|---|---|
| 기준 | 예 | ACTIVE | ESTOP_LATCHED·목표 0 |
| MCU 시계 −200 ppm | 예 | ACTIVE | ESTOP_LATCHED·목표 0 |
| MCU 시계 +200 ppm | 예 | ACTIVE | ESTOP_LATCHED·목표 0 |
| 편도 지연 비대칭 반전 | 예 | ACTIVE | ESTOP_LATCHED·목표 0 |
| 보수적 전방 지연 상한 증가 | 아니요 | ARMED | ESTOP_LATCHED·목표 0 |
| 판단 120 ms 지연 | 아니요 | FAULT_LATCHED | ESTOP_LATCHED·목표 0 |
| 원천 입력 120 ms 경과 | 아니요 | ARMED | ESTOP_LATCHED·목표 0 |
| ARM ACK 유실 | 아니요 | ARMED | ESTOP_LATCHED·목표 0 |

460800·8N1 직렬화, 추가 host 송신 큐 없음, 합성 피드백과 `output_ready=true`를 **시험 가정**으로 사용했다. 선언한 최대 프레임 직렬화·편도 지연 상한과 왕복 시각 구간으로만 후보를 판단하며 참 시계는 시험기 평가에만 쓴다. RTT/2를 편도 지연으로 대입하지 않았다.

한 연결에서 ARM 1개→새 세대 offer/ACK→DRIVE 최대 1개→STOP의 **제한 교환**이다. 연속 50 Hz 처리량, 실제 요청 상관 처리기, 실물 주파수/지연 상한, 실제 엔코더·모터나 출력 감시를 입증하지 않는다. ACK가 없는 조건은 시간 조건이 맞더라도 시험기가 DRIVE를 만들지 않았다. 모든 후보의 `ready_to_drive=false`, `runtime_connected=false`, `hardware_executed=false`를 유지한다.

[구동 계약 §24](../../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#24-1023-후속--별도-시계의-제한-프레임-교환)에서 결과와 후속 경계를 설명한다. 검사 수는 다른 실행과 중복되므로 합산하지 않는다.
