# run61 — B3 중간 STOP·같은 경기 재개: 원본 판정 실패

[M4 사전 등록](../M4_SAME_RACE_PLAN.md)대로 한 번 실행했다. 동작 자체의 확인과 등록한 결합 판정의 실패를 구별한다.

- **확인:** 실제 약 1.04 m/s 구간에서 명시 STOP. 중간 밀집 정지 창 최대 속도 0.000556 m/s, 확인 후/재ARM 전 1초의 실제 속도 0. 출발 사실·경기/영상 세대 유지, 새 ARM 뒤 0.375 s에 실제 재출발·3.073 m 진행.
- **실패 1(평가기):** CLEAR 전후 MCU 원본 시각 7.620 s에서 generation 2 STOP → 3 DISARMED 순으로 왔지만, 평가기가 pose 수신 시각 7.619 s를 끝 경계로 사용해 STOP 표본 하나를 잘못 제외했다. 원본 `only_registered_intermediate_latch=false`를 보존한다. 같은 MCU 스트림의 시각·세대·순서로 고친 사후 대조는 run62에 별도 보존하며 새 주행이 아니다.
- **실패 2(결합 행동):** 기존 앞차 정지창의 연속 HOLD가 0.912 s로 등록된 1 s에 미달했다. 이번 운영 STOP으로 접근 시점이 달라졌지만 기준을 바꾸거나 원래 판정을 성공으로 소급하지 않는다. 앞차 재개 후 0.537 s 재출발·재개 뒤 1.840 m, 나머지 11개 행동/기하 gate는 참이다.
- **종료:** 기존 마지막 STOP/밀집 정지·정상 종료·잔존 0·소스 불변. 원래 `no_mcu_latch_before_stop=false`는 의도적으로 넣은 중간 STOP 때문이며 그대로 남긴다. 재생 623/623·노면 9,709표본 이탈 0·130파일 전후/후처리 일치.

원본 `passed=false`, `m4_operation.passed=false`, `run_valid=true`다. `before_stop.passed=false`는 정지 검사 함수를 재사용한 진단 필드로, 그 구간에 차가 움직였다는 뜻이며 **실패 gate가 아니다**. 실제 `actual_motion_before_stop=true`다. 후속 평가 출력에서는 이 오해를 막도록 움직임 진단 이름을 분리했다.

run62는 관련 135회귀를 통과했다. 사후 재평가에서 등록 래치 조건은 통과했지만 기존 HOLD 미달 때문에 종합 판정은 여전히 실패다. 입력 원본은 바뀌지 않았다.

[원본 보고서](run/report.json), [명령](command.txt), [M4 사후 재평가](../run62_m4_clock_review/run61_reassessment.json). 실제 MCU·공식 출발등·새 경기 재출발·전체 랩 증명이 아니다.