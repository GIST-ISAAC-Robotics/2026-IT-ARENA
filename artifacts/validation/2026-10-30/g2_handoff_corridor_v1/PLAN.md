# G1/G2 중앙 통로 자동 전환 사전 대조

2026-10-30. 별도 G2의 signed 대기는 복합 오차 Gazebo1회에서 통과·복귀했다. 현재 자동 전환에는 상위 음수 초기화와 고정 진입 비교가 남아 있다. 먼저 운영 코드를 바꾸지 않고 `BypassExecutionConfig(corridor_policy='corridor_v2_signed_hold')`만 넣어 기존 두 준비 정책을 비교한다.

중앙 정지/이동 후 정지 각16초+STOP, 순차10Hz·28카운트/15:1/65 mm와 기존 combined 오차 생산, 독립 보호 함수·이상 bicycle 평가를 유지한다. `observed_entry_space_v1`+예약과 `observed_entry_relevance_v1`+예약 없음은 서로 다른 전략의 진단 대조이며 단일 변수 효과로 해석하지 않는다. 양쪽 모두 trim_before_compare_v1·known_hold_v1이다.

진입 전 관측 누적·진입 검사·거절 이유·실제 진행/접촉/바퀴노면/복귀·기록 재생을 함께 남긴다. 이 시험에는 물리 바퀴 역회전이 없으므로 상위 signed 전이는 별도 저장 입력/반례로 확인해야 한다. 기본 활성화·새 물성·보호 우회·정답 입력은 없다. 동일 실패 무근거 반복과 성공할 때까지 수치 조정은 하지 않는다.
