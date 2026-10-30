# 후방 재접근 Gazebo v1 — 사각 전이 미성립

OFF 1회에서 12초 진행 약 4.08 m, 접촉 이벤트 0, 바퀴 중심 도로 밖 0, STOP 이동 약 36.4 mm, 14개 정상 종료와 649/649 판단 동일 재생을 확인했다. 실제 헤더의 `rear_lane_keep_trial=false`와 실행 설정/신호 ACK도 통과했다. 원래 G1 최종 앞차 거리 1.44 m > 1.2 m, 측방 간격 약 139.9 mm < 150 mm 실패는 그대로다.

그러나 취득 시각별 광선·평가 자세 대조에서는 119스캔 중 `alongside` 54·`behind_visible` 12·`ahead` 53, `blind` 0이었다. 모든 자세 보간 구간은 50 ms 이내였다. 따라서 이는 **후방 사각에서 다시 접근하는 장면이 아니며 후보의 효과를 검증한 결과도 아니다.** ON은 실행하지 않았다.

실제 초반 구동 정지/재출발 때문에 4.48초 진행이 0.98 m·상대 종방향 −0.328 m였다. 합성 bicycle보다 자차의 앞서 나가는 시점이 늦었고, 옆차는 마스크 안으로 사라지기 전에 재가속했다. 이 증거를 바탕으로 [v2 사전 등록](../rear_reapproach_gazebo_v2/DRIVE_PLAN.md)에서 상대차 일정만 변경했다.

근거: [실행 보고](drive_off/report.json), [설정·신호 확인](drive_off/trial_overlay.json), [실제 전이](drive_off/rear_transition_audit.json), [바퀴 중심](drive_off_wheel_centers.json), [사전 경로 검사](preflight_result.json). 원시 기록과 전체 재생 상세는 로컬에 보존한다. 거리 25 mm 귀속 허용치는 평가용이며 제어기의 장애물 여유가 아니다.
