# closed_loop_v1/run1 중단 기록 (실행기 실패, 제어 결과 아님)

- 실행 보조(Claude Haiku 별칭 `haiku`)가 `run.sh`를 자신의 백그라운드 셸로 띄운 직후 보조 세션을 끝냈다. 시작 01:18:51Z, 마지막 기록 01:19:28Z(`run1/follow_decisions.jsonl` 머리말만), 녹색 전(`WAIT_GREEN`)에서 멈췄다.
- `run.sh`의 마무리 단계까지 가지 않아 `run1.exit`·`run1_surface.json`·`report.json`이 없다. 01:29Z 확인 시 시뮬/launch 잔여 프로세스는 없었다. 보조 세션 종료와 함께 셸이 정리됐다는 것은 시각 일치에 근거한 **추정**이다.
- 이 중단 전까지 기록된 머리말: `execution_config.continuous_forward_pace=true`, `observed_forward_pace=true`, 실행 인자 `evaluation_handoff_continuous_pace:=true`(프로세스 목록). 실행 뒤 입력 해시 재검사 불일치 0.
- 원본은 그대로 둔다. 같은 등록 조건의 1회 재실행은 `../closed_loop_v2/`에서 보조가 전경으로 끝까지 기다리는 방식으로 했다.
