# r477 사용자 요청 중지·회수

사용자가 FAST 전환을 위해 잠시 작업 종료를 요청하여 첫 `start_left20_yaw_plus3` 시험에 **기존 소유 explicit STOP을 요청하고 정상 회수**했다. 새 시험·벽 변형·깊은 원인 분석은 시작하지 않았다. WSL은 root에 반환했으며 소유 작업은 남지 않았다.

## 실제 결과와 종료 원인

- 실제 주행은1회다. 첫 WSL 진입 `E_UNEXPECTED`는 주행 미시작으로 보존했고, root의 배포판 복구 후 같은 동결 입력으로 실제1회를 수행했다.
- 신호 출발 후 최대 **41.96 m** 진행했고, 원 평가 종료 진행은41.95 m/랩 길이44.64 m다. 원 `passed=false`, `full_lap=false`, runner exit1을 유지한다.
- 마지막 진전은sim51.447 s였다. 사용자 중지 요청 직전 마지막 모니터는sim58.389 s/무진행6.942 s였으며, 그때까지8초 abort/STOP을 시작한 적이 없었다. 실제 STOP 트리거는 **사용자 정리 요청**이다. 요청·도구 전달 중 무진행8초를 넘었으며 원 평가 최장 정지는 **10.060 sim초**로 남았다. 이를 자연8초 abort나 완주로 바꾸지 않는다.
- frozen helper의 `early_stop.json`에는 일반 정체 사유가 하드코딩되어 있다. 원문은 바꾸지 않고 [user_stop_intent.json](cases/start_left20_yaw_plus3/user_stop_intent.json)으로 실제 요청 원인을 분리했다. 원 abort `mcu_latched_before_stop: software_stop`은 소유 helper STOP이 evaluator STOP보다 먼저 들어간 결과로 보존한다.
- 원 벽 접촉 표본0, 조밀 평면 벽 겹침0, 바퀴 중심 노면 이탈0이다. 실제 모든 접촉이 없었다는 보증이나 실차 증거로 쓰지 않는다.
- `/autonomy/enable=false`와 `/actuation/stop` 성공, `ESTOP_LATCHED`·출력0을 확인했다. 원 STOP 안정0.5 sim초, 변위0.000330 m다. `shutdown_clean=true`, `source_unchanged=true`, 동결1,409입력 변경0·archive 불변이다.
- 후속 소유 감사 exit0, runner768·`GZ_PARTITION=arena_autonomy_test_768` 관련 `remaining_owned_work=[]`다. Gazebo/ROS·소유 WSL 호출을 모두 회수하고 반환했다.

## 재개 위치

41.96 m 정체의 원 plan/current/scan 입력과 trace·simulation log를 그대로 보존했다. 깊은 원인 분석은 하지 않았다. 다음 재개 시 같은 소스/원 기록에서 최초 지속0명령과 현재 경로 판단을 분석할 수 있다. 원 실행 gate는 실제1회에 사용됐으므로 이 조건을 같은 경로로 다시 실행하지 않는다. `wall_surface_inward10`은 미실행이며 새 실행은 root의 후속 판단을 따른다.

근거: [기계 인계](cases/start_left20_yaw_plus3/pause_summary.json), [원 report](cases/start_left20_yaw_plus3/run1/report.json), [소유 PID/partition](cases/start_left20_yaw_plus3/execution_v1/owned_processes_after.json), [입력 불변](cases/start_left20_yaw_plus3/integrity.json), [원자료 SHA](cases/start_left20_yaw_plus3/preserved_evidence.json). source/archive SHA는 [PREPARED.md](PREPARED.md)와 동일하다. 중지 이후 공통 소스·설정·판정선은 수정하지 않았다.
