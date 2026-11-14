# r524 운영 compact trace 후보 — stage READY

2026-11-14, GPT. **공통 두 파일의 stage 패치만 준비했다. 명시 `g1_trace_detail=compact_v1`에서 매 판단의 큰 센서/상태 기록을 작은 사건 요약으로 바꾸며, 기본 `full` 기록은 원본과 byte 동일·재생 통과를 확인했다.** 공통·Jetson·WSL·설치 변경과 성능 재시험은 없다.

## 근거와 변경 경계

[r522](../jetson_traceoff_continuity_r522_v1/RESULT.md)의 trace OFF 단1회에서 active 양수 공백이1.326→.604초로 줄었다. 다만 직접 control85.22ms와 `motion_stale`가 남았고 계산 건수/순서도 달랐다. 이 결과는 상세 trace를 줄일 근거이며, r524의 실제 지연 개선량이나 전체10Hz 보장이 아니다.

[candidate.patch](candidate.patch)의 대상은 다음 두 파일뿐이다. 원본은 [source_reference.json](source_reference.json)과 `baseline/`에 보존했다.

- `follow_trace.py`: `detail='full'` 기본 유지. compact에서는 `record_call()`이 **`snapshot_call()`보다 먼저** 작은 scalar 입력·출력 요약을 기록한다. `emit('call', ...)`도 scan/history/path/큰 하위 dict를 순회하기 전에 요약한다. `runtime_plan_submitted/completed`는 식별자·시각·출력 요약으로 바꾸고 kind에 `_summary`를 붙인다.
- `g1_g2_node.py`: `g1_trace_detail`을 생성자에 전달한다. parent current-recheck 경로의 **기록만을 위한** `freeze(current_result)`는 compact에서 생략한다. 계산용 `snapshot_call()`·IPC 소유 사본·current 판정·기존 `status_detail()`은 유지한다. `/drive` 발행 경계를 지난 뒤 상태전이/명령 감소와 explicit STOP의 작은 사건을 남긴다.

`LocalPursuit.command()`가 `record_call()`을 부르기 전에 별도 snapshot freeze를 하는 구조는 아니다. 원 비용은 `record_call` 내부 snapshot freeze와 이어지는 `emit` freeze, G1/G2의 큰 evidence freeze에 있다. `record_call`만 빈 함수로 바꾸면 후자의 비동기 경로가 남으므로 두 진입을 함께 다뤘다. evidence dict를 조립하는 작은 shallow 작업은 남겨 제어 분기를 재구성하지 않았다.

새 thread/logger/설정 모듈은 없다. 기존 `QueuedLog` writer 하나와 유실/오류 footer를 그대로 쓴다. 운영 패널, `/autonomy/status`, 독립 guard 상태·제동 근거, `WallFollow.state_log`, RaceOps 로그는 수정하지 않는다.

## 남는 사건과 생략 범위

| 항목 | compact 기록 |
|---|---|
| header/footer | 기존 설정·제어 모드·scope 유지, `trace_detail=compact_v1`, `replayable=false` 추가 |
| 각 판단 | 원 reference/epoch/time-policy/측정속도·선택 side, 명령 속도/조향, 고정된 이유/behavior/handoff 필드 |
| current 정지·결과 폐기 | current 속도/이유/허가 false, delivery 폐기사유, 작은 compute 식별자·시각 |
| STOP/RESET | 기존 reset 이벤트 그대로, explicit stop의 실제 발행 이후0 명령 사건 |
| 발행 상태전이·감속 | 발행 여부·상태·속도·조향·`command_decreased`, 변함없는 정상 control은 추가 사건 생략 |
| 독립 작은 사건 | `parallel_discard`, `route_publication`, 정책/환경 사건 그대로 |
| 큰 반복 자료 | scan, history 배열, 전체 path/region/상태 트리 생략 |

`control_event.at_s`는 발행 뒤 관측한 controller clock이며 원 센서 stamp를 대체하지 않는다. `speed_mps`는 기존 `last_speed`의 명령값이고, `command_decreased`는 실제 발행 경계를 지난 명령 감소를 뜻한다. **독립 guard 통과·MCU 적용·실측 감속 증거는 아니다.** 이 경계는 사건의 `scope=drive_publication_not_guard_or_mcu`에도 기록했다. 독립 guard의 실제0 출력/사고 원인은 기존 별도 로그를 계속 사용한다.

compact는 입력 전체가 없으므로 상세 재생 입력으로 사용하지 않는다. `replay_events()`가 명시적으로 거절한다. full의 schema/header/footer와 원 replay 자료는 바꾸지 않았다. `g1_complete_plan_trace=true`와 함께 쓰더라도 compact에서는 plan **summary**이며 full plan 입력 보존으로 발표하지 않는다.

## 선택·통합 조건

root 검토 후 `g1_g2_handoff_trial`의 기존 YAML `local_pursuit.ros__parameters`에 `g1_trace_detail: compact_v1`을 명시하거나 CLI에 `-p g1_trace_detail:=compact_v1`을 추가한다. 기존 `g1_trace_path` 절대경로, 배타 생성, controller mode·typed execution·corner/scope 검증은 그대로 필요하다. 빈 경로나 trace OFF 선택을 새로 허용하지 않는다. `full` 또는 옵션 생략은 기존 동작이다.

다른 controller 실행 파일의 옵션 전달은 이번 범위에 넣지 않았다. 공통 통합 시 [영수증](verification_receipt.json)의 baseline SHA와 현재 두 파일을 대조하고, root가 포함하려는 최신 r508/기타 수정과 함께 검토한다. 현재 stage 기준은 `g1_g2_node.control()`의 기존 `motion.drain_pending()`도 포함하며 이를 제거하지 않았다. r517/r519 결과는 수정하지 않았다.

수용 기준은 기본 full 자료 보존, compact의 대형 payload 미접근, STOP/RESET/폐기·발행 감소 근거 유지, 기록 오류가 제어값을 바꾸지 않음, 기존 제어 snapshot/발행 계약 불변이다. 이번 PC 검사는 이 범위를 확인하며 실제 성능은 미확인이다.

## 검사 결과

- 신규 **12개 통과**: [첫10개](stage_tests.log), [설정 전달·선행 freeze 분기2개](focused_followup.log). 원 full과 byte 동일/재생, compact snapshot 함수 미호출, 지원되지 않는 큰 하위 객체를 건드리지 않음, 값의 복사 소유권, current 정지·폐기사유, 발행 이후 상태/감속/STOP, 경로/모드 검증 및 기록 실패를 확인했다. 노드 연결은 실제 메서드 AST를 실행한 PC 단위 검사이며 ROS 생성자 실행은 아니다.
- 기존 관련 **36개 중35개 통과**: [초기34통과 로그](existing_tests.log), [Windows 실제 파일 runner로 spawn1개 통과](focused_followup.log). snapshot 소유권, full plan 기록, 발행 acknowledgement/입력 snapshot 계약을 재사용했다. 추가 성능 계측은 없다.
- 남은1개는 [baseline에서도 재현된 기존 테스트 문제](baseline_test_limitation.json)다. `test_actual_node_option_defaults_off_and_requires_registered_runtime`가 AST에서 인접2문장을 가져오지만 현재 두 번째 문장은 vehicle-marker 검사여서 fixture의 `vehicle_marker_channel` 부재로 실패한다. stage가 추가한 분기가 아니며 공통 테스트를 고치거나 통과로 표기하지 않았다.
- 초기 spawn 실패는 Windows `<stdin>` 경로를 child가 열지 못한 실행 방식 문제였다. 원 실패를 보존하고 [파일 runner](run_stage_tests.py)로 해당1개만 재실행해 통과했다. 전체 검사 반복은 하지 않았다.

**판정: stage 통합 검토 READY. compact 기본 채택·설치·Jetson 성능 확인은 아직 하지 않았다.**
