# r449 실제 plan 제출·완료 기록 옵션

독립 stage의 **기존 3파일 + 신규 시험 1파일**을 준비하고 Windows 검증을 완료했습니다. 공통 미이관·기본 비활성·주 담당 검토 대기 상태입니다. `g1_complete_plan_trace` 기본은 **false**이며, `true`를 명시하고 등록 runtime을 선택한 경우에만 기록을 추가합니다. 기존 r441의 누락 입력을 소급 복원하는 변경은 아닙니다.

## 변경과 데이터 계약

| 파일 | 변경 |
|---|---|
| `runtime_dual_delivery.py` | 성공한 실제 `worker.offer_plan` 직후 부모 소유 `plan_input`과 실제 `lane.inflight` 식별자를 제출 이벤트로 기록. 실제 수집한 plan 결과는 입력 중복 없이 완료 이벤트로 기록 |
| `runtime_node_backend.py` | 선택 ON에서 실제 자식 생성에 전달한 `evaluator_header` 원문을 한 번 보존하고 기록 옵션을 delivery에 전달 |
| `g1_g2_node.py` | 기본 false인 `g1_complete_plan_trace` 선언. 등록 runtime 없이 true만 선택하면 구성 오류로 거절 |
| `tests/test_runtime_plan_trace.py` | 소유권·원본 보존·폐기·세대/reset·current 반복·기록 오류·실제 Lane spawn과 상태형 재생 검사 |

새 이벤트의 스키마는 다음과 같습니다. 모두 기존 `FollowTrace.emit`과 bounded async writer를 사용합니다. 새로운 파일 writer나 동기 파일 쓰기는 없습니다.

- `runtime_evaluator_header`: `schema=1`, `header=<실제 evaluator_header 사전>`, `scope=submitted_and_collected_plan_only`, `publication_proven=false`.
- `runtime_plan_submitted`: 실제 `stage`, `generation`, `sequence`, `reset_reason`, `submitted_wall_s`, `input=<worker.plan_input>`.
- `runtime_plan_completed`: 실제 `stage`, `generation`, `sequence`, `submitted_wall_s`, `began_wall_s`, `completed_wall_s`, `received_wall_s`, `output`. **input은 중복 기록하지 않습니다.**

제출과 완료는 `(generation, sequence)`로 연결합니다. `*_wall_s`는 lane의 monotonic clock이며 input 안의 센서/시뮬 시각과 섞지 않습니다. 입력의 원 timing·false·host_estimated·nonfinite tag·wheel 부호·원시 배열을 그대로 넘기고, 기존 `FollowTrace.emit`의 freeze가 writer 소유 사본을 만듭니다. 새 `snapshot_call`이나 별도 deepcopy를 추가하지 않았습니다. OFF에서는 새 이벤트 payload 구성·metadata 접근·freeze/copy/emit을 하지 않으며, 옵션 boolean과 분기만 추가됩니다.

**제출은 계산 완료 증명이 아니고, 완료도 적용·발행 증명이 아닙니다.** 부모가 완료 뒤 후보를 만료 폐기해도 제출/완료 기록은 남습니다. 반대로 세대 변경으로 기존 `DualSlots.collect`가 버린 늦은 결과에는 완료 이벤트를 만들어 붙이지 않습니다. 제출만 있고 완료가 없는 경우를 임의의 성공 출력으로 채우면 안 됩니다. STOP/epoch/generation/reset/current 반복과 발행 경계는 그대로입니다. `runtime_dual_state.py`, 정책, `FollowTrace`, writer는 수정하지 않았습니다.

## 검증과 원 실패

Windows Python 3.14.6에서 별도 validation 사본과 격리 pytest 9.0.2를 사용했습니다. pytest는 이 산출물의 `test_dependencies/`에만 설치했으며 전역 Python/공통 저장소 환경은 변경하지 않았습니다.

첫 관련 5파일 실행은 **71 통과·3 실패**였습니다. 실패 3개는 신규 테스트의 가짜 노드가 `has_parameter`를 제공하지 않아 실제 선택 함수 진입에서 실패한 것입니다. 원 [실행 로그](verify_windows_v1/execution.log)와 [영수증](verify_windows_v1/receipt.json)을 보존했습니다. stage 구현 3파일은 그대로 두고 테스트 fixture만 고친 뒤 신규 파일 11검사만 재실행하여 **11 통과·실패 0·skip 0**을 확인했습니다([최종 로그](verify_windows_v2/execution.log), [영수증](verify_windows_v2/receipt.json)). 기존 관련 63회귀는 첫 실행에서 통과했고 재실행하지 않았습니다. 즉 최종 범위는 **기존 63 + 신규 11 = 서로 다른 74검사**이며, 74개가 한 번에 통과했다고 기록하지 않습니다.

신규 검사에서는 다음을 확인했습니다.

- 기본 OFF가 capture 필드를 읽거나 새 emit을 호출하지 않음. ON/OFF의 실제 부모 폐기 결과와 delivery counts 동일.
- 실패한 occupied offer에는 새 기록 없음. 성공한 offer의 부모 소유 입력은 호출자 mutation과 이후 부모 후보 폐기/변경에도 원문 유지.
- false/host_estimated/nonfinite tag/음수 wheel 값 보존. 실제 header도 입력 사전의 이후 mutation과 분리.
- 세대 변경 뒤 늦은 결과는 기존대로 버림. 다음 제출의 실제 reset 이유/세대/sequence 보존.
- 동일 plan에 대한 반복 current 결과가 제출/완료 이벤트를 중복 생성하지 않음.
- freeze 오류와 writer 오류가 제어 결과를 바꾸지 않고 기존 footer `capture_errors`/`failed`에 각각 드러남.
- 실제 G1 옵션 선언의 false 기본 및 등록 runtime 선택 조건을 AST 경계에서 검사. 실제 ROS 노드 생성 검증으로 확대하지 않음.

## 실제 Windows spawn 근거

실제 `RuntimeDualProcess`/`Lane`과 작은 결정론 stateful fake evaluator로 계획 3개를 제출·계산한 뒤, 부모가 모두 만료 폐기하도록 했습니다. **제출 3·완료 3**이 남았고 기록된 header, generation 전환, 실제 `reset_reason`, 입력만 사용한 재생 결과가 **3/3 exact**였습니다. 세대/sequence는 `(0,1) → (0,2) → (1,3)`, reset 이유는 `null → null → decision_source_expired`이며 누적 상태 출력은 `1 → 3 → 3`입니다.

[작은 원 trace](spawn_evidence_v2/trace.jsonl)와 [spawn 영수증](spawn_evidence_v2/spawn_receipt.json)을 별도로 보존했습니다. plan/current 자식 PID **44044/54116**은 모두 exit0, 강제 종료 0, 잔존 0입니다. 최종 pytest PID 38936은 exit0이며 첫 실패 실행 PID 35280의 exit1도 보존했습니다. `mp.active_children()`와 후속 [Windows PID 조회](windows_pid_audit.json) 모두 잔여 0입니다. 두 validation 사본의 입력 변경 0, 격리 밖 package import 0, 공통 원본 SHA 변경 0입니다. WSL·Jetson·ROS·센서·장치·주행은 사용하지 않았습니다.

## 통합 검토 사항과 한계

[changes.json](changes.json)에 정확한 원 SHA와 stage SHA가 있고, [apply.patch](apply.patch)가 4파일 적용용 diff입니다. patch SHA256은 `360f90cdc516e88c9078a96e23869dfbf8925a6a16e4a3f05ece73c90ee13c7d`입니다. 검토 후 주 담당이 원 SHA를 확인하여 이관합니다. 이번 작업에서 공통 파일/운용 설정은 바꾸지 않았습니다.

선택 ON에서는 원 전체 입력을 추가로 freeze·queue에 넣는 진단 비용이 있으며 그 비용이나 실제 ROS 주기를 계측하지 않았습니다. queue drop/쓰기 실패/종료 불완전은 기존 footer 또는 footer 부재로 드러나므로, 이후 재생은 먼저 연속 sequence와 footer의 오류·누락 0을 확인해야 합니다. header와 실제 제출 순서로 evaluator를 만들고 generation이 바뀔 때만 해당 제출의 reset 이유를 전달해야 하며, 별도 현재 policy를 조합하여 원 설정을 바꾸지 않아야 합니다.

이번 exact 재생은 **작은 fake evaluator의 전달·소유권·상태/reset 기록 계약**을 확인한 것입니다. native 실제 GUARD 정책의 전체 저장 재생이나 실시간 주행을 입증하지 않습니다. 기존 일반 `follow_trace.replay_events`는 모든 runtime 이벤트를 다루는 도구가 아니므로 새 전체 runtime parser로 확장하지 않았습니다. 다음 선택형 기록을 소비하는 분석은 위 이벤트/식별자 계약을 사용해야 하며, 기존 r441 입력 부족 판정은 그대로 남습니다.
