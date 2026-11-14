# r536 — 최신 current 전달 경로의 좁은 감사

**추가 production 수정의 근거가 되는 새 반례는 확인하지 못했다.** 현재 소스에는 controller6 ms drain, 실제 dt를 사용하는 fresh current의 20 ms 조기 전달, compact 기록이 연결돼 있다. 기존 100 ms 대기 이외에 동일 current를 다시 기다리게 하는 별도 고정 gate·옛 stamp 재발행은 확인되지 않았다. 다만 이 조합의 실제 처리율·잔여 command stale는 **Jetson에서 아직 측정하지 않았다.** 충분한 실시간 성능을 확인했다는 뜻은 아니다.

Windows에서 저장 r523·r526와 필요한 공통 소스만 읽었다. Jetson 접속/조회·WSL·ROS 실행·공통 편집은 하지 않았고 기존 r526 회귀를 반복 실행하지 않았다. [현재 소스 SHA/함수 위치](manifest.json), [저장 자료 감사](saved_audit.json), [분석 코드](audit_saved.py).

## 현재 코드에서 확인한 전달 경계

| 구간 | 현재 동작과 판단 |
|---|---|
| 입력 소비→평가 | G1G2 controller는 motion drain6 ms 후 runtime control로 들어가 실제 `now`와 출력 dt를 읽는다. guard 기본2 ms·64개 및 원 stamp 검증은 유지한다. |
| current 준비→poll | 완료 알림은 원 결과를 collect한 뒤 동일 control을 호출한다. `RuntimeDualDelivery.poll`의 재collect는 이미 비운 lane에서 대기하거나 같은 결과를 재수신하는 동작이 아니다. `prepared`는 source/command/geometry·epoch·previous angle·proof를 확인한다. |
| 20 ms/100 ms | current는 새 reference, 20 ms 하한, 실제 출력dt의 3 rad/s 예산을 충족하면 기존100 ms gate를 통과한다. dt가 없거나 조향 예산이 부족한 경우, unsupported/nonretained plan의 기존100 ms 동작은 유지한다. 이것을 모든 결과의 무조건20 ms 발행으로 설명하지 않는다. |
| poll→발행 | G1G2는 `command_reference_s=checked.output.reference_s`를 선택하고 WallFollow가 원 stamp로 발행한다. command가 `None`이면 발행하지 않으며 옛 명령 stamp를 새로 갱신하지 않는다. 최종 조향 clamp에는 control 시작의 같은 dt를 사용한다. |
| ack→소비 | 같은 callback에서 실제 발행 뒤 원 expected reference/speed/angle을 검증하고 commit한다. ready는 한 번 소비된다. 추가100 ms sleep/timer 예약은 없다. STOP/reset은 미발행 후보를 취소한다. |
| 다음 계산 | 다음 current는 새 scan source/plan token이 필요하다. 동일 source를 반복 검사해 stamp만 새롭게 만드는 경로는 없다. 다음 계산 완료·통신 지연까지 보장하는 계약은 아니다. |
| compact | 명시 `g1_trace_detail=compact_v1`일 때 call을 scalar summary로 축소하고 실제 control·STOP 사건을 유지한다. 파라미터 기본값은 여전히 `full`이므로 소스 통합만으로 compact가 자동 선택되지는 않는다. 상세 plan trace는 별도 설정이다. current 준비/대기/모든 폐기 이력이 compact만으로 완전 보존되는 것은 아니다. |

`last_request_time_s`/`last_application`은 실제 `/drive` 발행 뒤 status/ack 처리가 끝난 시각으로 갱신될 수 있다. 따라서 **발행 뒤 CPU 작업이 실제로 얼마나 있는지는 재개 계측 항목**이다. 저장 r523에는 publisher 진입과 guard command callback 경계가 없어 이것을 추가 병목으로 확정하거나 임의 수정하지 않았다. 같은 이유로 command 관찰자 수신 지연 전체를 runtime gate 비용으로 간주하지 않는다.

## 저장 timeline으로 확인한 것과 남는 한계

[r526 저장 분석](../fresh_current_delivery_r526_v1/RESULT.md)의 원 3파일 SHA가 r523 자료와 동일함을 확인했다. 양수 `/drive` 176개 모두 원 current reference와 정확히 대응했다. current 회수→관찰자 명령 수신 중앙 **58.113 ms**, p95 **79.400 ms**, 최대 **104.534 ms**, 이전 명령 관찰 후100 ms 이전에 회수된 결과169개라는 값은 **r526 적용 전 r523**의 사실이다. 최신 조합에 이 지연이 그대로 남는다고 주장하지 않는다.

미발행 current 6건을 좁게 분리했다.

| current sequence | 원 reference(s) | 저장 기록에서 확정 가능한 내용 |
|---:|---:|---|
| 133 | 15.361354 | `new_plan_superseded_current_check` |
| 151 | 16.400453 | `new_plan_superseded_current_check` |
| 174 | 17.662622 | 회수 기록에 discard 사유 없음; ready 전이/원 출력 미기록으로 미분류 |
| 190 | 18.582826 | `new_plan_superseded_current_check` |
| 320 | 26.300075 | 회수 기록에 discard 사유 없음; ready 전이/원 출력 미기록으로 미분류 |
| 337 | 27.180791 | `new_plan_superseded_current_check` |

4건은 새 계획에 따른 교체가 명시돼 있지만 당시 새 계획의 전체 intent/출력이 없으므로 정상·불필요 교체 여부를 더 단정할 수 없다. 나머지2건도 이유 없는 버그로 판정하지 않는다. 현재 코드는 같은 양수 의도 전이를 보존하는 경로와 상태/위험 변화 시 revision을 바꾸는 경로를 구분한다. 저장된 이전 실행의 결과를 최신 정책 재생 결과로 소급하지 않았다.

[r523](../jetson_drain_budget_r523_v1/RESULT.md)의 command stale173회·원 명령 나이200.045–247.407 ms, 출발 후 관찰된0 목표3.482초(17.95%)도 이전 조합의 기준값으로 남긴다. r526 이후 스케줄·계산량이 달라질 수 있고 원 ready/실제 dt가 없어 이 자료만으로 감소량을 수치 예측하지 않는다.

## 재개 시 한 번만 측정

[단일 통합 계측 레시피](MEASUREMENT_RECIPE.md)를 준비했다. 최신 설치에서 같은20초 입력·원시 시각·합성 camera/detector·독립 guard 조건으로 한 번 측정하며, current token의 **완료→허가→실제 발행→guard 수신/평가**를 scalar로 연결한다. 실제 출력·TTL·검사를 바꾸지 않는다.

**r523 node_runner를 그대로 재사용하면 안 된다.** 옛 실행기의 직접 drain은 최신 공통 drain과 중복되고, proof bootstrap과 `TraceOff` 대체기는 최신 proof/compact를 가린다. r500 경로 assert까지 포함한 네 가지 시험용 개입을 제거하고, 관찰 wrapper는 기존 구현을 한 번 호출해야 한다. 이는 아직 실행하지 않은 준비 절차이며 production 기능 추가가 아니다.

결론은 **추가 기능 없이 현재6 ms/20 ms/compact 조합을 먼저 통합 측정**하는 것이다. 새 current가 실제 준비됐는데도 미발행/불필요 대기하는 사례가 나오면 token·원 stamp·실제 dt·거절 이유로 좁혀 수정한다. Jetson 종료 상태와 재사용 전 접근 금지를 유지한다.

후속 실행 준비: [stage helper](stage/README.md)에 최신 설치 root·명시 compact profile을 받는 단1회 실행기와 scalar 관측 wrapper를 준비했다. [PC fixture/AST 7개](stage_fixture.log)가 통과했고 원 stamp·1회 호출·compact 불변을 확인했다. 실제 ROS/ARM 실행은 하지 않았다. 최초 float literal 비교 fixture 오류는 [별도 보존](fixture_initial_failure.log)했다.
