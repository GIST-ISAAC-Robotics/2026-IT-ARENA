# `AppliedGuard` 같은 `now_us` 상태 거부 — 진단·계약 검토(정책 변경 없음)

대상: `src/arena_vehicle_interface/arena_vehicle_interface/applied_guard.py` `AppliedGuard.update`, `actuation_ros.py` `VirtualMCU`/`GuardedSimActuator`. 이번 작업에서 코드를 바꾸지 않았다.

## 현재 동작

- 가상 MCU는 같은 sim 시각에 상태를 두 번 낼 수 있다. ① 5 ms `control_tick` 상태(그 시점 목표), ② `/actuation/tx` 수신 직후 상태(새 목표). 둘 다 `now_us`가 같을 수 있다.
- `GuardedSimActuator`는 수신 상태를 `now_us` 순서로 꺼내 `AppliedGuard.update`에 넣는다. `update`는 같은 epoch에서 `stamp <= last_stamp`이면 거부한다.
- 그래서 ①이 먼저 처리되면 ②(새 목표)는 버려지고, 새 목표는 다음 5 ms tick 상태에서 출력에 반영된다. 10/8 R1은 속도 전환 16/16에서 이 순서였고 출력이 MCU 적용보다 5~6 ms 늦었다.

## 의도와 위험

- 단조 stamp는 오래된·재전송·순서가 뒤바뀐 상태로 출력이 되돌아가는 것을 막는 보호 규칙이다. 이 성질은 유지해야 한다.
- 같은 stamp의 서로 다른 두 상태는 현재 계약에서 순서를 정할 근거가 없다. 결과적으로 출력 반영은 결정적으로 늦어지거나(①→②), 처리 순서가 실행마다 바뀌면 0/5 ms 사이에서 흔들릴 수 있다. 10/8 R1/R5의 출력 시각 차이는 1 ms 이하였으므로 이 경로가 그 반복 차이의 직접 원인이라는 근거는 없다.
- 실물 MCU에서 같은 마이크로초 두 상태가 나올지는 펌웨어 설계에 달려 있다. 시뮬레이션 지연 5 ms를 실차 지연으로 옮기지 않는다.

## 후속 결정 후보(채택하지 않음)

1. MCU 상태에 부팅별 엄격 증가 순번을 넣고 `(now_us, status_seq)` 사전식 순서로 단조성을 판정한다.
2. 같은 `now_us`에서 `accepted_seq`가 증가한 상태만 허용한다(재전송 방지 성질 유지 여부 검토 필요).
3. MCU가 같은 `now_us`의 tick 상태와 수신 상태를 하나로 합쳐 낸다.

어느 안이든 보호 의미와 실물 MCU 계약을 바꾸므로 ADR·계약 시험·실물 펌웨어 설계와 함께 결정해야 한다. 이번 계측은 출력 단계 이후(플러그인 수신·적용)를 관측하는 것이며 이 정책을 바꾸지 않는다.
