# r516 저속 지속·정체 복구의 최소 실행 범위

2026-11-14, Windows 읽기 전용 설계. r515 실행 중인 공통 소스·설정·장치·WSL·Jetson을 변경하거나 실행하지 않았다. r511 결과를 재사용하고 관련 함수만 확인했다. 아래 수치/흐름 중 제안은 제안으로 표시하며 새 기능이 구현된 것으로 보지 않는다.

## 결론

**우선안은 현재 관측 경로를 유지하는 전진 복구다.** 기존 정상 양수 명령은 계속 사용하고, 경로 선택 때문에0이 된 경우에만 현재 외벽/현재 차선의 짧은 경로를 기존 점유·조향·제동 계산에 한 번 더 넣는다. 불확실성만으로 즉시0을 만들거나 별도 정체 상태기계·새 확인 횟수를 더하지 않는다. 실제 점유가 같은 명령을 막으면 속도값만 바꿔 억지로 통과시키지 않고 다른 현재 경로/조향을 비교한다.

**후진은 현재 공통 제어 체인에서 지원되지 않는다.** signed 엔코더가 있는 것과 후진 명령·후방 경로가 구현된 것은 다르다. 근거 없이 뒤로 움직여 센서를 갱신하는 방식은 제외한다. 후진을 다음 작은 수정이라고 분류하면 실제 범위를 숨기는 셈이다.

## 기존 기능에서 재사용할 것

| 현재 코드 | 재사용 범위 | 지금 남는 장애점 |
|---|---|---|
| `race_runtime_policy/current_route_priority.py:observed_command/select/rebase` | 현재 경로+원시 점유 검사 이후 중복 측방 가림/전역 가시성 조건을 붙이지 않음. 현재 입력 재검사 유지 | 원래 경로가 양수일 때 작동하므로 `follow_boundary_unconfirmed` 자체는 고치지 못함 |
| `moving_pass.py:MovingPassGuide._current_follow_return`, `_wall_covers_guide` | 같은 현재 외벽으로 추월 경로와 중심 복귀 경로를 비교. 현재 경로 점유 접두 길이가 더 긴 쪽을 선택하며 복귀에는 추종 간격 적용 | r515에서 현재 외벽의 뒤쪽 범위 문제 해소 결과를 먼저 판단. 경로가 아직 비어 있으면 속도만 낮춰도 생성되지 않음 |
| `bypass_entry_trial.py:_compare_entries`, `entry_path` | 현재 경로/반환 그대로 조향각·조향 변화율·곡률·원시 점유·제동·추종 간격을 계산. 명령은 기존 경로로 전달 | `known_occupied_command_hold`의 clearance0이면 같은 조향의 작은 양수도 해결책이 아님 |
| `moving_pass.py:MovingPassAdapter._prepare_follow_handoff` | 복귀 중 최신 센서로 LocalFollow를 준비하고 centered 시 그 상태를 인계 | 기본 오프셋 경로에서 멀리 떨어진 상태에 즉시 인계하면 r511 앵커 거절 재발 |
| `local_follow.py:standing_wall_reacquire`, `low_speed_arc_reacquire`, `follow_steer_in_place` | 현재 벽 재획득, 정지 상태에서 경로 방향으로 조향만 바꾼 뒤 현재 입력으로 다시 평가 | 현재 경계의 길이·앵커가 성립해야 함. r503 앵커 약0.2135m, r513 약0.1243m는 상한0.12m를 넘음 |
| `released_straight_recovery.py:compare_released_straight` | 기존 경로에서 짧은 직진 또는 제자리 조향 후보를 비교하는 기하 패턴 | RELEASED·현재 baseline 양수 등 특정 범위. MOVING_PASS/경계 소실에 이름만 바꿔 적용하면 안 됨 |
| `bounded_recovery.py:PulseBudget/recovery_evidence` | 명시 단발을 시간/모터축 적분으로 끝내는 도구. 기본0.25s/0.05m, 자동 반복 재무장 없음 | 운영 미연결. 거의0조향, quiet feedback, 지지된 현재 직진 경로 필요. r511 횡편이/조향 정체와 바로 맞지 않으며 정상 주행에 붙이면0.25s마다 멈추는 역효과 |

`guard_crawl.py`의0.10m/s 재검사도 존재하나 특정 GUARD/가림 실패와 기존 경로가 있어야 한다. 현 `current_route_priority`가 이미 우회한 옛 전역 관측 관문을 다시 연결하는 용도로 쓰지 않는다.

## 제안하는 최소 구현

1. **r515 결과가 먼저다.** 현재 외벽 기반 이동/복귀가 이어지면 정체 기능을 별도로 추가하지 않는다. 잔여 정지가 실제 점유인지, 경로 생성/인계 조건인지 결과의 reason으로 분류한다.
2. 경로가 유효하고 양수인 정상 명령은 그대로 반환한다. 기존0명령 중 현재 경로는 있으나 선택·추종 방식이 막힌 경우에만 **동일 현재 경로의 저속 후보 최대1개**를 비교한다. 출발 시 실제 속도를0이나 목표속도로 바꿔 넣지 않는다. 정상 흐름의 매 scan에 새 경로 후보군을 늘리지 않는다.
3. 현재 차선/외벽 경로를 유지하면서 중심 복귀를 비교한다. r511처럼 원점과 기본0.505m 오프셋 경로의 거리가 큰 동안에는 LocalFollow로 강제 인계하지 않는다. 정상 경로로 돌아왔을 때 기존 인계 함수를 사용한다. 전체 앵커 상한0.12m→0.25m 변경과 TTL 연장은 이번 최소안에서 제외한다. r511 진단에서 앵커만 넓혀도 뒤의 점유/추종 정지가 남았다.

다음은 **제안용 의사코드**이며 기존 함수명을 활용한 연결 위치 설명이다. `one_current_guide_retry`는 새 얇은 연결 함수의 가칭이다.

```python
normal = existing_command(current_scene, actual_feedback, previous_steering)
if normal.speed > 0:
    return normal                 # 불확실성만으로 새 감속/정지 관문을 추가하지 않음
if existing_stop_or_input_failure:
    return normal                 # 기존 STOP/입력 실패 처리를 그대로 사용

# 이미 이번 입력으로 만든 현재 차선/외벽 경로만 사용. 기억 연장/GT/새 점 생성 없음.
guide = existing_current_guide
if guide is not None:
    candidate = one_current_guide_retry(
        compare=_compare_entries, guide=guide,
        measured_speed=actual_feedback, previous_steering=previous_steering,
        target_cap=min(operator_limit, 0.10), follow_headway=True)
    if candidate.selected and candidate.selected.speed_mps > 0:
        return existing_output(candidate)   # /drive -> 독립 guard -> /drive/safe

return existing_zero_speed_presteer_or_hold
```

0.10m/s는 새 실물 채택값이 아니라 저장 입력/시뮬 저속 후보 상한의 제안이다. 조립 직후 지상 시험의 현행 목표0.05m/s·상한0.1m/s 지시와 별개로 자동 설치하지 않는다. `_compare_entries`가 이미 같은 경로의 제동/간격으로0을 냈다면 속도 후보를 반복 순회하지 말고, 기존 중심 복귀 또는 직진/제자리 조향 비교로 넘어간다. 유효한 전방 경로도 없는 경우에는 시간만 기다렸다 자동 출발시키지 않는다.

최소 수정 파일은 상황에 따라 `moving_pass.py`의 후보 선택/인계 또는 해당 경로 helper1곳과 집중 회귀다. planner와 guard의 관측 판단을 일괄 재작성하거나 새 노드·서비스·복구 상태기계를 만들 이유는 현재 근거에 없다. 구현 여부는 r515 결과로 결정한다.

## 제한 전진과 후진의 계약 경계

| 계층 | 현재 확인한 사실 | 후진에 필요한 별도 작업 |
|---|---|---|
| 수동 단발 | `development_jog.py:plan`은 forward/left/right 및 양수속도만 허용.0목표 후 STOP 경로 보유 | reverse 인터페이스/시간·거리 예산/취소 계약 추가 |
| 계획·독립 보호 | `lidar_safety.py:184`가 음수 명령을 `invalid_or_reverse_command`로 거절. `local_path` pursuit는 전방 x>0.10 목표를 선택 | 후방 목표·후진 곡률/조향 변화·후진 차체 sweep와 현재 후방 관측 검사 필요 |
| DRIVE 계약 | Python `actuation_contract.py:323`, portable `actuation_command.hpp:652`는0≤speed≤max | 호스트/수신기 양쪽의 부호 허용·상한·프로토콜/회귀 일치 |
| 속도 제어 | `mcu_speed_loop.py:33`, `portable/speed_pi.hpp:43`는 음수 target을 거절 | signed target PI, 방향 전환 때0출력·적분 초기화·실제 정지/재가속 계약 |
| 실제 출력 | `md20a_forward_output.hpp`는 전진 전용. runtime `main.cpp:27`의 DIR은 실장 프로필 고정 전진 극성 | 검증된 양방향 DIR/PWM 매핑, 방향 전환/실측 확인. MD20A 하드웨어 자체가 후진 불가하다는 뜻은 아님 |
| 엔코더 | 단일 모터축 signed count/속도와 signed 운동 이력 보존. `forward_planning_speed`는 음수 피드백도 전진 계산에 abs 상계로 사용 | 부호를 관측할 수 있다는 것만으로 지면 후진 거리·차체 정지·미끄럼을 보증하지 못함 |
| 조향 | 현재 중앙 등가 바퀴각↔서보 위치 표는 양/음 각도를 지원 | 후진은 같은 조향각에서 yaw 변화 방향이 달라짐. 기존 전진 pursuit에 음수 속도만 넣는 것으로 해결되지 않음 |

**후방 관측을 과거 가정으로 판단하지 않는다.** r513 저장 입력의 `rear_half_angle_deg`는0이며 CAD 거리 한정 구조 가림4개가 있다. 과거 기본 후방60° 가림을 현행 입력에 그대로 적용해서 후방이 전부 안 보인다고 단정하지 않는다. 다만 현재 소프트웨어가 후진할 차체 영역에 충분한 반환이 있는지 검사하는 경로는 없다. 실제로 관측하지 못한 후진 영역을 빈 공간으로 채워 복구시키는 안은 제외한다.

따라서 이번 최소 복구 변경에 후진까지 포함하는 것은 범위가 맞지 않는다. 후진은 위의 방향성 전 계층과 후방 관측을 함께 구현·검증하는 별도 기능이다. 현재 STOP/guard를 우회해서 시험하지 않는다.

## 대표 시험 3개

1. **r503/r513 저장 정체 입력 + r515 폐루프 결과:** 같은 현재 경로·점유에서 정상 양수 명령은 불변, 경계 기준/인계 원인의0만 후보1개로 비교한다. r511 앵커값을 바꿔 성공으로 꾸미지 않는다. 후보가 나온 경우에만 후속 대표1회로 실제 진행·추종 복귀·접촉·최종 STOP을 판단한다. 저장 재생 양수를 완주로 쓰지 않는다.
2. **동일 현재 경로의 부분 가림/실제 점유 대조:** 측방 일부 가림이나 이전 상태만 달라지고 현재 경로·전진 sweep가 같으면 양수 지속, 같은 경로에 실제 접촉 반환을 넣으면 기존 점유/제동 결과를 유지한다. 현재 전방 경로가 사라진 표본은 맹목 전진·후진으로 바꾸지 않는다. 새 전역 가시성 관문을 추가하지 않았음을 함께 확인한다.
3. **저속 정지→조향→전진 및 방향/STOP 경계:** 기존0목표 조향으로 회전 방향을 맞춘 뒤 새 현재 입력에서 전진 가능해지면 RESET 없는 기존 복귀를 사용한다. 명시 STOP·오래된 입력·소유 세대 변경에서 전진 후보가 발행되지 않고, 음수 target은 현재 guard/DRIVE/PI에서 계속 거절되는지 확인한다. 별도 단발을 선택한다면 기존 PulseBudget의1회 예산·취소·재무장 금지 회귀를 재사용한다.

이번에는 위 시험을 실행하지 않았다. 기존 관련 검사를 다시 돌려 설계 완료 수치로 포장하지 않으며, 구현 선택 후 변경 영향에 맞는 항목만 수행한다.

## 근거

- `current_boundary_traffic_r511_v1/RESULT.md`, `geometry.json`, `anchor_probe.json`: 앵커·기억 만료와 단순 완화 한계.
- 현재 공통 소스의 함수/계약 읽기. 정확한 파일 해시는 `source_manifest.json`에 남긴다. 이는 읽은 시점의 스냅샷이며 병행 작업의 결과를 이 문서로 덮어쓰지 않는다.
