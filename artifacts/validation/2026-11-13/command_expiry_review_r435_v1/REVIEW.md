# r435 — r431 명령 만료 직전 시간축·출력 상한 검토

2026-11-13, GPT. Windows에서 저장 자료만 분석했다. 추가 승인된 `AppliedGuard` 순수 Python 비교 1회 외에 주행·ROS·WSL·장치·빌드·공통 코드 수정은 하지 않았다. 원 `passed=false`, `harness_valid=false`, `run_valid=false`와 모든 실패 기록을 보존한다. 수치·원본 SHA-256·JSON 배열/JSONL 위치는 [timing.json](timing.json)에 정리했다.

## 결론

**r431의 직접 수정 대상은 시뮬 출력기에 이미 선택된 차량 조향 한계가 전달되지 않는 코드 오류다.** 계획 계산 폭주나 전체 시뮬 정지를 근거로 command deadline을 늘릴 상황이 아니다.

당시 launch는 `max_steering_angle_rad`를 차량 기하에서 구해 virtual MCU와 출력기 모두에 넘겼지만, `GuardedSimActuator`는 이 파라미터를 읽지 않고 `AppliedGuard`를 기본 `max_angle=0.45`로 만들었다. MCU는 실제 명령 `0.4578436017036438 rad`를 수용했다. 출력기는 이 각도를 거절하도록 구현되어 있었고, 저장 상태를 넣은 순수 비교에서도 같은 거절을 재현했다.

로그의 연속은 **19.380초 각도 변경 수용 → 19.382초 `output_ready=false` → seq920/deadline 갱신 중단 → 19.472초 정확한 deadline에서 `command_expired`**다. 최종 출력 상태의 `last_status_us=19377000`도 각도 변경 직전 상태에서 멈춘다. 다만 최초 출력기 거절 메시지와 host TX 시계열은 저장되지 않았다. 따라서 **상한 불일치와 해당 상태의 거절은 확정**, 실제 callback별 전달 시각은 미기록으로 구분한다.

## 코드 근거와 기록의 범위

이하 행 번호는 [r431 동결 소스](../crop_reach_drive_r431_v1/sources.zip) 기준이다. 공통 파일은 후속 수정될 수 있으므로 당시 내용과 SHA는 동결본을 기준으로 한다.

- `src/arena_bringup/launch/simulation.launch.py:1302–1316, 1377–1381`: 선택한 차량의 조향 한계를 출력기와 MCU에 전달한다.
- `src/arena_vehicle_interface/arena_vehicle_interface/actuation_ros.py:328–336`: `GuardedSimActuator`는 `applied_max_speed_mps`만 읽고 `AppliedGuard(..., max_speed=limit)`를 만든다. 조향 파라미터는 사용하지 않는다.
- `src/arena_vehicle_interface/arena_vehicle_interface/applied_guard.py:6, 30–39, 52`: 기본 조향 한계는 0.45 rad이며 초과 상태는 `invalid_applied_status`로 거절한다. 거절 시 `last_stamp`도 갱신하지 않는다. `watch():75–76`은 이후 사유를 `applied_stream_timeout`으로 바꿀 수 있다.
- `actuation_ros.py:214–218, 121–130`: 출력 ACK가 차단되면 `output_ready`가 false가 되며, host는 준비되지 않은 offer에서 DRIVE를 발행하지 않는다. 이 경로는 관측한 seq 갱신 중단을 설명한다. 각 host callback의 실제 실행/반환 시각은 저장되지 않았다.
- `scripts/validate_g1_traffic.py:858–886, 1098–1144, 1254–1257`: MCU status는 전체 배열로 보존하지만 `/drive`, `/drive/safe`, `/actuation/output_status`는 최신값만 기억한다. trajectory는 약 0.1 sim초마다 서로 다른 토픽의 최신값을 묶은 표본이다. output_status는 최종값만 report에 남고 host TX/RX 원시 기록은 없다.

## 만료 직전의 정확한 관측

MCU `now_us`는 virtual MCU 노드의 ROS `/clock`을 μs로 표현한다. parent의 `at_s`/autonomy `sim_time_s`도 시뮬 시간이나 노드별 `/clock` 수신 시점이 같다고 보장하지 않는다. trajectory의 `t`는 평가기가 받은 ego pose 시각이다. worker의 `*_wall_s`는 `time.monotonic()`이며 시뮬 시각으로 환산하지 않았다. 아래 시각 차이를 실제 callback 지연으로 확대하지 않는다.

| 시각·영역 | 저장된 사실 |
|---|---|
| parent `at_s=19.360` | follow seq602가 현재 재검사를 거친 0.35 m/s, 0.4578436017036438 rad를 `published_checked_route`, `retained=true`로 기록한다. command reference는 19.342초다. |
| MCU 19.377 | 배열 4791행: seq919, 각도 0.4162249267101288, `output_ready=true`. 최종 sink의 마지막 수용 시각과 같다. |
| MCU 19.380 | 4792행: seq920, 0.35 m/s, 각도 0.4578436017036438, deadline 19.472초. 당시 ACK를 반영한 `output_ready=true`. |
| MCU 19.382 | 4793행: seq920 유지, 최초 `output_ready=false`. 상태는 여전히 ACTIVE다. |
| evaluator pose 19.391 | 마지막 trajectory: 진행 9.81 m, 지면속도 0.286805 m/s, `/drive`·`/drive/safe` 최신값 모두 0.35 m/s, safety `clear`, MCU ACTIVE. 이 표본에 safe 명령 source stamp는 없다. |
| autonomy 19.442 / 19.460 | 각각 `HANDOFF_FOLLOW`, `DECISION_PENDING`; 상위 speed command 0, steering 0.4578436017036438. 0 명령의 host 수신/MCU 수용 증거는 없다. |
| MCU 19.467 | 마지막 정상 상태: seq920, deadline19.472, output_ready=false, feedback capture19.455/valid=true. |
| MCU 19.472 | 4811행: `FAULT_LATCHED / command_expired`, feedback capture19.465/valid=true. 이전 deadline과 정확히 같다. |

19.380 이후 fault까지 MCU 상태 수신은 이어지며 마지막 2 sim초의 최대 상태 시각 간격은 5 ms다. 피드백도 갱신된다. 이는 전체 시뮬이 멈추거나 엔코더 입력이 끊겨 발생한 만료라는 설명과 맞지 않는다. 개별 노드의 스케줄 지연이 전혀 없었다는 일반적인 증명은 아니다.

최종 sink 상태는 `now_us=20356000`, speed0, blocked=true, `reason=applied_stream_timeout`, `last_status_us=19377000`이다. 이는 최초 거절 사유 자체가 아니라 후속 watch가 남긴 최종 사유다. `actuation_diagnostics.json=[]`도 지연 부재의 증명으로 사용하지 않는다.

## 계획·현재 재검사 계산

| follow seq / reference | plan submit → began → complete → receive (wall s) | 실제 계산 | current 재검사 |
|---|---|---:|---|
| 601 / 19.34 | 539.140873218 → 539.140958075 → 539.171298525 → 539.171724006 | 30.340 ms | seq301, submit539.176050252 → began539.176144079 → complete539.189810554 → receive539.190199164; 13.666 ms, `rebase_current_released`, 0.35 m/s |
| 604 / 19.44 | 540.278703654 → 540.278812400 → 540.305649610 → 540.306103263 | 26.837 ms | 별도 current 결과 없음. plan이 0 m/s, `parallel_hold_reference_not_straight`를 반환 |

직전 reference19.14/19.24의 plan 계산은 40.546/32.558 ms, current는14.238/14.885 ms다. 첫 0 명령을 낸 다음 plan도 완료됐다. 이후 reference19.54의 약101 ms 계산을 앞선19.382 출력 차단 원인으로 소급하지 않는다.

seq603은 current sequence302를 `retained_route_not_proven`으로 버렸다는 기록만 있다. submit/ready 시각과 입력·결과가 없으므로 이를 앞뒤 계산과 동일 호출처럼 연결하지 않는다. 19.442 autonomy 기록의 motion은 `reason=ok`, 유효점474, 마지막 광선19.4218047초, reference19.442초다. 원시 scan도19.222/19.322/19.422초에 존재한다. 입력 스트림 누락은 지지되지 않지만, 경로가 관측 범위 내에서 허가되는지는 별개의 판단이다.

## 저장 상태를 이용한 순수 비교

Windows `python -B`에서 **r431 `sources.zip` 안의 원 `AppliedGuard`를 직접 import**했다. 원 actuation 배열1–4792행을 바꾸지 않고 순서대로 넣어 기존 explicit ARM과 상태 이력을 유지했다. 두 비교는 `max_angle`만 다르고 `timeout_us=100000`, `wall_timeout_s=30`, `max_speed=2.5`는 같다. 비교 상태의 속도0.35는 속도상한에 걸리지 않는다.

원 sink callback wall 시각이 없으므로 `now_us=status.now_us`, `wall_s=status.now_us/1e6`을 단조 시험 시각으로 사용했다. 이는 스키마·각도 검사 재현이며 ROS 스케줄이나 실제 지연 재현이 아니다.

| `max_angle` | 19.377 상태 | 19.380 상태 |
|---|---|---|
| 기존0.45 | 수용, speed0.349999994, last_stamp19377000 | **거절**, `invalid_applied_status`, blocked=true, speed0, last_stamp19377000 유지 |
| 기록된 명령각0.4578436017036438 명시 | 수용 | **수용**, speed0.349999994, last_stamp19380000 |

비교 exit0. 두 번째 값은 원 명령을 수용하는지 분리한 비교 입력이며 새 운용 상한을 정한 것이 아니다. 후속 수정은 이미 선택한 차량 기하의 파라미터를 단일 원천으로 읽어 전달해야 한다.

## r428·r418 마지막 창과의 차이

각 첫 latch 직전2 sim초만 같은 방식으로 집계했다. 서로 다른 경로·상태의 시험이므로 전체 정책 성능의 직접 대조는 아니다.

| 시험 | 마지막 2초 trajectory | STOP 직전 MCU | 원 종료 사유 |
|---|---|---|---|
| r431 | 19표본 모두 drive/safe 양수, 진행8.13→9.81 m | 마지막수용19.380, deadline19.472에서 fault | 자체 `command_expired` |
| r428 | 19표본, drive양수8·safe양수10, 진행10.02→10.13 m | seq1074를22.460에 수용, zero_target, deadline22.543. 첫 latch22.466 | **사용자 중단** helper의 software_stop. 8초 정체로 분류하지 않음 |
| r418 | 20표본 모두 drive/safe0, 진행10.10 m 고정, safety clear20 | seq1928을39.560에 수용, zero_target, deadline39.654. 첫 latch39.570 | 문서에 기록된 **8 sim초 무진행** 후 helper software_stop |

r428 마지막 plan은 `follow_partial_transformed_gap_uncovered`/`follow_release_wait`의0 명령이다. r418 마지막 세 plan은 약0.5 m/s를 제안하지만 current가 `route_current_large_motion_region_unobserved`로0을 반환하고 parent도0을 기록한다. 두 시험 모두 마지막까지 zero 명령의 seq/deadline을 갱신했다. r431처럼 수용된 양수 명령의 갱신이 끊긴 사건과 다르다. 각 helper의 최초 STOP 요청 정확시각은 없으므로 후행 evaluator STOP과 서비스 호출 지연을 계산하지 않는다.

## 후속 한 가지와 남는 한계

**`GuardedSimActuator`가 launch에서 받은 차량 `max_steering_angle_rad`를 읽어 `AppliedGuard(max_angle=...)`에 전달하도록 연결한다.** 보호각이나 deadline을 임의로 늘리는 변경이 아니라 이미 선택한 차체 한계를 누락한 전달 오류 수정이다. 공통 코드 수정·관련 회귀·최종 검토는 주 담당이 수행한다.

r431은 current crop retry/수락/부모 발행이 모두0이며 전체 랩이나 r418 정체 해소를 입증하지 않는다. 순수 비교도 출력기 각도 검사만 다룬다. 실제 최초 `invalid_applied_status` ACK의 시각, 각 `/drive/safe` 발행·host TX 수신 시각은 원자료에 없으므로 새 시각을 만들어 채우지 않았다. 별도 전체 시뮬 재실행과 실물 구동 검증을 수행한 것으로 해석하지 않는다.

검토에서 사용한 원파일33개는 SHA-256 재대조에서 변경0이다. 원 실패·STOP·정상 회수 결과는 [r431 실행 영수증](../crop_reach_drive_r431_v1/execution_v1/receipt.json), 사용자 중단은 [r428 결과](../crop_reach_drive_r428_v1/RESULT.md), 기존 정체는 [r418 결과](../crop_edge_drive_r418_v1/RESULT.md)에 보존되어 있다.
