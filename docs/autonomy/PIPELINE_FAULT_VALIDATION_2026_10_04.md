# RGB·프로세스 중단과 정지·재출발 경계 검증

날짜: 2026-10-04. 상태: **제한된 통합 검증 마무리, 간헐 MCU 갱신 지연은 미해결**. [개발 계획 A4](../reports/DEVELOPMENT_PLAN_2026_09_22.md)의 제한된 통합 시험이며, [C1/IMU 입력 보호](SENSOR_INPUT_GUARDS_2026_10_04.md) 다음 단계다. GPT가 범위·안전 기준·Gazebo 검증기를, GPT 6.1 Sol이 합성 검증기 보완을, GPT 6 Luna가 실행·대기를 담당했다. 운영 제어기·보호 한도·물성·센서 배치는 변경하지 않았다. 이번 변경은 로컬이며 GitHub에 게시하지 않았다.

## 1. 목적과 판정 경계

RGB 입력 또는 영상 처리기·지역 제어기·LiDAR 보호층이 멎었을 때, 어느 단계에서 0명령을 만들고 어떤 조건에서 다시 움직이는지 확인한다. 센서 복구에 따른 자동 재개, 영상 처리기 고장 고정, MCU 고장 고정을 같은 동작으로 취급하지 않는다.

```text
합성 RGB·순차 LiDAR·IMU·모터축 카운트
                ↓
local_pursuit → /drive → lidar_safety → /drive/safe
                                           ↓
                              actuation_host → virtual_mcu
                                                    ↓
                                       guarded_sim_actuator
                                                    ↓
                                       단순 종방향 시험 모형
```

검사기는 센서와 `/clock`만 발행한다. `/drive`·`/drive/safe`·최종 구동 출력의 발행자가 각각 실제 운영 노드 하나인지 검사하며, 제어기에 지도·정답 위치·신호 상태 토픽을 제공하지 않는다. 출발 판정은 RGB 픽셀의 빨강→초록 변화로 확인한다. 시험기의 서비스 `ARM`은 경기 출발 신호가 아니다.

- 합성 시험은 직진 속도 상한 0.6 m/s, 임의 ±2 m/s² 종방향 모형이다. 조향·접지·충돌은 적분하지 않는다. 센서 fixture는 차량 기준 유한 벽이고 공식 코스가 아니다.
- 모터축 28카운트/회전·15:1·바퀴 반지름 0.0325 m로 양자화한 카운트만 제어 입력으로 반환한다. MCU 실물 펌웨어나 실제 카운트 기반 모터 PI가 아니다.
- 시뮬레이션 시간 5 ms씩 진행, IMU 200 Hz·엔코더 100 Hz·순차 LiDAR 10 Hz·RGB 10 Hz다. 벽시계 sleep과 DDS/계산 시간 때문에 실시간 1배속 시험은 아니다. RGB 장치 최대 수신률 시험도 아니다.
- 첫 0명령의 **검사기 수신 시각**, MCU 고장 상태, 모형 속도 0, 프로세스 종료는 별도 증거다. 종료된 노드는 자체 0명령을 발행할 수 없으므로 해당 값이 없으면 null로 보존한다.
- 관측 표본 사이의 모든 순간, 실차 제동거리, 최대 고장 검출 지연, Jetson 처리 성능을 보증하지 않는다.

## 2. 기존 정책과 확인할 복구 조건

| 조건 | 운영 코드의 정책 | 확인할 경계 |
|---|---|---|
| 부팅·녹색 영상만 입력 | 초기 출발 허가 없음 | MCU ARM만으로 주행하지 않음 |
| 주행 중 RGB 중단·같은 취득 시각 반복 | 마지막 유효 영상이 만료되면 `SENSOR_STOP` | `/drive`·안전 명령·최종 출력 0과 모형 정지 |
| 새로운 시각의 정상 영상 복구 | 기존 출발 상태를 유지하고 자동 재개 | 자동 재개임을 명시; 명시적 재허가라고 부르지 않음 |
| 같은 녹색 영상에 새 취득 시각 | 정상 정지 장면과 내용 고착을 구별하지 못함 | 정상으로 통과하는 관측 한계로 기록 |
| 영상 처리기 정지/강제 종료 | watchdog/프로세스 오류를 고정 | 영상 재수신·`/autonomy/reset`만으로 재출발하지 않음 |
| 지역 제어기 정지 | LiDAR 보호층이 명령 노후화로 0 발행 | 제어기 재개 시 기존 출발 상태가 남으면 자동 재개 가능 |
| 지역 제어기 재시작 | 출발 상태 초기화 | 녹색만으로 불출발, 새로운 빨강→초록 필요 |
| LiDAR 보호층 정지/종료 | MCU가 유효 명령 만료를 고정 | 보호층 복구만으로 불출발; CLEAR만으로 DISARMED; 별도 ARM 뒤 주행 |
| 명시 STOP | `ESTOP_LATCHED` | 실제 모형 정지와 정지 유지 뒤 프로세스 정리 |

이 정책은 [영상 분리 ADR 0025](../decisions/0025-isolated-vision-processing.md)·[구동 계약](../hardware/ACTUATION_CONTRACT_2026_09_22.md)에 근거한다. 모든 상위 센서 정지를 MCU 고장 고정으로 바꾼 것은 아니다. 경기 중 사람 개입·재출발 장소와 최종 허가 절차는 B3에서 통일해야 한다.

RGB 신선도 한도는 기존 **시뮬레이션 시간 1초**, 영상 처리 요청 watchdog은 **벽시계 1초**다. 두 값을 같은 지연으로 해석하면 안 된다. RGB가 마지막 한도까지 없어도 LiDAR 조향/보호가 계속 동작할 수 있으며, 그동안의 주행 거리도 0이 아니다. 예를 들어 2.5 m/s를 그대로 유지한다면 1초에 2.5 m를 이동하지만, 이는 계산 예시이지 본 시험에서 관측한 정지 거리나 허용 안전거리의 근거가 아니다. 실제 카메라·마커 의존 기능과 제동 응답에 맞춘 한도 검토는 별도로 필요하다.

## 3. 실행·실패 이력

### 최초 합성 실행 `ros_v1`

[보고서](../../artifacts/validation/2026-10-04/pipeline_fault_validation/ros_v1/report.json), [당시 소스](../../artifacts/validation/2026-10-04/pipeline_fault_validation/ros_v1/source_snapshot.zip), [실행 명령](../../artifacts/validation/2026-10-04/pipeline_fault_validation/run_ros_v1.sh)을 보존했다. 순수 단위 검사 7개와 개별 동작 32개는 당시 판정을 통과했지만, 최종 `/drive/safe` 발행자 목록이 `['lidar_safety', 'lidar_safety']`여서 **전체 실패**다. 종료 코드·실행 소스 해시는 일치했고 검사기가 알고 있는 영상 프로세스 잔존은 없었다. 당시 최종 그래프에 남은 두 endpoint를 곧바로 두 프로세스가 동시에 실행 중이었다는 증거로 해석하지 않는다.

독립 검토에서는 no-resume 판정이 복구 직후의 짧은 양수 출력을 놓칠 수 있고, 마지막 명시 STOP 없이 정리하며, 영상 내부 강제 종료·알려지지 않은 같은 세션 자식을 충분히 확인하지 않는 한계도 발견했다. 이를 보완한 시험 결과와 최초 판정을 구분한다. 차량 코드의 정상 동작을 증명하기 위해 시험 실패를 지우거나 중복 발행자를 무조건 허용하지 않는다.

### 보완 판정과 `ros_v2`의 별도 정지

복구 동작 **직전**부터 최종 출력·출력 감시·MCU의 모든 수신 표본이 0인지 검사하도록 보완했다. 세 토픽 각각의 신선한 표본과 모형 정지를 마지막에도 확인한다. 표본 수신 공백의 임시 판정 한도는 시뮬레이션 시간 0.25초·벽시계 3초이며, 이는 검증기 관측 연속성 조건이지 운영 watchdog 한도 변경이 아니다. source 시각이 고장 주입 전인 0명령은 첫 고장 대응 출력에서 제외하고, source 시각 없는 출력은 수신 시각만 따로 기록한다. 마지막 명시 STOP·정지 유지와 세션 자식 잔존·영상 내부 강제 정리도 검사한다.

[보완 실행 `ros_v2`](../../artifacts/validation/2026-10-04/pipeline_fault_validation/ros_v2/report.json)는 **전체 실패**다. 순수 판정 검사 30개는 통과했으나, 첫 영상 처리기 복구 후 제어기 재시작 구간에서 가상 MCU가 `control_deadline_missed`로 정지를 고정했다. 첫 MCU 고장 상태는 시뮬레이션 시각 **13.900 s**이고, 최종 재출발 대기가 끝난 **19.060 s**를 고장 발생 시각으로 쓰면 안 된다. 직전에 출력 감시에서도 `applied_command_expired`가 관측됐다. 그 뒤 상위 안전 출력은 양수로 회복했지만 MCU/최종 출력은 0을 유지했다.

이는 보호 고정이 동작한 근거지만, 정상 복구까지 검증한 성공 결과가 아니다. 제어 갱신 공백이 실제 CPU 스케줄·DDS 시계 수신·다른 실행 간섭 중 어디에서 생겼는지는 이 기록만으로 확정하지 못한다. 시험 한도를 늘리거나 자동 CLEAR로 실패를 덮지 않았다. 실행 소스 해시는 일치하며 모든 소유 자식 정리와 잔존 검사는 통과했다. [상태 전환](../../artifacts/validation/2026-10-04/pipeline_fault_validation/ros_v2/events.json)과 [실행 명령](../../artifacts/validation/2026-10-04/pipeline_fault_validation/run_ros_v2.sh)을 보존한다.

### 동일 조건 재실행 `ros_v3`

[보고서](../../artifacts/validation/2026-10-04/pipeline_fault_validation/ros_v3/report.json)·[상태 전환](../../artifacts/validation/2026-10-04/pipeline_fault_validation/ros_v3/events.json)·[소스 사본](../../artifacts/validation/2026-10-04/pipeline_fault_validation/ros_v3/source_snapshot.zip)·[명령](../../artifacts/validation/2026-10-04/pipeline_fault_validation/run_ros_v3.sh): **34/34 사례·실행 소스 해시·종료 감사 통과**. `ros_v2` 이후 운영/시험 소스·입력 조건·보호 한도는 바꾸지 않고 추가 1회만 수행했다. `ros_v2`의 실패를 소급 통과시키거나 간헐 지연이 해결됐다는 결론은 아니다.

§2의 정지·복구 정책을 실행으로 확인했다. strict no-resume 16사례에서 복구/서비스 호출 직전부터 마지막까지 수신한 세 출력 토픽의 비영 속도가 없었다. 영상 처리기 고장은 SIGCONT·새 영상·reset으로 풀리지 않았고, 제어기 재시작 후에는 새 빨강→초록이 필요했다. 반면 일시 RGB 누락과 제어기 SIGSTOP→SIGCONT는 기존 출발 허가를 유지해 자동 재개했다. LiDAR 보호층 복구만으로는 MCU 정지가 풀리지 않았으며 CLEAR만으로 DISARMED, 별도 ARM 뒤에만 다시 움직였다.

아래는 해당 실행에서 **고장 주입→검사기 최종 0출력 수신**의 시뮬레이션 시간과 임시 모형의 총 변위다. `.42~.60 m/s` 수준에서 주입했으며, 제동 성능·실시간 마감 보증이 아니다. `/sim/cmd_vel`에는 취득 시각이 없으므로 이 첫 0은 수신 시각 기준이다. RGB 누락은 마지막 유효 영상의 나이가 이미 일부 지난 시점에 시작했다.

| 주입 | 최종 0 수신까지(sim s) | 안정 정지까지 모형 변위(m) |
|---|---:|---:|
| RGB 발행 중단 | 0.77 | 약 0.52 |
| 같은 RGB 취득 시각 반복 | 0.75 | 약 0.50 |
| 영상 처리기 SIGSTOP | 0.30 | 약 0.23 |
| 영상 처리기 SIGKILL | 0.04 | 약 0.07 |
| 지역 제어기 SIGSTOP | 0.22 | 약 0.15 |
| 지역 제어기 소유 그룹 SIGKILL | 0.21 | 약 0.14 |
| 보호층 SIGSTOP | 0.09 | 약 0.09 |
| 보호층 SIGKILL | 0.09 | 약 0.14 |

영상 처리기 SIGSTOP의 최종 0 수신은 **벽시계로는 약 1.20초**다. 시뮬레이션 시간이 느리게 진행되므로 0.30 sim s와 다르다. 보호층을 종료한 경우에는 `/drive/safe` 자체의 새 0이 아니라 **하류 MCU 만료/출력 차단**으로 멈춘다. 지역 제어기 SIGKILL 사례는 부모와 그 소유 영상/보조 프로세스 그룹을 함께 종료한 조건이며, 부모만 죽고 고아 영상 프로세스가 남는 조건까지 포함하지 않는다.

마지막 명시 STOP 후 그래프 수렴을 기다리는 동안에도 0출력을 유지했다. 최종 검사 시작 시 옛 보호층 PID 765는 실제 종료 코드 −9였고, 새 PID 1017만 살아 있었다. 두 endpoint 조회가 **6.130초 후 하나**로 바뀌었다. 이는 종료 endpoint 정보 제거가 늦어지는 상황과 일치하며, 둘을 동시에 살아 있는 제어기로 취급할 근거는 아니다. 다만 `ros_v1`의 내부 DDS 원인을 직접 추적한 결과로 확대하지 않는다. 종료 감사는 의도적으로 주입한 −9를 정상 종료 코드 0과 구별했고, 비의도 강제 정리·살아 있는 소유 세션 자식은 없었다.

최종 보고서의 21개 소스/검증/참고 설정 해시와 현재 파일을 대조했다. 참고 `vehicle.yaml`·공식 트랙 설정은 출처 보존용이며 합성 fixture가 불러 쓰는 설정은 아니다. 실제 자식 모듈의 import 경로·해시도 기록했다.

### 관련 회귀

[최종 관련 회귀](../../artifacts/validation/2026-10-04/pipeline_fault_validation/regression_final.log)는 **267개 통과**다. [명령](../../artifacts/validation/2026-10-04/pipeline_fault_validation/run_regression_final.sh)에 명시한 검증기·종료·영상·구동 계약/ROS·엔코더·LiDAR 보호/운동 입력 파일을 검사했다. 저장소 전체 724개 집합을 다시 돌린 결과나 ROS 통합 실행 성공으로 해석하지 않으며, 순수 검증기 30개·종료 20개와 중복 합산하지 않는다. 운영 센서율·제어기·보호 한도·물성은 이번 변경에서 그대로다.

### `control_deadline_missed` 진단 계측과 표적 재현 (후속)

`ros_v2` 실패의 근거를 다시 보면, 출력 감시가 마지막으로 받아들인 MCU 상태는 sim 13.805 s였고 13.885 s에 `applied_command_expired`로 막혔다. MCU는 13.900 s에 다시 상태를 냈고 그때 `control_deadline_missed`를 고정했다. 즉 MCU 상태 발행이 약 95 ms(sim) 동안 관측되지 않았으며, 이는 50 ms 제어 공백 한도(`max_control_gap_us`)를 넘는다. 500 ms 무진행 감시나 100 ms 명령 TTL과는 다른 값이다. 다만 기존 기록에는 MCU 쪽 벽시계 문맥이 없어서 **타이머 콜백 자체가 늦게 진입했는지**, **`/clock`이 한 번에 크게 진행했는지**, 혹은 다른 콜백 경로가 먼저 공백을 발견했는지 구분할 수 없었다. 검사기의 수신 공백 최대 0.100 sim s/0.407 wall s는 검사기 수신 간격일 뿐 MCU 제어 공백의 직접 측정값이 아니다.

이번 후속에서는 원인을 고치는 대신 **다음 발생을 판별할 최소 계측**만 넣었다([변경 전 기준본·해시](../../artifacts/validation/2026-10-04/pipeline_fault_validation/deadline_followup/baseline_manifest.json)).

- `ActuationReceiver`: 첫 `control_deadline_missed` 순간, `last_control_us`를 갱신하거나 상태를 전환하기 **전에** 검사 경로(`tick`/`offer`/`receive`)·현재 시각·마지막 실제 제어 tick 시각·공백·한도·직전 상태/세대/만료 시각을 한 번만 고정한다. 이후 tick·CLEAR·ARM·두 번째 공백은 횟수만 늘리고 첫 기록을 덮지 않는다. 판정·`status()` 형식·STOP/CLEAR/ARM 조건은 바꾸지 않았다.
- `VirtualMCU`: 1 ms 타이머 진입마다 직전 진입의 벽시계/sim 시각 두 값만 갱신하고, 실제 제어 tick은 최근 16개만 고정 크기로 보관한다. 첫 고장 때만 `/actuation/diagnostics`(transient local, depth 1)로 JSON 한 건을 발행한다. 고주기 콜백의 파일 기록·대형 JSON은 없다. 추가 비용은 tick당 tuple 저장과 `time.monotonic()` 한 번 수준이다(별도 측정은 하지 않음).
- 판별 방법: 공백 직전 타이머 진입 간격(`since_previous_timer_entry_wall_s`)이 크면 MCU 프로세스/executor가 그동안 타이머를 돌리지 못한 것이고, 진입 간격은 짧은데 sim 시각이 크게 뛰었다면 `/clock` 진행/수신이 몰린 것이다. 단, OS 스케줄링·DDS 수신 지연·같은 executor의 다른 콜백 중 무엇이 타이머를 늦췄는지는 이 계측만으로 확정하지 못한다.
- 검사기: `--scenario restart_repro --cycles N`은 `ros_v2` 실패 구간과 같은 순서(정상 출발 기준선→RGB 누락/복구→영상 SIGSTOP 고정→reset→제어기 재시작→새 빨강→초록→명시 STOP·그래프 수렴)만 실행한다. 기본 `full`의 34조건 순서는 그대로다. 실패 시 최신 상태·그래프·소유 프로세스 종료 코드를 `failure_state`에 남긴다.

계측 구현·판정은 Claude Opus 5.5, ROS 실행·대기는 Sonnet 5.5(서브에이전트 자기 보고 `claude-sonnet-5-5`)가 맡았다. 보호 한도·`--sleep-wall .01`·`--wall-cap 600`은 `ros_v2`/`ros_v3`와 같다.

| 실행 | 결과 | 제어기 재시작 구간 검사기 수신 공백 최대(sim / wall) | MCU 진단 발행 |
|---|---|---:|---:|
| [`repro_r1`](../../artifacts/validation/2026-10-04/pipeline_fault_validation/deadline_followup/repro_r1/report.json) | 15/15 사례·소스 해시·종료 감사 통과 | 0.005 s / 0.454 s | 0 |
| [`repro_r2`](../../artifacts/validation/2026-10-04/pipeline_fault_validation/deadline_followup/repro_r2/report.json) | 15/15 통과 | 0.005 s / 0.408 s | 0 |
| [`repro_r3`](../../artifacts/validation/2026-10-04/pipeline_fault_validation/deadline_followup/repro_r3/report.json) | 15/15 통과 | 0.005 s / 0.411 s | 0 |

판정: **재현되지 않음, 진단 대기 상태**. 3회 모두 `control_deadline_missed`·`applied_command_expired` 없이 RGB 빨강→초록 재출발, 명시 STOP·그래프 수렴, 모든 소유 프로세스 종료 코드 0·잔존 PID 없음을 확인했다. 3회에서 재현되지 않았다고 해서 간헐 지연이 해결됐거나 발생 확률이 낮다는 근거는 아니다. 실패를 얻기 위한 추가 반복은 하지 않았다.

관측 하나는 남긴다. 제어기 재시작 구간마다 검사기 자신의 수신 공백이 약 0.41~0.45 wall s로 커졌으나 sim 공백은 0.005 s였다. 시간을 발행하는 검사기가 멈추면 `/clock`도 진행하지 않으므로, 검사기의 정지만으로는 MCU의 sim 제어 공백이 생기지 않는다. 따라서 `ros_v2`의 공백은 검사기가 시간을 진행하는 동안 MCU 쪽 타이머가 따라오지 못한 경우와 일치하지만, 이는 추정이며 원인을 확정하지 않는다. 다음 발생 시 `report.json`의 `mcu_deadline_diagnostics`·`failure_state`로 판별한다. ROS 경로에서 고장을 일부러 주입해 진단 발행을 확인하는 시험은 하지 않았고, 발행 형식은 노드 콜백 단위 시험으로만 확인했다.

운영 조건: `/actuation/diagnostics`의 transient local 1건은 가상 MCU 노드가 살아 있는 동안에만 조회할 수 있고 현재 파일 자동 저장은 이 합성 검사기(`validate_pipeline_faults_ros.py`)만 수행하므로, 일반 Gazebo/B1 실행에서는 자동 보존되지 않는다. B1 시험기는 이 토픽을 구독해 보고서에 저장하고 노드 종료 전에 회수해야 한다.

[관련 회귀](../../artifacts/validation/2026-10-04/pipeline_fault_validation/deadline_followup/regression_after.log)는 **112개 통과**다(구동 계약·ROS 노드 콜백·조향 전송·검증기 판정 4개 파일, [명령](../../artifacts/validation/2026-10-04/pipeline_fault_validation/deadline_followup/run_regression.sh)). 새 시험은 한도와 같은 공백 50 ms에서는 고장·기록이 없고 50.001 ms에서는 고장이 나며, 첫 기록이 이후 tick·CLEAR·ARM·두 번째 공백에도 바뀌지 않는다는 것, CLEAR 없는 ARM 거절 정책이 그대로라는 것, 노드 진단이 한 번만 사본으로 발행된다는 것을 확인한다. 저장소 전체 724개와 34조건·Gazebo는 다시 실행하지 않았다. [재현 명령](../../artifacts/validation/2026-10-04/pipeline_fault_validation/deadline_followup/run_repro.sh), [변경 후 해시](../../artifacts/validation/2026-10-04/pipeline_fault_validation/deadline_followup/after_change_sha256.txt)를 보존했다.

### Gazebo 주행 중 보호층 종료

[결과](../../artifacts/validation/2026-10-04/pipeline_fault_validation/gazebo_safety_exit_v1/report.json)·[정지 궤적](../../artifacts/validation/2026-10-04/pipeline_fault_validation/gazebo_safety_exit_v1/stop_trace.json)·[당시 소스](../../artifacts/validation/2026-10-04/pipeline_fault_validation/gazebo_safety_exit_v1/source_snapshot.zip)·[실행 명령](../../artifacts/validation/2026-10-04/pipeline_fault_validation/run_gazebo_v1.sh)을 보존했다.

공식 코스·슬롯 3·`local_fast`·순차 C1 10 Hz·운동 보정 `both`·제어 50 Hz·`virtual_mcu`·방지턱 해제 조건이다. 운영 노드를 바꾸지 않고 검사기에 `safety_exit`를 추가했다. 소유 launch의 `lidar_safety` PID와 시작 시각을 확인한 뒤 **해당 노드만 SIGINT**로 종료했다. MCU STOP 서비스를 대신 호출하지 않았으며 MCU·출력 감시와 다른 센서는 계속 동작했다. 단일 노드의 정상 종료에 따른 발행 중단이며, Gazebo에서 SIGKILL 고장까지 시험한 것은 아니다.

| 항목 | 확인 결과 |
|---|---:|
| 고장 주입 전 진행 | 12.01849 m, 전체 랩 아님 |
| 주행 최고 속도 | 2.25250 m/s ≈ 8.11 km/h |
| 보호층 종료 요청 순간 속도 | 2.03045 m/s ≈ 7.31 km/h |
| 요청 후 안정 정지 판정까지 | 시뮬레이션 시간 1.534 s, 0.5초 안정 확인 포함 |
| 요청 위치→정지 위치 평면 변위 | 1.15424 m, 궤적 누적 길이와 다름 |
| 최종 MCU/출력 | `FAULT_LATCHED` / `command_expired` / 0 m/s·출력 차단 |
| 정지 중 충돌 표본 | 0 |
| 운동 상태 거절 | 0 / 42 상태 표본 |
| 종료 | 14/14 자식 정상 종료, 강제 정리·잔존 PID 없음 |

주행 중 명령 발행 중단에 대한 **가상 MCU 만료와 차량 물리 모델 정지**를 확인했다. 실물 MD20A 제동 모드·타이어/질량·실제 MCU 속도 제어는 검증하지 않았다. 이전 명시 STOP 시험의 약 0.78 m와 속도·종료 지연·주입 시점이 다르므로, 차이 전부를 MCU 만료 시간이나 성능 퇴보로 해석하지 않는다.

`wsl_nvidia` 요청에 대해 [OGRE 로그](../../artifacts/validation/2026-10-04/pipeline_fault_validation/gazebo_safety_exit_v1/ogre2.log)의 D3D12/NVIDIA RTX 5070 Laptop 사용을 확인했다. GPU 메모리 예산·mapped-buffer 경고는 남지만 이번 정상 종료 판정과 구분한다. 기존 오프라인 `sensor_wall_timeout_s=30`을 사용했고 ROS 시뮬레이션 시간의 명령 만료/피드백 한도를 완화하지 않았다.

## 4. 재현

WSL Ubuntu-24.04에서 ROS Jazzy와 `install/local_setup.bash`를 불러오고, `src/arena_autonomy`·`src/arena_vehicle_interface`를 `PYTHONPATH` 앞에 둔다. 전역 ROS 종료 명령을 쓰지 않으며 검사기가 생성한 격리 domain/launch의 PID·시작 시각·소유 관계만 대상으로 한다.

- [합성 통합 검사기](../../scripts/validate_pipeline_faults_ros.py): 새 `--output` 경로 필수. 상위 제어·보호·MCU를 모두 실제 별도 ROS 프로세스로 실행한다.
- [순수 판정 회귀](../../tests/test_pipeline_fault_validation.py): 오래된 상태·PID 재사용·잘못된 관측으로 시험이 통과하지 않는지 검사한다.
- [Gazebo 검사기](../../scripts/validate_basic_autonomy.py): `--actuation-mode virtual_mcu --actuation-stop safety_exit`로 주행 후 LiDAR 보호 노드만 SIGINT 종료한다. STOP 서비스는 대신 호출하지 않고 MCU 만료·차량 모형 정지를 별도로 확인한다.
- [소유 PID 검증·종료](../../scripts/lidar_shutdown.py), [종료 회귀](../../tests/test_lidar_shutdown.py): launch 소유 노드인지 두 번 확인하고 식별 실패 시 신호를 보내지 않는다.

## 5. 남는 한계와 다음 작업

이번 범위의 통과로 A4 전체·실차 안전을 완료 처리하지 않는다. 신선한 내용 고착, 그럴듯한 IMU/엔코더 오측정, 실물 C1 무응답 오정지, 실제 드라이버 시간·물리 STOP·MCU 멎음에 대한 독립 출력 차단은 남는다. 새 시험용 보호 시간·물성을 실측 사양으로 올리지 않는다.

이번 결과를 기준으로 다음 기능 개발은 **B1의 정지 표적/벽 가림과 앞차 추종·정지**를 권고한다. B1은 아직 구현하지 않았다. 이동 차량 추월과 고속화는 이후다. `ros_v2`의 갱신 공백은 미해결로 유지한다. 표적 재현 3회에서는 재발하지 않았고 첫 공백 진단이 준비됐으므로, B1 작업 중 재발하면 진단 기록으로 원인을 구분한다. 반복 보호 정지·누적 적체·정지 실패가 나오면 우선 조사한다. 동일 소스의 한 번 성공으로 정상 연속 가용성을 보증하지 않지만, PC/DDS의 모든 내부 원인 규명이 끝날 때까지 B1의 오프라인 기능 개발을 막지는 않는다.

중도 복구 절차는 B3에서 설계하되, 이번에는 운영 정책을 임의로 통일하거나 전체 재허가 구조를 새로 만들지 않았다. 실제 하드웨어·센서 로그가 생기면 C2에서 관측 한계와 제동/시간 경계를 확인한다.
