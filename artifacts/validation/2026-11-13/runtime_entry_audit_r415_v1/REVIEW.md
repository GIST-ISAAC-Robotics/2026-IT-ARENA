# r415 실물 운영 진입점 읽기 전용 감사

2026-11-13. 현행 이관 문서·운영 안내와 공통 parser/launch/firmware, r401·r403·r404·r407·r408의 해당 결과만 대조했다. **문서에 있는 preview/RaceOps/운영 CLI/jog 명령의 옵션 이름 오류는 발견하지 않았다. 다만 최종 운영 노드·host·실장 펌웨어를 이어 주는 인계가 빠져 있다.** 아래 4건을 구분해 반영하면 된다. 이번 작업은 이 문서만 작성했으며 장치·ROS·WSL·Jetson·빌드·flash·구동을 실행하지 않았다.

## 수정할 핵심 4건

| 항목 | 현재 근거와 영향 | 문서에 반영할 내용 / 실제 남은 작업 |
|---|---|---|
| 1. 구현 완료와 실물 확인의 구분 | [실물 이관](../../../../docs/hardware/PHYSICAL_HANDOVER_APPROVED_LAYOUT.md) §6은 PCNT·서보 UART·PWM/DIR·ROS↔MCU를 앞으로 구현하도록 적고, §9는 물리 STOP의 **펌웨어 구현**도 미확정으로 묶는다. 현행 [runtime drive main](../../../../src/arena_vehicle_interface/firmware_runtime_drive/main/main.cpp)은 IMU 초기화·encoder·UART·MD20A 출력·GPIO8 STOP을 연결한다. [host](../../../../scripts/run_runtime_actuation_host.py)는 원천 시각 보존/lease·서비스 ACK·종료 STOP을 구현했다. | ‘소프트웨어/HAL 연결·빌드는 완료, 미교정 구성 및 설치 상태·장착 동작은 별도’로 바꾼다. r404 실제 센서 수신과 r407 ROS/가짜 MCU 연결 결과를 재사용한다. §5.9의 ‘현재 앱에는 서보 제어가 없다’는 **당시 IMU 앱 복원 기록**으로 한정한다. 이 감사는 지금 장치에 설치된 앱을 조회하지 않았으므로 현재 설치 펌웨어를 단정하지 않는다. 기존 드라이버를 다시 구현할 과제로 남기지 않는다. |
| 2. preview와 최종 운영 진입의 분리 | 이관 문서 끝의 `run_hardware_sensor_preview.py ... --plan-only`는 parser와 일치하고 목적도 비구동 preview이다. 다만 `--vehicle-config`가 없으면 legacy preview 기하를 사용하며, final profile·runtime host·B3 시작 명령이 아니다. [r401 profile](../runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml)은 최종 기하/4구간 mask/원 frame/추정시각을 전달하지만 `race_control_mode=legacy`이다. [운영 안내](../../../../docs/autonomy/RACE_OPERATOR_RUNBOOK.md) §0의 RaceOps 실행만으로 planner/guard가 B3로 바뀌지 않는다. | 기존 preview 명령은 그대로 두고, 아래 **최종 profile을 받는 두 소비자와 host** 진입을 추가한다. B3를 planner와 guard 양쪽에 명시한다. [r407 profile](../runtime_ops_r407_v1/prepared/runtime_profile.yaml)의 변경은 B3 양쪽과 개발 상한 enable/0.3의 네 값이다. `local_pursuit.launch.py`는 `demo.launch.py`를 포함하는 시뮬 실행 경로이므로 실물 실행 템플릿으로 대체 사용하지 않는다. |
| 3. host 필수 입력과 실제 motor feedback 연결 | host parser(94–104행)는 `--port --output --seconds --clock-policy --max-speed-mps --max-steering-rad`를 요구한다. `--motor-feedback-config`가 없으면 encoder publisher를 만들지 않는다. r404는 encoder/host 없이 수행했고, legacy preview의 IMU bridge는 ESP 포트를 별도 소유한다. 따라서 preview 명령에 host만 덧붙이면 동일 포트 중복 소유 또는 feedback 누락이 생길 수 있다. | runtime firmware 운영 시 ESP UART는 host 하나가 소유하고, 그 host의 `/hardware_runtime/imu`와 `/drive_motor/encoder`를 두 노드가 사용하도록 인계한다. C1/RGB는 r404의 별도 생산자 연결을 재사용하되 예전 IMU bridge를 같은 ESP 포트에 병행하지 않는다. clock JSON은 `runtime_host_clock_policy_v1`의 명시 입력이며 TTL 두 값이 같아야 한다. [motor JSON](../runtime_profile_r399_v1/generated_v1/motor_feedback.json)의 28 count·11.911764705882353:1·반경0.0325 m를 유지하되 `encoder_sign=+1` 등 가정을 교정 완료로 승격하지 않는다. clock 가정 및 설치 부호를 확인한 별도 파일 경로가 실제 인계 입력이다. |
| 4. 실장 영점 → 운영 firmware 연결은 실제 미완료 | [steering_zero_profile.py](../../../../scripts/steering_zero_profile.py)는 `record`→`confirm-mount`→`check`/`export-cpp`를 제공한다. 그러나 [board_profile.hpp](../../../../src/arena_vehicle_interface/firmware_runtime_drive/main/board_profile.hpp)의 `installed_board_profile()`(50행)은 빈 `RuntimeBoardProfile`을 반환한다. 서보 geometry/명령 한계도 [RuntimeServoConfig](../../../../src/arena_vehicle_interface/portable/runtime_servo.hpp)에서 0으로 시작한다. [build_runtime_drive.sh](../../../../scripts/build_runtime_drive.sh)의 인자는 fresh build·ST source·evidence 세 개뿐이며, 위임받는 빌드도 외부 영점 JSON/header를 자동 소비하지 않는다. | 영점 JSON/export 완료를 운영 firmware에 적용된 것으로 쓰면 안 된다. 사람이 얻은 중립 count·CAD 방향 대비 count 부호·position mode·안전 count 범위/assembly ID와 기존 계획의 모터 방향·encoder 방향·PI/duty/속도 한계, servo ID/주기/속도 및 최종 geometry를 **실장 board profile에 연결하는 통합**이 남는다. `p.servo.zero`에는 export된 `arena_steering_mount::Profile()`을 연결할 수 있으나 현재 자동 연결은 없다. 이는 문장 수정만으로 완료되지 않는다. 값이 없으면 `profile_valid=false`, 준비/ARM 거절이 정상이다. host의 `--enable-drive-wire`가 이를 대신하지 않는다. |

네 번째 항목은 새 안전 조건이나 새 시험 제안이 아니다. 현행 `RuntimeBoardProfile.valid()`가 이미 요구하지만 아직 채워지지 않은 입력·연결의 목록이다. 기존 [장착 조향 인계](../../../../docs/hardware/STEERING_FUSION_2026_11_13.md)의 ‘실제 프로필과 운영 HAL 연결은 후속’이라는 설명은 현행 코드와 맞는다. 엔코더 count/방향은 firmware의 `encoder_direction` 적용과 host의 `encoder_sign`을 같은 원천 의미로 정해야 하며, 양쪽에서 서로 독립적으로 부호를 뒤집어 확정하면 안 된다.

## 잘못된 옵션으로 분류하지 않은 것

- r401의 `g1_g2_simulation_trial=true`는 이름에 simulation이 있으나 [G1G2Node](../../../../src/arena_autonomy/arena_autonomy/g1_g2_node.py)의 253–255행은 등록된 hardware 환경도 명시 허용한다. 실물 설정은 `execution_environment=hardware_runtime_candidate_v1`, `use_sim_time=false`, `host_estimated_v1`, `motion_scan_timing_verified=false`이다. 이 flag를 임의로 끄거나 시각 검증을 true로 바꾸지 않는다.
- 반대로 `g1_current_corner_budget=r396_v1`는 [선택 검사](../../../../src/arena_autonomy/arena_autonomy/runtime_node_backend.py)가 verified simulation만 허용한다. 실물은 `off`를 유지한다. r406 등의 시뮬 후보/5구간 재현 mask를 r401 최종4구간 설정에 혼합하지 않는다.
- r401이 쓰는 `official_4red_local_candidate`와 운영 안내 §1의 `official_4red_candidate`는 모두 실제로 등록된 값이다. 후자는 parser 오류는 아니지만 최종 profile을 실행할 때는 그 profile의 전자를 유지한다.
- RaceOps의 `vehicle_id/host/port/token_path/record_path/log_path`, `race_operator.py`의 기존 명령/`--send`, `development_jog.py forward --speed .15 --seconds .5 --speed-limit .2`는 현행 선언과 일치한다. jog는 기본 미전송이며 자동 ARM/RESET이 없다. r408의 50순수검사를 실물 송신 완료로 바꾸지 않는다.

## 운영 진입 명령 3개

아래는 **현재 parser와 기존 profile을 맞춘 인계 템플릿**이며 이번에 실행한 명령이 아니다. Jetson에서 ROS Humble 및 이 공통 코드가 설치된 overlay를 불러오고 저장소 루트를 작업 디렉터리로 삼는다. 모든 터미널은 같은 `ROS_DOMAIN_ID`, ROS/Python 환경을 사용한다. 로컬 결합에는 기존 `ROS_LOCALHOST_ONLY=1`을 유지할 수 있다. 소스 Python 경로는 `PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:${PYTHONPATH:-}"`이다. ROS overlay를 빼거나 WSL Jazzy 설치 경로를 Jetson에 그대로 복사하지 않는다.

`RUN_OUTPUT`은 새 결과 디렉터리로 정해 부모 폴더를 준비한다. `RUNTIME_PORT`는 해당 runtime firmware가 설치된 ESP의 실제 by-id 경로이다. `CLOCK_POLICY`와 `MOTOR_FEEDBACK_CONFIG`는 위 3번에서 확인한 JSON 파일의 절대 경로로 정한다. 값이 빠지면 아래 shell 표현은 실행 전에 중단한다. 예제의 host는 ARM/DRIVE wire를 기본 차단하며, **사람이 실장 구성/출력 허가를 완료한 뒤 명시적으로 `--enable-drive-wire`를 추가하는 단계**와 구별한다. 그때도 자동 ARM은 없으며 RaceOps의 현재 상태/버튼 계약을 따른다.

1. 독립 보호층. r401의 `lidar_safety` 항목을 정확한 노드명으로 읽고 C1 원천 topic만 remap한다.

```bash
ros2 run arena_autonomy lidar_safety --ros-args \
  --params-file "$PWD/artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml" \
  -p race_control_mode:=b3_candidate \
  -r /scan/native:=/hardware_runtime/c1/native
```

2. 계획/현재 계산 노드. r401과 같은 final geometry/time/frame을 사용하고 r407에서 연결 확인한 B3·명시 개발 상한만 선택한다. 노드명을 바꾸면 YAML의 `local_pursuit` 항목이 적용되지 않을 수 있으므로 remap하지 않는다.

```bash
ros2 run arena_autonomy g1_g2_handoff_trial --ros-args \
  --params-file "$PWD/artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml" \
  -p race_control_mode:=b3_candidate \
  -p development_speed_limit_enabled:=true -p development_speed_limit_mps:=0.3 \
  -p g1_current_corner_budget:=off \
  -p g1_trace_path:="${RUN_OUTPUT:?새 결과 디렉터리}/follow_decisions.jsonl" \
  -r /scan/native:=/hardware_runtime/c1/native \
  -r /camera/color/image_raw:=/hardware_runtime/camera/image_raw
```

3. runtime UART host. 최종 송신 상한은 중앙등가 조향의 float32 값이며 서보축 각도/degree가 아니다. 1.5 m/s는 r401 설정의 명령 상한이고 위 개발 상한0.3 m/s와 구분한다. motor JSON의5 m/s는 feedback 타당성 한계로, 주행 목표/허가 속도가 아니다.

```bash
python3 scripts/run_runtime_actuation_host.py \
  --port "${RUNTIME_PORT:?runtime ESP by-id 경로}" \
  --output "${RUN_OUTPUT:?새 결과 디렉터리}/host" --seconds 60 \
  --clock-policy "${CLOCK_POLICY:?명시 clock policy JSON}" \
  --motor-feedback-config "${MOTOR_FEEDBACK_CONFIG:?확인한 motor feedback JSON}" \
  --imu-topic /hardware_runtime/imu \
  --max-speed-mps 1.5 --max-steering-rad 0.4578436017036438
```

세 명령이 전체 장치를 자동으로 시작하지는 않는다. 실제 C1·RGB 생산자와 원시각/원 frame 계약은 [r404 결과](../runtime_live_r404_v1/RESULT.md)를 따른다. 웹 운영 프로세스는 [운영 안내 §0](../../../../docs/autonomy/RACE_OPERATOR_RUNBOOK.md)의 현행 `race_ops` 명령을 그대로 사용한다. `/drive/safe`는 보호층만 발행하며 여기서 직접 발행하는 명령은 없다. host60초는 이번 템플릿의 제한 실행 시간일 뿐 새 TTL/제동 한계가 아니다. 종료 STOP송신/ACK와 실제 차체 정지는 구별한다.

## 이미 확보된 결과와 사람에게 남기는 입력

검증 근거는 [r401](../runtime_launch_profile_r401_v1/RESULT.md)의 최종 profile 전달·실제 ROS 두 노드 구성, r403 원 frame 수용/거절 공통 연결, [r404](../runtime_live_r404_v1/RESULT.md)의 실제 C1 183·RGB400·IMU3,795 전달과 양쪽0명령/정상 회수, [r407](../runtime_ops_r407_v1/RESULT.md)의 실제 ROS/가짜 MCU PTY에서 STOP·정지 중 상한·RESET 없는 **재개 준비**·신호 대기 밖 수동출발 거절, [r408](../development_jog_r408_v1/RESULT.md)의 최신 host 상태에 대한 jog 순수검사이다. r407의 센서 없는 재개 수락은 ARM 증가/양수 주행이 아니며, 원 stale 요청 실패도 보존한다.

조립 후 사람에게 남는 것은 기존 계획의 **설치된 firmware/포트 확인, 장착 영점·좌우 한계, 모터/encoder 방향과 출력 구성, 실제 센서 축/위치**이다. final vehicle YAML의 축거0.128 m·윤거0.1239 m·타이어폭0.0261 m·반경0.0325 m, marker Z0.09 m의4가림은 선택한 설계 입력이지 실측 설치 결과가 아니다. C1 `c1_unmounted_preview`, IMU `lsm6dsox_sensor`, RGB `b0390_receive_time_unverified` 이름과 false timing provenance는 이름만 바꾸거나 재stamp하여 검증값으로 만들지 않는다. 이미 끝난 개별 센서 수신·손회전·서보 무부하 검사를 다시 요구하지 않는다.

현재 사람 없는 상태에서 문서/진입 명령 정리는 가능하나, **실물 운영 완료 선언에는 4번의 미교정 board profile 연결과 조립 후 입력이 남아 있다.** 새 보호층·정책·의무 시험은 제안하지 않는다. root가 운영 문서에 통합할 수정 근거로 이 문서만 인계한다.
