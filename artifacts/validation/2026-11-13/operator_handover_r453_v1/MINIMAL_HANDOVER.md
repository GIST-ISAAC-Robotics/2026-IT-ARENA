# 운영 안내에 합칠 최소 명령안 — 실행하지 않은 제안

2026-11-13, GPT. r439의 설치 조향 상한 선택과 r444/r446의 명시 estimated 진입, r429 로그 경로를 한 절에 연결할 초안이다. 현재 선택·실행·출력 허가가 아니다. 실장 bundle/영점/부품값을 생성하지 않는다. 기존 원 보고서는 보존한다.

## 선행 선택

Jetson의 확인한 ROS/설치 overlay와 같은 ROS domain을 사용한다. r415의 환경·센서 생산자/단일 ESP 포트 소유 절차를 유지한다. 아래 변수는 절대 경로로 직접 지정하며 가상 값을 넣지 않는다.

| 입력 | 의미 |
|---|---|
| `INSTALLED_PROFILE_DIR` | 실제 확인 영점/board로 만든 설치 bundle. firmware 빌드에 선택한 것과 동일 |
| `BASE_RUNTIME_YAML` | 기존 운영 입력. r401은 설계·추정시각 profile이며 실장 교정 완료가 아님 |
| `SELECTED_RUNTIME` | r439가 새로 생성하는 설정 폴더 |
| `RUN_OUTPUT` | 이번 실행의 새 결과 폴더. host가 만들 하위 `host` 폴더는 미리 만들지 않음 |
| `TOKEN_PATH` | 확인한 운영 토큰 파일 경로. 부모 폴더는 존재해야 함 |
| `RUNTIME_PORT`, `CLOCK_POLICY`, `MOTOR_FEEDBACK_CONFIG` | 설치 앱/포트와 명시 clock/실제 선택 motor feedback 파일 |

r439 RESULT의 `prepare`→`host-arguments`→`verify` 명령을 그대로 사용한다. 그 결과인 `INSTALLED_HOST_ARGUMENTS` 배열과 `MANUAL_STEERING_LIMIT=min(0.2, 생성 L)` 계산도 그대로 재사용한다. 값은 반올림하거나 생성 YAML/manifest를 사후 수정하지 않는다. 여러 터미널은 같은 선택 파일과 인자를 명시한다(Bash 배열은 자동 상속되지 않으므로 host 터미널에서 r439 배열 생성/verify를 수행한다).

선택은 둘 중 하나다. 아래 실행 예제는 두 번째를 **명시 선택했을 때만** 사용한다.

| 선택 | planner | guard |
|---|---|---|
| 기존 현재 후보 OFF | scope 기본, crop false, corner off 유지 | scope 기본, crop false 유지 |
| 실물 estimated 관측 후보 | scope=`host_estimated_crop_v1`, crop=true, corner=`r396_v1`, 기존 native 두 계산 유지 | 같은 scope/crop |

두 번째도 `execution_environment=hardware_runtime_candidate_v1`, `lidar_input_representation=native_estimated_v1`, `lidar_compensation=both`, `use_sim_time=false`, `motion_scan_timing_verified=false`, `motion_scan_time_policy=host_estimated_v1`이다. r401의 이 값을 유지하며 verified로 바꾸지 않는다. 일반 `local_pursuit` 실행 파일로 대체하지 않는다.

## 같은 설치 선택으로 자율 구성

기존 C1/RGB 생산자는 별도로 실행하며 이전 IMU bridge를 runtime host와 같은 ESP 포트에 병행하지 않는다. 각 명령은 기존처럼 별도 터미널에서 실행하는 예제다.

```bash
ros2 run arena_autonomy lidar_safety --ros-args \
  --params-file "${SELECTED_RUNTIME:?설치 선택 폴더}/runtime_profile.yaml" \
  -p race_control_mode:=b3_candidate \
  -p g1_observation_input_scope:=host_estimated_crop_v1 \
  -p g1_crop_edge_component_trial:=true \
  -r /scan/native:=/hardware_runtime/c1/native
```

```bash
ros2 run arena_autonomy g1_g2_handoff_trial --ros-args \
  --params-file "${SELECTED_RUNTIME:?설치 선택 폴더}/runtime_profile.yaml" \
  -p race_control_mode:=b3_candidate \
  -p development_speed_limit_enabled:=true -p development_speed_limit_mps:=0.3 \
  -p g1_observation_input_scope:=host_estimated_crop_v1 \
  -p g1_crop_edge_component_trial:=true -p g1_current_corner_budget:=r396_v1 \
  -p g1_trace_path:="${RUN_OUTPUT:?새 결과 폴더}/follow_decisions.jsonl" \
  -r /scan/native:=/hardware_runtime/c1/native \
  -r /camera/color/image_raw:=/hardware_runtime/camera/image_raw
```

상한0.3은 기존 r439 예제의 명시 초기값을 보존한 것이며 패널 표시의 필수 개발 플래그가 아니다. 패널은 별도 플래그 없이 상한을 제공하고, 상한 변경은 기존 정지·MCU 비ARM 상태에서 수행한다. 이 값은 실제 속도나 장착 허용값의 증명이 아니다.

host는 r439의 동일 명령/`INSTALLED_HOST_ARGUMENTS`를 사용한다. `--output "$RUN_OUTPUT/host"`, 명시 clock/motor JSON, IMU topic을 유지한다. r439 예제의 `--seconds 60`은 실행 시간이고 command TTL이 아니다. **기본 예제에 `--enable-drive-wire`가 없어 실제 ARM/DRIVE 전송은 차단된다.** 실제 설치/출력 허가 후 이 옵션을 명시하는 기존 단계가 남으며, GUI 준비 버튼도 그 단계를 대신하지 않는다. 이 초안에서 옵션을 자동 추가하지 않는다.

```bash
ros2 run arena_autonomy race_ops --ros-args \
  -p vehicle_id:=isaac -p host:=0.0.0.0 -p port:=8765 \
  -p token_path:="${TOKEN_PATH:?확인한 운영 토큰}" \
  -p record_path:="${RUN_OUTPUT:?새 결과 폴더}/race_record.json" \
  -p log_path:="$RUN_OUTPUT/ops_autonomous.jsonl"
```

처음에는 최초 출발 준비→WAIT_GREEN의 실제 신호/명시 수동 확인을 따른다. 이미 출발한 경기의 일반 명시 STOP 뒤에는 START로 재개 준비하며, RESET은 정지를 유지한 상태 초기화다. UI의 준비/서비스 ACK와 실제 차체 정지는 구별한다. 미리보기는 기본 OFF이며 기존 토픽만 선택한다.

## 수동 구성은 별도로 선택

자율 planner를 종료·회수하고 다른 `/drive` publisher가 없는 상태에서 guard+host+패널을 사용한다. 기존 자율 프로세스를 자동으로 죽이거나 두 발행자를 허용하지 않는다. guard에는 같은 생성 YAML과 선택 scope를 사용한다. 자율 패널 프로세스도 종료하고 다음 구성 하나만 시작한다.

```bash
ros2 run arena_autonomy race_ops --ros-args -p operation_mode:=manual \
  -p vehicle_id:=isaac -p host:=0.0.0.0 -p port:=8765 \
  -p token_path:="${TOKEN_PATH:?확인한 운영 토큰}" \
  -p manual_steering_limit_rad:="${MANUAL_STEERING_LIMIT:?r439의 min(0.2,L)}" \
  -p log_path:="${RUN_OUTPUT:?새 결과 폴더}/ops_manual.jsonl" \
  -p manual_output_root:="$RUN_OUTPUT/manual_jogs"
```

정지 상태에서 패널 상한 적용→수동 준비(CLEAR/ARM,0목표)→목표속도/시간/중앙 등가 바퀴각 선택→단발→0목표/STOP 확인을 따른다. 최대0.2m/s·2초, 조향은±min(0.2rad, 설치 생성 상한), +좌/−우다. `v×t`는 등속 가정 예상 이동거리다. 수동은 매 단발 후 다시 준비하며 자율 START/RESET 의미를 적용하지 않는다. 서버 운영 최대값·pulse 유효 상한을 구별하고 실장 속도/출력 입력이 미정이면 확인 완료로 채우지 않는다.

## STOP 뒤 회수

자율 사건은 기존 요청번호로 좁히고 판단 trace를 연결할 수 있다. 아래 `REQUEST_ID`는 실제 로그의 요청번호다. 없는 값을 만들지 않는다.

```bash
python3 scripts/race_event_export.py \
  --log "$RUN_OUTPUT/ops_autonomous.jsonl" \
  --decisions "$RUN_OUTPUT/follow_decisions.jsonl" \
  --request "${REQUEST_ID:?기록된 사건 요청번호}" \
  --output "$RUN_OUTPUT/event_autonomous"
```

수동에는 자율 status 기반 `sim_s`가 없으므로 요청 중심 필터를 쓰지 않고 전체 기록을 내보낸다. `wall_s`는 해당 프로세스의 monotonic 기록이며 물리 취득시각이나 별도 장치 시계와 자동 일치하는 값이 아니다.

```bash
python3 scripts/race_event_export.py \
  --log "$RUN_OUTPUT/ops_manual.jsonl" --output "$RUN_OUTPUT/events_manual"
```

종료 뒤 RUN_OUTPUT 전체를 회수한다: 패널 원 JSONL/CSV/HTML, 자율 trace·경기 복구 기록(자율일 때), `manual_jogs`의 request/result/stdout(수동일 때), `host`의 registered/events/summary/원 UART 자료, 선택한 설치/운영 manifest와 실제 실행 인자. 동일 run의 파일을 연결하며 host ACK·새 MCU 정지 상태와 실제 차체 정지를 구분한다. 현재 exporter의 `sim_s` 명칭은 자율 노드가 기록한 ROS 시간 필드명이며 실물 `use_sim_time=false`에서는 시뮬 시간이라고 설명하지 않는다. 원 상세 자료는 기존 Git 제외 정책을 유지한다.
