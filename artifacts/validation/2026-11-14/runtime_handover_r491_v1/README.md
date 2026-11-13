# r491 실물 이관 소스 후보와 지상 시연 명령

Windows 파일 작업 전용이다. 장치·ROS·WSL·Jetson·flash·ARM·구동을 실행하지 않았다. 실제 영점·방향·PI·duty·포트는 `null`이며 합성 실장값을 넣지 않았다. 최초 목록 `inventory_v1`과 리소스 보완 목록은 그대로 보존한다. r492 회수 후 주 담당이 r491/r494/r495를 공통 이관했고, 그 최신 원본으로 [final_v1 ZIP](final_v1/runtime_handover_sources.zip)을 **1회 생성**했다. 721파일/필수 누락0/전체 SHA·공통 원문·이번 이관6파일 일치를 확인했다. ZIP은 로컬 보존·Git 제외이며 source-only다. [최종 영수증](final_v1/verification_receipt.json).

## 소스 묶음 준비

r404의 두 패키지+scripts 재귀 수집을 재사용한다. `.sh`, CMake, sdkconfig도 포함하고, 기존 native `.so`/캐시/빌드/설치 산출물과 private 폴더는 제외한다. ARM 확장은 목적지 환경에 이미 검증된 동일 소스 빌드가 있는지 주 담당이 판단한다. x86 `.so`를 ARM에 넣지 않는다. `runtime_installed_limits.py`나 정책을 prepare가 수정하지 않는다.

완료한 생성 명령은 다음과 같다. `final_v1`은 이미 존재하므로 재실행하지 않는다. 도구는 기존 출력 덮어쓰기를 거절한다.

```powershell
python -B artifacts/validation/2026-11-14/runtime_handover_r491_v1/stage/scripts/prepare_runtime_handover.py --output artifacts/validation/2026-11-14/runtime_handover_r491_v1/final_v1 --archive
```

ZIP SHA-256: `73bebdcdc176eef04b641dc78c986b76ec32431b194c787cba0f351ad6f95965`. manifest SHA-256: `ee450fef0d971d2b94e75544420de5e13d6e9e694b4d2a12bdf6f848581098a5`.

필수 목록은 prepare의 `REQUIRED`에 있다. r474 route, r481 local_follow, r482 known_structure/native 소스, r486 moving_pass/entry, r489 independent_guard_route와 runtime host·설치 profile·영점·운영 패널/jog·preview/센서 bridge·빌드 도구를 포함한다. 패키지의 다른 하위 모듈도 재귀 포함되며 각 파일 SHA가 manifest에 남는다. 빠진 필수 파일이 있으면 archive를 만들지 않고 exit2다. 미래 새 모듈도 수집되지만 실제 선택/설치/활성화는 별도다.

`handover/runtime_profile.template.yaml`은 r401 원 설정을 참고용으로 보존한 것이다. 장착 profile이나 최신 실물 활성화 선택이라고 부르지 않는다. 과거 r404 `bundle.zip`은 재생성하지 않는다. 공통 A3 `prepare_jetson_bundle.py`는 runtime scripts를 완비한 운영 묶음이 아니므로 이 작업에서 사용하지 않는다.

공개 리소스는 확장자를 일괄 허용하지 않고 다음 6개만 `PUBLIC_RESOURCES`로 포함한다. 원문/SHA를 유지하고 자동 선택하지 않는다.

| 경로 | 실제 읽기/사용 경계 |
|---|---|
| `config/hardware/g1_input_shadow_v1.yaml` | `probe_g1_hardware_shadow.ros_arguments()`의 기본 params-file. preview 실행에 필요하며 실장 주행 profile이 아니다. |
| `config/perception/vehicle_marker_approx_b0390.json` | 명시 `vehicle_marker_profile_path`용 선택형 공개 근사 profile. 기본은 빈 경로/OFF이며 `accuracy_verified=false` 유지. |
| `config/perception/vehicle_marker_gist49_calibration_only.json` | 명시 이미지 점검/보정용 공개 템플릿. `purpose=calibration_only`로 runtime occupancy용이 아니다. |
| `config/perception/vehicle_marker_samples_template.json` | 보정 입력 빈 서식. `measured_distance_m` 등 원 `null`을 그대로 보존한다. |
| `src/arena_description/config/vehicle.final_cad_2026_11_13.yaml` | preview `--vehicle-config`의 명시 최종 CAD 기하 입력용. 실측 설치값이나 기본 선택이 아니다. |
| `src/arena_description/config/b0390_c1_cad_20261113_final_nowire.yaml` | 공개 최종 CAD 센서 규격/설계 참고. 별도 활성화 없이 포함만 한다. |

서보 generator는 이미 수집되는 `steering_cad_table.py`의 `SAMPLES`를 사용하므로 외부 CSV를 기본으로 읽지 않는다. `CadMapping.from_csv()`는 명시 경로용이며 이전 CAD 원본 자료 전체를 추가할 이유가 없다. C1/IMU/RGB preview의 규격은 해당 Python/C++ 구현에 있으며 추가 센서 JSON을 기본으로 읽지 않는다. YAML 안의 출처 이미지/URL은 provenance 문자열이고 이 운영 경로가 다시 읽는 파일이 아니므로 원본 사진/지도/artifacts를 따라 복사하지 않는다.

반대로 runtime host의 `--clock-policy`는 필수 명시 JSON, `--motor-feedback-config`는 명시 선택 JSON이다. 설치 generator의 board/zero, 설치 limits의 선택 YAML/manifest와 함께 **새 실장 입력으로 별도 제공할 항목**으로 남긴다. 예전 시험 JSON·credential·실측 보정물을 자동 복사/채택하지 않는다. `resources_inventory_v2`는 721파일/필수 누락0이며 ZIP은 아직 생성하지 않은 중간 목록이다.

## 실장 입력을 받은 뒤 파일 생성

아래 Bash 변수는 확인한 실제 값/파일의 절대 경로를 사용한다. `NEUTRAL_COUNT`, `DIRECTION`, `SAFE_MIN`, `SAFE_MAX`, `BOARD_JSON`을 임의 예제로 채우지 않는다. `BOARD_JSON.maximum_speed_mps=0.1`은 이번 지상 명령 ceiling이며 영점/PI/방향의 실측 대체가 아니다. 초기 목표 .05와 MCU 최대 .1은 다르다. MCU의 기존 progress 최소명령 .1 때문에 MCU 최대를 .05로 내리는 것은 유효 profile이 아니다.

```bash
python3 scripts/steering_zero_profile.py record \
  --neutral-count "${NEUTRAL_COUNT:?실제 count}" --assembly-id "${ASSEMBLY_ID:?}" \
  --confirm-wheels-straight --evidence-note "${ZERO_EVIDENCE:?}" --output "${ZERO_RAW:?새 경로}"
python3 scripts/steering_zero_profile.py confirm-mount "$ZERO_RAW" \
  --direction "${DIRECTION:?실제 부호}" --safe-min-count "${SAFE_MIN:?}" --safe-max-count "${SAFE_MAX:?}" \
  --confirm-position-mode-and-mounted-limits --output "${ZERO_MOUNTED:?새 경로}"
python3 scripts/runtime_installed_profile.py prepare \
  --board-profile "${BOARD_JSON:?실장 확인값}" --steering-profile "$ZERO_MOUNTED" \
  --assembly-id "$ASSEMBLY_ID" --output "${INSTALLED:?새 경로}"
python3 scripts/runtime_installed_profile.py verify --profile-dir "$INSTALLED"
python3 scripts/runtime_installed_limits.py prepare --installed-profile "$INSTALLED" \
  --runtime-params "${REVIEWED_RUNTIME_YAML:?주 담당이 선택한 실제 운영 YAML}" --output "${SELECTED:?새 경로}"
python3 scripts/runtime_installed_limits.py verify --selection "$SELECTED" --installed-profile "$INSTALLED"
```

생성 YAML을 planner/guard 양쪽에 선택해야 한다. r489 builder는 해당 planner 선택에서 guard 정책도 연결한다. 실물 `host_estimated_v1`, `timing_verified=false` 원천 계약을 그대로 유지하며, 파일이 들어 있다는 이유로 estimated crop/corner를 기본 활성화하지 않는다.

빌드 환경에서 사용할 **빌드만 하는** 진입점은 아래와 같다. 이번 작업에서 실행하지 않았으며, flash는 포함하지 않는다. 단독 `export-cpp`만으로 설치 profile이 선택되지 않는다.

```bash
bash scripts/build_runtime_drive.sh "${FRESH_BUILD:?}" "${PINNED_ST_SOURCE:?}" "${BUILD_EVIDENCE:?}" --installed-profile "$INSTALLED"
```

## 조립 후 지상 .05 시작 / .1 상한

확인한 Jetson ROS/overlay와 같은 domain을 모든 터미널에 적용한다. 경로는 복사한 새 묶음 루트 기준이다. Python 소스와 ROS executable의 overlay가 다른 버전이면 소스 묶음만 새로 만들었다고 실행 코드가 갱신된 것은 아니다. C1/IMU/카메라 장착·축/시각 확인과 실제 runtime firmware 설치 확인이 먼저다. runtime host가 UART를 소유하면 같은 포트의 preview IMU bridge를 동시에 실행하지 않는다.

```bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:${PYTHONPATH:-}"
python3 scripts/runtime_installed_limits.py verify --selection "$SELECTED" --installed-profile "$INSTALLED"
HOST_ARGS_JSON=$(python3 scripts/runtime_installed_limits.py host-arguments --selection "$SELECTED" --installed-profile "$INSTALLED")
mapfile -t HOST_LIMIT_ARGS < <(printf '%s' "$HOST_ARGS_JSON" | python3 -c 'import json,sys; print("\n".join(json.load(sys.stdin)))')
L=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["max_steering_rad"])' "$SELECTED/runtime_host_limits.json")
MANUAL_L=$(python3 -c 'import sys; print(min(.2,float(sys.argv[1])))' "$L")
```

독립 guard:

```bash
ros2 run arena_autonomy lidar_safety --ros-args --params-file "$SELECTED/runtime_profile.yaml" \
  -p race_control_mode:=b3_candidate -r /scan/native:=/hardware_runtime/c1/native
```

runtime host는 아래 상태로 송신 차단이다. 조립 후 출력 점검 때 `--enable-drive-wire`를 **명시 추가**하며, 그것도 자동 ARM은 아니다. 이전 r415의 host1.5/development.3 값을 지상 시연에 재사용하지 않는다.

```bash
python3 scripts/run_runtime_actuation_host.py --port "${RUNTIME_PORT:?실제 runtime ESP by-id}" \
  --output "${RUN_OUTPUT:?새 경로}/host" --seconds 60 \
  --clock-policy "${CLOCK_POLICY:?확인한 JSON}" --motor-feedback-config "${MOTOR_FEEDBACK_CONFIG:?확인한 JSON}" \
  --imu-topic /hardware_runtime/imu --max-speed-mps 0.1 "${HOST_LIMIT_ARGS[@]}"
```

수동 패널 구성은 자율 planner를 동시에 시작하지 않는다. UI에서 처음 목표 **.05 m/s**, 짧은 단발 **.5 s**를 선택한다. 아래 최대는 .1이며 실제 거리 .025m를 보증하는 값은 아니다. 준비/ARM/실행은 기존 패널 상태 절차를 사용한다.

```bash
ros2 run arena_autonomy race_ops --ros-args -p operation_mode:=manual \
  -p manual_maximum_mps:=0.1 -p manual_steering_limit_rad:="$MANUAL_L" \
  -p token_path:="${OPS_TOKEN_PATH:?비공개 경로}" -p manual_output_root:="$RUN_OUTPUT/manual"
```

패널 대신 CLI의 **미송신 미리보기**를 보는 명령:

```bash
python3 scripts/development_jog.py forward --speed .05 --seconds .5 --speed-limit .1 \
  --steering-deg 0 --max-steering-rad "$L"
```

실제 단발은 이미 ARM·보호층·장착 STOP 점검을 마친 상태에서만 `--send --output "$RUN_OUTPUT/jog"`를 명시한다. 패널 수동 자식과 CLI를 동시에 발행하지 않는다. 자율 노드는 별도로 같은 선택 YAML과 `race_control_mode:=b3_candidate`, `development_speed_limit_enabled:=true`, `development_speed_limit_mps:=0.05`가 필요하다. 현재 자율 UI의 최대는 YAML `max_speed_mps`에서 오므로 수동 최대 .1 설정과 혼동하지 않는다. 이번 명령 카드는 지상 수동 시연 우선이며 자율 .1 최대의 UI/실물 적용 완료를 선언하지 않는다.

STOP은 패널 전체 STOP 또는 수동 STOP이다. CLI 운영 STOP은 `python3 scripts/race_operator.py stop --send`, 독립 host 서비스 요청은 `ros2 service call /actuation/stop std_srvs/srv/Trigger '{}'`이다. 실제 GPIO8 STOP도 별도로 눌러 PWM/차체 정지를 확인한다. ACK, 코드 연결, 종료 로그는 실물 정지 증명과 다르다. 스위치 해제만으로 재시작하지 않는 기존 latch를 유지한다.

## 반드시 남는 제한

- `.1`은 ROS float32 최근접 변환 시 `.10000000149011612`여서 정확한 host/MCU `.1`보다 커진다. 이번 ZIP에 포함된 r491 공통 수정은 수동/jog와 자율 발행 지점에서 기존 하향 함수를 사용해 `.09999999403953552`를 전송한다. 보호 한도를 epsilon으로 늘리지 않는다. 이전 설치본에는 이 수정이 없을 수 있으므로 새 소스 ZIP 생성과 실제 실행 코드 갱신을 구분한다.
- **13 km/h는 현재 지정한 명령값 한도일 뿐 실측 과도 속도의 절대 상한이 아니다.** 현행 MCU `max_wheel_mps=5 m/s` 피드백 범위와 별개이며, 실제 차단 경로 설정·확인 전 공중 고속시험을 진행하지 않는다. 기존 jog 자체 허용 최대도 .2m/s다. 값 하나를 5→3.61로 바꾸어 물리 과도 속도 보증이라고 하지 않는다.
- `max_wheel_mps`는 [receiver](../../../../src/arena_vehicle_interface/portable/actuation_receiver.hpp) 37행의 기본값이며 370–374행은 초과 feedback를 무효화하고 ARMED이면 `encoder_implausible` FAULT로 전이한다. 260–273행은 목표0/세대폐기, [main](../../../../src/arena_vehicle_interface/firmware_runtime_drive/main/main.cpp) 117–119행은 unarmed PWM0이다. 비ARM이면 invalid feedback 반환이다. 물리 관성/샘플 간 최고속도를 보증하지 않는다.
- 이번 ZIP의 [board_profile](../../../../src/arena_vehicle_interface/firmware_runtime_drive/main/board_profile.hpp)와 [profile generator](../../../../scripts/runtime_installed_profile.py)는 r495의 선택형 `max_wheel_mps` 전달을 포함한다. 생략 시 기존5.0이며 명시 유한 양수·명령 상한 이상인 값만 전달한다. `maximum_speed_mps`는 별도 **명령** 한도다. 실제 board JSON/경계값은 아직 선택하지 않았다. 주 담당의 [PC C++ 계약 검사](../installed_feedback_limit_r495_v1/integration_v1/native_receipt.json)는 compile/run exit0이며 firmware/실물 검증이 아니다. host motor-feedback JSON의 `maximum_speed_mps`는 관측 타당성 한계이며 MCU 차단 설정을 바꾸지 않는다.
