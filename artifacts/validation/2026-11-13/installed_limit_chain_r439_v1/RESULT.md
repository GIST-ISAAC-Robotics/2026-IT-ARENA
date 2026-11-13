# r439 설치 조향 상한의 운영 설정 연결 — stage 결과

2026-11-13, GPT. [읽기 전용 감사](REVIEW.md) 뒤 주 담당의 추가 지시로 최소 도구를 독립 stage에 구현했다. **아래는 stage 검증 당시 결과이며, 후속 주 담당 검토에서 신규2파일을 공통 이관했다.** [이관 영수증](integration.json). 기존 설치 export·firmware bundle·빌더·ROS 노드·host 코드는 수정하지 않았다.

## 변경

- 신규 [`scripts/runtime_installed_limits.py`](stage/scripts/runtime_installed_limits.py): 기존 `runtime_installed_profile.verify()`로 선택한 설치 bundle을 확인하고, 그 `runtime_board.json`의 `servo.command_limit`만 상한 원천으로 사용한다. 기존 `vehicle_geometry.float32_limit_not_above()`를 재사용한다.
- 신규 [`tests/test_runtime_installed_limits.py`](stage/tests/test_runtime_installed_limits.py): 축소/기존 상한 전달, 원 설정 보존, 선택 bundle 혼합·ROS/host 불일치와 파일 변경 거절을 확인한다. 합성 fixture는 장착값이 아니다.

입력은 **검증된 설치 bundle + 기존 runtime YAML**, 출력은 새 디렉터리의 원 YAML 사본·파생 `runtime_profile.yaml`·`runtime_host_limits.json`·manifest다. 두 ROS 노드의 `max_steering_angle_rad`만 변경하며 B3/속도/기하/frame/시간/토픽 등 나머지 값은 보존한다. YAML 서식은 재직렬화되므로 보존 기준은 읽은 값/형식이다. 원 YAML bytes는 별도 사본으로 남긴다.

기존 YAML의 어느 한 노드에 파생 상한보다 **더 작은 운용 상한**이 있으면 `would widen existing steering limit`로 생성 자체를 거절하고 원 파일을 보존한다. 이를 자동 확대하거나 두 기존값의 min을 새로운 정책으로 조용히 채택하지 않는다. 이번 목적의 r401 CAD 상한→장착 축소는 통과하며, 더 작은 별도 운용 상한은 운영자가 의도적으로 호환되는 기본 프로필을 선택해야 한다.

MCU의 설치 원상한은 조용히 다시 쓰지 않는다. 예를 들어 board0.2 rad는 그대로 두고 planner·guard·host에 **같은0.19999998807907104 rad**를 전달한다. 이는 float32로 정확히 표현되며 MCU 상한을 넘지 않는다. 따라서 네 숫자를 억지로 같게 만드는 대신 **단일 원천 + 세 송신 소비자 동일 + 송신상한≤MCU**를 계약으로 삼았다. 기존 CAD 상한0.4578436017036438은 이미 float32 값이므로 네 값이 그대로 일치한다.

`verify`는 생성 당시 설치 manifest/board/source 해시와 현재 선택 bundle을 대조하고, 원 YAML로부터 두 상한만 파생됐는지 확인한다. 선택할 실제 ROS YAML 및 host 상한도 추가 인자로 비교할 수 있다. `host-arguments`는 같은 검증 뒤 기존 host용 `--max-steering-rad` 인자를 JSON 배열로 반환한다. 어떤 명령도 노드·host·장치·ARM을 시작하지 않는다.

## 확인

Windows Python3.14.6에서 표준 `unittest` **최종11개 통과**, 실패/skip0이다. 기존 더 작은 상한의 자동 확대 거절을 포함한다. 초기10검사와 대조 자료는 `verify_v1/`에 보존하고 보완 후 확인은 `verify_v2/`에 남겼다. PyYAML6.0.3은 검사용 `verify_v1/deps/`에만 캐시에서 설치했고 전역 환경을 바꾸지 않았다.

실제 r401 YAML의 planner122/guard40개 parameter를 별도로 대조했다.

| 설치 원상한 | planner·guard·host 파생 상한 | 기존 r401과 다른 parameter | 나머지 값 |
|---|---:|---|---|
| 0.2 | 0.19999998807907104 | 두 `max_steering_angle_rad`만 | 전체 exact |
| 0.4578436017036438 | 0.4578436017036438 | 없음 | 전체 exact |

두 경우 모두 원 firmware bundle bytes/기존 verify가 보존됐다. 기존 export·조향 변환·차량 기하·빌더2개·r401 YAML의6개 입력 SHA도 전후 동일하다. 근거는 [최종 unit 로그](verify_v2/unittest.log)와 [r401 전체값 대조](verify_v2/result.json)다. 실제 ROS 생성자·WSL·C++/IDF 빌드·flash·실물 출력은 실행하지 않았다. 기존 r401의 실노드 검증을 새 축소 상한의 실노드 검증으로 확대하지 않는다.

## 이관 후 사용할 최소 흐름

아래는 **후속 운영 안내 초안**이며 이번에 실물 파일로 실행한 명령이 아니다. `INSTALLED_PROFILE_DIR`은 사람이 확인한 기존 설치 bundle, `BASE_RUNTIME_YAML`은 사용할 운영 YAML, `SELECTED_RUNTIME`은 새 출력 디렉터리다. 기존 firmware 빌드는 같은 `INSTALLED_PROFILE_DIR`을 명시 선택한다.

```bash
python3 scripts/runtime_installed_limits.py prepare \
  --installed-profile "$INSTALLED_PROFILE_DIR" \
  --runtime-params "$BASE_RUNTIME_YAML" --output "$SELECTED_RUNTIME"

# 검증된 JSON 배열을 두 개의 Bash 인자로 변환한다. eval은 사용하지 않는다.
set -euo pipefail
HOST_STEERING_JSON=$(python3 scripts/runtime_installed_limits.py host-arguments \
  --selection "$SELECTED_RUNTIME" --installed-profile "$INSTALLED_PROFILE_DIR")
mapfile -t INSTALLED_HOST_ARGUMENTS < <(python3 -c \
  'import json,sys; print(*json.loads(sys.argv[1]), sep="\n")' "$HOST_STEERING_JSON")
test "${#INSTALLED_HOST_ARGUMENTS[@]}" -eq 2
test "${INSTALLED_HOST_ARGUMENTS[0]}" = --max-steering-rad
python3 scripts/runtime_installed_limits.py verify \
  --selection "$SELECTED_RUNTIME" --installed-profile "$INSTALLED_PROFILE_DIR" \
  --runtime-params "$SELECTED_RUNTIME/runtime_profile.yaml" \
  --host-steering-rad "${INSTALLED_HOST_ARGUMENTS[1]}"
```

같은 ROS 환경·domain의 각 터미널에서 r415의 두 노드는 **둘 다 새 params-file**을 사용한다. 아래 기존 B3/개발 속도/토픽 인자는 그대로 두었다.

```bash
ros2 run arena_autonomy lidar_safety --ros-args \
  --params-file "$SELECTED_RUNTIME/runtime_profile.yaml" \
  -p race_control_mode:=b3_candidate \
  -r /scan/native:=/hardware_runtime/c1/native
```

```bash
ros2 run arena_autonomy g1_g2_handoff_trial --ros-args \
  --params-file "$SELECTED_RUNTIME/runtime_profile.yaml" \
  -p race_control_mode:=b3_candidate \
  -p development_speed_limit_enabled:=true -p development_speed_limit_mps:=0.3 \
  -p g1_current_corner_budget:=off \
  -p g1_trace_path:="${RUN_OUTPUT:?새 결과 디렉터리}/follow_decisions.jsonl" \
  -r /scan/native:=/hardware_runtime/c1/native \
  -r /camera/color/image_raw:=/hardware_runtime/camera/image_raw
```

host를 시작할 터미널에서 위 배열 생성/verify를 수행한 뒤, 기존 하드코딩 `--max-steering-rad`를 **제거하고 배열을 그대로 전달**한다. 값은 사람이 다시 계산하거나0.2로 반올림하지 않는다.

```bash
python3 scripts/run_runtime_actuation_host.py \
  --port "${RUNTIME_PORT:?runtime ESP by-id 경로}" \
  --output "${RUN_OUTPUT:?새 결과 디렉터리}/host" --seconds 60 \
  --clock-policy "${CLOCK_POLICY:?명시 clock policy JSON}" \
  --motor-feedback-config "${MOTOR_FEEDBACK_CONFIG:?확인한 motor feedback JSON}" \
  --imu-topic /hardware_runtime/imu --max-speed-mps 1.5 \
  "${INSTALLED_HOST_ARGUMENTS[@]}"
```

기존 출력 허가 절차는 바꾸지 않는다. 위 예제 host는 기존과 같이 `--enable-drive-wire`가 없어 ARM/DRIVE wire가 차단된다. 이 도구가 기존 명령을 몰래 교체하거나 실행 중인 host의 상한을 바꾸지는 않는다. **선택 산출물 생성은 실제 launch 또는 MCU 적용 완료가 아니다.** 같은 설치 bundle의 firmware 선택과 생성한 파일/인자의 운영 진입 선택은 별도 단계이며 주 담당이 공통 이관·운영 문서와 연결한다. Bash/실노드 실행은 이번 Windows 검증 범위에 포함하지 않았다.

별도 **수동 패널 구성**을 선택할 때는 `race_ops_node`의 독립 인자도 `manual_steering_limit_rad:=min(0.2, 생성 L)`로 맞춘다. 여기서 생성 L은 `INSTALLED_HOST_ARGUMENTS[1]`이다. 예를 들어 아래 계산값을 기존 수동 패널 명령의 ROS 인자 `-p manual_steering_limit_rad:="$MANUAL_STEERING_LIMIT"`에 전달한다. 생성기가 이 세 번째 노드의 설정까지 자동 변경하지는 않는다.

```bash
MANUAL_STEERING_LIMIT=$(python3 -c \
  'import sys; print(min(0.2, float(sys.argv[1])))' "${INSTALLED_HOST_ARGUMENTS[1]}")
```

기존 수동 기본값은0.2 rad이며(`race_ops_node.py:161,196–197`), guard는 자기 상한으로 송신각을 제한한다. 이 추가 인자는 UI 목표각도 실제 선택 상한에 맞추기 위한 운영 설정이다. **자율 패널 구성에는 수동 명령 발행이 없으므로** 자율 구성의 두 ROS 노드+host 생성/검증 범위는 그대로다. 수동 실행은 자율 제어 노드를 종료한 별도 구성에서 기존 절차로 선택하며, 이번 검증에서는 패널이나 출력을 실행하지 않았다.

적용 대상은 신규2파일뿐이며 [변경 목록·SHA](changes.json)와 [적용 diff](apply.patch)에 정리했다. 원 r430 export/빌더는 그대로 재사용하고 실제 장착 count·영점·기하·보호층은 새로 만들지 않았다.
