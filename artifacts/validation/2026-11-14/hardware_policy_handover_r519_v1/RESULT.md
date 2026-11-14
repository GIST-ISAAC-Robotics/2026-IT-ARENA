# r519 실물 정책 연결 후보 — PC 계약 READY, 설치·활성화 미실행

2026-11-14, GPT. 기존 r444/r487의 명시 estimated 입력 계약과 `runtime_installed_profile.py` → `runtime_installed_limits.py` 경로를 연결하는 [stage helper](stage/prepare_hardware_policy.py)를 준비했다. **PC 검사 7개 통과. r500의 실행 YAML은 여전히 미선택이며 새 ZIP·Jetson 설치·ROS 노드·포트·펌웨어·출력 실행은 없다.**

## 지금 준비된 것

- [정책 overlay](draft_v1/policy_overlay.yaml): `current_route_priority` 선택 조건인 `g1_traffic_recovery_trial=true`, `g1_guard_observation_policy=selected_r181`을 planner/독립 guard 양쪽에 명시한다. 두 노드 모두 `bounded_local_v4`, crop 선택, `host_estimated_crop_v1`, B3를 일치시킨다.
- `use_sim_time=false`, `motion_scan_timing_verified=false`, `motion_scan_time_policy=host_estimated_v1`, `native_estimated_v1`, `lidar_compensation=both`를 유지한다. 측정되지 않은 시각을 verified로 승격하지 않는다.
- planner는 `cad_runtime_r377_v1`, process/fresh-native/worker-current/native 계산, retained reference, `r396_v1`을 명시해 기존 native 두 계산 runtime을 선택한다. `g1_g2_simulation_trial`은 기존 capability 키 이름이며 이 후보가 실물 검증 완료라는 뜻이 아니다.
- [필수 입력](draft_v1/required_inputs.json)의 assembly, 장착 frame·외부변환·IMU bias, 실제 port, clock-policy/motor-feedback 경로는 **모두 null**이다. [상태](draft_v1/status.json)의 servo neutral·MCU 상한·installed profile도 null이다. r401의 `c1_unmounted_preview`·속도·실측값을 채우지 않았다.
- `prepare`는 사용자가 명시한 base YAML과 입력 JSON, 기존 검증 가능한 실장 bundle만 받는다. 기존 생성기가 조향 상한을 양쪽 ROS YAML/host 인자로 전달한다. 원 base/입력 bytes와 SHA를 보존하며 공통 파일은 고치지 않는다.

`draft_v1/policy_overlay.yaml`은 **단독 실행 YAML이 아니다**. helper는 정책·선택 연결만 확인하고, ROS 전체 생성자·ARM native 실행·실측 정확도·속도 상한 전달·firmware 설치를 인증하지 않는다. 기존 선택 조건에 새 정지 조건을 추가하지 않았다.

## 최종 소스 고정 후 파일 생성 레시피

새 source bundle/설치는 [r517 READY](../runtime_update_recipe_r517_v1/READY.md)의 기존 r500 방식을 따른다. 이 stage helper는 자동 source bundle 수집 대상이 아니므로 PC에서 먼저 선택 YAML을 만들거나, 별도 파일로 명시 전달해야 한다. `ROOT`는 이 저장소 안의 현재 stage 배치를 기준으로 찾는다.

1. 실제 확인한 board/servo 영점·방향·끝점으로 기존 `scripts/runtime_installed_profile.py prepare`를 수행하고 `verify`한다. 미측정 값을 합성하여 이 단계를 통과시키지 않는다. 생성은 flash/ARM과 별개다.
2. `draft_v1/required_inputs.json`의 별도 사본에 실제 측정값/선택 경로를 입력한다. time-policy JSON과 motor-feedback JSON의 파일 유효성·실제 적용은 기존 host가 확인한다. 이 helper는 장치를 열거나 그 파일을 실측 인증하지 않는다. 기존 등록 LiDAR 원천 frame은 `laser_frame` / `c1_unmounted_preview` 두 개다. 장착 후에도 실제 발행값과 맞는 등록 이름을 명시하고, 이름만 보고 미장착/교정 완료를 판정하지 않는다. 임의 새 frame명은 기존 `MotionConfig` 계약에서 거절된다.
3. root가 선택한 **실제 운용 base YAML**을 명시한다. 양쪽 기존 `max_steering_angle_rad`가 필요하다. 차량 기하·속도·추월 profile·절대 `g1_trace_path`·토픽 연결을 검토한다. r401 참고 템플릿을 자동 선택하지 않는다.
4. 아래 파일 작업만 수행한다. `$reviewedBase`, `$hardwareInputs`, `$installedBundle`, `$newSelection`은 실제로 준비한 경로를 명시한 뒤 사용한다. `$newSelection`은 아직 없는 새 디렉터리다.

```powershell
$env:PYTHONPATH = (Resolve-Path 'build/jetson_tools/invoke/vendor').Path
$env:PYTHONDONTWRITEBYTECODE = '1'
python -B artifacts/validation/2026-11-14/hardware_policy_handover_r519_v1/stage/prepare_hardware_policy.py prepare `
  --base-params $reviewedBase --hardware-inputs $hardwareInputs `
  --installed-profile $installedBundle --output $newSelection
python -B scripts/runtime_installed_limits.py verify `
  --selection "$newSelection/selected" --installed-profile $installedBundle
python -B scripts/runtime_installed_limits.py host-arguments `
  --selection "$newSelection/selected" --installed-profile $installedBundle
```

생성 결과는 `selected/runtime_profile.yaml`, `selected/runtime_host_limits.json`, 기존 manifest, 원본 사본 및 `handover.json`이다. 마지막 파일에는 supplied MCU 상한·추월 선택 상태와 `installed=false`, `hardware_ready=false`, `runtime_launch_validated=false`, `speed_limits_propagated=false`가 남는다. MCU 상한은 실장 bundle에서 읽을 뿐 임의 생성/변경하지 않는다.

## 실제 선택 시 연결할 진입점

최종 활성화 담당은 같은 선택 YAML을 **`g1_g2_handoff_trial`과 `lidar_safety` 양쪽에** 전달한다. 일반 `local_pursuit` 실행만으로 native 두 계산 경로가 선택되는 것은 아니다. `g1_g2_handoff_trial`의 ROS 노드 이름은 `local_pursuit`이므로 YAML section은 현재 이름을 유지한다. 선택을 설명하는 명령 형태는 다음과 같으며, 이번 작업에서는 실행하지 않았다.

```bash
ros2 run arena_autonomy g1_g2_handoff_trial --ros-args --params-file "$SELECTED/runtime_profile.yaml" -r /scan/native:=/hardware_runtime/c1/native
ros2 run arena_autonomy lidar_safety --ros-args --params-file "$SELECTED/runtime_profile.yaml" -r /scan/native:=/hardware_runtime/c1/native
```

실제 IMU/wheel/native 입력 이름은 최종 base/현재 센서 발행과 대조한다. host는 기존 `scripts/run_runtime_actuation_host.py`의 `--port`, `--clock-policy`, `--motor-feedback-config`, `--max-speed-mps`와 생성된 `host-arguments`를 사용한다. host가 `/drive/safe`를 구독하는 기존 경계 및 수동/자율 운용 선택은 유지한다. r491의 조립 후 저속 수동 점검 절차와 자율 planner를 동시에 시작하지 않는다. 이 문서는 ARM/drive-wire 허가를 추가하지 않는다.

**r494/r514 MovingPass는 별도 선택이다.** `g1_g2_moving_pass_trial`과 `g1_g2_moving_pass_profile`은 root가 검토한 base 값 그대로 보존하고 `handover.json`에 기록한다. helper가 임의로 `true`나 특정 profile을 넣지 않는다. r474/r489 current-route/guard 키가 맞거나 ZIP에 최신 `moving_pass.py`가 들어 있어도, 이 선택과 실제 source SHA가 확인되지 않으면 최신 추월 정책이 실물에서 채택됐다고 말할 수 없다.

## 확인한 범위와 남은 경계

- [stage smoke](stage_smoke.log): 4개 테스트. null draft, 합성 bundle과 기존 limits 생성/검증의 연결, 원본/속도/추월 선택 보존, 미입력·assembly 불일치 거절, guard/time/native 선택 불일치를 확인했다. 합성 수치는 `TemporaryDirectory`의 테스트 fixture에만 존재하며 운용 후보에 남기지 않았다.
- [기존 계약 smoke](existing_contract_smoke.log): 기존 `test_estimated_observation_scope.py` 중 생성자 선택·corner scope·실제 header metadata 관련 3개만 실행했다. 새 패키지 설치/ARM 계산/시뮬 재실행은 없다.
- [r487 결과](../runtime_estimated_handover_r487_v1/RESULT.md)는 저장 시뮬 수치를 기존 변환으로 옮긴 plan/current 8호출의 입력 계약 근거다. r481 현재 경로 수락과 r482 measured-region 재사용은 당시 미도달이었다. r489/r494/r514의 모든 분기나 실물 연속 주행 근거로 확대하지 않는다.
- r500 launcher의 SHM 환경 적용은 미확인이다. r508의 16 MiB 비교가 채택되기 전 transport 기본값을 넣지 않았다. r517에 기록한 servo probe `.c` 누락은 별도 펌웨어 재빌드 경계이며 이 host 정책 생성기와 무관하다.

**판정: r519 파일 생성/정책 연결 준비 READY. 실제 base·실장 bundle·측정 입력 선택과 root 최종 소스 고정 후 생성 가능. 현재 실물 활성화·설치·실측 교정 완료는 아니다.**
