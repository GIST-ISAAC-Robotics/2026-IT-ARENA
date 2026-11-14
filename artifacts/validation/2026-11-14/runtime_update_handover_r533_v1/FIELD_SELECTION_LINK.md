# 현장 재개 시 조향 선택 연결 — stage 보완

**최신 stage 선택은 [metadata v2 JSON](stage/steering_selection.assembled_metadata_v2.json)이다.** 공통 프로필의 `endpoints_confirmed` 설명만 정정되어 SHA가 `4cb691932495ba86cd1b92d2bf289397fbaa91a5a6aa2f64b1ec7115f74ea160`으로 바뀌었다. 수치·측정 근거·vehicle SHA는 동일하다. 기존 e3077 선택 JSON/ZIP은 보존하며 새 helper는 stale SHA를 정상 거절한다. v3는 Y갈림길 반례로 미채택이다. 현재 stage 보완은 v3에 소급 반영하지 않으며 r553 결과/root GO 전 v4를 생성하지 않는다.

2026-11-14, GPT. **실행기는 선택할 수 있지만 START/재개 버튼은 새 파일을 읽지 않는다.** r533 선택 JSON을 실제 실행 인자로 옮기는 연결을 [작은 helper](stage/steering_launch_selection.py)에 추가했다. 장치·프로세스를 실행하지 않고 JSON 인자 조각만 출력한다. 공통 scripts·패널·runbook과 고정 `selected_v2`는 수정하지 않았다.

| 현재 호출 경로 | 확인한 선택 방식 / 누락 |
|---|---|
| 센서 preview → `probe_g1_hardware_shadow.py` | `--vehicle-config`가 `preview_parameters()`를 거쳐 중앙각/기하로 전달된다. 옵션을 생략하면 새 measured YAML이 선택되지 않는다. 비구동 preview다. |
| planner·guard | 실제 생성한 동일 `runtime_profile.yaml`을 `--params-file`로 각각 받아야 한다. 패널 실행·START로 변경되지 않는다. |
| UART host | 프로필/vehicle JSON을 직접 읽지 않고 필수 `--max-steering-rad`를 받는다. runbook이 연결한 옛 r415 예시는0.4578436rad이므로 이번 선택의 재개 명령에 복사하지 않는다. |
| `race_ops` 수동 단발 | `manual_steering_limit_rad` 별도 매개변수다. 기존 `min(0.2, 선택 실행 상한)`을 유지하며 자동 확대하지 않는다. |
| `race_ops` START/재개 | 현재 실행 중인 구성으로 준비한다. JSON/firmware 자동 갱신 기능이 아니다. 새 선택은 노드/host 시작 명령에 연결한 뒤 기존 START 흐름을 사용한다. |
| `steering_panel_remote.py` | 장착 수동 3점 측정용 경로다. 경기 운영 프로필 선택기나 MCU runtime 설치기가 아니며 다른 세션의 원 측정을 수정하지 않았다. |

선택은 **영점2811·운영2600–2980counts, 대칭 중앙 상한0.28475436568260193rad(16.315°)**다. 대칭 경로의 오른쪽43counts(약4.243°)는 미사용이며 바퀴각은 CAD 환산이다. `vehicle.max_steering_angle_rad=0.30666234131856146`는 내륜 joint 상한으로 중앙 상한과 구분한다.

## 선택 확인과 실제 재개 연결

현재 PC에서는 아래 읽기 전용 명령으로 선택값과 preview/host/panel 인자 조각을 얻는다. 실제 board가 없으면 `installed_profile_link_verified=false`, 실제 YAML도 없으면 planner/guard 인자는 null이다. 이 출력은 완전한 실행 명령이나 설치 완료 증명이 아니다. 옛 [v1 출력](steering_launch_arguments_v1.json)은 당시 SHA 기록으로만 보존한다.

```powershell
python -B artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/stage/steering_launch_selection.py `
  --steering-selection artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/stage/steering_selection.assembled_metadata_v2.json
```

실제 board를 확보한 뒤 기존 설치 생성·양쪽 ROS 상한 생성은 [선택 안내](STEERING_SELECTION_READY.md)의 명령을 사용한다. 새 helper에는 그 결과 두 경로를 함께 넣는다. 원래 더 작은 base 조향 상한은 기존 생성기가 자동 확대하지 않고 거절하므로 호환되는 base를 명시 선택해야 한다.

```sh
python handover/steering_launch_selection.py \
  --steering-selection handover/steering_selection.json \
  --installed-profile "$FRESH_INSTALLED" \
  --runtime-selection "$FRESH_SELECTION/selected"
```

출력 `planner`와 **`guard_with_status`**를 각각 기존 노드 시작 명령에, `host` 상한을 host 명령에, `race_ops` 인자를 기존 운영 패널 명령에 전달한다. 호환용 `guard`는 params-file만 담고, `guard_with_status`에는 선택 status 인자까지 들어간다. 실제 YAML이 이미 있으면 `--runtime-params "$FINAL_RUNTIME_YAML"`을 추가하여 양쪽 상한이 L과 같은지 검사한다. 옛0.4578436rad이 어느 한쪽에 남거나 옛 r547 replay profile을 복사하면 거절한다. 기존 설치 생성기는 호환 base의 두 상한을 L로 파생한다. 수동 panel은0.2rad을 유지한다.

guard status는 **기본 full**이다. r547 compact 옵션을 선택할 때만 다음 두 인자를 helper에 추가한다. `FRESH_RUN_ABSOLUTE_PATH`는 Linux의 새 실행결과 절대 경로이며 원격 폴더를 여기서 만들거나 조회하지 않는다.

```sh
--guard-detail compact_v1 --guard-result-path "$FRESH_RUN_ABSOLUTE_PATH"
```

이는 r547와 같은 `safety_status_detail=compact_v1`, `safety_status_evidence_path=<new_run>/guard_status_contexts.jsonl` 두 값만 선택한다. 기존 r547 JSON 예시 전체는 replay용이므로 실제 운용 YAML을 덮어쓰지 않는다. r547 회수 절차대로 원 status와 sidecar를 함께 보존한다. planner `g1_trace_detail=compact_v1`은 독립 선택으로 그대로이며, 운영 panel은 별도 mode 전환 없이 기존 `/safety/status`를 구독한다. r551 표시 수정은 소스 포함 여부와 별도로 실제 설치를 확인해야 한다.

기존 trace 경로·포트·clock·motor feedback·STOP/ARM 절차를 대체하지 않는다. 재시작 후 같은 경기 복구/START는 현행 운영 절차를 따른다. MCU에는 같은 installed bundle을 사용한 실제 빌드/설치 확인이 별도로 필요하며 이 도구는 firmware를 조회하거나 설치하지 않는다.

## 검증과 보존

- [최신 표적3검사](stage/latest_launch_tests.log) 통과: 설명만의 새 SHA 선택·원본 보존, 실제 두 노드의 옛 상한 거절, full 기본/compact 절대 sidecar 선택, 합성 board에서 최신 프로필→기존 설치 생성기→양쪽 ROS/host L 연결. 실제 motor/PI 값은 만들지 않았다.
- 설명 SHA 갱신은 [metadata-only helper](stage/refresh_steering_selection_metadata.py)가 보존한 이전 프로필 SHA와 새 프로필을 비교한다. 설명 이외의 운영 값/측정 provenance가 바뀌면 자동 갱신하지 않는다. 현재 변경 키는 `selection_basis.flag_meanings.endpoints_confirmed` 하나다.
- [초기 검사](steering_launch_tests.log) 3통과,1실패 후 [해당1개 재검사](steering_launch_retry.log) 통과: 고유4검사 완료. 실패는 기존 생성기가 작은 base를 확대하지 않고 거절하는 계약을 시험이 잘못 예상한 것이며, 기대값만 수정했다.
- 실제 선택 SHA/ready/CAD 변환, preview 인자 전달, 수동 패널 상한 수용, 합성 board로 기존 설치 bundle/양쪽 ROS 경로 연결과 불일치 거절을 확인했다. 합성 값은 임시 디렉터리에서만 썼다.
- [selected_v2 보존 검사](selected_v2_preservation.log): 기존 ZIP SHA `92aa7039b188cf16ee31c3ecabe941793837ca9a157b9de9533198807feff9aa`,734파일 그대로 통과. 검증기는 고정 ZIP의 안내를 최신 안내 내용과 비교해 소급 무효화하지 않고 당시 manifest SHA로 검증한다.
- 최신 helper와 재개 안내는 stage collector의 **다음 선택 묶음**에만 추가된다. 고정 v2/v3 ZIP/manifest는 변경하지 않았고 v4는 생성하지 않았다. 실제 board·motor/PI 입력·Jetson·ESP32·WSL·flash·출력은0이다.
