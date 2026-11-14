# r533 후속 — 명시 조향 선택을 포함하는 stage 사본

**현재 선택 상태:** `selected_v3`는 추가 Y갈림길 반례로 미채택이며 현재 실행을 권장하지 않는다. [별도 상태 기록](SELECTED_V3_STATUS.json)을 따른다. ZIP은 보존하고 r553 주행 결과 뒤 root GO 전 v4를 생성하지 않는다. 제어 후보의 최신 revision을 이 준비 도구가 임의 채택하지 않는다.

2026-11-14, GPT. **후속 root 최종 선택으로 `selected_v3` source-only 묶음을 생성했다.** [최신 결과](SELECTED_V3_RESULT.md)를 따른다. 아래는 준비 시점의 연결 설명이며, 기존 `stage/prepare_update.py`, `selected_v1`/`selected_v2`와 r500 rollback 기준은 보존했다. 기존 r491 collector와 설치 생성기를 그대로 호출하며 별도 설치 체계를 추가하지 않았다.

**현장 연결 보완:** [선택 JSON → 실행/재개 인자 helper](FIELD_SELECTION_LINK.md)의 초기판은 `selected_v3`에 포함했다. 현재 stage판은 최신 profile 설명 SHA·r547 선택형 compact·실제 양쪽 ROS 상한 확인을 연결하며 다음 묶음용이다. 고정 v2/v3는 원본 그대로 보존한다.

[최신 stage 선택 JSON](stage/steering_selection.assembled_metadata_v2.json)이 root가 공통 배치한 다음 입력을 묶는다. [이전 선택 JSON](steering_selection.assembled_2026_11_14.json)은 보존본이며 현재 프로필에는 SHA 불일치로 거절된다. 변경은 `selection_basis.flag_meanings.endpoints_confirmed` 설명뿐이고 운영 수치는 같다.

| 입력 | 값 |
|---|---|
| 실제 영점/운영 count 프로필 | `config/hardware/steering_zero.assembled_2026_11_14.json`; SHA `4cb691932495ba86cd1b92d2bf289397fbaa91a5a6aa2f64b1ec7115f74ea160` |
| 선택 vehicle YAML | `src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml`; SHA `dcb8a7130f3eea4a474aff4602b86547bb7f700e6fecaff3049bb8dbd32a5021` |
| host/MCU 중앙 명령 상한 | `0.28475436568260193 rad` |
| vehicle 내륜 joint 상한 | `0.30666234131856146 rad` |

[stage 사본](stage/prepare_update_with_steering.py)에 선택형 `--steering-selection`을 추가했다. 이를 쓰면 두 원파일과 `handover/steering_selection.json`, `handover/STEERING_SELECTION.md`가 ZIP manifest에 포함된다. SHA·ready 프로필·CAD 환산 뒤 중앙 상한·안내 동봉을 확인한다. 실제 board도 별도 입력하면 선택 영점·L·기하와 일치해야 하며 board 값은 이 도구가 만들거나 수정하지 않는다. source-only 선택에서도 기존 `required_inputs.json`의 포트/시각/장착 미결값은 null이다. 바퀴각 실측 주장은 없다.

지금 실행 가능한 선택 검사에는 전체 소스 수집·ZIP 생성·전체 저장소 불변 검사가 없다.

```powershell
python -B artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/stage/prepare_update_with_steering.py selection-check `
  --steering-selection artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/stage/steering_selection.assembled_metadata_v2.json
```

**아래는 다음 선택 생성용 명령이며 현재 실행 금지/미실행이다.** root가 r553 결과와 최종 제어 SHA를 선택한 뒤 GO한 경우에만 새 폴더에 생성한다. v2/v3를 덮어쓰지 않는다.

```powershell
python -B artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/stage/prepare_update_with_steering.py prepare `
  --final-source-selection $rootFinalSourceSelection `
  --steering-selection artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/stage/steering_selection.assembled_metadata_v2.json `
  --output $rootApprovedFreshOutput
```

`--base-params`, `--hardware-inputs`, `--board-profile`, `--steering-profile`의 기존 네 입력은 실제 준비되었을 때만 함께 전달한다. 선택 설명파일만으로 실제 board/PI/encoder/포트를 완성하지 않는다. 동봉 안내는 명시 영점으로 기존 `runtime_installed_profile.py prepare` → 정책/양쪽 ROS 상한 생성 → 동일 host arguments → 동일 native firmware installed profile 순서를 적는다. 선택 vehicle YAML은 preview의 `--vehicle-config`로 전달한다. ZIP 교체만으로 MCU/host 적용이 끝나지 않으며 기존 r500 설치본을 rollback 기준으로 유지한다.

검증: [8검사](steering_selection_tests.log) 통과. 실제 두 파일의 선택·SHA·ready/CAD 계약, 선택 옵션 유무, 묶음 안의 선택 누락/변경 검출, 실제 board 입력과 선택 불일치 거절을 확인했다. source 수집은 합성 fixture로 대체했으므로 다른 담당의 공통/펌웨어 작업에 전체 불변 조건을 걸지 않았다. 기존 collector의 최종 생성 시 무결성 확인은 그대로다. Windows 파일 작업만 했으며 Jetson/ESP32/WSL/flash/출력은0이다.

검사용 `steering_test_tmp_v1/`에는 합성 source ZIP과 motor board fixture가 있으며 운영 입력이 아니다. 자동 승인 검토가 이 폴더의 재귀 삭제를 `blocked by policy`로 거절하여 원형 보존하고 `.gitignore`로 공개 대상에서 제외했다. 이를 최종 선택 또는 설치 자료로 사용하지 않는다.
