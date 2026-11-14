# selected_v4 사전 목록 — 미선정·미동결·ZIP 없음

2026-11-14, GPT. [source_selection.json](source_selection.json)에 기존 collector가 한 번 읽은 **737개 소스 경로와 SHA**를 기록했다. 이 목록은 현재 관측한 후보이며 root 최종 GO 뒤 다시 수집해 동결한다. `selected_v4/`와 ZIP은 만들지 않았다. 실제 주행·성능 결과를 새로 얻거나 기존 시험을 반복하지 않았다.

필수 운영 선택10파일의 현재 SHA는 관련 공통 통합 기록과 일치했다.

| 선택 | 경로 |
|---|---|
| r557 공통3 | `current_wall_reacquire.py`, `current_path_recheck/_geometry.py`, `race_runtime_policy/retained_recheck/_geometry.py` |
| r554 | `bypass_entry_trial.py` |
| r543 | `lidar_safety.py`, `guard_status_payload.py` |
| r551 | `race_ops_node.py`, `race_ops_page.py` |
| r541/r542 | `config/hardware/steering_zero.assembled_2026_11_14.json`, `src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml` |

Python 모듈 경로의 공통 접두사는 `src/arena_autonomy/arena_autonomy/`이다. 전체 경로·전체 SHA와 증분은 manifest 및 [v3 대비13경로](observed_delta_from_v3.json)에 있다. `local_follow.py`, `local_path.py`도 v3 이후 바뀐 현재 공통으로 수집됐으므로 최종 GO 대상에 표시했다. 이 준비 작업이 별도 제어 수정을 채택한 것은 아니다.

조향 profile SHA **4cb69193…**, 선택 descriptor SHA **b225f76e…**, vehicle SHA **dcb8a713…**를 유지했다. 영점2811·count2600–2980·CAD 대칭 중앙 상한0.28475436568260193rad이며, 오른쪽43counts 미사용과 실제 wheel각 미측정은 같다. 현재 descriptor는 `stage/steering_selection.assembled_metadata_v2.json`이다.

## 기존 복제/생성 진입점

‘clonebuilder’는 별도 프로그램이 아니라 기존 **r491 collector → r533 prepare_update_with_steering** 흐름을 뜻한다. 도구를 새로 만들지 않았다. root가 최종 소스·실제 새 label·출력 폴더를 승인한 다음에만 아래 기존 명령을 실행한다.

```powershell
python -B artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/stage/prepare_update_with_steering.py prepare `
  --final-source-selection $rootFinalSourceSelection `
  --steering-selection artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/stage/steering_selection.assembled_metadata_v2.json `
  --output $rootApprovedFreshOutput
```

지금은 **명령 미실행**이다. 최종 GO 때 관측 목록과 다른 SHA가 있으면 변경 경로/당시 bytes를 root에게 보고하며 공통 원본을 임의로 복구하지 않는다. 생성 뒤에만 ZIP/manifest/CRC를 검증한다. 기존 v2/v3 ZIP·manifest는 SHA 보존을 확인했고 v3 미채택 상태를 유지했다.

## 운영 연결·다른 세션 경계

실제 재개 연결은 [FIELD_SELECTION_LINK.md](../../FIELD_SELECTION_LINK.md)를 따른다. 최신 launch helper와 선택 안내를 묶음에 포함할 준비가 됐다. guard status 기본은 full, compact는 명시한 새 절대 sidecar 경로와 함께 선택한다. planner의 compact trace는 별도다. 두 노드의 실제 YAML·host 상한을 연결해야 하며 START가 profile/firmware를 바꾸지 않는다. 실제 설치·입력 지연·주행 성능을 이 소스 목록으로 입증하지 않는다.

사용자 전용 MOTOR/ESP32/Jetson 작업 및 probe/firmware 원본은 건드리지 않았다. collector에 잡힌 보조 소스128개는 `auxiliary_source_snapshots_not_verified_installed_apps`에 분리했다. 새 `scripts/render_jetson_direct_wiring.py`도 원형 snapshot이며 운영 선택·검증된 실장 앱으로 채택하지 않는다. 수집에 없는 `.c` 기반 probe firmware 재빌드는 기존 별도 경계다.

실제 board·PI·포트·설치 입력은 null이며 이를 채우거나 앱 설치 상태를 조회하지 않았다. Windows 파일 읽기/목록 작성만 수행했고 WSL·장치·Jetson·flash·실행0이다.
