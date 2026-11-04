# 게시 후보 목록(11/4 장착 비교 ~ JR60P20 채택)

스테이징은 최종 검토 뒤 별도로 수행한다. 이 목록은 작업 트리에서 확인한 후보이며, 커밋이나 승인 기록이 아니다. 집계 근거는 [publication_scan.json](publication_scan.json)([스캔 코드](publication_scan.py))에 있다.

## 포함

| 묶음 | 경로 |
|---|---|
| 런타임(이전 v2~v4 변경 포함) | `src/arena_autonomy/arena_autonomy/{lidar_safety,local_follow,local_pursuit,partial_observation}.py`(수정), `src/arena_autonomy/arena_autonomy/known_structure.py`(신규), `src/arena_bringup/launch/{simulation,demo,g1_follow,local_pursuit}.launch.py`(수정) |
| 검사기·진입점 | `scripts/validate_basic_autonomy.py`, `scripts/validate_g1_traffic.py`(수정), `scripts/run_current_solo.py`(신규) |
| 센서 프리셋 | `src/arena_description/config/b0183_c1_jr60p20_sim.yaml`(채택), `src/arena_description/config/b0183_c1_jr50_sim.yaml`(이전, 재현용) |
| 출처 계약 | `config/tests/lidar_mount_{adapted_r2,pilot_r3,speed_r4,launch_record_r5,traffic_marker_r6}_sha256.json` |
| 시험 | `tests/test_source_provenance_r0_plus_b3.py`(수정), `tests/{test_known_structure_policy,test_structure_route_speed,test_jr60_adoption_profile}.py`(신규) |
| 문서 | `README.md`, `docs/PROJECT_CONTEXT.md`, `docs/README.md`, `docs/HARDWARE_AND_RULE_QUESTIONS.md`, `docs/activity/2026-11-04.md`, `docs/decisions/0029-temporary-jr60p20-vehicle.md` |
| 무시 규칙 | `.gitignore`: 이번 비교 주행 4개 폴더의 `*.mp4`만 추가 |
| 증거 | `artifacts/validation/2026-11-04/` 아래 `lidar_mount_adapted_v2`, `lidar_mount_pilot_v3`, `lidar_mount_speed_v4`, `lidar_packaging_v3`, `lidar_jr_forward_v1`, `lidar_jr60_adoption_v1`. `.gitignore`가 적용된 상태로 폴더 단위로 추가한다 |

출처 사슬과 시험에 필요한 증거는 모두 추가 가능한 상태임을 확인했다. 해당 파일은 `lidar_mount_adapted_v2/runtime_before.zip`, `lidar_mount_speed_v4/{runtime_before,pilot_before,launch_forwarding_before}.zip`, `lidar_jr60_adoption_v1/traffic_marker_before.zip`, `lidar_jr_forward_v1/{profiles/jr60p20.yaml,jr60p20_1_registered.json}`이다.

## 제외

- 저장소 루트의 `0`, `POWERSHELL_LASTEXITCODE=0`, `POWERSHELL_LASTEXITCODE=1`은 이번 작업과 무관한 파일이다. 추가하지도 삭제하지도 않는다.
- 기존 규칙으로 무시되는 파일: `*.jsonl`(단 `signal_sequence.jsonl`은 추적), `trace.json`, `actuation_trace.json`, `stop_trace.json`, `acquisition.json`, `runtime_world.sdf`, `batched_*.obj`, `__pycache__`.
- 이번에 추가한 규칙으로 무시되는 파일: 3인칭 주행 영상 14개(약 17 MB). 보고서는 영상에 링크하지 않는다.
