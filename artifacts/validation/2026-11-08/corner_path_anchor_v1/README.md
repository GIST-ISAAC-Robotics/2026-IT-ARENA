# 코너 원호 위 앵커 오차를 생성 경로 거리로 판정

상태: **기본 OFF `piecewise_wall_trial` 안의 앵커 오차 계산을 보완했고, 회귀 시험과 저장 입력 재생까지 마쳤습니다.** Gazebo·빌드·커밋·실제 주행은 하지 않았습니다. 저장된 43.82–48초 정지 구간 210판단 모두에서 후보가 양수 명령을 냈습니다. 다만 이는 정지한 차량의 저장 센서를 그대로 넣은 개루프 대조입니다. **폐루프 재출발이나 완주를 증명하지 않습니다.**

## 1. 원인

[traffic_reacquire_after_budget_v1](../traffic_reacquire_after_budget_v1/README.md)의 probe에서 43.72초 선호 왼쪽 `piecewise_corner` 경로는 길이 2.278 m였습니다. 그런데 벽 y거리가 0.669 m라서 앵커 판정에서 떨어졌습니다. 기준은 `|wall_distance_m − 0.505| ≤ 0.12`입니다. 43.92초도 0.697 m로 같은 이유였습니다. 같은 폴더 `anchor_geometry.png`를 보면 차량 원점은 그 경로에서 1–2 cm 안에 있었습니다.

직선 벽에서는 앵커 벽 점의 y좌표가 곧 경로까지의 거리입니다. 그러나 직각 벽을 반경 0.505 m 원호로 도는 동안에는 차량 옆 벽 점이 모서리 쪽으로 이동합니다. 그래서 차량이 경로 위에 있어도 y좌표는 오프셋보다 커집니다. 합성 직각 코너의 원호 35°·40° 지점에서는 벽 y거리가 0.660·0.686 m였고, 경로 거리는 0.0003·0.0019 m였습니다(`explore.py`, `explore_v1.log`). 이 값은 실제 0.669·0.697 m와 같은 형태입니다.

## 2. 구현 (두 소스 파일)

| 파일 | 변경 |
|---|---|
| `local_path.py` | `piecewise_corner` 분기에서 `corner_path`가 성공해 경로를 반환할 때만 진단 `path_anchor_error_m`를 추가합니다. 반환하는 최종 경로의 각 선분에 차량 원점 (0,0)을 직교 투영(0..1로 제한)한 최소 거리입니다. `wall_distance_m`(원시 벽 y좌표)·경로·관측 길이는 그대로 둡니다. 기본 OFF·직선·곡선 fallback 출력에는 이 필드가 없습니다. 외삽·새 센서점·미관측 채움은 없습니다. |
| `local_follow.py` | `anchor_error(details, config)` helper를 추가했습니다. `piecewise_wall_trial`이 켜져 있고 `wall_model == 'piecewise_corner'`이며 새 필드가 있으면 그 값을 씁니다. 그 값이 비유한·음수·숫자 아님이면 `inf`로 거부합니다. 그 밖(기본 OFF, 다른 모델, 새 필드가 없는 기록·스텁)에서는 기존 `abs(wall_distance_m − wall_offset_m)`을 그대로 씁니다. 다음 세 곳의 앵커 검사를 모두 이 helper로 바꿨습니다. ① `sufficient` — 첫 후보, 기억 지지(`supported`), 반대 벽, 양측 확인, 재획득에 공통으로 쓰입니다. ② 반대 벽 대안 진입 조건(`preferred_info`). ③ `boundary_bridge`의 짧은 현재 경로 조건. |

임계값 0.12, `boundary_agreement` 0.08, 0.65 m 길이, TTL, 확인 횟수, 점유·보호, 속도·STOP은 바꾸지 않았습니다. 새 runtime flag나 CLI도 없습니다. 이 값은 근접 장애물 여유가 아니므로 `wall_distance_m`을 쓰는 다른 계산에는 적용하지 않았습니다. 전후 차이는 `source.diff`에 있습니다.

## 3. 출처 계약

| 항목 | 값 |
|---|---|
| 수정 전 스냅샷 | `before.zip` sha256 `5db75a1dc3886a63acae9729481144bbf25cbd6c0402b8dd41fa381d48c113c6`. `src/arena_autonomy` 전체(`__pycache__` 제외), `tests/test_piecewise_wall.py`, `tests/test_source_provenance_r0_plus_b3.py`, r21 계약까지 110개 파일 |
| `local_path.py` | `d53210ad…68ac4`(r21 after) → `c9d4efedf4ab899d928106a8f7251638c4a2af6eb0b045ecdb369031b084e2a5` |
| `local_follow.py` | `a5915f2f…f00e7db`(r19 after) → `79a53ca3ce50b13ee92d877262610f09bf18f3c17edbdad3fb3730cb40e2427c` |
| 계약 | `config/tests/corner_path_anchor_r22_sha256.json`(sha256 `807cd3e4…f9f973e36c5`). parent는 `corner_path_budget_r21_sha256.json`(`d101a648…092885`)입니다. 생성 스크립트는 `prepare_contract.py`입니다. |
| 시험 연결 | `M3_SUCCESSORS` 끝에 r22를 추가했습니다. 과거 계약은 수정하지 않았습니다. |

## 4. 검증

모든 실행은 WSL에서 했습니다. 단계별 명령과 종료 코드는 `*.exit.txt`에 있습니다. 묶음 실행 `run_all.sh`는 Claude Haiku 5.5 보조가 foreground로 실행해 결과를 회수했습니다(자체 보고 모델 ID `claude-haiku-5-5`). 묶음 종료값은 `run_all.exit.txt`(`replay=0 related=1 provenance_and_callers=1`)입니다.

| 검사 | 명령(저장소 루트) | 결과 |
|---|---|---|
| 새 기하 시험 | `bash …/run_tests.sh new tests/test_corner_path_anchor.py` | EXIT=0, 17 passed |
| 관련 회귀 21파일 | `run_all.sh` 안의 `run_tests.sh related …` | EXIT=1, **452 passed, 1 failed** |
| 호출자·출처 39파일 | `run_all.sh` 안의 `run_tests.sh provenance_and_callers …` | EXIT=1, **389 passed, 2 failed** |
| 저장 입력 재생 | `run_all.sh` 안의 `run.sh replay` | EXIT=0, WALL_S=41 |

### 새 시험 `tests/test_corner_path_anchor.py`

- 정확한 좌·우 90° 바깥 벽(오프셋 0.505)을 씁니다. 차량 위치와 heading을 원호 35°·40° 지점으로 옮긴 회전 좌표로 실제 `local_path`를 호출합니다. 벽 y거리가 0.625 m 이상이어도 경로 오차가 0.01 m 미만이면 앵커를 통과해야 합니다. 같은 info에 OFF 설정을 쓰면 기존과 같이 거부해야 합니다.
- 같은 입력을 `before.zip`의 수정 전 `local_path`로도 계산합니다. 경로 배열과 info가 새 필드 하나를 제외하고 같아야 하며, 입력 점 배열은 바뀌지 않아야 합니다.
- 원호 법선 방향으로 ±0.2 m 벗어난 차량은 경로 오차가 세계 좌표 원호 거리와 1 cm 안에서 일치하고, 앵커에서 거부되어야 합니다.
- 직선 벽과 기본 OFF 코너에는 새 필드가 없고 출력도 같아야 합니다. `FollowConfig()` 기본값도 `piecewise_wall_trial=False`, 0.12로 유지되어야 합니다.
- NaN·inf·음수·None·문자열은 거부해야 합니다. 새 필드가 없는 기록·곡선 fallback·다른 `wall_model`은 기존 검사를 유지해야 합니다.

### 회귀 실패 3건: 기존 실패

[r21 작업](../corner_path_budget_v1/README.md)과 같은 시험이 같은 기대 해시로 실패했습니다.

- `test_corner_follow.py::test_corner_source_contract_…`: 고정 옛 목록의 `local_follow.py` 기대 해시 `79f0ec5d…`가 맞지 않습니다. 실제 값은 이번 수정으로 `a5915f2f…`에서 `79a53ca3…`로 바뀌었습니다. 이미 지난 목록이며 r22 사슬 검사(`test_source_provenance_r0_plus_b3.py`)는 통과했습니다.
- `test_r0_safety_contract.py::…`와 `test_measured_lane_preference_provenance.py::…`: 기대값과 실제 값이 r21 로그와 바이트 단위로 같습니다.

## 5. 저장 입력 재생 (`replay.py`)

입력은 `traffic_lap_budget_v1/two_actor_course_lap/run1/follow_decisions.jsonl`의 0–48초입니다. 헤더 config(`piecewise_wall_trial=True`, 0.505, 0.12)와 reset 380회를 그대로 둔 `BypassHandoffFollow`로 2,019판단을 재생했습니다. 원본 SHA-256은 재생 전후 같았습니다.

기준선은 다시 실행하지 않고 `traffic_reacquire_after_budget_v1/replay_result.json`(수정 전 소스, 2,019판단 전체 일치)을 재사용했습니다. 재사용 조건은 다음과 같고 모두 참이었습니다(`baseline_reuse`).

- 로드된 61개 소스 집합이 같습니다.
- 두 수정 파일 외의 해시가 모두 같습니다.
- 기준선의 두 파일 해시가 `before.zip`과 같습니다.

| 항목 | 결과 |
|---|---|
| 기록 대비 변화 | 220판단. 전부 43.62초 이후 |
| 첫 변화 | 43.62초(seq 2180). 기록은 `wall_segment_short` 뒤 기억 경로 지지였고, 후보는 새 관측 경로 2.36 m를 `unfiltered`로 채택했습니다. 속도 0.55는 같고 조향이 −0.2618에서 −0.2627로 바뀌었습니다. |
| 43.72–43.80초 | 기록은 `follow_boundary_unconfirmed` 정지(0)이고, 후보는 `local_follow` 0.55→0.35입니다. |
| 43.82–48초 정지 구간 210판단 | 기록 양수 0회 → 후보 양수 210회(0.30–0.40 m/s). 경로 벽은 left이고 `FOLLOW`입니다. 새 관측 채택 42회, 기억 사용 0회입니다. 조향은 210회 모두 −0.06입니다. |
| 변화 사유 | `follow_boundary_unconfirmed → local_follow` 215회, 같은 `local_follow` 안의 경계 선택·조향 변화 5회 |

정지 구간 입력은 차량이 멈춘 뒤의 같은 장면입니다. 따라서 후보 명령이 반복되는 것은 예상된 결과입니다. 실제로 움직인 뒤의 재획득, 다음 코너, 교통 상호작용은 이 재생으로 알 수 없습니다.

## 6. 한계와 미확인

- 폐루프 Gazebo 재주행은 하지 않았습니다(이번 범위 밖). 단순 양수 명령은 재출발 성공이 아닙니다.
- `corner_corridor.py`·`corridor_roles.py`·`boundary_evidence.py`에도 같은 `wall_distance_m` 앵커 대조가 있습니다. 지시 범위(두 파일)에 따라 바꾸지 않았습니다. 이 경로들이 같은 코너에서 거부하는지는 이번 재생 결과에서 따로 분리하지 않았습니다.
- 재생 행에는 후보의 `path_anchor_error_m` 값을 기록하지 않았습니다. 실제 장면의 경로 거리는 기존 probe/geometry 그림 근거에 의존합니다.
- 전체 저장소 시험은 다시 하지 않았습니다. 영향 범위 60파일만 실행했습니다.

## 파일

`before.zip`, `source.diff`, `prepare_contract.py`, `explore.py`·`explore_v1.log`, `run.sh`, `run_tests.sh`, `run_all.sh`, `replay.py`, `replay_result.json`, `replay_rows.jsonl`, `replay_v1.log`, `loaded_source_snapshot.zip`, `pytest_*.log`·`*.exit.txt`, `summary.json`
