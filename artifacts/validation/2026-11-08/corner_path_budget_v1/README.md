# 관측 벽 길이와 오프셋 경로 길이 비교

상태: **기본 OFF `piecewise_wall_trial` 안의 계산 보완을 구현했고, 저장 입력 재생까지 마쳤습니다.** 새 주행·Gazebo·빌드·커밋은 하지 않았습니다. 정지 구간의 저장 입력에서 양수 명령이 나왔습니다. 다만 이는 저장 센서와 대조한 결과이며 **실제 재출발을 증명하지 않습니다.**

## 1. 진단 경위 (구현 전)

교통 한 랩 원본은 첫 코너 뒤 10.90 m에서 장기 정지했습니다. 기존 정지 재획득 옵션은 선호 벽이 충분해야 동작합니다. 그래서 같은 저장 입력 1,518판단에서 변화가 없었습니다. [원인 재현](../traffic_reacquire_replay_v1/README.md)을 먼저 참고합니다.

확인할 점은 두 길이의 차이였습니다. 하나는 벽을 따라 1.8 m를 읽는 것이고, 다른 하나는 그 벽에서 0.505 m 떨어진 주행 경로를 1.8 m 확보하는 것입니다. 바깥 직각 벽에서는 평행 이동과 코너 원호 때문에 경로가 짧아집니다. 저장 입력에서 오른쪽 벽의 관측 길이는 1.8 m였지만, 생성 경로는 약 0.628 m로 요구치 0.65 m보다 짧았습니다.

`geometry.py`는 같은 실제 반환으로 벽 탐색 길이만 1.8/2.4/3.1 m로 바꾸어 비교합니다. 첫 실행은 `motion_scene` import 위치 오류로 실패했습니다. 이 실패 기록은 `geometry_v1.log`로 보존합니다. `geometry_v1.exit=0`은 Python 성공을 뜻하지 않습니다. import를 고친 v2(`geometry_v2.exit`: `PYTHON_EXIT=0`)의 결과가 `geometry.json`과 `observed_geometry.png`입니다. 두 파일은 수정 전 `local_path.py`로 계산한 것이며, 이번 작업에서 덮어쓰지 않았습니다.

| t (s) | 벽 1.8 m 경로 | 벽 2.4 m 경로 | 벽 3.1 m 경로 |
|---|---|---|---|
| 27.92 | 0.654 지지 | 0.832 지지 | 0.832 지지 |
| 28.22 | 0.630 `corner_reference_short` | 0.955 지지 | 0.955 지지 |
| 33.02 | 0.628 `corner_reference_short` | 1.214 지지 | 1.866 지지 |

## 2. 구현 (`local_path.py` 한 파일)

`piecewise_corner=True` 분기에서 기존 `corner_path`가 성공한 경우에만 다음을 추가했습니다. 기본 OFF 경로, 직선 경로, 곡선 적합 fallback 경로는 바이트 단위로 같은 계산을 그대로 거칩니다. 새 runtime flag는 없습니다.

1. 기존 연결 분할·앵커·최소 길이·표본화·`corner_path`를 먼저 그대로 수행합니다.
2. 원점 최근접 경로점에서 경로 끝까지의 길이(`_path_ahead`, `corner_distance`의 `reference_ahead_m`와 같은 정의)가 요청 `horizon`보다 짧은 경우를 봅니다. 이때 같은 선택 벽의 호길이에 실제 관측이 더 남아 있으면 **딱 한 번** `last`를 `min(arc[-1], last + max(0, horizon-path_ahead) + .05)`로 늘립니다. `.05`는 재표본화 표현 여유입니다. 같은 `first`·연결 구간·앵커·실제 벽 점으로 다시 표본화한 뒤 기존 `corner_path`를 호출합니다. `local_path` 재귀 호출이나 다른 구간으로의 교체는 없습니다.
3. 새 후보는 다음 세 조건을 모두 만족할 때만 채택합니다. 그렇지 않으면 원 후보를 그대로 둡니다.
   - `corner_path`가 성공해야 합니다. 이 함수가 한 모서리, 선분 잔차 0.03 m, 각도 40–135°, 조향 최소 반경 조건을 그대로 검사합니다. 두 번째 모서리가 들어오면 이 단계에서 거부됩니다.
   - 원 경로의 36점이 모두 새 경로의 **선분**에서 0.03 m 이내여야 합니다(`_covers`). 0.03 m는 기존 벽 적합 허용량과 같습니다.
   - 경로 앞 길이가 줄지 않아야 합니다.
4. 채택한 경우 `boundary_observed_ahead_m`는 실제로 사용한 `last-anchor_s`로 기록합니다. 진단 `corner_path_budget`(원 벽 예산·원 경로 길이·새 경로 길이·`extended`)은 확장한 경우에만 추가합니다.

0.65 m 경계 길이, TTL, 안쪽 지지, 점유, STOP·독립 보호, 속도 값은 바꾸지 않았습니다. 관측 끝 밖 외삽·없는 점·미관측 자유 공간도 추가하지 않았습니다. 변경 전후 차이는 `local_path.diff`(58행)에 있습니다.

## 3. 출처 계약

| 항목 | 값 |
|---|---|
| 구현 전 스냅샷 | `before.zip` sha256 `8edc03035f6a5bb0c82429dcb21edeb398b444d79a4ba28e68cf9f91d2f56407`. `src/arena_autonomy` 패키지 전체(`__pycache__` 제외), `tests/test_piecewise_wall.py`, `tests/test_source_provenance_r0_plus_b3.py`, r20 계약까지 110개 파일 |
| `local_path.py` | before `688632e4…887e8e`, after `d53210ade534f93ab47637f485387ba3d3e88090e5cbed75951409433b668ac4` |
| 계약 | `config/tests/corner_path_budget_r21_sha256.json`. parent는 `official_grid_multi_r20_sha256.json`(sha256 `9f9aba9b…104874`). `approved_changes`는 `local_path.py` 하나, `approved_evaluation_scripts`와 `new_dependencies`는 비어 있음 |
| 시험 연결 | `tests/test_source_provenance_r0_plus_b3.py`의 `M3_SUCCESSORS` 끝에 r21 추가. 기존 계약 파일은 수정하지 않음 |

## 4. 검증

모든 실행은 WSL에서 했습니다. 환경은 `2026-11-01/turn_preview_v1/environment.sh` 또는 같은 PYTHONPATH를 쓰는 `run_tests.sh`입니다. 실행과 대기는 이번 작업의 Claude Opus 5.5가 직접 했으며, 보조 에이전트는 쓰지 않았습니다.

| 검사 | 명령(저장소 루트 기준) | 결과 |
|---|---|---|
| 합성 직각 벽 전/후 | `bash …/run_v3.sh synthetic_check v1` | `synthetic_check_v1.exit.txt` EXIT=0, `synthetic_check.json` |
| 관련 회귀 18파일 | `bash …/run_tests.sh related <18 files>` | EXIT=1, **425 passed, 1 failed** (`pytest_related.log`) |
| 호출자·출처 시험 39파일 | `bash …/run_tests.sh provenance_and_callers <39 files>` | EXIT=1, **389 passed, 2 failed** (`pytest_provenance_and_callers.log`) |
| 저장 입력 재생(after) | `bash …/run_v3.sh replay_after v1` | `replay_after_v1.exit.txt` EXIT=0 WALL_S=31 |

### 회귀 실패 3건: 모두 기존 실패

아래 3건은 [11/7 기준선](../../2026-11-07/track_migration_v1/checks/pytest_baseline.log)에도 같은 이름으로 실패가 기록되어 있습니다.

- `test_corner_follow.py::test_corner_source_contract_has_explicit_parent_and_unchanged_previous_bytes`: `local_follow.py` 해시가 고정된 옛 후속 목록과 다릅니다. 이 파일은 수정하지 않았으며 before.zip과 같은 바이트입니다.
- `test_r0_safety_contract.py::test_current_g1b_r0_contract_allows_only_the_registered_launch_change`: 다른 파일의 해시(`e3938990…`)입니다.
- `test_measured_lane_preference_provenance.py::…`: `local_pursuit.py`의 r19 after 해시입니다.

### 합성 장면 (`tests/test_piecewise_wall.py`에 추가한 시험 포함)

장면은 안쪽 벽이 x=0.1에서 끝나고, 바깥 벽이 x=1.1에서 90° 꺾이는 구조입니다. offset은 0.505 m이며 좌우 거울상을 모두 확인했습니다.

| 경우 | 수정 전 경로 | 수정 후 |
|---|---|---|
| 바깥 벽 반환이 길게 남음 | 0.611 m(벽 1.8 m) | **1.826 m**(벽 3.039 m). 원점 부근과 반경 0.505 원호를 유지하고, 원 경로를 0.03 m 이내로 포함함. 입력 배열 불변 |
| 같은 구간 관측 끝이 짧음(벽 2.0 m에서 끝) | 0.611 m | 0.798 m. 실제 끝까지만 읽으며 1.8 m로 채우지 않음 |
| 위 장면에 끊긴 먼 벽 조각(x=1.1, y 0.9–1.6) 추가 | — | 0.798 m. 먼 조각이 없는 경우와 경로·info 동일(붙이지 않음) |
| 추가 창에 두 번째 모서리 | 0.611 m | 원 후보 보존(`corner_path_budget` 없음) |
| 직선, 기본 OFF | — | 수정 전 모듈과 출력·info 완전 동일 |

참고: 관측 끝이 원호 중간에서 끝나면 기존 `corner_path`가 가장 가까운 원호점에서 자릅니다. 이 때문에 경로 끝이 관측 끝의 오프셋보다 y 방향으로 약 0.011 m 더 나갈 수 있습니다. 이는 수정 전부터 있던 표현이며 이번 변경과 무관합니다.

## 5. 저장 입력 재생 (교통 한 랩 run1, 0–38초)

원본 `traffic_lap_newcourse_v1/two_actor_course_lap/run1/follow_decisions.jsonl`의 sha256은 `aebdc4e7…8d8585`이고, 재생 전후로 같았습니다. 헤더 config 그대로(`piecewise_wall_trial=True` 기록값) 전체 `BypassHandoffFollow(..., allow_simulation_trial=True)`를 구성했습니다. reset 379회와 call 1518회는 seq 순서대로 재생했습니다.

**OFF/수정 전 기준의 재사용.** 기본값 재생이 원기록과 1518/1518 일치한다는 [이전 결과](../traffic_reacquire_replay_v1/replay_result.json)를 재사용했습니다. 재사용 조건은 해시로 확인했습니다(`baseline_reuse.valid=true`).

- 원본 sha가 같습니다.
- 이전 실행에서 로드한 `local_path.py`가 before.zip과 같은 `688632…`입니다.
- 이번 실행에서 로드한 60개 소스 중 이전과 달라진 파일은 `local_path.py` 하나뿐입니다.

이 조건이 성립하므로 수정 전 재생을 반복하지 않았습니다. 수정 후 로드 소스는 `after_loaded_source_snapshot.zip`에 보존했습니다.

| 구간 | 결과 |
|---|---|
| 기하(벽 1.8 m 요청) | 27.92초: 경로 0.654→**0.832 m**. 28.22초: 0.630→**0.955 m**. 33.02초: 0.628→**1.797 m**. 세 시점 모두 `observed_corner_distance_support`이며, 이는 수정 전 2.4/3.1 m 탐색 결과와 같은 값·같은 판정입니다 |
| 전체 차이 | 원기록 대비 comparable 출력 차이 617/1518 call. 첫 차이는 24.02초(조향 1e-5 rad 수준, 오른쪽 경로 후보 길이 증가). 확장 채택 call은 123회 |
| 정지 전 0–28.12초 (1023 call) | 명령/사유 차이 120회, 속도 차이 22회. 24.82–25.10초는 원기록이 `follow_boundary_unconfirmed` 정지였으나 수정 후 `local_follow` 0.65–0.9 m/s(`bilateral_observed_corner_distance`)입니다. 25.12–25.30초에는 상태가 갈라져 수정 후에 정지/`follow_release_wait`이 생겼습니다. 원기록은 25.22초에, 수정 후는 25.32초에 재출발했습니다(0.1초 늦음). 그래도 정지 길이는 원기록 24.82–25.20초(0.4초)보다 수정 후 25.12–25.30초(0.2초)가 짧습니다. 27.84초는 swept clearance 0.4 m의 FOLLOW 감속(0.3 m/s)에 한 call 먼저 들어갔습니다 |
| 정지 구간 28.12–38초 (495 call) | 원기록: 양수 0, `follow_boundary_unconfirmed` 495. **수정 후: 양수 495/495, 0.85–0.95 m/s, `local_follow` 495**. 새 스캔 경계 선택은 `bilateral_observed_corner_distance` 99회, 경로 벽 right, 조향 -0.024~+0.003 rad, swept clearance 0.84–0.92 m |

해석과 한계는 다음과 같습니다.

- 정지 원인이었던 "반대 벽 코너 근거 길이 미달(0.628 < 0.65)"은 같은 실제 반환을 더 읽는 것으로 해소되었습니다. 0.65 m 기준과 안쪽 지지 검사는 그대로 통과했습니다.
- **개루프 재생입니다.** 24.02초 이후 제어기 상태가 원기록과 갈라지지만, 센서 입력은 원래 주행(정지한 차량)의 것입니다. 정지 구간의 양수 명령은 정지 위치에서 본 저장 스캔에 대한 판단일 뿐입니다. 실제 재출발, 코너 통과, 접촉 여부는 확인하지 않았습니다.
- 정지 구간의 조향이 거의 0인 이유는 다음과 같습니다. 저속 lookahead(0.30 m) 안의 경로가 바깥 벽 평행 직선이기 때문이며, 원호는 그보다 앞에서 시작합니다. 직진 swept clearance는 0.84–0.92 m입니다. 0.85–0.95 m/s 명령 뒤 실제로 원호를 따라 꺾는지는 폐루프에서만 알 수 있습니다.
- 25.12–25.30초에 상태가 갈라져 생긴 정지/release wait 차이도 개루프 산물일 수 있습니다. 실제 영향은 판단하지 않았습니다.
- 처리 시간은 비교하지 않았습니다(comparable 비교에서 벽시계 비용 제외).

## 파일

- 구현 기록: `before.zip`, `apply_patch.py`(적용 스크립트, 1회 실행), `local_path.diff`
- 진단(구현 전): `geometry.py`, `geometry.json`, `observed_geometry.png`, `geometry_v1.*`(import 실패 보존), `geometry_v2.*`, `run.sh`
- 합성: `synthetic_check.py`, `synthetic_check.json`, `synthetic_check_v1.{log,exit.txt}`
- 재생: `replay_after.py`, `replay_after_result.json`, `replay_after_rows.jsonl`, `replay_after_v1.{log,exit.txt}`, `after_loaded_source_snapshot.zip`, `analyze_after.py`, `analyze_after.json`
- 회귀: `run_tests.sh`, `pytest_related.*`, `pytest_provenance_and_callers.*`
- 실행 래퍼: `run_v3.sh`(명령·종료 코드·소요 시간 기록)
