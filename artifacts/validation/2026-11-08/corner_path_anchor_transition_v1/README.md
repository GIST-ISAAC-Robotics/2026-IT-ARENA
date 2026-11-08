# 경로 표현 전환 시 앵커 지표 적용 누락 보완 (r23)

상태: **기본 OFF `piecewise_wall_trial` 안에서 지표 적용 누락을 고쳤습니다.** 회귀 시험과 저장 입력 재생까지 마쳤습니다. Gazebo·빌드·커밋·실제 주행은 하지 않았습니다.

저장된 r22 run1의 0–48초 입력을 기록 config 그대로 재생했습니다. 44.02–48초 정지 구간 200판단에서 기록은 양수 명령 0회였고, 후보는 200회 모두 양수 명령을 냈습니다. 다만 이는 정지한 차량의 저장 센서를 그대로 넣은 개루프 대조입니다. **폐루프 재출발이나 완주를 증명하지 않습니다.** 44초 구간의 실제 시험은 상위 검토 뒤 별도로 준비할 예정이며, 이 작업에서는 실행하지 않았습니다.

## 1. 원인

[traffic_anchor_transition_v1](../traffic_anchor_transition_v1/README.md) 진단 결과는 다음과 같습니다.

- 43.82초: 좌우 경로가 모두 `piecewise_corner` 표현이었고 `path_anchor_error_m`가 있었습니다.
- 44.02초: 선호 왼쪽 경로가 일반 매개곡선으로 바뀌었고, 이 표현에는 해당 필드가 없었습니다.
- 44.22·47.92초: 좌우 경로가 모두 일반 곡선이었습니다.

실제 경로 최근접 거리는 약 5 cm였습니다. r22는 `piecewise_corner` 표현에만 지표를 넣었습니다. 그래서 같은 길이 다른 형식으로 표현되면 옛 벽 y좌표 검사로 돌아갔습니다. 이번 수정은 새 안전층이나 임계값 완화가 아니라, 빠진 지표 적용을 채우는 것입니다.

## 2. 구현 (두 소스 파일, `source.diff`)

| 파일 | 변경 |
|---|---|
| `local_path.py` | 마지막 일반 매개곡선 반환에 진단을 추가합니다. 조건은 **호출 인자가 `piecewise_corner=True`일 때만**입니다. 추가하는 값은 `_origin_error(path)`로 계산한 `path_anchor_error_m`입니다. 경로 계산, 선분, `wall_distance_m`, 벽 모델(일반 곡선은 `wall_model` 키 없음)은 바꾸지 않습니다. 기본 호출(OFF)의 반환은 바이트 단위로 같습니다. |
| `local_follow.py` | `anchor_error` 조건을 `wall_model == 'piecewise_corner'`에서 `wall_model in (None, 'piecewise_corner')`로 넓혔습니다. 나머지 조건은 그대로입니다. `piecewise_wall_trial`이 켜져 있어야 하고 필드가 있어야 합니다. 비유한·음수·숫자가 아닌 값은 그대로 `inf`로 거부합니다. 명시된 다른 `wall_model`, 필드가 없는 경우, OFF에서는 기존 `abs(wall_distance_m − wall_offset_m)`를 씁니다. helper 사용처 세 곳은 r22와 같고 새 조건은 없습니다. |

임계값은 하나도 바꾸지 않았습니다. 0.12, 0.08, 0.65 m, TTL, 확인 횟수, 물성, 속도, STOP, 점유는 그대로입니다. 새 flag나 CLI도 없습니다.

## 3. 출처 계약

| 항목 | 값 |
|---|---|
| 수정 전 스냅샷 | `before.zip` sha256 `12362675e30eb5280acf39f9cb8aca1d4de6aaa25cbd6cfdf9c4de6b58938c5d`. 111개 파일: `src/arena_autonomy` 전체(`__pycache__` 제외), 수정한 세 시험 파일, 부모 r22 계약 |
| `local_path.py` | `c9d4efed…84e2a5`(r22 after) → `7ae30cae4784468dc332a0dd6a447c3d4301be5ce9bcdf358c9b24a31ead0268` |
| `local_follow.py` | `79a53ca3…2427c`(r22 after) → `8b98fb2dbad3d233e3ba9f74f355d3eb66c5903fa0c8f98db5d31882bc411d6f` |
| 계약 | `config/tests/corner_path_anchor_transition_r23_sha256.json`(sha256 `4fab1c8b5ccf0fa04f8cc839b3405a62a2f1374de107e0b07d4b1e1f72f7e14e`). parent는 `corner_path_anchor_r22_sha256.json`(`807cd3e4…e36c5`)입니다. 생성 스크립트는 `prepare_contract.py`(`prepare_contract_v1.exit.txt` EXIT=0)입니다. 스크립트 안에서 before 해시가 r22 after와 같은지 assert로 확인했습니다. |
| 시험 연결 | `M3_SUCCESSORS` 끝에 r23을 추가했습니다. 과거 계약과 manifest는 수정하지 않았습니다. |

수정 후 시험 파일 해시는 다음과 같습니다.

- `test_corner_path_anchor.py` `d70d20f7…e8cd`
- `test_piecewise_wall.py` `bbfb7201…24a6`
- `test_source_provenance_r0_plus_b3.py` `c59d5428…cc92`

## 4. 시험 변경 (`tests/`)

- `test_piecewise_wall.py::test_straight_and_default_output_unchanged_and_missing_wall_is_not_filled`: 직선 벽의 경로 배열은 계속 `assert_array_equal`로 같아야 합니다. info 비교만 바꿨습니다. 이제 `path_anchor_error_m` 하나를 제외하고 OFF와 같아야 하며, OFF info에는 이 필드가 없어야 합니다.
- `test_corner_path_anchor.py` 수정 사항:
  - `test_straight_keeps_path_and_default_off_outputs_have_no_new_field`(이름 변경): 위와 같이 진단 하나를 제외하고 비교합니다. 직선 경로 오차는 0이어야 하고, OFF 코너 info에는 새 필드가 없어야 합니다.
  - `test_other_models_and_missing_field_keep_wall_distance_check`: r22의 "모델 None + 필드" 입력은 이제 정상 입력입니다. 이 경우만 "모델 None + 필드 없음(옛 기록)"으로 바꿨습니다. `piecewise_corner` 필드 없음과 `wall_model='other'` 기대는 그대로 둡니다.
- `test_corner_path_anchor.py`에 새로 추가한 시험:
  - `test_curve_fallback_with_field_uses_path_error_only_when_trial_on`: 모델 None + 필드는 ON에서 경로 오차를 씁니다. OFF에서는 벽 거리를 씁니다. NaN·음수·문자열은 `inf`입니다.
  - `test_same_path_keeps_anchor_metric_when_representation_switches`(좌·우 × 이탈 0/±0.2, 6개): 대상은 직각 코너 앞 직선 구간에서 벽에 대해 30° 회전한 차량입니다. 바깥 면이 보이면 `piecewise_corner`, 보이지 않으면 일반 곡선으로 표현됩니다. 실제 `local_path`를 호출합니다. 두 표현 모두 `path_anchor_error_m`가 실제 이탈 |lateral|과 1 cm 안에서 같아야 합니다. 경로 위에 있으면 통과하고 0.2 m 벗어나면 거부해야 합니다. 일반 곡선의 값은 시험 안의 독립 선분 거리 계산과 같아야 합니다. 경로 배열은 기본 OFF 호출 및 수정 전(r22 before.zip의 r21) 선택형 호출과 `assert_array_equal`로 같아야 하고, 나머지 info도 같아야 합니다.
  - `test_arc_vehicle_with_short_outgoing_face_uses_returned_curve_distance`(좌·우): r22의 원호 35° 장면에서 바깥 면을 0.3 m만 남기면 일반 곡선으로 바뀝니다. 이 경우를 확인합니다.
    - 옛 벽 거리 검사는 거부합니다(0.155 > 0.12).
    - 반환 곡선까지의 독립 계산 거리(약 0.099)를 써서 통과합니다.
    - 반환 곡선은 실제 원호보다 모서리를 깎습니다. 따라서 값이 0은 아니며, 반환 경로 기준 거리라는 점을 시험에 명시했습니다.
- 기존 r22 원호 시험 17개(좌우 원호 4, 이탈 4, 직선/OFF 1, 비유효값 5, 다른 모델 3)는 모두 유지되며 통과합니다.

합성 조건 탐색 기록은 `explore.py`·`explore2.py`·`explore3.py`와 각 `*_v1.log`에 있습니다(탐색용이며 exit 파일은 없습니다). 탐색 결과, 반환 일반 곡선을 법선으로 0.2 m 평행 이동하는 방식은 앵커와 시작점이 함께 바뀌어 깨끗한 이탈 조건이 되지 않았습니다(`explore3_v1.log`). 그래서 이탈 판정 시험에는 회전 직선 장면을 사용했습니다.

## 5. 검증

모든 실행은 WSL에서 했습니다. 묶음 `run_all.sh`는 Claude Haiku 5.5 보조가 foreground 에이전트 호출 안에서 1회 실행했고, 완료까지 대기해 결과를 회수했습니다(자체 보고 모델 ID `claude-haiku-5-5`). 실행 명령은 다음과 같습니다(저장소 루트).

```
wsl.exe -e bash -lc "cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' && bash artifacts/validation/2026-11-08/corner_path_anchor_transition_v1/run_all.sh"
```

종료 기록은 다음과 같습니다.

- `run_all.outer_exit.txt`: `OUTER_EXIT=0`
- `run_all.exit.txt`: `replay_base=0 replay_candidate=0 compare=1 new=0 related=1 provenance_and_callers=1`

`compare=1`은 대조 스크립트의 검사 범위 오류였습니다. `before.zip`에는 `arena_autonomy`만 있으므로, 기준선 실행의 `arena_vehicle_interface` 3개 모듈은 작업 트리에서 읽힙니다. 이 3개가 "적재 파일 집합"에 섞여 불일치로 집계되었습니다. 비교를 `src/arena_autonomy/`로 한정하고, 나머지는 `other_loaded_from_work_tree`에 기록하도록 고쳤습니다. 그 뒤 `bash …/run.sh compare`만 다시 실행했습니다(재생은 다시 하지 않음). 결과는 `compare_v1.exit.txt` EXIT=0이고, 8개 검사가 모두 참입니다.

| 검사 | 명령 | 결과 |
|---|---|---|
| 새·수정 시험 3파일 | `run_tests.sh new tests/test_corner_path_anchor.py tests/test_piecewise_wall.py tests/test_source_provenance_r0_plus_b3.py` | EXIT=0, **159 passed** |
| 관련 회귀 21파일(r22와 같은 목록) | `run_tests.sh related …` | EXIT=1, **462 passed, 1 failed** |
| 호출자·출처 39파일(r22와 같은 목록) | `run_tests.sh provenance_and_callers …` | EXIT=1, **389 passed, 2 failed** |
| 기준선 재생(before.zip 소스) | `run.sh replay base` | EXIT=0, WALL_S=39 |
| 후보 재생 | `run.sh replay candidate` | EXIT=0, WALL_S=40 |
| 대조 | `run.sh compare` | 1차 EXIT=1(위 범위 오류) → 수정 후 EXIT=0 |

합계는 851 passed / 3 failed입니다. r22는 841 passed / 3 failed였습니다. 늘어난 10개는 새 시험 9개와, r23 사슬 항목이 추가되며 생긴 parametrize 1개입니다.

### 회귀 실패 3건: 기존 실패와 같음, 새 실패 없음

| 시험 | 기대(고정 옛 값) | 실제 | r22 로그 대비 |
|---|---|---|---|
| `test_corner_follow.py::test_corner_source_contract_has_explicit_parent_and_unchanged_previous_bytes` | `79f0ec5d…` | `8b98fb2d…`(r23 `local_follow.py`) | 같은 옛 기대 해시입니다. 실제 값만 이번 수정 해시로 바뀌었습니다(r22 실제 `79a53ca3…`). |
| `test_r0_safety_contract.py::test_current_g1b_r0_contract_allows_only_the_registered_launch_change` | `3fd02294…`/`4343966e…` | `e3938990…`/`e60cb355…` | 바이트 단위로 같습니다. |
| `test_measured_lane_preference_provenance.py::…preserves_previous_bytes_and_offline_origin` | 동일 | 동일 | 같습니다. |

옛 고정 해시는 지시에 따라 수정하지 않았습니다. r23 사슬 검사(`test_source_provenance_r0_plus_b3.py`)는 통과했습니다.

## 6. 저장 입력 재생 (`replay.py`, `compare.py`, `summary.json`)

| 항목 | 내용 |
|---|---|
| 입력 | `traffic_lap_anchor_v1/two_actor_course_lap/run1/follow_decisions.jsonl` 0–48초(sha256 `a84d1802…e95647`, 재생 전후 동일). call 2,019개, reset 381개. |
| 컨트롤러 | `BypassHandoffFollow` |
| 기록 config | `piecewise_wall_trial=True`, `standing_wall_reacquire=False`, 0.12/0.08/0.65, offset 0.505 |
| 기준선 | probe는 `standing_wall_reacquire=True`였으므로 재사용하지 않았습니다. **기록 config로 before.zip 소스를 직접 1회 재생**했고, 기록 대비 불일치는 **0/2019**였습니다. 적재된 `arena_autonomy` 58개 파일은 before.zip 해시와 모두 같습니다. 후보와 다른 파일은 수정한 두 파일뿐입니다. |
| 계측 | `local_path` 함수만 감쌌습니다. 출력은 바꾸지 않고, 43.4초 이후의 표현과 진단을 기록합니다. 단순 `LocalFollow` 재생이나 기록 정답 대입은 쓰지 않았습니다. |

| 항목 | 결과 |
|---|---|
| 후보 대 기록 변화 | **205판단**, 범위 43.92–48.00초. 43.92초 이전의 1,814판단은 기록과 같습니다. |
| 첫 변화 43.92초 (seq 2196) | 기록: `wall_segment_short` 뒤 기억 경로 지지(`observed_history_support`, 경로 출처 43.703초) → 후보: 새 관측 왼쪽 일반 곡선(`unfiltered`, 1.546 m)을 채택했습니다. 속도는 0.55로 같습니다. 조향만 −0.28826에서 −0.28921로 바뀌었습니다. 이 곡선의 `wall_distance_m`는 0.706(옛 오차 0.201)이고 `path_anchor_error_m`는 **0.111**입니다. 허용량 0.12에 가까운 값입니다. |
| 43.94–44.00초 | 속도와 사유는 같고, 경로 출처 차이로 조향만 다릅니다(4판단). 44.00초에는 기록 0.45, 후보 0.40이었습니다. |
| **44.02–48초 정지 구간 200판단** | 기록 양수 0회(`follow_boundary_unconfirmed` 199, `bilateral_reference_support` 1) → **후보 양수 200회**(0.30–0.75 m/s). 모두 왼쪽 경로, `FOLLOW`이며 기억 사용 0회입니다. 사유는 `local_follow` 160, `local_follow|unfiltered` 40입니다. |
| 43.82초 이후 210판단 | 기록 양수 10(43.82–44.00초) → 후보 210 |
| 변화 사유 | `follow_boundary_unconfirmed → local_follow` 160 + 39 + `bilateral_reference_support → unfiltered` 1, `local_follow` 안 조향 변화 4, `observed_history_support → unfiltered` 1 |
| 표현 통계(43.4초 이후 성공한 `local_path` 호출) | 기준선: `piecewise_corner`+필드 7, 일반 곡선 무필드 81. 후보: `piecewise_corner`+필드 5, **일반 곡선+필드 41**(43.92초 1회 + 44.02초 이후 40회). 후보의 호출 수가 적은 것은 첫 후보가 채택되어 지지·반대 벽 재호출이 줄었기 때문으로 추정합니다(호출 단위로 분리 확인하지 않음). |
| 44.02초 이후 후보 일반 곡선 앵커 오차 | 40회, 모두 0.12 이하. 범위 0.048–0.080 m로, 진단 probe의 약 5 cm와 맞습니다. |
| 조향(변화 판단) | −0.294 ~ −0.06 rad |

해석과 주의점은 다음과 같습니다.

- 정지 구간 입력은 차량이 멈춘 뒤의 같은 장면입니다. 후보 명령이 반복되는 것은 예상된 결과입니다.
- 44.02초에 후보는 정지 상태에서 0.75 m/s를 명령했습니다. 이 값은 기존 속도 규칙의 출력이며, 이번 수정이 속도 계산을 바꾸지는 않았습니다. 실제 가속, 다음 코너, 교통 상호작용은 이 재생으로 알 수 없습니다.
- 첫 변화 43.92초의 경로 오차 0.111은 허용량에 가깝습니다. 이 프레임에서 후보는 기억 경로 대신 새 관측 곡선을 택했습니다. 이 곡선은 반환 곡선 기준 거리입니다. 원래 원호 기준 거리가 아니므로, 모서리를 깎은 곡선에서는 실제 원호와 몇 cm 차이가 날 수 있습니다(§4의 합성 원호 시험 약 0.099).

## 7. 한계와 미확인

- 폐루프 Gazebo 재주행은 하지 않았습니다(지시 범위 밖). 양수 명령이 재출발 성공을 뜻하지는 않습니다.
- `corner_corridor.py`·`corridor_roles.py`·`boundary_evidence.py`의 `wall_distance_m` 앵커 대조는 r22와 마찬가지로 범위 밖이라 바꾸지 않았습니다.
- 전체 저장소 시험은 하지 않았습니다. r22와 같은 영향 범위 60파일만 실행했습니다.

## 파일

`before.zip`, `source.diff`, `prepare_contract.py`(+`_v1.log/.exit.txt`), `explore*.py`·`explore*_v1.log`, `run.sh`, `run_tests.sh`, `run_all.sh`(+`run_all.exit.txt`, `run_all.outer_exit.txt`), `replay.py`, `replay_{base,candidate}_result.json`·`_rows.jsonl`·`_v1.log`·`_v1.exit.txt`, `compare.py`, `compare_v1.log/.exit.txt`, `summary.json`, `pytest_{new,related,provenance_and_callers}.log/.exit.txt`

## 상위 검토 보충 (2026-11-08)

`compare_v1`의 종료 코드 1은 `before.zip`에 다른 패키지가 없어서 생긴 것입니다. 후보 해시 목록에만 보이는 `arena_vehicle_interface`의 3모듈(`__init__`, `drive_feedback`, `queued_log`)은 r22 `two_actor_course_lap/registered.json` sources와 현재 바이트 해시가 같습니다. 따라서 범위 수정은 0입니다. GPT가 독립적으로 확인했고 Claude가 재계산했습니다. 자세한 내용은 `parent_review.json`에 있으며, 원 로그는 보존했습니다.
