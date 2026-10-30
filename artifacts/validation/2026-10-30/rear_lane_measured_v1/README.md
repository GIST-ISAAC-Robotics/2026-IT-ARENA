# 측정속도 비클리핑 직선 선호 — 경계 수정과 `observed_measured_v1` 연결

2026-10-30. 10/29 [오프라인 v4](../../2026-10-29/rear_lane_preference_shadow_v4/README.md)를 운영 코드에 **별도 명시 정책**으로 연결했다. 기본값은 `rear_lane_keep_trial=false`/`rear_lane_keep_policy='timed'` 그대로이며 기존 `observed`(v3)의 의미·출력도 바꾸지 않았다. 운영 채택이나 실차 안전 승인이 아니다. 실제 주행 결과는 [Gazebo 제한 1회](../rear_lane_measured_gazebo_v1/README.md)를 따른다.

## 1. 목표 상한 경계 반례와 수정

v4 원본 `parallel_speed.proposal`은 기본 `target_cap=.35`를 `0 < target_cap <= config.max_speed_mps`로 검사했다. 설정 최대속도가 0.35 m/s보다 낮으면 정상 입력에서도 `invalid_target_cap`을 내고, v4 평행 후보(`parallel_state.MeasuredParallelTrial`)는 새 스캔이 와도 계속 0명령 HOLD에 머문다. 이는 측정속도 검사가 아니라 **명령 상한 검사**의 결함이다.

- **원본 실패 재현:** [`test_v4_boundary_reproduction.py`](test_v4_boundary_reproduction.py)가 수정하지 않은 v4 파일을 그대로 읽어 최대속도 0.2/0.3/0.34999 m/s에서 `invalid_target_cap`과 5회 연속 HOLD를 재현한다. 1.0 m/s 설정(완료한 기존 사례)에서는 영향이 없음을 함께 확인한다.
- **수정(10/30 후속본만):** 요청 상한은 유한한 양수인지만 검사하고, `min(target_cap, config.max_speed_mps)`를 기존 경로 cap과 다시 `min`으로 결합한다. 측정속도는 여전히 자르지 않으며, 측정값이 설정 최대속도를 넘으면 기존과 같이 측정속도 정지공간·해당 경로 이동 영역 관측을 따로 요구한다.
- **유지한 반례:** 0/음수/NaN/±inf/bool/None/문자열 상한, 음수·NaN·inf·스윕 범위 초과 측정속도, 부족한 정지공간, 미관측 이동 영역, 비정상 스캔 나이, 새 스캔 재개 확인, v4의 상태 전이 37건.
- **추가 반례:** 낮은 최대속도에서 원래 `parallel_lane_trial.proposal`과 결과 동일, 어떤 요청 상한에서도 명령 ≤ 설정 최대속도, 낮은 최대속도의 측정 초과 시 자체 영역/정지공간 검사, 같은 스캔 반복으로 재개 확인 수가 늘지 않음, 선택기 요청 상한 ≤ 설정 − 엔코더 1카운트분.

| 검사 | 결과 | 근거 |
|---|---|---|
| r1 | 원본 재현 4통과. 수정본 69통과·1실패 — 새로 쓴 선택기 반례가 다른 최대속도 설정의 선택기를 재사용해 RELEASED 장면이 만들어지지 않은 **검사 작성 오류**. 제품 코드 결함이 아님 | [`boundary_r1.log`](boundary_r1.log) |
| r2 | 해당 검사가 자기 설정의 선택기를 만들도록 고친 뒤 원본 재현 4통과, 수정본 70통과 | [`boundary_r2.log`](boundary_r2.log) |

v4 원본 파일과 `offline_sources2.zip`은 수정하지 않았다. 런타임 시험 파일은 [`make_runtime_tests.py`](make_runtime_tests.py)가 v4 `test_candidate.py`에 모듈 경로만 바꾸고 [10/30 반례](runtime_tests_extra.py.txt)를 덧붙여 만든다.

## 2. 연결 구조

| 파일 | 내용 |
|---|---|
| `measured_lane_preference_trial.py` | v4 `candidate.py`와 설명·속도 제안 import 경로만 다름 |
| `measured_parallel_trial.py` | v4 `parallel_state.py`와 설명·import 경로만 다름 |
| `measured_parallel_speed_trial.py` | v4 `parallel_speed.py` + 위 목표 상한 경계 수정 |
| `local_follow.py` | `observed_measured_v1` 허용. 이 정책일 때만 평행 후보와 선택기를 새 모듈로 선택 |

v4 오프라인은 선택기뿐 아니라 평행 후보도 함께 교체했으므로 연결도 같은 범위를 바꾼다. `local_pursuit.py`는 정책 문자열을 그대로 `FollowConfig`에 넘기므로 바뀌지 않았다. 판단 기록 헤더의 `follow_config.rear_lane_keep_policy`가 재생 정책을 결정한다. 원본과의 차이는 `tests/test_measured_lane_preference_trial.py`가 정확한 치환 목록으로 검사한다.

보존·이관 목록(`prepare_jetson_bundle.py`, `profile_follow_trace.py`, `validate_g1_traffic.py`, `validate_pipeline_faults_ros.py`)에 새 모듈 3개를 더했다. 출처는 새 후속 계약 `config/tests/measured_lane_preference_r1_sha256.json`(부모: `observed_lane_preference_r1`)이 [연결 직전 바이트](before_connection.zip)와 현재 바이트·v4 원본 해시를 연결한다. 과거 계약/스냅샷은 덮어쓰지 않았고, v3 계약 검사는 현재→후속→v3 `after` 순서로 되돌려 대조하도록만 바꿨다. 계약 생성은 [`prepare_contract.py`](prepare_contract.py)다.

CRLF로 기록돼 온 `local_follow.py`·`validate_g1_traffic.py`는 줄바꿈을 그대로 두고 필요한 줄만 바꿨다(새 파일은 LF).

## 3. 연결 검사 — [`check_connection.sh`](check_connection.sh) r1

모든 단계 종료 코드 0. 앞 단계가 실패하면 뒤 단계로 넘어가지 않는 구조다.

| 단계 | 결과 |
|---|---|
| 검사 직전 소스 보존 | [`connection_r1/sources.zip`](connection_r1/sources.zip)(로컬 보존)·[해시 JSON](connection_r1/sources.json) |
| 관련 회귀 | 598 통과 ([log](connection_r1/tests.log)) |
| 과거 기록 동일 재생 | OFF 649/649, 2초 `timed` 653/653, v3 `observed` 652/652, 입력 불변 ([요약](connection_r1/replay_summary.json), 호출별 보고는 로컬) |
| 런타임 ↔ v4 오프라인 | v3 실행 입력 652호출에서 두 번 계산 결정론 652/652, v4 `shadow_gazebo.jsonl` 후보 명령과 652/652 일치. v3 기록 대비 명령 변경 106·선호 적용 142(v4 결과와 같음) ([json](connection_r1/runtime_vs_v4.json)) |
| 빌드·설치 | `arena_autonomy` 빌드, 설치 9모듈 실제 파일·바이트 일치, 기본 OFF/`timed`, 세 정책의 실제 선택 클래스 확인 ([log](connection_r1/installed.log)) |

빌드 경고는 기존과 같은 underlay/byte-compile 안내다. 이 검사들은 함수 회귀·고정 입력 재계산·설치 확인이며 새 궤적이나 주행 성공이 아니다. 저장소 전체 회귀는 이번에 다시 실행하지 않았다.

## GPT 인계 검토와 중단

10/30 15:51 KST. 변경 코드와 후속 계약의 현재/보존 바이트, 실행 기록의 명시 정책·입력 해시, 검사/주행 결과를 대조했다. 관련 불일치는 없었다. 사용자의 인계 후 일시 중단 요청으로 Claude 워커를 종료하고 다음 주행은 시작하지 않았다. 전체 회귀나 시뮬레이션을 중복 실행하지 않았다. [검토 범위](gpt_handoff_review.json).
