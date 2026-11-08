# 최신 제어기 Jetson 재생 묶음 정비 (r42)

**r41까지의 최신 제어기를 담은 96파일·43,413,654바이트 묶음을 새로 만들었다. 격리 추출본과 현재 저장소에서 두 대표 기록(r25 단독 1,540판단, r41 코너 1,628판단)을 각각 1회 재생했고, 명령이 모두 같았다.** 추출본에서 실제 import된 프로젝트 모듈 70개는 모두 추출 폴더에서 로드됐다. 두 실행 모두 manifest 96파일 해시와 일치했다. Jetson 접속·전송·설치, ROS·Gazebo 실행은 하지 않았다.

**완료 전 수정(r42 내):** GPT 최종 검토에서 v1 묶음의 문서상 `profile_follow_trace.py` CLI가 묶음에 없는 `tests/test_follow_profile.py`를 무조건 해시해 실패한다는 지적을 받았다. 이를 고친 **현재 이관 대상은 [v3 묶음](#cli-진입점-수정-v2v3)** `build/jetson_follow_20261108_v3.tar.gz`(SHA-256 `c7a4f6150a0bd32e48957e4d1455c4ee7f74fc1481d00e11d73e04d386e798df`)다. 아래 v1 결과와 archive는 수정 전 기록으로 보존한다.

- v1(수정 전) 로컬 Git 제외 archive: `build/jetson_follow_20261108_v1.tar.gz`, SHA-256 `fd55c3cc43efaa04a12b0fca760325ec7fce41fc6d628b453aadf5772d56efbb`
- [파일 목록·폐포 기록](bundle_manifest.json), [묶음 검사](verify_result.json), [동일성 요약](parity.json), [저장소 실행](result_repo.json)·[추출본 실행](result_bundle.json), [실행기](bundle_parity.py)
- 이전 [11/2 묶음](../../2026-11-02/handoff_profile_bundle_v1/README.md)과 그 archive는 덮어쓰지 않았다.

## 문제와 변경

`prepare_jetson_bundle.py --follow-profile-only`의 `FOLLOW_PROFILE_TOOLS`와 `profile_follow_trace.py`의 `SNAPSHOT_MODULES`는 고정 목록이었다. 변경 전 관련 회귀에서 `local_path`가 import하는 `piecewise_wall` 누락으로 2개가 실패했다([기준선](checks/pytest_baseline.log)). 현재 코드에서는 아래 17개 모듈이 빠져 있었다.

`adaptive_corridor_offset`, `adaptive_entry_preview`, `corner_boundary_support`, `current_lane_preference`, `fragmented_wall_hypothesis`, `known_structure`, `masked_motion_speed`, `moving_pass`, `moving_pass_wall_padding`, `parallel_side_departure`, `piecewise_wall`, `rear_face_motion`, `short_entry_transition`, `solo_corner_approach`, `solo_straight_budget`, `stopped_entry_centering`, `wall_reference_support`

- 새 `scripts/python_dependency_closure.py`: 명시 진입점에서 AST로 내부 import를 닫는다. 대상 코드를 실행하지 않는다. 최상단·함수 내부 지연 import, 상대 import, 상위 패키지 `__init__`를 따라간다. 내부 패키지·탐색 루트에 있는 이름인데 파일이 없으면 `MissingInternalDependency`(파일:줄)로 실패한다. `from 패키지 import 이름`은 `__init__`에 정의된 이름일 때만 속성으로 허용한다. 그 밖의 이름은 `stdlib`/`third_party`로만 분류하며 설치 여부는 확인하지 않는다. 표준 라이브러리 이름을 가리는 프로젝트 모듈은 거절한다.
- `prepare_jetson_bundle.py`: follow-profile 모드는 진입점 6개(계측·감사·검사 도구 4개와 numpy/pytest만 쓰는 시험 2개), 데이터·문서 3개, 명시 trace만 담는다. 외부 의존성이 `numpy`, `pytest` 밖이면 생성 전에 실패한다. manifest에 `follow_profile_tools_version: 2`와 `dependency_closure`(진입점·루트·외부 이름)를 기록한다. 다른 묶음 모드는 바꾸지 않았다.
- `profile_follow_trace.py`: `SNAPSHOT_MODULES`는 같은 `{패키지: (모듈,...)}` 형태를 유지하되 이 도구의 폐포에서 계산한다(현재 `arena_autonomy` 80개, `arena_vehicle_interface` 2개). 소스 스냅샷은 폐포 파일(패키지 `__init__`·수집기 포함)을 반드시 담는다. `tests/test_follow_profile.py`는 v3 수정 뒤 있을 때만 담고, 없으면 `source_snapshot_omitted`와 `source_snapshot_complete: false`로 보고한다.
- `verify_follow_profile_bundle.py`: 압축을 풀지 않고 archive 안의 `.py`만으로 폐포를 다시 계산한다. 내부 누락, 허용 밖 외부 의존성, manifest와 다른 폐포를 문제로 보고한다. 폐포 기록이 없는 이전 묶음은 `checked: false`로 표시하고 기존 해시 판정은 유지한다.
- `tests/test_follow_profile.py`는 보조 시험(`test_follow_trace` 등)을 통해 `rclpy`·`cv2`·`launch`를 끌어오므로 최소 묶음의 진입점에서 제외했다. 이전 묶음에 들어 있던 이 파일도 보조 시험이 없어 묶음 안에서는 실행할 수 없었다.

주행 제어기·보호층·기본값·물성은 바꾸지 않았다. 원시 trace는 기존 정책대로 Git 제외 대상이다(`artifacts/validation/2026-11-0[78]/**/*.jsonl`, `build/`).

## 소스 계약

[`follow_bundle_closure_r42_sha256.json`](../../../../config/tests/follow_bundle_closure_r42_sha256.json)의 부모는 `current_corner_support_r41`이다. 변경 대상 3개 스크립트는 `handoff_profile_r1` 등 이전 계약에 등록돼 있어 평가 도구 변경(`approved_evaluation_scripts`)으로 기록했다. `python_dependency_closure.py`는 새 의존성으로 기록했다. 변경 전 바이트는 [before.zip](before.zip)에 보존했다. 시험 사슬 `M3_SUCCESSORS`에 r42만 추가했으며 이전 계약은 고치지 않았다. 생성기는 [register_contract.py](register_contract.py)다. v3 묶음 안의 두 스크립트와 수집기 해시는 계약 after 값과 같다(v1 묶음의 `profile_follow_trace.py`는 최초 after `ddc6b2ec…`). `prepare_jetson_bundle.py`는 PC 쪽 도구라 묶음에 들어가지 않는다.

## 검사

| 검사 | 결과 |
|---|---|
| 변경 전 관련 회귀 ([집합](checks/test_set.txt)) | 4실패/267통과. `piecewise_wall` 누락 2개 + 기존 계약 검사 2개 |
| 변경 후 같은 집합 + 새 수집기 시험 | **2실패/285통과**. 남은 2개(`test_measured_lane_preference_provenance`, `test_corner_follow` 계약 검사)는 기준선과 같은 해시 단언이며 이번 변경과 무관하다 |
| `SNAPSHOT_MODULES` 사용 스크립트의 시험 8파일 ([집합](checks/consumer_set.txt)) | 121통과 |
| 새 시험 | 수집기 9개(지연·상대·패키지 import, 내부 누락 4형태, 패키지 속성, 범위 밖 상대 import, 표준 라이브러리 가림, archive 원본=디스크 결과), 묶음 6개(최근 모듈 포함·허용 밖 외부 의존성 거절·archive 재폐포 4형태·이전 묶음 미검사 표시) |
| Python 3.10 문법 | 변경·신규 파일 7개 `ast.parse(feature_version=(3, 10))` 통과. 실제 Jetson Python 실행은 아님 |

중간 단위검사 1회는 `ROOT`를 바꾼 시험에서 trace 검사보다 폐포 계산이 먼저 실행돼 실패했다(1실패/104통과). trace 정책 검사를 먼저 하도록 순서를 고쳤고, 같은 이름의 [로그](checks/pytest_unit.log)는 수정 후 105통과 결과로 덮였다. 당시 명령은 로그 끝 8줄만 출력했으므로, 세션 기록 원문에서 그 출력만 [복구](checks/pytest_unit_first_run_recovered.txt)했다. 전체 로그는 복구할 수 없다.

## 격리 재생

[run_bundle.sh](run_bundle.sh)는 Claude Haiku 5.5 보조 실행으로 1회 수행했다(종료 0). 실행 순서는 다음과 같다.

1. 묶음을 생성한다.
2. 검사기로 해시와 archive 재폐포(91파일, 외부 `numpy`·`pytest`)를 통과시킨다.
3. `build/jetson_follow_20261108_v1_extract`에 `filter='data'`로 추출한다.
4. 저장소와 추출본을 각각 `python -I -B` 자식으로 실행한다. 이때 `PYTHON*`/`ROS*`/`AMENT*`/`COLCON*`/`RMW*`/DDS 환경은 제거한다(`sys.flags.isolated=1`, 남은 해당 환경 0개).

기록 순서·footer 무결성은 `replay_events`와 같은 조건으로 검사했다. 각 판단은 묶음의 `profile_follow_trace.timed_call`로 1회 계측했으며 warmup은 없다.

| 기록 | 판단 수 | 저장소=추출본 명령 | 원 기록 출력과 차이 | 총비용 p50 / p95 / 최대 (저장소 → 추출본) | 20 ms 초과 |
|---|---:|---|---:|---|---:|
| [r25 단독](../solo_gap_envelope_connected_v3/README.md) | 1,540 | 동일 (속도·조향·사유·판단 해시) | 0 | 12.19 / 20.64 / 42.68 → 12.14 / 20.75 / 47.88 ms | 106 → 117 |
| [r41 코너](../current_corner_support_v1/README.md) **(주행 실패 사례)** | 1,628 | 동일 | 0 | 29.04 / 47.46 / 100.77 → 29.26 / 47.84 / 101.40 ms | 1,377 → 1,377 |

- 두 기록 모두 현재 r41 코드의 출력이 원 기록과 전부 같았으므로 별도로 보고할 출력 차이는 없다. r41 기록은 주행 실패 장면의 판단 재현일 뿐 주행 성공 근거가 아니다.
- r41 장면은 전체 판단이 handoff FOLLOW 상태였다. 판단 함수 자체도 p50 24.0 ms, p95 42.0 ms(저장소)였다. 11/2 교통 기록(p95 27.17 ms)과는 입력·코드가 달라 직접 비교하지 않는다. 다만 이 장면의 PC 비용은 참고선 20 ms를 대부분 넘는다. 실제 Jetson·ROS 마감 성능으로 환산하지 않는다.
- 정적 폐포는 실행되지 않은 지연 import까지 포함한다. 이번 재생에서 실제 import된 스냅샷 모듈은 80개 중 64개였다.
- 비용은 WSL2 x86_64·Python 3.12.3·NumPy 1.26.4의 단일 재생 값이다. 저장소/추출본 차이는 같은 코드의 실행 간 변동이며 반복 벤치마크는 하지 않았다.

## CLI 진입점 수정 (v2→v3)

v1 parity는 `timed_call`을 직접 import했으므로 `main()`의 실패를 드러내지 못했다. 수정 내용은 다음과 같다.

- `profile_follow_trace.py`: `SNAPSHOT_OPTIONAL`과 `snapshot_paths()`를 추가했다. 폐포 파일은 필수이고, `tests/test_follow_profile.py`는 있을 때만 기록한다. 없으면 보고서에 누락과 `source_snapshot_complete: false`를 남긴다. 저장소 실행은 이전처럼 완전 스냅샷이다.
- 시험: `test_follow_profile.py`에 tests/ 없는 묶음 배치에서 문서상 CLI를 subprocess로 실제 실행하는 검사를 추가했다. `test_follow_profile_bundle.py`에는 CLI 필수 스냅샷 파일이 모두 묶음에 있는지 보는 검사를 추가했다.
- r42 계약은 같은 번호 안에서 다시 생성했고 `revision` 필드에 경위를 남겼다. `profile_follow_trace.py` after는 `ddc6b2ec…`에서 `f3dbc219…`로 바뀌었다. 최초 계약 사본은 [v2/r42_contract_initial.json](v2/r42_contract_initial.json)이다.

| 출력 | 결과 |
|---|---|
| [v2](v2/cli_result.json) `build/jetson_follow_20261108_v2.tar.gz` | **실패 보존.** v1 `.py`만 푼 폴더에서 같은 문서상 명령을 실행하자 `FileNotFoundError: …/tests/test_follow_profile.py`로 결함이 재현됐다. v2 CLI는 정상 기동해 누락을 보고했으나, 제가 만든 픽스처 footer의 `accepted/written`이 1 적어 `incomplete trace capture`로 정상 거절했다(종료 1) |
| [v3](v3/cli_result.json) `build/jetson_follow_20261108_v3.tar.gz` | **통과.** 98파일, 해시와 archive 재폐포(91파일, numpy·pytest) 통과. v1 대비 바뀐 파일은 `scripts/profile_follow_trace.py` 하나, 추가 파일은 픽스처 2개이며 v1 parity의 두 원 trace는 같은 바이트다. 묶음 안 `profile_follow_trace.py`는 갱신된 r42 after와 같다 |

v3 CLI 검사는 추출 폴더에서 `PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts python3 -B scripts/profile_follow_trace.py <trace> --input-mode stream --warmups 0 --repeats 1 --output …`를 실행했다. PYTHON·ROS·AMENT·DDS 환경은 제거하고 `PYTHONNOUSERSITE=1`로 두었다. 같은 환경에서 `runpy`로 같은 파일을 `__main__`으로 다시 실행해 실제 import 위치를 기록했다.

| 파생 픽스처 | 판단(사이 reset) | CLI 종료 / passed | 기록 일치 | 스냅샷 | 프로젝트 모듈 위치 |
|---|---|---|---|---|---|
| r25 앞 100판단 | 100 (411), `local_follow` 92·양수 속도 92 | 0 / true | 100/100 | 86파일, 시험 누락 보고 | 32개 모두 추출본 |
| r41 앞 100판단 | 100 (381), `handoff_follow` 100·양수 속도 16 | 0 / true | 100/100 | 86파일, 시험 누락 보고 | 64개 모두 추출본 |

픽스처는 원 기록의 header와 앞 판단·reset에 footer만 새로 붙인 파생 입력이다([생성기](v3/make_fixtures.py), [해시](v3/fixtures.json)). 원 기록은 읽기만 했다. 전체 3,168판단은 다시 재생하지 않았다. 제어기 runtime 파일이 v1과 같은 바이트이므로 전체 명령 동일성은 v1 [parity.json](parity.json)에 근거한다. CLI는 앞부분 입력의 진입점 검사일 뿐 전체 장면이나 주행 결과의 근거가 아니다.

회귀(v3 이후): provenance·profile·묶음·수집기·소비 관련 8파일 **246통과**([로그](checks/pytest_v3_after.log)). 진입점 시험만 먼저 돌린 결과는 35통과였다([로그](checks/pytest_v2_entry.log)). v2·v3 실행은 Claude Haiku 5.5 보조가 각 1회 수행했다.

## 한계와 다음 작업

- Jetson 전송·Python 3.10 실행·ARM NumPy 확인, ROS 노드·영상·보호층을 포함한 실시간 마감은 미검증이다. 전송·설치는 별도 승인 범위다.
- `timed_call`은 `raw_scan`을 항상 넘긴다. `partial_observation_policy='strict'` 기록은 이 실행기에서 거절했다. 두 대상 기록은 `bounded_local_v4`다.
- 다음 후보는 r41 장면 handoff FOLLOW 판단(p50 약 24 ms)의 함수별 비용 분해이며 주행 설계 판단은 GPT 주 에이전트가 정한다.

## 활동 기록 (11/8, 일일 마감에서 합칠 항목)

- Claude가 구현·검사를, Claude Haiku 5.5 보조가 묶음 생성·격리 재생 1회를 맡았다. GPT가 설계와 최종 검토를 맡는다.
- 변경 파일: `scripts/python_dependency_closure.py`(신규), `scripts/prepare_jetson_bundle.py`, `scripts/profile_follow_trace.py`, `scripts/verify_follow_profile_bundle.py`, `tests/test_python_dependency_closure.py`(신규), `tests/test_follow_profile_bundle.py`, `tests/test_source_provenance_r0_plus_b3.py`(r42 추가), `tests/test_follow_profile.py`(CLI 진입점 시험 추가), `config/tests/follow_bundle_closure_r42_sha256.json`(신규, 완료 전 재생성), 이 폴더(v2·v3 포함), `docs/PROJECT_CONTEXT.md` 해당 항목.
- 커밋·푸시·외부 게시·Jetson 접속은 하지 않았다.
