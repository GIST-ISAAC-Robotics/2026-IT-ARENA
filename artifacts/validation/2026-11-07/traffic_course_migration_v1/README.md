# 교통·운영 평가 도구의 새 코스 명시 선택 이관

2026-11-07. 작업: Claude Opus 5.5(구현), GPT(설계·최종 검토). 최초 도구 이관은 정적 검사까지 수행했다. 이후 r6로 실제 두 장면을 각 1회 수행했으며 결과는 아래 마지막 절을 따른다.

## 변경 범위

`scripts/validate_g1_traffic.py`, `scripts/official_traffic_course.py`, `scripts/race_ops_eval.py`, `scripts/summarize_motion_lap.py`의 옛 `it_arena_official` 고정을 `--track` 명시 선택으로 바꿨다. 선택지는 `validate_basic_autonomy.TRACK_WORLDS` 하나를 공유하고, 모듈 전역을 덮지 않고 world/track을 인자로 넘긴다. 제어기·`src/`·launch·센서/물성은 바꾸지 않았다.

| 파일 | 변경 |
|---|---|
| `official_traffic_course.py` | `entry_files(world)`, `inputs(..., world, track)`. 설정의 `track`(생략 시 `official`)이 선택 track과 다르면 거부한다. 기본 입력 목록·순서는 그대로 |
| `validate_g1_traffic.py` | `--track`(기본 `official`). 비기본 track은 `--official-signal-course`와 `--local-sensor-config`를 함께 요구한다. 같은 world를 중심선·도로/벽 다각형·입력 해시·`track:=` launch 인자·합류 감사에 전달하고 보고서에 `track`을 남긴다 |
| `race_ops_eval.py` | `--track`, `--local-sensor-config`, `Course(world)`. 정지·재배치·상대 s좌표와 ID30·섬 판정은 `official`에만 등록돼 있어 **새 코스는 모든 운영 시나리오를 실행 전 거부**한다(`REGISTERED_SCENARIOS`) |
| `summarize_motion_lap.py` | `--track`. `report.json`의 track과 다르면 거부하고, 해당 중심선을 겹쳐 그린다 |
| `tests/test_traffic_course_track.py` | 새 시험 10개 |
| `config/tests/traffic_course_track_r5_sha256.json`, `M3_SUCCESSORS` 한 줄 | 부모 `piecewise_corner_r4`. 4개 스크립트를 `approved_evaluation_scripts`로 등록하며 이전 계약은 바꾸지 않았다 |

기본 track의 인자·보고서 형식은 유지된다. 비기본일 때만 `track` 키와 `track:=` 인자가 추가된다. 다만 4개 스크립트의 바이트가 바뀌었으므로 이를 해시로 묶은 과거 등록(r16 `candidate_r16/registered.json` 등)의 `run.py` 재실행은 해시 검사에서 거부된다. 과거 재현은 각 등록의 `source_snapshot.zip`/`sources.zip`으로 한다.

## 새 코스 재등록 제안 (미실행)

- [`new_track_course.json`](new_track_course.json): [`prepare_registration.py`](prepare_registration.py)로 r16 `course.json`에서 생성했다. 바꾼 키는 `derived_from.changed_keys`에 남겼고, 판정·상대 크기·1.6 m 출발 간격·시드·시간 한도는 그대로다.
  - `track=official_v2026_11_06`, `first_corner_s_m=6.7`: [새 중심선](geometry_probe.json)의 s=0 직선 끝 꼭짓점이다. 평가에만 쓴다.
  - `slow_lead`: 중앙에서 0.30 m/s로 14 s 움직이는 앞차다(4.2 m, 기존 `slow_lead` 판정 근거 그대로).
  - `straight_moving_lead`: r16 장면을 그대로 옮겼다. 오른쪽 0.12 m 앞차가 0.30 m/s로 4 s 움직인 뒤 0.58 m/s로 가속한다.
- [`proposed_runs.json`](proposed_runs.json): r16 등록 인자에서 `--config`·`--local-sensor-config`(B0183→`b0390_c1_jr60p20_sim.yaml`)·`--case`만 바꾸고 `--track official_v2026_11_06`을 더한 2회 실행안이다. 실제 실행 시에는 r16과 같은 방식으로 `registered.json`과 소스 ZIP을 새로 만든다.

[`checks/registration_check.json`](checks/registration_check.json) 결과는 다음과 같다(`check_registration.py`, exit 0).

| 확인 | 결과 |
|---|---|
| 위치 | ego slot3 s=41.66 (9.89, 3.52), 상대 시작 s=43.26, 중심 간격 1.60 m. 종료 s는 `slow_lead` 47.46, `straight_moving_lead` 47.888(직선 내, 첫 코너 1 m 전 한계까지 3.88/3.45 m 여유). 오른쪽 상대의 도로 가장자리 최소 거리는 0.11 m(1 cm 표본 464개), 시작 차체는 도로 안이다 |
| 옛 각본 거부 | `official_traffic_v1.json`과 r16 `course.json`은 새 track에서 거부된다. 제안 설정은 기본 track에서 거부된다. r16 인자에 `--track`만 더하면 launch 전에 exit 3으로 끝난다 |
| dry launch 인자 | `ros2 launch` Popen 직전에 argv만 저장하고 종료했다(Gazebo 미실행). 두 제안 모두 `track:=official_v2026_11_06`, `local_sensor_config:=…/b0390_c1_jr60p20_sim.yaml`이다. r16 대비 바뀐 launch 키는 track·센서·출력 경로 3종뿐이며, handoff/moving_pass/g1/pilot 인자는 모두 같다 |
| 기본 경로 보존 | r16 등록 인자(기본 track)의 dry argv가 변경 전/후 코드에서 바이트 동일하다(`checks/default_argv_compare.txt`: IDENTICAL) |

## 실행한 검사

| 명령(WSL) | 결과 |
|---|---|
| `scratch_check.sh new_tests …` (scratch 사본) | py_compile OK, 16 passed, exit 0 |
| `negative_control.sh` (변경 전 스크립트 + 새 시험) | 9 failed / 1 passed, exit 1. 새 시험이 변경 전 동작을 잡는다 |
| `baseline_related.sh` / `dryrun_apply.sh` (관련 26파일, scratch) | 변경 전 103 failed, 적용 dry-run도 같은 103건, 새 실패 0. scratch에는 `artifacts/`가 없어 실패가 많다 |
| `apply_patch.py --check` → `--apply` (작업트리) | exit 0, 패치 사본과 바이트 일치 |
| `run_tests.sh related_worktree` (관련 26파일 + 새 시험) | **530 passed, 3 failed, exit 1**. 3건 모두 변경 전부터 실패: `test_g1_dense_stop`·`test_physics_step_option`의 계약 검사는 오래된 해시 `61917b…`를 기대하고(변경 전 바이트 `76606f…`도 불일치), `test_g1_side_occlusion::…left_side_scene…`는 바꾸지 않은 기본 `DEMO_WORLD` 경로이며 변경 전 사본에서도 실패했다 |
| `environment.sh … check_registration.py` | exit 0 |

로그·exit 파일은 `checks/`에 있다. `base/`는 적용 직전 원본 사본, `patched/`는 적용본, `before.zip`은 후속 계약의 before 스냅숏이다.

## 남은 일과 위험

- 새 코스 교통 2회 실주행은 하지 않았다. 등록·실행은 검토 후 별도로 한다.
- `slow_lead`도 r16과 같은 제어 구성(`--moving-pass-trial` 등)으로 제안했다. 중앙 0.30 m/s 앞차를 추월하려 할 수 있으며, 그 경우 결과로 기록한다. 순수 추종 구성으로 따로 실행할지는 검토자가 정한다.
- `race_ops_eval`의 새 코스 운영 시나리오(정지 지점, 재배치 s, ID30/섬 진행 판정, 주차 상대 s=2.1)는 미등록이다. 새 코스의 ID30 의미와 위치를 정한 뒤 `REGISTERED_SCENARIOS`와 위치 기본값을 함께 재등록해야 한다.
- 새 코스의 launch 단면 매개변수(.505/.305)는 launch가 `track`으로 선택한다. 이번에는 `track:=` 인자 전달까지만 확인했다(실행 중 적용은 기존 course_section 주행에서 확인됨).
- 후속 계약 이름 `r5`는 이 작업에 사용했다. 다른 후속 변경이 있으면 이 계약 다음으로 잇는다.

## 새 직선 대표 2회 실행 (2026-11-07 후속) — launch 가드로 차단

**결론: 두 장면 모두 주행 결과 없음.** `slow_lead` run1은 launch가 Gazebo 시작 전에 거부했다. 원인은 `src/arena_bringup/launch/simulation.launch.py:213-218`의 `_autonomy_executable` 가드다. 이 가드는 G1/G2 handoff(`reserve != off`)에서 `track != 'official'`이면 `ValueError('G1/G2 comparison requires a guarded official track, actor and recorded trial')`를 낸다. 제어기의 동작 실패가 아니라 실행 경로의 결함이다. 앞 단계 dry-args 검사는 `ros2 launch` 직전 argv까지만 봤기 때문에 launch 내부의 이 가드를 잡지 못했다(검증 한계).

`straight_moving_lead`도 같은 `--handoff-reserve paced` launch를 쓰므로 같은 거부가 확정적이다. 같은 실패를 반복하지 않도록 **실행하지 않았다**. 해제하려면 공용 launch 가드를 바꿔야 하는데(예: `official_v2026_11_06` 허용), 이 작업 범위에서는 launch 수정이 금지돼 있어 결정을 넘겼다.

| 항목 | 내용 |
|---|---|
| 최종 선택 | [`accepted_runs.json`](accepted_runs.json). `slow_lead`는 제안에서 `--moving-pass-trial`만 빼서 순수 추종 대조로 쓰고, `straight_moving_lead`는 제안 그대로다. `proposed_runs.json`은 원본 보존 |
| 등록 | `prepare_runs.py register <case>`: `<case>/registered.json`(861 입력 해시, 실행 인자, 명령)과 `source_snapshot.zip`. 두 장면 모두 실행 전에 등록했다 |
| 빌드 | 하지 않았다. [`checks/install_check.json`](checks/install_check.json)은 시험 전용 `test_drivetrain.cpp`까지 비교해 `build_needed=true`로 과잉 판정했다. 실행 대상별로 다시 비교한 [`checks/install_check_targets.json`](checks/install_check_targets.json)에서는 모든 출력이 원본보다 새로워 `build_needed=false`, share 파일 54개도 src와 일치했다 |
| 명령 | `wsl.exe -e bash artifacts/validation/2026-11-07/traffic_course_migration_v1/run.sh slow_lead` (LF wrapper, `environment.sh`) |
| 중단된 시작 | Haiku 보조(`haiku`)가 백그라운드로 시작한 직후 턴을 끝내 셸이 종료됐다. 소스 ZIP 작성 전 단계였다. [`slow_lead/aborted_attempt0/`](slow_lead/aborted_attempt0/NOTE.md)에 보존했고, 이후 실행은 주 세션 백그라운드에서 같은 등록으로 1회 했다 |
| slow_lead run1 | `run1.exit`: `HARNESS_EXIT=1 SURFACE_EXIT=1 INTEGRITY_EXIT=0 LEFTOVER_PGREP=1(잔여 없음) ASSESSMENT_EXIT=1 WALL_S=21`. `report.json`: error `owned process exited early`, `green_observed=false`, `shutdown_clean=false`(launch 미기동, `gazebo_stop_service.error=launch_not_running`), `remaining_test_pids=[]`, `source_unchanged=true`. `postflight_integrity.json`: changed 0. 노면 평가(`surface.log`)와 `assess.py`(`assessment.log`, `case_window` 없음)는 주행 표본이 없어 실패했다. 판정 근거가 아니다 |
| 미검증 | B0390 실제 초록→진행, .505/.305 실제 적용, 속도·정지, 간격·접촉, 추월/양보/복귀, STOP·조밀 정지·정상 종료는 **모두 이번 교통 실행에서 확인되지 않음** |

다음 지점: GPT/root가 launch 가드를 바꾸면, 변경된 소스로 두 장면을 새 이름으로 다시 등록해 각 1회 실행한다. 현재 등록은 바뀌기 전 소스 해시라 `run.py`가 거부한다. `slow_lead/run1`은 원본 그대로 보존한다.

## r6 실제 실행 결과·인계 후 검토

launch 가드의 `track` 조건만 `OFFICIAL_2X5_TRACKS`로 확장했다. guard/actor/evidence 조건은 유지했다. `handoff_track_guard_r6_sha256.json`·관련 213회귀를 통과했고 아래 두 장면을 새 소스 등록으로 각각 1회 실행했다. 앞 절의 run1 미기동 실패는 당시 결과로 보존한다.

| 장면 | 결과 | 한계 |
|---|---|---|
| 중앙 앞차 0.30 m/s, 추월 OFF | `r6/slow_lead/run2`: raw·보충 통과, 진행 4.48 m·최고 1.091 m/s, 진행 후 평균 0.335 m/s, 최소 간격 1.023 m, 접촉/노면 밖 바퀴 0 | 14초 대표 추종 1회. 약10 Hz 평가 표본에서 0.05 m/s 미만 정지 구간 없음 |
| 오른쪽 0.12 m 앞차 0.30→0.58 m/s, r16 추월 ON | `r6/straight_moving_lead/run2`: **실패**. 진행 2.77 m·최고 1.195 m/s, 0.05 m/s 미만 4.249초, 복귀 완료 없음 | 양보/뒤 복귀 명령은 있었지만 실제 차량이 멈췄다. 최소 간격 하계 0.570 m·접촉 0·최대 두 바퀴 잔디. r16 인계 개선을 실제로 확인한 결과가 아님 |

두 실행 모두 B0390·.505/.305 단면·실제 초록 뒤 출발·명시 STOP·정상 종료·867개 등록 해시 불변·잔여 프로세스 없음이다. 원시 `inside_road=false` 등 옛 차체 도로 밖 0 판정은 바꾸지 않았다. 현행 최대 두 바퀴 정책은 별도 `assessment_run2.json`·`surface_run2.json`에 기록했다.

두 번째 장면의 후반에는 `/drive/safe` 약0.376 m/s, MCU ACTIVE·모터 환산 약0.406 m/s가 지속됐지만 차체 속도는 거의0이었다. 보호층 정지나 복귀 미선택과 구별되는 **헛돎**이다. 모터축 하나의 값으로 어느 바퀴가 헛돌았는지까지 알 수 없다. 도로·잔디 1 mm 단차의 기여를 [선택형 평탄 대조](../traffic_surface_alignment_v1/README.md)로 분리한다. 제어기·마찰·모터 출력을 동시에 바꾸지 않는다.

Claude는 5시간 사용량90% 경고 뒤 안전한 인계와 종료를 마쳤다. GPT가 후속 분석·구현을, GPT-6 Luna가 두 번째 실행·결과 회수를 담당했다. 새 코스 운영 시나리오 재등록, 단독 완주, 더 빠른 상대 추월은 여전히 남아 있다.
