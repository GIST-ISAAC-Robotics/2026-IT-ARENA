# GUARD 곡선 후보 — 구현 중 인계 (미완)

2026-10-27. Claude(Opus 5.5)가 작업하다 **5시간 사용량 경고**로 새 실행·작업 묶음 없이 GPT에 인계한다. 이 문서는 재개용이며 결과 보고서가 아니다. 기본 `g1_guard_curve_trial=false`, 기본 strict·통과 선택기 OFF는 바뀌지 않았다. Gazebo 시험은 시작하지 않았다.

## 1. 상태

- 실행 중인 Gazebo/ROS/pytest 프로세스 0(인계 직전 WSL `ps` 확인), Claude 보조·감시 작업 0, 예약 대기 알림 0(앞 묶음 알림은 이미 취소).
- 출발점: [operation_v1](../g1_pass_release_operation_v1/README.md) 수용 상태(선택기 `ff310084…`, 전체 5,885 통과/19 건너뜀). run1 실패 원인은 GUARD의 직선 전용 평행 후보(`reference_not_straight` 807판단)였다.
- 변경 파일 6개는 run1 스냅샷 대비 [`wip_changes.patch`](wip_changes.patch), 파일 원본은 [`wip_changed_files.zip`](wip_changed_files.zip), 전후 해시는 [`wip_hashes.json`](wip_hashes.json)에 보존했다.

| 파일 | WIP sha256(앞 12자) | 내용 |
|---|---|---|
| `src/arena_autonomy/arena_autonomy/pass_release_trial.py` | `7a2e5463f157` | 새 후보 `_guard_curve`, `region_blocked`(RELEASED 거절 규칙 추출, 동작 동일 의도), `side_extent`, GUARD 진입 횡위치 `guard_anchor`, `select()`→`_select()` 래퍼로 `last_choice` 기록, `SelectorConfig.guard_speed_cap_mps=.35`, 모듈 설명 |
| `src/arena_autonomy/arena_autonomy/local_follow.py` | `a7cd99b3f48f` | `FollowConfig.guard_curve_trial=False`, `pass_release_trial` 없으면 ValueError |
| `src/arena_autonomy/arena_autonomy/local_pursuit.py` | `3184e8664866` | ROS 인자 `g1_guard_curve_trial`(기본 false), 선택기 없으면 ValueError |
| `src/arena_bringup/launch/demo.launch.py` | `900965c73573` | 같은 launch 인자·SetParameter |
| `src/arena_bringup/launch/local_pursuit.launch.py` | `7aee1a9b82c2` | 같은 인자 전달 |
| `scripts/g1_partial_trial.py` | `5e96c6e72f43` | `guard_curve_arguments`, trace에 `selector_guard_candidate`, `pass_release_evidence(trace, guard_curve=False)`가 `guard_baseline` 선택을 GUARD·accepted일 때만 허용하고 행 수 집계 |

## 2. 구현한 설계(검증 전)

`_guard_curve`는 `guard_curve_trial`이 켜진 GUARD 분기에서만 호출된다.

1. **조건:** 평행 후보 출력이 0이고 이유가 `parallel_hold_reference_not_straight`일 때. 직전 선택이 `guard_baseline`이면 평행 후보의 `parallel_hold_release_wait`도 포함한다(곡선 출구에서 평행 후보 확인 대기로 끊기는 것 방지). 기존 G1 이유가 `local_follow`/`local_follow_corridor_candidate`이고 속도 > 0, 측정 속도 ≥ 0이어야 한다. 그 밖의 경우(전방 gap_hold, release_wait, 입력 거절, 역행, 시각/세대 고장, STOP)는 종전 경로가 먼저 처리하므로 양수 후보를 만들지 않는다.
2. **목표:** `min(G1 목표, 0.35)`. 예측은 `max(측정 속도, 목표)`로 한다(감속 완료 가정 없음).
3. **누적 접근 거절:** 해제되지 않은 쪽(`seen` 그리고 `passed and not present`가 아님)마다 현재/직전 조향의 `command_arc`(1 cm)+`extrema_arc` 자세에서 차체 네 모서리의 **G1 경로 좌표 횡위치** 최대를 구한다. GUARD 진입 첫 관측 때의 값(`guard_anchor`)보다 1 cm 넘게 그 쪽으로 가면 `guard_curve_rejected_toward_<side>`로 거절한다. 기준은 GUARD를 벗어나거나 `invalidate()`·`reset()`이면 지운다.
4. **이동 검사:** 차체 모서리 횡변위(자차 좌표) ≤ 1 cm이면 `guard_curve_small_motion`. 그보다 크면 `motion_region`과 `region_blocked`로 일반 미관측·1 cm 초과 가림 꼬리·전면 가림을 거절한다(`guard_curve_rejected_large_motion_*`). 통과는 `guard_curve_observed_motion`(작은 꼬리면 `..._small_masked_tail`)이다.
5. **기록:** 선택은 `selector_choice='guard_baseline'`, 상태는 `GUARD` 그대로다. `selector_guard_candidate`에 trigger·G1/목표/예측 속도·조향·envelope·쪽별 anchor/current/predicted·횡변위·영역·accepted를 남긴다. 거절하면 평행 후보 출력(정지)에 거절 이유를 붙인다. `passed`/`vanish`는 건드리지 않는다.

## 3. 실제로 실행한 검사

- 6개 파일 `ast.parse` 통과.
- [`handoff_quick_tests.log`](handoff_quick_tests.log): `tests/test_pass_release_trial.py`·`test_g1_partial_traffic.py`·`test_local_follow.py`·`test_source_provenance_r0_plus_b3.py` 중 **193 통과·7 실패**. 7건은 모두 출처 사슬 시험이며 `local_pursuit.py` 현재 `3184e866…` ≠ 계약 `after` `a72ed22b…`(계약 미갱신, 예상된 미완 상태)다. 기존 선택기 67 반례는 플래그 OFF 기본값에서 통과했다.
- 새 후보에 대한 단위·기하 반례, run1 입력 대조, 빌드, 전체 회귀, Gazebo는 **미실행**이다.

## 4. 남은 일(권장 순서)

1. **run1 805 입력 설계 대조(가장 먼저):** `g1_pass_release_operation_v1/run1_corner/follow_decisions.jsonl`에 헤더 설정 + `guard_curve_trial=True`를 적용한 개루프 비교. `../g1_pass_release_operation_v1/compare_cross_inputs.py`를 복사해 OFF=기존 헤더, ON=`replace(cfg, guard_curve_trial=True)`로 바꾸면 된다. 확인할 것: 20.3–39.3 s에서 `guard_baseline` 채택 수와 `guard_curve_rejected_*` 분포, 특히 `toward_left`. U자 코너는 왼쪽으로 돌고 옆차도 왼쪽이다. 중앙 경로를 따라가면 경로 횡위치는 거의 그대로일 것으로 **추정**하지만, 자차가 GUARD 진입 때 중앙 오른쪽에 있었고 G1이 중앙으로 돌아가려 하면 1 cm 기준으로 계속 거절될 수 있다(미확인, 가장 큰 불확실성).
2. **반례 시험**(`tests/test_pass_release_trial.py`, 기존 픽스처는 `band_check`·`threat_view`를 대체함):
   - 평행 후보가 진행 가능하면 평행 유지
   - `reference_not_straight` 외 이유(`unchanged_gap_hold`, `lateral_motion_not_small`, `partial_observation_outside_trial`, `short_reference`)에서는 후보 없음
   - G1 gap_hold·release_wait·0 속도·비정상 이유에서는 양수 없음
   - 목표 상한 0.35와 더 낮은 G1 목표 존중
   - 측정 0.8 m/s일 때 예측 속도 0.8 사용
   - 옆 물체 쪽 연속 조향의 누적 접근(호출마다 경로 횡위치 0.4 cm 증가 → 거절)
   - 큰 이동의 미관측·1 cm 초과 꼬리·전면 가림 거절
   - present 옆차가 있어도 멀어지는 이동만 허용
   - 시각 역행·세대·장면 나이·NaN·역행 속도의 기존 정지 유지와 anchor 초기화
   - 상태 GUARD·`vanish` 불변
   - `release_wait` 연속 조건은 직전 `guard_baseline`일 때만
   - `FollowConfig(guard_curve_trial=True)` 단독은 ValueError, 플래그 OFF면 기존 출력 동일
   - `side_extent` 곡선 경로 기하(중앙 추종 ≈ 0, 안쪽 조향 > 1 cm)
   - `guard_curve_arguments`/`pass_release_evidence(guard_curve=True)`
3. **시험기 연결:** `scripts/validate_g1_traffic.py`(**CRLF**, 바이트 단위로 수정)에 다음을 추가한다.
   - `--guard-curve-trial` 옵션
   - `guard_arguments = guard_curve_arguments(args.guard_curve_trial, args.pass_release_trial)`
   - 690행 `extra`에 추가
   - 1162행 블록에서 `pass_release_evidence(trace, guard_curve=bool(guard_arguments))`와 보고서 `guard_curve_arguments`
4. **출처 계약**(코드 확정 뒤): `config/tests/partial_observation_r0_sha256.json`에서 다음만 갱신하고 사유를 덧붙인다. `before`·zip은 유지한다.

   | 구분 | 경로 | 현재 기록 → WIP |
   |---|---|---|
   | `approved_changes`의 `after` | `local_follow.py` | `7696ef1e…` → `a7cd99b3…` |
   | 〃 | `local_pursuit.py` | `a72ed22b…` → `3184e866…` |
   | 〃 | `demo.launch.py` | `ac80cc3f…` → `900965c7…` |
   | 〃 | `local_pursuit.launch.py` | `a8bd0725…` → `7aee1a9b…` |
   | `approved_evaluation_scripts`의 `after` | `scripts/validate_g1_traffic.py` | 수정 뒤 값 |
   | `dependencies` | `pass_release_trial.py` | `ff310084…` → 최종값 |
   | 〃 | `scripts/g1_partial_trial.py` | `b696b9e5…` → 최종값 |

   그다음 `config/tests/profile_bundle_selector_r0_sha256.json`의 `parent_contract_sha256`을 새 부모 해시로 맞춘다. `tests/source_contract_helpers.py`가 `approved_changes`의 `after`=현재 바이트를 검사한다.
5. `colcon build --symlink-install --packages-select arena_autonomy arena_bringup`, 관련 회귀(`../g1_pass_release_operation_v1/related_tests.txt` + 새 반례), 전체 회귀. `../g1_pass_release_operation_v1/run_pytest.sh <log> <tests...>`는 ROS 환경과 `PYTHONPATH`를 덧붙여 로드한다.
6. 스냅샷과 대표 코너 1회:
   - `../g1_pass_release_operation_v1/run1_snapshot.py`, `run1_trial.sh`를 이 폴더로 복사해 이름만 바꾼다.
   - 주행 명령에 `--guard-curve-trial`을 추가한다(나머지 인자·등록 조건은 run1과 동일).
   - 사후 평가는 `../g1_pass_release_operation_v1/g1_window_eval.py <run> <out>`(run27 재현 확인 포함)으로 하고, 선택 이유 집계에 `guard_baseline`을 더한다.
   - 확인 항목: 소스 불변, 판단 동일 재생, 바퀴 노면, 접촉, 정지·재개 15 s 창, 코너 출구, STOP·14/14 종료.
   - 원래 40 s 판정선은 그대로 둔다.

## 5. 열린 설계 질문(근거 미확보)

- `guard_anchor`는 스캔마다 다시 관측되는 G1 경로 좌표에 걸려 있다. 경로 추정이 바뀌면 1 cm 기준이 흔들릴 수 있다(오거절 또는 몇 mm 허용).
- 해제되지 않은 쪽에 `present` 옆차를 포함했다. 곡선에서 옆차와 나란히 가는 경우가 되므로 GPT 최종 검토 대상이다.
- `local_follow_corridor_candidate`를 허용 이유에 포함했다. 코너 corridor 출력의 측방 의미는 따로 확인하지 않았다.
- `side_extent`는 1 cm 표본+모서리 좌표 극값만 본다(연속 영역 인증 아님). G1 경로 자체가 옆 물체 반환으로 휘었을 가능성은 점유 검사·이동 영역 검사에 맡긴다.
