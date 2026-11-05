# 단독 직선 속도 후보 재적용 준비 v1 — 격리 사본·저장 입력

- 날짜: 2026-11-05 (KST). 구현·재생·회귀·보고서는 Claude Opus 5.5가 맡았다.
- 대상: [11/3 solo_speed_envelope_v1](../../2026-11-03/solo_speed_envelope_v1/README.md)의 기본 OFF 후보다. 근거리 조향은 그대로 두고, 현재 양쪽 벽이 직선으로 보일 때 그 관측 길이로만 속도를 올린다.
- 범위: 런타임 `src/`·`scripts/`·`config/`·`tests/`, 공용 문서, `install/`은 바꾸지 않았다. Gazebo도 실행하지 않았다. 모든 구현은 WSL 격리 사본(`~/arena_scratch/straight_refresh/`)에서 했다. 11/3 `staged_install`·옛 소스 묶음은 덮어쓰지 않았고, 옛 적용 전후의 **차이만** 현재 코드에 다시 적용했다.
- 제외: 11/3 차이에 함께 있던 외부 녹화 카메라 자세(`model.sdf.xacro`)는 이번 속도 구현에 넣지 않았다.
- 기준: 현재 단독 1랩 [run1](../solo_jr60p20_baseline_v1/README.md)은 32.949 s, 지면 최고 2.393 m/s다.

> **최신 적용 결과(13:13 KST):** GPT 승인에 따라 `apply_live/candidate.patch`를 작업트리에 적용했다(적용 전 해시 일치·`git apply --check` 0·적용 뒤 해시 일치, 계약 r15). 실제 설치 환경 회귀는 653 통과/2 실패(기존 실패 2건과 같음, 새 실패 0). `--straight-budget` 단독 1랩 1회: **31.850 s**(OFF 32.949 s), 지면 최고 2.854 m/s, 10 km/h 이상 0.663 s, 접촉·노면 0, STOP·정상 종료. 43.61 m에서 0.138 s 실제 정지 1회가 생겼다(후보 미작동 구간). [폐루프 결과](../solo_straight_closed_loop_v1/README.md)

## 수용 조건과 결과

| 조건 | 결과 | 근거 |
|---|---|---|
| OFF 동일(단독) | run1 저장 입력 1,779호출: 기준(r13)·후보 모두 기록과 1,779/1,779 일치. 기준과 후보의 판단값 해시도 1,779/1,779 같음 | `summary.json` `off_identity.solo_off` |
| OFF 동일(교통 대표 2건) | r13 이동 추월 `moving_pass_withdrawal_v1/candidate_r13/run1` 549/549, 이동 앞차 기준 `moving_pass_baseline_v1/run1` 740/740. 두 트리 모두 기록과 일치하고 서로도 같음 | `off_identity.r13`, `off_identity.mpb` |
| ON: 실제 근거 직선에서만 속도 상승 | 변경 208호출이 모두 상승(하강 0)이고 사유는 전부 `straight_budget`. 랩 안 110호출, 결승 뒤 STOP 전 98호출. 랩 안 상승 지점의 정답 경로 곡률은 최대 0.126 /m(평가값, 제어 입력 아님) | `solo_on.increased_segments`, `changed` |
| ON: 조향 변화 0, 정지 변화 0 | 조향 변경 0, 판단 사유 변경 0. 0명령 16호출은 OFF와 같은 호출 집합 | `steering_changed`, `zero_set_identical` |
| 42–44 m | 변경 0호출(11/3 결과와 같음) | `changed_in_42_44_m` |
| 관측 불충분 시 원래 목표 | 증속하지 않은 판단의 사유(1,779호출): `existing_path_curved` 1,096, `original_state` 324, `no_longer_budget` 91, `arc_leaves_observed_sides` 32, `observed_hit_remains` 17, `other_speed_limit` 6, `already_faster_than_budget` 3, `different_original_limit` 2. 모두 원래 출력을 그대로 냈다 | `budget_reason_counts` |
| 관련 반례 | 11/3 시험 9개(매개변수 포함 12건) 통과: 전방 무반환·전방 물체·끊긴 옆벽·직전 조향 원호·기억/회랑/부분 관측/재개 대기 상태·곡선 경로·기본 OFF와 비호환 구성·음수 측정 | `targeted_tests.log` |
| 회귀 | 같은 47개 파일(후보는 새 시험 1개 추가)에서 기준 636 통과/3 실패, 후보 648 통과/3 실패. 실패 3건은 두 트리에서 같은 시험이며 원인은 아래 한계 참고 | `regression_base.log`, `regression_cand.log` |
| 출처 계약 | 후보 트리에서 r14 계약을 포함한 일반 후속 계약 검사 89건 통과 | `targeted_tests.log` |
| 플래그·연결 | 정적 확인 9개 통과(아래) | `wiring_check.json` |

**ON의 크기(개루프 명령 모델, 실제 차체·랩 아님):** 계획 최고 2.630 → 2.778 m/s. 부모 발행 모델의 랩 안 최고 2.376 → 2.778 m/s. 목표 95%(2.639 m/s) 이상 발행 시간 0 → 1.12 s(최장 1.04 s). 랩 안 명령 적분 +0.74 m로, 2.5 m/s 전후에서 약 0.3 s다(추정). 증가 구간은 출발 직선 2.0–5.6 m(76호출)와 마지막 직선 45.5 m~결승이 대부분이다. 부모 발행 모델은 [diagnosis_v1](../solo_jr60p20_baseline_v1/diagnosis_v1/README.md)에서 실제 MCU 목표와 최대 0.03 m/s 차이로 대조한 모델과 같다. 11/3 저장 입력 결과(128/1,776 변경, 최고 2.38→2.78 m/s)와 같은 경향이다.

**계산 비용(WSL 재생 벽시계, nice 19, 다른 재생과 병렬):** 호출당 중앙값 13.08 → 13.66 ms, p95 19.59 → 22.30 ms. Jetson·ROS 지연은 아니다.

## 옛 차이와 현재 코드의 합치

11/3 적용 전후(`staged_deployment_before.zip` → `staged_deployment.zip`)의 차이만 옮겼다.

| 파일 | 변경 | 현재 코드와의 관계 |
|---|---|---|
| `local_follow.py` | ① `solo_straight_budget_trial: bool = False`와 구성 검사(단독·관측 경로 속도·`boundary_bridge`·`preview_above_base_speed`·`per_ray_candidate`·`bounded_local_v4` 필요, `ray_wall_continuity`와 함께 불가). ② `pursue_path(..., shape_diagnostic=None)`가 원래 곡률 상한을 진단값으로 남김. 상한 계산식은 같다. ③ `command()`의 마지막에 플래그가 켜졌을 때만 `solo_straight_budget.evaluate` 적용 | 현재 트리에서 늘어난 `known_structure`·구조 설명 광선 허가(`open_route` 조건)와 겹치지 않는다. 후보 모듈은 전방 ±100° 안의 무반환이 하나라도 있으면(구조로 설명된 광선 포함) 원래 출력을 낸다 |
| `solo_straight_budget.py`(새 파일) | 11/3 원본 바이트 그대로(`961b12c9…`) | 의존하는 `check_rays`(`turn_preview.py`)·`source_valid`(`observation_support.py`)는 11/3 당시와 현재 바이트가 같다 |
| `local_pursuit.py` | 파라미터 `g1_solo_straight_budget_trial`(기본 False) 1줄 | |
| `g1_follow.launch.py` | launch 인자 `g1_solo_straight_budget_trial`(기본 `false`)와 `SetParameter` | |
| `validate_basic_autonomy.py` | `--g1-solo-straight-budget`. 기록된 단독 bridge 프로필·`--g1-race-path`·corner_distance·per-ray 조합만 허용하며, 켰을 때 모듈 해시를 `input_sha256`에 기록 | |
| `run_current_solo.py` | `--straight-budget`(기본 OFF). 켜면 검사기 인자 끝에 위 플래그 하나만 덧붙인다 | 기본 인자는 run1 등록과 같음(`wiring_check`) |
| `tests/test_solo_straight_budget.py`(새 파일) | 11/3 원본 그대로 | |
| 출처 계약 | `config/tests/solo_straight_budget_rN_sha256.json`과 `test_source_provenance_r0_plus_b3.py`의 `M3_SUCCESSORS` 1줄. 부모는 적용 시점 사슬의 마지막 계약 | 아래 적용 절차 |

조향·HOLD·물성·브레이크·독립 보호층·부모 가속률(1.5 m/s²)·곡률/제동/차간/점유 한도는 바꾸지 않았다. 지도·정답 자세는 제어 입력에 쓰지 않는다.

**정적 연결 확인(`wiring_check.json`, 9/9 통과):** 진입점 기본 인자가 run1 등록과 같다. `--straight-budget`은 플래그 하나만 덧붙인다. 검사기가 플래그를 받는다. launch 인자 기본값이 `false`다. 노드 파라미터 기본값이 False다. `FollowConfig` 기본값이 OFF다. run1 단독 구성에서는 ON이 허용되고, r13 교통 구성에서는 ON이 `ValueError`로 거부된다. launch 확인 때 `arena_bringup` 공유 경로는 실제 `install/` 경로를 문자열로만 썼다(파일은 후보 사본의 launch를 실행).

## 산출물과 해시

| 파일 | 내용 |
|---|---|
| `candidate.patch` (`4b9ce0aa…`) | r13 기준(`base_sha256.txt`, 12:52 KST 사본)에 대한 전후 패치. 부모 계약 `moving_pass_withdrawal_r13`, 새 계약 r14 |
| `before_r14.zip`, `solo_straight_budget_r14_sha256.json`, `candidate_manifest.json` | 변경 전 바이트·계약·파일별 전후 해시 |
| `apply_live/` | 현재 작업트리 사본에 대한 같은 빌드(아래) |
| `build_candidate.py`, `snapshot.sh` | 사본 고정·후보 생성. 핵심 7개 파일의 후보 바이트가 r13에서 검증한 값(`VERIFIED_AFTER`)과 다르면 실패한다 |
| `regression.sh`, `replay_check.py`, `run_replays.sh`, `summarize.py`, `wiring_check.py` | 회귀·재생·집계·연결 확인 |
| `replay_*.json(l.gz)`, `summary.json`, `regression_*.log`, `targeted_tests.log`, `wiring_check.*` | 결과 |
| `apply_check.log` | r13 기준 패치의 현재 작업트리 적용 점검 **실패 기록**(exit 1). 13:02에 GPT가 `moving_pass_yield_r14` 계약을 추가하며 출처 시험 파일이 바뀐 탓이다. 그래서 출처 계약 부분은 적용 시점에 다시 만든다 |

**현재 작업트리 적용본 `apply_live/` (13:06 KST 사본, 1,172파일):** 같은 빌드에서 부모 계약은 `moving_pass_yield_r14`로 자동 인식되고 새 계약은 `solo_straight_budget_r15`가 되었다. 핵심 7개 파일 후보 바이트는 r13 검증본과 같았다(빌드 검사 통과). `git apply --check` 0(`apply_live/apply_check.log`, 적용 안 함). 이 사본에서 `test_solo_straight_budget.py`·`test_source_provenance_r0_plus_b3.py`·`test_moving_pass.py` 158건 통과(`apply_live/targeted_tests.log`). 패치 `apply_live/candidate.patch`(`e7067d1a…`). GPT가 이후 출처 사슬을 더 늘리면 아래 절차로 다시 만든다.

## 적용 절차(GPT 검토 후)

```bash
# WSL, 저장소 루트. 이 폴더 기준 스크립트. 작업트리는 4단계에서만 바뀐다.
A=artifacts/validation/2026-11-05/solo_straight_refresh_v1
bash $A/snapshot.sh base_apply apply_<시각>                     # 1) 현재 트리 고정
python3 -I -B $A/build_candidate.py --base base_apply --cand cand_apply --out apply_<시각>   # 2) 부모 계약 자동·핵심 바이트 검증
git apply --check $A/apply_<시각>/candidate.patch                # 3) 적용 점검
git apply $A/apply_<시각>/candidate.patch                        # 4) 적용(승인 후)
python3 -m pytest -q tests/test_solo_straight_budget.py tests/test_source_provenance_r0_plus_b3.py   # 5) 계약·반례
```

- 2단계는 `local_follow.py` 등 핵심 파일이 r13 이후 바뀌어 후보 바이트가 검증본과 다르면 실패한다. 그때는 재생·회귀를 다시 해야 한다.
- 적용 뒤 단독 1랩: `python3 scripts/run_current_solo.py <새 경로> --straight-budget` 환경(`turn_preview_v1/environment.sh`)은 run1과 같다. 새 등록·실행 여부는 GPT가 정한다.

## 한계

- 저장 입력 재생은 개루프다. ON 명령의 실제 차체 속도·랩 시간·접촉은 폐루프 1랩으로만 판정한다. 직선 증속 뒤 코너 진입 감속이 늘 수 있다.
- 회귀 실패 3건(기준·후보 동일):
  - `test_bypass_handoff::test_default_launch_and_drive_boundary_unchanged`: 격리 사본에서는 `install/`을 source하지 않아 `arena_description` 패키지를 찾지 못했다(환경).
  - `test_follow_profile::test_snapshot_covers_transitive_internal_imports_including_per_ray_timing`: `parallel_lane_trial`의 `parallel_side_departure` 의존이 스냅숏 목록에 없다.
  - `test_measured_lane_preference_provenance::…`: `local_pursuit.py` 해시 사슬 불일치.

  뒤의 두 건은 현재 기준 트리에서도 같은 결과다. 이 작업이 만든 실패는 아니며, 원인 수정은 하지 않았다.
- 교통 ON은 구성 검사가 거부하므로 재생하지 않았다. 교통 대표는 2건만 봤다.
- 비용은 병렬 재생 중의 WSL 측정이다.
