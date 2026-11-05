# 단독 직선 속도 후보 폐루프 1랩 v1 (`--straight-budget`)

- 날짜: 2026-11-05 (KST). 주행 시각 13:17:36–13:26:13 KST(`run1_preflight.log`), 실시간 517 s.
- 역할: 패치 적용·회귀·등록·평가·보고서는 Claude Opus 5.5가 맡았다. 시뮬레이션 실행·대기·회수는 Claude Haiku 5.5 보조(별칭 `haiku`)가 포그라운드로 1회 수행했다(`run.sh` 1회, 시간 초과 없음).
- 적용: [solo_straight_refresh_v1](../solo_straight_refresh_v1/README.md)의 `apply_live/candidate.patch`(`e7067d1a…`)를 적용했다. 적용 직전 대상 9개 파일의 바이트가 적용 전 해시와 같았고 `git apply --check`는 0이었다. 적용 뒤 해시도 매니페스트와 같았다(`apply.log`). 계약은 `config/tests/solo_straight_budget_r15_sha256.json`(부모 `moving_pass_yield_r14`)이다. 후보 A와 녹화 카메라 변경은 포함하지 않았다. 플래그 기본값은 OFF다.
- 조건: 오늘 [OFF 기준 run1](../solo_jr60p20_baseline_v1/README.md)과 같다(JR60P20·Ø20·ID49 0.140·v2026.11.01 팀 월드·물성·신호·STOP·`--video`). 다른 점은 진입점 `--straight-budget` 하나다. 등록 인자는 기준 인자 끝에 `--g1-solo-straight-budget`이 붙은 것뿐이다(`registered.json`).

## 결론

| 항목 | 이번(ON) | 오늘 기준 run1(OFF) |
|---|---:|---:|
| 검사기 판정 | `passed=true`, 1랩 | 같음 |
| **랩(첫 초록 표본 → 결승)** | **31.850 s** | 32.949 s |
| 지면 최고 / 최고 1초 평균 | **2.854 / 2.758 m/s** | 2.393 / 2.315 m/s |
| 10 km/h(2.778 m/s) 이상 | 0.663 s(처음 10.73 s, 진행 약 3.5 m 부근) | 0 s |
| 10 km/h의 95%(2.639 m/s) 이상 | 0.812 s, 2.28 m | 0 s |
| 2.0 m/s 이상 총 / 최장 연속 | 6.455 / 2.343 s | 5.316 / 2.260 s |
| 계획·`/drive/safe` 최고 | 2.778 / 2.778 m/s | 2.353 / 2.376 m/s |
| 출발(RGB 확인 → 0.1 m/s) | 8.255 s 확인, 초록 뒤 0.552 s 이동. 부정 출발 없음 | 0.552 s |
| 계획 0명령(출발 후) | 0.34 s(0.16 s, 모두 43.41–43.6 m) | 0.32 s(0.14 s) |
| **실제 지면 정지(<0.05 m/s ≥0.1 s)** | **1회: 43.61 m, 0.138 s(최저 0.001 m/s)** | 0회(최저 0.094 m/s) |
| 42–44 m 통과 | 2.329 s(평균 0.859 m/s) | 2.411 s |
| 접촉·노면 | 충돌 0·정지 중 0. 21,115표본 중 차체/조향 외곽 노면 밖 0, 바퀴 이탈 0, 벽 바퀴 0 | 0 |
| STOP·종료 | `stop_stable`, 조밀 자세 통과(최고 0.0088 m/s), MCU `ESTOP_LATCHED`/`software_stop`, 자식 15/15 정상, 잔여 PID 없음, 주행 뒤 시뮬 프로세스 없음 | 같음 |
| 입력·무결성 | 센서 전용 입력(제어·보호), 라이다 10 Hz·`range_max` 12 m, 등록 834개 해시 실행 전후 변경 0, `source_unchanged=true`, 모터 감사 9/9(28 count·8.970588·0.0325 m), `wsl_nvidia` | 같음 |

**해석:** 직선 속도 후보를 켜자 실제 차체가 처음으로 10 km/h를 넘었다. 랩은 1.099 s 짧아졌다(1회끼리의 비교이며 반복 분산은 모른다). 진행 구간별 차이(`progress_bins_now_minus_prior_s`)를 보면 26–30 m에서 −0.64 s, 2–8 m에서 −0.30 s, 18–20 m에서 −0.23 s가 줄었다. 0–2 m는 +0.18 s였다.

후보가 실제로 증속한 판단은 77호출이다. 위치는 1.2–46.2 m에 걸쳐 있고 42–44 m에서는 0호출이다. 최대 증가는 0.604 m/s다(`run1_analysis.json` `straight_budget`).

**주의 — 43.6 m의 실제 정지:** 원인 구조는 [diagnosis_v1](../solo_jr60p20_baseline_v1/diagnosis_v1/README.md)과 같다. 43.41 m에서 근거 전환 재개 대기 1호출이 있었고, 43.59 m에서 `follow_forward_gap_too_wide` 0.04 s와 재개 대기 0.10 s가 이어졌다. 기준 run1에서는 최저 0.094 m/s의 거의 정지였는데, 이번에는 0.138 s의 실제 정지가 되었다. 이 구간에서 후보는 작동하지 않았다(증속 0호출). 진입 속도·스캔 위상 차이 때문으로 추정하며, 1회라 원인을 확정하지 않는다. 그래도 42–44 m 통과 시간은 2.329 s로 기준보다 0.08 s 짧았다.

## 적용 뒤 확인

| 확인 | 결과 | 근거 |
|---|---|---|
| 설치 연결 | 설치 환경(`turn_preview_v1/environment.sh`)에서 `solo_straight_budget`·`local_follow`는 작업트리 `src`로, 설치된 `g1_follow.launch.py`는 작업트리 파일로 연결된다. 플래그 존재, 기본 OFF. 빌드 산출물 변경 없음(Python·launch 심볼릭 설치) | `checks_install.log` |
| 영향 범위 회귀(48개 파일, 실제 설치 환경) | **653 통과 / 2 실패.** 실패 2건은 적용 전 기준 트리에도 있던 기존 실패다: `test_follow_profile::…per_ray_timing`(스냅숏 목록에 `parallel_side_departure` 없음), `test_measured_lane_preference_provenance::…`(`local_pursuit.py` 해시 사슬 불일치). 이번 적용으로 생긴 새 실패는 0이다. 격리 사본에서 나던 환경 실패(`arena_description` 미발견)는 실제 설치 환경에서 사라졌다 | `checks_regression.log`, `checks.exit`, refresh `regression_base.log` |
| 출처·교통 보존 | r15 계약을 포함한 출처 시험과 `test_moving_pass.py`가 위 회귀 안에서 통과했다. 교통 OFF 재생(r13 이동 추월·이동 앞차 기준)은 적용 전 격리 검증에서 기록과 일치했다 | refresh `summary.json` |

## 영상

`run1/local_pursuit_third_person_drive.mp4`: 651프레임 기록·디코드 통과, 시뮬 8.25–40.75 s로 결승(39.92 s)을 포함한다. 첫 화면(`third_person_first_frame.png`)에서 그리드 위 차량, 전방 노면, 후면 ID49, 지주·Ø20 배선 예약 외곽 표시를 확인했다. 코너 앞 구간 프레임은 마감 시간 때문에 확인하지 않았다. 영상은 로컬에만 보존한다(이 폴더 `.gitignore`).

## 한계

- 1회 결과다. 랩 단축의 반복성, 43.6 m 실제 정지의 재발 빈도, 실차는 확인하지 않았다.
- 지면 속도는 평가 정답 기반이다. 판정은 시뮬레이션 시각 기준이다.
- 이번 적용으로 공용 `src/`·`scripts/`·`tests/`·`config/tests/`가 바뀌었다(커밋하지 않음). 기본 OFF라 다른 실행 경로의 동작은 같다는 것을 재생·회귀로 확인했으나, 폐루프 교통 실행은 하지 않았다.

## 파일

| 파일 | 내용 |
|---|---|
| `apply.log` | 적용 시각과 적용 뒤 해시 검사 |
| `checks.sh` → `checks_install.log`, `checks_regression.log`, `checks.exit` | 설치 연결·영향 범위 회귀 |
| `register.py` → `registered.json`, `registered_inputs.sha256`; `input_list.txt`, `git_state_registered.txt`, `worktree_diff.patch`, `registered_untracked_sources.zip` | 사전 등록(실행 전) |
| `verify.py` → `run1_verify_before.log`, `run1_verify_after.log` | 실행 전후 입력 해시·인자 |
| `run.sh`, `post.sh` → `run1_sim.exit`, `run1.exit`, `run1_*.log`, `run1_metrics.json`, `run1_surface.json`, `run1_motor_audit.json` | 1회 실행·회수 |
| `analyze.py` → `run1_analysis.json` | 출발·속도·0명령·42–44 m·후보 사유, 기준 run1 재계산 비교 |
| `run1/` | 검사기 산출물. 원시 기록은 루트 `.gitignore`, 영상은 이 폴더 `.gitignore`로 로컬 보존 |
