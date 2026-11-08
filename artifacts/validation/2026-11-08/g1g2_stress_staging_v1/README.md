# 새 코스 g1g2 C1 주입 CLI 허용 — staging 패치와 표적 시험

2026-11-08. 직전 정적 감사 [newcourse_sensor_stress_audit_v1](../newcourse_sensor_stress_audit_v1/README.md)의 두 거부 지점(`scripts/validate_g1_traffic.py`의 `--lidar-test-profile requires G1/M3` 검사와 등록 G1/G2 비교의 overlay 금지)에 대해 적용 후보를 이 폴더에만 만들었다. 실제 `src/ scripts/ tests/ config/ docs/`와 git index는 바꾸지 않았다. Gazebo·colcon·패키지 설치도 하지 않았다. 작업은 Claude(Opus 5.5)가 했다.

## 1. 파일

| 경로 | 내용 |
|---|---|
| `original/validate_g1_traffic.py` | 작업 시작 시 `scripts/validate_g1_traffic.py`를 바이트 그대로 복사한 사본(CRLF) |
| `patched/validate_g1_traffic.py` | 적용 후보(CRLF 유지, 변경은 아래 diff뿐) |
| `validate_g1_traffic.patch` | 적용 후보 unified diff(LF로 정리, `a/ b/scripts/validate_g1_traffic.py`) |
| `tests/test_g1g2_lidar_stress_cli.py` | 새 표적 시험 24건 |
| `run_tests.sh` | WSL pytest 실행기(`real`·`patched`·`mutant_*` 대상 선택) |
| `checks/` | 실행 로그·종료 코드, 변이 사본 2개, 회귀 파일 목록 |
| `MANIFEST.json` | 해시 목록 |

## 2. 해시

| 파일 | sha256 (작업 트리 바이트, CRLF) | LF 정규화 |
|---|---|---|
| `scripts/validate_g1_traffic.py` 원본(시작·종료 시 동일) | `e3e7ae73ee58fc5d952386b4060a33caa163928c4bd90d7c967c83351dd9b494` | `6c54baf10154e5ada0be03cc48f0fcc5c2120d7e1fddaee5323d682a88656a39` |
| `patched/validate_g1_traffic.py` | `77f97aef810e176bd96fc46a14c9365a9266fc1a03a2d57dd6ff78ae18447dce` | `a69725a40cd053081a0718a4ab6bec5efd3116bf8954d0baffb88e52492ad49e` |
| `config/tests/lidar_uniform30_delay50_jitter20.json`(변경 없음) | `df09b7e1af205ff63797051a341085d76f79ac1755641f49a12fb03357ec9efb` | — |

원본 해시 `e3e7ae…`는 `config/tests/moving_pass_faster_r31_sha256.json`에 `validate_g1_traffic.py`의 `after`로 등록되어 있다. 기존 계약은 작업 트리(CRLF) 바이트 해시를 쓰므로 새 계약의 `before`/`after`도 같은 기준이어야 한다. 기준 커밋은 `79e273dc`이다. 다만 이 파일은 커밋 이후 작업 트리에서 수정된 상태이며, HEAD blob 해시와는 다르다.

## 3. 변경 내용(적용 후보)

```diff
@@ -44,6 +44,9 @@
 CONFIG = REPO/'config/tests/g1_traffic_v1.json'
+# 새 코스 g1g2 C1 주입은 이 등록 프로필(경로·ImpairmentConfig 전체 값) 하나만 허용한다.
+G1G2_LIDAR_PROFILE = REPO/'config/tests/lidar_uniform30_delay50_jitter20.json'
+G1G2_LIDAR_REQUESTED = dict(mode='uniform', amplitude_m=0.03, bias_m=0., delay_s=0.05, jitter_s=0.02, seed=23)
@@ -529,12 +532,18 @@
-    if args.lidar_test_profile and (args.mode not in ('g1', 'm3') or not args.record_follow_decisions):
-        parser.error('--lidar-test-profile requires G1/M3 with decision recording')
+    if args.lidar_test_profile and (args.mode not in ('g1', 'm3', 'g1g2') or not args.record_follow_decisions):
+        parser.error('--lidar-test-profile requires G1/M3/G1G2 with decision recording')
+    g1g2_lidar_stress = bool(args.lidar_test_profile) and args.mode == 'g1g2'
+    if g1g2_lidar_stress and (args.track != 'official_v2026_11_06' or not args.official_signal_course
+                              or args.team_signal_course or args.handoff_reserve != 'paced' or args.seed != 0):
+        parser.error('G1/G2 lidar stress requires recorded paced official_v2026_11_06 signal course, seed 0')
     ...
     lidar_profile, lidar_requested = load_test_profile(args.lidar_test_profile, REPO)
+    if g1g2_lidar_stress and (lidar_profile != G1G2_LIDAR_PROFILE or lidar_requested != G1G2_LIDAR_REQUESTED):
+        parser.error('G1/G2 lidar stress accepts only config/tests/lidar_uniform30_delay50_jitter20.json')
@@ -584,7 +593,7 @@
-            args.race_operation_test or lidar_profile or motion_profile or args.partial_observation_policy != 'strict'
+            args.race_operation_test or (lidar_profile and not g1g2_lidar_stress) or motion_profile or args.partial_observation_policy != 'strict'
```

설계 요지:

- **허용 조합(이번 새 조합 한정):** `--mode g1g2` + `--record-follow-decisions` + `--official-signal-course` + `--track official_v2026_11_06` + `--handoff-reserve paced` + `--seed 0` + 등록 프로필 1개. 등록 프로필은 `load_test_profile`이 `resolve()`한 실제 경로가 `REPO/config/tests/lidar_uniform30_delay50_jitter20.json`과 같아야 한다. 또한 읽은 `ImpairmentConfig` 전체 값(`bias_m` 0, `seed` 23 포함)도 등록값과 같아야 한다. 따라서 같은 내용을 다른 경로에 복사한 파일, 저장소 밖 파일(기존 `relative_to` 검사), 값이 바뀐 등록 경로 파일은 모두 거부된다. `..`이나 절대경로로 같은 실제 파일을 가리키는 경우는 같은 파일이므로 허용된다. 이때 보고서에는 기존 코드대로 정규화된 상대경로가 남는다.
- **유지:** g1/m3의 기존 허용과 거부 문구 외의 동작. 모드 검사로 g1g2에서 motion(IMU/엔코더) 프로필을 거부하는 동작과 lidar+motion 동시 사용 금지(534행). merge와 센서 주입의 동시 사용 금지(567행). 등록 비교의 다른 overlay 금지(seed, motion, partial, zero-speed, physics-step, corridor, parallel·pass-release·guard·recovery, boundary-evidence, occupancy-timing). 10 Hz sequential 검사(609행). 주입 후 `audit_injection`/`require_injection_evidence`는 모드와 무관하게 이미 실행된다(1335–1347행). 자율주행·보호층·가드·기어·센서 코드는 바꾸지 않았다.
- `official_traffic_course.inputs`도 이미 paced·seed 0·기록 조건을 요구한다. 그래도 새 검사는 프로필을 읽기 전 parser 단계에서 같은 조건을 명시한다. 그래서 이 조합의 허용 범위를 패치 한 곳에서 읽을 수 있다.
- 기존 거부 문구 `G1/M3`는 `G1/M3/G1G2`로 바뀐다. 이 문구를 대조하는 기존 시험은 없다(`tests/`에서 문자열을 찾아 확인했다).

## 4. 새 표적 시험(`tests/test_g1g2_lidar_stress_cli.py`, 24건)

시뮬레이터는 띄우지 않는다. `artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1/two_actor_course_lap/registered.json`의 r27 인자를 그대로 읽고 `OUTPUT`만 생성 금지 경로로 바꾼다. 그다음 `check_route`를 sentinel로 바꿔 두 거부 지점과 10 Hz sequential 검사, 출력 경로 검사를 모두 지난 단계에서 멈춘다. `load_centerline`은 원래 함수를 그대로 호출하면서 월드 경로만 기록한다. 각 시험이 끝나면 출력 폴더가 만들어지지 않았는지 확인한다.

- `test_module_identity`: `harness.REPO`가 실제 저장소 루트인지 확인한다. 또한 `harness.__file__`이 실행기가 지정한 대상 파일(`HARNESS_EXPECTED_FILE`)인지 확인한다.
- 대조 1건: 무주입 r27 인자가 현 트리에서도 sentinel에 도달하고, 월드가 `official_v2026_11_06`인지 확인한다.
- 양성 3건: r27 + `--lidar-test-profile`을 상대경로, 절대경로, `..` 포함 경로로 각각 주고 모두 sentinel에 도달하는지 확인한다. 이때 로드 결과(정규 경로와 값)를 검사한다.
- 음성 parser 12건: 이유 문구까지 확인한다. 대상은 기록 없음, 이전 공식 코스(`--track` 생략), 데모 코스, 11/1 fixture 코스, paced 아님, reserve 없음, seed 1, motion 동시, 다른 프로필 2개(`lidar_uniform30.json`, `lidar_delay80.json`), per-ray overlay, fragmented overlay다.
- 음성 기타 7건: 같은 내용의 저장소 내부 사본(종료 시 삭제), 저장소 밖 사본, 등록 경로이지만 값이 다른 경우(seed 24로 바꾼 로더)를 확인한다. 등록 비교 overlay(`zero-speed`, `physics-step`, `parallel-hold`)는 기존 ValueError 문구로 거부되어야 한다. merge 프로필을 강제한 경우에는 `merge trial requires plain G1` 문구로 거부되어야 한다.

부정 확인(변이 대조): 두 변이로 이 시험이 실제로 각 변경을 검출하는지 확인했다.
- `checks/mutant_gate1_only`: 두 번째 거부 지점을 원래대로 되돌린 사본이다. 양성 3건이 실패했다. 따라서 첫 지점만 통과하는 것으로는 이 시험을 통과하지 못한다.
- `checks/mutant_no_restrict`: 새 제한 검사 두 개를 제거한 사본(감사 초안 형태)이다. 코스·paced·seed·프로필 음성 10건이 실패했다.

## 5. 실행 방법과 결과

실행 위치는 WSL Ubuntu(ROS Jazzy)이며 Git Bash에서 다음 형식으로 실행했다.

```bash
MSYS_NO_PATHCONV=1 wsl.exe -e bash -lc "bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-08/g1g2_stress_staging_v1/run_tests.sh' <label> <real|patched|mutant_*> <files...>"
```

`patched`이면 `patched/`를 PYTHONPATH 맨 앞에 둔다. 그래서 `import validate_g1_traffic`만 staged 사본을 쓰고, 나머지 모듈(`validate_basic_autonomy`의 `REPO` 포함)은 실제 `scripts/`에서 온다. `validate_g1_traffic.py`에는 `__file__` 사용처가 없고 `tests/`에는 conftest가 없음을 확인했다. `python3 -B`와 `-p no:cacheprovider`를 써서 pyc와 캐시를 쓰지 않는다.

| label/대상 | 파일 | 결과 | 근거 |
|---|---|---|---|
| target/patched | 표적 1개 | **24 passed**, exit 0 | `checks/pytest_target_patched.{log,exit}` |
| target/real | 표적 1개 | 16 passed, 8 failed, exit 1. 실패 8건은 모두 원본에서 예상된 결과다. 양성 3건은 원본 532행에서 거부되었다. 저장소 밖, overlay 3건, merge는 더 이른 532행에서 다른 예외로 거부되었다 | `checks/pytest_target_real.*` |
| target/mutant_gate1_only | 표적 1개 | 21 passed, 3 failed(양성 3건) | `checks/pytest_target_mutant_gate1_only.*` |
| target/mutant_no_restrict | 표적 1개 | 14 passed, 10 failed(새 제한 음성) | `checks/pytest_target_mutant_no_restrict.*` |
| regression/patched | 표적 + `tests/` 15개(`checks/regression_files.txt`) | 321 passed, 2 failed, exit 1 | `checks/pytest_regression_patched.*` |
| regression/real | 같은 목록 | 313 passed, 10 failed, exit 1(표적 8 + 아래 2) | `checks/pytest_regression_real.*` |

`tests/` 299건의 결과는 실제 모듈과 staged 모듈에서 **항목별로 동일**하다(297 passed, 2 failed, 목록 diff 없음). 두 실패는 패치와 무관한 기존 실패다.
- `tests/test_physics_step_option.py::test_contract_records_evaluation_script_changes`: 이 시험의 계약 체인 목록이 `fresh_confirmation_r2`에서 끝난다. 그래서 그 체인의 마지막 `after` `61917b…`와 현재 원본 `e3e7ae…`(r31 등록)가 다르다. 출처 체인 목록 갱신이 밀려 있는 기존 상태다.
- `tests/test_g1_side_occlusion.py::test_official_post_rejects_only_the_left_side_scene_on_the_new_world`: `DID NOT RAISE ValueError('not wholly off-road')`. `validate_g1_traffic` 패치와 무관하며 원인은 조사하지 않았다.

## 6. 다음 단계(적용 측 참고)

- 패치를 적용한 뒤 `scripts/validate_g1_traffic.py`의 작업 트리 해시가 `77f97aef…`(CRLF)가 되는지 확인한다. `validate_g1_traffic.patch`는 LF로 정리되어 있으므로 CRLF 작업 트리에는 `git apply --ignore-whitespace`를 쓰거나 `patched/` 사본을 복사한다. 그동안 원본이 `e3e7ae…`에서 바뀌었다면 패치를 다시 기준에 맞춰야 한다.
- 새 출처 계약의 `before`는 `e3e7ae…`, `after`는 `77f97aef…`이다. 현재 존재하는 최신 계약은 r32(`moving_pass_wall_mask`)와 r33(`stopped_entry_centering`)이다. parent 선택과 체인 목록 갱신 대상은 적용 측에서 정한다.
- 표적 시험을 `tests/`로 옮길 때는 `REPO = Path(__file__).resolve().parents[1]`로 바꾼다. `STAGING`·`NEVER_CREATED`·사본 경로는 `REPO/'artifacts'` 아래의 임시 경로로 바꾼다. 출력 경로 검사가 `artifacts/` 아래를 요구하기 때문이다. `test_module_identity`와 `HARNESS_EXPECTED_FILE`은 staging 전용이므로 제거하거나 `REPO` 검사만 남긴다.

## 7. 한계

- CLI 사전 단계(`check_route` 직전)까지만 검증했다. launch 인자 전달, `rotating_lidar` 주입, 사후 `audit_injection`이 g1g2 실행에서 실제로 동작하는지는 실행해 보지 않았다(감사 README의 정적 판단만 있다).
- 표적 시험 일부는 monkeypatch(`check_route`, `load_test_profile` 값 변경, `merge_profile_for_case` 강제)에 의존한다. merge 음성은 실제 merge 사례 설정이 아니라 강제 반환값으로 시험했다.
- r27 인자의 무주입 대조가 현 트리에서 sentinel까지 간다는 것만 확인했다. 현 소스로 r27 주행이 재현된다는 뜻은 아니다(감사 README의 기준 표류 12개 파일 참조).
- 다른 작업이 진행 중이어서 `src/ tests/`의 일부 파일은 이번 작업 중에도 바뀌고 있었다. 이번 작업의 쓰기는 이 폴더로 한정했다. `scripts/validate_g1_traffic.py`의 해시가 시작과 끝에 같고 `git diff --cached`가 비어 있음을 확인했다.
