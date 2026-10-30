# 인계 — 후방 소실·재접근 묶음 (2026-10-29)

Claude 5시간 사용량 경고(five_hour 0.91, 주간 0.14)로 사용자 인계 규칙에 따라 중단한다. **이 할당은 완료되지 않았다.** 완료한 범위와 남은 범위를 아래에 나눈다. 결과 해설은 [README](README.md), 장면 등록과 변경 기록은 [SCENARIO_PLAN](SCENARIO_PLAN.md)에 있다.

## 1. 현재 코드 상태(커밋 없음)

| 파일 | 변경 | 상태 |
|---|---|---|
| `src/arena_autonomy/arena_autonomy/rear_lane_keep_trial.py` | 새 선택형 후보 `RearLaneKeepSelector`(SuspendedPassageSelector 상속), `LaneKeepConfig(hold_s=2.0)` | 작성 완료 |
| `src/arena_autonomy/arena_autonomy/local_follow.py` (CRLF 보존) | `FollowConfig.rear_lane_keep_trial`(기본 False, `traffic_path_recovery_trial` 필요), 선택기 연결 | 완료 |
| `src/arena_autonomy/arena_autonomy/local_pursuit.py` (CRLF 보존) | ROS 매개변수 `g1_rear_lane_keep_trial`(기본 false) | 완료. launch 인자는 추가하지 않음(G2 옵션과 같이 SetParameter로 지정) |
| `scripts/{validate_pipeline_faults_ros,prepare_jetson_bundle,profile_follow_trace,validate_g1_traffic}.py` | 모듈 보존/이관 목록에 `rear_lane_keep_trial` 추가 | 완료(`validate_g1_traffic.py` CRLF 보존) |
| `config/tests/rear_lane_keep_r1_sha256.json` | 후속 출처 계약(부모 `g2_handoff_manifest_r1_sha256.json`, 이전 바이트 `before_rear_lane_keep.zip`, 생성 `prepare_contract.py`) | 완료. 위 6파일의 현재 바이트 기준 |
| `tests/test_rear_lane_keep_trial.py`, `tests/test_rear_lane_keep_provenance.py` | 후보 반례 25건, 계약 검사 1건 | 완료 |
| `tests/{test_source_provenance_r0_plus_b3,test_corner_follow,test_g1_dense_stop,test_physics_step_option,source_contract_helpers}.py` | 계약 사슬 목록에 새 계약 추가(5파일 6곳) | 완료 |
| `.gitignore` | 이 폴더의 호출별 `*_trace.jsonl`·`*_records.json` 제외(폴더 약 1.2 GB 중 Git 대상 86파일) | 완료 |

계약 생성 뒤 위 6개 추적 파일을 다시 고치면 `prepare_contract.py`는 기존 파일이 있으면 중단하므로, 계약 JSON과 ZIP을 지우고 다시 만들어야 한다(아직 게시 전이므로 과거 증거 덮어쓰기가 아님).

## 2. 선택 후보와 판단

- 채택 후보: `rear_lane_keep_trial` **기본 OFF·운영 미채택**. 명목 합성 B에서 재접근 옆차 간격 0.05 → 0.14 m, 양보 정지 1.42/1.52 → 0.2 s. A·C·D·E·F는 OFF와 같음. 비용은 해제 뒤 2 s 동안 0.35 m/s 상한(사후 진단 G에서 8 s 진행 4.97 → 3.66 m). 복합 오차 B는 일부만 개선(간격 0.045 → 0.121 m, 정체 1.52 → 1.34 s).
- 현재 코드(OFF)의 판단: 순수 후방 미관측으로 멈추지 않음(A/C/D/E 해제·진행). 약점은 해제 직후 사라진 쪽으로의 약 9.6 cm 중앙 복귀.
- 채택 여부·2 s 시험값·0.35 m/s 비용 수용은 GPT/사용자 검토 대상이다.

## 3. 실패·반례와 원인(보존)

| 기록 | 원인 | 처리 |
|---|---|---|
| `invalid_current1_far_wall` | 합성 세계 끝벽 17 m가 C1 12 m 밖 → 정면 무반환 → 부분 관측 대기로 출발 안 함 | 평가기 세계 수정(끝벽 9 m), 무효 실행 보존 |
| `sweep1` y0.13/0.15/0.17 | 횡간격 7–11 cm 옆차 옆에서 평행 후보 스윕 간격 0(점별 팽창 0.10–0.15 m > 간격) → 자차 정지·양보, 장면 미성립. Gazebo `m3_openframe_v1/drive_left` 9.16–10.9 s에도 같은 정지 | v2 등록에서 옆차 0.20 m로 변경. 기존 한계로 보고(미해결) |
| `current2` F | 옆차가 먼저 지나가 “뒤 옆 관측 유지” 미성립, 성립 검사 코드가 문장보다 약함 | v3 F 수정·검사 강화 |
| `current2` E 재생 13 불일치 | 평가기의 입력 정지 대체 출력에 `mode` 필드 누락 | 평가기 수정, 이후 재생 일치 |
| `lane1` D 6.74 s 정체, B 앞 통과 뒤 보류 | 정지 상자 12 cm 옆 차선 고정 시 팽창 주기 반복 정지(기존 한계 노출) / 고정 `rear` 기록 사용 | 소실 방향·반대쪽 근접 반환 조건 추가 |
| `lane2` D 정체 지속 | 평행/G1 둘 다 같은 위치에서 막혀 속도 비교로는 못 잡음 | 반대쪽 근접 반환 조건(`lane3`) |
| `robust1_on` B 미적용 | 1 cm 허용 아래의 첫 복귀 조향이 다음 평행 후보를 1 cm 횡변위 검사로 막음 | 기준·현재 대비 증가 즉시 적용 |
| `robust2_on` B 조기 종료 | 앞차 가장자리를 반대쪽 근접 물체로 셈 / 반대 회전 뒤 돌출부 흔들림(최대 1.25 mm) 접근 판정 | 통로 안쪽 반환과 같은 종방향 물체 제외, 앞 모서리 기준(`lane5`, `robust3_on`) |
| `robust3_off`/`robust3_on` 오른쪽 A/B | 복합 오차 오른쪽 대칭에서 자차가 옆차보다 앞서지 못함(정체 2.9 s), 장면 미성립 | 원인 미분리 |

## 4. 실제로 수행한 검사(숫자)

- 합성 폐루프: 기하 검사 3회(`geometry1–3`), 현재 코드 `current2`·`current3` 각 9장면, 후보 `lane1–5`, 복합 오차 `robust1–3`, 빠른 앞차 `fast1–3`, 진단 sweep 12회. 각 실행의 판단 자체 재생은 `summary.json`의 `self_replay`(최종 판 모두 통과).
- 단위: `unit1`(16 실패, 주입 view 결함), `unit2` 23 통과, `unit3`(1 실패, 기준 변경 뒤 기대값), `unit4` 25 통과.
- 관련 회귀 `checks1_tests.*`: 24개 파일 **498 통과**, exit 0(출처 사슬·보존 목록·선택기·G2 연결 포함). 전체 6,500여 건 검사는 다시 실행하지 않았다.
- 빌드 `checks1_build.*` exit 0. 설치 확인 `checks1_installed.*`는 스크립트 결함으로 미실행(7절).
- Gazebo/ROS 주행: **실행하지 않음.**

## 5. 실행 담당과 프로세스

- 관련 회귀·빌드는 Claude Code `Agent`의 별칭 `haiku` 실행 담당에게 위임했다(모델 오류 없음). 합성 비교는 짧은 분석 실행이라 직접 실행했다.
- 위임 실행 `checks1.sh`의 마지막 단계(빌드·설치 확인)가 인계 시점에 진행 중일 수 있다. 결과 파일은 `checks1_build.exit`, `checks1_installed.exit/.log`. 잔류 확인 명령:
  `MSYS_NO_PATHCONV=1 wsl.exe -e bash -lc 'ps -eo pid,cmd | grep -E "pytest|colcon|gz sim|ros2|probe_rear" | grep -v grep || echo no_residual'`

## 6. 다음 정확한 순서

1. 설치 모듈 확인만 새 스크립트로 수행한다(빌드는 exit 0). `installed_matches_source` 세 항목 true와 `default_off` true를 확인한다.
2. 대표 Gazebo(미실행): `m3_openframe_v1/run_onroad.py`를 이 폴더로 복사해 옆차를 기존 상자로 되돌리고(`transform_actor` 제거), overlay를 B(`start_center_gap_m` 0, `lateral_offset_m` 0.20, 일정 0–4.5 s 0.15·4.5–12 s 0.60 m/s, `base_case` `slow_lead`)로 둔다. 0.20 m는 기존 `side_road_straight`/`side_grass_straight` 검사에 맞지 않으므로 어댑터에서 도로+잔디 20 cm 회랑 안·벽 비접촉 경로 검사를 추가하고 diff를 남긴다. ON은 `settings`에 `g1_rear_lane_keep_trial=True`를 더하고 `verify_runtime` 헤더에서 `rear_lane_keep_trial` 확인. OFF/ON 각 1회, 방지턱 OFF·공식 v2 신호·뒤 30 cm 카메라 유지.
3. 판정은 SCENARIO_PLAN 5절 그대로(진행·정체·접촉·노면·STOP·정상 종료·입력 무결성·실제 전이·동일 재생 분리).
4. 별도 미해결: 7–11 cm 병렬 정지, 정지 물체 옆 10–15 cm 팽창 주기 정지, 복합 오차 오른쪽 대칭 미성립, 평행 후보 0.35 m/s 비용.

## 7. 인계 시점 확인값

- `checks1_tests.exit`=0: 24개 파일 **498 통과**(실패 0).
- `checks1_build.exit`=0: `colcon build --packages-select arena_autonomy` 완료(stderr는 byte-compile 비활성·underlay 경고).
- **설치 모듈 확인은 실행되지 않았다.** `checks1.sh`가 빌드 뒤 두 번째 `source install/local_setup.bash`를 `set -u` 상태에서 실행해 `COLCON_CURRENT_PREFIX: unbound variable`로 종료했다(스크립트 결함, `checks1_installed.*` 없음). 실행 담당은 Claude Haiku 5.5(별칭 `haiku`, 자체 보고 `claude-haiku-5-5`)였고 모델 오류는 없었다. 잔류 프로세스 확인 결과 `no_residual`.
- 인계 규칙에 따라 다시 실행하지 않았다. 다음 담당은 두 번째 source를 `set +u; …; set -u`로 감싼 새 이름의 스크립트로 설치 확인만 수행하면 된다(기존 `checks1_*` 결과는 보존).
