# 인계 (2026-11-07, 5시간 사용량 경고 utilization .90)

작업: Claude Opus 5.5 → GPT. 남은 프로세스는 없다(gz/launch 0). 새 Gazebo 실행이나 다음 장면은 시작하지 않았다.

## 변경 파일과 계약
- r5 (교통·운영 도구 명시 track): `scripts/validate_g1_traffic.py`, `scripts/official_traffic_course.py`, `scripts/race_ops_eval.py`, `scripts/summarize_motion_lap.py`, 새 `tests/test_traffic_course_track.py`, `config/tests/traffic_course_track_r5_sha256.json`(부모 `piecewise_corner_r4`), before는 `before.zip`.
- r6 (launch 가드): `src/arena_bringup/launch/simulation.launch.py`의 `_autonomy_executable`에서 `track != 'official'`을 `track not in OFFICIAL_2X5_TRACKS`로 바꿨다. 다른 조건은 바꾸지 않았다. `tests/test_bypass_handoff.py`에 두 공식 track 허용, 새 track에서 guard/actor/evidence/mode/reserve 누락 거부, experimental·original 거부 검사를 추가했다. `config/tests/handoff_track_guard_r6_sha256.json`(부모 r5), before는 `r6/before.zip`.
- **마지막 후속 계약은 `handoff_track_guard_r6`이다.** `tests/test_source_provenance_r0_plus_b3.py`의 `M3_SUCCESSORS` 끝에 r5와 r6을 추가했다.
- 빌드는 하지 않았다. 설치된 launch는 src로 연결된 symlink이며 해시가 같다(`r6/install_check.json`).

## 실제 검사 결과
- `r6/guard_check.py`: exit 0. `paced`로 허용되는 경우는 official과 official_v2026_11_06에서 조건이 모두 갖춰졌을 때뿐이다.
- `run_tests.sh r6_scope` (bypass_handoff, traffic_course_track, official_track_v2026_11_06, track_variants, source_provenance): **213 passed, exit 0** (`checks/pytest_r6_scope.*`).
- r5 단계의 관련 회귀 결과(530 passed / 기존 실패 3)는 README에 있다.

## 실행 상태
| 장면 | 상태 | 위치 |
|---|---|---|
| slow_lead 첫 시도 | Haiku 보조가 턴을 끝내 셸이 끊겼다. Gazebo 시작 전이며 판정 대상이 아니다 | `slow_lead/aborted_attempt0/` |
| slow_lead run1 (r5 소스) | launch 가드에 거부됐다(wall 21 s, 주행 없음). 원본 보존 | `slow_lead/run1`, `slow_lead/run1.exit` |
| **slow_lead run2 (r6 소스)** | **완료.** `HARNESS 0 SURFACE 0 INTEGRITY 0 ASSESSMENT 0`, 271 s, 사후 해시 변경 0/867, 남은 프로세스 없음 | `r6/slow_lead/run2`, `assessment_run2.json` |
| straight_moving_lead (r6) | **등록만 했고 시작하지 않았다** | `r6/straight_moving_lead/registered.json` |

slow_lead run2 결과:
- 판정: raw `passed=true`(옛 판정 그대로, 차체 도로 밖 0·간격 포함), 보충 판정 모두 true.
- 센서·단면·출발: B0390 실행 프로필 일치, `wall_offset_m=.505`·`road_half_m=.305` 실제 적용, 실제 초록 0.684 s 뒤 출발.
- 속도·정지: 최고 1.091 m/s, 출발 후 평균 0.335 m/s, 출발 후 0.05 m/s 미만 구간 0 s, 안전 0명령 0회.
- 앞차와의 관계: 최소 간격 1.023 m(swept 하계 1.023), 접촉 0, 앞차 뒤를 유지했고 최대 횡편차 0.09 m.
- 노면: 벗어난 바퀴 최대 0(any/center/full).
- STOP·종료: stop_checks와 shutdown_clean 모두 true.

## 재개 명령 (GPT 판단 후 1회)
```
wsl.exe -e bash artifacts/validation/2026-11-07/traffic_course_migration_v1/r6/run.sh straight_moving_lead
```
- 실행 전 `r6/straight_moving_lead/registered.json` 해시는 `r6/run.py`가 확인한다. 소스가 바뀌었으면 거부된다.
- 출력은 `r6/straight_moving_lead/run2`, `run2.exit`, `assessment_run2.json`이다.
- Haiku 보조에 위임할 때는 `run2.exit`가 생길 때까지 턴을 유지하게 해야 한다(포그라운드 대기 루프 사용).
- README의 r6 실행 결과 절은 아직 쓰지 않았다. 위 표를 근거로 이어 쓴다.
