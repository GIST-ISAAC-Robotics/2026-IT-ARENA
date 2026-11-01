# 낮은 측정 속도 양측 호길이 재획득 인계

5시간 사용량 90% 경고로 Claude 작업을 여기서 마치고 GPT가 이어받는다. 실행 프로세스는 남아 있지 않다. 커밋·푸시·게시는 하지 않았다.

## 결론

구현과 출처 등록은 끝났다. 단독 1회 실행은 **미완주**이고 **후보는 작동하지 않았다.** 후보 검증 통과로 세지 않는다. 같은 마지막 코너(43.473 m)에서 멈췄지만 정지 경로는 진단의 실패와 다르다.

## 변경

| 파일 | 내용 |
|---|---|
| `src/arena_autonomy/arena_autonomy/local_follow.py` | `low_speed_arc_reacquire` 기본 OFF, `path_agrees_arc`(공통 호길이 0.25 m·각 5점·80% 통계 유지), 재획득 분기, `follow_steer_in_place`. 켜려면 `blend_open_speed_by_length` 필요 |
| `src/arena_autonomy/arena_autonomy/local_pursuit.py` | ROS 파라미터 `g1_low_speed_arc_reacquire` |
| `src/arena_bringup/launch/simulation.launch.py` | `g1_open_10kmh_turn15_arc` = turn15 + 옵션 한 키. 교통 미연결 |
| `scripts/validate_basic_autonomy.py` | 프로필 등록 3곳, 평가 전용 `imu_sample`, 궤적 행 `imu` |
| `tests/test_low_speed_arc_reacquire.py` | 신규. 저장 정지 장면은 원시 기록이 없으면 건너뜀 |
| `tests/test_source_provenance_r0_plus_b3.py` | 사슬 끝에 새 계약 추가 |
| `config/tests/low_speed_arc_reacquire_r1_sha256.json` | 부모 `released_rear_sweep_r1`. 현재 바이트와 일치 확인 |

수정 전 바이트는 [before.zip](before.zip), 차이는 [source_change.patch](source_change.patch)다. `follow_config` 헤더 기록은 `asdict`로 자동 연결되며 실행 헤더에서 `true`를 확인했다.

## 회귀와 재생

| 검사 | 결과 | 근거 |
|---|---|---|
| 신규 단위 회귀 | 36 passed | [pytest_unit.log](pytest_unit.log) |
| 관련 회귀 수정 전 | 13 failed, 3477 passed | [pytest_before.log](pytest_before.log) |
| 관련 회귀 수정 후 | 13 failed, 3514 passed. 실패 집합 동일(새 실패 0) | [pytest_after.log](pytest_after.log) |
| 저장 재생 OFF | 증속 2,320·코너 2,327·결합 2,281·rear sweep 3,964회 변경 0 | [replay_saved.json](replay_saved.json) |
| 저장 재생 ON | 증속·코너 변경 0, 결합 정지 전 1,918회 변경 0 | 같은 파일 |

명령은 `bash env.sh`, `checks.sh`, `unit.sh`, `replay_saved.py`다. 전체 회귀는 실행하지 않았다.

## 단독 1회 실행

조건은 [등록 입력](registered.json)대로다. `g1_open_10kmh_turn15_arc`, 2×5 신호·ID45 이동·방지턱 OFF·8.9706:1·C1 순차 10 Hz. 결합 실행 뒤의 다른 기본 OFF 변경도 소스에 포함되며 `changed_since_combined`에 있다. 자동 재실행은 하지 않았다.

| 항목 | 결과 |
|---|---|
| 종료 | 종료코드 1, `주행 정지 상태 지속`, 완주 0랩 |
| 진행 | 43.473 / 48.633 m, 관측 44.401초 |
| 후보 | 헤더 ON, 재획득 0회, 제자리 조향 0회, `activated=false` |
| 속도 | 차체 최고 2.406 m/s(8.66 km/h), 1초 평균 최고 2.308 m/s, 2 m/s 이상 3.801초, 95% 이상 0초 |
| 접촉·노면 | 접촉 표본 0, 벽 바퀴 표본 0, 잔디 바퀴 최대 0(25,530표본) |
| 정체 | 43.43 m 부근 5 cm 이내 6.598초, 평가기 7초 규칙으로 종료 |
| STOP·종료 | `stop_stable=None`, 실패 정리 STOP 확인·차체 정지 True, `shutdown_clean=True` |
| 소스 동일 | `source_unchanged=True` |
| 영상 | 885프레임 20 fps 전부 디코딩, 보정 영상 생성 |
| IMU | 궤적 965행 전부 기록 |

기존 44.193초·8.70 km/h 기록과 결합 실패 원본은 그대로 보존했다. 이번 실행은 랩 시간 비교에 넣지 않는다.

**후처리 예외:** 모터 감사는 종료코드 1이다. `subscription_evidence_present=false` 때문이며, 결합 실패 실행에서도 같은 항목이 실패했다고 기록돼 있다. 감속비는 8.9706, MCU 표본은 13,089개다. `report.signal_evidence`는 실패 보고에 없어 `signal=None`이고, 원시 `run1/signal_sequence.jsonl`은 대조하지 않았다.

**사전 점검 1회 오탐:** 첫 시도는 보조의 대기 명령 문자열을 `pgrep -f`가 잡아 빌드 전에 멈췄다(`preflight_attempt1.log`). 주행이 없었으므로 재실행이 아니다. 점검을 실행 파일 시작 패턴으로 고친 뒤 실행했다.

## 원시 기록에서 본 정지 경위

[failure_timeline.txt](failure_timeline.txt)에서 확인했다. 원인 분석은 끝내지 않았다.

- 42.8 m 부근 차체는 약 0.98 m/s로 감속이 전달됐다. 결합 실패의 1.76 m/s 진입은 재발하지 않았다.
- 45.02초에 경계 공백을 복도 후보로 넘겼고, 45.12–45.22초에 `parallel_hold_partial_observation_outside_trial`로 목표 0이 됐다.
- 45.32초에 `partial_observation`이 `known_obstacle_latched`(`previous_point_in_missing_sweep`)로 잠겼다.
- 정지 뒤 G1 경로는 살아 있었다. 47.02초 기준 새 경로 수락, 관측 경로 2.38 m, 여유 0.84 m, 잠기기 전 목표 0.85 m/s다.
- 소스상 잠금 해제는 `audit.reason=='ok'`인 새 스캔 2장이 필요한데, 정지 장면에 무반환 3~4표본이 계속 남아 약 7초간 풀리지 않았다.

재획득 분기는 기억 없는 상태에서 양측 비교가 실패할 때만 들어가므로 이번 정지에는 관여하지 않았다.

**가설(미검증):** 45.22–45.32초에 측정 속도 −0.08/+0.08 m/s, 차체 약 0.8 m/s였다. 진단에서 본 운동 입력 오차가 잠금의 이전 점 이송에 영향을 줬을 수 있다.

## 다음 판단

- 후보 A의 폐루프 재출발은 아직 검증되지 않았다.
- 같은 코너에서 재현 2종이 나왔다. 다음 단계는 `known_obstacle_latched` 잠금과 무반환 3~4표본의 관계를 저장 입력으로 먼저 보는 것이 좋다.
- 반복 실행, 잠금 조건 변경, 후보 B 착수는 하지 않았다. 판정선은 바꾸지 않았다.
- `docs/PROJECT_CONTEXT.md`, `docs/DEVELOPMENT_PLAN.md`, 활동일지는 수정하지 않았다. 기술 보고서는 이 인계 문서로 대신했다.

## 재현 명령

```bash
A="artifacts/validation/2026-11-01/low_speed_arc_reacquire_v1"
bash "$A/checks.sh" pytest_again.log
bash "$A/env.sh" python3 "$A/replay_saved.py"
```

`run_once.sh`는 `run1`이 있으면 멈춘다. 새 실행은 별도 폴더와 등록이 필요하다. WSL 스크래치 사본은 `~/arena_scratch/qc_diag`에 있다.
