# observed_measured_v1 Gazebo 제한 1회 — 측정 1.0 m/s 초과에서도 차선 유지, 간격 50→140 mm

2026-10-30. [사전 등록](DRIVE_PLAN.md)대로 10/29 v1과 같은 12초 후방 재접근 장면을 새 명시 정책 `observed_measured_v1`로 **1회** 실행했다. 등록 관문은 모두 통과했고, 이 1회에서 최소 옆차 간격은 139.84 mm로 v3의 50.34 mm보다 커졌다. 원래 150 mm 측방 판정은 과거 비교선으로 여전히 **실패**다. 반복성·전체 랩·오차 조합·독립 상대차·실차 결과는 아니며 기본값(OFF/`timed`)은 바꾸지 않았다.

## 결과

| 항목 | 이번 `observed_measured_v1` | v3 `observed` (10/29) | 기준 OFF (10/29 v2) |
|---|---|---|---|
| 진행 / 앞차 이동 중 최장 정체 | 4.73951 m / 0.711 s | 4.63952 m / 1.013 s | 4.67951 m / 1.008 s |
| 최소 옆차 간격 (표본 / 연속 하한) | **139.84 / 139.83 mm** | 50.343 / 50.342 mm | 50.156 mm |
| 중앙 쪽 최대 횡이동 | **10.78 mm** | 97.472 mm | — |
| 접촉 이벤트 / 표본 겹침 / 벽 표본 | 0 / 0 / 0 | 0 / 0 / 0 | — |
| 도로 밖 바퀴 중심(등록) | 최대 0 (최소 중심–경계 56.6 mm) | 0 | — |
| 접지면 any/center/full(진단) | 최대 0/0/0, 벌점 추정 사건 0 | (미계산) | — |
| STOP 이동 / 정상 종료 | 52.68 mm / 14개 모두 정상 | 37.345 mm / 14개 | — |
| 판단 재생 / 입력·소스 | 654/654 / 불변 | 652/652 | — |
| 원래 G1 종합 / 측방 150 mm | 실패 / 표본·연속 간격 실패 | 실패 / 실패 | — |

세 실행은 시작 시각·난수가 같은 쌍 실험이 아니다. 최종 앞차 거리는 0.8548 m로 원래 최종 간격 관문 안이다([진단](drive_on_front_target.json)).

**노면 도구 구분:** 등록 관문은 평면 **바퀴 중심**과 도로 경계의 표본 대조([`drive_on_wheel_centers.json`](drive_on_wheel_centers.json))다. 접지면 any/center/full 세 해석([`drive_on_surface_contact.json`](drive_on_surface_contact.json), `scripts/surface_contact_eval.py`)은 별도 진단이며 어느 것도 심판 판정이 아니다.

## 측정속도·선호 유지·해제

[분석](preference_analysis.json)·[호출별 행](preference_analysis.rows.jsonl)(로컬 보존)은 판단 기록에서 원시 `speed_mps`(모터축 환산값, 실제 차체 속도와 같다고 보지 않음)와 선호 상태를 분리했다.

- 실제 전이: 옆(0.24 s) → 뒤 관측(3.52 s) → 사각(5.82 s, 시뮬레이션 13.52 s) → 뒤 관측(9.32 s) → 옆(10.02 s) → 앞(10.62 s). [전이 감사](drive_on/rear_transition_audit.json).
- 선호는 사각 진입과 같은 13.52 s에 시작해 17.10 s까지 180호출 적용됐다. 시험 구간의 측정 최대는 **1.1183 m/s**였다.
- 측정값이 설정 최대 1.0 m/s를 넘은 17호출(14.16–15.10 s, 1.021–1.118 m/s)은 모두 평행 후보의 `measured_above_target_cap` 분기에서 측정속도 정지공간과 해당 경로 이동 영역 관측을 통과했고(`parallel_current_position`, `current_path_observed`), 선호가 유지됐다. v3에서 12.94 s의 1.021 m/s로 선호가 풀린 지점과 같은 종류의 입력이다.
- 시험 구간 명령 최대는 0.97569 m/s(설정 1.0 − 20 ms 엔코더 1카운트분)였고, 선호 중 조향 절댓값은 0.00197 rad 이하였다. 측정값은 자르지 않았고 명령은 설정 이하였다.
- 해제는 1회: 17.12 s에 옆차가 다시 보이며 기존 GUARD(`side_threat_not_passed`, 0.35 m/s)로 돌아갔다. 시간 만료나 측정속도 검사 때문이 아니다.

이 1회에서 사전 등록한 진단 기준(최소 간격 ≥ 80 mm)을 넘었으므로 "개선이 관찰됐다"고 기록한다. 약 140 mm는 자차가 원래 차선을 유지했을 때 이 장면의 옆차 배치가 만드는 간격과 같은 수준이며(10/29 합성 명목 140.00 mm), 새 회피 행동으로 간격을 넓힌 것은 아니다. 사각 진입 시각이 v3(12.22 s)와 달라 같은 시점의 입력을 비교한 것은 아니다.

## 근거와 실행

- 등록 요약 [`registered_summary.json`](registered_summary.json)(10/29 `summarize_runs.py`), 원래 보고 [`drive_on/report.json`](drive_on/report.json), 실행 계약 [`drive_on/trial_overlay.json`](drive_on/trial_overlay.json)(실제 헤더 `rear_lane_keep_policy=observed_measured_v1`, 소스 불변, 신호 70 ACK), 재생 [`drive_on/follow_trace_replay_summary.json`](drive_on/follow_trace_replay_summary.json).
- 실행 소스는 [`drive_on_sources.zip`](drive_on_sources.zip)(SHA-256 `66e34394…d1cd`, 기존 정책대로 로컬 보존)에 준비 시점 그대로 있다. 원시 센서·자세·명령·호출별 기록도 로컬에 두고 판정·요약만 Git 대상으로 한다. 연결 코드·검사는 [연결 묶음](../rear_lane_measured_v1/README.md).
- 사전 검사 [`preflight_result.json`](preflight_result.json) 통과(경로·배치 반례, ON/OFF 차이가 정책 두 값뿐). 실행 스크립트는 v1 복사본이며 차이는 정책 값·설명·폴더 경로뿐이다([대조](copied_from_v1.json)).
- 실행: `drive_trial.sh on` → `post_analysis.sh`, Claude Haiku 5.5 보조가 실행·대기·회수를 맡았다. `trial=1`은 원래 G1 평가기의 측방 150 mm 실패에 따른 종료 코드이며 하네스 유효·정상 종료와 구분한다. 잔류 프로세스 없음.

## 남은 한계

- 1회·왼쪽·직선·각본 상자 상대차다. 오른쪽, 복합 센서 오차(10/29 합성에서 오른쪽은 초기 정체 2.9 s로 장면 미성립), 코너 직전/직후, 곡선 맥락 상실 시의 중앙 복귀는 확인하지 않았다.
- 측정 초과 허용은 실제 이동 영역이 관측될 때만 성립한다. 후방 사각의 실제 부재는 여전히 증명하지 않으며(`hidden_rear_risk_unresolved`), 실물 모터축·제동·슬립은 미검증이다.
- 원래 150 mm 측방 판정 실패와 이전 v3/2초 후보의 결과는 그대로 보존한다.
