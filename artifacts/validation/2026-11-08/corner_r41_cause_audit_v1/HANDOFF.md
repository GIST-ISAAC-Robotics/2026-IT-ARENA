# r41 코너 원인 감사 인계 (미완료)

2026-11-08. five_hour 0.94 경고(주간 0.50)로 작업 총괄 지시에 따라 조사 초기에 중단했다. 이 폴더에는 이 파일만 있으며, 분석 스크립트·재생·계측·도표는 아직 만들지 않았다.

## 실행 상태

- 직접 시작한 백그라운드 프로세스·재생·Gazebo/ROS 실행은 없다. 원본·production 파일은 수정하지 않았다.
- 실행한 것은 읽기 전용 조회뿐이다(Windows Python으로 trace 행 종류 계수와 1개 call 출력 구조 출력).

## 확인한 사실 (관측)

- r41 trace: `artifacts/validation/2026-11-08/current_corner_support_v1/run1/follow_decisions.jsonl`(59,970,932 B). 행 종류는 header 1·**call 1,628**·**reset 381**·footer 1이다. 기존 r43 기록 sha256은 `43bdc149…6555`(`grid_cost_r43_v1/replay_r43.py` TRACE_SHA)이며, 이번 세션에서 재계산하지는 않았다.
- r43 `grid_cost_r43_v1/summary.json`: r43 전후 코드에서 r41 1,628판단의 recorded_mismatch는 0, stream은 `051d7364…`이다. **r44 이후 현재 코드로 r41 전체를 재생한 기록은 이번 세션에서 확인하지 못했다.** r44 `replay.json`은 r32/right20/medium 등의 결과만 있다.
- 헤더 설정: `traffic_boundary_refresh_trial=True`, `piecewise_wall_trial=True`, `parallel_hold_trial=True`, `pass_release_trial=True`, `guard_curve_trial=True`, `boundary_evidence_mode=corner_distance_candidate`, `boundary_min_ahead_m=0.65`, `boundary_anchor_error_m=0.12`.
- 출력 구조: `output[2]`는 최상위 `reason=handoff_follow`이며, 내부 원인은 `output[2]['normal']`에 있다(`reason`, `boundary_selection`, `path_wall`, `wall_fallback`, `boundary_from_memory`, `corridor_candidate.reason`, `partial_observation`, `selector_choice`, `selector_reason`, `current_lane_preference.reason`, `current_corner_support`(채택 시)). 상위에는 `preparation.reason`, `follow_entry_space.reason`, `moving_pass.reason`, `relative_yield`가 있다.
- 예시 t=34.40 call: 출력 1.0 m/s이고 normal.reason=local_follow, path_wall=right, wall_fallback=true다. corridor_candidate는 `long_outer_reference_unavailable`, selector_choice는 baseline이다.

## 다음 명령 / 남은 작업 (과제 1–4 모두 미완료)

1. 현재 코드 전체 재생 1회: `grid_cost_r43_v1/replay_r43.py`의 `run repo <label>` 방식(WSL, `env -i … python3 -I -B`, `bundle_parity.replay_once`)을 r41 하나에 적용한다. 다만 r43 `check_sources`는 r43 계약을 기준으로 하므로, r44 이후 소스 검사는 별도 해시 기록으로 대체해야 한다. call/reset 수와 recorded_mismatch 0을 확인한다.
2. 원인 분류: call마다 `normal.reason`·`boundary_selection`·`corridor_candidate.reason`·`selector_choice/reason`·`partial_observation.reason`과 속도를 ≥33 s 구간에서 표로 만든다. 같은 스캔(`input.source_first_s` 동일, 약 5호출)과 새 스캔을 구분한다. `parallel_hold_baseline_unavailable`/`reference_not_straight`는 `pass_release_trial.py`의 해당 분기와 `local_follow.py` `_baseline_command`의 실제 baseline 값을 읽기 전용 래핑으로 확인한다(전후 명령 불변 검증).
3. 도표: 이전 `corner_path_support_probe_v1/capture.py`·`geometry.py` 방식(센서 반환점·path_points·차량 외곽)을 참고해 대표 3–5개 시점만 그린다. actor truth(`run1/actor_poses.jsonl`)는 평가 전용으로 분리 표시한다.
4. README 결론: 경로근거/선택기 차단/실제 간격·관측 부족 중 어느 것인지 정리한다. 이미 있는 reference_not_straight 저속 fallback은 새 대안으로 제안하지 않는다.
