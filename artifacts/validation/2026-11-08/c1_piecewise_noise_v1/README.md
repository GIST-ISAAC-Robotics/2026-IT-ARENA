# C1 잡음 모서리 분할 r35

2026-11-08. **구현·관련 회귀 완료, 같은 C1 조건 실제 시험 1회 완료. 원 판정과 보충 판정 모두 실패.** 사전 판정([PLAN.md](PLAN.md))은 바꾸지 않았다.

## 결과(B C1 주입, 60초, 1회)

| | r34 B(원 B) | r35 B |
|---|---|---|
| 종료 줄 | `HARNESS_EXIT=1 … ASSESSMENT_EXIT=1` | `HARNESS_EXIT=1 SURFACE_EXIT=0 INTEGRITY_EXIT=0 LEFTOVER_PGREP=1 ASSESSMENT_EXIT=1 WALL_S=775` |
| 원 판정 실패 | `no_permanent_stall`, `final_gap`, `full_lap` | 같음 |
| 보충 판정 실패 | `one_lap`, `no_contacts`, `additional_valid`, `sustained_motion`, `decision_scans_acquired` | `one_lap`, `sustained_motion` |
| 진행 / 최장 저속 | 9.11 m / 48.396 s(s 6.13) | **42.55 m**(44.64 m 미달) / **6.764 s**(60.595 s부터, s 39.57, 사례 끝까지 미회복) |
| 최고 속도 / 최소 간격 | 1.558 m/s / 0.849 m | 1.563 m/s / 0.809 m |
| 접촉 | 추가 actor가 뒤에서 관통 1회 | 0 |
| 소스·STOP·종료 | 902입력 불변, 정상 | 903입력 불변, STOP 67.847 s 정상(`ESTOP_LATCHED`/`software_stop`), 정상 종료, 잔여 프로세스 0 |
| 판단 기록 | 사례창 밖 취득 누락 2회 | 3,040회(사례창 안 2,989), 취득 누락 0, `dropped=0` |
| 주입 | 지연 0.032–0.070 s, 오차 ≤0.0300 m, 682=682 | 지연 0.032–0.070 s, 오차 ≤0.0300 m, 683=683 |

원 수치는 [assessment](B_c1_injection/assessment_run1.json), [report](B_c1_injection/run1/report.json), [종료 기록](B_c1_injection/run1.exit)에 있다. 이 원본은 수정하지 않았다.

### 결과 감사([audit/](audit/README.md))

완료된 원본만 읽었고 새 실행이나 재생은 하지 않았다. 도구는 [audit_r35_result.py](audit/audit_r35_result.py)이고 종료 코드는 0이다. 저속 구간 요약은 [r35_low_speed_compact.json](audit/r35_low_speed_compact.json)에 있다.
- **6.764 s 정체:** 제어기 HOLD가 사례 끝까지 이어졌다. 판단 출력 338회는 모두 0이다. 기록 사유는 `parallel_hold_partial_observation_outside_trial`이고, 선택자는 `RELEASED/parallel/large_motion_region_unobserved`다. 보호층 사유는 내내 `clear`다. 주 actor는 좌전방에서 멀어졌다(1.38→6.78 m). 따라서 보호층이나 실제 선행차는 원인이 아니다. 붙잡은 지속 patch (0.64, −0.19) m는 평가 전용 벽 메시와 겹친다. 좌표계 가정 아래 정적 벽일 가능성이 높으며 이 대조만으로 확정하지 않는다. 같은 위치를 무주입 r34 A는 `SOLO` 선택자로 지나갔다. 미관측으로 판정된 구체 영역과 `parallel` 선택이 해제되지 않은 이유는 미확인이다.
- **짧은 구간:** 44.695 s 구간(0.502 s)과 38.859 s 구간(0.202 s)도 `parallel` HOLD 직후에 생겼다. 두 구간 모두 0.1 s 만에 회복했다.
- **처리시간:** 기록된 값은 `motion.processing_ms`(상태 표본) 하나다. 이 값의 p50은 r34 A 0.506 ms, r34 B 0.499 ms, r35 B 0.506 ms다. 판단 전체와 `piecewise_wall`의 계산 시간은 기록되어 있지 않다. 스캔 나이 p50은 B 두 run 모두 0.197 s다. A의 0.156 s보다 0.041 s 길다.
- **다음 조사:** 60.36 s 전후 저장 판단 입력을 같은 바이트로 오프라인 재생한다. 이 재생은 새 주행이 아니다. 여기서 미관측 영역과 `parallel` 진입·유지 조건을 확인한다. 같은 조건의 주행은 반복하지 않는다.

이 결과는 한 seed, 한 거리오차·지연 조합의 제한 결과다. 첫 코너 정체가 이 run에서 재현되지 않았다는 사실을 일반 성능이나 IMU/회전율 오차·실차 강건성으로 확대하지 않는다.

## 구현·사전 검증(실행 전 기록, 보존)

- [설계·사전 판정](PLAN.md), [첫 코너 원인과 그림](../c1_corner_observation_probe_v1/README.md).
- 생산 변경은 `src/arena_autonomy/arena_autonomy/piecewise_wall.py` 하나다. 기존 선택형 모서리 표현에서 잡음 때문에 RDP가 과분할된 경우에 한해, 같은 원시 점의 두 선분을 찾는다. 모든 점의 최대잔차3cm·최소면길이15cm·각도40–135도·조향반경을 유지한다. `piecewise_wall_trial` 기본false는 유지한다.
- 합성 거리오차 양방향/3seed와 기존 모서리, 직선·짧은면·복수모서리·큰 돌출점 **23검사 통과**. 최초 평균선 후보는 seed0 양방향에서 실패했으며 `before_minimax_center/`에 보존했다. 최종 후보는 점을 지우지 않고 반환 띠 중앙으로 선 위치를 조정한다.
- [관련 회귀](regression_result.json) **243통과**, 출처 사슬 r34→r35 검증 포함. 이전 고정 목록 때문에 실패하던 두 출처 검사는 [별도 정리](../provenance_test_refresh_v1/README.md) 후 같은 최신 사슬을 사용한다.
- [무주입 A 재생](replay_result.json)은3,051회 전체 판단이 같다. 주입 B는525번째18.08초에서 처음 달랐으며 이후 입력으로 새 주행을 주장하지 않았다.
- 실행 도구는 기존 B의 인자·코스·overlay를 같은 바이트로 재사용하고, 실제 import되는 변경 모듈 해시도 대조한다. 사례창 밖 종료 후 판단 기록과 원시 최종간격 실패는 [사전 판정](PLAN.md)에 따라 별도로 남긴다.

역할은 다음과 같았다. 산출물 등록과 실행은 GPT-6 Luna, 설계·구현·검토는 GPT 주 에이전트, 결과 감사와 이 문서 정리는 Claude(Opus 5.5)가 맡았다.
