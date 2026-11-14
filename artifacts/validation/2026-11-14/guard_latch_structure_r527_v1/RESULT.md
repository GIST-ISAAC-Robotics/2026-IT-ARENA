# r527 소실점 래치 구조 검토

2026-11-14. **현재 원시 점유와 충돌하지 않는 명령도, 과거 점의 재관측 실패 때문에 무기한0으로 유지될 수 있는 구조가 있다.** 다만 r520의 최초 래치가 그 경우인지, 실제 위험 점이 사라진 경우인지는 원시 자료 담당의 판단이 필요하다. 주 담당의 후속 전달은 WAIT_GREEN7.607s에 이미 래치가 있었고 첫 helper 양수는7.72s이므로 helper가 최초 원인이 아니라는 것이다. 이 시각은 주 담당의 원시 분석 결과를 전달받은 것이며 이번 담당이 별도 재분석하지 않았다. 이번 작업은 코드만 읽었으며 r520 원시 자료 분석·실행·공통 변경은 하지 않았다.

## 확인된 판단 흐름

1. `partial_observation.py:416`의 `_observe_missing_history`는 과거 반환을 엔코더/gyro로 현재 좌표에 운반하고, 새 unknown 방향과 감속 외곽에 겹치는 후보를 만든다. 기본 팽창은 `0.03+2×경과시간`m이고, `TimedPolicy._refine_threat`는 점별 취득시각에 기반한2m/s 이동 가능 외곽으로 대체한다. 현재 실제 충돌 검사와는 다른 소실 가설이다.
2. 현재 점에서20mm 이내이거나,20–40mm 거리에서 인접3광선 현의 수직거리10mm 이내이면 소실 가설에서 뺀다(`partial_observation.py:460–466`). 선택형4점 표면도 대응거리40mm를 넘기지 않는다(`surface_reobservation.py`). 공통 표면 정합 역시 정적 배경·낮은 잔차 가정이 있다. 벽의 비강체 굴곡/국소 밀림을 일반적으로 설명하는 모델이 아니다.
3. 남은 후보가 있으면 `previous_point_in_missing_sweep` 래치를 만들고 최초 전체 위협점과 배경을 보존한다(`partial_observation.py:491–523`, `traffic_recovery_trial.py:195`). **이후 호출에서 새 소실 후보가0개가 되더라도 기존 `latched`는 저절로 풀리지 않는다.** 새 원시 기억은 갱신하지만 래치는 별도다.
4. 기본 해제는 완전한 전방 관측의 서로 다른 새 scan2회다(`partial_observation.py:190–199`). 부분 결측 경로에서는 `clear_sources=0`이므로 벽 틈 등으로 결측이 계속되면 이 해제가 성립하지 않을 수 있다.
5. 교통 회복은 원 위협점 재관측 또는 옛 위치 주변의 현재 실제 반환 지지2회를 요구한 뒤 현재 입력을 다시 검사한다(`traffic_recovery_trial.py:229–321`). `SurfaceAnchorTrial`은 최초 생성 후1s 또는 갱신된 앵커와0.3s를 넘으면 `ended=True`; `PositiveClearanceTrial`도 최초1s 후 `ended=True`다(`surface_anchor_trial.py:64–65`, `anchor_clearance_trial.py:90–91`). **만료는 래치 해제가 아니라 해당 해제 경로의 종료다.** 이후 완전관측2회나 명시 reset 등이 없으면0을 계속 유지할 수 있다.
6. `lidar_safety.py:208`은 `max(명령속도, |실측속도|)`로 partial을 평가한다. `partial.allowed=False`이면220–221행에서 즉시 차단하며, 현재 원시 점유의 `swept_limit`/`braking_speed`는254–257행이므로 도달하지 않는다. 따라서 `known_obstacle_latched` 로그만으로 현재 충돌 여유가0이라고 판단하면 안 된다. 그 여유는 별도 계산해야 한다.

현재 `command_observation_priority`는 전역 결측폭·과거 자유공간 warmup·시험속도상한을 중복 관문에서 제외하지만, 위 소실점 래치는 유지한다(`partial_observation.py:386–414`). 같은 scan 반복에서 생긴 후보는 `same_scan_missing_wait`로 처리해 새 영구래치를 피하는 별도 경로가 있으나, **fresh scan에서 만들어진 기존 래치를 푸는 기능은 아니다**(`traffic_recovery_trial.py:161`).

기존 `tests/test_independent_guard_route.py:98`,`:130`, `tests/test_partial_observation.py:58`은 실제 사라진 근거리 점이 시간 만료만으로 허가되지 않는 것을 명시적으로 요구한다. 따라서 이 구조는 단순 누락이 아니라 기존 정책의 의도이며, 현장 벽을 다루려면 명령별 허용 정책을 분리해야 한다. 이번 검토에서 테스트를 실행하지 않았다.

## 기존 경로 중 재사용할 부분

| 현재 코드 | 재사용 가능한 기능 | 그대로 적용되지 않는 부분 |
|---|---|---|
| `current_wall_reacquire.current_wall_candidate` | 현재 짧은 곡선/조각 경로, 최대0.10m/s, 현재/직전 조향의 원시 점유·per-ray 여유·제동 | 계획기 후보이며 독립 guard의 래치를 해제하지 않는다. 이 함수를 guard 허가로 복사하면 독립성이 사라진다. |
| `independent_guard_route.front_evidence` | 원 각도 metadata·유효 근거리·현재 전방 반환 확인 | 관측만으로 충돌/옛 점 부재를 증명하지 않는다. |
| `recovery_geometry.timed_missing_sweep`와 `sweep_rectangles` | 낮춘 명령속도와 실제 측정속력의 최댓값, 현재/직전 조향에서 기억 위협의 제동외곽 교집합 재계산 | 현재는 래치 생성에 사용하고, 생성된 래치의 명령별 재허가에는 사용하지 않는다. 2m/s 모델을 유지하면 낮춘 속도에서도 위협이 남을 수 있다. |
| `known_structure.StructureAwareMixin._structure` (`:422–499`) | 실제 숨은 점을 운동 이력으로 계속 운반하고 현재/직전 조향의 여유→제동속도 상한으로 제한 | 알려진 구조 사각·정지 기억 가정의 경로다. 일반 전방 소실점 전체를 정적 벽으로 취급하여 이 경로에 넣으면 정책 가정이 바뀐다. 기존 Mixin은 기반 `allowed=False`를 뒤집지 않는다(`:370`). |
| `lidar_safety.py:254–268` | 독립 현재 원시 점유 제동, 현재 마커 점유 검사 | 현재는 래치 실패 뒤 도달하지 못하며, 기존 주검사는 현재 조향만 사용한다. 새 저속 검사는 직전 조향도 유지하는 재사용 단위가 적합하다. |

`guard_observation_policy`의0.25/0.15m/s 재시도는 planner selector의 `partial.allowed=True`/`fully_observed` 등을 요구한다. 독립 `known_obstacle_latched`0을 풀어주는 기존 완성 경로가 아니다. r521/r525의 계획기 양수 결과만으로 독립 guard까지 통과했다고 해석하면 안 된다.

## 최소 설계 제안 — 아직 구현·검증하지 않음

**기억 보존과 이번 명령의 허가를 분리하는 guard 내부의 낮은 속도 재검사**가 가장 작은 방향이다. `latched=False`나 기억 TTL 삭제, unknown의free 변환으로 해결하지 않는다.

1. 선택된 현재 명령 정책에서 `previous_point_in_missing_sweep` 때문에만 차단된 경우, STOP·입력 시각·운동 무결성·원 ray 정합·현재 전방 관측을 그대로 통과시킨 뒤 최대0.10m/s 후보를 독립 계산한다. 실측속도가 더 높으면 그 실측속도로 제동 외곽을 검사한다. 후보속도를 실제속도 대신 넣어 짧은 지평을 위조하면 안 된다.
2. 현재 원시 반환·실제 마커 점유와 기억한 전체 위협점을 **별도 집합으로 보존**하고 현재/직전 조향에서 여유를 계산한다. 현재 자차좌표에 운반한 실제 기억점이 명령 외곽에 겹치거나 급박 거리이면0을 유지한다. 현재 점유와 운반된 기억의 간섭이 없고 재관측 실패/옛 probe 종료만 남은 경우는 래치 상태를 유지한 채 명시0.10m/s 상한으로 이번 명령만 허용하는 안이다. 2m/s 도달집합을 과거 전체시간에 걸쳐 계속 확장하는 기존 소실 가설은 진단으로 남기되 이 저속 명령의 영구0 근거로 강제하지 않는다. **이는 동적 물체 가능성을 해소한 것이 아니라 그 불확실성을 남긴 명시 저속 운용 정책**이다. 다음 유효 입력마다 독립 재검사한다.
3. 오래된 원 좌표를 운반하려면 유효한 운동 이력이 필요하다. 기존 probe1s 종료를 단순 무시하거나 과거 좌표를 현재좌표로 간주하면 안 된다. 구조 기억의 방식처럼 실제 보존점의 좌표만 연속 운반하고 원 관측시각·최초 증거는 별도 보존하는 형태를 재사용한다. 실제 원시 점은 기존 per-ray 여유를 유지하고, 기억점은 기존 차체/측정 충돌 여유를 유지하되 과거 전체시간×2m/s를 현재 실제 점유라고 부르지 않는다. 일률적인 점 대응 거리 확대나 벽이라는 라벨 주입은 하지 않는다.
4. 첫 구현 판정은 원 성공 양수 유지, 실제 원시 점유 충돌0 유지, 실제 숨은 근거리 물체가 낮은 제동외곽에 들어오면0 유지, 벽 굴곡/틈으로만 과거 위치 대응이 실패해도 낮은 외곽이 비간섭이면 양수, 기억·UNKNOWN·원 scan 불변이어야 한다. 두 조향·실측속도·입력 오류 반례도 포함한다. 이는 수용 조건 제안이며 검증 완료가 아니다.

우선 r520 최초 `latch_evidence`의 전체 위협점, `timed_missing_diagnostic.checked_speed_mps`, actual commanded/measured speed, 현재/낮춘 명령의 독립 swept 여유를 비교하면 소실 가설과 현재 충돌을 분리할 수 있다. 재관측 실패거리·표면 정합 reason과 두 probe의 ended/만료 여부도 확인 대상이다. 이 원시 자료 확인은 주 담당이 수행하며 이번 r527은 중복 분석하지 않았다.

### 큰 추적기 없이 연결할 함수·상태

- **원 증거:** `RecoveryPolicy._capture_latched_threat`가 `clearance_probe.original=(retained, background, original_reference)`를 이미 만든다. 이 원본과 `latch_evidence`는 보존한다. `anchor_probe.retained`는 표면 정합으로 이동될 수 있으므로 새로운 독립 현재좌표 원본으로 무조건 가져오지 않는다.
- **최소 추가 상태:** 원 전체 위협점의 현재좌표 복사본과 그 좌표의 기준시각만 둔다. 시작은 `clearance_probe.original[0]`, `[2]`다. 이후 유효한 운동 입력마다 `transport_points(stored, history, reference, now)`로 자기 차량의 운동만 반영하고 기준시각을 갱신한다. 물체 신원·속도·궤적을 새로 추정하는 추적기가 아니다. 원본 관측시각/점은 덮어쓰지 않는다. 운반 불가·시각 역전·형상/메타데이터 무결성 실패는0이다.
- **순수 기하 함수:** `known_structure._structure`의 `min(swept_limit(cloud,current_angle), swept_limit(cloud,previous_angle)) → braking_speed` 부분을 작은 함수로 재사용한다. 새 경로에서는 `_inside` 밖이나3m 밖이라는 이유로 기억을 지우는 로직, FIFO절단, 가상 그림자 점 생성은 가져오지 않는다. 실제 기억점과 원시 현재점의 여유를 각각 기록한다.
- **guard 호출 위치:** `lidar_safety.control`의 partial 평가 직후, 기존 `if not partial['allowed']` 차단 전에 명시 선택 정책/특정 래치 원인에 한정한 낮은 속도 재검사를 둔다. 통과해도 원 래치/불확실성은 남기고 이번 호출에만 적용할 cap을 반환한다. 그 뒤 기존 독립 raw swept·마커 점유·STOP/시각 처리가 그대로 이어져야 한다. 계획기의 `current_wall_candidate` 출력이나 proof를 보호층의 허가로 받아들이는 방식은 피한다.
- **상태 표기:** `latched=True`, `memory_retained=True`, `unresolved_dynamic_motion=True`, 이번 명령 cap/실측속도/두 조향 여유/원 래치 reason을 함께 보존한다. `allowed=True`를 전체 공간 자유나 소실 물체 부재 증명으로 표기하지 않는다. 완전관측2회 등 기존의 정식 래치 해제 절차와 이번 저속 통과를 구분한다.
- **필수 정지 반례:** 현재 raw 실제 근접 장애물, 운반된 옛 위협점의 현재 조향 충돌, 직전 조향만 충돌, 실측속도가0.1m/s보다 높아 제동외곽이 커지는 경우, 운동 이력 공백, 원 scan 불일치, 전방 전체 관측 소실, STOP/명령 만료. 재관측 probe만1s 종료되고 현재/기억점 둘 다 명령 외곽 비간섭인 경우는 새 저속 허가의 양성 비교다. 과거 기억을 삭제한 결과와 구별해야 한다.

이 안에서도 소실 물체가 기억된 좌표에서 독립적으로 이동할 가능성은 남는다. 이를2m/s 전체 도달집합과 동일한 보장으로 설명하면 안 된다. 대신 현재 실제 점유·보존된 실제 기억의 간섭·낮은 명령속도를 기준으로 운영하며, 새 반환이 생기면 즉시 현재 점유 검사에 들어가는 정책 선택이다. 최종 채택은 주 담당의 원시 분석과 반례 검증 뒤 판단한다.

범위: 소스/기존 테스트 읽기만 수행. 모델·코드·문턱 변경0, 런타임/WSL/Jetson/ROS 실행0. 읽은 파일 해시는 `source_review.json`에 기록했다.
