# r422 crop 경계 계획과 현재 저속 회복의 연결 설계

2026-11-13. **권고는 SOLO의 세 진입 게이트만 명시 simulation 조건에서 확장하고, 현재 검사 본문은 유지하는 것이다.** `fully_observed`를 전체적으로 바꾸거나 `bounded_side_missing`을 완전 관측으로 취급하지 않는다. 구현·저장 재생·WSL/Jetson/ROS/시뮬 실행은 하지 않았다. 이 문서만 작성했다.

r418의10.10m 정체, 후반 guard clear/MCU ARMED와 부모 drive0은 주 에이전트가 전달한 실행 관측이다. r421 후속 감사에서는 후반10초의99고유 current 모두 plan 양수/crop 진단, current0과 인접 부모0의 대응을 확인했다. 또한 연결된 plan/current의 **전체 원 scan 사전이99/99 동일**하고 reference 차이는2–4ms라고 전달받았다. 원로그·시각별 원인은 r421 담당이며 여기서 대량 재조사하지 않았다. 따라서 아래는 **현행 코드의 연결 결함과 최소 설계**이고, 정체 전부의 원인이나 새 후보의 회복을 실증한 결과가 아니다. 실물/`host_estimated_v1` 적용 제안도 아니다.

## 확인한 차단 지점

[partial_observation.py](../../../../src/arena_autonomy/arena_autonomy/partial_observation.py)의 r417 경계 성분은 원 배열에서 두 실제 flank까지 성분을 확장한다. 원시각/각도 메타데이터, 전체 공백 폭·비율, 근거리 반환, 현재 방향과 이전 소실 위협 검사를 통과하면 `allowed=True`, `reason='bounded_side_missing'`, `crop_edge_component.scope='bounded_simulation_edge_component_not_free_space'`를 남긴다. 이 진단은 도중에 만들어질 수도 있으므로 **키 존재만으로 accepted로 해석하면 안 된다.**

현재 SOLO baseline은 [retained_recheck/_parallel.py](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/retained_recheck/_parallel.py)의 baseline 분기에서 [_geometry.py](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/retained_recheck/_geometry.py)로 들어간다. 이 기본 경로는 `partial.reason`을 요구하지 않는다. 현재 path 운반·추종·점유를 검사한 다음 [current_region.py](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/current_region.py)의 전체 motion region에서 정지할 수 있다. 즉 새 계획을 기본 current가 전혀 이해하지 못하는 문제와 구별한다.

그 뒤 다음 세 조건이 새 계획을 제외한다.

| 지점 | 현행 조건/연쇄 | 최소 변경 |
|---|---|---|
| [solo_gap.py](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/solo_gap.py):17–24 | 원 current가 `route_current_large_motion_region_unobserved`이고 partial `fully_observed`일 때만0.15m/s core+공백 재검사 | 기존 fully 경로 그대로 + 명시 crop scope 허용 |
| [solo_relative.py](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/solo_relative.py):20–26 | 위0.15 실패 진단과 fully 조건이 있어야0.10m/s core+상대 이웃 재검사 | 같은 scope만 추가, 앞 시도 실패/cap 요구 보존 |
| [corner_candidate.py](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/corner_candidate.py):17–31 | 위0.10 실패가 `outside_small_body_side`이고 fully 조건이어야 거리 제안 | 같은 scope만 추가, 원 실패 종류와 나머지 조건 보존 |

`solo_crawl`은 unobserved=0인 mask-only 재시도이다. 이번 unknown 경로를 연결하는 데 그 판정을 넓힐 필요가 없다. GUARD/RELEASED/parallel의 scope, plan 단계의 `observation_policy`/`released_plan`과 별도 `current_path_recheck` 사본도 그대로 둔다. 전체 검색에서 발견한 같은 문자열을 일괄 바꾸는 수정은 금지한다.

## 최소 구현 형태

**기존 current 한 번 →0.15 재검사 → 필요할 때0.10 재검사 → 해당 실패에서 단일 거리 제안**이라는 기존 순서를 사용한다. 새 wrapper/추가 속도열/성공까지 반복을 만들지 않는다.

1. `[기존 fully scope] OR [명시 crop scope]`를 판별하는 작은 순수 helper를 공통 policy 패키지 안에 둔다. 기존 fully 쪽은 현재 조건 그대로 평가한다. 새 쪽만 `allowed is True`, reason 정확히 `bounded_side_missing`, 경계 성분 mapping과 정확한 scope/비어 있지 않은 source 성분을 요구한다. 기존 `FOLLOW`·SOLO·baseline·`local_follow`·`boundary_from_memory is False`와 속도/조향/실패 종류 조건도 유지한다. 일반 bounded-side, forward/metric missing, latch/허용 false, 진단이 없는 기록을 넓히지 않는다.
2. 새 scope의 활성화는 [Evaluator](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/evaluator.py)가 명시 keyword로 전달한다. 기본값은 false다. 권고 범위는 r418에서 이미 사용한 **`cfg.crop_edge_component_trial is True`와 `current_corner_budget='r396_v1'`의 조합**이다. 기존 [runtime_node_backend.validate_corner_selection](../../../../src/arena_autonomy/arena_autonomy/runtime_node_backend.py)의 native2slot 등록·native 실행·use_sim_time=true·motion timing verified·verified_only 검사를 재사용한다. 실제 노드의 `mode='both'`, `native_simulation_v1`, hardware 입력 아님 검사는 기존 [crop_edge_selection](../../../../src/arena_autonomy/arena_autonomy/crop_edge_selection.py)을 유지한다. crop ON/current-corner OFF를 새 회복 경로로 자동 확대하지 않는다.
3. 현재와 submitted snapshot 모두 `native_measurements_v1`/`simulation_acquisition`, `timing_verified is True`, 시각 정책 verified_only인지 확인한다. **첫 시험은 새 crop scope에만 `data['current']['scan'] == data['submitted']['scan']`의 전체 원 scan 동일성도 요구한다.** 비교는 [follow_trace.freeze](../../../../src/arena_autonomy/arena_autonomy/follow_trace.py)가 NaN/+Inf/−Inf를 구별해 보존한 원 snapshot 사전에서 한다. thaw한 NaN 배열의 단순 동등 비교나 ranges만의 비교를 사용하지 않고, 원 시각·source/frame·epoch·각도/인덱스 등 scan 사전 전체를 포함한다. 기존 fully 경로에는 이 조건을 추가하지 않는다. missing verified-only 필드의 기존 legacy 해석은 유지하되 host estimate를 true로 바꾸지 않는다. 현재 scene/history는 이미 복원·검증한 같은 객체를 전달한다. scope 판별 결과를 저장 결과 사전/partial reason/cfg에 써 넣거나 global patch하지 않는다. `MotionConfig`에는 노드의 mode/verified/use_sim_time가 없으므로 그것만으로 실행 환경을 입증했다고 쓰지 않는다.
4. keyword는 `released_relative → solo_relative → solo_crawl → solo_gap`에서 필요한 곳까지 전달한다. `solo_crawl`/`released_relative`는 전달만 하고 자체 허용 판단을 바꾸지 않는다. `corner_candidate.eligible`의 두 호출(Evaluator 선검사와 try 내부)에 같은 값을 전달한다. 주요 수정 후보는 이4파일·corner_candidate·evaluator·작은 helper이며 새로운 launch/default는 필요 없다. 기존 environment/header를 재사용하므로 OFF header에 추가 키를 넣을 이유도 없다.

과거 `crop_edge_component`는 **어떤 계획 종류를 현재 재검사할지 고르는 표식**일 뿐이다. 그 flank 거리/width/source_indices를 현재 허가로 가져오지 않는다. 원 scan이 같아도 현재 좌표로 변환한 rays·점유·질의는 새로 계산한다. current마다 새 `PartialObservationPolicy`를 만들어 빈 기억으로 replay하거나, plan의 기억 상태를 guard에 복사하는 설계도 불필요하다.

단일 scan 한정의 장점은 관측된99개 모두를 버리지 않으면서 새 배열에 옛 인덱스를 적용할 여지를 제거하고, 원시 관측 변화와2–4ms의 현재 운동/기하 재검사를 분리하는 점이다. 구현도 원 사전 비교 한 번이며 시간 tolerance를 새로 만들 필요가 없다. 단점은 **신선한 다음 scan이 도착하면 같은 실제 성분이어도 이번 확장에 진입하지 못한다**는 점이다. scan rate/스케줄링이 바뀌면 추가 정지나 적용률 저하가 가능하며, 이 제한을 일반 운용 계약으로 바로 채택하면 안 된다. 양쪽 history/reference/measured/previous steering까지 같다고 요구하지 않고,2–4ms를 새 허용 상한으로 만들지도 않는다. 같은 raw를 다시 쓰더라도 현재 scene·양쪽 조향·D(measured)·source 나이는 갱신된 reference에서 재검사한다. 원 취득시각은 그대로다.

## 그대로 남아야 하는 현재 허가 계약과 한계

- core는 원 경로/출처와 submitted→current epoch/side, 경로 나이0.5초, 현재 source 유효성/시간/운동 이력, 경로 이동 한계, 조향 변화율과 **현재·직전 양쪽 조향**의 점유 swept 검사를 수행한다. 최초 계획 reference/source stamp를 current 시각으로 바꾸지 않는다.
- 0.15는 기존 [released_body_gap.check](../../../../src/arena_autonomy/arena_autonomy/released_body_gap.py),0.10은 기존 relative-neighbor body proof를 그대로 호출한다. 거리 후보는 D(v)에 실제 measured speed·나이·dt·반응0.1초·감속2m/s²를 유지하고 단일 제안 뒤 core와 새 motion_region/proof를 검사한다. 10mm/6°/50mm·mask 꼬리·0.15 measured 한계·최소10mm 질의거리·TTL을 낮추거나 늘리지 않는다.
- 최종 [bounded_body_nonreturn](../../../../src/arena_autonomy/arena_autonomy/current_path_recheck/_gaps.py)와 [range_mask_component](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/range_mask_component.py)는 **현재 raw/scene frame·시각·원 배열 일치, 가까운 점유, 실제 두 이웃, 전체 연결 성분 및 공백 합계**를 검사한다. 불관측 질의를 과거 crop 범위만큼 제외하거나 free로 채우지 않는다. 최종 region의 linspace가 proposal의 부분집합이라는 가정도 하지 않는다.
- 중요한 한계: plan의 crop은±100° 경계를 가로지를 수 있지만, 현재 body/r396 proof는 해당 질의가 속한 전체 gap에 `abs(direction) <= 100°`가 있으면 `front_gap_outside_scope`로 거절한다. **게이트를 연결해도 이 검사는 그대로다.** 해당 crop 성분이 현재 이동 질의와 무관하면 다른 후방 gap만 검사할 수 있고, 겹치면 계속 정지할 수 있다. 어느 쪽인지는 실제 r418 current 입력 재생 전에는 모른다. 이를 통과시키려고100° 범위나 raw 성분을 줄이는 것은 이번 연결안이 아니다.
- 결과가 양수여도 부모 발행 freshness/세대/STOP과 독립 `/drive → guard → /drive/safe`·MCU 계약은 그대로다. 기존 guard의 실제 현재 partial/점유 판정을 건너뛰지 않는다. guard clear/MCU ARMED는 부모 양수 명령이나 물리 이동의 증거가 아니다.

## 필요한 좁은 저장 비교

r421이 계보를 확인한 **정체 구간의 실제 current 한 대표**부터 사용한다. 원 output/submitted/current/header와 당시 소스를 보존하고 원 evaluator 결과가 먼저 일치하는지 확인한다. 해당 current는 이미 결정된 plan 출력과 짝이므로 독립 current 비교가 가능하지만, 이를 누락 없는 전체 plan 상태 이력 재현으로 부르지 않는다.

판정은 `새 scope 진입 /0.15·0.10 적용 또는 실패 이유 /단일 proposal 생성 /최종 current 허가 /양수 출력`을 구분한다.0.15에서 회복되면 불필요하게 거리 제안을 강제하지 않는다. 계속0이면 최종 거절 원문을 남긴다. 행동 비교와 추가 호출 수를 먼저 확인하며, 이번 설계만으로 별도 전체 프로파일/실주행을 요구하지 않는다.

필요 반례는 새로 넓힌 경계에 한정한다.

| 대조 | 기대 |
|---|---|
| crop OFF, 또는 corner OFF; 기존 fully 기록 | 새 분기 미진입. 원 출력/진단/입력 exact 동일. fully 기존 적용 행도 그대로 |
| crop 표식만 있으나 allowed=false/latch, 일반 bounded-side·forward·metric, wrong scope | 새 partial 진입 거절. `partial.reason`을 변조해 통과시키지 않음 |
| host_estimated/unverified, laser_scan 표현, source/frame/epoch 불일치·만료 | 기존 입력 경계 또는 새 simulation scope에서 거절. 설정 실패와 제어 STOP을 구분 |
| 원 계획은 동일하고 current의 source stamp/range/angle/metadata 중 하나만 변경 | 새 same-scan scope 거절. 새 scan의 옛 crop 인덱스 재사용 없음. 어느 조건에서 거절됐는지 기록 |
| 동일 raw이지만 현재 이력/previous steering 때문에 한쪽 swept에 가까운 점유 또는 gap이100° 안으로 옮겨짐 | 현재 양쪽 swept/최종 공백 검사 거절. 앞쪽 crop을 좁게 잘라 통과시키지 않음 |
| 현재 full-gap helper에 가까운/없는 flank·넓거나 여러 성분을 넣는 필요한 합성 반례 | 기존 raw/전체 gap proof 거절 유지. 두 snapshot까지 임의 변조한 경우 진짜 plan 재현으로 세지 않고, 동일성 통과와 proof 통과도 구분 |
| measured speed가 기존 범위 초과하거나 D(measured)>허용 거리 | 기존 범위/제동거리 거절. 요청속도만 줄여 실제 운동을 숨기지 않음 |

이미 검증한 r417 plan 반례 전체와 r405/396 전체 저장 묶음을 무조건 다시 돌릴 필요는 없다. 기존 OFF/fully 대표와 새 current 표식·현재 raw 변조의 차이를 좁게 확인하면 된다. 변조 입력은 합성 반례로 구분하고 원 입력·실제 센서 성공으로 합산하지 않는다.

**채택 판단:** 세 gate의 단절을 수정할 근거는 있다. 그러나 최종 raw proof의100° 제외나 외곽 조건으로 계속 정지하면, 현 제약 안에서는 이번 연결이 회복에 충분하지 않다는 결론으로 끝낸다. 새 속도 상수·허용각·TTL을 덧붙이지 않는다. 전체랩/실물 효과·안전 승인은 이 문서의 범위 밖이다.
