# r469 정체 구조 자문 최종 보고 (읽기 전용)

r469의 두 정지는 원인이 서로 다르며, 첫 번째는 실측 속도 입구 한 곳의 수정으로 해소될 가능성이 높고 두 번째는 예외 조건들이 서로 합성되지 않는 구조 문제입니다. 코드·문서·시뮬·장치는 변경하지 않았고, 저장된 `follow_decisions.jsonl`을 읽기 전용으로 집계한 것 외에는 프로젝트 코드를 실행하지 않았습니다.

## 1. 관측 근거 (r469 원로그 직접 집계)

| ref (sim s) | 관측 | 판정 |
|---|---|---|
| 11.42 → 11.72 → 12.42 → 13.22 | SOLO → GUARD → SOLO(`world_fixed_return`) → GUARD(양쪽 4점, ds 약 −0.32) → RELEASED(양쪽 `vanish=rear`) | 이후 끝까지 RELEASED 유지 |
| 18.22 | 왼쪽 `clear` 51→0 (좌코너에서 측방 창 일부 미관측), 실측 0.796 | `retained=True`, 목표 상한 .35 |
| 18.52–18.82 | 꼬리 10.08–11.36 mm, 미관측 0 | `released_declared_structure_motion`, plan/current/발행 .35 (r468 정상 동작) |
| 19.02 | plan 실측 .306으로 body gap 수용, current(19.026) 실측 .367 | `invalid_motion_or_scan` → 0 발행 |
| 19.12 | 실측 .122, plan/current 수용 | 부모 .1 발행 1회 |
| 19.22–29.32 | 실측 0, 이전 조향 .3399(포화), 미관측 270–371/2850, 왼쪽 가림 약 1500, 꼬리 13.9–15.1 mm, 전면 가림 0 | `large_motion_region_unobserved` → `parallel_hold_reference_not_straight` 반복 |

- **첫 0 (19.02):** 점유·관측 실패가 아니라 `released_body_gap.py:12`의 `0 <= speed <= .35` 범위 입구에서 거절된 것입니다. 10 mm 자기 가림 문제와 무관합니다.
- **지속 0 (19.22 이후):** 미관측과 10 mm 초과 꼬리가 동시에 있는 상태이며, current 호출 자체가 없습니다.
- **속도 양자화:** 로그의 실측 속도는 모두 0.0612 m/s의 배수입니다. 한 카운트가 약 0.612 mm이므로 10 ms 창당 1카운트와 수치가 맞습니다(창 길이는 추정).

## 2. 질문 1 — 반복 정체를 만드는 구조적 중복·불일치

1. **예외가 서로 합성되지 않습니다.**
   - 구조 그림자 예외는 `unobserved == 0`을 요구합니다(`structure_region.py:17`, `current_region.py:44`).
   - body gap `check`는 미관측을 증명한 뒤 10 mm 꼬리 규칙을 구조 예외 없이 다시 적용합니다.
   - `lower_masked_motion`은 retained가 아니고 미관측 0일 때만, rear fringe/sweep은 retained일 때만 동작합니다.
   - crawl .10(`released_plan.assess`)은 retained False와 정확한 선행 시도 서명을 요구합니다.
   - 19.22의 상태(미관측 > 0, 꼬리 > 10 mm, retained, 속도 0, 포화 조향)는 어느 예외에도 해당하지 않습니다.
2. **같은 이동 영역을 세 곳에서 다른 예외 목록으로 평가합니다.** plan RELEASED 분기, GUARD 사슬, current 사슬이 각각 다르고, current는 plan이 표시한 항목만 재검사합니다.
   - `bounded_body_nonreturn`도 `current_path_recheck/_gaps.py`와 `retained_recheck/_gaps.py`에 서명이 다른 두 사본으로 존재합니다(diff로 확인).
3. **실측 속도 입구가 명령 상한과 같은 값입니다.** `released_body_gap.py:12,38,47`, 두 `_gaps.py:23`, `current_region.py:66` 등이 해당합니다. .35를 명령하면 실측은 .306과 .367을 오가므로 거절이 구조적으로 발생합니다. `propose`의 `speed > target` 건너뛰기도 같은 성격입니다.
4. **정지가 흡수 상태입니다.** 속도 0, 고정된 포화 조향, 같은 스캔이면 같은 판정이 계속 반복됩니다. `propose`는 실패한 시도를 버려서 진단도 남지 않습니다.
5. **선택기 상태가 낡지 않습니다.** 측방 반환 몇 점으로 랩 끝까지 RELEASED가 유지됩니다. 안쪽 코너에서 측방 창에 미관측 표본이 생기면 `retained`가 다시 걸려 상한 .35와 다른 예외 집합으로 바뀝니다. SOLO에는 이 plan 단계 관문이 없습니다.

## 3. 질문 2 — 추천 수정 순서

**⓪ 진단 기록 (동작 변경 없음).** `propose` 실패 시 상한별(.35/.25/.15) 거절 사유를 기록하고, r469의 19.22 이후 입력을 오프라인으로 재생합니다. ②의 설계가 이 결과에 달려 있습니다.

**① 목표 상한과 실측 허용의 분리 (가장 작고 첫 0을 직접 해소).**
- 목표 조건 `target <= .35`는 그대로 둡니다.
- 실측 조건만 `speed <= cap + 1 양자`로 바꿉니다. 양자는 엔코더 설정에서 유도한 이름 있는 상수 하나로 둡니다.
- 실측값은 클램프하지 않습니다. 영역과 제동 포락은 지금처럼 `max(speed, target)`의 실제 속도로 계산됩니다.
- 적용 위치는 `released_body_gap.check/propose`(`speed > target` 포함), 두 `_gaps.py`, `current_region.py:66`, `released_rear_sweep`이며, .15 입구에도 같은 방식을 씁니다.
- 결과적으로 .367은 통과하고 .428은 범위 밖으로 남습니다.
- 다만 `bounded_body_nonreturn`의 고정 상수(이웃 0.6 m 등)가 .35 이하를 암묵적으로 전제하는지는 확인하지 못했습니다.

**② 증명 합성 (핵심 구조 개선, 추천).** `released_body_gap.check` 안에서 한 번에 판정합니다.
- 미관측 부분집합은 기존 body gap/인접 구조 증명으로 처리합니다.
- 남은 가림 꼬리는 `current_mask_shadow`로 처리합니다.
- 수용 조건은 전면 가림 0, 두 부분집합 모두 설명됨, 숨은 폭 합계가 기존 5 cm 예산 이내, 근접 실제 반환 없음, 측방 present 없음입니다.
- `check`는 plan(`propose`)과 current(`_released.py`, `current_region.py:80`)가 함께 쓰므로 한 곳의 변경이 양쪽에 반영됩니다.
- crawl .10의 전제도 "합성 증명 통과"로 일반화하면 정지 상태에서 빠져나올 출구가 생깁니다.

**③ 근거 노화에 따른 RELEASED→SOLO 강등 (②가 19.22를 풀지 못할 때의 대안이자 보완).**
- 조건은 해당 측이 present가 아니고 passed이며, 연속 clear 약 10스캔과 `last_present` 이후 주행 약 1 m입니다. 충족하면 그 측을 reset합니다.
- 안쪽 코너의 관측 불가능 표본만으로는 `retained`를 다시 걸지 않습니다.
- 측방 검출은 모드와 무관하게 매 스캔 수행되므로, 실제 차량이 다시 나타나면 GUARD로 재진입합니다.
- r469에서는 14–15 s쯤 강등되어 18.2 s 코너를 SOLO로 진입했을 것으로 추정합니다.
- SOLO current 사슬에도 자체 예외 취약성이 있어 이것만으로는 충분하지 않습니다.

①이 19.22 이후의 정체까지 막았을지는 증명되지 않았습니다. 필요한 수정이지만 충분한지는 미확인입니다.

## 4. 질문 3 — 현재 보이는 경로 우선의 조건과 반례

**수용 조건 (모두 현재 스캔 기준이며, 과거 허가를 재사용하지 않습니다).**
- 전면 가림이 0이고, 실제 속도의 제동거리까지 전방이 관측된 자유 공간입니다.
- 패딩한 이동 영역 안에 실제 반환이 없습니다.
- 자유가 아닌 표본은 반환으로 양옆이 둘러싸인 선언 구조 그림자이거나 차체 인접의 한정된 공백뿐입니다.
- 숨은 폭 합계가 5 cm 이하입니다. 차량 폭 150 mm는 이 안에 숨을 수 없습니다.
- 측방 present가 없습니다.
- 불확실할 때는 0이 아니라 상한 .35로 대응합니다. 측방 이력은 지우지 않습니다.

**반드시 계속 거절해야 하는 반례.**
- 그림자 안의 가까운 실제 반환
- 선언되지 않은 누락 섹터(센서 결측)
- 전면 가림
- 5 cm를 넘는 그림자
- 측방 present
- 반환으로 닫히지 않고 진행 방향으로 열린 미관측
- 실측이 상한보다 2양자 이상 높은 경우
- 강등 직후 재등장한 측방 차량
- 후방으로 사라진 추종 차량의 재접근

## 5. 최소 유효 검증과 실차 미검증 위험

**최소 검증.**
- r469의 217개 입력과 r465의 207개 입력을 원순서로 재생합니다.
- 기준본이 exact로 일치하는지 먼저 확인한 뒤, 0→양수 전환이 의도한 호출에만 생기는지, 양수→감소가 0인지, ①·②에서 측방 상태 변화가 0인지 확인합니다.
- 위 반례를 fixture로 만들어 확인하고, 그 뒤 단독 주행 1회를 수행합니다.

**실차 미검증으로 남는 위험.**
- 실물 엔코더의 양자화 폭과 떨림
- 가림 기하와 장착 오차
- 그림자 안의 작은 물체
- 후방 원뿔에서의 동적 접근
- ARM 계산 비용 (r467/r470의 별도 사안이며 이번 정체 원인과 구분합니다)

## 6. 미해결 사항

- 19.22에서 `propose`가 상한별로 실패한 정확한 하위 조건은 로그에 없습니다. 제 추정은 증명 거절 또는 13.9 mm 꼬리이며, ⓪의 재생으로 확정해야 합니다.
- 정체 구간에서 `known_structure`가 `explained_sectors 3/4`, 미설명 섹터 123.15°를 보고했습니다. 이 때문에 `current_mask_shadow`가 거절하면 ②만으로는 풀리지 않을 수 있습니다.
- 11.42와 12.42의 대칭 측방 반환이 무엇인지 모릅니다. 시설이나 벽일 것으로 추정하지만 점 좌표가 없습니다.
- 18.22에 왼쪽 측방 창에 미관측이 생긴 원인을 확인하지 못했습니다.
- r457이 같은 코너를 어느 모드로 통과했는지 확인하지 않았습니다.
- 로그의 `current_path_recheck_policy: selected_v1`이 두 `_gaps.py` 사본 중 어느 쪽에 연결되는지 확인하지 않았습니다.

## 7. 검토 파일

- `src/arena_autonomy/arena_autonomy/`
  - `pass_release_trial.py` (전체)
  - `structure_region.py`
  - `released_body_gap.py`
  - `current_path_recheck/_gaps.py`
  - `masked_motion_speed.py`
  - `static_side_return.py`
  - `known_structure.py` (`current_mask_shadow` 부분)
  - `local_follow.py` (`forward_planning_speed`)
- `src/arena_autonomy/arena_autonomy/race_runtime_policy/`
  - `current_region.py`
  - `released_plan.py`
  - `retained_recheck/_released.py`, `retained_recheck/_gaps.py`
  - `observation_policy.py`, `guard_observation_policy.py`
  - `released_relative.py`
  - `solo_gap.py`, `solo_relative.py`, `solo_crawl.py`, `solo_low.py`
  - `evaluator.py` (import 목록만)
- `artifacts/validation/`
  - `2026-11-14/released_structure_r468_v1/RESULT.md`
  - `2026-11-13/solo_full_lap_r465_v1/RESULT.md`
  - `2026-11-14/solo_full_lap_r469_v1/` 의 `RESULT.md`(준비 기록만 있음), `early_stop.json`, `run1/follow_decisions.jsonl`

이상으로 자문을 마치며, 새로운 작업으로 확장하지 않았습니다.