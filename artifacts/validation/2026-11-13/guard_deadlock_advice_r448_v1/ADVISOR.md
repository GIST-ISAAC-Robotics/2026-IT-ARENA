# Fable 5.2/high 자문 원문 — 공개 문서 호칭만 정리

r441 자문 결과를 보고드립니다. 이 정체는 더 낮은 목표 속도 하나로 벗어날 국소 문제가 아니라, 두 가지 구조적 원인이 맞물린 결정적 교착으로 판단합니다. 다만 "과거 기억" 문제는 아니며, 위협은 매 스캔 현재 관측으로 다시 세워지고 있습니다. 코드와 문서는 수정하지 않았고 실행도 하지 않았습니다.

## 관측 (저장 로그·코드에서 직접 확인)

- **경위**:
  1. 33.42 s에 우측 4점 반환으로 GUARD에 진입해 1.04 m/s에서 목표 .35로 감속했습니다.
  2. 33.62 s에 같은 반환이 `world_fixed_return: true`로 판정되어 SOLO, 1.0 m/s로 복귀했습니다.
  3. 33.72–33.82 s에 selector가 아닌 기존 G1이 `follow_boundary_unconfirmed` HOLD를 2주기 내어 완전히 정지했습니다.
  4. 33.92 s에 정지 상태에서 좌 3점·우 4점이 다시 present가 되어 GUARD로 재진입했습니다.
  5. seq 892에서 .15가 한 번 수락되었고(tail .957 cm), 실제 발행은 .10이었습니다.
  6. seq 895–998의 104행은 모두 거절되었습니다.
- **baseline은 정상**: 104행 모두 baseline 경로가 있고 `fully_observed`, `boundary_from_memory=false`였습니다. 처음과 마지막 행의 hold 전 목표는 .70 m/s였습니다. 차를 세운 것은 selector GUARD 검사뿐입니다.
- **tail은 조향의 함수**: .15 후보의 tail은 조향각에 단조로 따라갑니다(.347 rad → 1.065 cm, .404 rad → 1.276 cm). `unobserved`와 front mask는 0이고 previous 조향은 .30으로 고정되어 있었습니다.
- **좌측이 통과 상태여도 거절**: 좌측이 passed·not present인 28행에서도 `large_motion_masked_toward_left`로 거절되었습니다.
- **반환은 정지해 있음**: 우측 반환은 104행 내내 present였고 front ds는 .015–.038 m였습니다. 좌측은 2↔3점으로 깜박였고 front ds는 .310–.340 m였습니다.
- **정지 중에는 해제 불가**: `StaticSideTracker`는 window 8행 안에서 자차 이동 .12 m 이상을 요구하므로 정지 중에는 world-fixed 판정이 나올 수 없습니다.
- **같은 규칙의 공유**: `region_blocked`의 1 cm 규칙은 GUARD plan, RELEASED, current 단계가 함께 씁니다. r353, r364, r437도 같은 계열의 정체였습니다.

## 추론

- **R1, 순환 의존**: 현재 보이는 정적 반환을 위협에서 빼는 유일한 수단이 자차 이동을 요구합니다. 10 Hz·8행(0.7 s)에서 .12 m는 평균 약 .17 m/s가 필요하므로, .10/.15 crawl로는 움직이더라도 해제에 도달하지 못합니다.
- **R1의 근소한 누락**: 33.62 s의 world-fixed 근거는 HOLD 2주기(0.3 s 공백 > `maximum_gap_s` .25) 때문에 지워진 것으로 보입니다.
- **R2, 기하 불일치**: 1 cm tail 규칙은 최종 차량(축거 .128 m, ±123° 가림 폭 7.26°)에서 조향 약 .33 rad 이상·정지 출발이면 .15 m/s에서도 만족할 수 없습니다. 장면과 무관하므로 단계별 저속 cap을 추가해도 같은 정체가 다른 단계에서 반복됩니다.
- **방아쇠**: 정지를 만든 것은 코너의 G1 2주기 HOLD입니다. 이것을 오늘 새로 손대는 것은 권하지 않습니다.

## 권고 (추천 순서)

1. **정지 중 world-fixed 판정 추가 (R1 해소, 1순위)**
   - `StaticSideTracker`에 대안 조건을 둡니다. window 전체(8행, 0.7 s 이상) 동안 기존 일관성 식이 허용오차 .05 안에 유지되면 world-fixed로 인정하고, rows가 지워질 때까지 유지합니다.
   - 미관측을 시간으로 해제하는 것이 아니라, 현재 관측된 반환이 정지해 있다는 판정입니다. 경로 점유 검사, 독립 guard, 1 cm 규칙, unobserved 처리는 그대로입니다.
   - 정체 행마다 GUARD 전체 검사 3회가 SOLO 경로로 바뀌므로 계산 비용은 줄어드는 방향입니다.
2. **masked tail 허용의 기하 재유도 (R2, 조건부)**
   - 1번 적용 뒤 current 단계에서 같은 tail 거절(r437형)이 나올 때에만 적용합니다.
   - 근거는 ±123° 쐐기의 폭입니다. lidar 0.2 m 이내에서 약 2.5 cm 이하로, 기존 `accepted_hidden_width_m` .05와 규정 차폭 .10보다 좁습니다.
   - 최소 구현은 상수 하나(.01 → 약 .015)이지만 `masked_tail_allowance_m ≤ small_excursion_m` 검증에 걸리고 판정선이 바뀝니다. GPT 주 담당과 사용자의 결정 사항입니다.
3. **GUARD .1 cap (보조)**: 단독으로는 권하지 않습니다.

## 핵심 반례

- **.1 cap 단독**:
  - .1에서의 tail은 약 .68–.96 cm로 추정되며(재계산하지 않음), 조향 .40 부근은 경계입니다.
  - 통과하더라도 좌회전으로 좌측 extent가 anchor보다 1 cm 넘게 커지면 접근 표본 전부의 자유 관측이 요구됩니다. masked-left 수백 표본 때문에 수 cm 뒤 다시 거절될 것으로 예상합니다.
  - crawl 속도로는 static 해제에도 도달하지 못합니다.
- **1번 대안**:
  - 옆에 정지해 있는 상대 차량은 벽과 구별되지 않아 SOLO로 풀립니다.
  - 그 차가 움직이면 이동 5 cm 이내에 불일치로 rows가 지워져 다음 스캔에 GUARD로 돌아가고, 유한 장애물 검사와 독립 guard는 남습니다.
  - 그래도 판정선 변경이므로 승인이 필요합니다.
  - 좌측의 2↔3점 깜박임이 rows 공백을 만들 수 있으므로 저장 재생에서 먼저 확인해야 합니다.

## 계속 보완할지의 판단 기준 (질문 3)

- 1번의 저장 진단이 통과하고 제한 대표 1회에서 20 m 지점을 넘으면 복구 체인을 유지합니다.
- 다시 mask tail 계열로 서면 2번 한 가지만 추가합니다.
- 그 뒤에 새로운 종류의 정체가 나오면 체인 보완을 멈추고 기존 단순 구성의 fallback 확인으로 전환합니다.
- 원인 두 개에 수정 두 개까지가 상한이라고 봅니다. fallback의 구체 구성은 이번 자문에서 특정하지 못했습니다.

## 검증과 기록

- **오늘은 helper 진단이 합리적**입니다. tracker의 상태는 (stamp, front_ds, travel)뿐이므로, 저장된 104행만으로 selector prefix 없이 단독 재생할 수 있습니다. 전체 prefix 재현에 시간을 더 쓰는 것은 권하지 않습니다.
- **단일 제출 기록으로 충분**하다고 봅니다. `offer` 성공 시점의 input + generation/sequence/reset_reason이면 됩니다. 조건은 두 가지입니다.
  - 채택 시점이 아니라 제출 시점에 기록해, 부모가 버린 결과의 입력도 남겨야 합니다. r445의 누락 원인이 이것입니다.
  - reset이 같은 순번 흐름에 남아야 합니다.
- 기본 OFF와 "원 출력 exact 재현 후에만 후보 비교" 관문은 그대로 두시면 됩니다.

## 검토한 파일

- **전체 열람**: `pass_release_trial.py`, `static_side_return.py`, `race_runtime_policy/observation_policy.py`
- **부분 열람**: `m3_execution_trial.py`, `lateral_intent.py`, `docs/PROJECT_CONTEXT.md` 상단
- **r441 산출물**: `RESULT.md`, `analysis_v1/guard_followup.json`, `analysis_v1/guard_original_rows.json`, `run1/follow_decisions.jsonl`(seq 820–998), `run1/lidar_blind_sectors.json`
- **grep만**: `released_plan.py`, `crop_current_candidate.py`, `solo_crawl.py`, `current_region.py`, `corner_budget.py`, `masked_motion_speed.py`, `retained_recheck/`

## 증거 보고

- **결과**: 자문 완료. 변경 파일은 없습니다.
- **수행한 확인**: Windows에서의 파일 읽기·grep과, 저장된 JSON/JSONL을 Python으로 읽기 전용 집계한 것뿐입니다. WSL, 시뮬레이션, ROS, 후보 재계산, 하위 에이전트는 사용하지 않았습니다.
- **미확인 항목**:
  - .1 m/s에서의 실제 tail 값(추정치).
  - current 단계 코드 본문. 1번 적용 뒤 current 단계의 거동은 추론입니다.
  - 정적 반환의 물리적 정체. 코너 벽이라는 것은 추론입니다.
  - 좌측 깜박임이 tracker rows에 주는 영향.
  - Jetson에서의 비용.
  - r445 산출물은 열람하지 않았고 전달해 주신 내용만 사용했습니다.
- **가정**: tracker의 travel이 실측 속도 적분이라는 전제, 그리고 .68–.96 cm 추정에 쓴 envelope 적합식(d ≈ .317v + .249v²)입니다.
