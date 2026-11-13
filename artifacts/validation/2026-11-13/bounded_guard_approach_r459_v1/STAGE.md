# r459 독립 stage — native 대조 전 검토본

후속 native 검증·최종 회수와 이관 검토는 [RESULT.md](RESULT.md)에 기록했다. 아래는 native 전 준비 시점의 설명이다.

공통 미이관이며 정책 기본값은 바꾸지 않았다. `selected_r181 + traffic_recovery_trial`에서만 현재 region/config/차체 치수를 공용 `observed_guard_approach`에 전달한다. original_v1/옵션 미제공은 기존 strict 출력·absent 조기 거절을 유지한다.

수정은 `pass_release_trial.py`, 두 `_present.py`, 두 `_guard.py` 및 신규 `test_bounded_guard_approach.py` 총 6파일이다. [changes.json](changes.json)에 원 SHA와 stage SHA, [review.patch](review.patch)에 최소 내용 diff, [apply.patch](apply.patch)에 실제 개행을 포함한 적용 diff를 둔다. 실제 이관 시 stage 사본 SHA를 기준으로 한다.

명시 proof는 원 region 통과·일반unknown0·전면mask0·기존1cm 예산 및 실제 query 수/분류를 대조한다. 각 실패 mask 표본도 현재 차체 앞뒤 범위 안, 옆면 바깥 기존1cm 이내여야 한다. 측방별 mask x-span에는 기존 `ACCEPTED_HIDDEN_WIDTH_M=.05`를 다른 기하 정의의 보수적 상한으로 재사용한다. free가 한 점 이상 있어야 하지만 이는 전체 가림이 아니라는 조건이지 연속 관측·숨은 물체 부재 보장이 아니다. 전후 free 개수는 진단만 남긴다. 가림은 `currently_observed`에 합산하지 않고 bounded_mask/unknown과 observed vs observed_with_mask_budget를 분리한다. 얇거나 뾰족한 물체가 허용 가림 쐐기에 숨는 위험은 남는다.

absent current는 현재 side present·입력/기하 검사 순서를 유지한 채 앵커 거절만 모으고 원 region 이후 공용 분류를 수행한다. rejected가 있으면 region/approach 실패의 외부 reason은 기존 `guard_rebase_toward_unreleased_side`여서 관측 정책의 낮은 속도 재검사 진입점을 유지한다. 새로운 속도 단계, 앵커 상승, 측방 상태 해제, world-fixed 판정은 없다.

Windows 관련 143검사를 통과했고, 최종 분류·실제 saved ray 반례의 신규 24검사가 통과했다(중복 포함 횟수는 합산하지 않음). 앞선 v1은 원 original_v1 범위까지 proof가 전달되어 기존 2검사가 실패했고, 신규 fixture padding 누락 2건도 있었다. v2의 원2실패는 selected_r181 gate로 고쳤으며 기존 기대값을 바꾸지 않았다. 원 로그 v1/v2를 보존한다. 최종 실제 측방/전방 유한 장애물·전방 unknown 반례는 두 current 계열의 명시 selected_r181에서 모두 0이다.

r456의 실제 정체 복제106개를 Windows Python으로 대조한 `local_windows_v1`은 기존 `.15` 단계106개 수용·owner 상태 불변이다. 이는 native 전체 plan/실제 주행 결과가 아니다. 그 뒤 outer selector reason을 가림 예산 수용으로 구분한 명칭으로 명확히 했으며 결정 기준은 같다. native381과 실제 후속 current 대조는 WSL 자원 배정 후 진행한다. r452 원 실패는 보존한다.
