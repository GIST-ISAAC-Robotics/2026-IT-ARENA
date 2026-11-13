# r423 동일 scan current 연결 대표 결과

**현재 저속 회복의 진입 단절은 연결됐지만, 최종 공백 proof에서 여전히0m/s다.** 단위30검사와 원 baseline exact·입력/소스 불변·정상 회수는 통과했다. 후보의 주행 회복은 실패했으며 **조건부313개 비교와 새 시뮬레이션은 실행하지 않았다.** 공통 코드도 변경하지 않았다.

근거는 [단위 영수증](verify_v1/units.json), [대표 영수증](verify_v1/representative.json), [원 행·자식 종료·로그 ZIP](verify_v1/representative.zip)이다. 이 결과 문서만 추가하며 기존 준비물·manifest·원 실행 결과는 수정하지 않았다.

## 실행과 정확성

- 실행 root: `/home/jinhyeong/it-arena-r423-crop-current-a`. native1회 빌드 exit0, 양 arm의 binary SHA 동일. JUnit은 **30 passed, failures/errors/skipped0**이다.
- 원 r418 trace seq1003/index312, plan(361,649) reference39.420초 → current(361,650) reference39.422초의 한 입력을 각 arm1회 평가했다. 원 계획 목표0.5m/s, 현재 측정속도0m/s다.
- baseline의 raw evaluator 출력/checked target과 원 부모 cap 후 기록 대조는 모두 exact였다(`raw_expected_diff=null`, `recorded_parent_diff=null`). 원 baseline 결과는0m/s·`route_current_large_motion_region_unobserved`다.
- candidate의 부모 compact evidence는 `same_scan=true`, 원 submitted timing verified/verified_only/simulation_acquisition을 보존했고 **`crop_scope=true`**였다. 양 arm 모두 부모·child 입력 불변, 외부 project import0, top current policy 호출1회를 기록했다.
- candidate는 원 반환 필드를 유지하고 `current_solo_gap_retry`, `current_relative_body_retry` 두 실패 진단만 추가했다. `before_corner`와 최종 출력은 exact 동일하다. `verification_passed=true`는 이 비교/회수 계약 통과이며 양수 회복을 뜻하지 않는다.

## 실제 거절 순서

| 단계 | 저장 결과 |
|---|---|
| 원 current | 거리0.221500m, 전체3,663질의·unobserved794, 최종0m/s |
|0.15m/s 재검사 | 전체1,244질의·unobserved294. 차체 옆 외곽 **10.632309mm > 기존10mm**로 `outside_small_body_side`, applied=false |
|0.10m/s 재검사 | 전체618질의·unobserved146. 외곽은 **7.390028mm**이나, 전체 raw gap **113..118**에서 `front_gap_outside_scope`, applied=false |
|0.10의 이웃/공백 | 실제 두 flank 거리11.961338997m /1.696940660m, gap0.0879658235rad. 상대 이웃 최소거리0.198951790m, query reach0.083951790m + padding0.115m를 기록. 먼 flank나10mm 이내라는 사실로 전방 제외 검사를 대체하지 않음 |
| 거리 proposal | **미진입.** 기존 eligible은0.10 proof의 `outside_small_body_side`를 요구하지만 실제 이유는 `front_gap_outside_scope`다. proposal/final proof/final region은 null이며 새 목표 검사도 수행하지 않음 |
| 최종 |0m/s, `route_current_large_motion_region_unobserved`, route accepted=false. source timestamp renewal/old occupancy permission reuse=false |

따라서 이 대표의 남은 차단은 미진입 조건 자체가 아니라 **연결 후 실행된 현행 전체 raw gap의 전방 제외 계약**이다.100°/10mm/6°/50mm나 속도·TTL을 바꾸지 않았으며, 통과시키기 위한 후속 구현을 자동 시작하지 않는다. 다음 설계 판단은 이 원 실패를 근거로 주 에이전트가 수행한다.

## 종료와 실행하지 않은 범위

baseline/candidate 각각 completed=expected=1, error=null. 두 current Lane 모두 **exitcode0·forced=false·alive=false**였다. 외부 실행 자식도 returncode0·reaped=true·forced=false다. 두 단계의 source_changes/preparation_changes는 빈 목록이며 로컬 준비 manifest 재대조도 변화0이다.

`representative_recovered=false`, `next_stage_allowed=false`, `full_comparison_completed=false`로 종료했다. `conditional-full` 영수증/행은 없고 나머지312개를 호출하지 않았다. 이번 것은 저장 current 한 건의 비교이며 새 ROS/guard/MCU/물리 정지 시험, 성능 계측, 전체랩 또는 실물 적용 검증이 아니다. r418 원 주행 실패도 그대로 남는다.

## 확인한 SHA256

| 자료 | SHA256 |
|---|---|
| bundle_sources.zip | `ad95f7c6785accedf4dd7ed76a20b83c03699f201e642354a4a3823140b8803e` |
| bundle_manifest.json | `713c5e6ce6e510841ac76f4974a922803b3f8ade188d30bac2f664f9cfd64031` |
| preparation_manifest.json | `f35c1a2247ffeb8e4e78818baee610e86c13725570bd152f08bc4873cd9c6a18` |
| verify_v1/units.json | `7c80d73bbe3ee3e3a682c86403a702f0735a66fcc27f547297b295dc99ec9536` |
| verify_v1/units.zip | `4a8f3a359bbcd1babf0a820f884f110a3288755ef04cfd1390ef823ea4d363af` |
| verify_v1/representative.json | `17d9c4faa1486d41511d591d32b9a9a79dbdb33ef254f062f0f8b0480f64ae14` |
| verify_v1/representative.zip | `d4a742ea88561f0a6abf93582aed897df4f58182194800464beb4604eade9959` |
| 양 arm 공통 native binary | `b0425c6f6228016da8201477277558812923ae9ffa666ac2fbe057e15ae4b550` |
| 대표 원 입력 묶음 | `900289908cfc053d245455e7f93ed9e2a41b93b2820931e3506f18176137b24c` |

대표 ZIP 내부 report와 외부 representative.json의 값도 일치했다. RESULT.md는 실행 뒤 추가한 요약이며 원 준비 manifest의 동결 목록을 변경하지 않는다.
