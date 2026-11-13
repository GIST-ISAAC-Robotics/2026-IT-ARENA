# r423 동일 scan crop 계획의 현재 재검사 연결 후보

**준비 전용·공통 미이관·미실행.** r418의10.10m 정체/원 소스와 r421 감사는 보존한다. r422의 세 진입 게이트 설계를 구현하되 실제 current IPC 계약을 확인해 전달 방식을 좁혔다. WSL/ROS/Jetson/장치/새 주행은 이 준비 단계에서 실행하지 않는다.

## 변경 범위

staged8파일의 base/staged SHA는 [source_ledger.json](source_ledger.json), 부모 검토용 차이는 [source_changes.patch](source_changes.patch)다.

- `race_runtime_policy/crop_current_scope.py`: 새 표식의 후보 분류와 내부 동일 scan 증거 확인. 기존 fully 분기는 그대로다.
- `solo_gap.py`, `solo_relative.py`, `corner_candidate.py`:0.15→0.10→단일 거리 후보의 세 partial 게이트에 명시 scope만 추가한다. 기존 속도·기하·실패 이유·최종 proof는 변경하지 않는다.
- `solo_crawl.py`, `released_relative.py`: 새 keyword 전달만 한다. 자체 mask-only/RELEASED 조건은 그대로다.
- `evaluator.py`: 기존 r396 등록 환경과 crop flag를 사용해 scope를 한 번 분류하고 명시 전달한다. 새 header/default/운영 옵션을 만들지 않는다.
- `runtime_dual_state.py`: 실제 `offer_next_current`가 사용하는 `current_request`를 추출한다. 부모가 소유한 frozen plan/current scan 전체를 직접 비교하고 선택 ON에만 compact 증거를 보낸다.

현재 core·scene·두 조향 union·motion_region·전체 raw 공백 proof·guard는 원문 그대로다.100°/10mm/6°/50mm·물성·TTL·측정속도·원 취득시각을 바꾸지 않는다. `front_gap_outside_scope`이면 그 거절을 보존한다. wrapper나 새 속도 상수를 더하지 않는다.

## 초기 설계에서 바뀐 전달 방식

[r422](../crop_current_design_r422_v1/REVIEW.md)는 child의 submitted에 전체 scan이 있다고 가정한 첫 설계다. 구현 전에 실제 `runtime_dual_process.plan_reference`가 representation/source_epoch만 남기는 것을 확인했다. 하네스에서만 submitted 전체를 복원하면 실제 운영 연결과 달라지므로 그렇게 하지 않았다.

부모의 `owned_plan['input']['scan'] == owned['scan']`를 원 frozen 사전에서 직접 계산한다. NaN/+Inf/−Inf 태그, 원 배열과 metadata를 모두 비교하며 scan 중복 IPC/매 호출 JSON hash는 없다. `crop_current_input`의 schema·same_scan·두 reference·submitted verified/time policy/time basis만 추가한다. 원 축약 submitted는 그대로다. OFF의 request 사전은 기존 구성과 exact 동일하다.

이 증거는 **내부 부모 소유 입력 동일성**일 뿐이며 외부 ROS 메시지에서 받는 필드나 점유/주행 허가가 아니다. child는 기존 r396 환경검사와 현재 native simulation/verified-only 입력을 확인한다. partial의 정확한 reason·allowed·scope와 비어 있지 않은 인덱스 표식은 후보 분류에만 쓰며 과거 flank/width를 현재 proof에 쓰지 않는다. 악의적으로 내부 worker 입력 전체를 조작한 상황을 인증하는 보안 체계는 아니다.

단일 scan 한정은 r421 후반99/99에 해당한다. 전체313에서는305개이며 나머지8개는 새 scope 대상이 아니다. 새 scan이 도착하면 같은 실제 성분이어도 이번 확장에 진입하지 않을 수 있다. 이를 일반 실물 운영 계약으로 채택하지 않는다. reference/history/measured/previous steering은 plan과 같다고 요구하지 않으며 현재 좌표와 D(measured)는 다시 검사한다.

## 동결 입력과 비교 순서

원 `r418/run1/follow_decisions.jsonl` SHA256은 `67bd68142f4d0596ed60bd89fe280fd5b9a3e60e79802d3d7d0d1d6da6ba30da`이다. r421의 동결 receipt와 nested plan/current 입력 동일성을 확인해313고유 current를 원순서로 추출한다. plan 상태 이력 전체는 재생하지 않는다. baseline source는 r418 source archive의 정적 import 폐포이며 준비 시 현재 common과 같은지 확인한다.

1. **단위검사:** 새 scope/falseallowed/generalbounded/거짓 표식, 원 scan·metadata 차이, timing 보존, OFF/fully 선택 불변을 검사한다. 실제 `RuntimeDualProcess.offer_next_current`를 FakeLane으로 호출해 부모가 만든 true/false evidence와 원본 불변을 확인한다. 표식을 주행 허가로 세지 않는다. 변경하지 않은 full-proof 정책의 합성 대조 전체나 r417 plan184검사를 반복하지 않는다.
2. **대표1회:** trace seq1003, plan(361,649)→current(361,650) 한 건에서 baseline raw evaluator 출력과 원 checked target을 먼저 exact 대조하고, 부모 cap 후 원 기록도 별도로 비교한다. baseline 실패 시 candidate를 실행하지 않는다. 이어 staged candidate를 한 번 호출한다. 실제 부모와 같은 `current_request`를 사용하며 입력의 유일한 확장은 부모가 계산한 compact 증거다.
3. **조건부 전체:** 대표가 새 scope에서 실제 양수로 회복하고 baseline/입력/회수 검사가 통과했을 때만 나머지312건을 원순서로 각 arm1회 비교한다. 대표 결과는 재사용해 합계313건이며 같은 대표를 다시 실행하지 않는다. 비대상은 전체 출력 exact, 거절은 원 필드 불변, 회복은 원 경로·조향·출처/시각·현재 proof·독립 guard 조건을 대조한다. 대표가0이면 나머지는 호출하지 않는다.

0.15/0.10/거리 proposal/final proof는 **반환된 원 진단**을 저장한다. 후속 성공 반환이 이전 실패 진단을 보존하지 않는 기존 구조에서는 없어진 중간 이유를 만들어 채우지 않는다. `before_corner`와 최종 출력도 모두 남긴다. 계측 예산용 Lane timeout은20초·phase 전체180초이며 실제 명령 TTL/실시간 합격 판단이 아니다. 성능 미세 계측은 추가하지 않는다.

## 실행 담당 고정 명령

WSL x86_64에서만 아래 순서로 실행한다. 실제 ROS 노드/센서/시뮬/장치를 시작하지 않는다. 새 root와 새 receipt가 필요하며 같은 결과 경로로 재실행하지 않는다. root의 `src/`/입력은 `bundle_manifest.json`, 준비 폴더는 `preparation_manifest.json`으로 전후 대조한다. inherited PYTHONPATH는 isolated 경로 뒤에 유지한다.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
python3 -B artifacts/validation/2026-11-13/crop_current_scope_r423_v1/execute_isolated.py --stage units --root "$HOME/it-arena-r423-crop-current-a" --receipt artifacts/validation/2026-11-13/crop_current_scope_r423_v1/verify_v1
python3 -B artifacts/validation/2026-11-13/crop_current_scope_r423_v1/execute_isolated.py --stage representative --root "$HOME/it-arena-r423-crop-current-a" --receipt artifacts/validation/2026-11-13/crop_current_scope_r423_v1/verify_v1
python3 -B artifacts/validation/2026-11-13/crop_current_scope_r423_v1/execute_isolated.py --stage conditional-full --root "$HOME/it-arena-r423-crop-current-a" --receipt artifacts/validation/2026-11-13/crop_current_scope_r423_v1/verify_v1
```

각 명령의 exit0과 stage `verification_passed=true`를 확인한 뒤 다음 명령으로 간다. units는 native1회 빌드(최대120초)·동일 binary의 양 arm 복사·좁은 단위검사(최대120초)다. representative는 baseline/candidate 각 current1회·arm별 최대240초이며, 내부 Lane은 정상 close·exit0·forced=false·alive=false를 요구한다. `representative_recovered=false`/`next_stage_allowed=false`면 원0출력과 proof 이유를 회수하고 종료한다. conditional-full을 호출하더라도 이 경우 제어 호출 없이 skipped를 기록한다.

조건부 전체는 baseline312 exact 확인 후 candidate312를 검사한다. 이 단계가 끝나도 저장 비교 완료일 뿐 실주행/실물 검증이 아니다. 모든 단계는 `verify_v1/{stage}.json`과 `{stage}.zip`에 원 행·예외·회수를 보존한다. 실패 후 source/조건을 고쳐 같은 경로에 덮어쓰지 않는다. baseline 정확성·후보 회복·비대상 불변·정상 종료를 하나의 성공 숫자로 합치지 않는다.

원시 입력/상세 행/ZIP은 기존 Git 제외 정책을 따른다. 새 common 통합·실제 주행은 주 에이전트의 별도 판단 사항이다.
