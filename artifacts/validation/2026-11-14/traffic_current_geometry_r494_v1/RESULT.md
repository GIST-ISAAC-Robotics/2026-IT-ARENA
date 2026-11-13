# r494 현재 점유 경로 비교로 추월 유지·중심 복귀

**공통 이관 후속:** r492 단독 변형의 정상 종료·소스 회수 뒤 주 담당이 생산2파일을 공통에 이관했다. [적용/원 바이트](integration_v1/applied.json)를 보존했고, r496의 [공통 native 영수증](../traffic_current_geometry_r496_v1/native_v1/receipt.json)에서 실제 공통 import·stage 내용 동일·기존 native 활성과 **166 관련 회귀 통과**를 확인했다. 아래8검사는 이관 전 저장 입력/반례 검사다. 원 r490 실패를 바꾸지 않으며 r496 실제 교통 주행은 별도로 수행한다.

아래 내용은 이관 전 stage 생산2파일의 검토 근거입니다. `selected_r181 + traffic_recovery`에서 **현재 벽으로 재확인한 guide**의 경로·실제 반환 점유/제동에 집중하도록 바꿨습니다. 옛 조향을 끝까지 유지하는 별도 전체 제동 곡선과 전역 관측 veto를 이 경로에서 제거했습니다. 현재 조향 변화율 제한·원 측정속도·전체 실제 반환·padding·제동식·STOP은 유지합니다. adviser/WSL/Jetson/ROS/실차는 사용하지 않았습니다.

## 원인 근거

`baseline_geometry.json`은 원소스와 r490 저장입력의 국소 기하 계산이며 원 출력도 `baseline_geometry.log`에 보존했습니다.

| call seq / 기준시각 | 현재 유지 조향 clearance | 직전 조향 clearance | 새 중심 조향 clearance | 원 중단 근거 |
|---|---:|---:|---:|---|
| 525 / 10.92s (runtime73) | .56m | .44m | 2.5m | 옛 조향 .44m가 유지·복귀 후보 모두 제한. ahead-clear=false |
| 528 / 11.02s (runtime74) | .48m | .48m | 2.5m | 원 유지 target .9m/s인데 전역 unobserved5로0. 중심도 unobserved2로 거절 |
| 567 / 12.32s (runtime87) | .04m | .04m | .08m | 현재 직접 점유/제동 정지. 전역 unknown이 원인이 아님 |

525의 target rear-bound .1499m와 요구 .1635m를 바꾸지 않았습니다. 이 숫자로 두 현재 경로의 비교 자체를 막는 구조를 선택 정책에서 제거했습니다. 567은 현재 경로 자체 .04m여서 그대로 정지합니다.

## 두 파일의 변경

`bypass_entry_trial._compare_entries`: 현재 guide/reference가 `current_outer_wall_rechecked_for_remembered_guide`이며 기존 `enabled(config)`가 참인 경우만 새 정책을 적용합니다. 초기 진입·default 비교는 기존 판정입니다. 동일 현재 명령의 `swept_limit`은 모든 반환과 시차 padding으로 계산하고, 직전 조향은 slew 제한에 유지합니다. 별개로 직전 원호를 끝까지 진행한다는 가정으로 교정 명령을 막지 않습니다.

현재 관측 벽 경로와 실제 점유/제동을 검사한 후보에 큰 이동 영역의 완전 관측을 다시 요구하지 않습니다. 이를 `unobserved=0`이나 free 증거로 위장하지 않고 `motion_region.evaluated=false`, `global_visibility_proof_required=false`, `unknown_filled=false`로 명시합니다. 기존 `entry_measured_envelope`와 pursuit에는 **원 측정속도**를 그대로 전달합니다.

`MovingPassGuide._current_follow_return`: 현재 대상 반환을 계속 요구하되 ahead-clear 숫자 문턱 없이 중심 후보를 비교합니다. 현재 두 경로의 **실제 점유 접두 진행량**을 기존 2cm 격자와 작은 편이 순위로 비교하고 중심이 더 낫고 양수 제동 명령이 있을 때만 상태를 바꿉니다. target 소실을 free로 바꾸지 않고 실제 기둥/다른 반환을 삭제하지 않습니다. 중심 follow headway에는 `min(현재 명령 swept clearance, 현재 중심 경로의 실제 occupied prefix)`를 기존 standstill/headway/제동식에 넣습니다. 기존 `preview_clearance`의 x 진행량이며 곡선 호길이를 더 길게 계산하지 않습니다.

새 `returning_to_current_follow`는 앞차 뒤라는 미확인 위치 관계를 주장하지 않으며 기존 `returning_behind`와 구분합니다. centered 뒤 준비한 FOLLOW/monitor를 넘기고 **그 FOLLOW의 STOP을 그대로 반환**합니다. 인계 사유는 `returned_to_current_follow`, 추월 완료는 false이고 reset 시 새 상태도 지웁니다.

## 검증

`pytest_final.log`: 신규4 + 기존 `test_moving_pass_current_return.py` 4 = **8 passed**. 첫 실행은7통과/1실패였고, default fixture가 original_v1으로 바꾸면서 의존 선택 released_body_gap_trial을 켜둔 설정 오류였습니다. fixture의 의존 선택만 끄고 같은 작은 묶음을 확인했으며 생산 판정선은 바꾸지 않았습니다.

`saved_comparison.json`: 원 scan·signed motion·기준시각·직전 조향, 원 tracker 현재 관측 및 guide 상태에서 시작한 국소 비교입니다.

| 원 call seq | 원 출력 | 후보 출력/상태 |
|---|---:|---|
| 525 | .871706m/s | .9m/s, 기존 편이 경로 유지 |
| 528 | 0 | .545935m/s, 중심 복귀. 중심 occupied prefix .596748m > 유지 .527968m |
| 531 (후속 원 입력) | 원 .809029m/s 추월 | 같은 새 복귀 경로 .540541m/s 유지 |
| 567 | 0 | 0 유지, 현재 clearance .04m의 실제 점유 정지 |

528의 실제 측정 .918376m/s를 목표 .545935m/s로 대체하지 않았습니다. 다음531도 원 실행의 운동 입력으로, 후보 명령을 적용한 폐루프가 아닙니다. 원 r490 실패를 성공으로 소급하지 않습니다.

`occupied_counterexamples.json`: 전방 물체와 **측방 차량이 복귀 곡선을 가로지르는 경우**를 현재 유효 광선에 ray cast했습니다. 각각101/84개의 합성 반환이 생겼고 중심 후보 clearance0/target0으로 거절됐습니다. unknown 광선은 채우지 않았습니다. 대상 미관측이면 새 중심 복귀를 시작하지 않으며, centered 인계에서 준비된 FOLLOW가0이면0을 유지하는 검사도 통과했습니다.

`default_exact.json`: 비선택 원본/후보 3대표의 전체 비교 출력 **허용오차0 동일**입니다. 기존4회귀도 기본 설정의 target 조건·headway 순서·인계/reset을 확인했습니다. Windows NumPy2.5의 기본2D cross만 이전 r486 runner와 같은 determinant 식으로 호환했으며 호출수/범위는 `test_receipt.json`에 있습니다. 생산 NumPy·수치 정밀도는 바꾸지 않았습니다.

## 한계와 이관

현재 곡선과 일부 반환은 전체 가시성 증명이 아닙니다. 이전에 보이지 않은 물체·숨은 차체 일부·급격한 상대 운동과 실제 조향 응답 지연은 남습니다. 현재 점유/제동·독립 guard를 유지하지만 저장 국소 비교만으로 접촉0·지속 복귀·완주·실차 안전을 주장할 수 없습니다. 늦은 직접 점유567은 계속 정지합니다.

`source_manifest.json`에 생산2파일 before/candidate SHA, 원입력 SHA 및 공통 원본 불변을 기록했습니다. 검토 대상은 `candidate.patch`/stage이며 공통 이관은 root 소유입니다. 자기 소유 실행 PID는 없습니다. 원 r490 실패와 시험 소스/로그를 보존했습니다.

```powershell
python artifacts/validation/2026-11-14/traffic_current_geometry_r494_v1/run_tests.py
git apply --check artifacts/validation/2026-11-14/traffic_current_geometry_r494_v1/candidate.patch
# root beforehash 재대조·검토 후 이관
```
