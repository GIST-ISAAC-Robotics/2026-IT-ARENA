# r424 — r423 동일 scan current 연결 독립 검토

2026-11-13, GPT 정적 검토. **검토 범위에서 새 P1 기능 결함은 발견하지 않았다. 대표 입력의 current 거절이 유지되어 공통 미채택으로 종료하는 판단에 동의한다.** 임계값 변경이나 추가 실행을 제안하지 않는다.

검토 입력은 [r423 README](../crop_current_scope_r423_v1/README.md), [변경분](../crop_current_scope_r423_v1/source_changes.patch), [source ledger](../crop_current_scope_r423_v1/source_ledger.json), staged 8파일, 준비·비교 하네스 및 회수된 대표 결과다. bundle SHA256은 `ad95f7c6785accedf4dd7ed76a20b83c03699f201e642354a4a3823140b8803e`로 지정값과 일치했다. Windows 표준 라이브러리로 준비 manifest 26파일과 ledger의 staged/base 8행을 읽어 해시 불일치가 없음을 확인했다. 검토자는 제어기 import·시험·WSL·ROS·장치 호출을 하지 않았으며 r423/common을 수정하지 않았다.

## 연결과 기존 경계

- **실제 부모 입력에서 비교한다.** staged `runtime_dual_state.py:12`의 `current_request`는 원 `plan_reference` 축약을 유지한다. 실제 `offer_next_current`의 deep-copy된 `owned_plan['input']`/`owned`에서 전체 frozen scan 사전을 직접 비교한 뒤 compact 증거만 보낸다. 원 배열·비유한값 태그·취득시각·source frame/epoch/time basis가 비교에 포함된다. 이후 변경 가능한 외부 ROS bool을 허가로 읽는 연결은 없다. crop/current-corner OFF에서는 추가 IPC 키가 생기지 않는다.
- **선택과 허가를 분리한다.** `crop_current_scope.py:7,30,51`은 정확한 crop 표식·allowed·SOLO/baseline·원 timing과 내부 부모 증거를 분류에만 쓴다. `evaluator.py`의 기존 r396 환경검증이 native/verified simulation 선택을 제한한다. `partial_retry_scope`의 `allowed=True && fully_observed` 분기는 원 조건과 동등하다. 새 표식의 과거 flank/width는 현재 proof로 전달하지 않는다.
- **현재 입력의 검사를 유지한다.** `Evaluator.__call__`은 현재 reference·history로 scene을 생성한다. `solo_gap.rebase`/`solo_relative.rebase`는 현재 core를 다시 호출하고 현재 raw scan, 측정속도, 최종 조향과 `previous_steering_rad`, dt로 영역·공백을 검사한다. `corner_candidate`도 같은 scope를 받되 기존 실패 이유와 최종 현재 검사를 유지한다. 동일 raw scan이라는 사실로 motion/history/조향까지 같다고 가정하지 않는다. 원 epoch/side/source age·경로 0.5초·현재 명령/원천 만료·부모 최종 발행 검사는 변경분에 없다. 현재 raw 전체 공백의 `front_gap_outside_scope` 검사도 그대로다.
- **추가 보안 인증으로 해석하지 않는다.** compact 증거는 내부 부모/자식 호출 계약이다. 내부 worker 요청 전체를 위조한 상황까지 인증하는 장치가 아니며, README도 그 범위를 명시한다. 이번 frozen 경로에서 증거를 외부 입력으로 대체하거나 원시각을 새로 찍는 경로는 발견하지 않았다.

## 비교 하네스와 실행 증거의 범위

`prepare.py`는 r418의 고유 current 313행을 `(generation, sequence)`로 추출하고 nested plan/current와 outer 입력의 일치를 요구한다. 이 검사는 원 planner의 누락된 상태 이력 재현을 주장하지 않는다. `run_arm.py`의 candidate도 실제 부모와 같은 `current_request`를 사용하며, 축약 submitted에 원 scan을 추가하지 않는다.

`support.py:35,41`은 저장 결과의 `new_plan_deceleration_cap`을 제거하고 `checked_target_mps`를 복원해 **원 worker 생출력**을 비교한다. 저장된 부모 cap을 별도로 다시 적용한 결과도 원 기록과 비교한다. 이는 현재 common `_cap_ready`의 속도·진단 변경 구조에 맞는다. `execute_isolated.compare_pair`는 두 arm의 생출력을 비교한다. 따라서 원 부모 제한을 candidate 개선으로 세거나 생출력 oracle에 섞는 문제는 발견하지 않았다. 다만 저장 cap의 재적용은 부모 스케줄러 전체를 새로 실행한 증거는 아니다.

[대표 결과](../crop_current_scope_r423_v1/verify_v1/representative.json)와 해당 ZIP 안 arm 보고서를 읽어 다음을 확인했다. 실행은 주 에이전트가 수행한 결과이며 검토자는 재실행하지 않았다.

- 단위 단계의 `verification_passed=true`; 30개 통과는 회수된 실행 결과다. OFF 요청 구조와 fully gate 조건 검사는 있으나, 이를 313개 OFF/fully 전체 출력 회귀가 끝났다는 뜻으로 확대하지 않는다.
- trace seq1003/current index312에서 baseline raw oracle 및 저장 부모 cap 결과의 차이가 모두 null이다. 양 arm 입력 불변, 정책 호출 각 1회, 외부 프로젝트 import 없음, 자식 `exitcode=0/forced=false/alive=false`를 확인했다.
- candidate는 `crop_scope=true`이나 속도는 0을 유지했다. 0.15 검사는 `outside_small_body_side`이며 exterior `0.010632309368771894 m`; 0.10 검사는 `front_gap_outside_scope`이며 exterior `0.007390027845075989 m`다. 반환된 전체 gap은 indices113..118, 양 이웃 거리 `11.961338996887207/1.6969406604766843 m`다. 마지막 거절이 `outside_small_body_side`가 아니므로 거리 proposal의 기존 진입 조건은 거짓이다.
- 추가 retry 진단 두 개를 제외한 원 거절 필드는 보존됐다. source/preparation 변경 목록은 비어 있다. `representative_recovered=false`, `next_stage_allowed=false`, `full_comparison_completed=false`다.

따라서 이번 대표에서는 **새 연결이 전달되지 않아 정지한 경우가 아니라, 전달 후 기존 전체 공백 proof가 거절한 경우**로 구분할 수 있다. 이것은 숨은 장애물의 존재·부재를 판정한 결과가 아니며, 다른 312입력의 결과도 말해 주지 않는다. 조건부 전체 313 비교·새 주행·독립 guard/MCU 실시간 경로는 실행하지 않았다. r423의 공통 미채택과 기존 r418 정체 기록을 그대로 보존한다.
