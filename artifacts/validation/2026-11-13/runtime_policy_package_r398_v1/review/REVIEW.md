# r398 독립 검토 — 실행 전

현재 판단: **공통 이관 전 제한 차등검사 준비 완료. 새 P1 알고리즘 결함은 정적 검토에서 찾지 못했다.** 아래 검사는 아직 실행하지 않았다. `static_review.json`은 Windows에서 파일 바이트와 AST만 읽은 결과이며 정책 실행 결과가 아니다. 공통 소스와 staged 정책 본문은 이 검토에서 수정하지 않았다.

## 확인한 범위

- ledger 17개 원본의 SHA와 staged SHA를 확인했다. 모든 함수·클래스 AST는 import 재배치 외 같고, `_geometry.rebase`의 명시 시각 정책 수용 부분만 의도된 차이다. 그 파일은 해당 hunk를 되돌리면 원 파일 전체가 같다. `prepare_policy.py`는 재실행하지 않았다.
- 계획은 원 r278 `install()` 뒤 r361 `enabled()` 순서다. 현재 체인은 r375→r370→r352→r346→r343→r281→r266이며 `core_policy`의 연결도 같은 r266 `observation_policy`에 도달한다. 두 체인은 동일한 현행 공통 구현을 참조한다.
- `runtime_time_policy`와 입력의 `timing_policy`가 다르면 계산 전에 거절한다. `host_estimated_v1`은 원 `timing_verified=False`를 보존하며 `unverified_estimate`, 현재 호출의 `MotionHistory.scan_time_policy`와 일치해야 한다. 실제 취득 시각 정확성을 얻었다는 의미는 아니다.
- `restore_call`은 저장된 wheel/gyro의 이미 환산된 값을 그대로 복원한다. 과거 wheel unit을 재환산하지 않는다. r377에 없던 track/radius/wheel width는 같은 final YAML 변환으로 **양쪽 header에 동일하게 명시**한다. 따라서 과거 r377 출력이나 r386 final 출력과의 비교를 이번 이관 동등성으로 혼동하지 않는다.
- 새 모듈의 `supported()`는 재검사 범위이며 출력 허가가 아니다. current 결과의 `runtime_authorized=False`와 `independent_guard_required=True`는 유지된다. ROS backend, 보호층 발행 경계, 실물 입력의 실제 품질·실물 구동은 이번 준비 범위에 없다.

## 통합 시 확인할 한 가지

검토 당시 공통 `runtime_dual_process._worker`는 `finally`에서 pipe만 닫으며 evaluator의 `close()`를 호출하지 않는다. 각 Lane 자식이 종료되므로 자식 내부 plan patch가 부모로 누출되는 P1 결함은 아니지만, `Evaluator.close()`가 존재한다는 사실만으로 정상 종료에서 명시 복구를 확인했다고 쓰면 안 된다. 공통 이관 담당에게 생성 성공한 evaluator의 선택적 `close()`를 finally에서 호출하는 최소 연결을 권고했다. 이 검토는 공통을 수정하지 않았다.

`Evaluator`의 직접 `close()`는 r361 wrapper 복구 후 r278 원 함수를 되돌리며 반복 호출을 허용한다. 직접 close 및 controller 생성 실패 뒤 복구를 검사하도록 준비했다. 한 자식에는 한 stage evaluator만 두는 현재 계약을 유지하며, 동시에 중첩된 plan evaluator를 지원한다고 확대하지 않는다. 잘못된 원시 schema/정책은 zero 결과 또는 worker 실패로 거절될 수 있다. 이를 허가된 결과로 바꾸는 fallback은 없다.

## 차등검사와 한계

`compare_spawn.py`는 실제 `runtime_dual_process.Lane`의 spawn을 사용한다. 한 번에 자식 하나만 띄우고 원 체인/새 체인을 순차 실행한다.

| 묶음 | 내용 |
|---|---|
| 계획 prefix24 | r377 완료 기록에서 `(generation, sequence)` 중복 제거, 처음24개. 12/24까지 전체 판단·선택기 전체 상태·경로/정지/재개 상태 exact 비교 |
| 현재223 | 기록의 `current_rebase.compute.plan` 원 출력과 `plan_reference(plan.input)`, 현재 입력을 그대로 양쪽 체인에 전달하여 전체 결과와 상태 exact 비교 |
| 같은 숫자·다른 시각 정책 | 계획24개 + current 0/60/119/123/125/222. `timing_verified True→False`, 명시 policy와 raw `time_basis`만 변경. 모든 숫자와 이력은 exact 불변 |
| 제한 반례 | 시각 정책 불일치, sim false+verified_only, stale scan, wheel history 누락, 전부 unknown, epoch 변경, history 정책 불일치, 직접 close/실패 복구 |

기존 공통 `comparable_decision`이 지정한 surface-fit `processing_ms` 두 위치만 0으로 정규화하며 원값은 JSONL에 보존한다. 그 밖의 부동소수·타입·키·출력과 상태에 허용오차를 두지 않는다. current 223개는 저장상 모두 SOLO이므로 이번 표본을 RELEASED/GUARD의 주행 검증이라고 표현하지 않는다. 나머지 정책의 함수본문 보존은 정적 근거다. 원 실행에서 기록되지 않은 계획이나 전체 스케줄을 복원하지 않으므로 전체 원 주행 재현도 아니다.

처리시간·20 ms 주기·전체 코스·실물 시각·ARM/출력 권한은 이 검사로 증명되지 않는다. 패키지에 묶였다는 이유로 운영 완성이나 기본 활성으로 취급하지 않는다.
