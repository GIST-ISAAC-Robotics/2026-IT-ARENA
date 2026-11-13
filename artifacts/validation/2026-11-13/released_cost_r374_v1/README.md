# RELEASED 계획 중복 계산 제한 검토 r374

2026-11-13. GPT 코드·저장 기록 분석과 WSL 동일 입력 비교 완료. **전체 출력은 같지만 비용 개선이 미미하여 미채택한다.** Jetson·센서·ROS·공통 소스는 변경하지 않았다.

## 결과: 미채택

[적중 전체 배열 검증](verify_native/report.json)과 [별도 비용 비교](cost_native/report.json) 모두399판단의 전체 출력이 정확히 같고 입력·소스 해시 변동이 없었다. 실제 캐시 적중은7회이며 검증 실행에서는7회 모두 region/context의 전체 배열까지 일치했다. 검증 실행의 재계산 시간은 성능값으로 쓰지 않는다.

| WSL native 비용, RELEASED343호출 | baseline | 후보 |
|---|---:|---:|
| 합계 |13.105288초|13.066187초|
| 중앙 |40.1963ms|40.3546ms|
| p95 |44.5547ms|44.7380ms|

합계 감소는 **0.298%**이며 중앙/p95는 개선되지 않았다. 아래 사전5% 기준에 미달하므로 **추가 Jetson/실시간 A/B 없이 종료**한다. 기존 원 기록과의 전체 진단 차이30건은 양쪽 공통이며 원 비동기 실행 재현 성공으로 바꾸어 쓰지 않는다. `candidate.py`는 시험 산출물로 보존하고 공통 기본값에 넣지 않는다. `prepared_manifest.json`은 실행 준비 당시 기록 그대로이며 이후 이 README의 결과 갱신은 그 동결 기록을 덮어쓰지 않는다.

## 확인한 비용과 적용 한계

- [r369 상태별 원격 비교](../retained_schedule_r369_v1/README.md)에서 계획 SOLO 중앙59.65–60.54ms, RELEASED138.74–141.40ms, RELEASED 현재 검사63.50–66.70ms다. 서로 다른 상태/입력 집합이므로 이 차이를 모두 제거 가능한 낭비로 간주하지 않는다.
- [r359 제한 프로파일](../latest_plan_profile_r359_v1/runtime/runtime_result/plan_profile.json)의 index60은 `motion_region`80.86ms와 `band_check`4회26.44ms, index120은 각각41.82ms와25.74ms다. index90은 `released_body_gap.propose`88.49ms이며 내부 `_bounded_nonreturn`60.25ms를 포함한다. **중첩 누적 시간은 합산하지 않는다.** cProfile 오버헤드가 포함된 3표본이며 상태별 평균으로 확대하지 않는다.
- RELEASED는 기존 G1 계산에 현재 측방 띠/창 검사, 실제 이동 영역의 외곽 생성·광선 분류, 실패 시 여러 저속 후보의 영역 검사와 좁은 결측 검사를 추가한다. 이 중 대부분은 서로 다른 영역/판정이어서 단순 삭제할 수 없다. 원인을 메모리 부족이나 GPU 미사용 하나로 단정할 근거는 없다.
- `pass_release_trial.py`의 retained 분기는 목표를 `.35m/s`로 낮춘 뒤 영역을 검사한다. 이후 `released_body_gap.propose`가 같은 장면·속도·조향·dt의 `.35m/s` 영역을 다시 검사할 수 있다. 저장 r353399호출에서 RELEASED343, masked 재시도171, body-gap 채택5를 확인했다. 채택5는 모두 retained이며 두 region 사전이 정확히 같다. 실패 후보의 중복까지 모두 기록된 것은 아니므로 전체 빈도는 실행으로 확인해야 한다.

[정적 근거](static_evidence.json). r353의 기록된 call 순서는 폐기된 계획까지 포함하는 전체 자식 실행 순서가 아니다. 따라서 r359에서 보존한 원 기록과의 차이를 이번 후보의 오류/성공으로 바꾸어 쓰지 않는다.

## 준비한 최소 후보

[`candidate.py`](candidate.py)는 명시 `enabled()` 안에서 한 `PassReleaseSelector.select` 호출에 한하여 동일 `motion_region`을 재사용한다. 공통 기본값에 연결하지 않았다.

1. 동일 장면·동일 불변 구성 객체와 `max(abs(speed), abs(target))`, 현재/직전 조향, dt를 키로 쓴다. 이것이 원 `motion_queries`가 사용하는 전체 입력이며 반올림은 없다. 원 장면 참조를 보유해 객체 id 재사용을 막는다.
2. 첫 계산은 기존 함수 그대로 수행한다. 반환 사전·문맥 배열을 별도로 복사하고 다음 동일 계산에 다시 복사해 반환한다. 호출자가 `context.clear()` 또는 배열을 변경해도 저장본에 영향을 주지 않는다.
3. 후보/저속 상한/미관측/점유/가림 판정·광선·외곽 표본·시각·TTL·STOP은 원 코드가 그대로 수행한다. 다른 select 호출·센서 시각·epoch로 결과를 넘기지 않는다. 예외/정상 종료에서 두 원 함수를 복원한다.

이 후보가 줄일 수 있는 것은 같은 호출 안의 동일 영역 생성·광선 분류뿐이다. 새로운 영역의 검사를 생략하지 않는다. 광선 수치 커널 추가, 판정선 조정, 주기 변경, GPU 이관은 포함하지 않는다.

## 한 대표 묶음의 실행과 중단 조건

[`compare.py`](compare.py)는 두 독립 현재 제어기에 같은 r353 입력399개와 reset을 적용한다. 실행 순서를 AB/BA로 교대한다. 원 r278 경계 정책을 양쪽에 동일하게 설치하며 원 기록 출력과의 차이는 별도 보고만 한다.

- 명령뿐 아니라 모든 판단 진단을 JSON 바이트로 비교한다. 기존 `comparable_decision`에서 지정한 표면 정합 `processing_ms` 위치만 정규화한다. 허용 오차를 늘리지 않는다.
- `--verify-hits`에서는 캐시 적중마다 원 함수를 다시 실행해 **region 사전·문맥의 전체 배열 dtype/shape/바이트열**까지 대조한다. 이 모드는 성능 비교가 아니다.
- 기본 실행은 동일 입력의 전체/RELEASED 중앙·p95·합계와 캐시 적중/절약 질의 수를 기록한다. 모든 Python 소스와 원 입력·후보 파일을 실행 전후 해시로 검사한다.
- 판단 차이1건, region/context 차이1건, 소스 변동, 미완료면 즉시 실패로 종료한다. 코드 수정으로 판정/비교 정밀도를 완화하지 않는다.
- 비용 실행에서 RELEASED 합계 감소가5% 미만이면 **이 후보의 실시간 A/B 재생을 추가하지 않고 미채택 종료**한다. 5%는 안전 판정선이 아니라 후속 실험의 시간 배분 기준이다. 통과해도 공통 채택이나10Hz 성공을 뜻하지 않는다.

아래는 이번 검증에 사용한 실행 진입점이다. 저장소 루트에서 수행하고 출력 경로는 이 artifact 아래의 새 디렉터리만 허용한다. native 모듈이 준비된 격리 실행 사본을 사용하고 같은 파일명 산출물을 덮어쓰지 않는다.

```sh
python3 artifacts/validation/2026-11-13/released_cost_r374_v1/test_candidate.py
python3 artifacts/validation/2026-11-13/released_cost_r374_v1/compare.py --backend native_v1 --verify-hits --out artifacts/validation/2026-11-13/released_cost_r374_v1/verify_native
python3 artifacts/validation/2026-11-13/released_cost_r374_v1/compare.py --backend native_v1 --out artifacts/validation/2026-11-13/released_cost_r374_v1/cost_native
```

추가 원격31초 실제 시간 재생은 수행하지 않는다. 현재 검사·부모 일정·명령 만료·ROS·실차 안전의 검증은 이 직렬 비교에 포함되지 않는다. 중복 제거로 RELEASED 중앙139ms 문제를 해결했다고 쓰지 않는다.
