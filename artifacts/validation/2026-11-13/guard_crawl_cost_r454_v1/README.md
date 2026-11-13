# r454 GUARD 단일 저속 재검사의 Jetson 국소 비용

2026-11-13, GPT. 주 담당의 별도 Jetson 소유권 배정으로 수행한다. WSL은 다른 담당 소유이며 사용하지 않는다. 센서 포트·ROS/MCU·ARM·flash·모터·서비스·전력/주파수 설정에 접근하거나 변경하지 않는다.

## 입력·실행 경계

r451 `analyze.py`의 국소 복원 절차를 따라 r441 원 trace895–998의104행만 선택한다. 원 scan/history/reference/epoch/기하·서명 있는 측정속도·양측 상태/guard anchor·경로를 유지한다. 원 정책의 `abs` 전달은 그대로 재현하고 새 adapter에는 별도의 원 signed 측정값을 전달한다. 원시 trace 전체 SHA와 선택한 행, header, r451 `result_integrated_v2/checks.json` 기대출력을 보존한다. baseline 행동 필드는 r451 국소 복원과 동일한 제한을 가지며 원 전체 plan의 숨은 상태를 복원한 것이 아니다.

최신 공통 두 Python package를 snapshot으로 복사한다. r447의3 native C++ 파일과 setup.py 총4개 SHA가 현재 공통·기존 remote source와 같을 때만 기존 ARM native 바이너리를 새 `/home/q/it-arena-r454-a`로 복사한다. 기존 파일이나 설치를 수정하지 않는다. 실제 import는 새 폴더의 native_v1/ABI1만 허용한다.

## 각 행의 단일 쌍

1. 원 `_guard_curve_original`을 한 번 호출한다. 내부 `.35/.25/.15` 세 검사의 결과·기존 recheck 진단을 원 기록과 대조한다.
2. 원 비교가 통과하면 계산된 세 검사 증거를 현재 `guard_crawl.assess`에 전달해 한 번 호출한다. 내부 `.1` 전체 검사1회임을 확인하고 r451 통합v2 결과와 대조한다.
3. baseline reason/path와 current dispatch의 `_present.supported`를 포함해 결과를 비교한다. **이 지원 판정은 독립 current 계산/보호층/부모 발행 확인이 아니다.** selector sides/guard anchor/last_choice 불변과 입력 SHA를 확인한다.

허용선은 r451 그대로 실수 절댓값1e-12, 정수·키·구조 exact다. 최초 불일치에서 중단하며 원 실패를 보존한다. ARM↔x86 정수 표본 차이가 있으면 별도 기록하고 제어 판정 차이와 구분한다. 자동으로 기준을 완화하거나 성공할 때까지 반복하지 않는다. warmup·기존 r447198계산·전체 plan replay·ROS 생성자 추가 호출은0이다.

## 시간 경계

`perf_counter_ns`로 원 세 검사 함수 전체, 추가 adapter 전체, 각 내부 GUARD 검사 시간을 기록한다. 함수 입출력을 바꾸지 않는 호출 래퍼만 사용하며, adapter 기타 비용은 전체에서 내부 `.1` 시간을 뺀 값이다. 따라서 래퍼/타이머 자체의 작은 비용이 포함되고 별도 보정하지 않는다. 입력 복원/scene/view 구성, 비교·직렬화, 프로세스/import/native setup, 독립 current 실행은 해당 비용에서 제외한다. 입력 복원 비용은 별도 setup 필드다.

같은 행에서 기존3검사→adapter 순서로 한 번씩 측정한다. 순서를 교차하거나 주파수·온도를 고정한 실험이 아니다. 첫 호출과 median/p95(nearest-rank)/max를 구분한다. 값은 실제 Jetson의 국소 함수 wall 시간이며 WSL 비용, 센서 동시입력,10Hz 전체 운용 또는 실차 진행/완주 증명이 아니다.

동결 근거는 `prepared.json`, `source_manifest.json`, `bundle_sources.zip`이다. 실행은 `remote_bootstrap.py` 단일 batch이며 제한120초와 자기 process group 회수를 둔다. 종료 후 결과 원문·소스 불변·자기PID 회수를 보고하고 Jetson 소유권을 반환한다.
