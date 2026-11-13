# r476 r474 최종 선택 경로의 Jetson 계산 비용

**새 현재 경로 우선 분기를 탄 current 107개는 median 12.514ms, p95 13.425ms였다.** 전체 current 113개는 최대77.727ms로 이번 저장 표본에서는100ms 초과가 없었다. 반면 전체 plan217개 중9개는100ms를 넘었으며 최대244.954ms였다. plan 전체의10Hz 마감 충족이나 실제 센서/ROS 전체10Hz 완료로 해석하지 않는다.

## 범위와 방법

`current_route_priority_r474_v1`의 최종 v2 동결 소스를 사용했다. r469의 원 `runtime_evaluator_header`, generation/reset 순서의217개 실제 plan 제출 입력, 실제 후속 current113개를 재사용했다. 실측 센서로 이름을 바꾸거나 wheel/시각/epoch 수치를 합성하지 않았다. 마커 r466 기본OFF를 유지했다.

별도 `/home/q/it-arena-r476-a`에서 **plan217회 + current113회**, 합계330회만 순차 계산했다. 각 current에는 같은 `(generation, sequence)`의 **이번 ARM 계산 plan output**과 원 제출 입력·actual current를 연결했다. 원 x86 plan을 그대로 복사해 current 결과를 비교한 것이 아니다. 첫 호출은 아래에 따로 표시하며 통계에 포함했다. 별도 warmup0·반복0·계산 후 추가 평가0이다.

시간은 `Evaluator('plan'|'current', header)(input)` 호출의 wall time이다. 입력 복원/scene/policy/freeze를 포함하며, 모듈 import·생성자·request 구성·hash/비교·파일쓰기·SSH/IPC는 제외했다. 두 lane을 동시에 실행하지 않았으며 ROS나 센서 처리 부하도 없다. p95는 각 유한 표본의 선형 보간값이다.

## 결과

| 대상 | n | median | p95 | max | 100ms 초과 |
|---|---:|---:|---:|---:|---:|
| plan 전체 |217|73.596ms|91.116ms|244.954ms|9|
| 새 current-route plan |209|73.525ms|86.986ms|130.143ms|3|
| current 전체 |113|12.539ms|63.552ms|77.727ms|0|
| 새 current-route current |107|12.514ms|13.425ms|72.406ms|0|
| 기존 다른 current 경로 |6|65.248ms|75.789ms|77.727ms|0|

첫 plan은50.367ms, 첫 current는25.655ms였다. 첫 호출 제외 중앙값은 plan73.602ms/current12.532ms다. 생성자 시간은 plan194.517ms/current9.273ms로 별도 기록했다. process peak RSS는149,776KiB이며 입력/기대 출력 전체를 메모리에 보유한 이 계측 프로세스의 값이다.

plan100ms 초과9개 중6개는 ref16.82–17.32의 기존 `static_side_current_lane` 경로(118.753–244.954ms)다. 새 우선 경로3개는 ref16.72(130.143ms),18.32(108.385ms),19.92(126.971ms)다. 이번에는 내부 함수별 계측이나 재실행을 하지 않았으므로 각각의 시간 원인은 단정하지 않는다. 모든 느린 행은 `summary.json`에 남겼다.

원 첫 지속 정지 actual current seq823(ref19.026, 원 실측 .367350252m/s)은 새 ARM plan .6을 받아 **.6m/s, `rebase_current_follow`,12.058ms**였다. 이는 기존 r474 x86 후보와 같은 결과다. 실제 부모 발행이나 폐루프 진행의 증거는 아니다.

## ARM과 기존 x86 후보의 동등성

기존 `follow_trace.equal_output`의 rel/abs tolerance1e-10을 그대로 적용했다. 기준을 넓히거나 실패 항목을 지우지 않았다.

- 명령: plan217/217·current113/113 동등.
- 전체 output: plan217/217·current111/113 동등.
- raw exact: plan3/217·current0/113. 허용범위 내 부동소수점 차이를 exact로 표시하지 않았다.
- 비교기 실패 current2개는 기존 `rebase_current_released_parallel` 경로다. 명령·reason은 동일하며 아래 region 정수 계수만 기준 밖 차이였다. 새 우선 경로 current107개는 전체 비교기를 통과했다.

| trace seq / reference | 필드 | x86 | ARM |
|---|---|---:|---:|
|751 /17.224|`current_route_proof.region.queries`|5716|5714|
|751 /17.224|`current_route_proof.region.masked_left`|310|308|
|755 /17.324|`current_route_proof.region.queries`|5654|5652|
|755 /17.324|`current_route_proof.region.masked_left`|244|242|

이는 관측된 cross-architecture region count 차이이며 세부 원인을 새로 추적하지 않았다. 같은 ARM에서 원/후 정책을 반복한 비교나 실제 센서 정확도 검증은 이번 범위가 아니다. 원 출력은 `remote_result/results/*_outputs.jsonl`에 보존했다.

## 환경·무결성·회수

기존 SSH 접근 도구와 비공개 credential을 사용했다. 사전 loadavg0.00/0.00/0.00, 관련 실행0·열린 센서 포트0을 확인했다. Python3.10.12/aarch64, 기존 governor와 전력 설정을 유지했다. 설정값은 변경하지 않았으며 안전한 환경 메타데이터를 실행 receipt에 보존했다.

기존 r470 `/home/q/it-arena-r470-a/stage`의 ARM native를 새 사본에만 복사했다. `setup.py`와 `_native` 원소스의 SHA가 r474와 일치함을 로컬·원격에서 확인했고, binary SHA는 **5654cd2f52af838b52384a084f159bb1319925db6690945b4403cb7ae33f5bff**다. 재빌드·패키지 설치·전역환경 변경0이다.

로컬/원격 묶음484파일 변경0, 원 r469 trace 불변, native 원본 불변이다. 소유 child PID62739는 exit0, 강제 종료0, process group 잔존0이며 사후 관련 실행/열린 센서 포트0이다. SSH 전경 wrapper도 exit0으로 끝났고 결과 ZIP SHA를 대조해 회수했다. **Jetson 자원은 root에 반환했다.** WSL·ROS·센서·ESP32·카메라·모터·서보·ARM/flash 접근은 없었다.

## 근거와 재집계

- `prepared.json`, `bundle/source_manifest.json`: source/입력 SHA·고정 계산 범위.
- `remote_result/execution_receipt.json`, `resource_return.json`: 환경·프로세스·원본 불변·정상 회수.
- `remote_result/results/result.json`: 모든330회 시간과 비교 결과, import 경로.
- `summary.json`: 느린9행·cross-architecture 차이2행·분기별 집계.
- `prepare.py`, `run_cost.py`, `execute.py`: 준비와 실행 recipe. 동일 시험 재실행용 승인이 아니다.
- `python -B summarize.py`: 이미 회수한 파일만 Windows에서 재집계하며 evaluator를 실행하지 않는다.

결론적으로 이번 표본의 새 current 경로 비용은10Hz 한 주기보다 충분히 작지만, 전체 plan의100ms 초과9개가 남는다. 실행 주기·lane 병행·센서·독립 guard를 포함한 운용 판단은 주 담당의 통합 검토와 실제 실행 근거에서 별도로 다룬다.
