# r454 Jetson GUARD 단일0.1 재검사 비용

2026-11-13, GPT. 실제 Jetson ARM native의104개 국소 저장 입력에서 **새 adapter 추가 비용은 median18.556ms, p95 18.798ms**였다. 기존0.35/0.25/0.15 세 검사46.233ms와 합친 median은64.851ms다. 각 행의 추가/기존 비용 비율 median은40.15%이므로 무시할 수 있는 비용으로 취급하지 않는다. **전체 plan·독립 current·센서/ROS 동시 부하·10Hz·실차 주행을 검증한 결과는 아니다.**

원 세 검사104행, 새0.1 수용104행, 현재 경로의 dispatch 지원104행 모두 원 비교 기준을 통과했다. 제어 함수는 기존312회+새104회만 계산했으며 warmup·재실행·r447198계산 반복은0이다. 첫 불일치 중단 조건을 둔 한 batch가 exit0으로 완료됐고 실제 불일치는 없었다.

## 결과 일치와 유지 조건

기준은 r451 `analyze.py`의 원 국소 복원과 `result_integrated_v2`다. 현재 공통 Python 사본과 r447 ARM native를 격리 루트에서 사용했다. 원 입력의 scan/history/시간·epoch/측정 부호/기하·경로·양측 상태/guard anchor/1cm 한계를 바꾸지 않았다. 원 정책의 abs 전달과 새 adapter의 signed 검사도 그대로다.

| 확인 | 결과 |
|---|---|
| 기존0.35/0.25/0.15의 원 guard 증거·이유·recheck 진단 |104/104 일치. 실수 절댓값1e-12, 정수/키/구조 exact 유지 |
| 새 adapter의 전체 출력 대 r451 통합v2 |104/104 같은 기준 통과, 실수 최대 차이1.388e-17, 정수/구조 exact |
| 새0.1 수용 |104/104 |
| `_present.supported` |104/104. **지원 계약 확인이며 current 검사 자체의 실행은 아님** |
| 새 masked tail |7.317418–9.197843mm,10mm 한계 유지 |
| 새 미관측·전면 mask 표본 |모두0 |
| 원 signed 측정속도 |0–0.061225m/s, median0 |
| sides/guard anchor/last_choice, 입력 SHA |모두 불변 |

r447의 과거 ARM↔x86 region count 차이를 이번 결과에 가져오지 않았다. 이번104행에서는 정수 표본 차이가 없었다. 다른 입력집합의 r447 원 실패는 그대로 보존하며 이번 성공으로 바꾸지 않는다. 새 유한 장애물/음수 반례는 이번 비용 batch에 추가하지 않았고, 기존 관련 검증을 중복하지 않았다.

## 같은 Jetson에서 측정한 국소 wall 시간

단위ms, p95는 nearest-rank다. 한 행에서 원 세 검사→새 adapter를 한 번씩 실행했다.

| 범위 | n | 첫 호출 | median | p95 | max |
|---|---:|---:|---:|---:|---:|
| 기존 세 검사 함수 전체 |104|69.993|46.233|47.524|69.993|
| 그중0.35 검사 |104|41.111|18.663|19.271|41.111|
| 그중0.25 검사 |104|15.691|14.927|15.430|15.754|
| 그중0.15 검사 |104|13.145|12.607|12.925|13.388|
| 새 adapter 전체 |104|18.883|18.556|18.798|18.958|
| 그중0.1 전체 GUARD 검사 |104|11.223|11.210|11.383|11.483|
| adapter 나머지(차감) |104|7.660|7.361|7.496|7.670|
| 기존3검사+새 adapter 합계 |104|88.876|64.851|66.380|88.876|

adapter 나머지에는 조건 확인·`threat_view` 구성·메타데이터 조합과 타이머/호출 래퍼 비용이 포함된다. 세부 함수마다 프로파일하지 않았으므로7.361ms 전부를 특정 함수 비용이라고 단정하지 않는다. 추가 GUARD11.210ms와 adapter 전체18.556ms를 혼용하지 않는다.

입력 복원/scene/최초 view 구성은 별도 setup으로 median16.092ms, p95 16.596ms, max51.986ms였다. 파일 입출력·결과 대조/직렬화·프로세스/import·원 baseline 계획 계산·독립 current/guard/ROS 발행 비용은 위 합계 밖이다. 따라서 국소 합계104행이100ms 미만이어도 전체 제어 주기 충족이라고 할 수 없다. 기존 WSL/Windows 비용과 개선율을 계산하지 않았다.

Python3.10.12/aarch64, native ABI1이다. 전력·governor·thread 환경은 바꾸지 않았고 schedutil을 유지했다. 첫 호출을 제외하거나 warmup으로 숨기지 않았다. 교차 순서·고정 주파수·장기 부하 실험은 아니다. 단일 batch 전체 소요9.733초는 설정/비교/직렬화 등을 포함하므로 제어 주기와 구분한다.

## 소스·실행·회수

- 동결235파일, 기존 r447과 같은 native 소스/setup4개를 Windows/Jetson 양쪽에서 대조했다. r447 바이너리를 사본으로 재사용했으며 새 빌드·설치0이다. 외부 project import0, 원 입력 및 공통 출처/remote 동결 변경0.
- source archive SHA256: `972a543cb3e81f980aded3b187764582ce6586b4cc0f6e30969c93c9308131a5`.
- ARM native SHA256: `a2afb70525be1e86f63ae072abc4fbaeec10d65f0596e5f642f135ba6a1de394`.
- result ZIP SHA256: `e8faba24bcd6d96fa77d2a62c7a9f58a1cc892f05efe8f075db73598d9f6968d`.
- 실행 루트 `/home/q/it-arena-r454-a`, 계산PID59596·exit0·강제 종료0·잔여 process group0. 종료 후 별도 읽기 확인에서도 PID 부재, 관련 계산 프로세스0·센서 포트0·소스 변경0이다.
- Jetson 소유권은 결과 회수/최종 PID 확인 직후 주 담당에게 반환했다. 이후 집계는 Windows의 회수 자료만 사용했다. WSL·센서·ARM·flash·모터·서비스/전력 설정 변경은 없다.

원 증거: [source manifest](source_manifest.json), [준비 영수증](prepared.json), [건별 원 결과](remote_result/cost/rows.jsonl), [원 report](remote_result/cost/report.json), [실행/환경](remote_result/execution.json), [종료 확인](final_audit.log), [집계](summary.json).

이 결과로 새로운 common 최적화나 재시험을 추가하지 않는다. 기존 경로를 보존한0.1 후보의 계산상 수용과 실제 추가 비용까지 확인했으며, 실제 정체 해소·부모 발행·전체 주기 판단은 주 담당의 별도 대표 결과에 남긴다.
