# run53 — 두 앞차 대기열의 서로 다른 재출발

[사전 등록](../QUEUE_STAGGERED_PLAN.md), [설정](../../../../../config/tests/m2_queue_staggered_v1.json), [실행 명령](command.txt).

- 원본 [`report.json`](run/report.json): `passed=true`, `run_valid=true`. 가까운 앞차 HOLD **3.235 s**, 재개 지연 **0.633 s**, 재개 후 진행 **1.840 m**, 전체 진행 **4.260 m**, 최종 차체 간격 **0.625 m**. 최소 표본/보수적 구간 간격 **0.362777/0.362776 m**.
- [`queue_eval.json`](queue_eval.json): 먼 차만 먼저 움직이는 8.2–8.9 s 창에서 자차·가까운 차 모두 정지, 안전 명령 0·HOLD. 먼 차 실제 이동 **0.240 m**, 9개 trace 표본·밀집 자세 포괄, 등록 추가 조건 통과.
- 추가 앞차의 시험기 유효, 최소 자차 간격 **1.012776 m**, 접촉 사건 0. 명시 STOP 6항목·정상 종료·잔존 0, 노면 9,739표본 이탈 0. 판단 재생 **797/797 동일**.

## 보조 소스 해시 오류와 독립 검증

`files_to_hash.txt`의 CRLF가 bash에서 파일명 뒤 CR로 남아 `sha256sum`이 실패했다. 비어 있는 before/after 파일의 `diff_exit=0`은 불변 증거가 아니다. 원본 스크립트·빈 파일을 보존했고 같은 주행을 재실행하지 않았다.

별도 [`source_archive_audit.json`](source_archive_audit.json)은 평가기가 주행 전에 생성한 **74개 소스 SHA와 ZIP**을 현재 파일에 대조해 전부 일치했고, 원본 보고서의 실행 전후 `source_unchanged=true`도 확인했다. run52에 보관한 평가기/시험/overlay 사본도 그대로다. 이는 실행기 전체 117항목 보조 manifest 검증을 성공으로 소급한 것이 아니다.

범위는 공식 코스 직선의 두 각본 actor·한 조건이다. 먼 차량의 식별/추적·코너 병렬주행·측후방 복귀·전체 랩·실차 성능을 증명하지 않는다. run52 경로 검사는 직선 끝 위치와 actor 간 표본 겹침 검사이며, 각 actor 외곽을 매 표본마다 도로/벽과 대조한 검사라고 확대하지 않는다. 실제 자차 노면 검사는 별도 결과다.
