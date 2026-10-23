# UART 주기 어댑터 검토 수정 v3 — 변이 스크립트 고정 문자열 오류로 실패

검사 exit **1**, 소스 해시 차이 0 B다. 2차 검토(GPT)가 지적한 “빈 큐 응답 뒤 마지막 조회에서 보인 사건을 Clean으로 처리” 경쟁을 고친 소스의 첫 실행이다.

- 결함 재현은 의도대로 나왔다: 첫 판본 헤더(`4f5fb2ec…`) 7건 실패, 직전 수정본 헤더(review_v2, `95649ef4…`) 새 반례 `observed_pending_after_empty_fed` 1건만 실패, 수정본(`96d9a514…`) 0건.
- 관련 9회귀 통과.
- 실패 원인: `check_uart_bench_mutants.py`의 `per_call_event_budget` 변이가 이번 수정에서 지운 `bool empty = false;` 줄을 고정 문자열로 써서 `mutation anchor count is 0`으로 중단됐다. 어댑터 결함이 아니라 검사 스크립트의 고정 문자열 오류다. 변이 결과는 없다.

고정 문자열만 고친 재실행은 [v4](../actuation_uart_bench_review_v4/README.md)다. 이 폴더는 실패 근거로 보존한다.
