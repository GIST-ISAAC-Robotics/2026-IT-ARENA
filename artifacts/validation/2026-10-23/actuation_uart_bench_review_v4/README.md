# UART 주기 어댑터 검토 수정 v4 — 관측된 pending 경쟁 수정 최종본

검사 exit 0, 실행 전후 소스 해시 차이 0 B다.

## 고친 경쟁(2차 검토 지적)

[review_v2](../actuation_uart_bench_review_v2/README.md) 판본의 `check_events`는 `return empty || pending == 0 ? kClean : kUnconfirmed`였다. `next_event`가 큐가 비었다고 답한 직후 ISR이 새 사건을 넣으면, 마지막 `pending_events()` 조회에서 사건이 실제로 보여도 `empty` 때문에 Clean이 됐다. 읽은 뒤 확인에서 이 경로를 타면 내용 모르는 오류 꼬리를 남긴 채 조각을 연결부에 넣을 수 있었다. 진입 검사 이전부터 모르는 일반 경쟁과 달리 **끝 조회에서 존재를 확인한 사건**을 무시한 것이다.

수정: 마지막 조회가 0일 때만 Clean, 그 밖에는 미확인(읽은 뒤면 조각 폐기·세션 경계)이다. 필요 없어진 `empty`는 지웠고 `-Wall -Wextra -Werror -pedantic` 경고 없이 빌드된다. 마지막 조회 이후 도착하는 사건은 여전히 다음 확인 몫이며 이 한계는 바뀌지 않는다.

## 결과

- 결함 재현기 7건: 첫 판본 헤더(`4f5fb2ec…`) **7건 실패**(exit 1), 직전 수정본 헤더(`95649ef4…`) 새 반례 **`observed_pending_after_empty_fed` 1건만 실패**(exit 1), 수정본(`96d9a514…`) 0건(exit 0). 공유 사례 33번 `pending_after_empty_post_read`는 가짜 `next_event`가 빈 큐로 답한 직후 프레임 오류를 넣어 같은 경쟁을 검사한다.
- 관련 **9회귀 통과**(공유 33사례 ASan/UBSan·`-Os`, 재현기, 정적 경계 검사, smoke 공유 사례).
- 변이 **23/23** 검출, 정상 대조 통과. 새 변이 `clean_after_empty_drain`(빈 큐로 끝나면 pending과 무관하게 Clean)은 33번 사례에서만 검출됐다. [v3](../actuation_uart_bench_review_v3/README.md)의 고정 문자열 오류를 고친 `per_call_event_budget`도 검출됐다.

전체 무관 회귀는 반복하지 않았다. 실행은 Claude Haiku 5.5 보조, 원시 결과 대조는 Claude Opus 5.5가 맡았다. 실제 UART/보드 실행이 아니다.

[실행 스크립트](run_checks.sh), [종료값](repro_exit.txt), [첫 판본 재현](repro_v1.log), [직전 수정본 재현](repro_review_v2.log), [수정본](repro_fixed.log), [관련 회귀](related_tests.log), [변이 요약](mutants/summary.json), 판본별 헤더 사본·해시·원시 스냅샷을 같은 폴더에 보관한다.
