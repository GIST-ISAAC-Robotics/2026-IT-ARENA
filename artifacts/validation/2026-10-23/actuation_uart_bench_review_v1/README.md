# UART 주기 어댑터 최종 검토 수정 v1 — 결함 재현·관련 회귀·변이

검사 exit 0, 실행 전후 소스 해시 차이 0 B다. 최종 검토(GPT)가 지적한 계약 불일치를 고친 소스의 첫 증거이며, 이후 대상 빌드 v3에서 공유 가짜 I/O의 큰 stack 프레임이 발견되어 [v2](../actuation_uart_bench_review_v2/README.md)로 대체했다. 이 폴더의 결과는 그 수정 전 소스에 대한 것이다.

- 결함 재현: [첫 회귀](../actuation_uart_bench_v1/README.md) 스냅샷의 어댑터 헤더(SHA-256 `4f5fb2ec…`)로 [재현기](../../../../scripts/uart_bench_defect_repro.cpp)를 빌드하면 6건 모두 실패(exit 1), 수정본 헤더에서는 0건(exit 0). [결함 판본 결과](repro_defect.log), [수정본 결과](repro_fixed.log).
- 관련 회귀 9개 통과(`test_actuation_uart_bench.py` 7개·`test_actuation_smoke.py` 2개). 공유 사례 32개를 ASan/UBSan과 `-Os`에서 실행.
- 변이 22종: 정상 대조 통과, **22/22** 검출.

[실행 스크립트](run_checks.sh), [종료값](repro_exit.txt), [변이 요약](mutants/summary.json), 원시 스냅샷·해시를 같은 폴더에 보관한다. 실제 UART/보드 실행이 아니다.
