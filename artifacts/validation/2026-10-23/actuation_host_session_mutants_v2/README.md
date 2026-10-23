# 호스트 요청 상관 후보 — 수신/처리 시각과 검출력

- [실행](run_checks.sh), [관련 회귀](tests.log): **190 passed·0 skipped·exit 0**, [소스 해시 차이](source_hash_diff.txt) 0 B. 스냅샷 보존.
- [변이 결과](mutants/report.json): 대조 사본 **56통과**, **13/13** 의도적 결함이 시험 본문의 `AssertionError`로 검출됐다. 문법/수집/TypeError를 검출 성공으로 세지 않는다. 첫 실행의 [비Assertion 실패](../actuation_host_session_mutants_v1/README.md)도 보존한다.
- STOP 허가 폐기·미확인 ARM 선점·로컬 STOP 의도, ACK ID/결과/이유/기한/순서, 옛 연결, ARM 불확실 구간/처리 지연, 원천 watermark, 명시 CLEAR를 각각 제거/변경한 사본을 검사했다. 검사 통과는 모든 결함이 없다는 증명이 아니다.
- 원래 수신 구간 `received`와 소유 작업 처리 구간 `processed`를 구별한다. 오래된 수신 구간을 재작성하지 않고, ARM 기준은 두 증거를 처리한 구간이다. 기한 전 수신했더라도 기한 후 처리한 ACK로 허가를 복원하지 않는다. 처리 시각 역행/명시 수신–처리 순서 불확실/수신 순서 역전은 닫힌 상태가 된다.
- 관련 실제 C++ 교환은 PC 시험이며 보드/UART/ROS·물리 구동 없음. `ready_to_drive=false`, `runtime_connected=false`를 유지한다.
