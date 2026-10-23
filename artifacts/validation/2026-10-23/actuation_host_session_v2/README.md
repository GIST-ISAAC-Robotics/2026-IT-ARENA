# 호스트 요청 상관 후보 — v2

- [실행](run_checks.sh), [로그](tests.log): **183 passed·0 skipped·exit 0**, [소스 해시 차이](source_hash_diff.txt) 0 B. [v1 시험 오류](../actuation_host_session_v1/README.md)는 보존했다.
- 양의 ACK와 새 세대/소유권/출력 준비 offer를 모두 요구한다. 도착 순서 반전, 요청 기한, 잘못된 ACK/옛 연결·boot, 단절/시계 변경, STOP 선점, 로컬 STOP 의도 유지, CLEAR 후 별도 ARM, 원천 시각 단조성을 검사했다.
- 실제 portable C++ 송수신 코드를 PC에서 호출한 ARM→STOP 사례에서도 늦은 ARM ACK가 구동 증거를 복원하지 않았다. 호스트 후보는 제어 envelope를 반환하지만 자체 전송/DRIVE 생성/물리 출력은 없다.
- 시간/통신 상한과 피드백은 합성 조건이다. `session_conditions_satisfied`는 조건부 증거 검토 결과이며 `ready_to_drive=false`, `runtime_connected=false`다. 운영 ROS·실물 firmware·전기 STOP 완료가 아니다.
- 다음 검출력 검사에서 추가한 불확실한 수신 구간 반례는 이 v2 실행 후 변경이며 소급 포함하지 않는다.
