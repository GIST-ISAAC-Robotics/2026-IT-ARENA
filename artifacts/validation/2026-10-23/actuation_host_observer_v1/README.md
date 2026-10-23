# compact 호스트 보관 후보

- [실행 스크립트](run_checks.sh), [결과](tests.log): **326 passed·0 skipped**, [exit 0](checks_exit.txt), [소스 해시 차이](source_hash_diff.txt) 0 B. 검사 당시 소스는 `source_snapshot.tar.gz`에 보존했다.
- 새 `CompactHostObserver`는 `scripts/` 안의 출력 없는 후보다. offer/ACK/state/diag를 각각 보관하고, 원래 수신 시각 구간을 유지한다. 최신 4종과 유한 이력만 유지한다.
- 새 세대가 오면 옛 offer를 폐기하며 중복 임대·이전 세대/boot·연결 ID·host 시계 역행을 구별했다. ACK에는 boot/generation이 없어 상관 확인된 서비스 성공이나 ARM 허가로 쓰지 않는다.
- 실제 C++ 송신 driver를 새로 빌드해 DISARMED→ARMED→STOP의 프레임을 수신했다. CRC decoder의 분할 입력도 별도로 검사했다. **호스트 테스트 프로세스 간 연결**이지 실제 UART나 장치 측정이 아니다.
- `ready_to_drive=false`, `runtime_connected=false`. 운영 ROS/`CommandProducer`·시리얼 포트·출력에는 연결하지 않았다.
- [설계와 남은 조건](../../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#22-1023-후속--호스트의-compact-메시지-보관-후보).
