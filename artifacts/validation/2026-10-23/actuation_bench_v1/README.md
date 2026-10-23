# 출력 비활성 연결부의 공유 검사

- [실행 스크립트](run_checks.sh), [회귀 로그](tests.log): **230 passed·0 skipped·exit 0**. [소스 해시 차이](source_hash_diff.txt) 0 B, 당시 `source_snapshot.tar.gz` 보존.
- `ActuationBenchEndpoint`는 수신/송신 연결부를 소유하지만 출력 준비는 항상 false다. 수정 가능한 receiver/TX 참조, 가짜 엔코더 주입, ARM 준비값을 노출하지 않는다.
- 공유 사례 **15개**를 호스트 ASan/UBSan과 크기 최적화 빌드에서 각각 검사했다. ARM/DRIVE 거절, STOP 고정, 피드백 없는 CLEAR 거절, 부분 송신 중 단절, 재연결, 과대 입력, 시계 역행, 부분 수신 만료, sink 계약 위반을 포함한다. 기존 코어 20·수신 연결부 12·송신 정책 9개와 구별한다.
- 실제 UART/GPIO/타이머·장치 I/O 없음. 호스트 실행이며 보드에서 실행한 결과가 아니다. 대상 링크와 전체 회귀는 별도 결과로 남긴다.
