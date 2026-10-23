# ARM 출력 준비 관문 — 첫 관련 회귀

송신 정책 후보에서 발견한 `ActuationIngress`의 ARM 준비 관문 누락을 보완한 검사다. 구동 출력은 없고 실제 감시 대신 호출 인수로 준비값을 주었다.

- `checks_exit.txt`: 0. `related_tests.log`: **168 passed**. 전후 소스 해시 차이 0 B.
- 실제 Python `VirtualMCU.on_tx`와 준비 참/거짓·정상 ARM·잘못된 버전/세션/누락 필드·STOP/CLEAR를 대조했다. 출력 준비가 거짓이면 raw `ARM`은 receive 이전에 `output_not_ready`로 거절한다. STOP/CLEAR와 별도 재허가를 유지했다.
- 준비값 기본 false·호출 간 저장 없음, 만료 drain 시점의 준비 판정, 수신기 시계 역행을 준비 거절로 가리지 않는 것, 거절 ACK 반환을 검사했다.
- `audit/report.json`: 위반 0. 정상 DRIVE 50 Hz는 송신 후보 v2와 같은 **17,662 B/s·offer 50개/s·ACTIVE**. 기존 운영 콜백의 65,818 B/s와 구분한다.
- 바깥 PowerShell 래퍼의 종료 코드 파일 쓰기는 인용 오류(exit 1)였지만 내부 bash는 모든 단계를 마치고 자체 exit 0을 저장했다. 검사를 재실행하거나 오류를 내부 성공으로 바꾸지 않았다.

명령·원본은 `run_checks.sh`, `source_snapshot.tar.gz`, `sources_before.sha256`/`sources_after.sha256`에 보존한다. 실제 출력 상실 감시·보드 실행·전기 STOP은 미검증이며 `firmware_verified=false`, `ready_to_drive=false`.
