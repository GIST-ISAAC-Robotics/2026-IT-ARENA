# ARM 출력 준비 관문 후 전체 회귀

- [실행 스크립트](run_checks.sh): WSL의 기존 ROS Jazzy 환경에서 코어 대조용 C++ 실행 파일 6개를 새로 빌드하고 명시한 환경변수로 전체 검사를 실행했다. ASan/UBSan·`-Wall -Wextra -Werror -pedantic`를 사용했다.
- [결과](full_tests.log): **3,791 passed, 0 skipped**, 내부 [exit 0](checks_exit.txt). 앞선 송신 v2의 사전 빌드 미지정 19 skipped를 이 결과로 소급 변경하지 않는다.
- [컴파일 로그](compiler.log)와 [소스 해시 차이](source_hash_diff.txt)는 0 B다. 검사 당시 소스는 `source_snapshot.tar.gz`, 바이너리 해시는 [binaries.sha256](binaries.sha256)에 보존했다. 후속 오프라인 시계 후보는 이 검사 뒤 추가했으므로 포함되지 않는다.
- 연결부가 호출마다 받는 준비값은 기본 false이며 ARM만 거절한다. STOP/CLEAR·명시 재허가, Python 콜백과의 판정 순서·거절 ACK를 관련 검사로 확인했다.
- **PC 회귀**다. ESP32 실행·UART·독립 출력 감시·전기 STOP·차체 정지는 검증하지 않았다. 자세한 의미는 [구동 계약 §20](../../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#20-1022-후속--arm-출력-준비-관문)에 둔다.
