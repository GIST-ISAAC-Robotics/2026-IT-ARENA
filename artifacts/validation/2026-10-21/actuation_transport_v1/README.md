# 바이트 프레임과 구동 계약의 결합 검사

- 실행 [run_checks.sh](run_checks.sh): 관련 **163회귀 통과**, 실제 종료 0, 소스 해시 불변.
- [결과](export/report.json): **12사례·828이벤트·상태 checkpoint 48개·전송 checkpoint 16개**, 재생 차이 0.
- [입력/기대값](export/vectors.json), [재생 출력](export/python_observations.json), 사전 소스 전체와 선택 소스 ZIP 보존.
- 분할/병합·CRC/길이 오류·중복/지연·재연결 부분 폐기·STOP/CLEAR/ARM·제어 tick 중단을 PC decoder→상태기계로 연결했다. CRC 프레임은 내부 명령이며 실제 ROS `type/command` 봉투나 UART 큐를 모델링하지 않는다.
- 합법 범위의 손상 길이가 STOP을 가리는 **기존 문서상 한계**를 재현했다. 이 경우 STOP 처리로 표시하지 않고 명령 만료가 목표 0을 만든 것으로 기록했다.
- 실제 UART/펌웨어·부분 프레임 시간 제한·스레드/인터럽트·전기 출력·차체 정지 검증 아님. 운영 소스/기본값 변경 없음.

해석은 [구동 계약 §10](../../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#10-1021-후속--분할-바이트와-만료stop의-결합-대조)를 따른다.
