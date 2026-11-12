# r240 — UART 전담 I/O와 ROS 분리

앞선 [r239](../sensor_link_r239_v1/README.md)의 331 ms 읽기 공백·140프레임 손실을 보존한다. 동일 펌웨어 `2c272833…66915`에서 호스트만 바꿨다. `SerialIngress` 한 작업이 포트 열기/읽기/쓰기/닫기를 담당하고 원문·최초 읽기 시각을 유한 큐로 ROS 소유자에게 넘긴다. ROS는 파싱·발행·제어 서비스만 처리한다.

큐는 최대512개, 읽기 한 건 최대1024 B·2 ms timeout이다. 수신 큐가 차면 명시 오류로 종료하고 자동 재연결/재송신하지 않는다. 원 수신의 `CLOCK_MONOTONIC`/`CLOCK_REALTIME`을 보존하므로 큐에서 기다린 자료를 최신 자료로 꾸미지 않는다. 여전히 미교정 읽기 시각 추정이며 실제 IMU 취득시각 보증은 아니다. `before_sources.zip`에 원 브리지를 보존했다.

## 실제 보드 20초 대표

- 원문 **11,967개·순번 공백0·CRC0·SPI/송신 큐/FIFO 오류0**.
- 공통 `MotionInput` **3,899/3,899 수락·거부0**. 독립 struct/CRC 순회와 저장 행, ROS 값/시각 일치.
- ARM `output_not_ready` 거부·CLEAR `not_ready` 거부·STOP 응답3개. 서비스 왕복10.75–14.23 ms, DRIVE 송신0, 가짜 엔코더0피드백 없음.
- 호스트 큐 최고1개, 큐 지연 최대0.553 ms·p95 0.130 ms, 공통 입력 callback 나이 최대17.76 ms·p95 6.75 ms.
- 포트·세 자식 프로세스 정상 종료, 원 IMU 앱 복구/readback 일치. bootloader·partition·NVS·전력 설정은 그대로다.

별도 보드 실행이므로 r239의 원인 전체가 해결됐다고 단정하지 않는다. 이번 정상 실행에는 331 ms 처리 정지가 재현되지 않았으므로 [r241](../sensor_link_stall_r241_v1/README.md)에서 ROS 소유자만350 ms 멈추는 대조를 했다. `serial_io.max_read_gap_ns`는 ROM 초기 출력과 앱 시작 사이도 포함하며 약631 ms이다. 이를 운용 IMU 중간 공백으로 해석하지 않는다. 원문 순번·gyro 시각 간격과 함께 읽는다.

PC의 단일 I/O 소유자·미처리 큐·오버플로 명시·부분 송신 실패 및 프레임 분리 검증은 **20 passed** (`tests.log`). 드문 GIL 전체 정지/OS·USB 고장·상시 운용 지연의 보장은 아니며, 출력 준비와 주행은 미완료다.

근거: `audit.json`, `bridge_summary.json`, `receiver_summary.json`, `supervisor.json`, flash/restore 요약, 소스/업로드 명세. Git 제외 회수본 `build/r240_results_private.zip` SHA256 `6364c29ad279c736e0c2aad25836262975b7adb3f5f8e20fcfe57e2bfcfec49c`.
