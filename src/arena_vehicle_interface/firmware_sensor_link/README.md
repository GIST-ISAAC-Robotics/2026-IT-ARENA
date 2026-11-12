# 실제 IMU + 출력 비활성 MCU 계약

조립 전 한 UART에서 센서와 명령 응답을 함께 받기 위한 앱이다. **PWM/DIR·서보·엔코더 피드백·PI 출력은 없다.** `ActuationBenchEndpoint`의 `output_ready=false`를 그대로 사용하며 ARM은 거부하고 STOP은 처리한다. 모터축 설정은 28카운트·2.5×81/17·65 mm이며, 설정값이 있다는 사실이 엔코더 작동 증거는 아니다.

UART0 GPIO43/44·921600·8N1, LSM6DSOX SPI GPIO10/11/12/13·mode3·1 MHz, I1 GPIO9는 확인된 probe와 같다. 콘솔을 끄고 ROM 초기 출력은 호스트 프레임 분리기가 버린다. IMU는 부팅 후 자동 시작하며 기존 probe의 10분 센서 실행 제한을 유지한다. `S/X/R` 문자 명령은 사용하지 않는다. STOP은 구동 계약의 정지이며 센서 취득을 끄는 명령이 아니다.

`imu_source.c`는 검증 probe에서 UART 초기화/송신/문자 명령 루프만 분리하고 FIFO 한 순회 상한을 128→8항목으로 줄인 센서 사본이다. 원 파일은 보존했다. raw FIFO tag, 센서 timestamp, MCU SPI 읽기 구간, 오류 계수와 68 B IMU1 CRC 형식은 유지한다. 취득 시각 교정이나 축/영점 보정은 추가하지 않았다.

한 작업이 UART RX, 5 ms 계약 tick, IMU SPI, TX를 소유한다. IMU 큐는 64프레임이며 넘치면 원 순번 공백과 tx_errors로 드러난다. `SensorLinkTx`는 IA 계약 프레임과 IMU1 원문을 완전한 프레임 경계에서 교대하고, UART 부분/0바이트 수락 때 이어 보낸다. IA 프레임 중간에 IMU 바이트를 넣지 않는다. UART 오류 뒤에는 연결을 끊고 센서를 중지하며 자동 재허가하지 않는다.

`scripts/build_sensor_link.sh`는 빌드만 수행한다. 호스트의 `bridge_sensor_link_preview.py`는 전담 I/O 작업이 한 번만 포트를 열며 IA 응답→`/actuation/rx`, IMU→`/hardware_preview/imu`를 보낸다. 송신은 ARM/CLEAR/DISARM/STOP만 허용하고 DRIVE를 거절한다. 이 진단 프로그램을 실차 구동 브리지로 사용하지 않는다. 최초 손실은 [r239](../../../artifacts/validation/2026-11-12/sensor_link_r239_v1/README.md), 분리 후 실물 수신·원 앱 복구 결과는 [r240](../../../artifacts/validation/2026-11-12/sensor_link_ingress_r240_v1/README.md)과 [r241 처리 지연](../../../artifacts/validation/2026-11-12/sensor_link_stall_r241_v1/README.md)을 따른다.

이 앱은 송신 코어의 기본 OFF인 `offer_status_v1`을 명시적으로 켜서 기존 offer에 MCU 샘플 시각·실제 ESTOP 래치·이유를 함께 보낸다. 브리지의 `--mcu-status`를 선택하면 `/hardware_preview/actuation_status`에 관측 상태를 추가로 발행한다. 원 MCU 시각과 실제 I/O 수신 시각을 보존하며, 최소 수신 offset에 의한 추정은 `time_contract_verified=false`다. 없는 엔코더나 출력 준비 상태를 만들지 않는다. 물리 비상정지 버튼 배선이나 운영 구동 허가를 구현한 것은 아니다.

[r254 실제 연결](../../../artifacts/validation/2026-11-12/mcu_status_r254_v1/README.md)에서 상태976개와 공통 관측 수락의 일치, IMU 수신·STOP 응답·원 앱 복구를 확인했다. 공통 관련93검사·소스 출처214검사와 `arena_vehicle_interface` 빌드를 통과했다. 센서 취득 시각 교정·실제 엔코더/모터·운영 상태 발행은 후속이다.
