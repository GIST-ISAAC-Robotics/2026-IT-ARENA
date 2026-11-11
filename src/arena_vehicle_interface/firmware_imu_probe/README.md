# ESP32-S3·LSM6DSOX 센서 진단 앱

실물 식별과 원시 FIFO 기록을 위한 앱이다. 모터 PWM/DIR·서보·엔코더/구동 명령 처리는 없고 보정 계수를 적용하거나 NVS에 저장하지 않는다. 현재 진단 통신은 실제 조회가 성립한 UART0 GPIO43/44·921600 baud를 사용한다. 보드 제조사나 다른 호환 보드의 USB 배치를 이 코드로 확정하지 않는다.

- 배선: 3V3→VIN, GND→GND, GPIO12→SCL, 11→SDA, 13→DO, 10→CS, 9→I1. [실물 표기와 계획](../../../docs/hardware/ESP32_IMU_BRINGUP_PLAN_2026_11_11.md)을 따른다.
- 센서: SPI mode 3·1 MHz, ±8g·±500 °/s, 명목 accel/gyro 208 Hz. 압축하지 않은 FIFO 원순서·tag·센서 timestamp raw를 전송한다. INT1은 FIFO watermark 알림으로 설정한다.
- 시작은 센서 ODR OFF/대기 상태다. UART `S`는 수집 시작, `X`는 FIFO bypass와 ODR OFF, `R`은 MCU 소프트웨어 재부팅이다. 시작 뒤 600초가 지나면 센서 취득을 종료한다. 전원 재인가 시 자동 측정하지 않는다. 대기 중에도 약 1초마다 상태 프레임은 송신한다.
- UART 송신 ring은 4096바이트의 유한 버퍼다. 송신은 해당 버퍼의 공간을 기다릴 수 있으므로 하드 실시간/무손실을 보장하지 않는다. 실제 FIFO overrun·프레임 sequence/CRC를 함께 확인한다. 읽기 주기는 폴링이고 인터럽트 카운터는 별도 관측이다.
- PSRAM 초기화/SDK 메모리 검사 외에 **할당한 256 KiB**에 패턴 쓰기/읽기를 수행한다. 전체 Flash 쓰기 내구성 시험은 하지 않는다. 외부 장치 연결 전 수행한 컴파일 검사는 실제 보드 동작 결과와 구분한다.

## 재현

[`build_imu_probe.sh`](../../../scripts/build_imu_probe.sh)는 기존 PC/WSL ESP-IDF 환경과 ST 드라이버 커밋 `2e86793125f7f46b6195fce93a2b3cb76a51e564`를 사용한다. 인수는 새 빌드 경로와 해당 드라이버 경로다. 처음 올리기 전에 원래 Flash 전체를 비공개 경로에 백업한다. 빌드의 `flash_args`/해시/`sdkconfig`를 보존하고 설치된 esptool 버전의 명령으로 올린다.

수신은 [`probe_esp32_imu.py`](../../../scripts/probe_esp32_imu.py)로 한다. 포트는 VID/PID·고유 정보·물리 경로를 확인해 선택하고 다른 장치의 번호를 재사용하지 않는다. 기본 10초 준비 뒤 300초를 기록하며 끝에 `X`를 보내고 비활성 상태 응답을 확인한다. 원문 UART와 전체 표본은 Git 제외 경로에 둔다.

## 진단 프레임 v1

Little endian·고정 **68바이트**, Python 구조체 `<IHBBIIQQIHBB16sIII`다. 순서는 magic `IMU1`, 전체 길이, 버전, type, boot ID, sequence, MCU 읽기 시작/끝 us, INT1 누적 카운터, FIFO level/status, 원래 tag, payload 16바이트, SPI 오류 누적, UART 송신 오류 누적, CRC32다. CRC는 앞 64바이트에 표준 CRC32를 적용한다. UART0 부트 ROM 출력과 부분 프레임은 magic/길이/버전/CRC로 재동기화하며 버린 바이트도 집계한다.

| type | payload/의미 |
|---|---|
| 1 | 원래 FIFO tag는 tag 필드, 뒤 6바이트를 payload 시작에 보존. tag>>3이 1이면 gyro, 2이면 accel, 4이면 timestamp. 나머지 바이트 0 |
| 2 | `<IIIhBB`: 센서 timestamp raw, gyro·accel 읽은 수, 온도 raw, 측정 활성 표시, 예약 |
| 3 | `<IIII`: 실제 Flash 크기, PSRAM 크기, 할당 메모리 시험 오류 수, reset reason. tag는 WHO_AM_I |
| 4 | tag가 시작 레지스터 주소이고 payload는 연속 16바이트 readback |
| 5 | payload 시작 uint32는 FIFO overrun을 관측한 폴링 횟수. 누락 표본 개수가 아님 |

동일 raw값을 반복했다는 이유로 표본을 버리지 않는다. FIFO 내부 취득 누락이 없다는 주장은 sequence/CRC만으로 하지 않는다. MCU 읽기 시각과 센서 timestamp를 구분하며, tag별 timestamp 대응·호스트 시계 연결이 검증되기 전에는 제어기의 취득 시각으로 채택하지 않는다. 원시 축은 유지하고 SI 변환은 수신기에서 한 번 한다. 영점 후보와 잔차는 오프라인 참고 통계로만 계산하며 장착 후 재확인한다.
