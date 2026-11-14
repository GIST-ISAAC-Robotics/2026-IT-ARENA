# ESP32 없는 Jetson 직접 제어 배선안 v1

**추가 부품 없는 연결 후보: 실제 pinmux·신호 정합·전원·정지 검증 전이며 통전 지시가 아닙니다.** [현재 검토 결과와 실측 대기](jetson_existing_parts_check_2026_11_14.md).

[한 장 배선도](jetson_direct_wiring_v1.html) · [SVG](jetson_direct_all_v1.svg) · [기존 ESP32 도면](esp32_gpio_wiring_v1.html)

J12 BOARD 물리 핀 번호. SoC GPIO 번호가 아님. 실제 핀1 방향은 보드에서 확인입니다. ESP32 GPIO 번호와 혼동하지 않습니다.

## USB 연결

| 기능 | 호스트 | 장치 | 케이블 | 조건 / 현재 상태 |
|---|---|---|---|---|
| 조향 | Jetson USB-C (Host) | Bus Servo Adapter(A) USB-C | 데이터 지원 USB-C ↔ USB-C | 점퍼캡 2개 B; 외부 UART TX/RX/GND 미연결; 별도12V 필요; USB 등록·통신 미시험 |

## J12 신호 연결표

| BOARD 핀 | net | 기능 | 연결 대상 | 미확인 / 변경 |
|---|---|---|---|---|
| 3 | IMU_SDA | I2C SDA | Adafruit 4438 하단 SDA | SPI에서 I2C로 변경; 드라이버·통신 미검증 |
| 5 | IMU_SCL | I2C SCL | Adafruit 4438 하단 SCL | 3.3 V 구동; 기존 SPI 펌웨어 재사용 불가 |
| 29 | MOTOR_DIR | GPIO output / PQ.05 | MD20A DIR | 전진/후진 부호와 부팅 LOW 미검증 |
| 31 | ENC_A_3V3 | GPIO input / PQ.06 | CF A1 (현재 직결 미채택) | TXS 내부풀업·지속 구동으로TXB ±2mA 요구 충족 근거 없음 |
| 32 | MOTOR_PWM | PWM / PG.06 / 32e0000.pwm | MD20A PWM | 20 kHz 후보; pinmux·파형·0출력·종료·OS정지 시 차단 미검증 |
| 33 | ENC_B_3V3 | GPIO input / PH.00 | CF A2 (입력 정합 확인 경로) | 이 핀은 PWM이 아닌 GPIO 입력으로 설정 |

## J12 전원·접지

| BOARD 핀 | 연결 대상 | 역할 |
|---|---|---|
| 1 | T3V3 → IMU VIN + IMU CS + CF VA/OE | 3.3 V 공급; J12 전체 합계100 mA 이내 후보 |
| 6 | IMU GND | 신호/전원 GND; 하단 DO는 미연결 |
| 9 | TGND → CF GND + 엔코더 GND + J01 CN5− | 센서 공통 기준; 모터 전력 귀환 아님 |
| 25 | MD20A 신호 GND | PWM/DIR 기준; VB− 전력선 별도 |

## 기존 PDB 채널 유지

| 보드 | 채널 / 단자 | 전압 | 한도 | 대상 |
|---|---|---|---|---|
| J01 | CH1 / CN2 | 19 V | 최대 2.3 A | Jetson DC 입력 |
| J01 | CH2 / CN3 | 5 V | CH2+CH3 합계 최대 1.5 A | 미사용 / 예비 |
| J01 | CH3 / CN4 | 5 V | CH2+CH3 합계 최대 1.5 A | 미사용 / 예비 |
| J01 | CH4 / CN5 | 5 V | 최대 0.5 A | T5V → 엔코더 VCC + CF VB |
| J01 | CH5 / CN6 | 3.3 V | CH5+CH6 합계 최대 0.6 A | 미사용 / 예비 |
| J01 | CH6 / CN7 | 3.3 V | CH5+CH6 합계 최대 0.6 A | 미사용 / 예비 |
| M01 | CH1 / CN2 | 입력 전압 그대로 | 입력 전류 공급 능력에 따름; 고정 A 정격 미제시 | MD20A VB+ / VB− |
| M01 | CH2 / CN3 | 입력 전압 그대로 | 입력 전류 공급 능력에 따름 | 미사용 / 예비 |
| M01 | CH3 / CN4 | 입력 전압 그대로 | 입력 전류 공급 능력에 따름 | 미사용 / 예비 |
| M01 | CH4 / CN5 | 입력 전압 그대로 | 입력 전류 공급 능력에 따름 | 미사용 / 예비 |
| M01 | CH5 / CN6 | 12 V | 이 채널 최대 2.0 A | Adapter A 전원 입력 → 서보 1개 |
| M01 | CH6 / CN7 | 12 V | 이 채널 최대 2.0 A | 예비; CH5와 병렬 금지 |

## IMU 국소 연결

- 하단 CS → T3V3 (I2C 모드)
- 하단 DO는 미연결; 기존 주소 선택을 유지하고0x6A/0x6B 확인
- 뒷면 주소 점퍼 SJ1이 닫힌 상태에서 DO를 GND에 연결하지 않음 (3.3V 단락 위험)
- 3Vo·I1·I2·SCX·SDX·상단 CS/DO는 이번 안에서 미연결

## 실물 연결 전 해결할 항목

- 발열 ESP32는 USB·전원·신호에서 완전히 분리; 기존 외부 배선 이상 미확정
- 서보는USB 경로 채택; J12 UART 미사용으로TX 풀업 조건 문제 회피. USB 등록·직접 통신 구현/시험은 별도
- CF TXS→Jetson TXB 입력 구동 요구 충족 근거 없음; 전압3.3V만으로 직결 승인 불가
- J12 I2C 이외 신호의 외부 pull-up/down은 >50 kΩ 조건; 10kΩ을 임의 추가하지 않음
- J12 3.3V와 PDB 3.3V/5V를 병렬 연결하지 않음; 총부하100mA와 전원 순서 확인
- 물리STOP은 사용자 지시로 제외. 명령 시간 만료·실행 오류 시 PWM0 처리는 구현/검증 대상
- 27/28 엔코더 우회안 미채택: USB-C/전력감시 I2C 공유와 LOW 여유 문제
- Jetson 전원OFF 사용자 확인: 이번 자료 검토 뒤 새 실물 통신·GPIO/PWM 출력 검증 없음

I2C 선택 이유: Adafruit4438 SCL/SDA의10kΩ 풀업을 TXB0108 경유 SPI에 그대로 옮기지 않고, J12의 I2C 전용 핀을 사용합니다. 기존 SPI 앱은 직접 재사용하지 않습니다.

물리 STOP은 사용자의 최신 지시로 제외했습니다. 핀15와 스위치 배선은 사용하지 않습니다. 나머지 여섯 기능은 유지하며 명령 만료 시 PWM0 처리가 필요합니다. OS 정지에 독립적인 모터 전력 차단은 구현되지 않았습니다.

조향은USB-C로 연결하므로 미채택J12 UART안의풀업 조건 문제를 피합니다. CF의TXS 내부풀업·지속 구동으로Jetson TXB의±2mA 요구 충족을 보증할 수 없어 엔코더 직결은 미채택입니다. 상세 공식 근거는 위 현재 검토 결과에 기록했습니다.

## 근거

- [추가 부품 없는 현재 검토 결과·실측 대기](jetson_existing_parts_check_2026_11_14.md)
- [기존 ESP32 연결표](esp32_gpio_pin_table_v1.md)
- [ESP32 없는 제어 조사](../MOUNTED_DC_TEST_2026_11_14.md#esp32-없이-제어하는-대안-조사)
- [NVIDIA carrier SP-11324 v1.3 (§3.3, Table3-3, Table5-3)](https://developer.nvidia.com/downloads/assets/embedded/secure/jetson/orin_nano/docs/jetson_orin_nano_devkit_carrier_board_specification_sp.pdf)
- [NVIDIA Jetson.GPIO 핀 정의](https://github.com/NVIDIA/jetson-gpio/blob/master/lib/python/Jetson/GPIO/gpio_pin_data.py)
- [NVIDIA GPIO/PWM 및 pinmux](https://github.com/NVIDIA/jetson-gpio#11-pwm)
- [Adafruit 4438 핀 설명](https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/pinouts)
- [Adafruit 4438 공식 회로도 / 주소 SJ1](https://github.com/adafruit/Adafruit-LSM6DSOX-PCB/blob/master/Adafruit_LSM6DSOX.sch)
- [TI TXB0108 입력 구동 조건](https://www.ti.com/lit/ds/symlink/txb0108.pdf)
- [Cytron MD20A 사양](https://www.cytron.io/p-20amp-6v-30v-dc-motor-driver)
- [Waveshare USB 제어·B점퍼 공식 연결](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example)
- [NVIDIA USB-C Host 모드](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/hardware_layout.html)

NVIDIA carrier specification v1.3 §3.3/Table3-3/Table5-3은 공식 검색 색인 원문으로 확인했습니다. 직접 PDF 열기는403이었고 실제 설치 pinmux는 SSH 시간 초과로 미확인입니다. J12의 핀당1A 커넥터 정격과3.3V 공급합계100mA 예산을 구분합니다.

생성: `python scripts/render_jetson_direct_wiring.py`. 설계 원본: `jetson_direct_v1.json`. 기존 ESP32 도면 파일은 덮어쓰지 않습니다.
