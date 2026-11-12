# ESP32 GPIO·전원 연결표 v1

갱신: 2026-11-13. ①~④는 관리명이며 물리 핀 순번이 아니다.

[설계 설명](../ESP32_GPIO_WIRING_2026_11_11.md) · [전체 배선도](esp32_gpio_wiring_v1.html)

## 전원 헤더: 3V3 2개 / GND 4개

| ESP32 관리명 | 연결 대상 | 역할 |
|---|---|---|
| 3V3① | Adafruit 4438 하단 VIN | IMU 전원 |
| 3V3② | T3V3 분배점 → CF VA + OE | 한 3V3 핀을 두 갈래로 분기 |
| GND① | Adafruit 4438 하단 GND | IMU 접지 |
| GND② | TGND 분배점 → CF GND + 엔코더 pin 2 + J01 CN5(CH4)− + STOP 스위치 선 2 | 센서/CF/STOP의 명시적 공통 접지 |
| GND③ | MD20A Signal GND | PWM/DIR 신호 기준 |
| GND④ | Adapter A H2 GND | UART 신호 기준 |

## 공통 분배점

| 분배점 | 공통으로 연결할 단자 | 제작 |
|---|---|---|
| T3V3 | ESP32 3V3② ↔ CF VA ↔ CF OE | 3접점 공통 분기: 기존 선의 납땜+절연 또는 체결 가능한 단자대. 별도 전원 모듈 아님. |
| T5V | J01 CN5(CH4)+ ↔ PGE-213 pin 1 VCC ↔ CF VB | 기존 CH4 +5 V 분기선 유지 |
| TGND | ESP32 GND② ↔ J01 CN5(CH4)− ↔ PGE-213 pin 2 GND ↔ CF GND ↔ STOP 스위치 선 2 | 5접점 공통 접지. 기존 엔코더/CF GND 분배점에 STOP 선 2를 추가한다. ESP32 GND② 하나로 연결하며 기존 4개 GND 핀 배정을 유지한다. |

## GPIO 신호

| 기능 / net | ESP32 | 연결 대상 | 확인 범위 |
|---|---|---|---|
| IMU_SCLK | GPIO12 | Adafruit 4438 하단 SCL | SPI 실측 |
| IMU_MOSI | GPIO11 | Adafruit 4438 하단 SDA | SPI 실측 |
| IMU_MISO | GPIO13 | Adafruit 4438 하단 DO | SPI 실측 |
| IMU_CS_N | GPIO10 | Adafruit 4438 하단 CS | SPI 실측 |
| IMU_INT1 | GPIO9 | Adafruit 4438 하단 I1 | 사용자 배선, IRQ 동작 미검증 |
| ENC_A_3V3 | GPIO4 | CF-0108 A1 | 공통 GND·CF 교체 후 손 회전 및 약5% 모터 계수 확인; 고속/파형 미검증 |
| ENC_B_3V3 | GPIO5 | CF-0108 A2 | 공통 GND·CF 교체 후 손 회전 및 약5% 모터 계수 확인; 고속/파형 미검증 |
| MOTOR_PWM | GPIO15 | MD20A PWM | 20 kHz·약5% 단발 구동/PWM0 정지 확인; 운영 제어 미채택 |
| MOTOR_DIR | GPIO6 | MD20A DIR | 20 kHz·약5% 단발 구동/PWM0 정지 확인; 운영 제어 미채택 |
| SERVO_TX | GPIO17 | Bus Servo Adapter (A) H2 TX/TXD | 점퍼 두 개 A, ID1/1 Mbps 왕복 실측 |
| SERVO_RX | GPIO18 | Bus Servo Adapter (A) H2 RX/RXD | 점퍼 두 개 A, ID1/1 Mbps 왕복 실측 |
| STOP_REQUEST | GPIO8 | STOP 스위치 선 1 (선 2 → TGND) | 조립 배정; 내부 풀업, O/개방=HIGH 정지·I/도통=LOW 허가 조건; 펌웨어/실물 미검증 |

## PDB 채널·커넥터·정격

| 보드 | 채널 | 커넥터 | 전압 | 출력 전류 한도 | 조립 기준 연결 |
|---|---|---|---|---|---|
| J01 | 입력 | CN1 | 21–36 V DC | — | 배터리 + / − |
| J01 | CH1 | CN2 | 19 V | 최대 2.3 A | Jetson DC 입력 |
| J01 | CH2 | CN3 | 5 V | CH2+CH3 합계 최대 1.5 A | 미사용 / 예비 |
| J01 | CH3 | CN4 | 5 V | CH2+CH3 합계 최대 1.5 A | 미사용 / 예비 |
| J01 | CH4 | CN5 | 5 V | 최대 0.5 A | T5V → 엔코더 VCC + CF VB |
| J01 | CH5 | CN6 | 3.3 V | CH5+CH6 합계 최대 0.6 A | 미사용 / 예비 |
| J01 | CH6 | CN7 | 3.3 V | CH5+CH6 합계 최대 0.6 A | 미사용 / 예비 |
| M01 | 입력 | CN1 | 24–36 V DC | — | 배터리 + / − |
| M01 | CH1 | CN2 | 입력 전압 그대로 | 입력 전류 공급 능력에 따름; 고정 A 정격 미제시 | MD20A VB+ / VB− |
| M01 | CH2 | CN3 | 입력 전압 그대로 | 입력 전류 공급 능력에 따름 | 미사용 / 예비 |
| M01 | CH3 | CN4 | 입력 전압 그대로 | 입력 전류 공급 능력에 따름 | 미사용 / 예비 |
| M01 | CH4 | CN5 | 입력 전압 그대로 | 입력 전류 공급 능력에 따름 | 미사용 / 예비 |
| M01 | CH5 | CN6 | 12 V | 이 채널 최대 2.0 A | Adapter A 전원 입력 → 서보 1개 |
| M01 | CH6 | CN7 | 12 V | 이 채널 최대 2.0 A | 예비; CH5와 병렬 금지 |

서보 어댑터와 ST3215-HS는 **3선 전용 케이블 1개**로 연결한다(D/DATA, V+, G/GND). 이는 기능 이름이며 실제 좌우 핀 순서가 아니다. 서보 전원 CH5(CN6)는 조립 기준 선택이며 CH6(CN7)은 예비다.

배터리 공통 +/− 2심을 상부 1단 J01 근처까지 올린 뒤 Y 케이블로 J01 CN1·M01 CN1에 분기한다. 각 가지에서 +끼리, −끼리를 연결한다.

케이블 묶음 표기: J01 CN2의 +/− 두 선은 한 전원 케이블로 Jetson 배럴잭에 연결한다. B0390은 22↔22 FPC 1개, C1은 지급 어댑터를 거친 USB 데이터 케이블 1개다. 굵은 선은 케이블 외피를 뜻하며 내부 도체를 단락시키는 표시가 아니다. [층별 케이블 배치도](vehicle_wiring_layers_v1.html).

전류 한도는 소비 전류가 아니다. J01 CH2+3 및 CH5+6은 각각 합계 한도, M01 CH5/6은 각 채널 한도다. 모터·서보 전력 귀환은 PDB로 직접 배선한다.

물리 STOP: 스위치 선 1 → GPIO8, 선 2 → TGND(기존 공통 GND에 1분기 추가). GPIO8은 내부 풀업 입력으로 설정하며 별도 3V3선은 연결하지 않는다. O/개방은 HIGH=STOP, I/도통은 LOW=허가 조건이다. I로 복귀해도 자동 출발하지 않는다. 무전원 도통 검사로 I/O 접점을 확인한다. 배선 설계 반영이며 펌웨어·실물 STOP 시험 완료를 뜻하지 않는다.

자동 생성: `python scripts/render_esp32_wiring.py`. 원본: [esp32_gpio_v1.json](esp32_gpio_v1.json).
