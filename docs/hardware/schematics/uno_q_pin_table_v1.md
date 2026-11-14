# Jetson + Arduino UNO Q 배선 설계 v1

**현장 보유 UNO Q의 선정 설계. 펌웨어 이식·실물 검증 전이며 현재 구동 명령이 아닙니다.**

[확대 가능한 전체도](uno_q_wiring_v1.html) · [PNG](uno_q_all_v1.png) · [설계 JSON](uno_q_v1.json)

UNO Q의 D/A는 실크 이름, Jetson J12는 BOARD 물리 번호입니다. 실제 위치·방향은 보드 실크로 확인합니다.

| 기능 | UNO Q 실크 | STM32 핀/기능 | 연결 대상 |
|---|---|---|---|
| HOST_RX | D0 | PB7 / USART1_RX | Jetson J12 pin8 TX |
| HOST_TX | D1 | PB6 / USART1_TX | Jetson J12 pin10 RX |
| ENC_A | A2 | PA6 / TIM3_CH1 AF2 | CF A1 (3.3V) |
| ENC_B | A3 | PA7 / TIM3_CH2 AF2 | CF A2 (3.3V) |
| MOTOR_PWM | D9 | PB8 / TIM4_CH3 AF2 | MD20A PWM |
| MOTOR_DIR | D4 | PA12 / GPIO output | MD20A DIR |
| IMU_SCK | D13 | PB13 / SPI2_SCK | Adafruit4438 SCL |
| IMU_MOSI | D11 | PB15 / SPI2_MOSI | Adafruit4438 SDA |
| IMU_MISO | D12 | PB14 / SPI2_MISO | Adafruit4438 DO |
| IMU_CS_N | D10 | PB9 / GPIO output | Adafruit4438 CS |
| IMU_INT1 | D2 | PB3 / GPIO EXTI3 | Adafruit4438 I1 |

## 전원과 공통 연결

| 항목 | 연결 |
|---|---|
| 전원 VIN | M01 CH6/CN7 + → UNO Q VIN; CN7 − → GND② (JANALOG7) |
| 3.3 V | UNO Q 3V3 → IMU VIN + CF VA/OE (T3V3) |
| 5 V | J01 CH4/CN5 + → PGE VCC + CF VB (T5V) |
| UART GND | GND① (JDIGITAL15) → Jetson J12 pin14 |
| 센서 GND | GND③ (JANALOG6) + IMU + CF + PGE + J01 CN5 −: TGND 5접점 |
| 구동 GND | GND④ (JSPI6) → MD20A 신호 GND; 전력 −는 M01 직접 귀환 |
| 서보 | Jetson USB-C → 데이터 C-to-C → Adapter(A) USB-C, 점퍼2개 B |
| 카메라/라이다 | 기존 Jetson CSI FPC / USB-A 그대로 유지 |
| 미연결 | 발열 ESP32 전부, UNO Q USB/5V/IOREF, 물리 STOP, Adapter H2 UART |

## 구현 및 통전 전 조건

- 기존 ESP32와 그 USB·신호·전원은 제거한다. 보드 단독 이상과 외부 하네스 건전성은 별도이며 외부 전압/극성을 확인한다.
- UNO Q에는 M01 CH6/CN7의12V를 VIN/GND로 공급한다. 6S 배터리 직접 연결·5V/3V3 핀에12V 연결·CH5/CH6 병렬은 금지한다. CN7 케이블/접점은 실물 확인한다.
- M01 입력 정격은24–36V이다. 6S가24V 아래인 구간의 동작은 보증되지 않는다. CH6 정격24W와 UNO Q 공급기준15W를 비교한 여유는 부팅 피크/효율/배선 손실 실측을 대체하지 않는다.
- UNO Q USB는 운영 중 연결하지 않는다. USB와 VIN의 diode-OR는 VIN 우선이나 USB 전류0을 보장하지 않는다. 플래시/정비 시 구동부 전원을 분리하고 전원 구성을 다시 확인한다.
- UART는J12-8 TX→D0 RX, D1 TX→J12-10 RX, J12-14 GND↔UNO Q GND이다. 1Mbps 8N1을 양쪽에서 설정한다. J12 물리 핀을 SoC GPIO 번호와 혼동하지 않는다.
- UART 양 핀에 no-pull/bias-disable을 명시한다. STM32 내부30–50kΩ pull을 사용하면 Jetson TXB의>50kΩ 외부 pull 조건과 충돌한다. 외부 풀업도 추가하지 않는다.
- Jetson pinmux/장치 노드와 콘솔 소유권을 확인한다. UNO Q USART1의console/shell/UART logging을 해제하고 Serial1 또는 전용 HAL로 단독 사용한다. Serial/Monitor/Bridge는 제어 경로에서 사용하지 않는다.
- TIM3은A2/PA6·A3/PA7의encoder 전용이다. 기존PWM 노드를 비활성화하고 prescaler0·x4를 설정한다. Arduino 표의A1/PA5 TIM3 표기는ST공식 AF와 다르므로 사용하지 않는다.
- TIM4 PWM은D9/PB8 CH3만 활성화하고 D10/PB9는IMU CS GPIO로 전용한다. D2는IMU IRQ 전용이며 JTAG/SWO/TIM2 기능을 겹치지 않는다.
- 20kHz PWM은50,000ns 주기의 오류 검사 가능한 HAL로 구현한다. analogWrite의PWM 준비 실패 시 digital HIGH 대체를 구동에 사용하지 않는다. 초기/오류/명령 만료 PWM0 및 재접속 자동 구동 금지를 구현한다.
- MCU 명령 만료는 MCU 시계와 별도 주기 작업으로 처리한다. UART write/RPC 대기에 종속하지 않는다. RAM/flash 업로드와 전원 재인가 뒤 MCU 시작 절차를 확인한다.
- 한쪽 전원OFF 상태의UART 역급전/입력 상태, 부트 출력, 리셋과brownout을 실측한다. 독립 전원 핫플러그 동작이나 냉간 부팅 안전성을 이 문서만으로 보증하지 않는다.
- 엔코더 최대 약5898edges/s의 기준 계수, IMU 약200Hz의누락/시각, UART CRC/sequence, 전체 부하의실제PWM0를 검증한다. 물리 STOP은 사용자 지시로 제외하고 다른 기능은 줄이지 않는다.
- GND①~④는 도면 관리명이며 서로 다른4개 헤더 접점이다. JDIGITAL15/JANALOG7/JANALOG6/JSPI6의GND 실크·커넥터 방향을 확인한다. 신호D6나5V/RESET과 혼동하지 않는다. T3V3는4접점, TGND는5접점 분기다.
- IMU SPI 사용 시 주소 선택 SJ1(0x6B solder jumper)은 열림 상태인지 확인한다. DO/MISO를3Vo나GND에스트랩하지않는다. 기존SPI성공기록만으로현재분해/재배선상태를확정하지않는다.

## 근거

- [Arduino UNO Q 공식 데이터시트: 전원·헤더](https://docs.arduino.cc/resources/datasheets/ABX00162-ABX00173-datasheet.pdf)
- [Arduino UNO Q 공식 회로도: UART 0Ω 경로·전원 OR](https://docs.arduino.cc/resources/schematics/ABX00162-schematics.pdf)
- [ST STM32U585 DS13086: AF Table28·GPIO 전기 특성](https://www.st.com/resource/en/datasheet/stm32u585ai.pdf)
- [TI TXB0108: 최소±2mA와외부pull 조건](https://www.ti.com/lit/ds/symlink/txb0108.pdf)
- [제조사 M01: CH5/CH6 각각12V2A·입력24–36V](https://github.com/MOSW626/istech-it-arena/blob/main/docs/hardware/PM_KAIST-MR-M01_Preliminary.pdf)
- [Arduino core0.55: D0/D1은Serial1](https://support.arduino.cc/hc/en-us/articles/27251870677916-Migrating-to-Zephyr-core-0-55-0-on-UNO-Q)
- [Arduino core 공식 핀/타이머 overlay](https://github.com/arduino/ArduinoCore-zephyr/blob/0e2f265190d9fc4166a467215129e1ea29e3f1dd/variants/arduino_uno_q_stm32u585xx/arduino_uno_q_stm32u585xx.overlay)
- [Waveshare Adapter(A) USB·B점퍼](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example)
