# ESP32 GPIO 연결표 v1

[설계 설명](../ESP32_GPIO_WIRING_2026_11_11.md) · [배선도](esp32_gpio_wiring_v1.html)

| 기능 / net | ESP32 | 연결 대상 | 현재 상태 |
|---|---|---|---|
| IMU_SCLK | GPIO12 | Adafruit 4438 하단 SCL | SPI 실측 |
| IMU_MOSI | GPIO11 | Adafruit 4438 하단 SDA | SPI 실측 |
| IMU_MISO | GPIO13 | Adafruit 4438 하단 DO | SPI 실측 |
| IMU_CS_N | GPIO10 | Adafruit 4438 하단 CS | SPI 실측 |
| IMU_INT1 | GPIO9 | Adafruit 4438 하단 I1 | 사용자 배선, IRQ 동작 미검증 |
| ENC_A_3V3 | GPIO4 | 신규 CF-0108 A1 (실크 확인, 도통 미검증) | 신규 CF 실크 확인, 도통/OE·파형 미검증 |
| ENC_B_3V3 | GPIO5 | 신규 CF-0108 A2 (실크 확인, 도통 미검증) | 신규 CF 실크 확인, 도통/OE·파형 미검증 |
| MOTOR_PWM | GPIO15 | MD20A PWM | 설계, 실물 구동 미검증 |
| MOTOR_DIR | GPIO6 | MD20A DIR | 설계, 실물 구동 미검증 |
| SERVO_TX | GPIO17 | Bus Servo Adapter (A) H2 TX/TXD | 공식 동일 이름 결선, 실물 미검증 |
| SERVO_RX | GPIO18 | Bus Servo Adapter (A) H2 RX/RXD | 공식 동일 이름 결선, 실물 미검증 |

자동 생성: `python scripts/render_esp32_wiring.py`. 원본: [esp32_gpio_v1.json](esp32_gpio_v1.json).
