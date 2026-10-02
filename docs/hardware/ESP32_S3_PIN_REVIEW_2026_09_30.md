# ESP32-S3-DevKitC-1-N8R8 구매 전 핀 검토

> [10/2 수정 구매안·현재 구성의 핀/USB 검토](ESP32_BUDGET_REVIEW_2026_10_02.md): Waveshare N8R8 핀 헤더 옵션·IMU를 포함한 목록과 모터축 엔코더 1개·MD20A 구성을 검토했다. 아래 9/30 배치는 당시 가정으로 보존한다.

날짜: 2026-09-30. **공식 DevKitC-1의 문서 기반 임시 배치**이며 구매 완료·배선 확정·펌웨어 실행·실차 안전 검증이 아니다. 사용자가 미정 부품에 합리적인 가정을 넣어 핀 여유를 확인하도록 요청했다.

## 결론과 가정

**현재 구성의 개발용 MCU로 DevKitC-1-N8R8을 선택하는 데 핀 부족 문제는 발견하지 않았다.** 메모리·부팅·USB·보드 LED 핀을 제외한 23개 중, 직렬 서보와 선택 입력까지 넣어 19개를 배치하고 4개를 남길 수 있다. 이 4개는 외부 JTAG와 겸용이며, 별도 JTAG 프로브를 쓰면 확장용으로 동시에 사용할 수 없다. 직접 USB 디버깅 경로는 별도로 남긴다.

- [구동 계약](ACTUATION_CONTRACT_2026_09_22.md): 단일 모터/ESC, 기계식 차동, 좌우 후륜 엔코더 2개를 유지한다. MCU가 BLDC 3상을 직접 구동하지 않는다.
- 엔코더는 A/B상 쿼드러처 2개를 가정한다. 공급 전압·출력 방식·최대 에지율은 미정이다.
- [IMU 후보](IMU_SELECTION_2026_09_19.md)는 Adafruit LSM6DSOX 4438, 일반 4선 SPI + INT1 데이터 준비 신호를 가정한다. SPI2를 사용하며 외부 메모리용 SPI0/1과 구분한다.
- Jetson은 보드 USB-UART를 통해 UART0으로 연결하는 안을 잡고, 칩 직접 USB도 유지한다. 카메라·LiDAR는 Jetson 측 장치이므로 MCU GPIO에 중복 산입하지 않는다.
- ESC는 우선 RC 펄스/PWM 명령 1핀으로 가정하고 UART·Classic CAN 대안을 별도로 계산한다. 펄스 폭·주기·중립·제동은 확정하지 않는다.
- [지원품 기록](../HARDWARE_AND_RULE_QUESTIONS.md)에 있는 ST3215-HS는 **TTL 직렬 버스 서보**다. 이전 대화의 일반 PWM 서보 1핀 예시는 이 지원품에 적용하지 않는다. UART1 TX/RX + 외부 버스 변환 회로 방향 제어 1핀을 보수적으로 확보한다. 실제 변환 회로가 자동 방향 전환형이면 방향 핀은 반환할 수 있다. 지원품 2개를 같은 버스에서 쓸 때는 서로 다른 ID로 연결할 수 있으며 전원 용량은 별도 계산한다.
- 정지 입력, 외부 구동 허가 회로, 외부 watchdog 진행 신호와 배터리 측정까지 넣는다. 선택 입력으로 고장 감시와 정지 해제 요청도 확보한다. 이 입력들은 안전 회로의 완성이나 규정 충족을 의미하지 않는다.

## 임시 배치

숫자는 기판 실크의 **GPIO 번호**이며 커넥터의 물리적 핀 순번이 아니다. 모든 외부 신호는 MCU 쪽에서 3.3 V 논리를 만족하도록 연결한다. 전원/GND는 아래 신호 핀 수에 포함하지 않았다.

| 기능 | GPIO | 신호/할당 가정 | 개수 |
|---|---|---|---:|
| 왼쪽 후륜 엔코더 | 4, 5 | A/B, PCNT 첫 유닛 | 2 |
| 오른쪽 후륜 엔코더 | 6, 7 | A/B, PCNT 두 번째 유닛 | 2 |
| IMU SPI | 12, 11, 13, 10 | 순서대로 SCLK, MOSI, MISO, CS; SPI2 기본 IO_MUX | 4 |
| IMU 데이터 준비 | 9 | INT1 입력. 일반 SPI라 QUADHD와 공유하지 않음 | 1 |
| ESC 명령 | 15 | 하드웨어 PWM/RC 펄스 출력 가정 | 1 |
| 조향 직렬 서보 | 17, 18 | UART1 TX/RX → 외부 TTL 버스 변환 회로 | 2 |
| 서보 송수신 방향 | 16 | 변환 회로의 방향 제어, 필요 시 사용 | 1 |
| 물리 STOP 상태 | 8 | 정지 회로/접점 상태 입력 | 1 |
| 구동 허가 요청 | 21 | 외부 허가/차단 회로에 전달하는 논리 신호 | 1 |
| 외부 watchdog | 47 | 정상 제어 작업 진행에 연계한 변화 신호 | 1 |
| 배터리 전압 | 1 | ADC1_CH0, 분압·필터·보호 회로 경유 | 1 |
| 선택: 외부 고장 상태 | 2 | ESC/전원/차단 회로가 제공할 때 사용 | 1 |
| 선택: 정지 해제 요청 | 14 | CLEAR 요청용 입력. 구동 ARM과 별개 | 1 |
| **사용 합계** | | 선택 입력과 방향 제어 핀 포함 | **19** |
| **남는 핀** | **39, 40, 41, 42** | 외부 JTAG 프로브와 겸용 | **4** |

Jetson USB-UART는 이미 보드에서 GPIO43/44에 연결되어 있다. 이 두 핀은 아래 제외 예산에 반영했으며 다른 장치에 재사용하지 않는다. UART0 통신에 부팅 로그가 섞일 수 있으므로 실제 프레이밍/로그 분리와 호스트 포트 열기 시 자동 리셋 동작을 확인해야 한다.

IMU 단자는 GPIO12→SCK, GPIO11→SDA/MOSI, GPIO13←DO(bottom)/MISO, GPIO10→CS(bottom), GPIO9←INT1로 대응한다. Adafruit 보드의 Vin은 MCU 논리에 맞는 3.3 V로 공급하는 안이다. 위쪽 보조 센서 단자와 혼동하지 않는다.

## 사용하지 않은 핀과 여유 계산

공식 보드 헤더에는 GPIO0–21과 GPIO35–48, 총 36개의 GPIO 위치가 있다. 메모리에 연결된 핀도 헤더에 보일 수 있으므로 전부 자유 핀으로 세면 안 된다.

| 제외 GPIO | 이유 | 개수 |
|---|---|---:|
| 35, 36, 37 | N8R8의 Octal PSRAM 내부 연결. 외부 사용 불가 | 3 |
| 0, 3, 45, 46 | 부팅 설정(strapping)에 영향을 주므로 이번 배치에서 회피 | 4 |
| 19, 20 | 칩 직접 USB 유지 | 2 |
| 43, 44 | 보드 USB-UART/UART0 유지 | 2 |
| 38, 48 | RGB LED가 초기판은 48, v1.1은 38. 판본 공통안을 위해 둘 다 회피 | 2 |
| **합계** | **36 − 13 = 23개를 배치 후보로 사용** | **13** |

GPIO26–34는 이 보드 헤더의 가용 핀으로 산입하지 않았다. GPIO39–42는 외부 JTAG 기능을 비활성화/재배치하고 사용할 수 있는 일반 GPIO다. 이번 기본안은 이 4개를 비워 두어 외부 JTAG도 보존한다. 보드 판본 확인 후 실제 LED에 연결되지 않은 38 또는 48을 되돌리는 방법도 있지만 이번 여유 수치에는 포함하지 않는다.

## 미정 사항이 달라질 때

아래 대안은 기본 19핀 구성과 각각 비교한 것이며 모든 선택 사항을 동시에 추가한 수치가 아니다.

| 변경 조건 | 배치 예시 | 사용/남는 GPIO |
|---|---|---|
| ESC가 UART 명령/피드백 방식 | PWM15 해제, UART2 TX39/RX40 추가 | 20 / 3 |
| ESC가 Classic CAN 방식 | PWM15 해제, TWAI TX39/RX40 + 선택 STBY41 확보 | 21 / 2 |
| 엔코더 2개에 Z상 추가 | GPIO39/40 입력 추가 | 21 / 2 |
| IMU가 I²C 방식으로 변경 | SPI 4선을 SDA10/SCL11 두 선으로 교체, INT1 유지 | 17 / 6 |
| 조향이 일반 PWM 서보로 변경 | UART17/18과 방향16을 해제하고 PWM16 사용 | 17 / 6 |
| UART ESC와 양쪽 엔코더 Z상을 함께 사용 | PWM15 해제, UART39/40, Z41/42 추가 | 22 / 1 |

CAN은 별도의 3.3 V MCU 호환 트랜시버·종단이 필요하다. MCU의 TX/RX를 CANH/CANL에 직접 연결하지 않는다. CAN FD나 전용 ESC 프로토콜은 이 가정에 포함하지 않았으며 실제 모델 확인 후 다시 판단한다. UART0(호스트)·UART1(서보)·UART2(ESC)로 역할을 나눌 수 있으나 통신 주기·버스 부하·실제 드라이버 동시 실행은 아직 시험하지 않았다.

## 실제 연결 전 남은 조건

1. **신호 전압:** ESP32 GPIO를 5 V 입력 허용으로 취급하지 않는다. 5 V push-pull 엔코더라면 변환 회로, 오픈 컬렉터라면 3.3 V 풀업과 출력 정격, 차동 엔코더라면 수신 회로를 확인한다. ESC의 3.3 V 입력 인식 조건도 확인한다.
2. **직렬 서보:** TX/RX를 무조건 서로 묶지 않는다. 적절한 단선 반이중 TTL 버스 회로/어댑터와 수신 전압을 확인한다. 제조사 ST3215-HS 전원 범위는 6–12.6 V이며 MCU의 3.3 V/USB 전원으로 구동하지 않는다. 지원 6S 배터리도 서보나 DevKit의 5 V 핀에 직접 넣지 않는다.
3. **전원:** MCU·IMU용 안정된 전원과 서보/모터의 전력 경로를 설계한다. USB와 외부 5 V의 역급전 방지, 공통 기준 접지 또는 절연, 배터리 ADC의 최대 충전 전압·과도전압·분압 저항/ADC 입력 범위를 확인한다. 배터리 수치 측정에는 ADC1을 배정했다.
4. **부팅/정지:** 비-strapping GPIO라도 부팅 중 출력 상태를 안전하다고 가정하지 않는다. 외부 구동 허가 기본 OFF, 입력 바이어스, MCU가 멎었을 때의 차단, ESC의 무신호·제동 동작을 설계·시험한다. GPIO8은 정지 상태를 읽는 핀이지 독립 차단 회로 자체가 아니다. CLEAR로 정지 고정을 풀어도 DISARMED에 머물고 별도 ARM이 필요하다.
5. **실시간 처리:** PCNT로 두 A/B 엔코더, SPI2로 IMU, 하드웨어 PWM으로 ESC, UART로 서보를 나누는 배치는 가능하다. 실제 최대 엔코더 에지율, 카운터 오버플로, 입력 필터, 제어/통신 지터와 ESP-NOW 부하는 실물 펌웨어에서 확인한다. ESP-NOW 자체에는 추가 외부 GPIO가 필요하지 않지만 휴대 송신기는 별도 장치다.
6. **판매품 대조:** 이 표는 공식 DevKitC-1-N8R8 기준이다. 구매 페이지의 실제 제조사·모듈 각인·보드 판본·헤더 핀맵·USB/전원 회로가 일치하는지 확인한다. 호환 보드의 판매명만으로 동일 배선을 확정하지 않는다.

## 대조 결과와 근거

문서의 GPIO 집합을 직접 계산해 기본 19개에 중복이 없고, 모두 헤더에 있으며, 제외 13개와 겹치지 않는 것을 확인했다. 대안별 핀 개수도 같은 후보 집합으로 계산했다. 보드 없이 수행한 문서/집합 대조이며 펌웨어 시험이나 전기적 안전 검증은 아니다. 실행 코드·차량 모델·기존 검증 결과는 변경하지 않았다.

공식 자료 확인일: 2026-09-30. ST3215-HS 위키 본문 열기는 접근 제한이 있었으며 동일 제조사 검색 색인의 사양·직렬 버스 설명을 확인했다. 배선 확정 시 실제 제공품 매뉴얼/어댑터 회로도를 다시 대조한다.

- [Espressif DevKitC-1 v1.1: 헤더·메모리 점유·전원·판본 차이](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html)
- [ESP32-S3 GPIO: strapping·USB·메모리·GPIO matrix](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html)
- [SPI Master: SPI2 IO_MUX와 일반 SPI](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/spi_master.html)
- [PCNT: 쿼드러처 디코딩](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/pcnt.html)
- [UART](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/uart.html), [MCPWM](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/mcpwm.html), [TWAI와 외부 트랜시버](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/twai.html)
- [Adafruit LSM6DSOX 핀과 전원](https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/pinouts)
- [Waveshare ST3215-HS 직렬 버스/전원 사양](https://www.waveshare.com/wiki/ST3215-HS_Servo_Motor)
- [Espressif 하드웨어 FAQ](https://docs.espressif.com/projects/esp-faq/en/latest/hardware-related/hardware-design.html)
