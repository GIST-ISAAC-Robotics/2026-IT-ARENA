# ESP32 주변 통합 배선 설계 v1

작성: 2026-11-11. 상태: **모듈 간 결선 설계 완료, 현재 IMU 외의 실물 결선·구동 시험은 미완료**.

기존에는 [승인 전장 구성](APPROVED_ELECTRICAL_LAYOUT.md)의 전원 블록도와 GPIO 후보표가 있었다. 이번에는 이를 실제 조립에 쓸 단자 대 단자 연결도로 구체화했다. 현재 IMU 배선을 유지하며 모터 드라이버·모터축 엔코더·조향 서보 어댑터를 추가한다. 새로운 버퍼·저항 네트워크·전원 모듈을 추가하지 않는다.

- **[배선도 전체 보기 — HTML](schematics/esp32_gpio_wiring_v1.html)**: 그림 두 장, 핀표, 출처를 포함한다. 외부 서비스 없이 열 수 있다.
- **신호 도면:** [SVG](schematics/esp32_gpio_signals_v1.svg) · [PNG](schematics/esp32_gpio_signals_v1.png).
- **전원·모터·서보 도면:** [SVG](schematics/esp32_gpio_power_v1.svg) · [PNG](schematics/esp32_gpio_power_v1.png).
- **[GPIO 연결표](schematics/esp32_gpio_pin_table_v1.md)** · [기계 판독 핀/전원 목록 JSON](schematics/esp32_gpio_v1.json).

SVG는 확대해도 선명한 원본이다. 도면은 **모듈 간 배선도**이며 모듈 내부 소자 회로·PCB 패턴·케이블 제작 치수·완성된 독립 STOP 회로를 나타내지 않는다. GPIO 번호는 헤더의 물리 순번이 아니다.

## 1. 기존 IMU 배선을 유지하는 이유

[11/11 실측](../sensors/ESP32_LSM6DSOX_HARDWARE_2026_11_11.md)에서 ESP32-S3·Flash 16 MB/PSRAM 8 MB와 LSM6DSOX SPI 수신을 확인했다. 현재 GPIO9–13은 엔코더4/5, 모터15/6, 서보17/18과 충돌하지 않는다. 따라서 IMU를 옮길 이유가 없다.

ESP32는 처음부터 호환 보드로 구매했으며 제조사/PCB 판본은 미확인이다. YD/VCC-GND 자료의 P1/P2 순번이나 USB 포트 좌우 위치를 이 보드에 적용하지 않는다. 실측 USB 브리지는 WCH VID/PID `1a86:55d3`, UART0 GPIO43/44 경로지만 정확한 브리지 칩 형번은 미확정이다. **보드의 GPIO 실크와 현재 통신된 USB 포트**를 기준으로 연결한다.

외부 장치에 사용하는 GPIO는 총 **11개**다. UART0 GPIO43/44는 보드 내부 USB-UART 전용으로 별도 예약한다. 현재 IMU 펌웨어는 이 문서의 모터·서보·PCNT 드라이버를 구현하지 않는다.

## 2. IMU — 현재 연결 그대로

| ESP32 | Adafruit 4438 | 역할 |
|---|---|---|
| **3V3** | **VIN** | IMU 전원 |
| GND 또는 G | GND | 공통 접지 |
| GPIO12 | SCL | SPI 클록 |
| GPIO11 | SDA | ESP32 → IMU |
| GPIO13 | 하단 DO | IMU → ESP32 |
| GPIO10 | 하단 CS | 센서 선택, LOW 유효 |
| GPIO9 | I1 | INT1 알림 입력, 동작 검증은 후속 |

핀 이름은 사용자 확인 실크를 따른다. **I1은 대문자 I와 숫자 1**이며 `11`처럼 보일 수 있다. 상단 `SCX / SDX / CS / DO / GND`, 하단 `3Vo / I2`는 이번 결선에서 사용하지 않는다. `3Vo`는 레귤레이터 출력이므로 VIN 대신 전원을 넣지 않는다. SPI는 실측했지만 I1 인터럽트 검증까지 통과한 것은 아니다. [Adafruit 핀 설명](https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/pinouts).

## 3. 모터 드라이버 — MD20A와 MB4266-24180

| 출발 단자 | 도착 단자 | 역할 |
|---|---|---|
| ESP32 GPIO15 | MD20A PWM | 속도 제어 PWM |
| ESP32 GPIO6 | MD20A DIR | 방향 제어 |
| ESP32 GND | MD20A 신호 GND | 신호 기준 |
| M01 CH1 + | MD20A VB+ | 모터 전원 입력 |
| M01 CH1 − | MD20A VB− | 전력 귀환 |
| MD20A MA | 모터 단자 6 (+) | 최초 시험용 모터 극성 |
| MD20A MB | 모터 단자 5 (−) | 최초 시험용 모터 극성 |

MD20A는 3.3 V 입력을 지원하므로 PWM/DIR에 CF-0108을 넣지 않는다. PWM 주파수는 제조사 표준 범위인 **20 kHz 이하에서 후속 펌웨어가 선택**하며 아직 운영값을 확정하지 않았다. 일반 RC 서보용 50 Hz 펄스 폭 명령과 구분한다. [Cytron 사양](https://www.cytron.io/p-20amp-6v-30v-dc-motor-driver).

제조사 진리표상 PWM LOW는 두 출력 LOW인 **brake**다. PWM HIGH/DIR LOW일 때 MA HIGH·MB LOW이며 DIR HIGH일 때 반대다. MA/MB는 양방향 H-bridge 출력이라 둘 중 어느 것도 고정 GND가 아니다. 위 모터 단자 5/6은 보존된 제조사 모터/엔코더 자료의 번호이며, 보유 케이블 색이나 단자 방향을 보지 않고 결정하지 않는다. MA→6/MB→5가 차량 전진이라는 뜻도 아니다. 바퀴를 띄운 시험에서 전진 명령과 엔코더 부호를 함께 확정한다. [공식 연결 데이터시트 §4](https://docs.google.com/document/d/1eKpt66z-1qLMJkaXrHU1AfKqSVeIpEnM43vECpkYuWA/edit).

**부팅·리셋 구간 PWM LOW는 아직 실측하지 않았다.** GPIO 초기화에서 LOW를 설정하더라도 그 이전 ROM/리셋 구간을 보장하지 않는다. 현재 진단 펌웨어를 사용한 채 모터 전원을 켜서 정지 동작을 기대하지 않는다. USB 포트 재개방 때 실제 MCU 리셋이 관측됐으므로, 후속 구동 펌웨어는 새 boot·기존 명령 무효·재허가 계약과 실제 PWM 파형을 확인해야 한다. 별도 EN 핀이 없는 MD20A에 GPIO21을 임의 연결하지 않는다.

## 4. 조향 — Bus Servo Adapter (A)와 ST3215-HS 1개

| 출발 단자 | 도착 단자 | 역할 |
|---|---|---|
| ESP32 GPIO17, UART1 TX | Adapter A H2 **TX / TXD** | 명령 송신 |
| ESP32 GPIO18, UART1 RX | Adapter A H2 **RX / RXD** | 응답 수신 |
| ESP32 GND | Adapter A H2 GND | 신호 기준 |
| M01 CH5 **또는** CH6 + | Adapter A 전원 + | 12 V, 한 채널만 사용 |
| 같은 채널 − | Adapter A 전원 − | 서보 전력 귀환 |
| Adapter A 서보 포트 D/DATA | ST3215-HS D/DATA | 단선 반이중 데이터 |
| Adapter A 서보 포트 V+ | ST3215-HS V+ | 서보 전원 |
| Adapter A 서보 포트 G/GND | ST3215-HS G/GND | 서보 접지 |

**점퍼는 A 위치**, UART 헤더는 **TX→TX / RX→RX**다. 일반 UART 장치의 교차 연결 습관을 적용하면 이 제품에서는 잘못 연결할 수 있다. 이는 [Waveshare 공식 배선 안내](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example)의 명시 사항이다. H2의 실제 핀 순서와 D/V/G 케이블 방향은 보드 실크로 확인하며 좌우 순번이나 선 색은 추측하지 않는다.

[제조사 회로도](https://files.waveshare.com/wiki/Bus_Servo_Adapter_A/Bus_Servo_Adapter_A.pdf)의 UART/버스 변환부는 3.3 V 논리이며 자동 방향 전환을 사용한다. 별도 방향 GPIO16이나 ESP32 3V3/5V 전원선을 어댑터 H2에 연결하지 않는다. GPIO17/18을 서보의 D 한 선에 직접 묶지 않는다. 어댑터 USB는 이번 구성에서 비워 둔다.

전원은 DC 5.5×2.1 mm 잭(중앙 +) 또는 해당 전원 단자 중 하나를 사용한다. **M01 12 V/2 A 한 채널**, 서보 한 개라는 기존 구성을 유지한다. CH5/CH6을 병렬로 연결하지 않는다. 실제 조향 전류·전압 강하, ID/baud, 링크 중립·끝단은 후속 시험 대상이다. 직렬 서보이므로 ESP32의 PWM 서보 출력 핀을 추가하지 않는다.

## 5. 엔코더 — CF-0108 확인 후 연결하는 조건부 부분

PGE-213은 모터축 엔코더 하나이며 HA/HB 두 신호를 읽는다. 아래 A1/B1 등은 **TXS0108E가 맞을 때의 IC 논리 이름**이다. CF-0108 판매명만으로 실장 IC·패드 이름·방향을 확정하지 않는다.

| 출발 단자 | 도착 단자 | 조건/역할 |
|---|---|---|
| J01 CH4 +, 5 V/0.5 A | PGE-213 XHP-4 pin 1 VCC | 엔코더 전원 |
| J01 CH4 − | PGE-213 pin 2 GND | 엔코더 전원 귀환 |
| 같은 J01 CH4 +5 V | CF VCCB/HV | 고전압 측 전원, 실물 확인 후 |
| ESP32 3V3 | CF VCCA/LV | 저전압 측 전원, 실물 확인 후 |
| ESP32 GND 및 엔코더 GND | CF GND | 공통 신호 기준 |
| PGE-213 pin 3 HA | CF B1/HV1 | A상 5 V 측 |
| CF A1/LV1 | ESP32 GPIO4 | A상 3.3 V 측, PCNT |
| PGE-213 pin 4 HB | CF B2/HV2 | B상 5 V 측 |
| CF A2/LV2 | ESP32 GPIO5 | B상 3.3 V 측, PCNT |
| ESP32 3V3 | CF OE | 아래 조건 충족 시 HIGH 유지 안 |

TXS0108E가 확인되면 VCCA=3.3 V, VCCB=5 V로 사용한다. **OE는 VCCA 기준**이며 5 V에 연결하지 않는다. 현재 도면은 모듈의 OE 배선/내장 저항을 확인한 뒤 3.3 V HIGH로 유지하는 구상이다. 이미 모듈에서 처리한 OE에 중복 배선하거나 부유 상태로 두지 않는다. 이는 3.3 V가 살아 있는 동안 enable되는 구성으로, 독립적인 전원 상승/하강 Hi-Z 제어를 제공하지 않는다. 해당 기능이 필요하면 실제 모듈 회로에 맞춰 재설계하며 이번에 추가 저항 회로를 만들어 넣지 않았다. [TI TXS0108E 데이터시트](https://www.ti.com/lit/ds/symlink/txs0108e.pdf).

**실물 확인 전 HA/HB→ESP32 직접 연결도, CF를 통한 시험 통전도 보류한다.** [기존 제조사 자료 판독](APPROVED_ELECTRICAL_LAYOUT.md#4-pge-213-엔코더와-cf-0108)에 HA/HB의 VCC 방향 4.7 kΩ pull-up과 출력당 10 nF가 있어, open-collector라고 해도 3.3 V 직접 입력 가능하다고 단정할 수 없다. TXS 계열 확인만으로 그 부하에서 정상 파형을 보장할 수도 없다. 실제 HIGH 전압·상승/하강·A/B 위상·최대 펄스율을 확인하고, TXB가 납품됐다면 이 배선안을 그대로 적용하지 않는다.

7 PPR은 제조사 표기다. PCNT x4 해석이면 28 counts/rev 후보이나 실제 축 회전과 계수 시험 전 운영값으로 확정하지 않는다. J01 CH4 5 V를 ESP32 5V/VIN이나 Jetson J12와 연결하지 않는다. CF 미사용 채널은 미연결로 둔다.

## 6. 전원과 GND

그림 02는 [승인 전원 배치](APPROVED_ELECTRICAL_LAYOUT.md#2-전원-연결)를 따른다. 배터리→J01/M01 내부 분배와 보호부는 이 도면의 상세 범위 밖이며 기존 기준을 유지한다.

- ESP32 전원은 현재 확인한 Jetson USB 케이블 하나다. IMU는 ESP32 3V3에서 공급한다.
- 모터 전력은 M01 CH1→MD20A VB+/VB−→MA/MB→모터로 전달한다. 서보 전력은 별도 M01 12 V 채널→Adapter A→서보로 전달한다.
- **ESP32·IMU·CF·엔코더·MD20A Signal GND·Adapter A H2 GND는 공통 기준**이다. 해당 전원 출력 −와의 도통도 확인한다. 그림에서 여러 GND를 그린 것은 새로운 절연 전원을 뜻하지 않는다.
- 모터/서보의 전류가 ESP32 헤더 GND·센서 점퍼선·USB 실드만을 통해 귀환하게 하지 않는다. 전력 귀환은 각 부하에서 해당 전원 −로 직접 마련하고, 신호 GND는 기준 전위를 연결한다. 같은 GND라는 이유로 모든 전선을 MCU 핀 하나에 몰아넣지 않는다.
- 19 V, 모터 배터리 전압, 12 V, 엔코더 5 V, MCU 3.3 V의 **+ 레일은 서로 분리**한다. 전력선에 브레드보드/일반 Dupont 점퍼선을 사용하지 않으며 굵기·커넥터 정격·체결은 실제 부하와 기보유 케이블에 맞춰 확인한다.

M01 입력 전압 범위와 6S 운용 전압, Jetson 공급 한도, 서보 단일 채널 전류의 남은 실측 조건은 [전장 기준 §7](APPROVED_ELECTRICAL_LAYOUT.md#7-정격상-주의--승인된-배치를-변경하라는-뜻은-아님)에 보존한다. 이번 도면 작성은 이 정격 문제의 시험 통과를 뜻하지 않는다.

## 7. 예약 핀과 후속 제어 구현

| 구분 | GPIO | 현재 처리 |
|---|---|---|
| USB-UART UART0 | 43 / 44 | 보드 내부 호스트 통신, 외부 배선 금지 |
| STOP 상태 | 8 | 외부 회로 미정, 연결하지 않음 |
| 구동 허가 요청 / watchdog | 21 / 47 | 상대 회로 미정, 연결하지 않음 |
| 배터리 ADC | 1 | 분압·보호 미정, 배터리 직접 연결 금지 |
| 선택 고장 입력 / CLEAR 입력 | 2 / 14 | 예약만 유지 |
| 여유 후보 | 7 / 16 / 39 / 40 / 41 / 42 | 39–42는 외부 JTAG와 겸용 |

부팅 strap GPIO0/3/45/46, 메모리 GPIO26–37, native USB19/20은 새 장치에 배정하지 않았다. GPIO38/48은 제조사 미확인 보드의 LED 등 연결 가능성을 고려해 기존 예약을 유지한다. 칩 GPIO 제약과 실제 PCB 배선은 별개다. [Espressif GPIO 안내](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html).

STOP 상태 입력 하나는 독립 전력 차단 회로를 대신하지 않는다. 상대 회로가 없는 예약 핀에 임의로 모듈 단자를 연결하지 않는다. 구현 시 [기존 구동 계약](ACTUATION_CONTRACT_2026_09_22.md)과 `/drive`→독립 보호층→`/drive/safe` 경계를 유지하고, UART/SPI/PCNT/PWM 동시 동작 및 리셋/통신 단절을 실물로 확인한다.

## 8. 이번 검토와 다음 조립 순서

이번에는 문서·공식 자료와 현재 IMU 배선을 대조하고, 11개 GPIO 중복/제외/예약 핀 충돌 검사, 도면 렌더링·육안 확인, 링크·JSON·diff 검사를 수행했다. ESP32/Jetson에 접속하거나 펌웨어를 업로드하지 않았고 모터·서보에 전원을 넣지 않았다.

1. 현재 IMU 7가닥은 유지한다. 새 배선 작업 때는 USB와 각 전원을 분리한다.
2. ESP32 GPIO4/5/6/15/17/18 실크, MD20A 입력 라벨, Adapter A H2/점퍼/DVG와 전원 극성을 대조한다.
3. CF 앞뒤·IC·LV/HV·OE 처리를 확인해 조건부 부분을 확정한다. 핀4/5는 그전까지 비워 둔다.
4. 엔코더 입력, 서보 읽기/제한 동작, 모터 무부하 저 PWM 순으로 후속 펌웨어와 함께 시험한다. 모터/서보 연결만으로 현재 IMU 진단 코드가 제어를 시작하지는 않는다.
5. STOP 회로·모터/서보 전원 과도·전체 동시 동작은 별도 통합 확인으로 남긴다.

핀/전원 목록을 바꾸면 [생성 스크립트](../../scripts/render_esp32_wiring.py)를 `python scripts/render_esp32_wiring.py`로 실행해 SVG·HTML·핀표를 다시 만든다. PNG는 SVG를 브라우저에서 1700×1230 / 1700×1390으로 렌더링한 사본이다. 그림과 펌웨어가 자동 연동되는 구조는 아니므로 실제 드라이버 도입 때 이 배정과 다시 대조한다.
