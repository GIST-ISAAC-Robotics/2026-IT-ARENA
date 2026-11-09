# IT ARENA 실차 전장 배선·조립 기준 R2

- 기준일: 2026-11-10
- 상태: **배선 설계 기준 확정 / 실물 통전·부하·주행 검증 전**
- 적용: VND018 N16R8, Jetson Orin Nano Super, J01/M01 지급 PDB, 6S 배터리, MB4266-24180, PGE-213, CF-0108, MD20A, Adafruit 4438, Bus Servo Adapter (A), ST3215-HS 1개, RPLIDAR C1, Arducam B0390
- 상위 문서: [실물 Bring-up Runbook](PHYSICAL_BRINGUP_AND_CALIBRATION_RUNBOOK.md), [구매·케이블 대조](PROCUREMENT_AND_CABLES_2026_11_08.md)
- 개별 배선표: [R2 connector-to-connector netlist CSV](electrical_netlist_r2_2026_11_10.csv)

> 이 문서는 기존 Electrical R1 설계에서 실제 선정된 CF-0108을 엔코더 수신부에 통합한 R2다. R1의 전원 분기·퓨즈·2접점 STOP·서보 RX 보호 버퍼를 유지하되, 엔코더의 별도 U1(SN74LVC2G17) 대신 CF-0108 표준 TXS0108E 모듈을 사용한다. 서보 RX 보호용 U2(SN74LVC2G17)는 유지한다. 이전 Runbook의 엔코더 전원(Jetson J12 pin 2/6) 연결은 폐기하고, **J01 CH2 5 V**로 통일한다.

## 1. 확정 회로 선택

| 항목 | R2 최종 선택 | 연결/설계 이유 |
|---|---|---|
| MCU 전원·Jetson 통신 | Jetson USB-A → VND018 CH343P USB-UART USB-C 한 가닥 | USB 5 V로 ESP32 전원, UART0(GPIO43/44)로 데이터 통신 |
| IMU 전원 | VND018 3V3 → Adafruit 4438 VIN | 3.3 V 논리, MCU와 같은 기준 GND |
| 엔코더 전원 | J01 CH2 5 V → FE 0.25 A → PGE-213 pin 1 | VND018 5Vin/5V 헤더와 Jetson J12 5V 한도에 의존하지 않음 |
| 엔코더 신호 | PGE A/B 5 V → CF-0108(TXS0108E) B1/B2 → A1/A2 → 100 Ω → GPIO4/5 | 8채널 중 2채널 사용, 5→3.3 V 자동 방향 변환 |
| 서보 전원 | M01 CH5 12 V → FS 2 A → S_SERVO → Adapter A H1 | CH5만 사용, CH6 비움 |
| 서보 통신 | GPIO17 → Adapter H2-3 TXD, Adapter H2-2 RXD → U2 → GPIO18 | U2의 Ioff로 MCU OFF 때 RX 역급전 위험 완화 |
| 모터 전원 | M01 CH1 배터리 pass-through → FM → MD20A VB+/VB− | 12 V/2 A 채널은 모터에 사용하지 않음 |
| 모터 제어 | GPIO15 PWM, GPIO6 DIR | 각각 1 kΩ 직렬, 드라이버 입력에 10 kΩ pulldown |
| 하드웨어 STOP | 2개의 독립 NC 접점: PWM 직접 개방 + GPIO8 상태 감시 | MCU 펌웨어와 독립적인 PWM 억제 경로 |
| 영상·LiDAR | B0390 → Jetson CAM0 CSI, C1 → Jetson USB | ESP32 GPIO와 무관 |

### 운용 한계로 고정할 사항

1. M01의 공식 입력 표기는 24–36 V다. 첫 PDB 통전은 배터리 24.0–25.2 V에서 한다. 24 V 미만의 M01 동작은 제조사 확인 또는 별도 부하 검증 전 승인하지 않는다.
2. ST3215-HS stall 전류 2.4 A는 M01 CH5 2.0 A보다 높다. 이 회로는 단일 서보의 무부하·저부하 조향 시험용이다. 기계적 끝단 구속·stall 시험은 금지한다.
3. J01 CH2/CH3는 5 V 합계 1.5 A다. CH2를 엔코더 전용으로 쓰고 CH3는 비운다.
4. MD20A PWM LOW는 coast가 아니라 brake 동작이다. STOP 시 PWM 입력을 LOW로 만들도록 설계한다.
5. CF-0108은 내부 IC가 TXS0108E로 식별되는 경우에만 아래 핀표대로 사용한다. 판매명 CF-0108 또는 보드 실크 CJMCU-0108만으로 IC를 확정하지 않는다. TXB0108 등 다른 IC면 이 배선에 전원을 넣지 말고 부품을 재식별한다.

## 2. 전원 분배·퓨즈·커넥터

### 2.1 배터리에서 PDB까지

    6S 배터리 XT60(+)
      └─ F0 15 A main fuse
          └─ S0 main switch (정격 ≥32 VDC, ≥20 A)
              └─ XT30 Y 분기
                  ├─ FJ 5 A ─→ J01 INPUT (+)
                  └─────────→ M01 INPUT (+)

    6S 배터리 XT60(−)
      ├────────────────────→ J01 INPUT (−)
      └────────────────────→ M01 INPUT (−)

- 배터리에서 각 PDB 입력 음극으로 별도 귀환선을 둔다. 음극선을 먼저 끊거나 MCU GND/USB 실드로 대전류를 귀환시키지 않는다.
- F0/FJ/FM/FS/FE는 전류 제한기가 아니라 과전류 보호 퓨즈다. 퓨즈·홀더는 6S 완충 25.2 V보다 높은 DC 정격과 적절한 차단 용량을 사용한다.
- XT60/XT30의 암수는 극성을 보증하지 않는다. 각 하네스 양 끝의 +/−를 멀티미터로 확인한다.

### 2.2 PDB 출력 배정

| PDB 출력 | 정격 | 최종 사용 | 배선 |
|---|---:|---|---|
| J01 CH1 | 19 V / 2.3 A | Jetson DC 입력 | XT30 수 피그테일 → DC 5.5×2.5 mm, 중앙 + |
| J01 CH2 | 5 V, CH2/CH3 합계 1.5 A | PGE-213 엔코더 전원 | CH2 + → FE 0.25 A → PGE pin 1; CH2 − → PGE pin 2 |
| J01 CH3 | 5 V, CH2/CH3 합계 1.5 A | 미사용 | 출력 단자 절연 |
| J01 CH4 | 5 V / 0.5 A | 미사용 예비 | VND018 5Vin/엔코더에 연결하지 않음 |
| M01 CH1 | 입력 배터리 전압 pass-through | MD20A 전력 입력 | XT30 수 피그테일 → FM 퓨즈 → VB+/VB− |
| M01 CH2–4 | 입력 배터리 전압 pass-through | 미사용 예비 | 출력 단자 절연 |
| M01 CH5 | 12 V / 2 A | 단일 ST3215-HS 서보 전원 | CH5 + → FS 2 A → S_SERVO → Adapter H1-2 DC+; CH5 − → H1-1 GND |
| M01 CH6 | 12 V / 2 A | 미사용 | CH5와 병렬 연결 금지 |

**퓨즈 기준 BOM**

| 표기 | 값 | 위치·보호 대상 |
|---|---:|---|
| F0 | 15 A | 배터리 양극 바로 뒤 주 퓨즈 |
| FJ | 5 A | J01 입력 양극 분기 |
| FM | 10 A 임시 기준 | M01 CH1에서 MD20A VB+로 가는 모터 전력선 |
| FS | 2 A | M01 CH5 서보 전원선 |
| FE | 0.25 A | J01 CH2 엔코더 5 V 전원선 |

FM 10 A는 R1의 초기 보호값이다. 모터 정격전류 표기 10.28 A와 근접하므로 지속 부하에서 퓨즈가 끊어질 수 있다. 전류 파형과 배선·커넥터 허용전류를 확인하기 전 임의로 퓨즈를 키우지 않는다. 정격 운전에서 반복 단선되면 원인을 측정한 뒤 퓨즈 특성과 배선 정격을 함께 재선정한다.

### 2.3 전원·접지 원칙

- Jetson은 J01 CH1 19 V만 사용한다. 5.5×2.5 mm 잭 중앙이 +, 외곽이 −다.
- VND018은 CH343P USB-UART 포트 한 곳으로만 공급한다. USB 직결 포트와 USB-UART 포트에 외부 5 V를 동시에 연결하지 않는다.
- IMU VIN은 VND018 3V3에서 공급한다. Adafruit 보드의 3Vo는 사용하지 않는다.
- PGE-213은 J01 CH2 5 V로 공급한다. Jetson J12 pin 2/4는 이번 R2 엔코더 전원에 사용하지 않는다.
- CF-0108 VA는 MCU 3.3 V, VB는 엔코더 5 V다. GND는 공통이다. OE는 GPIO14로 제어하며, 전원 안정 전까지 10 kΩ pulldown이 LOW를 유지한다.
- MD20A VB−는 M01 CH1−로 직접 귀환한다. 모터 전류를 ESP32 GND나 USB 케이블로 흘리지 않는다.
- Servo Adapter H1-1은 M01 CH5−로 직접 귀환한다. H2 GND는 논리 기준으로 VND018 GND와 연결한다.
- 논리 GND는 VND018–CF-0108–PGE GND–IMU GND–U2–MD20A signal GND–Adapter H2 GND 사이에서 공통 기준을 만든다. 전력 귀환은 각 PDB 채널 음극으로 별도 배선한다.
- 무전원 상태에서 J01/M01 출력 음극과 논리 GND의 도통 관계를 확인하고 기록한다. PDB 내부의 공통 음극 여부를 추정하지 않는다.

## 3. CF-0108 엔코더 변환 회로

### 3.1 모듈 식별과 기능

사용자 선정품은 [CF-0108 쿠팡 상품 13919546](https://www.coupang.com/vp/products/13919546)이며, 같은 모델명의 판매 등록은 8채널 로직 레벨 변환 모듈로 표기한다. 표준 CJMCU-0108 모듈은 TI TXS0108E를 사용한다.

- TXS0108E A 포트: 1.4–3.6 V
- TXS0108E B 포트: 1.65–5.5 V
- VCCA는 VCCB 이하로 유지한다. OE는 VCCA 기준 입력이며 LOW에서 모든 I/O가 Hi-Z가 된다. TI 권고대로 OE를 GND에 10 kΩ으로 pulldown하고, VCCA/VCCB가 안정된 뒤 MCU GPIO14로 HIGH를 인가한다.
- 방향 제어: 자동 감지
- 지원: open-drain 및 push-pull
- 최대 데이터 속도: push-pull 110 Mbps, open-drain 1.2 Mbps
- OE Low: 모든 출력 Hi-Z, OE High: 활성
- TI 공식 package addendum의 DGS 패키지 part marking: XS0108 (다른 패키지는 marking이 다를 수 있음)

공식 자료: [TI TXS0108E 제품·데이터시트](https://www.ti.com/product/TXS0108E), [TXS0108E 데이터시트 Rev. L](https://www.ti.com/lit/ds/symlink/txs0108e.pdf), [CJMCU-0108 모듈 설명·핀 기능](https://www.voltnova.pl/produkt/txs0108e-8-kanalowy-konwerter-napiec/).

**납땜 전 식별:** CF-0108/ CJMCU-0108은 판매 보드명이지 IC part number가 아니다. IC top-mark와 판매자 자료를 대조해 TXS0108E인지 확인한다. TI DGS 패키지의 공식 marking은 XS0108이며 다른 패키지/호환 clone은 marking이 다를 수 있다. YF08E만으로 TI 정품이라고 단정하지 않는다. IC를 식별할 수 없으면 통전하지 말고, 확정된 대안은 R1의 SN74LVC2G17 2채널 수신 회로다.

### 3.2 모듈 핀 연결

표준 모듈의 각 핀 실크를 기준으로 연결한다. 보드의 물리적 좌우/상하 위치는 복제품마다 달라질 수 있으므로 위치가 아니라 VA/VB/GND/OE/A1…/B1… 실크를 읽는다.

| CF-0108 단자 | 연결 대상 | 설명 |
|---|---|---|
| VA | VND018 P1-1 또는 P1-2, 3V3 | 저전압 A측 전원 |
| VB | J01 CH2 5 V, FE 퓨즈 뒤 | 고전압 B측 전원 |
| GND | 공통 논리 GND | VA/VB 기준 |
| OE | VND018 GPIO14/P1-20, OE–GND 10 kΩ pulldown | 부팅/전원 상승 중 LOW(Hi-Z), VA/VB 안정 후 GPIO14 HIGH로 활성. 모듈에 OE pull-up이 실장됐다면 제거 |
| B1 | PGE-213 pin 3, A | 5 V 엔코더 A 입력 |
| A1 | 100 Ω 직렬 → VND018 GPIO4/P1-4 | 3.3 V A 출력 |
| B2 | PGE-213 pin 4, B | 5 V 엔코더 B 입력 |
| A2 | 100 Ω 직렬 → VND018 GPIO5/P1-5 | 3.3 V B 출력 |
| A3–A8/B3–B8 | 미연결·절연 | 이번 회로에서는 사용하지 않음 |

- PGE-213 XHP-4 기준 핀 배정은 1=VCC, 2=GND, 3=A, 4=B로 적용한다. 케이블 색상으로 판단하지 말고 핀 번호와 맨선의 도통을 확인한다.
- PGE VCC–GND 바로 옆에 100 nF와 1 µF(정격 16 V 이상)를 병렬로 둔다.
- TXS0108E 모듈의 전원 디커플링 부품이 실장되어 있는지 확인한다. 미실장 보드면 VA–GND와 VB–GND에 각각 100 nF를 추가한다.
- CF-0108은 5 V 신호를 12/24 V 입력으로 보호하는 장치가 아니다. PGE A/B 외의 12/24 V 신호를 넣지 않는다.
- 별도 100 kΩ pulldown을 A/B 라인에 추가하지 않는다. TXS 자동 방향 감지와 내부 pull-up 동작을 방해할 수 있다.
- 엔코더 A/B 선은 모터 전력선과 분리해 배치한다. 가능하면 A/GND, B/GND를 짝지어 배선하고 길이를 짧게 유지한다.

### 3.3 PGE-213 전원·신호

    J01 CH2 +5V ─ FE 0.25A ─┬─→ PGE-213 XHP-4 pin 1 (VCC)
                             └─→ CF-0108 VB

    J01 CH2 GND ─────────────┬─→ PGE-213 XHP-4 pin 2 (GND)
                             └─→ CF-0108 GND

    PGE-213 pin 3 (A) ─────────→ CF-0108 B1
    CF-0108 A1 ─ 100Ω ─────────→ VND018 GPIO4 / P1-4

    PGE-213 pin 4 (B) ─────────→ CF-0108 B2
    CF-0108 A2 ─ 100Ω ─────────→ VND018 GPIO5 / P1-5

    VND018 P1-1 (3V3) ─────────→ CF-0108 VA
    VND018 GPIO14/P1-20 ────────→ CF-0108 OE
    CF-0108 OE ─ 10kΩ ─────────→ GND (default disabled)

PGE-213 판매 사양은 5 V, 14–20 mA, 2채널 7 PPR이다. 실제 출력 파형은 실물에서 확인한다. TXS0108E는 open-drain과 push-pull 모두 지원하므로 출력단 유형이 확인되기 전에도 모듈 선정은 가능하지만, 전압·위상·펄스 폭 검사는 반드시 수행한다.

## 4. ESP32-S3 VND018 핀·헤더 배정

### 4.1 보드 방향과 P1/P2

안테나가 위, USB-C 두 개가 아래를 향하도록 놓는다. 사용자가 제공한 실물 사진에서 왼쪽 헤더가 P1, 오른쪽 헤더가 P2다. GPIO 번호는 물리 헤더 순번이 아니므로 P1/P2 번호와 GPIO 실크를 모두 확인한다.

| P1 번호 | 실크/GPIO | R2 연결 |
|---:|---|---|
| 1 또는 2 | 3V3 | IMU VIN, CF-0108 VA |
| 4 | GPIO4 | 엔코더 A |
| 5 | GPIO5 | 엔코더 B |
| 6 | GPIO6 | MD20A DIR |
| 8 | GPIO15 | MD20A PWM |
| 10 | GPIO17 | Servo Adapter TXD |
| 11 | GPIO18 | U2 출력 → MCU RX |
| 12 | GPIO8 | STOP 상태 입력 |
| 15 | GPIO9 | IMU INT1 |
| 16 | GPIO10 | IMU CS |
| 17 | GPIO11 | IMU MOSI/SDI |
| 18 | GPIO12 | IMU SCLK |
| 19 | GPIO13 | IMU MISO/SDO |
| 20 | GPIO14 | CF-0108 OE enable |
| 22 | GND | 논리 GND |

| P2 번호 | 실크/GPIO | R2 연결 |
|---:|---|---|
| 1 | GND | 보조 논리 GND |
| 2 | TX / GPIO43 | CH343P UART0 TX, Jetson 링크 |
| 3 | RX / GPIO44 | CH343P UART0 RX, Jetson 링크 |
| 18 | GPIO21 | 미사용, 예약 |
| 17 | GPIO47 | 미사용, 예약 |
| 4 | GPIO1 | 미사용, ADC 분압 회로 없음 |
| 5 | GPIO2 | 미사용, 외부 FAULT 회로 없음 |

미사용 GPIO1/2/14/21/47에는 외부선을 연결하지 않는다. 배터리 분압기·외부 watchdog·ENABLE·FAULT·CLEAR 기능은 이번 R2 조립 범위에서 제외한다. 펌웨어에서도 해당 기능이 구현된 것처럼 표시하지 않는다.

### 4.2 USB 포트

- CH343P USB-UART 포트: Jetson USB-A에 연결한다. 전원 5 V와 UART0 데이터가 한 케이블로 전달된다.
- ESP32-S3 Native USB 포트(GPIO19/20): 개발 PC의 펌웨어 업로드·디버깅용으로 남긴다. 주행 중에는 비워 둔다.
- USB 두 포트를 동시에 외부 5 V 전원으로 공급하지 않는다.
- VND018 P1-21의 5V/5Vin은 R2에서 외부 전원 출력으로 사용하지 않는다.
- 제조사 핀표: [YD-ESP32-S3 README](https://github.com/vcc-gnd/YD-ESP32-S3).

## 5. IMU Adafruit 4438 배선

| Adafruit 4438 단자 | VND018 연결 |
|---|---|
| VIN | P1-1 또는 P1-2 3V3 |
| GND | P1-22 GND |
| SCK/SCL | GPIO12 / P1-18 |
| SDI/MOSI | GPIO11 / P1-17 |
| SDO/MISO | GPIO13 / P1-19 |
| CS | GPIO10 / P1-16 |
| INT1 | GPIO9 / P1-15 |
| 3Vo | 미연결 |

- SPI2 사용, 초기 시험 클록 1 MHz.
- CS에는 10 kΩ 풀업을 3V3에 연결해 부팅 중 비선택 상태를 유지한다.
- Adafruit 보드의 VIN을 3.3 V로 공급한다. 5 V 공급이나 3Vo 출력 사용은 금지한다.
- 축 방향·보드 장착 회전·센서 단위·ODR·data-ready 극성을 펌웨어에서 확인한다.

## 6. 서보 전원·통신·RX 보호

### 6.1 전원

    M01 CH5 +12V ─ FS 2A ─ S_SERVO ─→ Bus Servo Adapter H1-2 (DC+)
    M01 CH5 GND ────────────────────→ Bus Servo Adapter H1-1 (GND)
    Adapter H3: pin 1 DATA / pin 2 SERVO V+ / pin 3 GND
                                          └────────→ ST3215-HS D/V/G

- M01 CH5만 사용하고 CH6은 비워 둔다.
- Adapter H1의 2P 나사 단자를 사용한다. DC 5.5×2.1 잭과 H1을 동시에 연결하지 않는다.
- Adapter USB-C는 비워 둔다. 통신은 H2 UART를 사용한다.
- Adapter의 서보 포트는 H3 하나만 사용한다. H5는 미사용.
- S_SERVO는 ≥32 VDC, ≥3 A 정격 스위치로 한다.
- FS 2 A 퓨즈는 보호 소자이지 정전류 제한기가 아니다. ST3215-HS stall 2.4 A보다 작으므로 stall/기계적 끝단 구속 금지. 정상 조향 피크전류가 2 A에 접근하거나 전압 강하·리셋·과열이 발생하면 중단한다.

### 6.2 UART와 U2 수신 버퍼

U2는 SN74LVC2G17DBVR(SOT-23-6) 한 개다. 엔코더용 CF-0108과 다른 부품이다.

| U2 핀 | 연결 |
|---:|---|
| 1 (1A) | Adapter H2-2 RXD ← 1 kΩ 직렬, 100 kΩ pulldown to GND |
| 2 (GND) | VND018 GND |
| 3 (2A) | GND |
| 4 (2Y) | 미연결 |
| 5 (VCC) | VND018 3V3 |
| 6 (1Y) | 100 Ω 직렬 → GPIO18/P1-11 |

추가 연결:
- GPIO17/P1-10 → 1 kΩ 직렬 → Adapter H2-3 TXD.
- Adapter H2-1 GND → VND018 논리 GND.
- U2 pin 5–pin 2 바로 옆에 100 nF 디커플링.
- Adapter 점퍼는 A 위치. USB-C는 연결하지 않는다.
- Waveshare 표기 규칙은 TX→TX, RX→RX다. H2 핀 번호를 임의로 교차 연결하지 않는다.
- U2는 MCU 전원 OFF 때 Adapter RXD에서 GPIO18로 역급전되는 위험을 줄이는 수신 버퍼다. 갈바닉 절연이나 서보 전원 보호 장치가 아니다.
- 서보 ID·baud를 먼저 읽는다. 초기 통신 후보는 1 Mbps이며 공장 설정과 일치하는지 확인한다.

근거: [Waveshare Adapter A 결선](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example), [Adapter A 회로도](https://files.waveshare.com/wiki/Bus_Servo_Adapter_A/Bus_Servo_Adapter_A.pdf), [ST3215-HS 사양](https://www.waveshare.com/wiki/ST3215-HS_Servo_Motor), [TI SN74LVC2G17](https://www.ti.com/lit/ds/symlink/sn74lvc2g17.pdf).

## 7. MD20A·모터·하드웨어 STOP

### 7.1 전력·신호

    M01 CH1 + (battery pass-through) ─ FM 10A ─→ MD20A VB+
    M01 CH1 − ───────────────────────────────→ MD20A VB−

    MD20A MA ────────────────────────────────→ MB4266 motor terminal 1
    MD20A MB ────────────────────────────────→ MB4266 motor terminal 2

    VND GPIO15/P1-8 ─ 1k ─ S_RUN(NO) ─ S_STOP-A(NC) ─→ MD20A PWM
    MD20A PWM ─ 10k pulldown ─ MD20A signal GND

    VND GPIO6/P1-6 ─ 1k ─────────────────────→ MD20A DIR
    MD20A DIR ─ 10k pulldown ─ MD20A signal GND

- MD20A는 6–30 V, 20 A 연속/60 A 피크, 3.3 V 논리 입력 지원, 역극성 보호 없음.
- PWM 초기 설정은 20 kHz 이하. 첫 시험은 모터를 공중에 띄우고 PWM=0으로 시작한다.
- PWM LOW는 brake다. 정지 상태를 coast라고 가정하지 않는다.
- MA와 MB는 H-bridge 출력이며 어느 쪽도 GND가 아니다. 접지형 오실로스코프 GND 클립을 MA/MB에 연결하지 않는다.
- 모터 전력선은 PGE-213 엔코더 PCB의 녹색 2P 단자에 연결하지 않는다. 모터 전류 정격을 확인하지 못했으므로 모터 탭에 직접 연결하고 절연·고정한다.
- 방향은 바퀴를 띄운 상태에서 저출력으로 확인하고, 필요하면 DIR 논리 또는 펌웨어 부호를 바꾼다.

### 7.2 2접점 하드웨어 STOP

필요 부품:
- 잠금식/회전 해제형 비상정지 버튼, 서로 독립적으로 동작하는 NC 접점 2개(2NC).
- S_RUN: 별도 수동 운전 허가 스위치, 기본 OPEN.
- STOP 상태 입력 풀업 10 kΩ, PWM/DIR pulldown 각 10 kΩ.

배선:
- STOP 접점 A(NC): GPIO15에서 MD20A PWM으로 가는 경로에 직렬 삽입. 버튼을 누르거나 배선이 끊어지면 PWM 경로가 열리고 MD20A PWM pulldown이 LOW를 만든다.
- STOP 접점 B(NC): GPIO8/P1-12와 GND 사이에 연결. GPIO8은 3V3에 10 kΩ 풀업한다. 정상 해제 상태는 LOW, 버튼 눌림/배선 단선은 HIGH로 읽힌다.
- S_RUN은 STOP 접점 A와 직렬인 별도 수동 허가 스위치다. 전원 투입·리셋 시 OPEN을 기본값으로 한다.
- 버튼을 GPIO8에만 연결하지 않는다. 그 경우 MCU 소프트웨어에 의존하므로 독립 하드웨어 STOP이 아니다.
- STOP 해제 후에도 S_RUN은 OPEN으로 유지한다. 펌웨어는 명시 CLEAR 후 별도 ARM 전까지 재출발을 허용하지 않는다.

이 회로는 MCU 펌웨어와 독립적으로 MD20A의 PWM 입력을 억제하지만 모터 배터리 전원을 직접 차단하는 회로는 아니다. MD20A 자체 고장까지 막는 인증 안전회로로 해석하지 않는다. 실제 정지 응답·정지거리·버튼 동작을 시험해야 한다.

## 8. Jetson 카메라·LiDAR 연결

### 8.1 B0390

- Arducam B0390은 IMX219, MIPI CSI-2, 기본 2-lane, 카메라 보드 커넥터 15-pin 1.0 mm다.
- Jetson Orin Nano CAM0/J20은 22-pin 0.5 mm CSI 커넥터다.
- B0390 패키지의 **15-pin ↔ 22-pin FPC 케이블**을 사용한다. 22-pin ↔ 22-pin 케이블은 이 연결에 사용하지 않는다.
- CAM0는 2-lane 입력으로 사용한다. FPC의 접점 방향은 카메라와 Jetson 커넥터 실크/접점으로 확인하고 잠금쇠를 연 뒤 삽입한다. 반대로 강제 삽입하지 않는다.
- CSI 전원 핀에 외부 3.3 V/5 V를 주입하지 않는다. CSI 직결로 인식·스트리밍·재부팅 반복 시험을 한다.
- Arducam 자료: [B0390 공식 사양·패키지](https://www.arducam.com/arducam-imx219-visible-light-fixed-focus-camera-module-raspberry-pi.html). Jetson 커넥터 형식: [Arducam Jetson CSI connector guide](https://docs.arducam.com/camera-for-jetson-nano/mipi-camera-modules-for-jetson-nano/external-trigger-camera-application-note/).

### 8.2 RPLIDAR C1

- C1은 Jetson USB-A에 데이터와 전원을 함께 연결한다.
- ESP32 CH343P와 다른 USB 스택의 포트에 연결해 VBUS 부하를 분산한다.
- USB 장치 ID/경로를 확인해 고정 udev 별칭을 만들고, C1 드라이버의 실제 scan timestamp·점 순서·모터 속도를 확인한다.

## 9. 실제 조립 전 추가 부속 BOM

아래는 R2 회로를 구성하는 부속이다. 기존 구매품에 포함되지 않은 것은 조립 전에 확보한다. 수량은 회로 기준이며 보유품 대조 후 중복 구매하지 않는다.

| 부품 | 수량 | 값/규격 | 용도 |
|---|---:|---|---|
| CF-0108 | 1 | CJMCU-0108 계열; IC part number를 top-mark/판매자료로 TXS0108E 확인 | 엔코더 A/B 5→3.3 V |
| SN74LVC2G17DBVR | 1 | SOT-23-6 | 서보 RX U2 |
| SOT-23-6 변환기판 | 1 | DBVR 패키지와 핀번호 대응 | U2 실장 |
| 100 nF 세라믹 | 2 | U2, PGE 전원 | 디커플링 |
| 1 µF 세라믹 | 1 | ≥16 V | PGE 전원 |
| 100 Ω 저항 | 3 | 1/8 W 이상 | CF A1/A2 출력 2개, U2 출력 1개 |
| 1 kΩ 저항 | 4 | 1/8 W 이상 | PWM, DIR, Servo TX, Servo RX 입력 |
| 10 kΩ 저항 | 5 | 1/8 W 이상 | PWM/DIR pulldown, STOP pullup, IMU CS pullup, CF OE pullup |
| 100 kΩ 저항 | 1 | 1/8 W 이상 | U2 input pulldown |
| F0/FJ/FM/FS/FE | 각 1 | 15 A / 5 A / 10 A / 2 A / 0.25 A | DC 퓨즈·홀더 |
| S0 | 1 | ≥32 VDC, ≥20 A | 주전원 스위치 |
| S_SERVO | 1 | ≥32 VDC, ≥3 A | 서보 전원 스위치 |
| S_RUN | 1 | 저전류 접점, 기본 OPEN | 수동 구동 허가 |
| S_STOP | 1 | 잠금식 2NC 독립 접점 | 하드웨어 STOP |
| XT30·XT60 하네스 | 필요 수량 | 극성·전류 정격 확인 | 배터리/PDB/부하 전원 |
| 전선 | 필요 수량 | 배터리·모터 16 AWG, PDB 분기 18 AWG, 서보 22 AWG, 센서/신호 24–26 AWG 권장 | 전력/신호 분리 |

FM 10 A는 초기 설계값이다. 모터 정격 전류와 지속 운전 조건에 대한 계측 후 퓨즈 특성을 최종 승인한다. 퓨즈를 키울 때는 배선·커넥터·PDB·MD20A 허용전류도 함께 확인한다.

## 10. 조립 순서와 통전 게이트

| 순서 | 작업 | 통과 조건 |
|---:|---|---|
| 1 | CF-0108 칩 표기·보드 실크 확인 | IC top-mark/판매자료로 TXS0108E 식별, VA/VB/A/B/OE 실크 확인 |
| 2 | U2·저항·커패시터 기판 조립 | IC 방향·핀1·납땜 브리지·도통 검사 |
| 3 | PDB/배터리 하네스·퓨즈·스위치 조립 | 극성·연속성·단락 검사, 퓨즈 값/위치 기록 |
| 4 | VND018 헤더·IMU·CF·엔코더 신호 배선 | P1/P2 번호와 GPIO 실크 일치, 공통 GND 확인 |
| 5 | 서보 Adapter H1/H2/H3 배선 | 2P 극성, H2 TXD/RXD/GND, H3 D/V/G 확인 |
| 6 | MD20A PWM/DIR·STOP 회로 배선 | STOP 누르면 PWM 경로 개방, PWM pulldown LOW, GPIO8 상태 변화 |
| 7 | Jetson USB/CSI/C1 연결 | CH343P·Native USB·C1 USB·CAM0 구분 |
| 8 | 전원 단독 통전 | 배터리 24.0–25.2 V, 출력별 전압·무부하 전류·발열 기록 |
| 9 | 엔코더 단독 시험 | PGE 5 V, CF VA/VB/OE, A/B 파형, 3.3 V 출력, PCNT count/rev |
| 10 | IMU 단독 시험 | WHO_AM_I, SPI 1 MHz, 축·단위·INT1 |
| 11 | 서보 단독 시험 | ID/baud, U2 RX, 낮은 부하 이동, 전류·전압강하 |
| 12 | MD20A/모터 시험 | 바퀴 띄움, PWM/DIR, 방향·정지·전류·온도 |
| 13 | 전체 통합 | Jetson–MCU 통신, 센서 timestamp, watchdog, STOP, 동시 부하 |
| 14 | 저속 실차 | 직진·원호·조향각·정지거리 기록 후 제한적 속도 상승 |

### 전원 측정 순서

1. 배터리/USB 모두 분리하고 모든 전원선의 +/−, 단락, GND 연속성을 검사한다.
2. 전류 제한 벤치 전원으로 PDB 입력을 24.0 V에서 먼저 확인한다. 24 V 미만에서 M01을 정격 내라고 가정하지 않는다.
3. J01 CH1=19 V, CH2=5 V, M01 CH5=12 V, M01 CH1=배터리 pass-through를 무부하로 측정한다.
4. 부하를 하나씩 추가한다. 엔코더·IMU·ESP32·Jetson·서보·모터를 한 번에 처음부터 모두 켜지 않는다.
5. 전류 측정기는 부하와 직렬로 연결한다. 멀티미터 전류 모드로 배터리 +/−를 직접 가로지르지 않는다.
6. 전원 레일의 최소 전압, 부하 전류, 재부팅, 퓨즈·커넥터·전선 온도를 기록한다.

## 11. ESP32 프로그램 이관 기준

### 11.1 주변장치 할당

| 기능 | 주변장치 | 핀 |
|---|---|---|
| Jetson host link | UART0 / CH343P | GPIO43 TX, GPIO44 RX |
| Servo bus | UART1 | GPIO17 TX, GPIO18 RX |
| IMU | SPI2 | SCLK12, MOSI11, MISO13, CS10, INT1 9 |
| Encoder | PCNT quadrature | A=GPIO4, B=GPIO5 |
| Motor | LEDC PWM + GPIO DIR | PWM15, DIR6 |
| STOP | GPIO input + external NC contact | GPIO8 |
| CF translator enable | GPIO output | GPIO14 / P1-20, OE LOW by default |

ESP-IDF v6.1을 기준으로 한다. 기존 portable core는 프로토콜·수신 상태기계 후보이며, firmware_bench는 출력 비활성 UART 시험 앱이다. 실제 PCNT/SPI/LEDC/Servo driver와 통합 제어 task를 이관해야 한다.

### 11.2 UART0와 ROM 부팅 로그

- CH343P는 UART0(GPIO43/44)에 고정 연결되어 있다. Jetson 주행 프로토콜은 이 포트를 사용한다.
- ESP-IDF 앱/부트로더 콘솔은 Native USB Serial/JTAG(GPIO19/20)로 이동한다. CH343P UART0에 로그와 바이너리 프레임을 섞지 않는다.
- ESP32-S3 ROM 메시지는 기본 설정에서 UART0와 USB Serial/JTAG 양쪽에 출력될 수 있다. eFuse를 성급히 굽지 않는다. 호스트 수신기는 임의의 시작 잡음을 폐기하고 IA+길이+CRC가 유효한 첫 프레임에서 동기화해야 한다.
- 포트 개방/보드 reset/DTR-RTS/ROM 로그/USB 단절 후 재연결을 시험한다. 부팅 잡음이 유효 프레임으로 오인되지 않고 새 ARM 전까지 PWM이 LOW인지 확인한다.
- Jetson–MCU 링크 초기 baud는 2 Mbps를 후보로 두고 실제 최대 프레임 크기·전송 주기·CRC 오류·지연을 측정해 고정한다. 115200 bps를 무조건 기본값으로 쓰지 않는다.
- Native USB Serial/JTAG는 개발·로그·복구용으로 남기고 주행 중 데이터 링크는 CH343P UART0로 유지한다.

### 11.3 소프트웨어 상태·안전 기본값

- 부팅/리셋 시 PWM=0, DIR 기본 상태, S_RUN OPEN, STOP 상태 확인 전 DISARMED.
- STOP 접점이 열리면 PWM 경로가 물리적으로 열리고 GPIO8이 상태를 보고한다.
- 통신 단절·Jetson 종료·MCU reset·센서 고장 후 자동 ARM 금지.
- STOP 해제는 CLEAR와 ARM을 대신하지 않는다.
- GPIO1 ADC, GPIO2 FAULT, GPIO21 ENABLE, GPIO47 watchdog은 R2에서 외부 회로가 없으므로 물리 연결하지 않고 미구현으로 표시한다. GPIO14는 CF-0108 OE 제어로 재배정했으며 CLEAR 입력으로 사용하지 않는다.

## 12. 설계 확정과 실물 검증을 구분

**R2에서 핀·전원·신호 경로로 결정한 사항:** TXS0108E가 확인된 CF-0108의 결선, OE GPIO14 제어, PGE 전원 J01 CH2, ESP32 P1/P2 핀, IMU SPI, U2 서보 RX 버퍼, Adapter H1/H2/H3, MD20A PWM/DIR, 2NC STOP, B0390 15-to-22 FPC, C1 USB.

**현장에서 측정해야 하는 사항(설계 미결이 아니라 수령품/환경 승인 시험):**
- CF-0108 실제 IC top-mark와 보드 실크가 표준 TXS0108E와 일치하는지.
- PGE-213 XHP-4 pin numbering과 보유 케이블 도통.
- VND018 납품 PCB 리비전과 P1/P2 헤더 연속성.
- M01 24 V 미만 운용 가능 여부는 별도 제조사 확인/부하 시험 전 승인하지 않음.
- 서보 정상 조향 전류가 M01 CH5 2 A 공급 조건 안에 드는지.
- B0390 15-to-22 FPC 접점 방향과 CAM0 인식/스트리밍.
- STOP 누름·단선·MCU 멈춤 시 MD20A PWM LOW/실제 정지 응답.
- 실제 엔코더 count/rev, 감속비, 유효 바퀴 둘레, IMU 축 방향.

위 시험 전에는 실차 주행 승인으로 표시하지 않는다. 미확인 값을 임의로 정상이라고 채우지 않는다.
