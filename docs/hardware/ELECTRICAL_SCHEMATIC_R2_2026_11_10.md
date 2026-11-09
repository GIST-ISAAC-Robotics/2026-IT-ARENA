# IT ARENA 실차 전체 회로도·핀별 배선 기준 R2

- 작성 기준: 2026-11-10
- 기준 자료: 팀 보유 assembly_guide.pdf Electrical R1, 공식 J01/M01 Preliminary 사양, 팀 구매·케이블 대조, VND018 실물 앞/뒷면 사진, 제조사 자료
- 대상: 6S VEGA 배터리, J01/M01 PDB, Jetson Orin Nano Super, VND018 ESP32-S3-WROOM-1 N16R8, RPLIDAR C1, Arducam B0390, Adafruit 4438 LSM6DSOX, PGE-213, CF-0108, MD20A, MB4266-24180, Bus Servo Adapter (A), ST3215-HS 1개
- 상태: **배선 설계 기준 문서화 완료. 실제 납땜·통전·전류·파형·주행 검증은 별도 실행 필요.**
- 관련 절차: [실물 이관·검증 절차 R2](PHYSICAL_HANDOVER_R2_2026_11_10.md)
- 연결표: [핀-투-핀 Netlist CSV](electrical_netlist_r2_2026_11_10.csv)

> 이 문서는 보드 내부 회로를 새로 설계하는 문서가 아니라, 지급/구매 보드 사이의 전원·신호 배선과 추가 인터페이스 회로(U2, CF-0108, STOP)를 정의한다. PDB/Jetson/센서/서보 어댑터의 내부를 임의로 개조하지 않는다. 실제 기판의 실크와 제조사 문서가 다르면 이 문서의 위치 추정으로 연결하지 않는다.

## 1. 최종 시스템 블록 회로도

### 1.1 배터리와 PDB 전원 분기

~~~mermaid
flowchart TD
  BAT["6S 배터리 XT60"] --> F0["F0 15 A 주 퓨즈"]
  F0 --> S0["S0 MAIN 스위치<br/>≥32 VDC, ≥20 A"]
  S0 --> Y["XT30 Y 분기"]
  Y --> FJ["FJ 5 A"]
  FJ --> J01["J01 INPUT"]
  Y --> M01["M01 INPUT"]
  BAT --- NEG["배터리 음극"]
  NEG --> JN["J01 INPUT −"]
  NEG --> MN["M01 INPUT −"]
  J01 --> JCH1["CH1 19 V / 2.3 A"]
  J01 --> JCH2["CH2 5 V"]
  J01 --> JCH3["CH3 5 V"]
  J01 --> JCH4["CH4 5 V / 0.5 A"]
  J01 --> JCH56["CH5/6 3.3 V 합계 0.6 A"]
  M01 --> MCH1["CH1~4 배터리 전압 pass-through"]
  M01 --> MCH56["CH5/6 각 12 V / 2 A"]
  JCH1 --> JET["Jetson DC 19 V"]
  JCH2 --> FE["FE 0.25 A"]
  FE --> ENC["PGE-213 5 V"]
  FE --> CFVB["CF-0108 VB 5 V"]
  MCH1 --> FM["FM 10 A"]
  FM --> MD["MD20A VB+/VB−"]
  MCH56 --> FS["FS 2 A + S_SERVO"]
  FS --> SERVO["Servo Adapter A H1"]
~~~

**PDB 출력 고정 배정**

| PDB | 채널 | 정격 | 연결 | 미사용 |
|---|---|---:|---|---|
| J01 | CH1 | 19 V / 2.3 A | Jetson DC 잭 | |
| J01 | CH2 | 5 V, CH2+CH3 합계 1.5 A | FE 0.25 A → PGE-213 VCC 및 CF-0108 VB | |
| J01 | CH3 | 5 V, CH2+CH3 합계 1.5 A | | 예비 |
| J01 | CH4 | 5 V / 0.5 A | | 예비 |
| J01 | CH5/CH6 | 3.3 V, 합계 0.6 A | | 예비 |
| M01 | CH1 | 배터리 전압 pass-through | FM 10 A → MD20A VB+/VB− | |
| M01 | CH2~CH4 | 배터리 전압 pass-through | | 예비 |
| M01 | CH5 | 12 V / 2 A | FS 2 A + S_SERVO → Adapter A H1 | |
| M01 | CH6 | 12 V / 2 A | | 예비. CH5와 병렬 금지 |

J01 CH2와 CH3는 5 V 출력 합계 1.5 A다. CH2에 엔코더 약 14–20 mA만 연결하고 CH3는 비워 둔다. M01 CH5/CH6을 병렬 연결하지 않는다.

### 1.2 Jetson·MCU·센서 전체 구조

~~~mermaid
flowchart LR
  subgraph POWER["전원"]
    J01CH1["J01 CH1<br/>19 V"] --> JET["Jetson Orin Nano"]
    J01CH2["J01 CH2<br/>5 V + FE"] --> PGE["PGE-213 VCC"]
    J01CH2 --> CFVB["CF-0108 VB"]
    VND3["VND018 3V3"] --> IMUV["Adafruit 4438 VIN"]
    VND3 --> CFVA["CF-0108 VA"]
    M01CH1["M01 CH1<br/>Battery pass-through"] --> MD20["MD20A"]
    M01CH5["M01 CH5<br/>12 V"] --> ADP["Bus Servo Adapter A"]
  end
  subgraph HOST["Jetson"]
    JET -->|USB-A data + 5 V| CH343["VND018 CH343P USB-UART USB-C"]
    JET -->|USB-A 다른 스택| C1["RPLIDAR C1 USB adapter"]
    JET -->|CAM0/J20 CSI 22-pin| CAM["B0390 IMX219 15-pin"]
  end
  subgraph MCU["VND018 ESP32-S3"]
    CH343 <-->|UART1 GPIO43/44| JET
    IMUV <-->|SPI2 GPIO12/11/13/10 + INT1 GPIO9| VND["ESP32-S3"]
    PGE --> CF["CF-0108"]
    CF -->|A1/A2 → GPIO4/5| VND
    VND -->|GPIO15 PWM / GPIO6 DIR| MD20
    VND <-->|UART2 GPIO17/18 via U2| ADP
    VND <-->|GPIO8 STOP status| STOP["2NC STOP"]
    VND -->|GPIO14 OE enable| CF
  end
  ADP -->|D/V/G| SERVO["ST3215-HS ×1"]
  MD20 --> MOTOR["MB4266-24180"]
~~~

### 1.3 Jetson USB/CSI 연결

- Jetson USB-A 스택 1: VND018의 **CH343P USB-to-UART USB-C**에 연결한다. 이 한 가닥이 ESP32 전원(5 V)과 UART 통신을 함께 제공한다.
- Jetson USB-A 스택 2: RPLIDAR C1 어댑터를 연결한다. ESP32와 C1을 서로 다른 USB 스택에 배치한다.
- VND018의 Native USB-C(GPIO19/20)는 개발 PC 펌웨어 업로드/복구용으로 남긴다. 주행 중 CH343P 포트와 동시에 외부 5 V를 공급하지 않는다.
- Jetson CAM0/J20(22-pin, 0.5 mm)에 B0390의 15-pin 카메라 커넥터를 **15-pin↔22-pin FPC**로 연결한다. CAM0는 2-lane 입력이다. Jetson 커넥터 접점은 carrier PCB 쪽을 향하도록 넣고 잠금쇠를 닫는다. 카메라 쪽 커넥터 접점 방향은 B0390 실물과 케이블을 대조한다.
- B0390의 15-pin FFC 핀을 개별 점퍼선으로 연결하지 않는다. 제조된 FPC를 사용한다. CAM0에서 인식이 안 된다고 케이블을 뒤집거나 CSI 전원핀에 외부 전압을 주입하지 않는다.

## 2. 배터리 입력·퓨즈·스위치 회로

### 2.1 배터리 입력

~~~text
배터리 XT60 (+)
  └── F0 15 A (배터리 양극 가까이)
        └── S0 MAIN (≥32 VDC, ≥20 A)
              └── XT30 Y 분기
                    ├── FJ 5 A ── J01 INPUT (+)
                    └─────────── M01 INPUT (+)

배터리 XT60 (−)
  ├──────────────────────────── J01 INPUT (−)
  └──────────────────────────── M01 INPUT (−)
~~~

팀 보유 케이블 확인 기준:
- 배터리 단자: XT60 암.
- 보유 변환선: XT60 수 ↔ XT30 수.
- 보유 Y선: XT30 암 1개 → XT30 수 2개.
- 실제 커넥터 극성과 전선 굵기는 외형/색상만 믿지 말고 도통·전압으로 확인한다.

### 2.2 퓨즈·스위치 BOM

| 기호 | 값/정격 | 위치 | 용도/주의 |
|---|---:|---|---|
| F0 | 15 A, ≥32 VDC | 배터리 양극 직후 | 주 퓨즈 |
| FJ | 5 A, ≥32 VDC | J01 양극 가지 | J01 입력 가지 보호 |
| FM | 10 A, ≥32 VDC | M01 CH1+ → MD20A VB+ | 초기 시운전 제안값. 모터 정격 10.28 A와 근접하므로 반복 단선 시 원인 측정 후 재선정 |
| FS | 2 A, ≥32 VDC | M01 CH5+ → Adapter A H1-2 | 서보 전원 가지 보호. 전류 제한기가 아님 |
| FE | 0.25 A, ≥32 VDC | J01 CH2+ → PGE/CF 분기 | 엔코더 전원 가지 보호 |
| S0 | ≥32 VDC, ≥20 A DC 개폐 | 배터리 주전원 | AC 정격만 있는 스위치 금지 |
| S_SERVO | ≥32 VDC, ≥3 A DC 개폐 | Adapter A H1 전원 | 서보 전원만 차단 |
| S_RUN | 저전류 접점, NO | MD20A PWM 직렬 경로 | 기본 OPEN, 수동 구동 허가 |
| S_STOP | 잠금식 2NC 접점 | PWM 경로 + GPIO8 감시 | 두 NC 접점은 독립 접점이어야 함 |

F0/FJ/FM/FS/FE는 전류 제한기가 아니다. 퓨즈의 DC 전압 정격·차단용량·홀더 정격도 확인한다. FM 10 A는 지속 모터 운전에서 끊어질 수 있는 초기 보호 제안값이다. 원인 확인 없이 퓨즈를 키우거나 철사로 대체하지 않는다.

### 2.3 PDB 전압 운용 제한

- J01 입력: 21–36 V. CH1 19 V/2.3 A, CH2/CH3 5 V 합계 1.5 A, CH4 5 V/0.5 A, CH5/6 3.3 V 합계 0.6 A.
- M01 입력: 24–36 V. CH1~4는 입력 배터리 전압 pass-through, CH5/6은 각각 12 V/2 A.
- 지급 배터리 6S: 만충 25.2 V, 공칭 22.2 V, 운용 방전 구간에서 24 V 아래로 내려갈 수 있다.
- R1 시운전 기준으로 첫 PDB 통전은 24.0–25.2 V에서 한다. M01 24 V 미만 입력에서 12 V buck이 정상 유지된다고 제조사 확인을 받은 상태가 아니다. 주행 전 팩 전압별 CH5 12 V 출력과 발열을 확인한다.
- Jetson J01 CH1 19 V/2.3 A는 43.7 W 공급 한도다. Jetson 동봉 19 V/2.37 A 어댑터와 같은 급이다. 이 수치를 Jetson의 상시 소비전류로 해석하지 않는다. 기존 J01을 사용하고 전체 부하 중 전압 강하·재부팅·throttling을 관찰한다.

## 3. VND018 ESP32-S3 핀맵

### 3.1 실물 방향과 P1/P2

마스터가 제공한 VND018 실물 사진 기준: ESP32 안테나가 위, USB-C 두 개가 아래를 향할 때 왼쪽 헤더=P1, 오른쪽 헤더=P2다. 제조사 YD-ESP32-S3 README의 P1/P2 번호와 기능을 사용한다. 보드 판본이 다르면 실크와 도통을 우선한다.

**P1 핀 배정**

| P1 번호 | 실크 | ESP32 GPIO | 연결 |
|---:|---|---:|---|
| 1 또는 2 | 3V3 | — | IMU VIN, CF-0108 VA |
| 4 | 4 | GPIO4 | 엔코더 A 수신 |
| 5 | 5 | GPIO5 | 엔코더 B 수신 |
| 6 | 6 | GPIO6 | MD20A DIR |
| 8 | 15 | GPIO15 | MD20A PWM |
| 10 | 17 | GPIO17 | Servo Adapter H2-3 TXD |
| 11 | 18 | GPIO18 | U2 출력 → MCU RX |
| 12 | 8 | GPIO8 | STOP 상태 입력 |
| 15 | 9 | GPIO9 | IMU INT1 |
| 16 | 10 | GPIO10 | IMU CS |
| 17 | 11 | GPIO11 | IMU MOSI/SDI |
| 18 | 12 | GPIO12 | IMU SCLK |
| 19 | 13 | GPIO13 | IMU MISO/SDO |
| 20 | 14 | GPIO14 | CF-0108 OE 제어 |
| 22 | GND | — | 논리 GND |

**P2 핀 배정**

| P2 번호 | 실크 | ESP32 GPIO | 연결 |
|---:|---|---:|---|
| 1 | G | — | 보조 논리 GND |
| 2 | TX | GPIO43 | CH343P USB-UART TXD 물리 연결 |
| 3 | RX | GPIO44 | CH343P USB-UART RXD 물리 연결 |
| 4 | 1 | GPIO1 | 미사용(배터리 ADC 분압 미실장) |
| 5 | 2 | GPIO2 | 미사용(FAULT 미실장) |
| 17 | 47 | GPIO47 | 미사용 |
| 18 | 21 | GPIO21 | 미사용 |

GPIO35/36/37은 N16R8 모듈의 Flash/PSRAM 용도로 예약되어 외부 GPIO로 사용하지 않는다. GPIO19/20은 Native USB, GPIO48은 RGB LED다. GPIO0/3/45/46은 부팅/스트랩 관련이므로 외부 회로에 사용하지 않는다.

### 3.2 USB-UART와 UART peripheral

- CH343P는 VND018의 USB-UART 포트에 연결되어 P2-2(GPIO43 TX)와 P2-3(GPIO44 RX)를 사용한다.
- 펌웨어에서 Jetson host link는 UART1 TX=GPIO43, RX=GPIO44로 GPIO Matrix에 배정한다. UART0는 ROM/부트 용도로 남긴다.
- 서보는 UART2 TX=GPIO17, RX=GPIO18을 사용한다.
- 앱/진단 콘솔은 Native USB Serial/JTAG(GPIO19/20)로 옮긴다. CH343P 포트에 로그와 바이너리 프로토콜을 섞지 않는다.
- ROM 부팅 중 UART0 메시지가 CH343P에 먼저 나타날 수 있으므로 Jetson 수신기는 부팅 잡음을 버리고 매직 바이트+길이+CRC가 유효한 첫 프레임부터 동기화한다.
- 이 UART 매핑은 설계 목표다. 현재 firmware_bench가 이 주변장치들을 실제 구동하는 펌웨어라는 뜻은 아니다.

## 4. CF-0108 엔코더 레벨 변환 회로

### 4.1 확인된 것과 가정

- 팀 구매 문서에서 CF-0108은 **8채널 로직레벨 시프트 컨버터 모듈**로 선정됐다.
- 공개 판매 등록은 8채널 모듈이라는 제품명만 제공한다. 해당 CF-0108 납품품의 IC 종류·정확한 PCB 핀 배열을 판매 문서만으로 보증할 수 없다.
- 아래 주 결선은 흔히 유통되는 CJMCU-0108/TXS0108E 모듈과 동일하게 VA/VB/GND/OE/A1…A8/B1…B8 실크가 있고 내부 IC가 TXS0108E로 식별되는 경우의 회로다.
- **납땜·통전 전 조건:** 실제 CF-0108 보드에 VA, VB, GND, OE, A1/A2, B1/B2 실크가 있고 IC가 TXS0108E인지 확인한다. TXB0108, BSS138 또는 다른 IC면 이 결선을 그대로 적용하지 않는다. 판매명 CF-0108만으로 칩을 단정하지 않는다.

### 4.2 TXS0108E IC 핀 번호 (PW/DGS 20핀 패키지 기준)

IC 윗면의 1번 핀 표시를 기준으로 한다. 다른 패키지/칩이면 이 표를 적용하지 않는다.

| IC 핀 | 신호 | 연결 |
|---:|---|---|
| 1 | A1 | PGE-213 A 5 V 입력 채널 |
| 2 | VCCA | VND018 3V3 |
| 3 | A2 | PGE-213 B 5 V 입력 채널 |
| 4 | A3 | 미사용 |
| 5 | A4 | 미사용 |
| 6 | A5 | 미사용 |
| 7 | A6 | 미사용 |
| 8 | A7 | 미사용 |
| 9 | A8 | 미사용 |
| 10 | OE | VND018 GPIO14, GND에 10 kΩ pulldown |
| 11 | GND | 공통 논리 GND |
| 12 | B8 | 미사용 |
| 13 | B7 | 미사용 |
| 14 | B6 | 미사용 |
| 15 | B5 | 미사용 |
| 16 | B4 | 미사용 |
| 17 | B3 | 미사용 |
| 18 | B2 | PGE-213 B 출력 5 V |
| 19 | VCCB | J01 CH2 5 V |
| 20 | B1 | PGE-213 A 출력 5 V |

주의: 위 표는 TXS0108E IC 자체의 핀 번호다. CF-0108 breakout 보드의 헤더 패드 번호와 같지 않다. 보드 패드는 아래 실크 이름으로 연결한다.

### 4.3 표준 CJMCU/TXS0108E breakout의 헤더 배열

아래는 일반적인 2×10 CJMCU-0108 보드의 실크 배열 예시다. **실제 CF-0108 보드의 실크와 일치할 때만** 이 행 순서를 사용한다.

| 헤더 행 | 왼쪽→오른쪽 패드 순서 |
|---|---|
| 고전압 B행 | 1 VB, 2 B1, 3 B2, 4 B3, 5 B4, 6 B5, 7 B6, 8 B7, 9 B8, 10 GND |
| 저전압 A행 | 1 VA, 2 A1, 3 A2, 4 A3, 5 A4, 6 A5, 7 A6, 8 A7, 9 A8, 10 OE |

보드가 회전되어 있으면 좌우가 반대가 된다. 패드 위치를 외워서 꽂지 말고 VA/VB/A1/B1/OE 실크를 직접 확인한다.

### 4.4 CF-0108 엔코더 연결

| CF-0108 패드 | 연결 대상 |
|---|---|
| VA | VND018 P1-1 3V3 |
| VB | J01 CH2 5 V, FE 0.25 A 퓨즈 뒤 |
| GND | J01 CH2− 및 공통 논리 GND |
| OE | VND018 P1-20 GPIO14; OE–GND 10 kΩ pulldown |
| B1 | PGE-213 XHP-4 pin 3 (A, 5 V) |
| A1 | 100 Ω 직렬 → VND018 P1-4 GPIO4 |
| B2 | PGE-213 XHP-4 pin 4 (B, 5 V) |
| A2 | 100 Ω 직렬 → VND018 P1-5 GPIO5 |
| A3~A8/B3~B8 | 미연결, 절연 |

전원 상승 중 OE는 10 kΩ pulldown으로 LOW(Hi-Z)다. 펌웨어는 VND018 3V3와 J01 CH2 5 V가 안정된 뒤 GPIO14를 HIGH로 설정한다. TXS0108E는 VCCA=3.3 V, VCCB=5 V로 운용하며 VCCA≤VCCB 조건을 만족한다. 모듈에 이미 OE pull-up이 실장돼 있다면 추가 pulldown과 충돌하지 않는지 실물 회로를 확인한다.

**TXS0108E가 확인되지 않는 경우의 확정 대체 회로:** 기존 Electrical R1의 U1 SN74LVC2G17DBVR 2채널 슈미트 버퍼 회로를 사용한다. U1 핀1=엔코더 A, 핀3=엔코더 B, 핀2=GND, 핀5=3V3, 핀6=100Ω→GPIO4, 핀4=100Ω→GPIO5. 핀1/3 각각 100 kΩ pulldown, 핀5–2에 100 nF. **CF-0108과 U1을 직렬로 중복 연결하지 않는다.** 둘 중 하나만 엔코더 레벨 변환에 사용한다.

## 5. PGE-213 엔코더 전원·커넥터

PGE-213은 2채널 A/B, 판매 사양 7 PPR, 5 V 공급(팀 구매 기록상 14–20 mA)이다.

| XHP-4 핀 | 신호 | 연결 |
|---:|---|---|
| 1 | VCC | J01 CH2 5 V → FE 0.25 A |
| 2 | GND | J01 CH2− / 공통 논리 GND |
| 3 | A | CF-0108 B1 |
| 4 | B | CF-0108 B2 |

보유 4핀→맨선 케이블의 선 색을 핀 번호로 가정하지 않는다. 멀티미터 도통으로 XHP-4 각 핀과 맨선의 대응을 기록한다. PGE VCC–GND 가까이에 100 nF + 1 µF(16 V 이상)를 병렬 배치한다. 모터 전력선은 PGE PCB의 녹색 단자에 연결하지 않는다. 모터 두 전력 탭에 직접 연결하고 절연/기계 고정한다.

## 6. IMU Adafruit 4438 LSM6DSOX 배선

| Adafruit 4438 단자 | VND018 연결 | 설명 |
|---|---|---|
| VIN | P1-1 또는 P1-2 3V3 | 3.3 V 전원 |
| GND | P1-22 GND | 공통 논리 GND |
| SCK/SCL | GPIO12 / P1-18 | SPI clock |
| SDI/MOSI | GPIO11 / P1-17 | MCU→IMU |
| SDO/MISO (하단 DO) | GPIO13 / P1-19 | IMU→MCU |
| CS (하단 CS) | GPIO10 / P1-16 | active-low CS |
| INT1 | GPIO9 / P1-15 | data-ready/interrupt |
| 3Vo | 미연결 | 출력이므로 전원으로 사용하지 않음 |
| I2/INT2, SCX/SDX, 상단 CS/DO, STEMMA QT | 미연결 | 미사용 |

CS와 VND018 3V3 사이에 10 kΩ pull-up을 둔다. SPI 초기 시험은 1 MHz. 축 방향, ODR, full-scale, INT1 극성, 장착 회전은 실물에서 교정한다.

## 7. 서보 전원·통신·수신 보호 회로

### 7.1 서보 전원

~~~text
M01 CH5 +12V ─ FS 2A ─ S_SERVO ─→ Adapter A H1-2 (DC+)
M01 CH5 GND ────────────────────→ Adapter A H1-1 (GND)

Adapter A H3:
  pin 1 DATA ───────────────→ ST3215-HS D
  pin 2 SERVO V+ ───────────→ ST3215-HS V
  pin 3 GND ────────────────→ ST3215-HS G
~~~

- M01 CH5 한 채널만 사용하고 CH6은 비운다.
- Adapter A H1의 2P 나사 단자를 사용한다. DC 5.5×2.1 잭과 H1을 동시에 연결하지 않는다.
- Adapter A USB-C는 비워 둔다. 통신은 H2 UART 사용.
- Adapter A의 서보 포트 H3 하나만 사용한다. H5는 미사용.
- 단일 ST3215-HS: 6–12.6 V, stall/locked-rotor 2.4 A. M01 CH5는 12 V/2 A이므로 정상 저부하 조향 시험만 우선한다. 기계적 끝단 구속/stall 금지, 전류 2 A 근접·전압 강하·리셋·과열 시 즉시 중단.

### 7.2 UART와 U2 수신 버퍼

U2 = SN74LVC2G17DBVR, SOT-23-6. TI 데이터시트 DBV top view의 핀 번호를 사용한다.

| U2 IC 핀 | 신호 | 연결 |
|---:|---|---|
| 1 | 1A | Adapter H2-2 RXD ← 1 kΩ 직렬, 100 kΩ pulldown to GND |
| 2 | GND | VND018 GND |
| 3 | 2A | GND에 연결(미사용 입력 고정) |
| 4 | 2Y | 미연결 |
| 5 | VCC | VND018 P1-1 3V3 |
| 6 | 1Y | 100 Ω 직렬 → GPIO18 / P1-11 |

추가:
- VND018 GPIO17/P1-10 → 1 kΩ 직렬 → Adapter A H2-3 TXD.
- Adapter A H2-1 GND → VND018 GND.
- U2 pin5–pin2 바로 옆 100 nF 디커플링.
- Adapter A 점퍼는 A 위치. Adapter USB-C는 비운다.
- 제조사 표기대로 TX→TX, RX→RX. H2 핀: 1=GND, 2=RXD, 3=TXD.
- U2는 MCU OFF 상태에서 RX 선 역급전 위험을 줄이는 버퍼다. 절연기/서보 전원 보호기가 아니다.
- ST3215-HS ID와 baud를 먼저 읽는다. 1 Mbps는 초기 후보이며 실제 설정을 읽어 확인한다.

## 8. MD20A·모터·하드웨어 STOP

### 8.1 모터 전력과 제어

~~~text
M01 CH1 +Vin ─ FM 10A ─────────→ MD20A VB+
M01 CH1 − ────────────────────→ MD20A VB−

MD20A MA ──────────────────────→ MB4266-24180 모터 탭 1
MD20A MB ──────────────────────→ MB4266-24180 모터 탭 2

VND018 GPIO15/P1-8 ─ 1kΩ ─ S_RUN(NO) ─ S_STOP-A(NC) ─→ MD20A PWM
MD20A PWM ─ 10kΩ pulldown ─ MD20A signal GND

VND018 GPIO6/P1-6 ─ 1kΩ ────────────────────────────→ MD20A DIR
MD20A DIR ─ 10kΩ pulldown ─ MD20A signal GND

VND018 GND ─────────────────────────────────────────→ MD20A signal GND
~~~

- MD20A: 6–30 V, 20 A 연속/60 A 피크, 3.3 V 논리 입력 지원, 역극성 보호 없음.
- PWM 20 kHz 이하. 초기 시험은 모터를 공중에 띄우고 낮은 duty부터 시작한다.
- PWM LOW는 coast가 아니라 brake다. STOP 시 MD20A PWM을 LOW로 만든다.
- MA/MB는 H-bridge 출력이며 어느 쪽도 GND가 아니다. 일반 접지형 오실로스코프 GND 클립을 MA/MB에 연결하지 않는다.
- 모터 전력선은 PGE-213 PCB/녹색 2P 단자에 연결하지 않는다. 모터 탭에 납땜/압착 후 절연·기계 고정한다.
- 방향은 바퀴를 띄운 상태에서 낮은 duty로 확인한다. 초기 MA→빨간 표시 탭은 후보일 뿐 실제 전진은 확인한다.

### 8.2 2NC 물리 STOP

필요:
- 잠금식/회전 해제형 버튼, 서로 독립된 NC 접점 2개.
- 별도 S_RUN 수동 운전 허가 스위치(NO, 기본 OPEN).
- STOP 상태 입력용 10 kΩ pull-up, PWM/DIR 각각 10 kΩ pulldown.

배선:
- S_STOP-A(NC): GPIO15에서 MD20A PWM으로 가는 경로에 직렬 삽입. 버튼 누름 또는 선 단선 시 PWM 경로가 열리고 MD20A 쪽 10 kΩ pulldown이 PWM LOW를 만든다.
- S_STOP-B(NC): GPIO8/P1-12와 GND 사이에 연결. GPIO8은 3V3에 10 kΩ pull-up. 정상 해제=LOW, 버튼 누름/선 단선=HIGH.
- S_RUN(NO): PWM 경로에 S_STOP-A와 직렬. 기본 OPEN, 명시적 수동 허가 시에만 닫는다.
- 버튼을 GPIO8에만 연결하지 않는다. 그건 MCU 소프트웨어 요청일 뿐 독립 PWM 차단이 아니다.
- STOP 해제 전에 S_RUN을 먼저 OPEN으로 하고 MCU/Jetson이 DISARMED인지 확인한다. STOP 해제만으로 자동 ARM/재출발 금지.

이 회로는 MCU 펌웨어와 독립적으로 MD20A PWM 입력을 억제하지만 배터리 전원을 직접 차단하거나 MD20A 내부 고장까지 막는 인증 안전회로가 아니다. 정지 응답·정지거리·단선 동작을 실물로 확인한다.

## 9. 전력·신호 GND 기준

- 각 부하의 전력 음극은 원래 PDB 채널 음극으로 직접 귀환한다. MD20A VB−를 ESP32 GND/USB 케이블로 대신하지 않는다.
- 논리 GND는 VND018–CF-0108–PGE GND–IMU GND–U2–MD20A signal GND–Adapter H2-1 GND 사이에서 공통 기준을 만든다.
- Adapter H1-1의 서보 전력 귀환은 M01 CH5−로 직접 돌아간다. H2-1은 논리 신호 GND다.
- 모터·서보 대전류가 IMU/엔코더 GND 선을 통과하지 않게 한다. USB shield를 전력 귀환선으로 사용하지 않는다.
- 무전원 상태에서 J01/M01 출력 음극과 논리 GND의 공통성/절연 상태를 측정해 기록한다. PDB 내부 접지 구성을 추정하지 않는다.

## 10. 전체 VND018 실제 GPIO 사용량

| GPIO | 기능 | 방향 |
|---:|---|---|
| 4 | 엔코더 A | 입력 |
| 5 | 엔코더 B | 입력 |
| 6 | MD20A DIR | 출력 |
| 8 | STOP 상태 | 입력 |
| 9 | IMU INT1 | 입력 |
| 10 | IMU CS | 출력 |
| 11 | IMU MOSI | 출력 |
| 12 | IMU SCLK | 출력 |
| 13 | IMU MISO | 입력 |
| 14 | CF-0108 OE | 출력 |
| 15 | MD20A PWM | 출력 |
| 17 | Servo UART TX | 출력 |
| 18 | Servo UART RX | 입력 |
| 43/44 | CH343P USB-UART 물리 TX/RX | UART1에 GPIO Matrix 매핑 |

GPIO1(배터리 ADC), GPIO2(FAULT), GPIO21(ENABLE), GPIO47(WATCHDOG)은 R2에서 미사용/미배선이다. 배터리 분압 회로가 없으므로 배터리를 GPIO1에 직접 연결하지 않는다. GPIO14는 CLEAR가 아니라 CF-0108 OE로 사용한다.

## 11. 조립 전 부속 BOM

| 부품 | 수량 | 값/규격 | 용도 |
|---|---:|---|---|
| CF-0108 | 1 | 8채널 모듈. 내부 IC가 TXS0108E인지 확인 | 엔코더 A/B 5→3.3 V |
| U2 SN74LVC2G17DBVR | 1 | SOT-23-6 | 서보 RX 보호 버퍼 |
| SOT-23-6 변환 기판 | 1 | DBVR 핀번호 확인 | U2 실장 |
| 100 nF | 3 기본 + CF 미실장 시 2 추가 | 세라믹 | PGE 1, U2 1, PGE 보드/CF 모듈 전원 디커플링; CF 모듈에 VA/VB 캡이 없으면 2개 추가 |
| 1 µF | 1 | ≥16 V | PGE VCC 디커플링 |
| 100 Ω | 3 | 1/8 W 이상 | CF A1/A2, U2 출력 직렬 |
| 1 kΩ | 4 | 1/8 W 이상 | PWM, DIR, Servo TX, Servo RX 직렬 |
| 10 kΩ | 5 | 1/8 W 이상 | OE pulldown, PWM/DIR pulldown, STOP pull-up, IMU CS pull-up |
| 100 kΩ | 1 | 1/8 W 이상 | U2 input pulldown |
| F0/FJ/FM/FS/FE | 각 1 | 15/5/10/2/0.25 A, ≥32 VDC | 퓨즈·홀더 |
| S0 | 1 | ≥32 VDC, ≥20 A | 주전원 스위치 |
| S_SERVO | 1 | ≥32 VDC, ≥3 A | 서보 전원 스위치 |
| S_RUN | 1 | NO 저전류 접점 | 수동 구동 허가 |
| S_STOP | 1 | 잠금식 2NC | 물리 STOP |
| 배선 | 필요량 | 배터리/모터 16 AWG, J01 가지 18 AWG, Jetson 20 AWG, 서보 22 AWG, 엔코더 24 AWG, 신호 26 AWG 권장 | 전력·신호 분리 |
| 단자대/절연 기판/압착단자/수축튜브 | 필요량 | 브레드보드에 모터 전력 연결 금지 | 맨선 체결/절연 |

## 12. 설계 근거와 남은 실물 판정

이 문서는 기존 Electrical R1을 기준으로 전체 차량 배선을 다시 묶고, 엔코더 수신 U1 자리에 CF-0108을 조건부로 반영했다. 전원 분기·퓨즈·U2 서보 RX 버퍼·2NC STOP은 R1 회로를 유지했다.

**실물 CF-0108에 대해서만** 제품 등록은 8채널 레벨 시프터라는 점까지만 확인되며 내부 IC/실크는 확정하지 못했다. 따라서 아래 두 가지 중 하나만 시행한다.
1. 실물 CF-0108이 TXS0108E + VA/VB/GND/OE/A1..A8/B1..B8 보드와 일치하면 §4.2–4.4의 CF 결선을 사용한다.
2. 일치하지 않거나 칩 식별이 안 되면 CF에 전원을 넣지 않고, Electrical R1의 U1 SN74LVC2G17DBVR 회로를 사용한다.

나머지 회로는 핀 번호와 연결 경로를 고정했다. 다만 다음은 설계 미결이 아니라 실제 부품/환경의 승인 시험이다.
- PGE-213 커넥터 pin1 방향과 보유 하네스 도통.
- VND018 실물 P1/P2 헤더 연속성 및 납품 PCB 판본.
- CF-0108 내부 IC/실크가 §4의 TXS0108E 표준 보드와 일치하는지.
- M01 입력 24 V 미만에서 12 V 출력이 안정적인지.
- 정상 조향 시 서보 전류가 M01 CH5 2 A 범위에 드는지.
- B0390 15→22 FPC 접점 방향과 CAM0 인식/스트리밍.
- STOP 눌림/단선/MCU 멈춤에서 MD20A PWM LOW 및 실제 정지 응답.
- 모터축 encoder count/rev, 감속비, 유효 바퀴 둘레, IMU 축 방향.

## 13. 참고 자료

- 팀의 기존 Electrical R1 assembly_guide.pdf — U1/U2·퓨즈·STOP·PDB 전원 분기 근거(팀 제공 참고자료).
- [공식 J01/M01 PDB 사양](https://github.com/MOSW626/istech-it-arena/blob/main/docs/hardware/power-modules.md)
- [VCC-GND YD-ESP32-S3 제조사 핀맵](https://github.com/vcc-gnd/YD-ESP32-S3/blob/main/README.md)
- [TI TXS0108E 데이터시트 Rev.L](https://www.ti.com/lit/ds/symlink/txs0108e.pdf)
- [TI SN74LVC2G17 데이터시트 Rev.N](https://www.ti.com/lit/ds/symlink/sn74lvc2g17.pdf)
- [Adafruit LSM6DSOX 4438 핀 설명](https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/pinouts)
- [Waveshare Bus Servo Adapter A 결선](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example)
- [Waveshare Bus Servo Adapter A 회로도](https://files.waveshare.com/wiki/Bus_Servo_Adapter_A/Bus_Servo_Adapter_A.pdf)
- [Waveshare ST3215-HS 사양](https://www.waveshare.com/wiki/ST3215-HS_Servo_Motor)
- [Cytron MD20A 사양](https://my.cytron.io/p-20amp-6v-30v-dc-motor-driver)
- [NVIDIA Jetson Orin Nano 하드웨어 배치](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html)
- [Arducam B0390 제품 사양](https://www.arducam.com/90524.html)
- [Arducam Jetson Native Camera 연결 가이드](https://docs.arducam.com/Nvidia-Jetson-Camera/Native-Camera/Quick-Start-Guide/)
- [Motorbank PGE-213 상품/모델](https://www.motorbank.kr/goods/goods_view.php?goodsNo=1000009561)
