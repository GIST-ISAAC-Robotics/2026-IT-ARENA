# IT ARENA 실차 전장 연결 기준 — 사용자 확정 구성

- 기준 커밋: f3a2f7c031a52a52b4166c2df91d4377066c3654
- 기준일: 2026-11-11 (전원 배치는 유지, 실물 보드 식별·핀별 결선 보완)
- 목적: 이미 선정·구매한 부품을 우선 사용하고, 사용자가 승인한 전원·장치 배치만 연결한다.
- **이 문서는 승인되지 않은 U1/U2, 버퍼, 추가 저항 네트워크, 추가 퓨즈·스위치, 별도 안정화 회로를 설계에 추가하지 않는다.** 과거 임시 회로 기록은 현행 설계 근거로 사용하지 않는다. 사용자가 희망한 임시 IC 선택은 TXS0108E 계열이며 TXB0108 계열은 선택 대상이 아니다. 이는 CF-0108 실물 확인 전의 잠정 부품 방향이지 별도 임시 회로를 만들라는 뜻이 아니다.

**핀별 조립 기준:** [ESP32 통합 배선도 v1](ESP32_GPIO_WIRING_2026_11_11.md) · [그림 두 장과 연결표](schematics/esp32_gpio_wiring_v1.html). 아래 VND018은 구매 기록의 모델명이다. 11/11 실측 보드는 ESP32-S3·Flash16 MB/PSRAM8 MB 호환품이며 PCB 제조사/판본과 USB 브리지 세부 형번은 미확인이다. YD 보드 물리 핀 배열을 실물로 확정하지 않는다.

## 1. 승인된 시스템 배치

~~~mermaid
flowchart TD
  BAT["6S 배터리"] --> PDB["J01 / M01"]
  PDB -->|"J01 CH1 · 19 V"| JET["Jetson Orin Nano Super"]
  PDB -->|"M01 CH1 · 배터리 전압"| MD["MD20A"] --> MOTOR["MB4266-24180"]
  PDB -->|"M01 CH5 또는 CH6 · 12 V / 2 A 한 채널"| ADP["Bus Servo Adapter (A)"] --> SERVO["ST3215-HS × 1"]
  JET -->|"USB-A ↔ USB-C · 데이터 + 5 V"| MCU["VND018 ESP32-S3"]
  MCU -->|"3.3 V + SPI"| IMU["Adafruit 4438 IMU"]
  MCU <-->|"UART"| ADP
  JET <-->|"USB"| LIDAR["RPLIDAR C1"]
  JET <-->|"CSI · 22↔22 FPC"| CAM["B0390 / IMX219"]
  J01 -->|"CH4 · 5 V / 0.5 A · XT30"| ENC["PGE-213 엔코더"]
  ENC -->|"A/B"| CF["CF-0108 · 출력 방식 확인 후 적용"]
  CF -->|"필요한 경우에만 3.3 V 변환"| MCU
~~~

## 2. 전원 연결

| 장치 | 전원 공급 | 연결 |
|---|---|---|
| Jetson Orin Nano Super | J01 CH1 19 V | XT30 수 피그테일 → DC 5.5×2.5 mm, 중앙 + |
| ESP32-S3 호환 보드 | Jetson USB-A | Jetson USB-A → 실측 통신된 USB-UART USB-C. 같은 케이블로 5 V 전원과 USB-UART 통신. WCH 브리지 세부 형번 미확정 |
| IMU Adafruit 4438 | ESP32 3V3 | VND018 3V3 → IMU VIN, GND 공통 |
| PGE-213 엔코더 | **J01 CH4 5 V / 0.5 A** | J01 CH4 XT30 출력 → 엔코더 pin 1 VCC, pin 2 GND. Jetson J12는 사용하지 않음 |
| MB4266-24180 모터 | 배터리 → M01 CH1 | M01 CH1 출력 → MD20A VB+/VB− |
| ST3215-HS 서보 1개 | M01 CH5 또는 CH6 | 12 V / 2 A 한 채널 → Adapter A 전원 입력 |
| RPLIDAR C1 | Jetson USB | C1 USB 어댑터 → Jetson USB-A |
| B0390 카메라 | Jetson CSI | 카메라 22핀 ↔ Jetson 22핀 FPC. 11/10 사용자 실물 확인·연결 |

### 전원 관련 금지 사항
- ESP32는 Jetson USB로 전원 공급한다. VND018의 다른 USB-C 포트에 외부 5 V를 동시에 공급하지 않는다.
- IMU는 ESP32 3V3를 사용한다. IMU의 3Vo는 전원 입력이 아니다.
- 엔코더 전원은 **J01 CH4 5 V / 0.5 A**에서 공급한다. J01 출력은 XT30 암 커넥터이므로 보유 XT30 수 케이블을 사용한다. 기존 MCU 전용으로 표기된 CH4는 ESP32가 Jetson USB로 전원을 받으므로 비어 있다. Jetson J12와 VND018 5Vin은 사용하지 않는다.
- 서보는 M01의 12 V / 2 A 단일 채널만 사용한다. CH5/CH6은 각 독립 출력이며, 제조사가 병렬 운전/전류 공유를 명시적으로 허용하기 전에는 출력끼리 연결하지 않는다.
- 별도 DC-DC, 추가 전원 분배기, 대용량 콘덴서, U1/U2 버퍼 회로는 현재 승인 구성에 포함하지 않는다.

## 3. ESP32-S3 신호 핀 배정

| 기능 | ESP32 GPIO | 연결 대상 |
|---|---:|---|
| 엔코더 A/B | GPIO4 / GPIO5 | CF-0108을 사용하는 경우 변환된 A/B 출력 |
| 모터 PWM / DIR | GPIO15 / GPIO6 | MD20A PWM / DIR |
| IMU SCLK | GPIO12 | Adafruit 4438 하단 SCL |
| IMU MOSI | GPIO11 | Adafruit 4438 하단 SDA |
| IMU MISO | GPIO13 | Adafruit 4438 하단 DO |
| IMU CS | GPIO10 | Adafruit 4438 하단 CS |
| IMU INT1 | GPIO9 | Adafruit 4438 하단 I1, IRQ 동작 미검증 |
| 서보 UART TX / RX | GPIO17 / GPIO18 | Adapter A H2 TX/TXD / RX/RXD. 공식 동일 이름 결선, 점퍼 A. 실물 실크 대조 후 연결 |

보유 호환 보드에 YD/VND018 자료의 P1/P2 좌우·물리 순번을 적용하지 않는다. GPIO 실크와 실물을 대조하고, 확인되지 않은 물리 핀 번호는 임의로 만들지 않는다. 현재 IMU 배선은 유지하며 다른 장치는 설계 배정이다.

## 4. PGE-213 엔코더와 CF-0108

### 4.1 사용자가 제공한 PGE-213 제조사 상세자료에서 확인된 내용

| 항목 | 자료 표기 |
|---|---:|
| 모델 | PGE-213, 42파이 모터용 케이스 일체형 마그네틱 엔코더 |
| 채널 | 2채널 |
| 펄스 | 7 PPR / 모터 1회전 |
| 공급 전압 | DC 4.5–24 V |
| 입력 전류 | 14–20 mA |
| 출력 전류 | 최대 20 mA |
| 출력 누설 전류 | 최대 10 µA |
| 출력 상승/하강 시간 | 기준 조건에서 0.5 µs / 0.2 µs, 최대 1.5 µs |
| 커넥터 | XHP-4, 피치 2.5 mm |
| 커넥터 핀 | 1=VCC, 2=GND, 3=HA, 4=HB |
| 모터 단자 | 5=모터 −, 6=모터 + |

엔코더 전원은 **J01 CH4의 5 V / 0.5 A 출력**에서 공급한다. J01 출력 커넥터는 XT30 암이므로 보유한 XT30 수 리드를 사용한다. PGE-213 소비전류 14–20 mA는 CH4 0.5 A 한도 이내다. Jetson J12와 VND018 5Vin은 사용하지 않는다.

### 4.2 제조사 출력 회로도에서 읽히는 사항

사용자가 제공한 상세자료의 출력 회로도에는 다음 부품/연결이 그려져 있다.

- VCC 입력선에 10 Ω 직렬 저항
- VCC와 GND 사이 100 nF
- HA/HB 각 출력단에 4.7 kΩ 저항이 VCC 레일 쪽으로 연결된 형태
- 각 출력선에 470 Ω 직렬 저항
- 각 출력선에서 GND로 연결된 SMAJ26A 보호소자 및 10 nF 커패시터

따라서 '오픈 컬렉터일 것'이라는 가설만으로 ESP32 GPIO에 바로 연결할 수 있다고 결론 내리면 안 된다. 오픈 컬렉터는 출력 트랜지스터가 LOW로 당기는 방식이라는 뜻일 뿐, HIGH 전압은 외부/내부 pull-up이 결정한다. 제공된 회로도처럼 HA/HB에 4.7 kΩ이 VCC로 pull-up되어 있다면, 엔코더 VCC=5 V일 때 신호 HIGH가 5 V 쪽으로 올라갈 수 있다.

즉, **오픈 컬렉터로 확인되더라도 내부 4.7 kΩ pull-up이 5 V에 연결되어 있는지 확인해야 하며, 3.3 V 직접 입력 가능하다고 단정하지 않는다.** 제조사 회로도 판독과 실제 납품품이 일치하는지 답변/실험으로 확인한다.

### 4.3 출력 방식과 변환 필요성

- 5 V push-pull 출력이면 HIGH가 5 V이므로 ESP32-S3 GPIO에 직접 연결하지 않는다.
- NPN open-collector/open-drain이어도 내부 4.7 kΩ pull-up이 5 V VCC로 연결되어 있다면 출력 HIGH가 5 V가 될 수 있으므로 직접 연결하지 않는다.
- 3.3 V pull-up을 사용해 직접 읽을 수 있는 경우는 출력단이 open-collector/open-drain이고, 5 V로 연결된 내부 pull-up이 없거나 제조사가 3.3 V pull-up 사용을 명시적으로 허용한 경우에 한한다. 현재 자료만으로 이 조건을 확정할 수 없다.
- PGE-213은 출력 전류 최대 20 mA로 표기되어 있지만, 이는 3.3 V 직접 연결 허용 여부나 내부 pull-up 제거 가능성을 뜻하지 않는다.

### 4.4 실물 CF-0108 수령 및 IC 식별 (2026-11-12)

사용자가 제공한 실물 사진에서 다음을 확인했다.

- 중앙 IC 상단 마킹: `YF08E`, TI 로고, 추가 로트 표시 `69KG4 / CHQ7`. TI 공식 TXS0108E 데이터시트의 부품 표기표에서 **TXS0108EPWR(PW/TSSOP-20)의 정상 부품 마킹은 YF08E**다. 외형 및 마킹은 TXS0108E와 일치하며, TXB0108이 아닌 **TXS0108E일 가능성이 매우 높다**. 사진만으로 정품 여부나 정상 동작까지 증명되지는 않는다.
- 기판의 `C1`, `C2` 위치에 세라믹 커패시터 2개가 이미 실장돼 있다. **CF 모듈용 디커플링 커패시터를 추가 구매 대상으로 올리지 않는다.** 실제 회로와 해당 커패시터의 접속·값은 별도로 계측하지 않았다.
- 하단 실크: `VA A1 A2 A3 A4 A5 A6 A7 A8 OE`.
- 상단 실크: `VB`와 `B1`, `B7`, `B8`, `GND`는 판독 가능하나 **중간 B채널 글자가 중복/오기처럼 보인다**. 상단의 왼쪽에서 몇 번째인지 추정해 B2/B3 등의 기능을 단정하지 않는다.
- 양쪽 패드에 납땜 잔량과 돌출된 뾰족한 부분이 보인다(특히 하단 A측). 인접 패드 단락·납땜 브리지·핀헤더 접촉 불량을 전원 없이 먼저 검사하고 돌출부를 정리한다.

**공식 근거:** [TI TXS0108E 공식 데이터시트](https://www.ti.com/lit/ds/symlink/txs0108e.pdf) §4(PW 핀배열), Package Option Addendum(PWR 부품 마킹 YF08E), §7.3(출력 부하/커패시턴스 제약).

### 4.5 간단한 배선 후보 — 실물 확인과 파형 검증 전에는 통전 금지

기존 전원 배치는 변경하지 않는다. 엔코더 VCC/GND는 **J01 CH4 5 V/0.5 A**, ESP32는 Jetson USB로 구동, CF 전원은 두 레일에서 각각 분기한다.

| CF-0108 보드 실크 | 상대 장치 |
|---|---|
| `VA` | VND018 ESP32 `3V3` |
| `VB` (사진에서는 `UB`처럼 보일 수 있음) | J01 CH4의 5 V(엔코더 전원과 공유) |
| `GND` | 엔코더·ESP32와 공통 GND |
| `OE` | 활성 HIGH(3.3 V), 비활성 LOW. 부동 상태로 두지 않는다. 초기 배선에서 3.3 V 활성 방식은 전원 레일 안정 상태 확인 후 적용 |
| `B1` → `A1` | 엔코더 HA → GPIO4 (채널1 후보) |
| `B7` → `A7` | 엔코더 HB → GPIO5 (채널7 후보; 상단 중앙 오인쇄 피함) |

- B1/A1 및 B7/A7는 사진에서 라벨이 비교적 명료한 **배선 후보**이며, 실제 기판의 해당 채널 도통/변환 결과를 확인한 뒤 최종 확정한다. 채널 번호는 반드시 1·2를 연달아 사용할 필요가 없다.
- TXS0108E는 open-drain과 push-pull 방식의 변환을 지원하지만, PGE-213 제조사 회로도에는 각 신호에 5 V 방향 4.7 kΩ 풀업과 10 nF 커패시터가 그려져 있다. TI는 TXS0108E가 커패시터 부하와 외부 저항·에지 속도의 영향을 받는다고 설명한다. **IC 모델이 맞아도 엔코더 A/B 파형이 자동으로 보장되지는 않는다.**
- 최초에는 두 전원 레일/공통 GND, OE 상태를 확인하고 ESP32 신호선을 떼어둔 채 멀티미터·오실로스코프/로직 분석기로 A측 출력 전압과 에지를 확인한다. A측 HIGH가 3.3 V 범위인지 확인한 후 GPIO4/5로 연결한다.
- 모듈에 이미 장착된 C1/C2 및 PGE-213 내부 회로를 근거로, **추가 U1/U2·안정화 회로·별도 커패시터 구매를 자동으로 요구하지 않는다.**

### 4.6 다음 실물 시험에서 확정할 항목

1. 패드/실크 오기, 인접 패드 단락 여부 및 A1/B1·A7/B7 변환 동작.
2. VA=3.3 V, VB=5 V, GND 공통, OE 활성(3.3 V) 확인.
3. PGE-213 HA/HB 신호가 CF B측에서 A측으로 0–3.3 V 범위의 정상 쿼드러처 파형으로 전달되는지.
4. 신호 최대 에지율에서 PCNT 누락이 없는지. 부하 10 nF의 실제 영향은 파형으로 검증.

## 5. 장치별 신호 연결

### IMU Adafruit 4438
| IMU 단자 | VND018 |
|---|---|
| VIN | 3V3 |
| GND | GND |
| 하단 SCL | GPIO12 |
| 하단 SDA | GPIO11 |
| 하단 DO | GPIO13 |
| 하단 CS | GPIO10 |
| 하단 I1 | GPIO9 |

### MD20A
| MD20A 단자 | 연결 |
|---|---|
| VB+ / VB− | M01 CH1 + / − |
| MA / MB | MB4266-24180 모터 두 단자 |
| PWM | ESP32 GPIO15 |
| DIR | ESP32 GPIO6 |
| Signal GND | ESP32 GND |

MA/MB는 모터 출력이며 GND가 아니다. MD20A의 PWM LOW는 브레이크 동작이다. 최초 시험은 바퀴를 띄우고 낮은 PWM으로 한다.

### Bus Servo Adapter (A) / ST3215-HS
- M01의 12 V / 2 A 단일 채널을 Adapter A의 DC 5.5×2.1 입력 또는 해당 전원 단자에 연결한다.
- Adapter A 서보 포트에 ST3215-HS 한 개를 연결한다.
- ESP32 GPIO17(TX)→Adapter A H2 TX/TXD, GPIO18(RX)←H2 RX/RXD, GND↔GND로 연결한다. 이 제품은 일반 UART처럼 TX/RX를 교차하지 않는다.
- Waveshare UART 안내에 따라 점퍼는 A 위치로 둔다. 어댑터 H2 핀 실크와 공식 배선 예시를 대조해 연결한다.
- ST3215-HS의 stall 전류 2.4 A와 정상 조향 전류는 다르다. 기계적 끝단 구속을 하지 않고 실제 조향 전류·전압강하를 측정한다. M01 CH5/CH6 두 채널을 병렬 연결해 4 A를 얻는 방법은 제조사 병렬 운전 허용이 확인되지 않아 사용하지 않는다.

## 6. 카메라와 LiDAR

- 납품된 B0390은 **카메라 자체가 22핀**이며, 사용자가 2026-11-10 **22↔22 FPC로 Jetson에 연결**했다고 확인했다. 판매처 본문의 15핀 표기를 현물에 적용하지 않는다. 실제 CSI 포트는 소프트웨어 감지 결과와 대조하며, 별도 전원선을 카메라 FPC에 연결하지 않는다.
- FPC 접점 방향은 양쪽 커넥터 실물을 확인하고 잠금쇠를 연 상태에서 삽입한다.
- RPLIDAR C1 USB 어댑터를 Jetson USB-A에 연결한다. ESP32와 다른 USB 스택을 우선 사용한다.

## 7. 정격상 주의 — 승인된 배치를 변경하라는 뜻은 아님

1. J01 CH1은 19 V / 2.3 A다. 2.3 A는 공급 한도이지 Jetson 상시 소비전류가 아니다. 실제 부하에서 전압 강하와 재부팅 여부를 확인한다.
2. 공식 M01 입력 표기는 24–36 V다. 6S 배터리는 만충 25.2 V, 공칭 22.2 V이므로 운용 중 24 V 아래로 내려갈 수 있다. 지급 구성을 바꾸지 않되 팩 전압별 M01 동작과 출력 안정성을 시험하고 불안정하면 부하 운전을 중단한다.
3. MD20A는 6–30 V 입력이며 역극성 보호가 없다. VB+/VB− 극성을 확인한다.
4. 서보는 M01 12 V / 2 A 한 채널로 우선 운용한다. 2 A 한도에 걸리면 기계적 구속·링크 간섭·서보 설정을 먼저 점검한다. 별도 전원부는 실측 후에만 검토한다.

## 8. 구매 원칙

현재 구매 목록과 이미 선정된 부품을 우선 사용한다. 이 문서만으로 새 전자회로 부품의 추가 구매를 확정하지 않는다.

현재 추가 구매를 지정하지 않는 항목:
- U1/U2 버퍼 IC 및 변환 기판
- 별도 DC-DC 모듈
- 추가 레벨시프터
- 임의의 저항 네트워크
- 추가 콘덴서/대용량 커패시터
- 별도 전원 분배 PCB

먼저 기존 케이블·점퍼선·동봉품·CF-0108 실물을 확인한다. 추가 부품은 실제 연결 방식과 정격이 확인된 뒤 필요한 최소 수량만 구매 목록에 추가한다.

## 9. 근거와 미확인 범위

- 사용자가 제공한 PGE-213 상세자료에는 4.5–24 V 공급, 7 PPR, XHP-4 pin 1=VCC, 2=GND, 3=HA, 4=HB 및 출력 회로도가 있다. 회로도상 HA/HB에 VCC 방향 4.7 kΩ 저항이 표시되어 있어 오픈 컬렉터 가설만으로 3.3 V 직접 연결을 승인하지 않는다.
- 수령한 CF-0108 실물 IC에는 TI의 TXS0108EPWR(PW/TSSOP-20) 공식 마킹인 `YF08E`가 표시되어 있다. TXS0108E일 가능성이 매우 높지만 사진만으로 진품이나 전기적 정상 동작까지 보증하지 않는다. TXB0108 계열은 선택하지 않으며 PGE 출력 회로의 10 nF 부하가 TXS 성능에 미치는 영향은 실측한다.
- ESP32-S3 GPIO 절대 최대 입력 전압은 3.6 V다. 5 V push-pull 엔코더 출력을 직접 연결하지 않는다.
- J01 CH4는 5 V / 0.5 A 출력이며 현재 ESP32는 Jetson USB에서 전원을 받으므로 CH4를 엔코더 전용으로 재배정한다. J01의 출력 커넥터는 XT30 암이다. Jetson Orin Nano Developer Kit USB-A 포트 스택의 VBUS 한도는 3 A로 안내되어 있다.
- NVIDIA의 15→22 변환 FPC 안내는 카메라 쪽이 15핀일 때 해당한다. 현재 B0390 실물은 사용자 확인에 따라 22↔22 FPC를 사용한다. 상세 근거는 [케이블 대조 §3.1](PROCUREMENT_AND_CABLES_2026_11_08.md#31-b0390-ffc--어느-동봉-케이블을-쓰는가)을 따른다.

참고 자료:
- [PGE-213 제조사 상품](https://www.motorbank.kr/goods/goods_view.php?goodsNo=1000009561)
- [CF-0108 사용자 선정 상품](https://www.coupang.com/vp/products/13919546)
- [TI TXS0108E 데이터시트](https://www.ti.com/product/TXS0108E)
- [Espressif ESP32-S3 데이터시트](https://documentation.espressif.com/esp32_s3_datasheet_en.pdf)
- [NVIDIA Jetson Orin Nano 하드웨어 배치](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html)
- [Waveshare Adapter A UART 배선](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example)
- [Waveshare Adapter A 사양](https://docs.waveshare.com/Bus_Servo_Adapter_A)
- [Cytron MD20A 데이터시트](https://robu-prod-media.s3.ap-south-1.amazonaws.com/uploads/2019/05/MD20A-Datasheet.pdf)
