# IT ARENA 실차 전장 연결 기준 — 사용자 확정 구성

- 기준 커밋: f3a2f7c031a52a52b4166c2df91d4377066c3654
- 기준일: 2026-11-10
- 목적: 이미 선정·구매한 부품을 우선 사용하고, 사용자가 승인한 전원·장치 배치만 연결한다.
- **이 문서는 승인되지 않은 U1/U2, 버퍼, 풀업/풀다운, 추가 퓨즈·스위치, 별도 안정화 회로를 설계에 추가하지 않는다.** 과거 임시 회로 기록은 현행 설계 근거로 사용하지 않는다.

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
  JET <-->|"CAM0 CSI · 15↔22 FPC"| CAM["B0390 / IMX219"]
  JET -->|"J12 pin 2 · 5 V"| ENC["PGE-213 엔코더"]
  ENC -->|"A/B"| CF["CF-0108 · 출력 방식 확인 후 적용"]
  CF -->|"필요한 경우에만 3.3 V 변환"| MCU
~~~

## 2. 전원 연결

| 장치 | 전원 공급 | 연결 |
|---|---|---|
| Jetson Orin Nano Super | J01 CH1 19 V | XT30 수 피그테일 → DC 5.5×2.5 mm, 중앙 + |
| ESP32-S3 VND018 | Jetson USB-A | Jetson USB-A → VND018 CH343P USB-UART USB-C. 같은 케이블로 5 V 전원과 USB-UART 통신 |
| IMU Adafruit 4438 | ESP32 3V3 | VND018 3V3 → IMU VIN, GND 공통 |
| PGE-213 엔코더 | Jetson 40핀 J12 | pin 2 = 5 V, pin 6 = GND |
| MB4266-24180 모터 | 배터리 → M01 CH1 | M01 CH1 출력 → MD20A VB+/VB− |
| ST3215-HS 서보 1개 | M01 CH5 또는 CH6 | 12 V / 2 A 한 채널 → Adapter A 전원 입력 |
| RPLIDAR C1 | Jetson USB | C1 USB 어댑터 → Jetson USB-A |
| B0390 카메라 | Jetson CSI | 카메라 15핀 ↔ Jetson 22핀 FPC |

### 전원 관련 금지 사항
- ESP32는 Jetson USB로 전원 공급한다. VND018의 다른 USB-C 포트에 외부 5 V를 동시에 공급하지 않는다.
- IMU는 ESP32 3V3를 사용한다. IMU의 3Vo는 전원 입력이 아니다.
- 엔코더는 Jetson J12 5 V를 사용한다. VND018의 5Vin은 사용하지 않는다.
- 서보는 M01의 12 V / 2 A 단일 채널만 사용한다. CH5/CH6 병렬 연결 금지.
- 별도 DC-DC, 추가 전원 분배기, 대용량 콘덴서, U1/U2 버퍼 회로는 현재 승인 구성에 포함하지 않는다.

## 3. ESP32-S3 신호 핀 배정

| 기능 | ESP32 GPIO | 연결 대상 |
|---|---:|---|
| 엔코더 A/B | GPIO4 / GPIO5 | CF-0108을 사용하는 경우 변환된 A/B 출력 |
| 모터 PWM / DIR | GPIO15 / GPIO6 | MD20A PWM / DIR |
| IMU SCLK | GPIO12 | Adafruit 4438 SCK |
| IMU MOSI | GPIO11 | Adafruit 4438 SDI |
| IMU MISO | GPIO13 | Adafruit 4438 SDO |
| IMU CS | GPIO10 | Adafruit 4438 CS |
| IMU INT1 | GPIO9 | Adafruit 4438 INT1 |
| 서보 UART | 기존 프로젝트 UART 핀 배정 | Adapter A UART. 실제 보드 실크 및 제조사 배선 예시 확인 후 연결 |

VND018 실물 사진 방향은 안테나 위, USB-C 두 개 아래이며 이때 왼쪽 헤더가 P1, 오른쪽이 P2다. 물리 헤더 번호는 제조사 핀맵과 실물 실크를 대조한다. 확인되지 않은 물리 핀 번호는 임의로 만들지 않는다.

## 4. 엔코더와 CF-0108 — 출력 방식에 따라 결정

### 확인된 정보
- PGE-213: 2채널 자기식 증분 엔코더, 판매 페이지 표기 7 PPR.
- 전원: 팀 구매 기록의 5 V.
- 공개 판매 페이지에서 A/B 출력 회로가 push-pull인지 NPN open-collector/open-drain인지, 출력 HIGH 전압이 얼마인지 확인되지 않는다.

### 출력 방식에 따른 차이

**A. 5 V push-pull 출력**
- HIGH일 때 엔코더가 실제 5 V를 출력한다.
- ESP32-S3 GPIO에 직접 연결하면 안 된다. ESP32-S3 GPIO 절대 최대 입력 전압은 3.6 V다.
- CF-0108이 해당 push-pull 출력과 주파수에 적합한 레벨 변환기임을 확인한 뒤 A/B 두 채널에 사용한다.

**B. NPN open-collector/open-drain 출력**
- 엔코더 출력은 LOW로 당기는 동작만 하고 HIGH 전압을 직접 출력하지 않는다.
- 이 경우 신호선을 3.3 V로 pull-up하면 ESP32가 직접 읽을 수 있어 별도 레벨시프터가 필요하지 않을 수 있다.
- pull-up 저항값은 허용 sink 전류, 케이블 길이, 펄스 주파수에 맞춰야 한다. 출력 회로 확인 전 임의로 저항을 추가하지 않는다.

### 실물 확인 순서
1. PGE-213 제조사에 A/B 출력 회로, 출력 HIGH 전압, 출력 전류, 커넥터 핀 배열을 확인한다.
2. 확인 전 A/B를 ESP32 GPIO에 연결하거나 CF-0108을 전원에 연결하지 않는다.
3. 제조사 자료를 얻지 못하면 PGE 전원·GND를 먼저 식별한 뒤, 전류 제한된 5 V 공급으로 A/B 출력 파형을 계측한다.
4. CF-0108 실물의 IC와 보드 실크를 확인한다. 판매명만으로 TXS0108E/BSS138 등 내부 회로를 단정하지 않는다.
5. 엔코더 출력 방식과 CF-0108이 맞는 경우에만 연결한다.

### CF-0108 연결 원칙
CF-0108 실물의 앞·뒷면 및 IC 표기를 아직 확인하지 못했으므로 물리 패드 번호를 임의로 확정하지 않는다. 보드의 전원·GND·채널 실크를 확인하고 해당 모듈 회로에 맞춰 연결한다.

TXS0108E 기반 모듈로 확인된 경우에만:
- 저전압 측 = ESP32 3.3 V
- 고전압 측 = 엔코더 신호 측 5 V
- GND = 공통
- 엔코더 A/B = 5 V 측 두 채널
- ESP32 GPIO4/5 = 3.3 V 측 대응 채널

다른 회로라면 이 연결을 적용하지 않는다. 별도 버퍼 IC, RC 필터, 풀업/풀다운 네트워크를 임의로 추가하지 않는다.

## 5. 장치별 신호 연결

### IMU Adafruit 4438
| IMU 단자 | VND018 |
|---|---|
| VIN | 3V3 |
| GND | GND |
| SCK/SCL | GPIO12 |
| SDI/MOSI | GPIO11 |
| SDO/MISO | GPIO13 |
| CS | GPIO10 |
| INT1 | GPIO9 |

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
- ESP32와 Adapter A는 UART로 연결한다.
- Waveshare UART 안내에 따라 점퍼는 A 위치로 둔다. 어댑터 H2 핀 실크와 공식 배선 예시를 대조해 연결한다.
- ST3215-HS의 stall 전류 2.4 A와 정상 조향 전류는 다르다. 기계적 끝단 구속을 하지 않고 실제 조향 전류·전압강하를 측정한다.

## 6. 카메라와 LiDAR

- B0390 카메라 15핀과 Jetson CAM0/J20 22핀을 15↔22 FPC로 연결한다. 별도 전원선을 카메라 FPC에 연결하지 않는다.
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

- PGE-213 판매 페이지에서 확인되는 것은 2채널 자기식 엔코더, 7 PPR, 호환 모터 모델이다. 출력 회로 종류와 출력 전기 사양은 공개 페이지에서 확인되지 않았다.
- TXS0108E는 A측 1.4–3.6 V, B측 1.65–5.5 V, 자동 양방향 변환을 지원하지만 CF-0108 실물에 해당 IC가 실장됐다는 증거는 아니다.
- ESP32-S3 GPIO 절대 최대 입력 전압은 3.6 V다. 5 V push-pull 엔코더 출력을 직접 연결하지 않는다.
- Jetson Orin Nano Developer Kit USB-A 포트 스택의 VBUS 한도는 3 A로 안내되어 있다. 40핀 헤더의 정확한 전류 한도는 Carrier Board Specification을 적용한다.
- NVIDIA는 15핀 카메라를 22핀 CSI 커넥터에 연결할 때 15→22 변환 FPC를 안내한다.

참고 자료:
- [PGE-213 제조사 상품](https://www.motorbank.kr/goods/goods_view.php?goodsNo=1000009561)
- [CF-0108 사용자 선정 상품](https://www.coupang.com/vp/products/13919546)
- [TI TXS0108E 데이터시트](https://www.ti.com/product/TXS0108E)
- [Espressif ESP32-S3 데이터시트](https://documentation.espressif.com/esp32_s3_datasheet_en.pdf)
- [NVIDIA Jetson Orin Nano 하드웨어 배치](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html)
- [Waveshare Adapter A UART 배선](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example)
- [Waveshare Adapter A 사양](https://docs.waveshare.com/Bus_Servo_Adapter_A)
- [Cytron MD20A 데이터시트](https://robu-prod-media.s3.ap-south-1.amazonaws.com/uploads/2019/05/MD20A-Datasheet.pdf)
