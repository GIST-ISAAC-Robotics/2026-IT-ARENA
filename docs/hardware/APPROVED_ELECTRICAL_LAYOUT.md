# IT ARENA 실차 전장 연결 기준 — 사용자 확정 구성

- 기준 커밋: f3a2f7c031a52a52b4166c2df91d4377066c3654
- 기준일: 2026-11-10
- 목적: 이미 선정·구매한 부품을 우선 사용하고, 사용자가 승인한 전원·장치 배치만 연결한다.
- **이 문서는 승인되지 않은 U1/U2, 버퍼, 추가 저항 네트워크, 추가 퓨즈·스위치, 별도 안정화 회로를 설계에 추가하지 않는다.** 과거 임시 회로 기록은 현행 설계 근거로 사용하지 않는다. 사용자가 희망한 임시 IC 선택은 TXS0108E 계열이며 TXB0108 계열은 선택 대상이 아니다. 이는 CF-0108 실물 확인 전의 잠정 부품 방향이지 별도 임시 회로를 만들라는 뜻이 아니다.

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
- 서보는 M01의 12 V / 2 A 단일 채널만 사용한다. CH5/CH6은 각 독립 출력이며, 제조사가 병렬 운전/전류 공유를 명시적으로 허용하기 전에는 출력끼리 연결하지 않는다.
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

엔코더 전원은 사용자가 정한 Jetson 40핀 J12 pin 2의 5 V, pin 6의 GND를 사용한다. 5 V는 PGE-213의 4.5–24 V 입력 범위 안이다. VND018의 5Vin은 사용하지 않는다.

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

### 4.4 CF-0108 선택 방향 — TXS를 잠정 목표로, TXB는 선택하지 않음

사용자가 말한 'S가 있는 IC'는 **TXS0108E 계열**, 비교 대상 'B'는 **TXB0108 계열**로 기록한다.

- 잠정 목표: TXS0108E 계열 8채널 양방향 레벨 변환 모듈.
- TXB0108 계열은 이번 엔코더 연결의 선택 목표로 삼지 않는다.
- TI 자료상 TXS0108E는 open-drain과 push-pull 양쪽을 지원하며 자동 방향 감지형이다. TXB 계열은 push-pull 중심의 다른 구조다.
- 이는 사용자가 지정한 희망 부품 방향을 기록한 것이며, 내일 도착할 CF-0108의 실제 IC가 TXS0108E라고 확인됐다는 뜻은 아니다.
- CF-0108 실물의 IC top-mark, 보드 실크, 회로를 확인한 뒤에만 A/B 배선을 확정한다. 판매명 CF-0108만으로 내부 IC를 단정하지 않는다.
- TXS0108E도 외부 부하 커패시턴스에 영향을 받는다. PGE-213 회로도에는 출력당 10 nF가 그려져 있으므로, 실제 회로와 CF 모듈을 함께 사용할 때 파형/상승시간을 측정해야 한다. 모듈이 TXS0108E라는 이유만으로 무조건 적합하다고 간주하지 않는다.

### 4.5 CF 도착 전 임시 상태

별도의 임시 U1/U2 회로나 저항 네트워크를 새로 제작하지 않는다. 사용자가 말한 임시 선택은 TXS0108E 계열을 목표로 삼는다는 뜻으로 기록한다. CF-0108 도착 전에는 엔코더 전원/커넥터만 확인하고 A/B를 ESP32에 연결하지 않는다. 기존에 실제로 보유한 변환 부품이 별도로 확인되지 않는 한 임의 회로를 만들지 않는다.

### 4.6 핀 배정 상태

- 엔코더 커넥터의 논리 핀은 제조사 자료 기준 1=VCC, 2=GND, 3=HA, 4=HB로 확인됐다.
- ESP32 쪽 GPIO4/5는 프로젝트의 엔코더 A/B 후보 할당으로만 남겨 둔다.
- **VND018 P1/P2 물리 헤더 번호와 CF-0108 모듈의 실제 패드 번호는 아직 최종 매칭하지 않았다.**
- CF-0108 앞/뒷면 및 IC 표기를 확인한 뒤 물리 핀 대 물리 핀 표를 확정한다.

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
- ST3215-HS의 stall 전류 2.4 A와 정상 조향 전류는 다르다. 기계적 끝단 구속을 하지 않고 실제 조향 전류·전압강하를 측정한다. M01 CH5/CH6 두 채널을 병렬 연결해 4 A를 얻는 방법은 제조사 병렬 운전 허용이 확인되지 않아 사용하지 않는다.

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

- 사용자가 제공한 PGE-213 상세자료에는 4.5–24 V 공급, 7 PPR, XHP-4 pin 1=VCC, 2=GND, 3=HA, 4=HB 및 출력 회로도가 있다. 회로도상 HA/HB에 VCC 방향 4.7 kΩ 저항이 표시되어 있어 오픈 컬렉터 가설만으로 3.3 V 직접 연결을 승인하지 않는다.
- 사용자가 희망한 임시 IC 방향은 TXS0108E 계열이며 TXB0108 계열은 선택하지 않는다. TI TXS0108E는 open-drain/push-pull 자동 양방향 변환을 지원하지만 CF-0108 실물에 해당 IC가 실장됐다는 증거는 아니다. PGE 출력 회로의 10 nF 부하가 TXS 성능에 미치는 영향도 실측해야 한다.
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
