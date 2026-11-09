# IT ARENA 실물 이관·조립·펌웨어 포팅·시험 절차 R2

- 기준일: 2026-11-10
- 적용 회로: [전체 전장 회로도 R2](ELECTRICAL_SCHEMATIC_R2_2026_11_10.md)
- 핀-투-핀 연결: [R2 Netlist CSV](electrical_netlist_r2_2026_11_10.csv)
- 목적: **부품을 책상에 놓은 시점부터 배선·통전·ESP32 프로그램 이관·센서 교정·Jetson 통합·저속 주행까지 순서대로 진행**
- 주의: 문서 작성은 실제 조립/통전/펌웨어 빌드/주행을 수행했다는 뜻이 아니다. 각 단계의 측정 결과를 기록한 뒤 통과 처리한다.

## 0. 작업 기준과 완료 정의

### 0.1 작업 순서

1. 부품/커넥터 실물 확인
2. PDB 배터리 입력 하네스 및 퓨즈/스위치 조립
3. CF-0108 및 U2 인터페이스 기판 조립
4. VND018 헤더 납땜 및 센서/드라이버 신호선 배선
5. Jetson USB/CSI/LiDAR 배선
6. 무전원 도통·극성 검사
7. PDB 단독 통전 및 출력 검사
8. ESP32 전원/부팅/USB-UART 확인
9. ESP32 펌웨어 이관과 Jetson 통신 확인
10. 엔코더·IMU·서보·모터 개별 시험 및 교정
11. 하드웨어 STOP 및 통신 단절 시험
12. 센서 동시 구동, Jetson 부하, 저속 실차 주행

### 0.2 완료 정의

- **배선 완료:** 양끝 연결, 핀 번호, 케이블 라벨, 극성, 공통 GND가 Netlist와 일치하고 도통 검사가 기록됨.
- **통전 완료:** 각 레일의 무부하/부하 전압과 전류가 기록되고 리셋·발열·퓨즈 단선이 없음.
- **펌웨어 이관 완료:** 실제 ESP-IDF 프로젝트에서 UART/PCNT/SPI/LEDC/Servo UART/STOP을 구현하고 테스트 벡터와 실물 로그가 저장됨.
- **실차 준비 완료:** 독립 STOP, watchdog/통신 단절, 조향각, 엔코더 계수, 모터 방향·제동, 카메라/LiDAR 동시 구동을 통과함.

## 1. 준비물

### 1.1 공구/계측기

- 멀티미터(전압·저항·도통), 전류 측정용 DC 클램프 또는 직렬 전류계
- 가능하면 오실로스코프 또는 로직 분석기(엔코더 A/B, PWM/DIR, UART 확인)
- 전류 제한 가능한 벤치 전원(첫 통전용), 납땜 인두/납/플럭스/흡입기
- 압착기, 스트리퍼, 수축튜브, 케이블 타이, 라벨
- 절연된 단자대/소형 만능기판/나사 고정물
- 바퀴를 공중에 띄울 스탠드, 각도기/디지털 경사계, 자, 회전수 표시 테이프
- Jetson용 microSD/전원, 개발 PC, USB 데이터 케이블

**브레드보드에 모터·배터리·PDB 전력선을 연결하지 않는다.** 신호 확인용 임시 배선만 사용한다.

### 1.2 실물 인수 체크

| 부품 | 확인 |
|---|---|
| J01/M01 | 채널 실크, XT30 암 커넥터, 입력 극성 |
| 6S 배터리 | XT60 암, 팩 전압, 셀 밸런스/손상 |
| Jetson | DC 잭 5.5×2.5 mm, CSI CAM0/CAM1, USB-A 스택 |
| VND018 | ESP32-S3-WROOM-1 N16R8, CH343P USB-UART 포트, Native USB 포트, P1/P2 실크 |
| CF-0108 | IC top-mark, VA/VB/GND/OE/A1/B1… 실크, 2×10 헤더 배열 |
| PGE-213 | XHP-4 커넥터, 4핀 케이블, 엔코더 방향 |
| Adafruit 4438 | VIN/GND/SCK/SDI/SDO/CS/INT1 표기 |
| Adapter A | H1 전원 단자, H2 UART 3핀, H3 서보 3핀, 점퍼 A |
| ST3215-HS | 서보 D/V/G 케이블, ID/baud 설정 |
| MD20A | VB+/VB−, MA/MB, PWM/DIR/GND 단자 |
| B0390 | 카메라 15-pin FPC, 15↔22 FPC |
| C1 | USB 어댑터/케이블 |

### 1.3 CF-0108 식별 — 전원 넣기 전

- 판매 등록으로 확인된 것은 CF-0108이 8채널 로직 레벨 시프터라는 점이다. 내부 IC와 실제 보드 핀 순서는 판매명만으로 보증되지 않는다.
- R2 주 결선은 표준 CJMCU-0108/TXS0108E 모듈의 VA/VB/GND/OE/A1~A8/B1~B8 구조를 가정한다.
- 실제 보드에서 VA, VB, GND, OE, A1, A2, B1, B2 실크와 IC top-mark를 확인한다.
- IC가 TXS0108E인지 식별할 수 있고 보드 실크가 표준 배열과 일치할 때만 CF-0108을 사용한다.
- TXB0108/BSS138/기타 IC이거나 핀 실크가 다르면 CF-0108에 전원을 넣지 않는다. 대체는 회로도 R2 §4.4의 U1 SN74LVC2G17 회로다.
- CF-0108 보드 앞/뒷면 사진과 IC 글자를 작업 기록에 남긴다.

## 2. 배터리 하네스와 PDB 조립

### 2.1 무전원 상태 하네스 조립

1. 배터리에서 분리된 상태로 XT60 수↔XT30 수 변환선과 XT30 Y선을 준비한다.
2. 배터리 XT60 암과 변환선 XT60 수가 맞는지 확인한다.
3. 멀티미터 도통으로 배터리 +가 변환선 XT30 +로, 배터리 −가 XT30 −로 이어지는지 확인한다.
4. 배터리 양극 바로 뒤 F0 15 A 홀더를 넣고 S0 MAIN 스위치를 직렬로 연결한다.
5. S0 출력에서 XT30 Y 분기를 만든다.
6. J01 양극 가지에는 FJ 5 A를 직렬로 넣고 J01 INPUT+로 연결한다.
7. M01 양극 가지는 Y 분기에서 M01 INPUT+로 연결한다.
8. 배터리 음극은 J01 INPUT−와 M01 INPUT−에 각각 귀환한다.
9. F0/FJ를 제거한 상태에서 +/− 단락이 없는지 검사한다. 퓨즈는 전원 극성과 단락 검사를 통과한 뒤 장착한다.

### 2.2 PDB 출력 케이블

| 출발 | 도착 | 케이블/보호 |
|---|---|---|
| J01 CH1 +/− | Jetson DC 5.5×2.5 mm | XT30 수 피그테일, 19 V, 중앙 + |
| J01 CH2 + | FE 0.25 A → PGE VCC 및 CF VB | 24 AWG |
| J01 CH2 − | PGE GND 및 CF GND | 24 AWG |
| M01 CH1 + | FM 10 A → MD20A VB+ | 16 AWG |
| M01 CH1 − | MD20A VB− | 16 AWG |
| M01 CH5 + | FS 2 A → S_SERVO → Adapter A H1-2 | 22 AWG |
| M01 CH5 − | Adapter A H1-1 | 22 AWG |

- J01 CH3/CH4/CH5/CH6은 R2에서 사용하지 않는다. 단자 절연.
- M01 CH2/CH3/CH4/CH6은 R2에서 사용하지 않는다. 단자 절연.
- Adapter A H1 나사 단자와 DC 잭을 동시에 연결하지 않는다.
- XT30/XT60 암수는 극성을 보증하지 않는다. 케이블 양끝 극성을 다시 측정한다.

### 2.3 첫 PDB 통전

- 첫 시험 팩 전압은 24.0–25.2 V 구간으로 한다. M01 공식 입력 하한은 24 V다.
- 가능하면 전류 제한 벤치 전원으로 PDB 입력을 먼저 시험한다.
- Jetson, 서보, 모터, 엔코더는 처음부터 모두 연결하지 않는다.
- 무부하로 J01 CH1=19 V, CH2=5 V, M01 CH1=입력 pass-through, M01 CH5=12 V를 측정한다.
- 출력이 맞지 않거나 스위치/퓨즈/커넥터가 뜨거워지면 즉시 전원을 차단한다.
- 24 V 미만에서 M01 CH5 12 V 출력의 정상 동작은 제조사 확인 사항이 아니므로 실제 팩 전압별 출력/발열을 별도 기록한다.

## 3. 인터페이스 기판과 VND018 배선

### 3.1 CF-0108 조립

1. 전원을 모두 분리한다.
2. CF-0108의 IC와 핀 실크를 확인한다.
3. 표준 TXS0108E 모듈로 확인된 경우 VA=3.3 V, VB=5 V, GND 공통으로 배선한다.
4. OE와 GND 사이에 10 kΩ pulldown을 설치한다. GPIO14는 VA/VB 안정 후 HIGH로 올린다.
5. PGE A/B를 B1/B2에, A1/A2를 각각 100 Ω 직렬로 GPIO4/5에 연결한다.
6. A3~A8/B3~B8은 미사용으로 두고 노출 핀을 절연한다.
7. 모듈에 이미 전원 디커플링이 실장돼 있는지 확인한다. 없으면 VA-GND와 VB-GND에 100 nF를 추가한다.
8. B1/B2 데이터 라인에 임의의 100 kΩ pulldown을 추가하지 않는다. TXS0108E의 내부 pull-up 동작과 충돌할 수 있다.

### 3.2 U2 서보 RX 버퍼 조립

1. SN74LVC2G17DBVR을 SOT-23-6 변환 기판에 실장한다. 핀1 방향을 확인한다.
2. U2 pin5=3V3, pin2=GND, pin5-pin2에 100 nF.
3. Adapter RXD → 1 kΩ → U2 pin1(1A), pin1에서 100 kΩ pulldown to GND.
4. U2 pin6(1Y) → 100 Ω → VND018 GPIO18.
5. U2 pin3(2A)는 GND에 묶고 pin4(2Y)는 NC.
6. MCU TX GPIO17 → 1 kΩ → Adapter H2-3 TXD.
7. Adapter H2-1 GND → VND018 GND.

### 3.3 VND018 헤더 납땜 및 핀 연결

- ESP32 안테나가 위, USB-C가 아래일 때 왼쪽이 P1, 오른쪽이 P2다.
- 헤더를 납땜하기 전 P1/P2 실크와 핀 순서를 확인한다. 핀 번호는 GPIO 번호와 다르다.
- 신호선은 회로도 R2 §3과 Netlist CSV의 양끝 핀을 따라 연결한다.
- P1-1 또는 P1-2 3V3는 IMU VIN과 CF VA에만 사용한다.
- P1-22 GND를 논리 기준 GND로 사용한다.
- P1-4/5는 엔코더 A/B, P1-6 DIR, P1-8 PWM, P1-10/11 Servo TX/RX, P1-12 STOP, P1-15/16/17/18/19 IMU INT/CS/MOSI/SCLK/MISO, P1-20 CF OE다.
- P2-2/P2-3는 CH343P UART 물리 신호 GPIO43/44다.
- P2-4 GPIO1, P2-5 GPIO2, P2-17 GPIO47, P2-18 GPIO21은 이번 회로에서 연결하지 않는다.

## 4. 센서·구동 장치 배선

### 4.1 엔코더

1. PGE-213 XHP-4 pin 1~4와 보유 맨선 케이블의 도통을 확인한다. 색상으로 VCC/GND/A/B를 추측하지 않는다.
2. J01 CH2 5 V → FE 0.25 A → PGE pin1 VCC, CF VB.
3. J01 CH2− → PGE pin2 GND, CF GND.
4. PGE pin3 A → CF B1, PGE pin4 B → CF B2.
5. CF A1/A2 → 100 Ω → ESP32 GPIO4/5.
6. CF VA=ESP32 3V3, OE=GPIO14, OE pulldown 10 kΩ.
7. 전원 투입 전 CF IC와 핀 배열이 R2와 일치하는지 확인한다.
8. 첫 시험은 모터 전원을 분리하고 엔코더만 공급한다. PGE 5 V, CF VA 3.3 V, VB 5 V, A1/A2 출력 0~3.3 V를 측정한다.
9. 저속 회전에서 A/B 위상, high/low 레벨, 펄스 누락을 오실로스코프/로직 분석기로 확인한다.

### 4.2 IMU

1. Adafruit 4438 VIN=3V3, GND=VND GND.
2. GPIO12=SCK, GPIO11=MOSI, GPIO13=MISO, GPIO10=CS, GPIO9=INT1.
3. CS에 10 kΩ pull-up to 3V3.
4. SPI 1 MHz부터 WHO_AM_I를 읽고, 이후 ODR/full-scale/INT1 극성을 설정한다.
5. 보드 장착 방향을 기준으로 정지 바이어스와 축 부호를 기록한다.

### 4.3 서보와 실제 조향각 교정

1. M01 CH5 12 V → FS 2 A → S_SERVO → Adapter A H1-2(+), H1-1(GND).
2. Adapter A 점퍼는 A, USB-C는 비움.
3. GPIO17→H2-3 TXD, H2-2 RXD→U2→GPIO18, H2-1 GND→VND GND.
4. Adapter H3 pin1/2/3 = DATA/V+/GND를 서보 D/V/G에 연결한다.
5. 바퀴/조향 링크를 분리한 상태에서 서보 ID·baud·현재 위치를 읽는다. 1 Mbps는 후보일 뿐 실제 설정을 먼저 읽는다.
6. 서보 horn을 분리하고 무리하게 회전시키지 않는다. 토크 OFF/저토크 상태에서 중앙 위치를 정한 뒤 horn을 장착한다.
7. 링크를 연결하고 중립 명령을 줬을 때 타이어가 직진하는지 확인한다.
8. 차량을 위에서 내려다보고 각도기/디지털 각도계로 좌·우 바퀴의 실제 조향각을 측정한다.
9. 서보 위치 명령을 중앙 기준으로 작은 간격(예: ±5°, ±10°, ±15° 상당)으로 바꾸며 서보 raw position과 좌/우 바퀴각을 표로 기록한다. 이 값은 초기 시험 단계의 명령 간격이지 서보 raw 단위와 동일하다는 뜻은 아니다.
10. 좌·우 바퀴각을 각각 측정해 중앙 오차, 좌우 비대칭, 유격, 히스테리시스를 확인한다. 차량 모델에 사용할 대표 조향각을 결정한다.
11. 기계적 끝단에 계속 밀어붙이지 않는다. 소프트웨어 min/max를 기계적 간섭보다 안쪽에 설정한다.
12. M01 CH5 2 A 한도 안에서 실제 조향 피크전류와 12 V 최저전압을 측정한다. 2 A 근접/초과, 전압강하, 리셋, 과열 시 중단한다. ST3215-HS stall 2.4 A 시험은 하지 않는다.

### 4.4 MD20A와 모터

1. M01 CH1+ → FM 10 A → MD20A VB+, M01 CH1− → VB−.
2. MD20A MA/MB를 모터 탭에 연결하고 수축튜브/절연재로 완전히 절연한다.
3. GPIO15 PWM, GPIO6 DIR, VND GND→MD20A signal GND.
4. PWM과 DIR에 1 kΩ 직렬, 각 입력에 10 kΩ pulldown.
5. 모터를 공중에 띄우고 PWM=0으로 시작한다. PWM 주파수는 20 kHz 이하.
6. 낮은 duty에서 방향을 확인하고, 필요하면 MA/MB 또는 DIR 논리를 반전한다.
7. PWM LOW는 brake다. 제동 동작과 coast를 혼동하지 않는다.
8. 전류·전압·드라이버 온도를 기록한다. 모터 stall/구속 시험은 하지 않는다.

### 4.5 STOP 버튼

1. 2NC 잠금식 STOP 버튼을 장착한다.
2. 접점 A(NC)는 GPIO15 PWM에서 MD20A PWM으로 가는 직렬 경로에 넣는다.
3. 접점 B(NC)는 GPIO8과 GND 사이에 연결하고 GPIO8은 10 kΩ으로 3V3 pull-up한다.
4. S_RUN(NO)은 PWM 경로에 STOP-A와 직렬로 넣고 기본 OPEN으로 둔다.
5. 버튼 해제 상태: STOP-A/STOP-B 닫힘, GPIO8 LOW, 단 S_RUN이 OPEN이면 모터는 구동되지 않아야 한다.
6. 버튼 누름/배선 단선: STOP-A 경로 개방→PWM pulldown LOW, STOP-B 개방→GPIO8 HIGH.
7. 버튼을 누른 뒤 해제해도 S_RUN은 OPEN 유지. 펌웨어 CLEAR 후 별도 ARM 전까지 재출발하지 않게 한다.
8. MCU를 멈추거나 USB를 뽑은 상태에서도 STOP-A가 PWM을 억제하는지 확인한다. 정지거리도 측정한다.

### 4.6 카메라와 LiDAR

- B0390은 IMX219, 15-pin FPC 커넥터. Jetson Orin Nano CAM0/J20은 22-pin, 0.5 mm. 제조된 15↔22 FPC 사용.
- CAM0는 2-lane. 케이블 접점 방향은 Jetson carrier PCB 쪽으로 맞추고 잠금쇠를 닫는다. 카메라 쪽은 실물 케이블/커넥터 접점으로 확인한다.
- C1은 Jetson USB-A에 연결하고 VND018과 다른 USB 스택을 사용한다.
- 카메라/라이다 인식 후 Jetson에서 동시에 스트리밍해 USB 재연결·CSI 에러·프레임 누락·CPU/GPU 부하를 기록한다.

## 5. 무전원 검사와 첫 통전

### 5.1 무전원 체크리스트

- [ ] 배터리, USB, 벤치 전원 모두 분리
- [ ] 배터리 XT60→F0→S0→Y선 양극 연속성 확인
- [ ] J01/M01 입력 +/− 단락 없음
- [ ] 각 PDB 출력 케이블의 +/− 극성 확인
- [ ] F0/FJ/FM/FS/FE 퓨즈 값과 위치 확인
- [ ] Jetson DC 플러그 중앙 +/외곽 − 확인
- [ ] PGE XHP-4 pin1~4와 맨선 도통표 작성
- [ ] CF-0108 VA/VB/GND/OE/A/B 실크 및 IC 식별
- [ ] U2 SOT-23-6 pin1 방향과 pin별 도통 확인
- [ ] IMU VIN/GND/SPI/CS/INT1 연결 확인
- [ ] Adapter A H1/H2/H3 pin 번호와 D/V/G 케이블 연속성 확인
- [ ] MD20A VB+/VB− 극성 확인
- [ ] MA/MB가 GND/전원 레일에 단락되지 않았는지 확인
- [ ] PWM/DIR pulldown, STOP 접점 정상 상태 확인
- [ ] 신호 GND와 전력 귀환선을 구분

### 5.2 첫 통전 순서

1. 모터·서보·센서 부하를 모두 분리하고 PDB만 통전한다.
2. 24.0–25.2 V 입력에서 J01 CH1=19 V, CH2=5 V, M01 CH1=입력 pass-through, M01 CH5=12 V를 측정한다.
3. 각 출력의 무부하 전압, 배터리 전압, 전류, 발열을 기록한다.
4. Jetson만 J01 CH1에 연결해 부팅한다. 19 V 최저값과 부팅 반복을 확인한다.
5. Jetson USB-A→VND018 CH343P USB-C를 연결한다. VND018 부팅, COM/tty 장치, USB 재연결을 확인한다.
6. C1을 다른 USB 스택에 추가하고 카메라를 CAM0에 연결한다. 장치별 인식과 동시 구동을 확인한다.
7. J01 CH2 FE 퓨즈 뒤 5 V를 측정하고 PGE/CF를 연결한다. 3V3/5V 레일이 유지되는지 확인한다.
8. IMU를 추가하고 SPI 통신을 확인한다.
9. 서보 전원을 마지막에 추가한다. 낮은 부하에서 ID/baud/위치 읽기와 작은 이동만 확인한다.
10. 모터 전원은 바퀴를 띄운 상태에서 마지막에 추가한다.

전류 측정기를 배터리 +/−에 병렬로 직접 대지 않는다. 전류계는 부하와 직렬로 연결하거나 DC 클램프를 사용한다.

## 6. ESP32 프로그램 이관

### 6.1 프로젝트 기본값

- ESP-IDF v6.1, target=esp32s3.
- VND018 N16R8의 Flash/PSRAM은 실물 부팅 로그와 IDF 인식으로 확인한다.
- 기존 portable core는 프로토콜/수신 상태기계 참조 구현이다. firmware_bench는 출력 비활성 UART 시험 앱이며 PCNT/SPI/LEDC/서보/STOP 드라이버가 완성된 실차 펌웨어가 아니다.
- 하드웨어 초기화 전 PWM=0, DIR 기본값, S_RUN OPEN, DISARMED.
- 펌웨어를 먼저 UART/센서 입력만으로 시험하고, 구동 출력은 마지막에 활성화한다.

### 6.2 peripheral 할당

| 기능 | ESP-IDF peripheral | 핀 |
|---|---|---|
| Jetson host link | UART1 via GPIO Matrix | TX=GPIO43, RX=GPIO44 (CH343P 물리 연결) |
| Servo bus | UART2 | TX=GPIO17, RX=GPIO18 (U2 RX buffer) |
| IMU | SPI2 | SCLK12, MOSI11, MISO13, CS10, INT1 9 |
| Encoder | PCNT quadrature | A=GPIO4, B=GPIO5 |
| Motor | LEDC PWM + GPIO DIR | PWM15, DIR6 |
| CF-0108 OE | GPIO output | GPIO14, LOW default, HIGH after VA/VB stable |
| STOP status | GPIO input | GPIO8 |

UART1을 GPIO43/44에 배정하는 이유는 CH343P가 물리적으로 이 두 핀에 연결되어 있기 때문이다. UART0는 ROM/부트 용도로 남긴다. 앱/진단 콘솔은 Native USB Serial/JTAG(GPIO19/20)로 이동한다. ROM 부팅 문자가 CH343P에 먼저 나타날 수 있으므로 호스트는 시작 잡음을 버리고 매직+길이+CRC가 유효한 첫 프레임부터 동기화한다. 서보는 UART2를 사용해 Jetson host link와 분리한다.

### 6.3 이관 순서

1. portable wire/command/receiver/link/tx 코어를 ESP-IDF component로 가져오고 Python/ROS 참조 구현과 동일 입력·기대 결과를 비교한다.
2. UART1 host link를 GPIO43/44에 매핑한다. 로그는 Native USB Serial/JTAG로 분리한다.
3. Jetson↔ESP32 프레임 송수신, CRC, 분할/병합, 잘못된 길이, seq 중복/역순, USB 단절/재연결, MCU reset을 시험한다.
4. UART2 servo driver를 GPIO17/18에 구현한다. ID/baud/position readback부터 시험한다.
5. SPI2 IMU driver를 구현하고 WHO_AM_I, ODR, full-scale, INT1, 축 방향을 확인한다.
6. PCNT quadrature driver를 GPIO4/5에 구현한다. glitch filter는 실측 최소 펄스 폭보다 짧게 설정한다.
7. LEDC PWM + DIR driver를 GPIO15/6에 구현한다. PWM LOW=brake 의미를 상태기계에 반영한다.
8. GPIO14로 CF-0108 OE를 제어한다. 전원 안정 전 LOW, 안정 후 HIGH.
9. GPIO8 STOP 입력과 물리 2NC 접점을 구현한다. S_RUN OPEN/STOP latch/CLEAR/ARM 경계를 분리한다.
10. 모든 고장 상태에서 새 ARM 전까지 PWM=0, DISARMED를 유지한다.

## 7. 센서·구동 교정 절차

### 7.1 엔코더 count/rev

- PGE-213 판매 사양은 7 PPR이다. 쿼드러처 x4를 적용하면 이론상 encoder shaft 1회전당 28 count가 후보값이다.
- 모터축에 표시선을 긋고 정확히 N회전시킨 뒤 PCNT 변화량을 기록한다. 정방향/역방향 반복.
- 실측 count/motor rev와 28 count/rev 가정을 비교한다. 다르면 PPR 정의, 출력 edge, PCNT mode, 신호 누락을 확인한다.
- 실측 기어비는 모터축 N회전 대비 바퀴 회전수로 구한다. 공칭 8.9706:1은 실측 전 가정값으로만 둔다.
- 실제 유효 바퀴 둘레는 바닥에서 정해진 거리만큼 굴려 교정한다.
- distance/count = effective wheel circumference / counts per wheel revolution.
- 단일 모터축 엔코더 하나로 좌우 바퀴 개별 속도, 차동기 양쪽 회전, 차체 정지, 슬립을 관측할 수 없다.

### 7.2 IMU

- 차체를 수평/정지 상태로 두고 30~60초 raw gyro/accel을 기록한다.
- gyro bias 평균과 표준편차, accel 중력축, 장착 회전을 기록한다.
- yaw rate 부호가 실제 좌회전/우회전과 일치하는지 수동 회전으로 확인한다.
- 온도/전원 재인가 후 bias 변화를 비교한다. 지자기 센서는 이 보드에 없으므로 절대 yaw 보정으로 해석하지 않는다.

### 7.3 조향각

- 서보 horn/링크 분리 상태에서 서보 ID, baud, 현재 position을 읽는다.
- 중립을 정하고 horn을 장착한 뒤 타이어가 직진하도록 링크 길이/장착각을 조정한다.
- 서보 명령값을 작은 간격으로 바꾸면서 좌우 바퀴각을 각각 측정한다.
- raw servo position ↔ 좌/우 wheel angle 표를 만들고 중앙 오차, 좌우 비대칭, 유격, 히스테리시스를 기록한다.
- 소프트웨어 min/max는 기계적 간섭 전에서 제한한다. ST3215-HS 끝단 구속 금지.
- 실제 조향 피크전류와 M01 CH5 12 V 최저전압을 기록한다. 2 A에 근접하거나 넘으면 중단.

## 8. 단계별 시험 게이트

| 게이트 | 작업 | 통과 기준/증거 |
|---|---|---|
| H0 | 부품/CF/VND 판본 확인 | 사진, IC top-mark, P1/P2 실크, 커넥터 기록 |
| H1 | 배터리 하네스·퓨즈·스위치 | 극성/도통/단락 체크시트 |
| H2 | PDB 무부하 통전 | J01 19/5 V, M01 pass-through/12 V 측정 |
| H3 | ESP32 USB/부팅 | 부팅 로그, Flash/PSRAM, CH343P 연결 |
| H4 | Jetson host UART | UART1 GPIO43/44 프레임/CRC, reset/재연결 |
| H5 | IMU | WHO_AM_I, 축, bias, timestamp |
| H6 | Encoder | CF 5→3.3 V 파형, PCNT 방향/count/rev |
| H7 | Servo | ID/baud/readback, 실제 좌우 wheel angle, 전류 |
| H8 | MD20A | PWM/DIR, 모터 방향, 전류/온도, brake 동작 |
| H9 | STOP/고장 주입 | 버튼/단선/USB 분리/MCU reset 후 자동 재출발 없음 |
| H10 | 동시 구동 | C1+카메라+IMU+encoder+servo, CPU/메모리/지연 |
| H11 | 저속 실차 | 직진/원호/직각 코너/정지거리 기록 |

모든 게이트는 실제 로그·계측값·사진을 남긴 뒤 통과 처리한다. 문서 작성/코드 빌드/ROS 토픽 발행만으로 실물 게이트를 통과 처리하지 않는다.

## 9. 최종 실물 확인 항목

아래는 설계가 비어 있는 항목이 아니라 실물 부품을 받아 판정해야 하는 항목이다.

1. CF-0108 내부 IC와 VA/VB/GND/OE/A/B 실크가 R2 표준 TXS0108E 배선과 일치하는지.
2. PGE-213 XHP-4 pin 1~4와 보유 케이블 맨선 도통.
3. VND018 실물 P1/P2 헤더의 연속성과 보드 판본.
4. B0390 15-pin↔22-pin FPC 접점 방향과 CAM0 인식/스트리밍.
5. M01 입력 24 V 미만에서 12 V CH5가 유지되는지.
6. 정상 조향에서 서보 피크전류가 M01 CH5 2 A 공급 범위 안에 있는지.
7. STOP 버튼의 NC 2접점 동작, PWM 억제, 정지거리.
8. 실측 encoder count/rev, gear ratio, wheel circumference, IMU 축 방향.

