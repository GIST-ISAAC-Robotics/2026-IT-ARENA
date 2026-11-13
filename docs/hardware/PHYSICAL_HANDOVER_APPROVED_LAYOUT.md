# IT ARENA 실물 이관·조립·시험 절차 — 승인 구성 기준

- 기준: f3a2f7c031a52a52b4166c2df91d4377066c3654
- 전장 배치 기준: [APPROVED_ELECTRICAL_LAYOUT.md](APPROVED_ELECTRICAL_LAYOUT.md)
- 핀별 조립 기준: [ESP32 주변 통합 배선도 v1](ESP32_GPIO_WIRING_2026_11_11.md). IMU 배선 설계를 유지하며 CF 부분은 조건부다. [11/12 서보 제한 실측](../sensors/ST3215_HS_HARDWARE_2026_11_12.md)은 완료했고 모터·엔코더·서보 실장 교정은 후속이다. 서보 단독 시험 당시에는 IMU가 분리돼 있었으나, 11/12 후속 r249/r251에서 실제 IMU와 C1·카메라의 공통 노드 수신을 확인했다([현행 통합 상태](../reports/RUNTIME_SENSOR_INTEGRATION_2026_11_12.md)). 당시 분리 기록을 현재 연결 상태로 적용하지 않는다. 보유 ESP32는 제조사 미확인 호환 보드이므로 아래 구매명 VND018/CH343P를 실물 판본 확정으로 읽지 않는다.
- 목적: 보유/선정 부품을 그대로 사용해 전원 연결, 센서 연결, ESP32 프로그램 이관, Jetson 통합 및 교정을 진행한다.
- 이 절차는 추가 회로 제작을 전제로 하지 않는다. 실물에서 확인되지 않은 핀·전압·신호 방식을 추측해 연결하지 않는다.

## 1. 준비물과 보유품 우선 원칙

기존 구매 목록과 보유 케이블을 먼저 사용한다.

- J01/M01 지급 전원 보드와 6S 배터리
- Jetson 전용 19 V 전원선
- Jetson USB-A ↔ VND018 CH343P USB-C 데이터 케이블
- IMU용 점퍼선
- J01 CH4 5 V용 XT30 수 리드 및 PGE-213 보유 4핀 커넥터 케이블
- CF-0108
- M01 CH1 ↔ MD20A 전력선 및 MD20A ↔ 모터선
- M01 12 V 출력 ↔ Servo Adapter A 전원 케이블
- Servo Adapter A ↔ ST3215-HS 서보 케이블
- B0390 22핀 ↔ Jetson 22핀 FPC(11/10 사용자 실물 확인·연결)
- C1 USB 어댑터/케이블

추가 저항·콘덴서·버퍼 IC·DC-DC 모듈을 기본 준비물로 요구하지 않는다. 특정 부품이 실제 모듈 사양상 필요하다는 근거가 확인된 경우에만 구매 목록에 추가한다.

## 2. 조립 전 실물 식별

1. Jetson 전원 잭 규격과 극성, J01 CH1 출력 라벨을 확인한다.
2. 11/11 실측 통신된 호환 ESP32의 USB-UART 포트를 사용한다. YD 보드의 USB 좌우 위치나 CH343P 형번을 납품품에 단정하지 않는다.
3. Adafruit 4438의 하단 VIN/GND/SCL/SDA/DO/CS/I1 실크를 확인한다. 현재 배선은 유지한다.
4. PGE-213 XHP-4는 제조사 자료상 pin 1=VCC, 2=GND, 3=HA, 4=HB다. 보유 케이블 맨선과 각 핀의 도통을 측정하고 선 색상만으로 판단하지 않는다. 모터 단자 pin 5/6과 엔코더 신호 핀을 혼동하지 않는다.
5. J01 CH4는 5 V / 0.5 A 출력이며 현재 ESP32는 Jetson USB로 전원을 받으므로 CH4를 엔코더 전용으로 사용한다. J01 출력은 XT30 암이므로 보유 XT30 수 리드를 준비한다.
5. CF-0108은 11/12 사용자 전달 YF08E 마킹으로 TXS0108E 식별을 반영했다. [확인 범위](APPROVED_ELECTRICAL_LAYOUT.md#4-pge-213-엔코더와-cf-0108)에 따라 패드 단락·VA/VB·OE·실제 채널 대응을 확인한다. 신규 CF 사진은 B1~B8이 정상 순서로 보인다. B2 누락·B4 중복은 별도의 기존 헤더 보드다. 두 실물을 혼동하지 않고 도통 확인 후 통전한다.
6. Adapter A의 전원 입력, 서보 커넥터, UART 커넥터, 점퍼 위치를 확인한다.
7. MD20A의 VB+/VB−/MA/MB/PWM/DIR/GND 실크를 확인한다.
8. B0390의 카메라 쪽 22핀과 22↔22 FPC 연결은 11/10 사용자 확인 사항이다. 기본 영상 수신은 [실물 시험](../sensors/B0390_HARDWARE_2026_11_10.md)으로 확인했으며, 포트 실크·최종 접점 방향과 광학/인식은 별도 대조한다.

## 3. 전원 배선

모든 전원을 분리한 상태에서 시작한다.

1. J01 CH1의 19 V 출력을 Jetson DC 5.5×2.5 mm 입력으로 연결한다. 중앙 + 극성을 측정한다.
2. J01 CH4의 5 V / 0.5 A 출력을 XT30 수 리드로 엔코더 전원에 배정한다. CH4 +는 PGE-213 XHP-4 pin 1(VCC), CH4 −는 pin 2(GND)로 연결한다. Jetson J12 5 V는 사용하지 않는다.
3. M01 CH1 출력을 MD20A VB+/VB−로 연결한다. 극성을 두 번 확인한다.
4. MD20A MA/MB를 모터 두 단자에 연결한다. 모터선을 신호선이나 엔코더 보드 단자에 연결하지 않는다.
4. M01 CH5 또는 CH6 중 한 채널만 선택해 12 V를 Adapter A 전원 입력으로 연결한다. 두 채널을 병렬 연결하지 않는다.
6. Adapter A의 서보 출력 포트에 ST3215-HS 한 개를 연결한다. D/V/G의 실제 케이블 대응을 확인한다.
7. Jetson USB-A와 VND018의 CH343P USB-UART USB-C를 연결한다. 이 케이블로 ESP32에 전원과 통신이 함께 제공된다.
8. Jetson USB-A의 다른 포트에 C1 USB 어댑터를 연결한다.

## 4. 무전원 검사

- [ ] 배터리와 Jetson 전원, USB 케이블을 모두 분리했다.
- [ ] Jetson 전원선의 중앙 + / 외곽 −를 확인했다.
- [ ] M01 CH1 → MD20A VB+/VB− 극성을 확인했다.
- [ ] MD20A MA/MB가 전원/GND와 단락되지 않았는지 확인했다.
- [ ] M01 12 V 채널은 한 채널만 Adapter A에 연결했다.
- [ ] Adapter A와 서보 사이 D/V/G 케이블 대응을 확인했다.
- [ ] PGE-213 커넥터의 각 핀과 보유 케이블 선을 도통표로 기록했다.
- [ ] J01 CH4 5 V / 0.5 A XT30 출력의 극성을 확인하고 엔코더 전원으로 배정했다.
- [ ] PGE-213 pin 2 GND와 ESP32/CF-0108 신호 GND 사이의 공통 기준을 확인했다.
- [ ] CF-0108 실물 IC와 보드 실크를 기록했다.
- [ ] IMU 전원/신호 핀을 실크와 대조했다.
- [ ] 카메라 FPC의 핀 수와 접점 방향을 확인했다.

## 5. 단계별 첫 통전

각 단계에서 이상 발열, 냄새, 재부팅, 전압 강하가 나타나면 즉시 전원을 끄고 원인을 확인한다.

1. PDB만 연결하고 J01 CH1 19 V, M01 CH1 출력, M01 선택 12 V 채널을 측정한다.
2. Jetson만 연결해 정상 부팅을 확인한다.
3. Jetson USB-A ↔ VND018 CH343P USB-C를 연결해 ESP32 부팅 및 USB 장치 인식을 확인한다.
4. IMU를 ESP32 3V3/GND/SPI 핀에 연결하고 WHO_AM_I 및 raw data를 읽는다.
5. J01 CH4의 5 V / 0.5 A 출력을 측정하고 극성을 확인한 뒤 PGE-213 pin 1(VCC)/pin 2(GND)에 연결한다. Jetson J12는 엔코더 전원으로 사용하지 않는다. 엔코더 신호 A/B는 아직 ESP32에 연결하지 않는다.
6. 사용자가 제공한 PGE-213 상세 회로도에는 HA/HB에 VCC 방향 4.7 kΩ 저항, 각 출력의 470 Ω 직렬 저항 및 10 nF 커패시터가 표시되어 있다. 11/12 제조사 답변 전달에서 NPN open-collector·내부 풀업이 확인됐으므로 기존 5 V→3.3 V 변환 경로를 유지한다.
7. CF-0108의 TXS0108E 마킹 식별은 완료됐다. 패드/전원/OE 확인 후 MCU GPIO를 분리한 상태로 실제 HIGH/LOW·상승/하강과 PGE 출력 10 nF 부하에서의 파형을 확인한다.
8. CF-0108 실물 확인 전에는 A/B를 ESP32에 연결하지 않는다. GPIO4/5는 엔코더 신호용 후보이며, VND018의 물리 헤더 번호와 CF-0108의 실제 패드 번호는 아직 확정되지 않았다.
9. 서보는 후속 드라이버 준비 후 GPIO17(TX)→Adapter A H2 TX/TXD, GPIO18(RX)←H2 RX/RXD와 GND, 점퍼 A로 연결한다. 11/12 격리 앱으로 ID1/1 Mbps·현재 위치·작은 왕복과 토크 해제를 확인했으며, 장착 뒤 다시 시험한다. 점퍼캡은 두 개 모두 A다. 기존 IMU 진단 펌웨어로 복구했으므로 현재 앱에는 서보 제어가 없다.
10. 모터는 바퀴를 공중에 띄운 상태에서 마지막으로 연결한다. 낮은 PWM으로 회전 방향을 확인한다.

## 6. ESP32 프로그램 이관

11/13 현재 `firmware_runtime_drive`에는 IMU·PCNT 엔코더·서보 UART·MD20A PWM/DIR·GPIO8 STOP의 HAL과 Jetson runtime host가 연결되어 있다. 이 구현/빌드와 실제 설치·장착 동작은 구별한다. 현재 장치에 설치된 앱을 확인하지 않고 runtime firmware라고 가정하지 않는다. [운영 진입 감사](../../artifacts/validation/2026-11-13/runtime_entry_audit_r415_v1/REVIEW.md)의 근거를 따른다.

권장 순서:
1. 설치 앱·빌드/프로필 계보를 확인하고 기존 펌웨어와 통신 형식을 보존한다. 기본 `installed_board_profile()`은 미교정 비활성 상태이므로 빌드 성공만으로 출력 준비가 된 것은 아니다.
2. 조립 후 측정한 서보 영점·좌우 한계/방향과 기존 계획의 모터/엔코더 방향·출력 설정을 실장 프로필로 연결한다. [r416 빌드 연결](../../artifacts/validation/2026-11-13/runtime_installed_profile_r416_v1/RESULT.md)의 `runtime_installed_profile.py prepare`와 `build_runtime_drive.sh --installed-profile DIR`로 명시 전달한다. 기존 `export-cpp`만으로 자동 적용되지는 않는다. 실제 영점/출력값은 조립 뒤 확인하며 합성 검사 프로필은 설치하지 않는다.
3. runtime UART는 host 하나만 소유한다. host의 `/hardware_runtime/imu`와 `/drive_motor/encoder`를 사용하며 예전 preview IMU bridge를 같은 포트에서 함께 실행하지 않는다.
4. 이미 완료한 개별 센서 수신·손 회전·무부하 서보 검사를 반복하지 않는다. 장착 후 달라지는 축/부호·직진 중립·링크 간섭과 실제 출력만 아래 최소 확인 순서로 검사한다.
5. 실제 최초 부팅/리셋/통신 오류·물리 STOP에서 출력이 멈추는지 제한된 설치 확인을 수행한다. 기존 소프트웨어 회귀와 비구동 ROS/가짜 MCU의 STOP ACK를 물리 정지 증거로 바꾸지 않는다.

## 7. 센서·구동 교정

아래는 일반 교정 항목이다. 이번 조립 직후에는 뒤의 **시간 제한형 실물 확인**을 우선하며, 이미 확인한 센서 수신·손 회전·배선 검사를 반복하지 않는다. count/rev 정밀 재측정은 방향/환산에 이상 근거가 있을 때만 추가한다.

### 엔코더
- 제조사 자료 또는 계측으로 A/B 출력 회로와 전압을 확정한다. 제공된 회로도에 4.7 kΩ VCC pull-up이 보이므로 오픈 컬렉터라는 이유만으로 3.3 V 직접 입력을 가정하지 않는다.
- A/B 위상, 방향 부호, 실제 count/rev를 측정한다.
- PGE-213 판매 페이지의 7 PPR 표기만으로 PCNT의 x1/x2/x4 계수를 확정하지 않는다.
- 모터축을 정해진 횟수만큼 회전시키고 실제 누적 카운트를 기록한다.

### IMU
- 정지 상태에서 raw gyro/accel을 기록한다.
- 차체 좌표계에 맞춰 축 방향과 부호를 확인한다.
- 정지 바이어스와 장착 각도를 기록한다.

### 조향 서보
- 바퀴/링크를 연결하기 전 서보 ID와 현재 위치를 읽는다.
- 중립 명령에서 타이어가 직진하도록 horn과 링크를 맞춘다.
- 서보 명령을 작은 간격으로 변경하며 실제 좌우 바퀴 조향각을 각도계로 측정한다.
- 좌우 각도, 중앙 오차, 유격, 기계적 한계를 표로 기록한다.
- 기계적 끝단에 계속 밀어붙이지 않는다. M01 12 V/2 A 채널의 실제 조향 전류와 전압 강하를 측정한다.

### 모터
- 바퀴를 공중에 띄운 상태에서 낮은 PWM으로 방향을 확인한다.
- MD20A PWM LOW의 brake 동작을 확인한다.
- 전류와 드라이버 온도를 기록한다.

## 8. Jetson 통합 확인

- Jetson 부팅과 전원 안정성
- ESP32 USB-UART 장치 인식 및 통신
- IMU 데이터 수신과 timestamp
- 엔코더 카운트와 방향
- Servo Adapter A UART 응답 및 실제 조향각
- MD20A 방향/PWM/정지 동작
- C1 USB 인식 및 scan 데이터
- B0390 CSI 인식 및 영상 스트리밍
- 카메라·LiDAR·센서·제어 노드 동시 실행 시 전원/연산 안정성

각 항목은 로그·측정값을 남긴 뒤 통과 처리한다. 문서 작성이나 코드 빌드만으로 실물 시험 통과로 표시하지 않는다.

## 시간 제한형 실물 확인 — 이미 끝난 시험은 반복하지 않음

**11/14 실행 절차:** 차량 도착 후에는 [60cm 사각 벽 공간의 측정·보정·적용 순서](ASSEMBLED_CAR_BOX_CHECK_2026_11_14.md)를 먼저 따른다. 기둥 위치와 전선 두께의 실측, 기존 여유에 움직임 여유를 더한 자기 가림 설정, IMU 편향·마커 거리·서보 영점의 적용/재확인까지 포함한다. 자기 가림도 사각 벽 안에서 측정하며, 거리/부품 위치와 필요 시 몇 cm 재배치 기록으로 외부 벽을 구분한다. 열린 공간은 요구하지 않는다. 아래 기존 항목은 이 구체 절차의 배경이며 완료된 시험을 새로 반복하라는 지시가 아니다.

차량 완성 최신 예상은11/14 02:00경(미확정)이고05:00 출발 전 실물 시험 시간이 매우 짧다. 이 절은 기존 전기적 조립/통전 절차를 대체하지 않으며, 배선 담당자는 승인 전장도와 위 단계별 통전 절차를 따른다. 별도 연습 트랙은 제작하지 않는다.

1. **기존 시험 재사용:** C1/카메라/IMU 단독 수신, IMU 원시 수신, 엔코더 손 회전, 서보 무부하 왕복, 배선/커넥터 확인은 이미 기록이 있으므로 동일 조건으로 반복하지 않는다.
2. **4방향 벽 확인:** 작은 벽 공간에서 차량을 네 방향으로 놓고 C1 벽 반환과 방향, IMU yaw 변화 부호를 확인한다. 짧은 이동/회전에서 엔코더 방향도 함께 대조한다. 벽 직각도나 기준 위치를 모르면 절대 각도·거리 보정으로 주장하지 않는다.
3. **모터 출력:** 바퀴를 공중에 띄우고 낮은 PWM에서 시작해 점진적으로 올린다. 방향·응답·출력 여유와 명백한 진동/발열/전압 이상을 확인한다. 고속 시험도 바퀴를 띄운 상태에서 별도로 수행하고, 지면에서 최고속도를 시험하지 않는다.
4. **서보 장착 후:** 기존 서보 단독 시험을 반복하지 않고, 링크 장착으로 달라지는 중립 직진·실제 좌우 바퀴각·간섭/끝단 여유만 확인한다. 끝단에 지속적으로 밀어붙이지 않는다.
5. **짧은 지면 확인:** 공간이 허용되는 경우 저속 직진·좌우 회전·정지/재출발만 확인한다. 트랙 랩이나 다중 차량 실차 시험은 시도하지 않는다.
6. **엔코더 전원:** 배터리/PDB 연결이 필요한 추가 계수 측정은 조립 후 필요성이 확인될 때만 한다. 기존 손 회전 시험을 반복하지 않는다.

이 시간 제한형 확인은 정밀한 교정 전수 절차가 아니다. 눈에 띄는 방향 오류·출력 부족·기구 간섭처럼 실제 주행을 막는 문제를 먼저 찾아 수정하는 목적이다. 공식 코스는 대회장 연습 시간에만 확인한다.

### 설치 전 소프트웨어 인계에서 남은 항목

- r430에서 영점 export와 MCU에 같은 선택 `command_limit`·축거·윤거·바퀴각 한계를 전달하도록 수정했다. 축소 실장 한계도 검사하며 실제 count 범위를 넓혀 통과시키지 않는다. 관련45개 검사 요구를44통과 뒤 누락 시험 의존성 보완/1개 재검사로 충족했고 Python/C++ 양방향14표본을 대조했다. 새 IDF 빌드·flash·실물 검증은 아니다. [통합 결과](../reports/RUNTIME_SENSOR_INTEGRATION_2026_11_12.md#11-13-panel-steering-integration).
- r439의 [`runtime_installed_limits.py`](../../scripts/runtime_installed_limits.py)를 공통 이관했다. 실제 선택한 설치 bundle과 운영 YAML을 입력하면 planner·guard 두 조향 상한과 host 인자를 함께 생성한다. MCU 원상한을 바꾸지 않고 float32 송신값을 하향 정규화하며, 기존 YAML의 더 작은 상한을 자동 확대하지 않는다. 두 ROS 노드는 생성한 같은 params-file을, host는 생성한 인자를 사용해야 한다. 수동 패널은 별도로 `manual_steering_limit_rad:=min(0.2, 생성 상한)`을 전달한다. Windows11검사·r401 전체 parameter 보존 대조를 마쳤으며 실장값 선택/실제 launch/flash는 아직이다. [명령과 검증 범위](../../artifacts/validation/2026-11-13/installed_limit_chain_r439_v1/RESULT.md) · [이관 영수증](../../artifacts/validation/2026-11-13/installed_limit_chain_r439_v1/integration.json).
- 명령 만료/물리 STOP 처리와 MCU 루프 자체가 멈출 때의 출력 상태는 별개다. r432에서 이전 r416 생성 설정의 interrupt watchdog800ms·task watchdog5s/idle task 감시를 확인했다. 앱 제어 루프 자체의 명시적 task watchdog 등록/급식은 코드에서 확인되지 않아 루프 고장 시 독립 출력 차단으로 해석하지 않는다. 실제 출력 정지는 기존 최소 STOP 확인에 포함한다. 이 문서만으로 MCU 고장 시 독립 차단을 확인했다고 하지 않는다.

## 9. 이 문서에서 확정하지 않는 것

- CF-0108의 실제 패드 배열·정상 동작(YF08E 마킹에 의한 TXS0108E 식별은 11/12 반영)
- J01 CH4의 실제 5 V 출력 및 XT30 극성, PGE-213 전원 귀환과 ESP32 신호 GND 공통 기준
- PGE-213의 A/B 출력 회로와 커넥터 핀 배열
- 엔코더 실제 HIGH/LOW·변환 파형·최종 채널 연결(NPN open-collector·내부 풀업은 11/12 제조사 답변 전달로 확인)
- Adapter A UART 커넥터의 실제 보드 실크와 서보 케이블 핀 배열
- 6S 배터리 전압이 24 V 아래일 때 M01의 안정 동작
- 실제 서보 조향 전류와 전압 강하
- 물리 STOP의 설치 상태·실제 출력 정지·규정 인정. GPIO8 입력을 처리하는 runtime 펌웨어 코드는 구현되어 있으나 장착 동작과 구별한다. **11/13 사용자 지시의 결선 설계는 GPIO8↔2선 유지형 스위치↔TGND로 반영 완료**했으며 [통합 배선도](schematics/esp32_gpio_wiring_v1.html)를 따른다.

위 항목은 임의 회로를 추가해 덮지 않는다. 제조사 자료나 실물 측정으로 확인한 뒤 필요한 최소한의 연결만 추가한다.

## 조립 전 관측 실행

**11/13 후속 운영 명령 연결:** [설치 상한·실물 추정 입력·패널·로그를 합친 명령 조합](../../artifacts/validation/2026-11-13/operator_handover_r453_v1/MINIMAL_HANDOVER.md)을 코드와 대조했다. 실물 실행 전의 명령안이며 설치 bundle/영점/출력값을 대신 생성하지 않는다. 기존 관측 후보OFF 구성은 유지할 수 있다. estimated 후보를 명시 선택할 때는 생성한 같은 YAML/host 상한을 유지하고, planner와 guard 양쪽에 `host_estimated_crop_v1`/crop를, G1에 `r396_v1`을 함께 지정한다. `corner=off`에 scope만 붙이는 조합은 현재 validator가 거절한다. `timing_verified=false`와 G1 전용 진입을 유지한다. 수동 로그는 자율 사건 시각이 없으므로 전체 export와 단발/host 원자료를 함께 회수한다. [대조 근거와 미확정 입력](../../artifacts/validation/2026-11-13/operator_handover_r453_v1/REVIEW.md).

아래는 개별 관측을 위한 preview이며 최종 형상+B3+출력 host의 운영 실행 명령이 아니다. 최종 profile을 받는 planner/guard와 host의 세 명령, 필수 clock JSON·motor feedback JSON·포트 단일 소유는 [운영 진입 템플릿](../../artifacts/validation/2026-11-13/runtime_entry_audit_r415_v1/REVIEW.md#운영-진입-명령-3개)을 따른다. 템플릿은 실제 실차 송신을 실행한 결과가 아니며, `--enable-drive-wire` 없는 host는 출력 전송이 차단된다. 실물 추정시각을 verified로 바꾸지 않는다. 원 r415 템플릿의 `g1_current_corner_budget=off`는 그대로 보존한다. 후속 r444는 false/host_estimated를 유지하는 별도 명시 scope를 준비했으며, 기존 템플릿에 r396만 단독 추가해서는 안 된다. 선택 조합과 검증 한계는 [r444 결과](../../artifacts/validation/2026-11-13/estimated_observation_r444_v1/RESULT.md)를 따른다. 실제 선택 profile/실물 주행은 아직 미확인이다.

`scripts/run_hardware_sensor_preview.py`는 실제 C1·IMU·선택 B0390을 공통 비구동 노드에 연결하고 기록/종료를 묶는다. 펌웨어 업로드·ARM·DRIVE·가상 엔코더 입력은 하지 않는다. 원 `imu_probe` 앱은 `--imu-source probe`(기본), 출력 없는 통합 센서 앱은 `--imu-source sensor-link`를 명시한다. 앱 종류를 자동 변경하지 않는다. 카메라 미선택 시 영상 발행/처리 프로세스는 실행하지 않는다.

Jetson의 해당 소스 묶음 루트에서 ROS Humble 및 Python 의존성을 불러온 뒤 다음처럼 실행한다. 포트는 `ls -l /dev/serial/by-id/`로 확인한 현재 장치 경로, SDK 경로/해시는 검증한 실행 파일로 지정한다. 아래 대괄호 부분은 실제 값으로 바꾼다.

```bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=216
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:${PYTHONPATH:-}"
python3 scripts/run_hardware_sensor_preview.py \
  --output /tmp/arena-input-check-new \
  --imu-port '[IMU의 by-id 경로]' --c1-port '[C1의 by-id 경로]' \
  --c1-executable '[검증한 SDK 실행 파일]' --c1-sha256 '[해당 SHA256]' \
  --seconds 20 --camera --plan-only
```

`--plan-only`는 장치/ROS를 열지 않고 명령만 출력한다. 확인 후 이를 빼면 실제 관측을 시작한다. 출력 폴더가 이미 존재하거나 포트가 점유된 경우 중단하며 다른 작업을 강제 종료하지 않는다. 최초 검증은 [r280 결과](../../artifacts/validation/2026-11-12/reusable_preview_r280_v1/README.md)에 있다. FIFO/tracking 시각은 미교정 추정이며 엔코더·장착 축·구동 HAL·조향/제동 교정은 여전히 별도로 필요하다.
