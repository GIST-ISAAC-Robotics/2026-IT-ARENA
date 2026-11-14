# 추가 부품 없는 Jetson 대체안 · 검토와 실측 대기

2026-11-14. 사용자는 **현장에 추가 부품이 없다**고 확인했다. 최신 지시는 **물리STOP 제외, 조향·DC·엔코더·IMU·카메라·라이다 모두 유지**다. Jetson은 사용자 확인상 전원OFF이며 이번 실물 검증은 수행하지 못했다. 검토하던 watchdog·reset·latch·입출력 버퍼 추가안은 현재 실행안에서 제외했다. [기존 부품 배선안 v1](jetson_direct_wiring_v1.html)은 연결 후보이며 완성·통전 승인이 아니다. 기존 ESP32 도면은 보존한다.

## 현재 판단

**판정: 안정성 확인 미완료, 자동주행용 대체 제어기 준비 안 됨.** 자료상 가능한 경로와 실제 전압·파형·정지 확인은 다르다. 보유 부품으로 먼저 검토할 범위는 모터 전원 분리 상태의 조향 통신과 무출력 계측이다. 후속 사용자가 어댑터 USB-C와 C-to-C 보유를 알려 조향은 USB 제어 경로로 변경했다. J12 UART를 사용하지 않아 해당 풀업 문제는 현재 조향 경로에서 제외된다. 현재 장비에 새 배선·pinmux·출력을 적용하지 않았다.

| 항목 | 자료/구조 검토 | 실제 확인 | 다음 판정에 필요한 것 |
|---|---|---|---|
| PDB 전원 | 기존 J01/M01 채널·정격 유지 | 새 전체 하네스 정상 미확인 | 단락·극성·출력 전압을 기술자가 분리 계측 |
| Jetson 핀 | 신호6핀·전원/GND4핀 중복 없음; 조향은별도USB | 실제 mux·파형 미확인 | CSI를 보존한 설정 확인, 무부하 출력 측정 |
| 서보 USB | Jetson USB-C Host→데이터C-to-C→Adapter(A), 점퍼2개B; GPIO UART 미사용 | 사용자USB-C 보유 정정; USB 등록/직접 통신0회 | 외부 모터·서보 전원 분리 상태에서 USB 등록부터 확인 |
| MD20A | 제조사상3.3V 지원 | Jetson 출력·부팅/종료·정지 미확인 | 모터 무전원에서 LOW·PWM·종료 확인 |
| 엔코더/CF | TXS 내부풀업·수백µA 지속 공급으로TXB ±2mA 구동 요구 충족 근거 없음 | Jetson 손 회전0회 | CF→J12 직결을 확정 배선으로 채택하지 않음 |
| IMU I2C | BOARD3/5, VIN/하단CS=3.3V, DO 미연결 | 통신·시각·축 처리 미구현/미시험 | 실제 주소0x6A/0x6B·단위·취득 시각 확인 |
| 물리 STOP | 사용자 명시적 제외 | 연결하지 않음 | 이번 완성 조건에서 제외; 일반 명령 종료 처리는 별도 유지 |

Jetson의 소프트웨어 STOP·시간 제한은 Jetson 자체 정지와 독립적이지 않다. 스위치 정격을 모르는 상태에서 기존2선 STOP을 모터 전력선에 넣지 않는다. PWM 출력선을 접지에 단락시키는 방법도 사용하지 않는다.

MD20A 공식 데이터시트 §4 Table3은 **PWM LOW=Brake**만 명시하며, PWM 입력 무연결 상태의 LOW·정지 보장이나 내부 pulldown 존재/값은 확인되지 않았다. 따라서 **2선 스위치를 PWM에 직렬 삽입하여 입력을 끊는 안도 채택하지 않는다.** 내부 pulldown이 없다고 확정한 것은 아니다. 자체 시험 버튼은 최대 출력 구동이므로 저속 시험 대안으로 삼지 않는다. [Cytron 공식 연결 데이터시트](https://docs.google.com/document/d/1eKpt66z-1qLMJkaXrHU1AfKqSVeIpEnM43vECpkYuWA/edit?usp=sharing).

## 공식 자료 재검토: 단순 미확인과 조건 불일치 구분

- **Jetson:** NVIDIA SP-11324-001 v1.3 §3.3은J12 신호3.3V 및 I2C 외 외부 풀업/풀다운 **>50kΩ**를 명시한다. 해당UART/GPIO는TXB0108 경유다. [NVIDIA 공식 사양](https://developer.nvidia.com/downloads/assets/embedded/secure/jetson/orin_nano/docs/jetson_orin_nano_devkit_carrier_board_specification_sp.pdf). 직접PDF는403이었고 공식 검색 색인 원문으로 확인했다.
- **서보:** [Waveshare 공식 회로도](https://files.waveshare.com/wiki/Bus_Servo_Adapter_A/Bus_Servo_Adapter_A.pdf)를 시각 검토했다. U3/U4는3.3V지만 A점퍼 경유 외부TXD→U1TXD에 **R4=10kΩ 풀업**과Q1/R6 부하가 연결돼 J12의 풀업 조건과 불일치한다. RX에는 능동 출력U4와R8=10kΩ이 있어 TX와 조건을 구분한다. 이 비교는 **직결 호환 보증 불가**의 근거이며 모든 조건의 통신 실패나 ESP32 발열 원인을 입증하지 않는다. 이후 사용자가USB-C 보유를 정정해 현재는USB 연결을 채택했다. 위 조건 불일치는 미채택한J12 UART 직결안의 검토 기록이다.
- **엔코더:** [TI TXS0108E Rev.L §7.2/7.3.2](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/151/3157.txs0108e.pdf)는 내부 풀업4kΩ(HIGH)/40kΩ(LOW), 지속 공급 수백µA를 설명한다. [TI TXB0108 Rev.K §7.3.2](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/73/txb0108.pdf)는 외부 드라이버 최소±2mA를 요구한다. NPN open-collector→TXS→TXB 경로는 전압만 맞춘 것으로 정합을 보증할 수 없어 현재 직결 미채택이다. 절대 동작 불가라는 단정은 아니다.
- **DC·IMU:** MD20A의3.3V PWM/DIR, Adafruit4438의3.3V 전원/I2C 지원은 자료로 확인 가능하다. 실제Jetson 출력·통신 성공과는 구분한다. [Cytron](https://www.cytron.io/p-20amp-6v-30v-dc-motor-driver), [Adafruit](https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/pinouts).

자료상 호환 조건, 알려진 조건 불일치, 실물 모델 식별, 고장 하네스 상태를 분리한다. 전체 배선도는 서보USB를 포함해 작성할 수 있다. 현재 부품으로 모든 기능이 검증된 대체 제어기는 아니며, 남은 핵심은 엔코더 수신 정합과 실제 출력/종료 검증이다. 물리STOP은 완성 조건에서 제외했다.

USB 근거: [Waveshare USB 제어 및 B점퍼](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example), [NVIDIA USB-C Host 모드](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/hardware_layout.html).

## 가능한 확인 순서

1. 발열 ESP32의 USB·전원·신호 분리를 유지한다. 보드 단독 저항 측정과 외부 하네스 검사를 구분한다.
2. Jetson 헤더와 모터/서보 부하 연결 전에 외부 전원의 극성·전압을 확인한다. 이 문서는 재인가 명령이 아니다. 원인 불명 전원부의 통전은 현장 기술자 측정에 따라 결정한다.
3. 서보USB: USB·외부 전원 분리 상태에서 점퍼2개를B로 옮기고 기존ESP32 UART선을 제거한다. 외부 모터·서보 전원은 뺀 채 데이터C-to-C로 연결해 USB 등록부터 확인한다. USB는 서보12V 구동 전원을 대신하지 않는다.
4. USB 직렬 포트와 직접 통신 프로그램을 확인한 뒤, 하네스/전원 상태가 정상인 경우 서보 전원을 사용해 상태 읽기·토크 OFF부터 확인한다. 연결 직후 중앙 이동을 보내지 않는다.
5. DC는 모터 무전원에서 PWM0·방향·명령 종료를 확인한다. 부팅·종료·brownout의 짧은 펄스는 멀티미터 평균값만으로 확인할 수 없다. 파형 관측 없이 이 항목을 통과 처리하지 않는다.
6. 물리STOP 없이 나머지 기능을 유지하는 조건으로 판정한다. 명령 종료·실행 오류 처리와 실제 엔코더 계수 품질을 검증해야 하며, 이 문서 자체는 구동 명령이 아니다.

물리 확인은 **미수행**이다. 계측값·단위·연결 상태·시점을 함께 기록하며, ESP32에서의 과거 성공을 Jetson 직결 성공으로 소급하지 않는다.

## STOP 제외·나머지 기능 유지 조건의 후속 검토

**현재 결론:** 엔코더 자체는 이전 손 회전에서후진+690 counts를 읽었다. 검토 대상은 추가 부품 없는 Jetson 수신 경로이며, 엔코더 고장이나 모든 Jetson 연결의 원천적 불가능을 확정한 것이 아니다. 현재 확인한 무개조 경로로는 안정 동작을 확정하지 못했다. 원격 조회는 장치 발견 단계에서 실패했고, 이후 사용자가Jetson 전원OFF를 알려 원인이 분명해졌다. GPIO 요청·pinmux 변경·USB 직렬 열기·출력은 실행하지 않았다.

| 엔코더 연결 후보 | 검토 결과 |
|---|---|
| 기존CF TXS0108E → J12 31/33의내장TXB0108 | 서로 다른칩이다. TXB의최소±2mA 입력 구동 요구와외부풀업 조건 충족을 보장하지 못한다. 신호누락·중복·오계수 우려이며, cascade만으로즉시소손을단정하지 않는다. 저속손회전성공만으로최대속도·모터노이즈·부팅안정성을보증하지 않는다. |
| CF → J12 27/28의GPIO 대체기능 | TXB를피하지만미채택. PDD.00/PCC.07은gen2_i2c의USB-C 제어기FUSB301·전력감시INA3221과공유하는경로여서다른기능유지조건에맞지않는다. 1.5kΩ풀업으로인한LOW마진도불리하다. |
| 엔코더5V 출력 → Jetson 직접 | 내부4.7kΩ풀업 때문에5V가들어올수있어미채택. |
| 검증된3.3V push-pull 버퍼 추가 | 규격에맞는개선방향. 5V허용Schmitt입력·Ioff·TXB 요구이상의출력구동능력·전원디커플링이필요하다. CF뒤에넣으면CF앞단정합은남고, 엔코더출력직접입력방식은정격과LOW/HIGH마진을별도대조한다. 현장에없어설치하지않음. |

27/28의직접GPIO 기능은 [NVIDIA SP-11324 v1.3 Table3-3](https://developer.nvidia.com/downloads/assets/embedded/secure/jetson/orin_nano/docs/jetson_orin_nano_devkit_carrier_board_specification_sp.pdf)에있다. 그러나 [R36.4.4의FUSB301 버스구성](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonOrinNxNanoSeries.html)과 [INA3221 전력감시](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html#jetson-orin-nx-series-and-jetson-orin-nano-series)를같이보아야한다. 현재실장DT확인없이버스를끄거나핀을재배정하지않았다.

전압여유검토는단순DC모델이다. Jetson의1.5kΩ와TXS의LOW상태40kΩ풀업을병렬로두고TXS패스저항300~500Ω를가정하면, 센서쪽을이상적인0V로잡아도입력쪽LOW는약0.57~0.85V다. [NVIDIA I2C 패드설명](https://forums.developer.nvidia.com/t/jetson-orin-nano-nx-i2c-dc-characteristics-for-voltage-level-confirm/357653/6)은1.8V패드기준VIL최대0.36V와3.3V tolerance설정을명시한다. GPIO모드의정확한적용조건과저항보증최악값은미확인이고, 이계산은실측이나절대실패증명이아니다. 다만낙관적조건에서도여유가부족해안정성확정근거로쓸수없다. [TI TXS/TXB 정합보고서](https://www.ti.com/lit/an/scea054a/scea054a.pdf).

전체기능유지의실물수용조건은다음과같다. 현재모두통과했다는뜻이아니다.

- 조향: USB장치식별·직접직렬통신, ID1/1Mbps, 위치·전압응답과토크OFF부터확인. 기존중앙2811/왼쪽3031/오른쪽2550실측값을보존하고검증이동은내부범위로제한.
- DC: pinmux/PWM실제대응, 전원인가전PWM0, 유한시간명령종료·프로그램오류시PWM0, 재접속에서임의구동없음확인. 물리STOP은평가대상에서제외. Jetson전체정지상황의독립차단은본구성에서제공하지않는한계다.
- 엔코더: 정상전압/배선조건과입력마진, 정·역방향, 실제기준계수일치, 모터구동노이즈와전체컴퓨팅부하에서누락·중복·잘못된A/B전이검사. 공칭13km/h·바퀴65mm·감속비2.5×81/17·28counts/rev면약5898edges/s,총에지간격169.5µs. 이속도처리의실물보장은아직없다.
- 수신구현: GPIO cdev v2의A/B동시입력·양쪽edge·배치읽기·monotonic시간·sequence gap검사가후보. 늦게읽은현재핀값으로과거event방향을추정하지않는다. 기본32event버퍼는약5.43ms분량에불과하다. gap0도IRQ전누락을증명하지못하므로외부기준계수가필요하다. [Linux GPIO event/overflow](https://docs.kernel.org/userspace-api/gpio/gpio-v2-line-event-read.html).
- IMU·B0390·C1: IMU I2C주소/단위/축/시각계약을확인하고기존CSI·USB경로를유지한다. 실제세센서동시취득·제어부하에서통신누락/지연/CPU부하를검증한다.
- 전원: 발열보드와외부하네스원인을분리해판정하고기존PDB정격·부하전압·최고전력모드의안정성을실측한다. 기존도면의M01입력24V미만운용가능성은해결된것으로표시하지않는다.

## Arduino UNO Q 대체 가능성 검토

2026-11-14 사용자 요청에 따라 **UNO Q**를 별도로 검토했다. UNO R3/R4와 혼동하지 않는다. **MCU의 기능·핀 구성상 ESP32 역할을 이관할 수 있는 후보이며, 현재 배선·전원·ESP-IDF 바이너리를 그대로 교체하는 방식은 아니다.** 실제 보드 확보·설치·펌웨어 빌드·통전 시험은 수행하지 않았다. 물리 STOP 제외 결정은 유지하고 서보·DC·엔코더·IMU·카메라·라이다 기능은 모두 유지한다.

[Arduino 공식 데이터시트](https://docs.arduino.cc/resources/datasheets/ABX00162-ABX00173-datasheet.pdf)의 STM32U585 Cortex-M33/최대160MHz와 3.3V MCU 헤더, [공식 회로도](https://docs.arduino.cc/resources/schematics/ABX00162-schematics.pdf)의 직접 연결을 확인했다. MCU 입력으로 옮기면 앞서 검토한 CF TXS→Jetson TXB 연쇄를 피한다. 이것은 엔코더 전체 배선의 잡음·계수 정확도를 실증했다는 뜻이 아니다. 2026-11-13 갱신 데이터시트는 D3/PB0를 최대3.6V의 TT형 입력으로 명시하므로, 오래된 pinout의 포괄적인 5V 허용 문구를 적용하지 않는다. 엔코더는 기존 CF의 3.3V 쪽을 사용하는 후보로 유지한다.

| 기능 | 유지할 구성과 이관 작업 |
|---|---|
| DC 모터 | MD20A 유지. STM32 하드웨어 PWM 20kHz + DIR로 ESP32 LEDC를 대체한다. 명령 만료·통신 상실·초기화 실패 시 PWM0 처리와 재접속 자동 구동 금지를 MCU에서 수행한다. |
| 엔코더 | PGE-213·CF 유지. MCU 하드웨어 quadrature timer 또는 GPIO 양쪽 에지 계수로 ESP32 PCNT를 대체한다. 공칭 최대 약5898 edges/s의 정·역방향 및 기준 계수 검증이 필요하다. 타이머와 PWM/UART 핀의 동시 배정을 확정해야 한다. |
| 서보 | 기존 Adapter(A)와 ST3215-HS 유지. MCU UART 1Mbps 또는 이미 검토한 Jetson USB/B점퍼 경로가 가능하다. 엔코더 타이머와 USART1 핀을 공유하지 않도록 배정한다. 기존 수동 교정값은 보존한다. |
| IMU | LSM6DSOX SPI 드라이버의 ESP-IDF 부분을 Zephyr HAL로 교체한다. 기존 약200Hz 취득·단위·시각 계약을 유지하며 전송량을 낮추기 위해 취득률을 임의로 낮추지 않는다. |
| 카메라·라이다 | Jetson의 B0390 CSI와 C1 USB 연결을 유지한다. UNO Q로 옮길 필요가 없다. |

### 전원에 대한 판정

- UNO Q 공식 권장 공급 조건은 **5V·3A 공급원/케이블**이다. 상시15W 소비한다는 뜻은 아니다. 기존 J01 CH4의5V·0.5A는 이 공급 조건을 만족하지 못한다.
- VIN은7–24V이다. **6S 만충25.2V를 직접 넣는 경로는 정격 초과**다.
- J01 CH1의19V·2.3A는 전압 범위에는 맞지만 기존 Jetson 전용이다. 전체43.7W에서 Jetson 최고 모드·carrier·저장장치·센서까지 포함한 여유가 입증되지 않아 단순 분기를 승인된 경로로 표시하지 않는다.
- [NVIDIA 공식 USB 설명](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/hardware_layout.html#usb-3-2-type-a-ports-mark-6)은 USB-A 두 포트가 한 stack이며 stack당 VBUS3A 제한이라고 명시한다. USB가 무조건0.5/0.9A라서 불가능하다는 판단은 하지 않는다. 다만 공유 부하·상위 전원 예산·실제 케이블/급전 조건을 확인하지 않아 Jetson USB만으로 안정 급전이 확정된 것도 아니다.

### 소프트웨어에서 확인한 제약

기존 `portable/` 구동 계약·CRC·명령 만료 로직은 재사용 후보지만 ESP-IDF의 PCNT/LEDC/SPI/UART/시간 HAL은 교체해야 한다. `/drive`→독립 보호층→`/drive/safe` 경계와 피드백 계약은 유지한다.

[공식 ArduinoCore-zephyr의 analogWrite 구현](https://github.com/arduino/ArduinoCore-zephyr/blob/0e2f265190d9fc4166a467215129e1ea29e3f1dd/cores/arduino/wiring_analog.cpp)은 PWM 핀이 없거나 준비되지 않았을 때 값에 따라 digital HIGH/LOW로 대체한다. 모터에는 이 동작을 그대로 쓰지 않고 50,000ns 주기의 Zephyr PWM API 성공 여부를 확인하는 전용 HAL과 오류 시 LOW 처리를 구현해야 한다.

[Arduino Router](https://github.com/arduino/arduino-router)의 USB 경로는 Linux USB gadget/Monitor와 내부 UART를 거친다. USB-C가 STM32 직접 USB serial이라는 가정은 틀리다. 기본 내부 링크115200 baud의8N1 payload 상한은11,520B/s다. 현재 `firmware_sensor_link/main/imu_source.c`는 accel/gyro 각각208Hz 설정과 FIFO 항목별68B 전송을 사용하므로 두 센서만 계산해도28,288B/s이며 timestamp/진단은 추가된다. 이는 소스 설정 계산이고 UNO Q 실측은 아니다. 따라서 기본 링크로 기존 형식을 그대로 모두 보내는 구성은 부족하다.

내부 MCU/Router 양쪽 baud 상향 또는 측정 정보를 유지한 프레임 재설계가 필요하다. 설정 API 존재와 실제921600/1Mbps 안정 통신은 구분한다. 호스트 `Serial.begin()` 속도만 바꾸는 것으로 내부 링크가 바뀐다고 간주하지 않는다. 대안으로 IMU를 Jetson I2C에 두고 MCU가 모터·엔코더를 담당할 수 있으나 이 경우에도 IMU 기능·취득률·시각 계약을 검증해야 한다.

소스에서 확인한 고속화 방법은 MCU에서 Monitor 사용 전 `Bridge.begin(921600)` 등의 속도를 설정하고 Linux Router의 `--serial-baudrate`를 같은 값으로 맞추는 것이다. `Monitor.begin()`은 호환용 baud 인수를 무시한다. [기본115200 설정](https://github.com/arduino/arduino-router/blob/7ad8ad813155a295ecf8f949d531237e4b5a1b23/debian/arduino-router/var/lib/arduino-router/config/10-imola.conf), [USB serial 서비스](https://github.com/arduino/arduino-router/blob/7ad8ad813155a295ecf8f949d531237e4b5a1b23/debian/arduino-router/usr/lib/systemd/system/arduino-router-serial.service), [Bridge](https://github.com/arduino-libraries/Arduino_RouterBridge/blob/dc170bd52095ba302d2da280f77a1ad57bbe4148/src/bridge.h), [Monitor](https://github.com/arduino-libraries/Arduino_RouterBridge/blob/dc170bd52095ba302d2da280f77a1ad57bbe4148/src/monitor.h). 브라우저에서 개별 permalink 열기는 실패했으나 공식 raw 파일을 내려받아 직접 확인했다. 관련 로컬 근거는 Git 제외 `build/jetson_direct_diagram_20261114/uno_q_sources/`에 보존했다.

하드웨어 계수 배정 후보는 D1/PB6·D0/PB7의 TIM4 CH1/CH2, 모터 PWM은 별도 TIM3 CH3 또는 TIM1 CH4다. 기본 USART1 콘솔과 TIM4 PWM 노드를 해제하는 구성이 필요하며 D1의 부트/리셋 출력도 확인해야 한다. 엔코더와 서보 UART를 같은 D0/D1에 동시에 배정하지 않는다. [보드 overlay](https://github.com/arduino/ArduinoCore-zephyr/blob/0e2f265190d9fc4166a467215129e1ea29e3f1dd/variants/arduino_uno_q_stm32u585xx/arduino_uno_q_stm32u585xx.overlay), [STM32 QDEC 드라이버](https://github.com/arduino/zephyr/blob/zephyr-arduino-v4.4.2/drivers/sensor/st/qdec_stm32/qdec_stm32.c). 이는 배선 확정이나 빌드 검증 결과가 아니다. 제어 주기와 MCU 명령 만료 처리는 Monitor/RPC의 블로킹 대기와 분리해야 한다.

**판정:** 기능을 포기하지 않는 대체 설계는 가능하다. 충분한 전원 경로 확정, MCU 펌웨어/HAL·호스트 통신 이식, 무전원 출력·엔코더·동시 취득 검증을 마치기 전까지 차량에서 안정 동작한다고 확정하지 않는다. 현재 수행한 작업은 공식 자료·공식 소스·기존 프로젝트 코드 대조다.

## 현장 보유 UNO Q로 선정한 후속 배선

사용자가 **UNO Q를 현장에 보유**하고 있다고 확인하고, 더 나은 대안이면 전체 회로도를 작성하도록 요청했다. 현재 선택지는 예비 ESP32를 기다리는 방법, Jetson 직접 제어, 보유 UNO Q 이관이다. **보유 UNO Q를 보조 MCU로 쓰는 설계를 추천하고 별도 전체도를 작성했다.** 이는 실물 안정성 인증이나 구동 승인이 아니다. 같은 ESP32 예비품이 확보되면 기존 펌웨어 재사용에 따른 복구 시간 이점은 있으나, 현재 예비품은 없다.

| 비교 항목 | Jetson 직접 제어 | 선정한 UNO Q 구성 |
|---|---|---|
| 엔코더 수신 | CF TXS→Jetson TXB 정합이 미완료 | CF 3.3V→STM32 입력. 기존 연쇄를 피함 |
| 계수·PWM | Linux GPIO 이벤트 수집과 PWM 이관 필요 | TIM3 하드웨어 quadrature와 별도 TIM4 PWM 사용 |
| 명령 만료 | Jetson 실행 상태에 의존 | MCU 실행 중 별도 주기·시계로 만료→PWM0 구현 가능 |
| 전원 | 새 보드 부하 없음 | M01 예비 CH6/CN7의12V2A→UNO Q VIN 전용 |
| Jetson 통신 | 로컬 함수 호출 | J12 UART↔STM32 USART1 직접 통신. 내부 Router115200과USB 급전을 경유하지 않음 |
| 필요한 작업 | GPIO·PWM·IMU 이관 및 전기 정합 해결 | ESP-IDF HAL/호스트 UART 이식·pinmux·부팅/부하 검증 |

선정 산출물: [확대 가능한 전체 배선도](uno_q_wiring_v1.html), [PNG](uno_q_all_v1.png), [핀별 연결표](uno_q_pin_table_v1.md), [설계 JSON](uno_q_v1.json). 기존 ESP32와 Jetson 직접 제어 도면은 보존했다.

- **전원:** M01 CH6/CN7 +→UNO Q VIN, −→전원GND. 제조사 M01 p1–3의CH5/6 독립12V2A 출력을 확인했다. CH5는서보에유지하며두출력을병렬로묶지않는다. CH6의24W와공급기준15W를비교하면효율80% 가정시12V입력약1.56A다. 이수치는설계가정이며기동피크·전압강하실측이아니다. M01입력24–36V,6S가24V아래로내려가는구간의정격문제는해결된것으로표시하지않는다.
- **통신:** Jetson J12-8 TX→D0/PB7 RX, D1/PB6 TX→J12-10 RX, J12-14→UNO Q GND. 공식회로도의R2623/R2624는0Ω직렬이며UART외부풀업이없다. STM32의±8mA CMOS출력규격이TXB의±2mA요구를충족하고반대편은고임피던스입력으로받는다. STM32내부30–50kΩpull은반드시끄고1Mbps/8N1·케이블용량·한쪽전원OFF·부트출력을검증한다. 이는Adapter의10kΩ풀업이있는UART직결안과다른경로다.
- **핀:** A2/PA6=TIM3_CH1, A3/PA7=TIM3_CH2로엔코더, D9/PB8=TIM4_CH3로모터PWM, D4=DIR를배정했다. Arduino데이터시트의A1/PA5 TIM3표기는ST DS13086 Table28과달라사용하지않는다. IMU는D10 CS/D11 MOSI/D12 MISO/D13 SCK/D2 IRQ. 기존SPI를유지하고센서취득률을낮추지않는다.
- **구현:** TIM3 PWM을끄고encoder전용, TIM4는PB8 CH3만PWM으로설정한다. D10/PB9를TIM4 CH4로동시에설정하지않는다. USART1 console/shell/logging을해제하고Serial1/전용HAL을단독사용한다. Serial/Monitor/Bridge와블로킹write가MCU만료처리를막지않게한다. RAM업로드/flash모드와전원재인가뒤시작절차는미확인이며Linux없이냉간부팅이보장된다고표현하지않는다.
- **접지:** UART=JDIGITAL15, 전원=JANALOG7, 센서=JANALOG6, MD20A신호=JSPI6의4접점을배정했다. T3V3는UNO Q/IMU/CF VA/CF OE의4접점, TGND는UNO Q/IMU/CF/PGE/J01 CN5−의5접점이다. 모터·서보의전력귀환은각M01단자로유지한다.
- **유지:** 서보는Jetson USB-C→C-to-C→Adapter(A),점퍼2개B. 카메라CSI·C1 USB를유지한다. 물리STOP만사용자제외다. 운영중UNO Q USB는미연결이며VIN/USB diode-OR의급전분담을피한다.

최종 사용자 지시로 정지 스위치는 넣지 않았다. IMU SPI의 SJ1 열림과 DO/MISO 스트랩 금지를 연결 조건에 추가했다. 최종 도면의11신호·GND4접점·글자145개 겹침/잘림0, 로컬 링크와 기존ESP32 원본5개 해시 보존을 확인했다. 검증 결과는 `build/uno_q_wiring_20261114/validation.json`에 보존한다. 공식근거는핀표와JSON에모았다. 이번에는도면과문서만생성했으며SSH·pinmux변경·펌웨어업로드·배선이동·출력은실행하지않았다.

## ESP32 단락 보고

- 사용자 전달: 현장 하드웨어 기술자가3V3–GND 단락을 의심했다.
- **USB와 외부 배선을 모두 뺀 보드 단독 측정**임을 사용자가 확인했다.
- 표시값은 “0.1이었다가100이었다가 왔다갔다”이며 **Ω/kΩ 단위·안정값·계기 모드·접촉 상태는 미확인**이다.
- `0.1 kΩ = 100 Ω`이므로 숫자 변화만으로 저항 변화나 영구 단락을 확정하지 않는다. 충전 성분·접촉 영향도 구분해야 한다.
- 발열·LED 소등·무응답을 근거로 **재인가 보류는 유지**한다. 보드 단독 상태 확인과 외부 하네스 정상 판정은 별도다.

저항은 전원·저장 전압이 제거된 상태에서 측정하고 단위를 함께 읽는다. [Fluke 저항 측정 안내](https://www.fluke.com/en-us/learn/blog/digital-multimeters/how-to-measure-resistance).

## 미채택 대안

추가 watchdog·저전압 reset·ARM latch·PWM gate·Schmitt 버퍼안은 현장 부품 제약으로 제외했다. 메모는 Git 제외 `build/jetson_direct_diagram_20261114/unselected_extra_parts_v2.json`에 보존한다. 조달·제작·설치·물리 출력은0이다. 그 대안의 논리 검토를 현재 구성의 안정성 증거로 사용하지 않는다. 핀·전원·입력 공식 근거는 [v1 연결표](jetson_direct_pin_table_v1.md#근거)를 따른다.
