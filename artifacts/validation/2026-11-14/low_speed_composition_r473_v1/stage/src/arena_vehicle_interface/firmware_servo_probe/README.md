# ESP32-S3 · ST 버스 서보 격리 진단

이 앱은 Jetson USB-UART0(GPIO43/44, 921600 baud)와 Adapter A의 UART1(GPIO17 TX / GPIO18 RX)을 연결하는 제한된 시험 도구다. UART1과 Adapter A는 같은 이름의 TX→TX/RX→RX, 점퍼 A 배치를 사용한다. 모터 전원이 분리되고 서보혼·조향 링크가 없는 시험을 위한 것으로, 운영 구동/보호층을 구현하지 않는다.

- GPIO15 PWM·GPIO6 DIR을 앱 시작 시 LOW로 초기화한다. ROM/리셋 이전 구간의 실제 파형을 보장하지 않으며 모터 전원은 분리한다.
- 시작 상태는 읽기 전용이다. ID별 PING/READ만 허용하며 방송 쓰기, EEPROM·ID·baud·offset·mode·잠금 레지스터 쓰기는 거부한다.
- 호스트 `B` 명령은 **ESP32 UART1 속도만** 바꾼다. 서보의 baud 레지스터를 변경하지 않는다.
- `ARM id centre speed`는 현재 위치를 직접 재조회해 ±3 counts 이내인지와 mode=0을 확인한다. 허가 후 위치 명령은 centre±114 counts(약±10.02°), 0–4095 범위, 속도 raw≤400·지정 상한 이내, 가속도 raw5–20으로 한정한다. 현재 SRAM 토크 enable=0/1 외 특수값 2/128은 허용하지 않는다.
- 활성 상태에서 1초 동안 유효 요청이 없거나 전체60초가 지나면 토크 disable=0을 전송한다. 이는 소프트웨어 송신 조치이며 버스 단선/전원 이상에서 물리적 토크 해제를 보장하지 않는다. `X`도 같은 해제를 수행한다. 앱 재부팅 후 자동 ARM/이동은 없다.
- 읽기 원시 응답·MCU 전송 시작/종료 us와 호스트 monotonic receipt를 보존한다. 현재 위치·전류는 서보가 보고한 값이고 외부 각도/전류 계측값이 아니다.

호스트 도구: [`probe_bus_servo.py`](../../../scripts/probe_bus_servo.py). 기본은 `--mode read`, 움직임은 명시적 `--mode motion`에서만 가능하다. 도구는 단일 ID 조회, 설정/상태 기록 후 현재 위치→작은 양방향 이동→원위치→토크 해제를 수행한다. 반복은 `--cycles 1..5`, 한 회 이동 구간10초다. `--mode watchdog`는 현재 위치 고정/토크 ON 뒤 송신을1.5초 중단해 만료 이벤트·토크 OFF를 확인한다. 이상 응답·전압·온도·전류·위치 미도달을 확인하면 중단한다. EEPROM0–39를 전후 비교한다. 토크와 가속도·목표/속도는 SRAM 설정이며 물리 장착 후 운영값으로 재확인한다.

빌드는 ESP-IDF v6.1 환경에서 이 디렉터리에 `idf.py build`를 실행한다. 실제 사용한 SDK 커밋·sdkconfig·바이너리 SHA-256과 업로드 전 이미지, 원문 UART는 Git 제외 시험 폴더에 보존한다. 시험 종료 후 기존 IMU 진단 앱을 복구하고 실제 응답을 확인한다.

근거: [Adapter A 배선](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example), [ST 통신 프로토콜](https://docs.waveshare.com/Servo_Communication_Protocol), [ST 메모리 표](https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation). ST 계열 표를 HS 모델의 정밀 동특성·정확도 증거로 확대하지 않는다.
