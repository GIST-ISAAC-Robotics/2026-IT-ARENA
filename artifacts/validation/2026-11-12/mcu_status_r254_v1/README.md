# r254 실제 MCU 상태 연결 후보

## 필요한 변경

공통 운영 계층은 `boot/generation/state/estop_held/now_us` 상태를 읽지만 실제 UART 연결은 아직 이를 발행하지 않는다. 기존50Hz offer에는 estop 필드가 없고 상세 진단은1Hz여서, 없는 값을 정상으로 채우거나 진단을 최신 상태처럼 재발행하지 않는다.

기존 송신 정책에 **기본 false인 `offer_status_v1`**을 두고 선택한 센서 연결 펌웨어에서만 상태 샘플 시각·실제 estop 래치·이유를 같은 offer에 싣는 후보를 만들었다. 수신은 원 MCU 시각/원 I/O 수신시각과 최소 수신 offset의 **미교정 추정**을 함께 남긴다. 지연/큐 대기·동일 시각 반복·세대 역행·이전 boot가 새 상태로 바뀌지 않게 한다. 가상 엔코더 피드백이나 출력 준비를 생성하지 않는다.

## 검증 경계

- 후보 C++의 기본 OFF 바이트는 기존 송신기와 동일하고 ON은 같은 상태/시각의3필드만 추가한다. ESTOP 누름/해제(래치 유지)·프레임 상한, Python 시계의 묶음/350ms 지연/새 boot/세대/시계 변경 등을12개 검사로 확인했다.
- 실제20.51초 UART 수신→관측 전용 `/hardware_preview/actuation_status`→공통 `McuObserver`를 수행했다. ARM/CLEAR 거부·STOP 응답을 확인하고 원 IMU 앱을 복구했다.
- MCU의 ESTOP 플래그는 펌웨어 수신 코어 상태다. 이 앱에 물리 비상정지 버튼 입력이 있다는 뜻이 아니다. 실제 운용 상태 발행·구동 허가·모터/서보/엔코더 HAL 통합은 완료하지 않았다.
- 소스시각 추정의 최소 전송 지연·시계 배율은 미검증이다. 관측 결과를 공통 운영 주행이나 시각 교정 완료로 승격하지 않는다.

## 실제 보드 결과

MCU offer989개에서 warmup13개를 제외한 **상태976개가 발행/수신/공통 관측 수락 모두 일치**했다. 원 MCU 상태·이유·세대·estop·출력 준비값과 대응했다. 원시 CRC/JSON을 별도로 순회해 비교했으며 상태 지연중앙3.14ms·p954.80ms·최대6.15ms다. 모두 첫 STOP 뒤 `ESTOP_LATCHED`였고 세대1/2/3을 관측했다. 정상 ARM이나 실제 버튼 눌림을 실험한 것은 아니다.

IMU 원문11,973개·순번 공백/CRC/SPI/송신 오류0, 공통 gyro3,901개 수락·거부0이었다. 엔코더0·DRIVE0·출력 준비false이며 가짜0속도를 넣지 않았다. 제어 응답은10.87–13.59ms, 자식3개0·포트 미점유·원 IMU 앱 readback 일치다. 복구 후 센서는 원 앱의 대기 상태다.

원문은 Git 제외 `build/r254_recovered`, 회수 ZIP SHA256 `802b72d2291e2ed8780df7ced686ee1fe410c5e78b26c0451445096e05715899`다. 공개 근거는 `audit.json`, bridge/receiver/supervisor, flash/restore 요약에 둔다. 현재 회로/센서 취득 시각 교정·실제 구동 완료는 아니다.

이후 공통 송신 코어에는 기본 OFF 옵션을, 출력 없는 `firmware_sensor_link`에만 ON을 연결했다. 공통 브리지는 `--mcu-status`를 명시해야 관측 상태를 발행한다. 시각 연결 모듈은 실제 시험 후보와 바이트가 같으며 공통 관련93검사·소스 출처214검사와 `arena_vehicle_interface` 빌드를 통과했다. 근거는 `common_tests.log`, `provenance.log`, `colcon.log`다.
