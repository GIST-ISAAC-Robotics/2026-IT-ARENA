# MD20A 단발 벤치 · 엔코더 관측

사용자가 요청한 무부하 모터 시험용 격리 앱이다. 운영 속도 제어·ROS 구동 계약과 별개이며 `/drive` 또는 `/drive/safe`를 발행하지 않는다. 엔코더가 0이어도 출력을 증가시키지 않는다. 원 수동 진단 앱과 실패 기록은 별도 보존한다.

- GPIO15: LEDC 20 kHz/10 bit PWM, GPIO6: DIR LOW 고정. 역회전 명령은 없다.
- PGE/CF 관측: GPIO4/5 입력·pull 없음, PCNT x4·독립 GPIO ISR·1 µs PCNT filter. 상태마다 `raw_ab`도 읽는다.
- UART0 GPIO43/44, 921600 8N1. 응답은 CRC32+JSON, `info/status/edge/motor/reply`가 하나의 boot/sequence를 공유한다.
- 기본 PWM=0. `ARM`은 현재 boot·증가 요청 ID를 확인하고 2초 유효한 일회용 token을 발행한다. `PULSE`는 token을 소비하며 duty 1~100‰, 지속 시간 1~500 ms만 허용한다. PWM 분해능에 반올림하므로 50‰는 51/1024≈4.9805%다.
- UART 수신과 별도의 `esp_timer` 작업이 1 ms마다 마감 시각을 확인하고 `ledc_stop(...,0)`으로 종료한다. 정지 후 3초 cooldown, 명령 자동 재시도/재시작/출력 상승 없음. 잘못된 CRC/형식/boot/ID/범위·조립 중인 줄 250 ms timeout도 출력 종료·허가 해제다.
- `?\n`: 조회, `S\n`: 50 Hz 상태 송신, `X\n`: PWM 종료·허가 해제·1 Hz 대기. 활성 상태에서 새 ARM/PULSE는 거절하면서 출력을 종료한다.

구동 프레임은 `8자리 CRC32 + 공백 + ASCII 본문 + LF`다. 본문은 `ARM <boot> <id>` 또는 `PULSE <boot> <id> <token> <duty_permille> <duration_ms>`다. CRC는 본문 전체를 덮으며 암호학적 인증은 아니다. 출력 종료 후에도 DIR은 LOW다.

[`probe_esp32_motor.py`](../../../scripts/probe_esp32_motor.py)는 모터 전원을 분리한 상태에서 허가/만료/중복/형식 검사와 두 번의 무전력 PWM 시험을 수행한다. 첫 연결의 부팅 바이트는 CRC가 맞는 첫 프레임 앞의 접두부로 따로 기록하고, 정상 식별 뒤 프레임 손상/순번 누락/재부팅은 실패 처리한다. 사전 검사 후 **같은 포트를 계속 열어 둔 채** 사용자 전원 연결과 명시 단발 요청을 기다린다. raw UART·전체 JSONL·송신 명령·첫 실패는 Git 제외 `build/`에 보존한다.

## 물리 시험 조건과 한계

### 조립 후 5초 관측용 별도 빌드

`-D ARENA_MOTOR_GUARDED_5S=ON`은 `motor_probe_guarded_5s_v1`로 식별되며 501~5000 ms 명령을 최대50‰까지 허용한다. `-D ARENA_MOTOR_START15_5S=ON`은 별도 `motor_probe_start15_5s_v1`이며 같은 긴 명령의 상한만150‰로 둔다. 두 빌드 모두 500 ms 이하 명령의 상한은100‰이며 자동 출력 상승은 없다. 기본 빌드의500 ms 상한은 유지한다.

긴 명령의 무회전 시작 차단(약350 ms 안에4카운트), 회전 후 진행 소실 차단(160 ms), 표본 속도/관측 간격 검사, GPIO8 개방 STOP,5초 마감은 두 빌드에서 동일하다. **5초는 최대 지속시간**이며 시작 차단이 발생하면 실제 출력은 그보다 짧다. 엔코더 분해능·감속비를 실측하지 않은 표본 속도 검사는 실제 최고속도 보증이 아니다.

[`run_guarded_motor_bench.py`](../../../scripts/run_guarded_motor_bench.py)는 무전원으로 시작해 해당 앱과 조기 종료를 확인하고 명시 요청을 기다린다.15% 변형은 `--start15`가 필요하며 요청 `powered-start-15-5s`를 한 번만 처리한다. 먼저 전원 분리 상태에서 펌웨어를 설치·검사한 뒤 같은 UART 세션에서 모터 전원 연결을 확인한다. 사용자 관찰을 자동 판정하거나 채팅 중단 요청을 실시간 정지 장치로 사용하지 않는다.

[11/14 결과](../../../docs/hardware/MOUNTED_DC_TEST_2026_11_14.md)는 설치된 버전과 준비한 후보를 구분한다.

업로드·리셋·포트 재개방·시험 후 종료 전에는 MD20A의 모터 전원을 분리한다. 현재 회로에는 별도 출력 EN이 없고 ROM/리셋 중 PWM 파형은 미검증이다. MCU 타이머는 UART 작업과 분리되지만 MCU 전체 고장에 독립적인 전력 차단 장치가 아니다. 낮은 duty는 전류 제한이나 실제 속도 제한을 의미하지 않는다.

MD20A PWM LOW는 두 출력 LOW인 단락 제동이다. 회전 감속을 만들지만 정지 축에 서보와 같은 일정 유지 토크를 명령하지 않는다. GPIO readback/종료 시각과 실제 감속·정지를 구분하며, 엔코더 동작이 확인되지 않으면 감속 시간/각도/회전수 값을 계산하지 않는다.

실물 결과는 [2026-11-12 보고서](../../../docs/sensors/MD20A_MOTOR_HARDWARE_2026_11_12.md)에 기록한다. [Espressif LEDC](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/ledc.html), [Cytron MD20A](https://www.cytron.io/p-20amp-6v-30v-dc-motor-driver).
