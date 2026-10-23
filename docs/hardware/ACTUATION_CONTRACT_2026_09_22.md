# Jetson–MCU 구동 계약과 정지 절차: 1차 구현

> 2026-10-02 후속: 현행 시뮬레이션의 피드백은 [ADR 0026](../decisions/0026-motor-shaft-encoder-baseline.md)의 단일 모터축 엔코더다. 아래 좌우 카운트·양쪽 바퀴 저속 관측은 기존 모드의 계약이다. 새 모드는 실제 차체·양쪽 바퀴 정지를 보증하지 않는다. 기존 신선도/명령 보호 시간·STOP/CLEAR/ARM 경계는 유지하고, 모터축 모드에 **0.1 m/s 이상 명령에서 500 ms 무진행 정지**를 추가했다. 실측 전 임시값·적용 경로·고장 검증과 별도 평균 속도 PI는 [새 보고서](../autonomy/MOTOR_ENCODER_VALIDATION_2026_10_02.md)를 따른다.

날짜: 2026-09-22. 상태: **A1-1 PC 참조 구현과 시험 완료**. ESP32 펌웨어·실물 제동·Gazebo 코스 적용 완료가 아니다.

후속: [A1-2 ROS 연결/시험](ACTUATION_ROS_2026_09_22.md)을 완료했다. 별도 가상 MCU와 명시 허가/정지·복구 경로를 선택형으로 연결하고 ROS 14조건·Gazebo 약 12 m 부분 주행 두 경우를 검증했다. 아래 ‘다음 ROS 연결’은 A1-1 당시 이력으로 보존하며 실물 펌웨어·내부 속도 루프/ESC 검증은 여전히 남는다.

## 1. 역할 분담과 이번 범위

사용자가 재확인한 구조는 다음과 같다.

- **Jetson:** 인지·지역 경로·Pure Pursuit·목표 속도/조향각과 독립 안전 제한.
- **MCU:** 좌우 뒷바퀴 엔코더 취득, 평균 속도 피드백의 단일 모터 속도 제어, 조향 장치 명령, 최종 구동 허가/정지와 통신/제어 주기 감시.
- **모터 제어기/ESC:** 선정 장치가 지원하는 실제 전력 구동·BLDC 정류·제동. MCU가 모터 3상 출력을 직접 만들어야 하는 구조는 아니다.

ESP32-S3는 우선 후보이며 정확한 보드·엔코더·ESC·조향 명령 방식은 미정이다. 지금은 Jetson만 있으므로 첫 구현은 하드웨어 없이 검사할 수 있는 계약으로 제한했다. MCU 두 개로 좌우 바퀴를 따로 제어하거나 전자식 차동으로 바꾸지 않는다.

향후 연결 위치:

```text
Jetson /drive → 독립 LiDAR 보호층 → /drive/safe → 호스트 어댑터
                                                   ↓ 구동 계약
                                        MCU 정지/허가 관리자
                                                   ↓
좌우 엔코더 → 평균 바퀴 기반 속도 → 단일 속도 PI → ESC → 모터 한 개
                   └──────── 취득 시각·카운트·상태를 Jetson에 반환
```

**이번에는 위 ROS 경로에 새 모듈을 삽입하지 않았다.** 기존 주행 결과와 비교 기준선을 보존하며, PC 시험기가 호스트/MCU·가상 엔코더/모터를 대신한다. 실제 안전층을 우회해 `/drive/safe`를 임의 발행하도록 변경하지 않았다.

## 2. 구현한 파일

| 파일 | 역할 |
|---|---|
| [actuation_contract.py](../../src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py) | MCU 수신 상태기계·만료/순번/부팅 세션·엔코더 카운트 처리와 Jetson 명령 생산기 |
| [actuation_wire.py](../../src/arena_vehicle_interface/arena_vehicle_interface/actuation_wire.py) | 분할/합쳐진 바이트 프레임·길이·CRC32·JSON 오류 처리 |
| [mcu_speed_loop.py](../../src/arena_vehicle_interface/arena_vehicle_interface/mcu_speed_loop.py) | 좌우 평균 속도 PI, 출력 포화/적분 과누적 방지, 정지 시 적분 초기화 |
| [test_actuation_contract.py](../../tests/test_actuation_contract.py) | ROS/보드 없는 단위 검사 38개. 무작위 상태 전환 20 seed × 100회도 포함 |
| [validate_actuation_contract.py](../../scripts/validate_actuation_contract.py) | 고정 시간 10시나리오와 별도 프로세스 실제 시간 IPC 시험 1개 |

Python은 PC에서 계약을 실행·검증하기 위한 참조다. 실물에서는 이 상태 전환/검사 순서를 C/C++에 포팅하고 같은 입력·기대 결과로 대조할 예정이다. 최종 UART/CAN/micro-ROS 어댑터는 아직 없다.

## 3. 정지와 재출발

**현행 단일 모터축 계약(10/13 재확인):** 아래 상태 이름과 명시 STOP/CLEAR/ARM 절차는 유지한다. 다만 `stationary_ready`는 평균 바퀴 환산 속도의 절댓값이 0.03 m/s 이하로 200 ms 지속된 **엔코더 저속 준비 조건**이지 차체 정지 확인값이 아니다. `stationary_scope=mean_wheel_only_opposite_rotation_unobservable`와 함께 읽는다. 양쪽 바퀴 검사와 평균 상쇄 거절은 **9/22 legacy 쌍 모드에만** 해당한다. G1의 정상 HOLD/재개도 명시 STOP 해제와 별개다. [관측 한계·실제 기록 대조](../autonomy/MOTOR_MOTION_OBSERVABILITY_2026_10_13.md).

| 상태 | 의미 | 구동 목표 |
|---|---|---|
| `DISARMED` | 부팅 또는 명시적 구동 해제 | 0, 구동 금지 |
| `ARMED` | 명시적 허가를 받았으나 0속도 목표 | 0, 신선한 후속 명령만 허용 |
| `ACTIVE` | 허가·피드백·유효기간을 만족하는 양의 목표 | 유효 목표 속도 |
| `ESTOP_LATCHED` | 물리/소프트웨어 STOP 상태 유지 | 0, 후속 DRIVE 거절 |
| `FAULT_LATCHED` | 명령 만료·엔코더/제어 주기·시계/입력 고장 상태 유지 | 0, 후속 DRIVE 거절 |

- STOP은 다른 명령보다 우선한다. 물리 버튼 해제, 통신 복구, 엔코더 복구만으로 상태를 풀지 않는다.
- `CLEAR`는 정지 고정을 풀어 **DISARMED로만** 이동한다. 그 뒤 별도 `ARM`이 있어야 한다. 버튼이 눌려 있거나 해당 피드백 모드의 저속 준비 조건이 충족되지 않으면 거절한다(legacy는 양쪽 바퀴, 현행은 모터축 평균).
- **기존 legacy 쌍 모드:** 좌우 평균만 0인 것으로 준비 조건을 통과시키지 않으며 좌우 반대 회전도 거절한다. 두 바퀴 모두 0이어도 차체 미끄러짐은 구별하지 못한다. **현행 모터축 모드:** 반대 회전의 평균 상쇄도 구별할 수 없어 준비 조건을 통과할 수 있다. 두 모드 모두 실제 차체 정지의 증명이 아니다.
- `ARM`은 **신호등을 봤다는 뜻이 아니다.** 경기 최초 출발 신호와 중도 재출발 허용 절차는 Jetson의 경기 상태 관리에서 별도로 연결해야 한다. 시험기의 자동 ARM은 시험 스크립트에만 존재한다.
- 정상적인 0속도 명령은 비상정지와 다르다. 신호 대기/일시 보호 감속의 0 목표를 계속 받는 동안에는 `ARMED`를 유지한다. 센서 복구 후 자동 진행의 구체적 허용 조건은 후속 경기/보호 상태 설계에서 다룬다.
- 정지 시 조향각은 마지막 값을 유지한다. 즉시 중앙으로 꺾는 동작을 기본 안전 정책으로 가정하지 않는다. 실차 조향 전원/유지 정책은 별도로 결정한다.

## 4. 메시지와 시간 규약

### 호스트 → MCU

공통 필드는 `v`, `kind`, `boot`, `generation`, `lease`, `client`, `seq`다. `kind`는 `ARM`, `CLEAR`, `DISARM`, `DRIVE`이며, DRIVE에는 다음 필드가 추가된다.

| 필드 | 의미 |
|---|---|
| `speed_mps` | 차량 목표 전진 속도(m/s). 실제 지면 속도 측정값이 아님 |
| `steering_rad` | 차량 모델 기준 조향각(rad), 좌측 양수. 실제 서보 위치로의 변환은 후속 어댑터 |
| `source_age_us` | 호스트가 새 전송을 만들 때 **원래 목표 생성 이후 지난 시간** |
| `ttl_us` | 원래 목표의 최대 유효기간. MCU 한도를 넘길 수 없음 |

소프트웨어 STOP은 `{"v":1,"kind":"STOP"}`다. 현재 신뢰된 로컬 링크의 참조 규약에서는 만료된 구동 세션과 관계없이 정지를 받아들인다. 이 기능을 인증 없는 공개 네트워크에 노출하지 않는다.

### MCU → 호스트

상태/임대 응답에는 부팅 ID·상태 세대·소유 호스트·적용한 순번·목표 속도·조향각·정지 이유·명령 만료 시각, 좌우 누적 카운트·취득 시각·표본 순번·바퀴 속도·피드백 신선도·물리 STOP 상태가 있다. 시험기는 여기에 실제 가상 모형 속도와 정규화 모터 출력을 **평가용으로만** 기록한다.

상태 응답은 명령 수신/상태 적용 근거이지 실제 차체 정지 확인이 아니다. 실물 상태에는 선택한 ESC/서보가 실제로 제공하는 측정 피드백을 따로 추가해야 한다.

### 시계가 다른 문제를 이번에는 어떻게 다뤘는가

단순히 ‘받은 뒤 100 ms 동안 유효’라고 하면 오래 지연된 명령도 새것처럼 실행될 수 있다. 이를 막기 위해 MCU가 자기 단조 시계로 짧은 유효기간의 **임대 토큰(lease)**을 발행하고 호스트가 이를 되돌려준다.

```text
명령 만료 시각 = min(토큰 만료 시각,
                     토큰 발행 시각 + 명령 TTL − 원래 명령 나이)
```

호스트→MCU 이동에 걸린 시간은 토큰 발행 이후 전체 시간을 통해 보수적으로 반영된다. 오래된 명령을 재포장하지 않도록 호스트도 원래 표본 시각을 단조 증가로 검사한다. 반복/역순 순번은 수명 연장 없이 거절한다. STOP/해제/새 ARM 때 세대를 바꾸어 이전 토큰을 무효화하며, MCU 재부팅은 새 부팅 ID, 호스트 재시작은 새 호스트 ID를 쓴다.

이는 **구동 명령 만료만을 위한 참조 방식**이다. IMU/엔코더 취득 시각을 Jetson/라이다 시간축으로 옮기는 센서 시간 동기화까지 해결한 것이 아니다. 추가 왕복과 보수적인 수명 감소도 있으므로 실제 통신 지연/대역폭과 비교해 최종 프로토콜을 결정한다. 신뢰된 호스트가 원래 생성 시각을 정직하게 보존한다는 가정도 필요하다.

### 프레이밍과 방어

- `IA` 매직 바이트 + little-endian 16비트 길이 + UTF-8 JSON + 길이/본문 CRC32. payload 상한 1536바이트, 임대 보관 상한 32개다.
- 분할/합쳐진 프레임, 잘못된 길이/CRC, 중복 JSON 키·NaN/Infinity·잘못된 필드/자료형을 검사한다. 재연결 때 남은 부분 프레임을 폐기한다.
- CRC는 우발적 손상 검사이며 **인증/암호화가 아니다**. 실제 무선 경로에는 별도의 인증·페어링·재전송·응답 확인이 필요하다.
- 깨진 길이 때문에 불완전 프레임이 잠시 남더라도 유효 DRIVE가 없으면 수신기 자체 만료가 정지시킨다. 실제 수신 작업은 유한 큐/처리량과 부분 프레임 시간 제한을 갖추고, 물리 STOP은 일반 명령 대기열과 별도로 처리해야 한다.
- 참조 구현은 단일 제어 작업 호출을 전제로 한다. 펌웨어의 수신 인터럽트/통신 작업과 제어 작업 사이에는 짧은 큐 또는 적절한 동기화가 필요하다. 프레임 파싱/로그가 제어 작업을 막지 않게 한다.

## 5. 속도 루프와 임시 설정

동일 반지름의 후륜 두 개에서 바퀴 기반 속도를 구한 뒤 평균을 사용한다.

```text
왼쪽/오른쪽 속도 = 카운트 변화 × (2π × 바퀴 반지름 / 회전당 카운트) / 취득 시각 간격
제어 오차 = 목표 속도 − (왼쪽 속도 + 오른쪽 속도) / 2
```

좌우를 각각 같은 속도로 강제하지 않는다. 기계식 차동은 코너에서 좌우 회전 차이를 허용하며, 속도 루프 출력은 모터 한 개에 대한 하나다. 카운트 전송 도착 간격이 아니라 MCU 취득 간격으로 계산한다. 카운터 초기화/비현실적 점프·미래/오래된 표본·중복/역순을 구분한다.

PI는 출력 포화 시 더 누적되는 적분을 막고 정지/비허가 시 적분을 초기화한다. 출력 `effort`는 −1~1의 **가상 정규화 요청**이며 PWM duty나 특정 ESC 스로틀이 아니다. 음수와 `brake_requested`를 실제 ESC 후진 신호에 그대로 연결하면 안 된다. 제동/무신호/중립/역전의 매핑은 실물 장치 확인 후 작성한다.

| 임시값 | 사용처/주의 |
|---|---|
| 명령/임대 100 ms | 시험용 만료 상한. 규정/실차 안전값 아님 |
| 엔코더 신선도 50 ms·제어 갱신 공백 최대 50 ms | 실패 경로 검증용. 실제 MCU 지터/제동 여유로 재설정 |
| 바퀴별 0.03 m/s 이하를 200 ms 관측 | 허가/정지 해제 준비 조건. 실제 차체 정지의 증명 아님 |
| 반지름 0.025 m·2048 counts/rev | 기존 임시 차량 가정. 실물 사양 미정 |
| 목표 상한 2.5 m/s·조향 0.37 rad | 현재 지역 추종 프로필과 맞춘 참조 한도. 속도 향상 없음 |
| PI Kp 0.6·Ki 1.2 | 단순 가상 모터용. 실차에 그대로 적용하지 않음 |

고정 시간 시험은 제어 200 Hz·엔코더 100 Hz·호스트 명령 50 Hz다. 별도 프로세스 시험은 PC 단조 시계를 쓰며 반복 주기는 OS 스케줄링에 영향받는다. ESP32의 해당 주기 달성을 검증한 것이 아니다.

**주의:** 통신 콜백/상태 조회는 제어 주기 감시 시각을 갱신하지 않는다. 제어 작업이 돌아온 뒤 공백을 발견하면 고장으로 정지하지만, MCU 자체가 완전히 멎었을 때 이 소프트웨어도 실행되지 않는다. 실제 하드웨어 watchdog/출력 허가 회로는 여전히 별도로 필요하다.

## 6. 검증 결과와 실패 이력

최종 근거는 [windows_v3/report.json](../../artifacts/validation/2026-09-22/actuation_contract/windows_v3/report.json)과 [검사 요약](../../artifacts/validation/2026-09-22/actuation_contract/windows_v3/regression_summary.txt)이다. 결과에 기록된 다섯 소스 파일 SHA-256과 현재 파일이 일치함을 확인했다.

| 시험 | 결과 |
|---|---|
| 정상 허가 → 평균 엔코더 PI 주행 → 0 목표 정지 | 통과 |
| 주행 중 호스트 명령 끊김 | 만료 정지·재출발 없음 |
| 이전 명령 반복 재생 / 160 ms 늦은 명령 | 거절·수명 연장 없음 |
| 주행 중 엔코더 누락 후 복구 | 고장 정지 유지 |
| 물리 STOP 후 버튼 해제 | 비상정지 유지 |
| MCU 재부팅 / 호스트 재시작 | 구동 금지 또는 만료 고정·자동 허가 없음 |
| 정지 해제만 수행 | DISARMED 유지 |
| 정지 확인 → 명시적 해제 → 별도 허가 | 그 이후에만 재주행 |
| 별도 가상 MCU 프로세스·실제 시간 명령 송신 중단 | MCU 자체 시계로 만료·고정 정지·옛 명령 거절·가상 모터 정지 |

- 고정 시간 10조건 + 별도 프로세스 1조건 **11/11 통과**.
- 독립 프로세스 종료 코드 0, 강제 종료 없음, 해당 자식 프로세스 잔존 없음.
- Windows 표준 라이브러리 단위 검사 **38개**, WSL ROS 전체 회귀 **340개(17.06초)** 통과.
- 설계/수정/최종 근거 대조는 주 에이전트, 최종 실행/결과 수집은 기존 Luna 실행 에이전트가 수행했다. 중복 실행 감시는 하지 않았다.
- 초기 `development_v1`에서는 결정론적 10조건 파일 생성 후 Windows 로컬 IPC 생성이 `WinError 5`로 거절되어 전체 보고서가 만들어지지 않았다. 결과를 보존했고, 초기화 실패도 보고서에 남기도록 도구를 보완했다. 승인된 재실행 `windows_v2`는 당시 37개/11조건/전체 339개를 통과했다. 이 환경 오류를 차량 제어 실패나 통과로 섞지 않는다.
- 최종 검토에서 비정상 큰 정수의 `math.isfinite(10**400)` 검사에 `OverflowError`가 나는 것을 재현했다. 수신 계약·PI 검사에서 안전하게 거절하도록 수정하고 회귀 1개를 추가했다. 그 변경 뒤의 `windows_v3`가 최종 38개/11조건/전체 340개 근거다. 앞선 실행을 최종 소스로 재실행한 것처럼 덮어쓰지 않았다.

가상 모터는 `속도/위치 + 임의 가속·저항·제동`의 작은 종방향 모형이다. 평가용 모형 속도는 제어기에 전달하지 않고 양자화한 카운트를 통해서만 피드백한다. 이 시험은 **Gazebo 완주·실차 제동거리·실제 ESC·ESP32 성능·모든 센서 오류 내성**을 검증하지 않았다.

재현할 때는 기존 결과를 덮어쓰지 않는 새 출력 폴더를 지정한다.

```powershell
python tests/test_actuation_contract.py
python scripts/validate_actuation_contract.py --output artifacts/validation/2026-09-22/actuation_contract/new_run
```

## 7. 임베디드 구현 권고

1. **ESP32-S3를 우선 유지하는 것이 합리적이다.** Arduino와 ESP32-S3는 반드시 양자택일이 아니다. S3를 Arduino-ESP32 환경에서 개발할 수도 있다. 팀의 익숙함에 맞춰 Arduino-ESP32로 시작하되, 제어 주기·펄스 카운터·watchdog 같은 부분은 Espressif 하위 API를 명확히 관리하는 방식을 권고한다. 이번에 개발 환경/버전을 설치·확정하지는 않았다. [공식 Arduino-ESP32 시작 안내](https://docs.espressif.com/projects/arduino-esp32/en/latest/getting_started.html)
2. **엔코더는 매 펄스마다 무거운 코드를 실행하지 않는 방향을 우선한다.** ESP32-S3의 PCNT는 에지/방향 카운트와 quadrature 처리를 지원한다. 전기적 펄스 glitch filter도 있지만, 정상 짧은 펄스를 지우지 않도록 분해능·최대 회전수로 설정해야 한다. 이 하드웨어 필터는 IMU/속도값의 평활화와 다른 기능이다. [PCNT 공식 문서](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/pcnt.html)
3. **속도 제어, 센서 취득, 통신/로그의 실행 책임을 나눈다.** 제어 주기는 통신 수신 빈도에 종속시키지 않고, 블로킹 문자열 읽기·많은 로그·무선 재연결이 속도 루프를 멈추지 않도록 한다. 취득 시각은 데이터 생성 때 찍는다. [ESP Timer](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/esp_timer.html)
4. **watchdog은 실제 제어 작업의 진행을 확인해야 한다.** 다른 통신 작업이 대신 정상 신호를 넣어 멎은 속도 루프를 숨기지 않게 한다. 내부 watchdog 재부팅만으로 ESC 출력/구동 차단이 안전하다고 보장할 수는 없다. [Espressif watchdog](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html)
5. **USB 유선 명령을 먼저 권고하지만 핀·제동을 먼저 확정하지 않는다.** ESC가 torque/current/throttle/RPM 중 무엇을 받는지에 따라 MCU의 외부 속도 루프가 달라진다. 이미 강한 ESC 속도 루프가 있으면 중복된 루프의 역할과 응답을 조정해야 한다. ST3215-HS도 실제 버스/위치 피드백을 확인한 뒤 어댑터를 쓴다.

하드웨어팀과 정할 최소 항목은 **정확한 ESP32-S3 보드, 모터/ESC 모델과 제동/명령 형태, 엔코더 분해능·전기 규격·장착 지점, 조향 장치 통신, 전원과 독립 STOP 경로**다. 이번 추가 질문에서 사용자는 ESC 모델/명령 방식이 **미정**이라고 재확인했다. 아직 답이 없더라도 PC 계약 검증은 진행할 수 있다.

## 8. 다음 작업

1. **A1-2:** `/drive/safe` 뒤의 호스트/가상 MCU 어댑터 연결과 ROS 프로세스 종료·재시작·정지/재허가 검증. 느리게 흐르는 시뮬레이션 시계와 실차 단조 시계의 정책을 분리한다. 본 참조의 100 ms 실시간 한도를 느린 Gazebo 실행에 그대로 붙이지 않는다.
2. **A2:** 센서/명령/최종 출력 상태를 원래 시각과 함께 기록·재생하고 Jetson 시험 입력 묶음을 준비한다.
3. **A4:** 잡음/이상치·고착·좌우 개별 펄스 손실, MCU 속도 루프 내부 피드백 오류를 검증한다. 이번 카운트 유효성 검사는 모든 센서 잡음 대응 완료가 아니다.
4. 보드/ESC 선정 후 C++ 포팅·실제 I/O·물리/원격 STOP·전기적 차단을 저속으로 검증한다. 선택한 출력 제한·가감속/조향 변화율·게인을 실측으로 정한다. 현재 PI는 그 전체 구동 정책을 대체하지 않는다.

기존 코스·지역 추종·센서 배치·운영 속도·공식 원본·다른 작업 파일은 보존했다. 새로운 Gazebo 주행·Jetson 접속·구매·외부 댓글/이슈·GitHub 게시·컴퓨터 종료는 수행하지 않았다.

## 9. 10/15 후속 — 단일 모터축 계약의 MCU 포팅용 대조 벡터

실제 펌웨어가 도착했을 때 Python 참조와 같은 입력으로 비교할 수 있도록
[`actuation_conformance.py`](../../scripts/actuation_conformance.py)를 추가했다.
기존 참조 구현·ROS/Gazebo 런타임·기본 설정은 변경하지 않았다. **실제 펌웨어 구현/보드 실행은 아니다.**

- 명시 구성: 모터축 `drive_motor_shaft`, 28카운트/모터 회전·15:1·바퀴 반지름 0.0325 m. 나머지 만료/상태 한도는 현행 참조 계약을 유지한다. 설정 파일과 별도 회귀로 대조한다.
- 이벤트: MCU 시각의 정수 `now_us`, 엔코더 취득 시각/순번/부호 있는 누적 카운트, 제어 tick, 임대 발행, 물리 STOP, 명령, 재부팅. 명령에는 JSON과 실제 CRC 프레임 hex를 모두 저장한다. 수신 상태/호출 결과를 각 이벤트 뒤에 기록한다.
- 사례: 명시 ARM 없는 부팅, 양수/0목표, 중복 명령, 원천 만료, 명령/엔코더/제어 갱신 만료, 물리 STOP 해제, 소프트웨어 STOP 뒤 대기 명령, CLEAR→별도 ARM, 잘못된 legacy 피드백, 카운트 점프/음수/고착, 재부팅, 다른 호스트, 시계 역행, 중복 피드백, `2^53` 초과 카운트 보존이다.
- 비교: 상태·이유·정수 시각/카운트·null·bool은 정확히 비교한다. PC 부동소수 필드는 절대오차 `1e-9`다. 이는 **double 참조 재생용**이며 MCU float32 허용오차를 확정하지 않는다. JSON을 읽을 때 카운트를 double로 경유시키면 큰 정수가 손실되므로 거절한다.

[v2 결과](../../artifacts/validation/2026-10-15/actuation_conformance_v2/export/report.json)는
**19사례·1,324이벤트·39개의 수동 기대 상태 checkpoint**, 참조 재생 차이 0이다.
관련 회귀 **92개 통과**에는 관측 누락/이름/상태/자료형 변조·모터 환산식·int64 보존 반례가 포함된다.
v1은 새 테스트가 YAML 경로를 `drivetrain` 아래로 잘못 읽어 91통과/1실패 후 내보내지 않았다.
실제 `vehicle.sensors.drive_feedback` 경로로 테스트만 고친 뒤 v2를 실행했으며 [실패 로그](../../artifacts/validation/2026-10-15/actuation_conformance_v1/tests.log)와 소스를 보존했다.

이관 입력은 [vectors.json](../../artifacts/validation/2026-10-15/actuation_conformance_v2/export/vectors.json),
참조 출력은 [python_observations.json](../../artifacts/validation/2026-10-15/actuation_conformance_v2/export/python_observations.json)이다.
포팅 측은 각 사례의 초기 boot/config부터 같은 이벤트를 순서대로 적용하여 `{name, responses}`를 만들고
`compare_case(case, observed)`와 대조한다. 시각은 호스트 도착 시각으로 교체하지 않는다.
`reboot`만 시계 원점을 바꾸며 구동 허가·예전 세대 명령을 이어받지 않는다.

**남은 범위:** 현재 출력은 Python에서 생성하고 Python으로 다시 재생한 것이므로 독립 C++ 포팅 검증이 아니다.
수동 checkpoint도 전체 가능한 상태를 전수 검증하지 않는다. UART 분할/노이즈/부분 프레임 처리,
실제 PCNT/타이머·스레드 동기화·watchdog·I/O·핀/전압·MD20A 제동·조향 응답·실제 차체 정지는 별도다.
단일 모터축 0속도는 차체 정지 증거가 아니며 모든 결과에 `firmware_verified=false`, `ready_to_drive=false`를 유지한다.

## 10. 10/21 후속 — 분할 바이트와 만료/STOP의 결합 대조

[전송 대조 도구](../../scripts/actuation_transport_conformance.py)는 기존 포팅 벡터의 완성 메시지 검사에 더해, 지속되는 decoder에 분할/병합 바이트를 공급한 뒤 **완성된 메시지만** 수신 상태기계에 전달한다. 엔코더·제어 tick·물리 STOP은 독립 이벤트다. 모터축 28카운트·15:1·65 mm, 100 ms 임대·50 ms 피드백/제어 공백 기준을 유지했다. 운영 소스·ROS 토픽·기본 설정은 변경하지 않았다.

[첫 실행 결과](../../artifacts/validation/2026-10-21/actuation_transport_v1/README.md): 관련 **163회귀**, 12사례·828이벤트·상태 checkpoint 48개·전송 checkpoint 16개, 재생 불일치 0·소스 불변·실제 종료 0. STOP의 모든 분할 위치, 여러 청크 크기와 단일 비트 오류를 포함한다. 생성→재생만으로 독립 구현이 검증되는 것은 아니므로, 별도 수동 기대 상태/응답과 변조 거절도 검사한다.

| 조건 | PC 참조에서 확인한 동작 |
|---|---|
| DRIVE 조립 지연·중복 재전송 | 도착/완성 시각으로 만료를 연장하지 않음. 원천 나이 20 ms의 명령은 원래 임대 발행+80 ms에 만료 |
| DRIVE + STOP + 대기 DRIVE 한 청크 | STOP에서 세대가 바뀌고 뒤 명령을 거절. 중간 출력도 저장해 마지막 상태만 비교하지 않음 |
| 깨진 CRC·허용 범위 밖 길이 뒤 STOP | 다음 정상 STOP을 해석해 고정 정지 |
| **허용 범위 안의 손상 길이** 뒤 STOP | decoder가 더 많은 본문을 기다려 STOP이 아직 해석되지 않음. 기존 명령이 발행 후 100 ms에 만료되어 목표 0·고장 고정 |
| 위 불완전 프레임 중 물리 STOP 이벤트 | parser 완성을 기다리지 않고 같은 이벤트에서 목표 0. 버튼 해제만으로 재개하지 않음 |
| parser reset | 남은 바이트만 폐기. 구동 허가를 해제하거나 다시 허가하는 동작이 아님. 연결 끊김 정책/독립 만료가 별도로 필요 |
| 제어 tick 없이 통신/피드백만 지속 | 제어 주기 감시를 대신 갱신하지 못함. 임계 초과 뒤 `control_deadline_missed` 고정 |

손상 길이 때문에 STOP이 지연되는 현상은 §5에 이미 남긴 한계의 결합 재현이다. 이번에 새로 생긴 ROS 장애나 정상 STOP이 즉시 처리된 성공으로 쓰지 않는다. 위 100 ms는 **시험 명령의 발행 시점부터 목표가 0이 된 시점**이며 실차 정지 시간/제동거리가 아니다. 물리 STOP도 실제 버튼·출력 차단 회로를 시험한 것이 아니라 참조 API 이벤트다.

실물 수신기는 부분 프레임의 절대 조립 시간 제한·유한 큐/호출당 작업량·고장 시 폐기/재동기화가 필요하다. 데이터가 조금씩 계속 들어온다고 조립 타이머를 무한 연장하지 않아야 한다. 다만 baud·정상 최대 프레임·USB/스케줄 지연을 확인하지 않은 고정 시간값을 지금 채택하지 않는다. 다음은 현재 실제 인코딩 크기/발행 빈도의 링크 예산을 계산해 시간 제한 설계의 전제를 명시하는 것이다. CRC는 인증이 아니며 악성 송신자를 차단하는 시험도 아니다.

## 11. 10/21 후속 — 현재 인코딩의 직렬 링크 예산

[계산 도구](../../scripts/audit_actuation_link_budget.py)는 실제 `VirtualMCU.send_offer`와 `ActuationHost.on_drive`를 노드 생성 없이 호출했다. 부팅 근처·100초·24시간, 32자 boot/client 식별자의 3표본에서 CRC/헤더 포함 최대 길이는 **offer 673 B, DRIVE 봉투 260 B**였다. STOP 서비스 봉투 형태는 103 B로 계산했으며 실제 서비스 왕복을 실행한 것은 아니다. [결과와 소스](../../artifacts/validation/2026-10-21/actuation_link_budget_v1/README.md), 전체 **3,570회귀·소스 불변·종료 0**.

당시 계산은 정기 offer 20 ms와 비교용 DRIVE 50 Hz, UART 8N1의 바이트당 10 bit를 가정했다. **10/22 후속 감사에서 `on_tx`가 추가하는 offer를 빠뜨렸음을 확인했다. 아래는 정기 offer만 센 50/50 Hz 산술 이력으로 보존하며 현행 콜백의 전체 예산이 아니다. 정정은 §18을 따른다.** 전이중의 두 방향은 별도로 계산한다. 아래 부하는 `frame_bytes × 50 × 10 / baud`이며, 1 이상이면 이 표본 트래픽만으로도 지속 처리가 불가능하다.

| baud | 호스트→MCU 부하 | MCU→호스트 부하 | 표본 두 방향 모두 용량 미만 |
|---:|---:|---:|---|
| 115200 | 1.129 | 2.921 | 아니오 |
| 230400 | 0.564 | 1.461 | 아니오 |
| 460800 | 0.282 | 0.730 | 예, 다른 트래픽/지연 여유 별도 |
| 921600 | 0.141 | 0.365 | 예, 실제 지원/오류율 별도 |

상태 전체 대신 현행 호스트 명령 생성에 필요한 필드만 남기는 **미구현 투영**은 offer 238 B였다. 이 경우 230400에서 두 방향 부하는 0.564/0.516으로 줄지만, 빠진 모터 피드백/진단/IMU를 어디에 전달할지 아직 설계하지 않았다. 투영을 실제 대체 구현으로 채택하지 않는다. 보드 선택 문서의 [USB-UART 우선안](ESP32_BUDGET_REVIEW_2026_10_02.md#usb-연결-검토---102-후속)과 직접 USB를 구분하며, 이 산술을 native USB의 실측 성능으로 옮기지 않는다.

전체 허용 프레임 상한 1,544 B의 순수 직렬화 시간은 115200/230400/460800/921600에서 각각 **134.03/67.01/33.51/16.75 ms**다. 따라서 모든 속도에 임의로 같은 20 ms 조립 제한을 붙이면 정상 프레임까지 잘릴 수 있다. 실제 수신 최대 크기·baud·처리/USB 지연과 제어 명령 수명을 함께 검토해야 한다. 이 표본은 모든 숫자/문자열 필드 폭의 최악 상한이 아니며 ACK·추가 센서·로그·큐·CPU 지연은 제외했다.

**결정:** 115200 기본 예제를 실차 설정으로 그대로 옮기지 않는다. 현재 ROS·MCU 만료 기준/제어·센서 취득 빈도·baud는 변경하지 않으며, 실물 링크 실측 후 충분한 여유를 둔 속도 또는 작은 주기 메시지를 선택한다. 다음 준비는 기존 wire 형식을 보존하는 고정 메모리 C++ 프레임 코어의 PC 대조다. 실제 JSON 명령 검증·상태기계·핀/전기 출력·펌웨어 동시성/타이머 이관은 별도다.

## 12. 10/21 마감 — C++ 프레임 코어의 제한 이관

[보드 독립 코어](../../src/arena_vehicle_interface/portable/README.md)는 기존 `IA`·길이·CRC32를 고정 배열로 처리한다. payload 1536 B 상한을 유지하고 호출당 입력은 256 B로 제한했다. 초과 입력은 조용히 잘라 정상 수신으로 만들지 않고 부분 버퍼를 폐기하며 실패를 반환한다. 상위 작업의 유한 큐/호출 빈도·callback 작업량과 부분 조립 만료는 별도로 설계해야 한다.

사용자 마감 요청 당시 작성 중이던 묶음만 [컴파일·대조 1회](../../artifacts/validation/2026-10-21/actuation_wire_cpp_v1/README.md) 수행했다. g++ 13.3.0·C++17·ASan/UBSan, 관련 **184회귀**, **243사례·2,294연산 차이 0**·기존 Python JSON 경계 **2,039회 차이 0**, 실제 종료 0·소스 불변·sanitizer 오류 보고 없음. 앞선 3,570개 전체 회귀와 이번 관련 검사를 합산하거나 최종 전체 실행으로 표현하지 않는다.

**중요한 경계:** C++ callback은 CRC가 맞는 raw payload를 내보낼 뿐이다. 비UTF-8/비JSON/중복 키/NaN/명령 스키마/정수·상태·임대 검증은 C++ 미이관이며, Python 검증기를 뒤에 둔 대조로 그 경계를 확인했다. 실제 MCU나 ROS 운영에 연결하지 않았고, C++ MCU 구동 계약 전체가 완성된 것이 아니다. 고정 메모리도 ESP32 최악 실행시간·펌웨어 전체 무할당·차체 정지 보장이 아니다.

재개 순서:

1. 현행 바이트 계약과 상태 벡터를 기준으로 부분 조립 절대 만료·재연결/수신 오류 정책을 별도 PC 후보로 설계한다. 실물 baud/큐/USB 지연이 없는 현재는 시간값을 운영 기본값으로 확정하지 않는다. 조각 수신만으로 만료/제어 watchdog을 갱신하지 않는다.
2. 엄격한 JSON 검증과 상태 계약을 단계적으로 이관한다. 부호 있는 64비트 카운트·MCU 시각, CLEAR→별도 ARM, 물리 STOP 분리, 프레임 유효성과 구동 허가의 분리를 유지하고 기존 벡터와 비교한다.
3. 실제 보드의 링크·PCNT·타이머·I/O/전기 STOP을 저속으로 확인하기 전 `firmware_verified=false`, `ready_to_drive=false`를 유지한다. G1은 실제 측정면 단면/반사·가림 자료 확보 뒤 관측 계약을 재검토하며, 이번 마감 후 같은 실패 주행을 자동 반복하지 않는다.

## 13. 10/22 후속 — 부분 프레임 절대 기한과 연결/오류 정책 후보

§12 재개 순서 1번을 **PC 검증용 후보**로 구현했다. [`actuation_link.hpp`](../../src/arena_vehicle_interface/portable/actuation_link.hpp)의 `FrameLink`가 기존 decoder를 감싸고, [참조 정책](../../scripts/actuation_link_reference.py)이 기존 `ActuationReceiver`와의 경계를 정의한다. wire 형식·payload 1536 B·호출당 256 B·CRC·고정 배열은 유지했다. 기존 decoder에는 CRC 실패와 같은 재검사를 외부에서 호출하는 `reject_head`만 분리했다. ROS 기본 경로·Python 런타임·운영 설정은 변경하지 않았다.

### 13.1 시각·기한 계약

| 항목 | 정한 의미 |
|---|---|
| 시계 | 호출자가 주는 단조 `uint64` tick(단위는 호출자 정의). 같은 값 허용. 직전 값보다 작으면 `clock_regressed` 고정. 32비트 카운터 wrap도 역행으로 본다(모듈러 산술 없음). |
| 기한 시작 | 부분 버퍼가 생긴 호출의 시각(anchor). 조각·빈 입력을 계속 넣어도 늦춰지지 않는다. 이전 호출의 바이트가 모두 소비된 경우에만 그 호출 시각으로 갱신한다(FIFO라서 남은 바이트가 모두 그 호출에 도착). 다른 재동기화는 이전 anchor를 유지하므로 기한이 실제 시작보다 이를 수는 있어도 늦지 않다. |
| 경계 | `now - anchor >= timeout`이면 만료. `anchor + timeout` 시각의 바이트는 이전 조각에 붙지 않는다. `now >= anchor`가 보장되어 덧셈 overflow가 없다. timeout 0은 영구 구성 오류다. |
| 만료 처리 | 머리를 손상으로 보고 CRC 실패 규칙으로 남은 바이트를 끝까지 재검사한다. 이미 완성돼 있던 CRC-valid 프레임만 `expired_drain`으로 전달하고 버퍼를 비운다. 새 바이트는 그 뒤에 처음부터 처리한다. |
| 무수신 | `poll(now)`로 독립 작업이 확인한다. 실제 폐기 지연은 호출 주기만큼 늦어질 수 있다. |

독립 oracle은 C++ 상태를 읽지 않고 바이트마다 도착 호출·시각을 보존해 계산한다. 매 `feed/poll` 뒤 **버퍼의 어떤 바이트도 timeout 이상 머물지 않음**과 anchor가 가장 오래된 바이트보다 늦지 않음을 불변식으로 검사했다.

### 13.2 연결·오류와 구동 허가의 분리

| 사건 | 링크(C++) | 참조 구동 정책 |
|---|---|---|
| CRC/길이 오류, 조립 만료 | 재동기화·drain, 연결 유지 | 상태 변경 없음. 기존 명령 만료가 별도 판정 |
| drain된 늦은 프레임 | `expired_drain`으로 표시 | 엄격 JSON 뒤 receive. 원래 임대/원천 시각으로 만료하므로 수명 연장 없음 |
| 단절, 연결 중 재연결(놓친 단절) | 부분 바이트를 **전달 없이** 폐기, `session_breaks` 증가. 단절 중 입력 거절 | 수신기가 고정 상태가 아니면 로컬 STOP 고정(세대 변경) |
| 호출당 256 B 초과·null 포인터 | `input_budget` 고정, 그 호출의 바이트 미처리 | 위와 같은 고정 |
| 시계 역행 | `clock_regressed` 고정, 그 호출 미처리 | 위와 같은 고정. 수신기 자체도 같은 시각에서 `clock_regressed` |
| `clear_fault(now)` | 시계 기준 재설정, `disconnected`로만 복귀 | 구동 상태 변경 없음 |
| 재구동 | 링크는 허가를 만들지 않음 | 새 offer 기반 CLEAR → 별도 ARM → DRIVE만 허용 |

프레임 유효성(CRC), 명령 유효성(엄격 JSON·스키마·세션·임대), 구동 허가(CLEAR/ARM 상태)는 서로 다른 단계다. 정책은 링크 연산에서 `tick`·CLEAR·ARM을 호출하지 않는다. 물리 STOP은 링크 상태와 무관하게 같은 이벤트에서 목표 0이며, 해제·재연결·고장 해제만으로 재개하지 않는다. 링크 바이트·poll이 계속돼도 제어 tick이 멎으면 `control_deadline_missed`가 된다. 부팅 첫 연결은 세션 경계가 아니므로 기존처럼 ARM만 필요하지만, 한 번이라도 연결 세션이 끊기면 DISARMED였더라도 CLEAR→ARM을 요구한다.

### 13.3 결과

[1차 대조](../../artifacts/validation/2026-10-22/actuation_link_cpp_v1/README.md): g++ 13.3.0 C++17·ASan/UBSan, 관련 **198회귀**·전체 **3,593회귀 통과**, 실제 종료 0·소스 해시 불변.

- 링크 단위 509사례(수동 경계 28·기존 프레임 241·무작위 240)와 정책 10사례의 링크 연산, driver 13,764줄·13,245연산에서 C++/oracle 차이 0. `-O2` 빌드도 같음.
- 수동 기대값: 기한 직전/정확히/직후(feed·poll), 침묵, 1 B trickle(링크 사례 100 tick 간격·oracle 회귀 1 tick 간격), 빈 입력, 완성 프레임 여럿+partial tail의 재기준, 이전 바이트가 남는 재동기화, 숨은 STOP drain 뒤 같은 feed의 정상 프레임, 만료된 tail의 비결합, 불량 CRC/길이, 단절·재연결·놓친 단절, 256/257 B와 null 입력, 시계 역행(연결/단절 중), `UINT64_MAX` 인접 timeout/시각, wrap, timeout 0.
- C++ 링크 출력을 그대로 참조 정책·기존 수신기에 넣은 재생 1,054이벤트·상태 checkpoint 43·링크 checkpoint 22 차이 0.
- 기존 프레임 코어 243사례·2,294연산·JSON 경계 2,039회 차이 0.
- 의도적 결함 9개 변이체를 모두 검출했다. Python 쪽은 조각마다 기한을 늘리는 oracle 변형과 세션 경계 STOP을 빼는 정책 변형이 실패하는지 회귀로 확인했다.

| 정책 사례(시험 timeout 30 ms) | 확인한 동작 |
|---|---|
| 합법 길이 1536 손상 머리 뒤 STOP | 29.999 ms 시점 poll에서는 미해석, 30 ms poll에서 drain되어 `ESTOP_LATCHED/software_stop`. §10의 같은 입력은 100 ms 명령 만료까지 STOP이 가려졌다 |
| 10 ms마다 1 B trickle | 기한 연장 없음. 조립 만료만으로 ACTIVE 유지, 원래 임대 발행+100 ms에 `command_expired` |
| 숨은 DRIVE의 늦은 drain | 수락되지만 원래 마감 320 ms 유지, 그 시각에 만료 |
| 단절 → 재연결 | 단절 즉시 목표 0. 이전 조각과 새 바이트 비결합, 이전 세대 DRIVE `session` 거절, CLEAR 뒤에도 ARM 전 DRIVE 거절 |

### 13.4 운영 timeout의 전제(미결정)

정상 프레임을 자르지 않으려면 timeout은 수신 측 최대 프레임의 직렬화 시간과 USB/스케줄 지연보다 커야 한다. §11의 8N1 산술로 최소 직렬화 시간만 보면 다음과 같다.

| baud | 최대 허용 1,544 B | offer 표본 673 B | DRIVE 표본 260 B |
|---:|---:|---:|---:|
| 115200 | 134.03 ms | 58.42 ms | 22.57 ms |
| 230400 | 67.01 ms | 29.21 ms | 11.28 ms |
| 460800 | 33.51 ms | 14.61 ms | 5.64 ms |
| 921600 | 16.75 ms | 7.30 ms | 2.82 ms |

1,544 B를 허용하는 timeout이 100 ms 명령 임대보다 길면 숨은 STOP은 drain보다 명령 만료로 먼저 목표 0이 된다. 그래도 drain은 이후 바이트의 재동기화를 보장한다. MCU 수신 방향의 최대 길이를 호스트 명령 크기에 맞게 줄이는 방법도 숨김 구간을 줄일 수 있지만 wire 계약 변경이므로 이번에 채택하지 않았다. 30 ms 같은 시험값은 460800 이하에서 최대 프레임을 자를 수 있으므로 운영값으로 쓰지 않는다.

### 13.5 한계와 재개 순서

- 이 결과는 PC 후보의 바이트 경계·참조 정책 검증이다. ROS/Python 운영 경로, 실제 UART/USB의 연결 감지·오류 보고, ESP32 크로스 컴파일·ISR/작업 동시성·최악 실행시간, 전기 STOP·차체 정지는 미검증이다. drain 작업량은 기존 CRC 재동기화와 같은 차수로 버퍼에 묶이지만 측정값이 아니며 callback 비용은 제한하지 않는다.
- UART에는 연결 개념이 없어 단절 사건을 받을 수 없을 수 있다. 그 경우 링크는 조립 기한·재동기화만 제공하고 세션 경계는 명령 만료·임대에 의존한다. 실제 전송 수단을 정한 뒤 단절 신호의 출처를 다시 정한다.
- 세션 경계에 로컬 STOP을 넣는 것은 기존 수신기를 고치지 않은 참조 구현이다. 수신기 사유는 `software_stop`이고 링크 사유는 정책 상태에 따로 남는다. C++ 상태기계 이관 때 별도 사유로 바꿀지 결정한다.

재개 순서(§12를 대체):

1. 엄격한 JSON 검증과 수신 상태 계약을 단계적으로 C++에 이관한다. 부호 있는 64비트 카운트·MCU 시각, CLEAR→별도 ARM, 물리 STOP 분리, 세션 경계 고정, 프레임 유효성과 구동 허가의 분리를 유지하고 기존 포팅 벡터·이번 정책 사례와 비교한다.
2. 실물 전송(USB-UART/직접 USB)·baud·최대 수신 프레임·지연을 측정한 뒤 운영 timeout과 MCU 수신 최대 길이를 정한다. 그 전에는 `firmware_verified=false`, `ready_to_drive=false`를 유지한다.
3. 실제 보드의 링크·PCNT·타이머·I/O·전기 STOP을 저속으로 확인한다.

## 14. 10/22 후속 — 엄격 JSON과 명령 필드 표현의 C++ 이관

§13 재개 순서 1번 중 **JSON·명령 내용 검사**만 PC 후보로 옮겼다. [`actuation_command.hpp`](../../src/arena_vehicle_interface/portable/actuation_command.hpp)의 `CommandParser`는 CRC가 맞은 payload를 고정 메모리 `Command`로 바꾼다. 수신 상태기계는 이관하지 않았고 운영 Python(`actuation_wire.py`, `actuation_contract.py`, `actuation_ros.py`)·ROS·MCU 연결도 바꾸지 않았다.

### 14.1 기준으로 삼은 기존 의미

| 단계 | 기존 Python | C++ 표현 |
|---|---|---|
| 전달 여부 | `Decoder`: strict UTF-8 → `json.loads`(중복 키 hook, NaN/Infinity 거절) → dict만 전달. 실패는 receive를 호출하지 않음 | `ParseStatus`: invalid_utf8 / syntax / nonfinite_constant / duplicate_key / not_object(+API 크기 오류). 판정 순서도 Python과 같음: UTF-8 먼저, 상수는 만나는 즉시, 중복 키는 그 객체가 닫힐 때 |
| 배치 | 포팅 벡터는 명령 객체 그대로. ROS 가상 MCU는 `type=="command"`이고 `command`가 dict일 때만 receive, 그 밖은 무시 | `kBareCommand`/`kEnvelope`, envelope 무시를 별도 분류 |
| 내용 분기 | `_check` 뒤 `v`가 정확히 int 1 → STOP 정확 형태 → 키 집합(schema) → 세션 → 임대 → 준비/소유권/순번 → DRIVE 값(`invalid_drive` 고장) → 원천 만료 | `classify`(invalid_message/STOP/schema/kind), `session_shape_ok`, `drive_values_ok(limits)`. 상태가 필요한 세션 일치·임대·소유권·순번은 값만 보존 |

**잘못된 DRIVE 값은 parser에서 버리지 않는다.** 예를 들어 ACTIVE 상태에서 세션이 맞고 `speed_mps`가 문자열이면 기존 receive는 `invalid_drive` 고장을 고정한다. C++는 이를 `kDrive`, `session_shape_ok=true`, `drive_values_ok=false`로 표현한다. 기존 receive는 세션 검사를 먼저 하므로 같은 잘못된 값이라도 세대가 틀리면 고장이 아닌 `session` 거절이 된다(코드 순서에서 나온 결론이며, 이번 수동 사례는 필드를 하나씩만 바꿨다). 따라서 상태기계가 기존 순서대로 판정해야 한다. 이 함정을 일부러 구현한 변이체(전처리 폐기)는 receive 동등성 67건 실패로 검출됐다.

### 14.2 고정 크기와 Python 임의 정밀도 사이의 경계

| 항목 | C++ 후보 | Python과의 관계 |
|---|---|---|
| 정수 | int64 범위는 10진에서 직접 정확히 읽음(double 경유 없음). 그 밖은 `kBigInt`와 부호만 | receive의 모든 정수 범위가 int64 안이므로 판정이 같음. 속도·조향 한도는 유한하고 2^63 미만일 때만 지원(`limits_supported`) |
| bool | `kBool`, int와 구분 | `type(True) is int`가 거짓인 것과 같음 |
| 실수 | `std::from_chars`. 범위 밖은 ±inf, 0이 아닌 최소값 미만은 ±0.0 | PC libstdc++ 13.3에서 subnormal·반올림 경계·`-0.0`의 비트가 Python `float()`과 같음. MCU 표준 라이브러리의 정확 반올림은 미확인. `from_chars` 내부 할당 여부는 allocator 계측을 하지 않아 PC·MCU 모두 미확인(코어 코드 자체는 무할당) |
| 문자열 | 코드포인트 수(escape 해석·surrogate 결합 뒤). `boot`·`client`·`request`는 64개까지 전체 저장, 짝 없는 surrogate 보존 | MCU boot id가 64 코드포인트 이하이면 비교가 정확함. 65 이상 client는 원래도 session, request는 원래도 ack 대상 아님 |
| 중첩 | 깊이 1536까지 비트스택, 중첩 객체의 중복 키도 검사. 값 내용은 타입만 | Python 3.12는 1536 B 안에서 가능한 최대 깊이(corpus 765단)를 허용. receive는 중첩 값 내용을 쓰지 않음 |
| 메모리·작업량 | 동적 할당 없음(코어 코드). x86-64에서 `CommandParser` 3,848 B, `Command` 1,144 B. 선형 스캔 + 같은 객체 안 키 쌍 비교 | 작업량은 payload 크기에 묶이지만 최악 실행시간을 측정하지 않았다 |

이번 범위에서 Python이 받아들이는 1536 B 이하 payload를 C++가 "미지원"으로 다르게 처리하는 경우는 발견하지 못했다. 다만 이 결론은 PC libstdc++ 13.3과 Python 3.12의 결과다.

### 14.3 결과

[1차 대조](../../artifacts/validation/2026-10-22/actuation_command_cpp_v1/README.md): g++ 13.3.0 C++17·ASan/UBSan과 `-O2`, 관련 **208회귀**·전체 **3,603회귀 통과**, 실제 종료 0·소스 해시 불변.

- payload 6,285(문법·경계 corpus, 기존 포팅 벡터 명령, 실제 `CommandProducer`·ROS 호스트/가상 MCU 콜백 payload, 무작위 변이 6,000)와 상태별 receive 사례 3,667, 기존 전송 12 스트림(FrameLink→parser)의 driver 10,763줄에서 C++/독립 기준 차이 0. 기준 판정과 실제 운영 Decoder의 전달 여부·dict도 차이 0.
- **receive 동등성 3,667건 차이 0**: 원래 JSON과 C++ 출력만으로 되살린 dict를 같은 수신기 상태에 넣어 결과·전체 status가 같다.
- 수동 기대값 209개: 오류 순서, escaped key 중복, surrogate, 반올림 경계(`2.5000000000000002` 수락·`…03` 고장), underflow `1e-400`→0.0(ARMED)·`5e-324`(ACTIVE), `-0.0`, `2^64`, seq `2^32`, client 64/65, 순서·escape만 다른 정상 명령, STOP 추가 필드(schema), invalid_drive 고장, 정상 값의 세대·client·임대 거절.
- 의도적 결함 17개 변이체를 모두 검출했다. 예비 실행에서 검출하지 못한 1개(surrogate 쌍 길이)는 명령 필드 사례 9개를 추가해 보완했다. 같은 예비 단계의 fixture 크기 오류도 증거 README에 남겼다.

### 14.4 한계와 다음 순서

- 상태기계·ack/offer·운영 연결·ESP32 툴체인·최악 실행시간·전기 출력은 미검증이다. `firmware_verified=false`, `ready_to_drive=false`.
- receive 동등성은 4개 대표 상태와 그 변형에서 확인한 것이며 모든 상태 조합을 전수한 것이 아니다.

다음은 C++ 수신 상태기계 이관이다. `Command` 표현을 입력으로 기존 판정 순서(시계 → 내용 분기 → 세션 → 임대 → 준비/소유권/순번 → DRIVE 값 고장 → 원천 만료)를 옮기고, 기존 포팅 벡터·전송 사례·이번 receive 동등성 사례와 비교한다. 그 다음이 실물 전송 측정과 운영 timeout 결정이다(§13.5 2·3번 유지).

## 15. 10/22 후속 — 수신 상태기계의 C++ 이관

[`actuation_receiver.hpp`](../../src/arena_vehicle_interface/portable/actuation_receiver.hpp)의 `ActuationReceiverCore`가 §14의 `Command`를 입력으로 `ActuationReceiver`의 `_check`·receive 판정 순서, 물리/소프트웨어 STOP, CLEAR→별도 ARM, 임대 32개, 모터축/legacy 피드백을 C++에서 계산한다. Python은 비교 기준으로만 썼고 운영 Python·ROS·MCU는 바꾸지 않았다. Claude의 1차 구현·대조 뒤 사용량 경고로 GPT가 독립 검토와 API 경계 수정을 이어받았다.

[1차 대조](../../artifacts/validation/2026-10-22/actuation_receiver_cpp_v1/README.md): 관련 215·전체 3,610회귀 통과. 포팅 벡터·전송·링크 정책·새 수동 반례·legacy와 명령 receive 사례의 이벤트 8,680개에서 result·전체 status 차이 0(ASan/UBSan, `-O2`). 의도적 결함 13/13 검출.

### 15.1 이관한 의미와 범위

| 항목 | C++ 후보가 유지하는 의미 |
|---|---|
| 호출 소유자 | 한 제어 작업 또는 잠금 아래에서 호출한다. 통신 callback·ISR와 동시 접근해도 안전한 객체는 아니다 |
| 검사 순서 | 시계 → 제어 tick 공백 → 피드백 신선도 → 명령 만료 → 모터 무진행. 수신은 내용 분기 → 세션 → 임대 → 준비/소유권/순번 → DRIVE 값 → 원천 만료 |
| watchdog | 통신 수신·offer·link poll은 제어 tick 시각을 갱신하지 않는다. 늦게 풀린 프레임에도 원래 임대/원천 만료를 사용한다 |
| 정지/재허가 | 목표 속도 0, 조향 유지, 소유권/임대/이전 순번 폐기. 물리 버튼 해제·재연결만으로 재무장하지 않으며 CLEAR와 ARM을 별도로 요구한다 |
| 피드백 | 단일 모터축 **28카운트/회전·15:1·지름65 mm**를 명시한 사례가 현행 기준이다. 평균 바퀴 환산 속도만 얻으며 모터축 평균 0은 실제 차체/좌우 정지 증명이 아니다 |
| legacy | 기존 좌우 모드도 과거 대조용으로 유지한다. `ReceiverConfig{}`의 기본은 이전 참조값이므로 현행 차량용 어댑터는 모터축 모드·28/15/0.0325를 명시해야 한다 |
| PC 결합 시험 | FrameLink → CommandParser → 수신기를 같은 작업에서 순서대로 호출한다. driver의 vector/문자열/JSON 출력은 PC 검사기이며, 고정 크기 펌웨어 큐나 전기 출력 구현이 아니다 |

수신기만으로 모터가 돌거나 정지하는 것은 아니다. 실제 출력 작업·독립 watchdog·물리 STOP 경로를 구현해야 하며, 통신 task가 멎을 때 제어 task까지 함께 멎지 않는지는 MCU에서 별도로 확인해야 한다.

### 15.2 Python과 의도적으로 다른 경계

- `now_us`는 0..`INT64_MAX-2×10^7` 범위만 허용한다. 상한 밖은 `invalid_clock`이며 임대/시간 합산의 여유를 확보한다. Python은 2^63−1까지 허용한다.
- generation/임대 순번을 int64 끝에서 재사용하지 않는다. `counter_exhausted` 고장으로 고정하며 새 부팅 식별자를 갖춘 재생성이 필요하다. Python의 임의 정밀 증가와 구별한다.
- boot 입력은 null이 아닌 포인터, 1~64 코드포인트, 각 값 U+10FFFF 이하를 요구한다. 구성/boot가 잘못된 객체는 `FAULT/invalid_config`로 시작하고 offer/ARM을 열지 않는다. 잘못된 구성을 고쳐 재생성해야 하며 물리 버튼 눌림/해제로도 정상 구성으로 바뀌지 않는다.
- PC용 카운터 끝값 주입 함수는 `ARENA_ACTUATION_TESTING` 빌드에만 존재한다. 실제 통합 빌드에서 정의하지 않는다.
- 링크 세션 경계는 참조 정책과 같게 로컬 STOP(`software_stop`)으로 고정하며 별도 사유는 넣지 않았다. Python의 상세 `deadline_diagnostic` dict는 옮기지 않았고 횟수만 둔다.

### 15.3 독립 API 검토에서 발견한 결함과 수정

정상 이벤트 대조 통과만으로 공개 API의 잘못된 호출까지 검증한 것은 아니었다. [수정 전 8사례](../../artifacts/validation/2026-10-22/actuation_receiver_api_v1/README.md)의 6개는 exit 1이었고, `freshness_invalid_time`은 exit 0이어도 UBSan 오류가 있었다. 정상 ARM→DRIVE→STOP 1개만 오류 없이 통과했다.

| 발견 | 수정 |
|---|---|
| 잘못된 feedback enum·구성·빈/긴 boot 입력이 초기 거절되지 않음, null boot는 ASan SEGV | 생성자 자체에서 구성/boot를 검사하고 복구 불가 구성 고장으로 보존 |
| 공개 `fresh_feedback(INT64_MIN)`의 signed overflow | 유효 시각·순서 확인 뒤에만 차분 |
| 세대 한계에서 CLEAR는 실제 FAULT인데 응답만 수락 | 전환 뒤 세대 소진도 확인해 실패 응답; 소진 뒤 offer도 거절 |
| 시험용 세대/임대 주입 함수가 일반 빌드에 공개 | 시험 매크로로 격리하고 일반 빌드의 비노출을 컴파일 검사 |

[v2](../../artifacts/validation/2026-10-22/actuation_receiver_cpp_v2/README.md)는 API 검사를 포함한 관련 225회귀와 ASan/UBSan·O2 각각 이벤트 8,680개/receive 3,667건의 불일치 0을 확인했다. 그러나 물리 STOP 코드가 바뀐 뒤 옛 줄을 찾던 변이체 검사기 때문에 전체 묶음은 실패했다. [v3](../../artifacts/validation/2026-10-22/actuation_receiver_cpp_v3/README.md)에서도 사전 사례 문자열 오기로 14통과/1실패가 나왔으며 두 실패는 그대로 보존한다.

최종 [v4](../../artifacts/validation/2026-10-22/actuation_receiver_cpp_v4/README.md)는 변이 주입 위치 사전 회귀 **15개**, 변이체 **13/13**, O2 직접 API·일반 빌드 검사 **10개**, 전체 **3,635회귀**를 통과했다. 실제 종료 0·실행 전후 소스 불변이다. v2 바이너리 8개와 입력 헤더/driver·README 11개가 같은 해시임을 확인해 재사용했으며, 정상 코어의 동일 export를 새 시험처럼 다시 세지 않았다. 정상 코어 sanitizer 오류는 없고, 일부러 정수 넘침을 넣은 변이체에서만 예상한 UBSan 오류가 검출됐다.

### 15.4 남은 실물 이관 조건

이번 구현은 **PC 의미 대조 후보**다. ROS 런타임 연결·ESP32 툴체인·PCNT/타이머·ISR/큐 동시성·최악 실행시간·실물 UART/USB 지연·MD20A/서보 출력·전기 STOP은 미검증이다. parser의 `from_chars` 내부 할당/MCU 지원 여부도 §14의 미확인 상태를 유지한다. `firmware_verified=false`, `ready_to_drive=false`이며 모터·센서·물성·보호 조건·운영 timeout은 바꾸지 않았다.

다음은 기존 [VND018의 CH343P USB-UART 우선안](ESP32_BUDGET_REVIEW_2026_10_02.md#usb-연결-검토---102-후속)을 기준으로 부팅/단절 사건, 타임스탬프 위치·발행 크기/빈도를 확인하는 준비와 측정이다. 직접 USB는 별도 경로이며 포트 이름만 보고 서로 바꾸지 않는다. 현행 673 B offer를 50 Hz로 보낼 수 있다는 가정이나 30 ms 시험 timeout을 실차 값으로 채택하지 않는다. 장치와 실측이 없을 때에는 입력 계약·대조 명령·부품 확인 항목까지만 준비하며 같은 합성 시험을 무한히 늘리지 않는다.

### 15.5 실물 연결 전 인수인계 순서

아래는 **미실행 준비 순서**다. 보드나 모터가 이미 연결되어 있다는 뜻이 아니며, 새 펌웨어·핀 배치·운영 통신 속도를 확정하지 않는다. 기존 [부품 확인 질문](../HARDWARE_AND_RULE_QUESTIONS.md)과 [실물 입력 감사](HARDWARE_CAPTURE_AUDIT_2026_10_13.md)를 재사용한다.

| 순서 | 준비·확인할 것 | 통과 근거 / 중단 조건 |
|---|---|---|
| 1. 실물·빌드 식별 | VND018 N16R8 납품 판본과 기존 YD V1.4 참고 회로도 대조, 사용 포트·보드·툴체인/라이브러리·빌드 옵션 기록 | CH343P UART0와 S3 직접 USB를 구별한다. `std::from_chars` 지원/수치 경계, 실제 객체 크기·스택·내부 할당은 목표 툴체인에서 별도 확인한다. PC 바이너리를 펌웨어로 배포하지 않는다 |
| 2. 구동을 분리한 연결 | 모터 구동을 연결하지 않은 상태에서 UART 수신/타이머/로그 어댑터부터 작성. 부팅 ID, 단조 시각, 제어 task 소유자와 유한 큐를 명시 | 모터축 모드·28카운트·15:1·반지름0.0325 m를 명시한다. `ReceiverConfig{}` legacy 기본값이나 PC의 vector 큐를 그대로 쓰지 않는다. `ARENA_ACTUATION_TESTING`은 운영 빌드에서 금지 |
| 3. 전송/세션 고장 | 포트 열기·닫기·DTR/RTS·보드 재부팅·케이블 단절·호스트 중단, 부분 프레임·지연·큐 포화·재연결 | 호스트가 본 USB 단절이 MCU UART에 자동 전달된다고 가정하지 않는다. 단절 사건을 못 받으면 명령 만료/독립 watchdog이 출력 허가를 닫아야 한다. 이전 boot/임대 명령 거절, 명시 CLEAR→별도 ARM 확인 |
| 4. 시간·처리량 | 실제 프레임 크기/빈도·방향별 byte 수, 수신/해석/제어 반영 시각, 큐 최고 점유·폐기 수, 제어 tick 간격·최대 실행시간·리셋 사유 기록 | PC 단조 시각과 MCU 시각은 별도 보존한다. 기존 왕복 시각/기록 감사 도구에 맞춰 변환하며 두 시계를 그냥 빼지 않는다. 참조 200/100/50 Hz를 실측 보장으로 쓰지 않는다 |
| 5. 출력·독립 정지 | 전기 담당자와 출력 OFF·물리 버튼·제어 task 고착·원격 STOP을 확인한 뒤 제한 구동 시험 | ACK/목표 속도 0과 실제 전기 출력/차체 정지를 별도로 측정한다. 단일 모터축 0도 차체 정지 증명이 아니다. MCU 재시작·통신 복구만으로 자동 구동하지 않는다 |

프레임 payload 상한 1536 B·호출 청크 상한 256 B는 현재 PC 코어 계약이다. 큐 포화 때 임의로 오래된 DRIVE를 늦게 재생하거나 보호 경로를 우회하지 않는다. 조립 timeout은 직렬화 시간(최대 1544 B, 460800 baud·8N1에서도 약 33.51 ms)과 실측 지연을 대조해야 하므로 **30 ms 시험값을 복사하지 않는다**. 반대로 제어/피드백 watchdog을 느슨하게 해 부하 시험을 통과시키지도 않는다. 처리량이 맞지 않으면 프레임 내용·빈도·통신 경로의 설계를 수정하고 같은 계약 벡터로 다시 대조한다.

이 단계에서 남길 최소 증거는 소스/설정 해시·보드/툴체인 식별·원시 시각/순번·입출력 크기·큐/실행시간 계측·고장별 판정이다. 실제 센서 입력은 기존 C2 정규화 계약을 사용하고 합성값과 구별한다. 최초 실물 통합 전까지 `firmware_verified=false`, `ready_to_drive=false`를 유지한다.

## 16. 10/22 후속 — ESP32-S3 목표 컴파일과 검사 앱 링크

§15 이후 보드 없이 확인할 수 있는 **목표 툴체인 호환성**까지 진행했다. [Xtensa 객체 검사](../../artifacts/validation/2026-10-22/actuation_xtensa_probe_v1/README.md)와 [SDK 검사 앱 빌드](../../artifacts/validation/2026-10-22/actuation_idf_smoke_v3/README.md)는 별개 증거다. 운영 ROS·차량 구동은 변경하지 않았다.

Espressif 공식 ESP-IDF **v6.1**, commit `fff9895c82d744c7237be8847347bdd1b07c6643`와 권장 Xtensa **esp-15.2.0_20251204**를 사용자 캐시에 격리했다. 컴파일러 압축 파일의 공식 SHA-256을 대조했다. 이는 호환성 검사에 사용할 고정 버전이지 최종 프레임워크를 팀 합의 없이 확정한 것이 아니다. C++17·예외/RTTI 비활성으로 빌드했으며 [공식 C++ 안내](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32s3/api-guides/cplusplus.html)와 [빌드 안내](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32s3/api-guides/build-system.html)를 대조했다.

| 확인 단계 | 실제 결과 | 확인하지 않은 것 |
|---|---|---|
| 목표 객체/ABI | ESP32-S3 ELF32 Xtensa 객체 생성. `FrameDecoder` 1816 B, `FrameLink` 1888 B, `CommandParser` 3656 B, `Command` 1144 B, `Receiver` 1616 B. binary64 정적 검사와 일반 빌드 시험 API 비노출 통과 | 전체 앱 메모리·task stack·실행 비용 |
| 표준 라이브러리 | 대상 libstdc++에 double `std::from_chars` 구현 존재, 실제 SDK ELF에도 링크됨 | 대상에서 수치 경계 실행·내부 동적 할당·소요 시간 |
| 검사 앱 | [`firmware_smoke`](../../src/arena_vehicle_interface/firmware_smoke/README.md) ELF/BIN 생성·소스 해시 불변. BIN 173,408 B, IDF DIRAM 배치 46,447 B | 보드 부팅·실행·전기 출력·실차 성능 |
| 공유 사례 | 단일 모터축 28/15/65 mm, STOP·CLEAR→ARM·tick 만료·숫자 경계 등 20개. 호스트 sanitizer/size 최적화 pytest **2개 통과** | Xtensa에서 사례가 실행된 것은 아님 |

검사 앱은 고정 합성 입력을 한 번 처리하고 진단을 출력하는 구조다. parser 작업 공간은 전역에 두며 main stack 8192 B는 검사값이다. SDK의 기본 flash 2 MB·PSRAM 비활성은 실제 N16R8 전체 구성/핀 배치에 대한 결정이 아니다. 고정 boot `smoke-only`를 실물 세션에 복사하지 않는다. UART 명령 수신·타이머 제어 task·PCNT·IMU·모터 PWM/DIR·서보·독립 출력 감시는 구현하지 않았다.

SDK 준비의 v1 경로 누락·v2 ensurepip 부재 실패는 보존했다. v3 전용 Python 환경으로 해결했으며 시스템 Python/ROS·Jetson을 변경하지 않았다. 공백 포함 저장소 경로는 SDK component 제한 때문에 새 공백 없는 소스 사본에서 빌드하고 원본/사본 근거를 보존했다. ELF의 `nm -u`에 남은 SDK `-u` 이름 3개를 숨기지 않았으며 relocation은 없다. 빌드 성공을 실제 실행 성공으로 확대하지 않는다.

다음은 기존 링크 정책과 수신 코어를 **구동 출력 없이** 결합하는 연결부다. 단절/재연결은 세대를 폐기하고 CLEAR→별도 ARM을 유지해야 한다. 링크 poll/수신으로 제어 tick을 대신하거나 늦은 프레임 수명을 늘리지 않는다. 실제 출력 준비 신호·직렬 통신/시간 동시성은 이후 어댑터에서 별도로 구현·측정하며, 이번 단계에서도 `firmware_verified=false`, `ready_to_drive=false`다.

## 17. 10/22 후속 — 단일 소유자 수신 연결부

[`ActuationIngress`](../../src/arena_vehicle_interface/portable/actuation_ingress.hpp)는 기존 프레임·JSON·상태기계를 고정 크기 연결부로 결합한다. PC 대조 driver의 vector 큐를 펌웨어에 옮기지 않고 완성된 프레임을 동기 처리한다. receiver와 연결부는 같은 작업/잠금에서 직렬 호출하며, 진단 callback은 재진입하거나 receiver를 변경하지 않는다.

기존 정책대로 연결 종료·놓친 단절·링크 고장에서는 부분 바이트를 폐기하고 필요하면 로컬 STOP을 넣는다. 이미 물리 STOP/FAULT 상태이면 사유를 덮어쓰지 않는다. 단절·연결 복구만으로 CLEAR/ARM을 만들지 않으며, 조립 만료 뒤 복구한 프레임도 원래 임대/원천 만료로 판단한다. 구성 오류와 signed 시각 범위 밖 입력은 연결부 재생성 전까지 거절한다. 엄격 JSON 실패와 무관 envelope는 receiver로 보내지 않지만 잘못된 명령 객체는 receiver의 원래 판정 순서로 보낸다.

[v1](../../artifacts/validation/2026-10-22/actuation_ingress_v1/README.md)에서 Python 참조 링크 정책 10조건과 envelope/Unicode request/비객체·중복 JSON·잘못된 구성/시각·다중 프레임을 ASan/UBSan과 `-Os`로 대조했다. 공유 앱 사례를 보강한 [v2](../../artifacts/validation/2026-10-22/actuation_ingress_v2/README.md)도 **52개 관련 회귀 통과**, exit 0·소스 불변이다. 기존 전체 3,635회귀와 별개이며 이번 연결부 추가 뒤 전체 저장소 회귀를 다시 돌린 것은 아니다.

[SDK v4](../../artifacts/validation/2026-10-22/actuation_idf_smoke_v4/README.md)는 기존 코어 20개 + 연결부 10개 합성 사례를 포함한 대상 ELF/BIN 링크에 성공했다. BIN 177,184 B·DIRAM 정적 배치 55,919 B. v3 바이너리/소스/실패를 보존했다. 큰 parser/연결부는 정적 공간에 두었지만 실제 실행 stack/할당/마감은 미검증이다.

이는 **수신 요청 상태를 계산하는 연결부**이지 출력 허가 장치가 아니다. ROS의 `output_ready`/AppliedGuard에 해당하는 실물 출력 준비 감시, UART RX task/유한 큐, 제어 tick·PCNT·PWM/DIR·서보·물리 STOP은 아직 연결하지 않았다. observer의 실행 시간도 제한하지 않는다. 보드에서 합성 사례를 실행한 적이 없으며 `firmware_verified=false`, `ready_to_drive=false`를 유지한다.

## 18. 10/22 후속 — offer 발생 빈도의 정정과 다음 재개

§11의 단순 예산은 `send_offer`가 **주기 20 ms 외에도 `on_tx`의 각 command 객체 뒤에 호출되는 점**을 누락했다. [`audit_actuation_callback_budget.py`](../../scripts/audit_actuation_callback_budget.py)가 실제 콜백을 노드 없이 합성 시간으로 호출해 확인했다. [증거](../../artifacts/validation/2026-10-22/actuation_callback_budget_v1/README.md): 1초·제어 tick 200 Hz·피드백 100 Hz·입력 50 Hz, 관련 **20회귀·종료 0·소스 불변**.

| 조건 | MCU offer/s | ACK/s | MCU→host B/s | 460800 baud·8N1 부하 | 921600 부하 |
|---|---:|---:|---:|---:|---:|
| 무명령 대기 | 50 | 0 | 30,445 | 0.661 | 0.330 |
| 정상 DRIVE 50 Hz | **100** | 0 | **65,818** | **1.428** | 0.714 |
| 이전 wire를 50 Hz로 반복 배송 | 100 | 0 | 64,442 | 1.398 | 0.699 |
| 모든 DRIVE에 request 포함 | 100 | 50 | 70,818 | 1.537 | 0.768 |

정상 DRIVE에서 host→MCU는 13,089 B/s다. 반복 배송은 실제 host 정책이 아닌 별도 고장 fixture이며 49개 거절·`command_expired/FAULT_LATCHED`를 확인했다. 즉 **거절해도 offer 송신 비용은 남지만 명령 수명은 늘지 않았다**. 상태 ROS 토픽·진단·센서·USB/CPU/큐 지연은 이 직렬 예산에서 제외했다. 출력 준비를 참으로 대체한 fixture이며 물리 출력이나 독립 보호 경로를 시험한 것이 아니다.

결론은 기존 ROS byte-array 연결이 고장났다는 것이 아니라, **현재 콜백을 UART로 그대로 이관하면 460800도 이 표본의 지속 송신에 부족하다**는 것이다. 921600의 부하가 1 미만이어도 보드 지원/지연/마감 보장은 아니다. baud·timeout·운영 Python/ROS를 자동 변경하지 않았다.

사용자의 마무리 요청으로 여기서 중단한다. **다음 재개는 구동 출력 없는 송신 정책 설계**다: 주기 offer와 사건 상태/ACK·상세 진단의 역할을 분리하고, 작고 유한한 송신 버퍼·상태 병합/우선순위·원래 발행 시각 보존을 설계한다. 병합으로 STOP/ACK를 잃거나 오래된 DRIVE를 새것으로 포장하지 않으며 임대/보호 시간 한도를 늘려 부하를 숨기지 않는다. 이번에는 송신 정책을 구현/활성화하지 않았다. 이후가 실제 UART/타이머·독립 출력 감시 연결과 보드 측정이며, G1 동일 실패 주행·G2/고속화·Jetson 변경으로 확장하지 않는다.

## 19. 10/22 후속 — 구동 출력 없는 송신 정책 후보

§18의 재개 순서대로 MCU→호스트 송신 정책을 **PC 검증 후보**로 구현했다. [`actuation_tx.hpp`](../../src/arena_vehicle_interface/portable/actuation_tx.hpp)의 `ActuationTxScheduler`가 수신기 상태를 유한 크기 프레임으로 바꾸고, 호출자가 준 sink에 부분 쓰기로 내보낸다. 운영 Python/ROS(`actuation_ros.py`)·기본값·baud/timeout·보호 한도는 바꾸지 않았다. 독립 보호 `/drive`→보호층→`/drive/safe`, STOP/CLEAR→별도 ARM, 임대/원천 TTL·피드백·제어 watchdog, 단일 모터축 28카운트·15:1·65 mm를 그대로 사용한다.

### 19.1 역할 분리와 호스트가 실제로 읽는 필드

기존 `ActuationHost`는 offer에서 정수 `generation`·`lease`·`lease_issued_us`, 문자열 `boot`·`state`, `owner` 키를 확인하고(`on_rx`), `output_ready`가 참일 때만 DRIVE를 만든다(`on_drive`). `CommandProducer`는 `boot`·`generation`·`lease`·`owner`·`state`만 읽는다. 따라서 상태 전체를 offer마다 반복할 필요가 없다.

| 종류 | 내용 | 언제 | 저장·병합 |
|---|---|---|---|
| `offer` | `type,v,boot,generation,state,owner,lease,lease_issued_us,lease_expires_us,output_ready` | 주기(시험 20 ms) + **세대가 바뀔 때만** 앞당김. 명령 수신마다 보내지 않음 | 플래그 하나. 임대는 프레임 내용을 고정하는 순간 발행 |
| `ack` | 기존과 같은 `{"type","request","accepted","reason"}` 바이트 형식 | 요청 ID가 있는 수신 명령 판정 뒤 | 큐 8개. STOP 결과 우선, 넘치면 명시 계수 후 폐기 |
| `state` | `boot,generation,state,reason,changes` | 세대/고정 상태가 바뀔 때(같은 세대 ARMED↔ACTIVE 제외) | 최신 하나로 병합, 병합한 관측 횟수를 `changes`로 표시 |
| `diag` | 수용 순번·목표/조향·만료·피드백·카운트·진단 계수 | 저주기(시험 100 ms), 최저 우선 | 최신 하나. 실수는 정수 micro 단위(`*_umps`, `steering_urad`) |

compact offer가 호스트 소비 필드를 빠뜨리지 않는지는 실제 `ActuationHost.on_rx/on_drive/control/watch`를 노드 없이 호출한 폐루프로 확인했다(§19.4). 빠진 status 필드(목표·조향·바퀴 속도·카운트·만료·`estop_held` 등)는 diag로, 정지 사유는 state로 옮겼다. ROS의 `/actuation/status`는 가상 MCU 쪽 별도 토픽이며 이 wire 계산에 포함하지 않는다.

### 19.2 유한 저장·소유권·시각 계약

- **소유자:** receiver와 같은 작업(또는 잠금)에서만 호출한다. offer가 임대 표를 갱신하므로 receiver를 비상수 참조로 받는다. ISR/통신 callback에서 직접 부르지 않는다.
- **저장:** ack 8개(요청 64 코드포인트), 전송 중 프레임 1개(최대 1,544 B), 나머지는 플래그·시각·계수뿐이다. 동적 할당 없음(코어 코드). pump 호출당 내용 고정은 2개 이하로 작업량을 묶었다. x86-64 `sizeof` 4,296 B.
- **고정 시점:** 이전 프레임을 sink가 다 받은 뒤에만 다음 내용을 고정한다. 고정한 프레임은 0 B/부분 수락이 반복돼도 끝까지 같은 바이트를 보내며, 그 사이 STOP·세대 변화가 와도 교체하지 않는다. 새 상태는 다음 프레임(세대 변화 offer → state)으로 나간다. 큐에 머문 과거 임대·시각을 새 상태로 재포장하지 않는다. sink가 막힌 동안에는 임대가 추가 발행되지 않는다.
- **세션 경계:** 링크 단절·놓친 단절에서 전송 중 꼬리와 ack 큐를 버리고, 끊긴 동안 쓰지 않는다. 수신 쪽(ingress)이 넣은 로컬 STOP을 다음 세대 offer/state로 알린다. 송신 객체는 CLEAR/ARM을 만들지 않는다.
- **영구 고장(침묵):** escape가 필요한 boot(인쇄 가능 ASCII·따옴표/역슬래시 제외만 허용), 정책값 범위 밖, 시계 역행, 요청보다 큰 수락값을 돌려준 sink. offer가 끊기면 호스트 임대가 만료되어 fail-closed가 된다.
- **최악 크기:** 같은 인코더 경로를 값 무시 writer로 따라가 계산했다. ack 856 B, state 247 B, offer 1,081 B, diag 1,062 B로 payload 상한 1,536 B 안이다. owner/request 64 코드포인트가 모두 astral이면 각 770 B가 된다.
- **시계:** `lease_issued_us/lease_expires_us`는 MCU 단조 시계다. 기존 호스트 `fresh_offer`는 호스트 시각에서 `lease_issued_us`를 빼며, 이번 감사도 두 쪽이 같은 합성 시계를 쓰는 fixture에서만 이를 허용했다. **실물에서는 두 시계를 그냥 빼지 않는다.** 수신 시각에 임대 길이를 더하는 것만으로도 신선도를 보장할 수 없다. 송신 큐·선로에 머문 시간이 이미 포함되므로 보수적인 시계 변환/왕복·전송 지연 상한을 함께 설계해야 한다. MCU의 원래 만료 판정은 유지하며 이 호스트 변경은 아직 하지 않았다.

### 19.3 우선순위·포화와 보장하지 않는 것

우선순위는 ack(STOP 결과 먼저) > 세대 변화 offer > state > 주기 offer > diag다. ack/state가 연속 2개에 닿았고 offer가 기다리면 offer를 먼저 보낸다. 선로 바이트 시간 `line_ns_per_byte`를 주면 diag는 **그 프레임 자체의 순수 직렬화 시간이 다음 offer 예정 시각 전 여유에 들어갈 때만** 시작한다. 이미 sink/FIFO에 들어간 바이트·실제 task 지연까지 포함한 도착 마감 보장은 아니다. 아직 한 바이트도 보내지 않은 내용을 버리는 것이므로 프레임 교체가 아니다.

- ack 큐가 차면 새 STOP 결과는 가장 오래된 비STOP 결과를 축출하고, 큐가 STOP 결과로만 차 있거나 새 결과가 비STOP이면 새 결과를 버린다. 호스트는 기존 wall timeout으로 "결과 미확인"을 받는다. **STOP은 수신 단계에서 이미 적용되며 ack 유실이 정지를 취소하지 않는다.** ACK accepted는 출력 허가가 아니며, offer의 `output_ready`는 출력 단계가 알려 준 값을 그대로 싣는다.
- 무한한 사건을 모두 전달하지 않는다. state는 최신 상태와 병합 횟수만, diag는 최선 노력이다. 포화 때 명령·임대 수명을 늘려 통과시키지 않으며, offer가 늦어지면 수신기 만료가 목표 0·고장 고정을 만든다.

### 19.4 결과

[감사·회귀 증거](../../artifacts/validation/2026-10-22/actuation_tx_policy_v2/README.md): C++ 정책을 lock-step driver로 호출하고, 8N1 선로의 바이트 시각을 분수 µs로 계산했다. MCU 송신 HW FIFO 128 B·pump 250 µs는 **가정**이며 호스트 송신 버퍼는 무제한으로 두었다. 실제 `ActuationHost` 콜백이 compact offer를 받아 ARM·DRIVE·STOP·CLEAR를 만들었다. §18과 같은 1초·tick 200 Hz·피드백 100 Hz·DRIVE 50 Hz·32자 식별자 조건이며 출력 준비는 참으로 대체했다. 기존(no-policy) 수치는 같은 실행에서 §18의 실제 콜백 감사를 다시 계산한 값이다.

| 조건(460800) | 기존 offer/ack per s | 기존 MCU→host B/s | 후보 offer/ack/state/diag per s | 후보 MCU→host B/s | 후보 부하 | 최종 상태(기존=후보) |
|---|---:|---:|---:|---:|---:|---|
| 무명령 대기 | 50 / 0 | 30,445 | 50 / 0 / 0 / 10 | 16,082 | 0.349 | DISARMED |
| DRIVE 50 Hz | 100 / 0 | 65,818 | 50 / 0 / 0 / 10 | **17,662** | **0.383** | ACTIVE |
| 이전 wire 반복 | 100 / 0 | 64,442 | 50 / 0 / 1 / 10 | 16,842 | 0.365 | FAULT_LATCHED `command_expired`, 거절 49 |
| 모든 DRIVE에 request | 100 / 50 | 70,818 | 50 / 50 / 0 / 10 | 22,662 | 0.492 | ACTIVE |

host→MCU는 13,092 B/s(기존 13,089)로 DRIVE 형식을 바꾸지 않았다. 기존 trace를 같은 460800 선로에 무제한 큐로 넣으면 DRIVE 50 Hz의 마지막 프레임 완료 지연이 1초 동안 448 ms까지 커진다(부하 1.428의 적체). 후보는 큐가 유한하고 offer 간격이 20 ms로 유지됐다. 460800 DRIVE 50 Hz의 최대 지연은 offer 대기 0·직렬화 5.0 ms, diag는 다음 offer 뒤로 미뤄져 trigger→도착 28.8 ms였다. request 사례의 ack는 trigger→도착 최대 9.2 ms다.

| baud | 최악 프레임 직렬화 ack/state/offer/diag (ms) | 1,544 B 프레임 | offer 대기 산술 상한* | FIFO 128 B 배출 |
|---:|---|---:|---:|---:|
| 115200 | 75.0 / 22.1 / 94.5 / 92.9 | 134.0 | 244.5 | 11.1 |
| 230400 | 37.5 / 11.1 / 47.3 / 46.4 | 67.0 | 122.3 | 5.6 |
| 460800 | 18.8 / 5.5 / 23.6 / 23.2 | 33.5 | 61.1 | 2.8 |
| 921600 | 9.4 / 2.8 / 11.8 / 11.6 | 16.8 | 30.6 | 1.4 |

\* 전송 중 최악 프레임 하나 + 연속 사건 2개(최악 크기)의 순수 직렬화. 최대 문자열 ack가 연속으로 오는 경우의 상한이며, 정상 32자 표본은 이보다 훨씬 작다. pump 주기·FIFO·CPU 지연은 더해야 하고 마감 보장이 아니다.

| 민감도(DRIVE 50 Hz) | 결과 |
|---|---|
| 921600 | ACTIVE, 부하 0.192, offer 20 ms |
| 460800·pump 1 ms / diag 끔 | 둘 다 ACTIVE·offer 20 ms. diag 끔 11,483 B/s |
| 230400(diag 끼워 넣기 판단) | ACTIVE, 부하 0.498/0.568(MCU→host/host→MCU), offer 20 ms. **diag는 70회 미뤄져 1초 동안 0개**(offer 8.8 ms + diag 26.7 ms가 20 ms 주기에 들어가지 않음) |
| 230400(판단 끔) | ACTIVE지만 긴 diag가 offer를 가려 offer 40개·최대 간격 40.25 ms. 판단을 넣은 근거로 보존 |
| 115200(판단 있음/없음, request 포함) | 모두 **FAULT_LATCHED `command_expired`(fail-closed)**. ARM에 쓴 임대(발행 220.5 ms)의 만료 320.5 ms 전에 첫 DRIVE(320 ms 송신, 261 B 직렬화 22.6 ms)가 도착하지 못해 `session` 거절됐다. 정상 50 Hz 운용 근거가 아니다 |

ACTIVE 중 전송 중 프레임에서 단절한 폐루프에서는 꼬리 101 B를 버리고 로컬 STOP(`software_stop`)으로 고정했다. 단절 50 ms 동안 프레임 0개, 재연결 뒤 첫 프레임은 새 세대 offer→state였고 호스트 DRIVE 송신 0·수신기 ESTOP 유지였다. 명시 CLEAR 뒤 250 ms 동안 DISARMED·DRIVE 0을 유지했고 별도 ARM 뒤에만 ACTIVE가 됐다. 이어서 호스트 STOP 서비스가 ack로 성공을 받았고 250 ms 동안 DRIVE 0·ESTOP을 유지했다.

회귀·변이체: [v1](../../artifacts/validation/2026-10-22/actuation_tx_policy_v1/README.md)은 관련 130회귀 통과 뒤 변이체 12개 중 `stop_ack_not_first` 1개를 검출하지 못해 **exit 2**로 끝났다(큐가 STOP 결과로만 찬 경우만 시험해 FIFO와 구별되지 않음). 실패를 보존하고 비STOP 3개 뒤 STOP 1개의 순서 시험을 추가했다. [v2](../../artifacts/validation/2026-10-22/actuation_tx_policy_v2/README.md): **exit 0**, 소스 해시 차이 0 B, 감사 위반 0, 관련 132회귀 통과, 변이체 **12/12** 검출(원본 사본 60 통과), 전체 **3,736 passed·19 skipped**. skip 19개는 기존 C++ 코어 대조 시험이 사전 빌드 바이너리 환경변수(`ARENA_*_CPP_BINARY` 등)를 요구해 건너뛴 것이며, 해당 코어 헤더 5개는 v4/연결부 v2 해시와 같다. [대상 빌드 v5](../../artifacts/validation/2026-10-22/actuation_idf_smoke_v5/README.md)는 코어 20 + 연결부 10 + 송신 9 합성 사례 앱의 ESP32-S3 ELF/BIN 링크에 성공했다(BIN 187,984 B, DIRAM 정적 83,807 B). 대상에서 실행하지 않았다.

### 19.5 발견·남은 조건

- **ARM 출력 준비 관문 미이관(§19 시험 당시):** Python `VirtualMCU.on_tx`는 `output_ready`가 거짓이면 ARM을 `output_not_ready`로 거절하지만 당시 portable 연결부에는 없었다. 송신 후보만으로 해결되지 않으며 아래 §20에서 보완한다. §19/v2 결과를 관문까지 검사한 결과로 소급하지 않는다.
- **호스트 쪽 변경 미실시:** 기존 호스트는 compact offer를 그대로 소비했지만 `fresh_offer`의 시계 차감은 합성 공유 시계 fixture에서만 성립한다. 실물용 수신 시각 기반 상한, `state`/`diag` 소비, 명시 선택 옵션은 구현하지 않았다. 코드상 호스트는 STOP ack를 받은 뒤에도 새 offer 전까지 기존 offer로 DRIVE를 만들 수 있으며, 그 DRIVE는 MCU 세대 검사에서 거절된다.
- **선로 여유:** 115200은 ARM 직후 첫 DRIVE부터 fail-closed였다. 230400은 offer는 유지했지만 현재 diag 크기로는 diag가 나가지 않는다. 정상 50 Hz 처리량 개선은 460800 합성 선로에서만 입증했다.
- **측정하지 않은 것:** 실제 UART HW FIFO/드라이버 링 버퍼(크게 두면 임대 노화가 늘어남), TX 완료 알림과 pump 주기, 대상 CPU 실행시간/WCET·stack, USB-UART 지연, 보드에서의 합성 사례 실행, 전기 STOP. 한도 안의 산술/시뮬레이터 결과는 하드웨어 지원이나 마감 보장이 아니다.

### 19.6 다음 순서

1. 수신 쪽 ARM `output_ready` 관문을 portable 연결부에 이관하고 기존 Python 판정과 대조한다.
2. 별도 시계·전송 중 노화를 포함한 보수적인 호스트 사용 상한을 설계하고, 명시 선택형 compact offer 소비와 `state`/`diag` 기록을 연결한다. 수신 시각에서 임대 전체 수명을 다시 시작하지 않으며 기본 ROS 경로·timeout은 유지한다.
3. 실제 UART TX(완료 알림→소유 작업 pump, 작은 링 버퍼)와 독립 출력 준비 감시를 구동 출력 없이 연결하고 보드에서 바이트 수·FIFO 점유·지연·실행시간을 측정한다. 그 전까지 `firmware_verified=false`, `ready_to_drive=false`.

## 20. 10/22 후속 — ARM 출력 준비 관문

송신 후보 검토에서 발견한 수신 연결부 누락을 보완했다. Claude 사용량 경고 후 GPT가 구현·검토를 이어받았고 GPT-6 Luna가 검사 실행·회수를 맡았다. Python/ROS 운영 코드는 바꾸지 않았다.

`ActuationIngress.feed`와 `poll`은 마지막 인수로 **이번 호출 시점의 `output_ready`**를 받는다. 생략하면 false이며 연결부는 이전 호출의 true를 저장하지 않는다. 실제 감시가 정상임을 증명하는 코드가 아니라, 향후 독립 출력 감시의 판정을 소비하는 관문이다.

- 엄격 JSON과 envelope 분리 후 raw `kind == ARM`이고 준비되지 않았다면 버전·세션·정지 준비 검사보다 먼저 `output_not_ready`로 거절한다. 실제 Python `VirtualMCU.on_tx`와 같은 순서다. 수신기 상태/세대·소유권·순번·임대·제어 tick을 갱신하지 않는다.
- `IngressDelivery.receiver_called=false`, `output_gate_rejected=true`로 구분한다. 요청 ID가 유효하면 ACK 큐가 이 거절 결과도 싣는다. ACK가 없어 호스트가 불필요하게 만료만 기다리던 경로를 만들지 않는다. 큐 포화의 기존 미확인 한계는 유지한다.
- STOP·CLEAR를 준비값으로 막지 않는다. 준비 false→true, 단절 복구, CLEAR만으로 자동 ARM하지 않으며 별도 ARM 명령이 필요하다.
- 분할 수신·만료 drain은 완성 명령을 판정하는 호출의 준비값을 사용한다. 수신기 시계가 이미 더 앞선 경우에는 먼저 기존 시계 고장을 드러내어 준비 거절이 시간 역행을 가리지 않게 했다. 기존 portable 시계 고장 이름/해제 의미는 유지한다.
- ARM 관문은 ACTIVE 중 출력 고착·준비값 상실을 차단하는 watchdog을 대신하지 않는다. 출력 상태·세대·취득 시각을 확인하는 실물 독립 감시기와 전기 STOP은 여전히 미구현/미측정이다.

[첫 관련 회귀](../../artifacts/validation/2026-10-22/actuation_arm_gate_v1/README.md): **168개 통과·내부 exit 0·소스 불변**, 실제 Python 콜백 대조 및 송신 감사 위반 0. 정상 DRIVE 50 Hz의 송신량·최종 상태는 §19 후보와 같다. 외부 래퍼 인용 오류는 내부 검사 성공과 구별해 보존했다. 공유 검사 앱의 연결부 사례는 10→12개이며 코어 20·송신 9개는 유지한다.

[최종 전체 회귀](../../artifacts/validation/2026-10-22/actuation_arm_gate_final_v1/README.md)는 기존 코어 대조 바이너리 6개를 새로 빌드·명시하여 **3,791 passed·0 skipped·exit 0·소스 불변**이다. [대상 링크 v6](../../artifacts/validation/2026-10-22/actuation_idf_smoke_v6/README.md)도 exit 0·소스 불변이며 BIN 188,256 B, 정적 DIRAM 83,807 B다. 새 시계 후보를 추가하기 전의 결과이며 보드에서 실행하지 않았다.

## 21. 후속 설계 — 별도 시계의 offer를 호스트에서 사용하기 전

현재 ROS는 합성 `/clock`를 공유한다. 실물 host 단조 ns와 MCU 부팅별 단조 µs는 offset과 주파수가 다르므로 `host_now - lease_issued_us`를 그대로 쓰지 않는다. 반대로 `수신 시각 + 임대 길이`도 전송 중 노화를 지워 버린다. 수신기가 원래 임대 만료를 검사하므로 오래된 명령이 실제로 재허가되는 것은 아니지만, 호스트가 낡은 권한으로 명령을 반복 생성해 거절·정지를 부르는 문제가 남는다.

다음은 운영 연결 전 **오프라인 판정 후보**로 검토한다. 기존 [C2 왕복 구간 추정](HARDWARE_CAPTURE_AUDIT_2026_10_13.md#6-왕복-시각으로-구하는-조건부-시간-구간)을 재사용하며 RTT/2를 편도 지연으로 채택하지 않는다.

1. offer의 `boot`·`generation`·임대 ID·발행/만료·형식, 호스트 epoch와 왕복 근거의 boot를 명시 대조한다. 임대 길이/식별자가 malformed이면 판정을 만들지 않는다. 유한한 시계 변환 유효 구간 밖으로 외삽하지 않는다.
2. 발행 시각의 host 구간 `I=[Ilo,Ihi]`와 만료 시각의 host 구간 `E=[Elo,Ehi]`를 구한다. 결정 시각 `N=[Nlo,Nhi]`에서 모든 가능한 발행 나이가 정책 상한 이내이고, 예상 송신 지연 상한을 더한 도착 상한이 **Elo보다 엄격히 작을 때만** 조건부 사용 가능으로 표시한다. 정확한 만료 시각은 이미 거절이다.
3. 수신 시각은 같은 host epoch의 별도 근거이며 임대 시계의 출발점이 아니다. 수신 이후 오래된 입력, 미래/역행/epoch 변경, 지연 상한 미확정 또는 넓은 불확실성은 거절/미확인으로 남긴다.
4. DRIVE까지 연결할 때에는 원래 `/drive/safe` 생성 시각의 ROS→host 변환과 source 나이 상한도 필요하다. 명령 생산기의 단조 watermark·ARM 이후 표본 경계, MCU의 `min(임대 만료, 임대 발행 + TTL − source_age)`와 MCU↔host 주파수 단위도 함께 대조한다. **offer가 신선하다는 결과만으로 DRIVE를 허가하지 않는다.**
5. 후보는 wire/ROS 노드/명령 생산기와 미연결로 둔다. 실측하지 않은 timestamp 오차·주파수/편도 지연 상한을 숫자 하나로 확정하지 않으며, 실물 통합 시 한계 검증 없이 운영 timeout/임대를 늘리지 않는다.

10/23에 [`audit_actuation_offer_clock.py`](../../scripts/audit_actuation_offer_clock.py)와 [검사](../../tests/test_actuation_offer_clock.py)로 위 시간 조건을 구현했다. [관련 217회귀·합성 10조건](../../artifacts/validation/2026-10-23/actuation_offer_clock_v1/README.md)은 exit 0·소스 불변으로 통과했다. 큰 host 시각에서 정수 정밀도, ±200 ppm 범위의 참 시계와 보수적 판정, 정확한 만료 경계도 확인했다. `timestamp_policy_satisfied=true`는 선언된 가정 아래 시간 조건만 충족한다는 뜻이다. 정지 상태에서도 시간은 신선할 수 있으며 결과는 항상 `ready_to_drive=false`다. 세션 근거·원천 입력 시각·실제 편도 지연/주파수는 여전히 별도 의무다.

## 22. 10/23 후속 — 호스트의 compact 메시지 보관 후보

[`actuation_host_observer.py`](../../scripts/actuation_host_observer.py)는 프레이밍/CRC를 통과한 네 종류 payload를 받는 **출력 없는 단일 소유자 후보**다. `scripts/`에 두었으며 운영 ROS 노드·serial·명령 생산기에 연결하지 않았다. host/MCU 시계를 합치거나 API를 호출하는 것만으로 ARM/DRIVE를 만들지 않는다.

- `offer`, `state`, `diag`, `ack`의 정확한 스키마·문자열/정수/크기 한도를 검사한다. 미확인 필드를 버리고 진행하지 않는다. 최신 네 종류와 기본 32개(설정 최대 128개) 이력만 보관하며 이력 손실을 계수한다. 저장 payload는 불변 문자열이고 외부에는 사본을 반환한다.
- 외부에서 확인한 MCU boot·host epoch·연결 ID를 명시한다. 다른 boot를 자동 채택하지 않고 재연결에는 새 객체/ID를 요구한다. 현재 연결의 host 수신 시계가 역행하면 보관본을 비우고 닫는다. 이 ID 확인은 인증·장치 식별·실물 재부팅 검출을 대신하지 않는다.
- 같은 boot의 generation·lease ID·발행 시각을 되돌리지 않는다. 새 세대 state/diag를 받으면 새 offer가 오기 전이라도 이전 offer를 제거한다. 새 세대의 거절된 offer도 옛 세대 허가를 복원하지 않는다.
- **ACK·state·diag·중복 offer는 원래 offer 수신 시각/유효기간을 갱신하지 않는다.** 수신 시각의 구간 불확실성도 보존한다. 시간 사용 검토 시에만 §21을 호출한다.
- ACK에는 MCU boot/generation이 없다. 따라서 기록은 하되 `ack_correlated=false`이며 서비스 결과 확정·ARM barrier 생성에 쓰지 않는다. 실제 호스트 연결은 요청 ID·연결별 미확인 결과·STOP 요청 즉시 캐시 무효화·명시 재허가를 별도로 설계해야 한다. 보관기만으로 늦은 ACK를 안전하게 소비한다고 선언하지 않는다.

[326 관련 회귀](../../artifacts/validation/2026-10-23/actuation_host_observer_v1/README.md)는 exit 0·소스 불변·skip 0이다. 실제 C++ 송신기를 호스트 프로세스로 빌드하여 네 프레임 종류와 ARM→STOP 세대 전환을 대조했다. 기존 CRC decoder의 분할 입력도 확인했으나 실제 UART·USB·장치 실행은 아니다. 모든 결과의 `ready_to_drive=false`, `runtime_connected=false`를 유지한다.

## 23. 10/23 후속 — 원천 입력 나이의 MCU 단위 환산 후보

MCU 시각 `m`(µs)와 host 시각 `h`(ns)의 관계를 `h = offset + 1000·rate·m`, 선언된 비율 범위를 `[rmin,rmax]`로 둔다. `source_age_us`와 TTL은 지속 시간이므로 offset을 빼는 것만으로 해결되지 않는다. 시계가 조금 느리거나 빠르면 같은 숫자 µs가 나타내는 실제 시간이 달라진다.

[`actuation_source_time_review.py`](../../scripts/actuation_source_time_review.py)는 이미 host 단조 시각 구간으로 옮겼다고 선언한 원래 입력 `S=[Slo,Shi]`, 결정 시각 `N=[Nlo,Nhi]`, ARM 기준 구간을 받는 **오프라인 산술 검토**다. ROS stamp를 변환하거나 원천 시각을 수신 시각으로 바꾸지는 않는다.

1. 원래 입력이 미래일 수 있거나 ARM 기준보다 앞설 수 있으면 조건 미충족이다. 원천 시계 epoch도 일치해야 한다.
2. 보수적 나이 `A = ceil((Nhi−Slo)/(1000·rmin))`를 MCU µs로 구한다. 음수 나이를 0으로 고치지 않는다.
3. 원천 수명 정책 `Thost`에 대해 `T = floor(Thost/(1000·rmax))`를 구하고 임대 길이/기존 상한 이하로 제한한다. 나이는 위로, 수명은 아래로 반올림하며 1 tick도 표현하지 못하면 거절한다.
4. 기존 수신기 식 `D = min(lease_expires_us, lease_issued_us + T − A)`를 그대로 대조한다. 예상 도착 상한이 가장 이른 D보다 엄격히 앞서고, 가능한 가장 늦은 D도 원래 입력의 가장 이른 `Slo+Thost`를 넘지 않아야 한다. fresh offer 하나로 오래된 원천을 구제하지 않는다.

이 후보는 패킷을 만들지 않는다. `source_watermark_checked=false`, `source_host_mapping_verified=false`, `arm_ack_correlated=false`, `command_created=false`, `ready_to_drive=false`다. 실제 연결에는 ROS→host 취득 위치·점프/배속 감시, 중복/역행 source watermark, 요청별 ARM 상관 확인, 실제 시계/전송 상한 검증이 남는다. 기본 `CommandProducer`와 ROS 경로를 바꾸지 않는다.

[관련 291회귀](../../artifacts/validation/2026-10-23/actuation_source_time_v1/README.md)는 exit 0·skip 0·소스 불변이다. ±200 ppm 범위의 합성 참 시계에서 허용한 만료가 원천 TTL을 넘지 않았으며, Python 단일 모터축 수신기의 실제 만료식과도 대조했다. ±200 ppm은 실물 오차 확정값이 아니다. 합성 참 자세/속도는 사용하지 않았으며 시험용 시계 정답은 검증 fixture의 평가에만 썼다.

## 24. 10/23 후속 — 별도 시계의 제한 프레임 교환

위 시간 산술·호스트 보관과 C++ 송수신 후보를 [`audit_actuation_separate_clock.py`](../../scripts/audit_actuation_separate_clock.py)의 PC 시험기로 대조했다. 하나의 ARM 요청, 새 세대/소유자 offer와 양의 ACK 확인, 최대 하나의 DRIVE, 최종 STOP만 수행한다. 운영 요청 관리자·연속 주행 제어기 구현이 아니다.

[299 관련 회귀·8조건](../../artifacts/validation/2026-10-23/actuation_separate_clock_v1/README.md)은 exit 0·skip 0·소스 불변으로 통과했다. 기준·±200 ppm·편도 지연 비대칭 반전에서는 시험기가 DRIVE를 만들어 C++ 수신기가 ACTIVE로 전이했다. 선언한 전방 지연 상한 증가·120 ms 늦은 판단·120 ms 오래된 원천·ARM ACK 유실에서는 DRIVE를 만들지 않았다. 늦은 판단 사례는 이미 수신기에서 만료 고장이 고정되어 있었고, 나머지 미송신 사례는 ARMED였다. 모두 마지막 STOP 후 ESTOP_LATCHED·목표 0이었다.

460800·8N1과 host 추가 송신 큐 없음, 합성 0카운트 피드백(5 ms 이하 tick), 출력 준비 참은 시험 가정이다. 실제 출력 비활성 연결부를 ARM한 것이 아니다. 가능한 시계 집합은 왕복 시각에서 구하고, 원천 나이와 수명을 보수적으로 환산한다. 참 시계는 시험기 평가에만 사용하며 RTT/2를 편도 지연으로 단정하지 않는다. 실물 통신에서는 실제 큐/스케줄링·시각 취득 오차를 포함해야 한다.

이 결과로 연속 50 Hz 처리량, 실제 USB-UART, 요청 상관 관리, 보드 출력 또는 실차 안전을 확인하지 않았다. 기존 운영 ROS 경로와 timeout은 바꾸지 않았으며 `firmware_verified=false`, `ready_to_drive=false`를 유지한다.

## 25. 10/23 후속 — 출력 비활성 통신 연결부

[`actuation_bench.hpp`](../../src/arena_vehicle_interface/portable/actuation_bench.hpp)의 `ActuationBenchEndpoint`는 기존 수신기·연결부·송신 정책을 한 소유자로 묶었다. 향후 출력 없이 통신만 연결할 때 사용하는 준비물이며 UART/GPIO/타이머 드라이버는 없다.

- 출력 준비는 항상 false이며 mutable receiver/TX나 피드백 주입 API를 제공하지 않는다. ARM/DRIVE를 허용하지 않고 STOP은 받아 고정한다. 피드백이 없으므로 CLEAR도 성공하지 않는다.
- `feed/poll/pump`와 `control_tick`을 구별한다. 통신을 자주 호출했다고 제어 tick이 갱신되지 않는다. 모든 호출은 같은 소유 작업에서 수행하며 ISR/콜백에서 객체로 재진입하지 않는다.
- 링크 세션 경계는 송신 중 꼬리/ACK를 폐기하고 재연결이 구동 허가를 만들지 않는다. sink는 비차단 부분 쓰기 계약이며 실제 UART 버퍼/완료 시각/동시성은 별도로 검증해야 한다.

[관련 230회귀](../../artifacts/validation/2026-10-23/actuation_bench_v1/README.md)는 exit 0·skip 0·소스 불변이다. 공유 벤치 15사례를 호스트 ASan/UBSan과 크기 최적화로 검사했다. 앱의 코어 20·수신 연결부 12·송신 9사례는 보존한다. [전체 회귀](../../artifacts/validation/2026-10-23/actuation_bench_final_v1/README.md)는 **3,923통과·skip 0**, [ESP32-S3 링크 v7](../../artifacts/validation/2026-10-23/actuation_idf_smoke_v7/README.md)도 exit 0·소스 불변이다. BIN 193,264 B·정적 DIRAM 131,039 B는 여러 endpoint/배열을 담은 검사 앱의 값이다. 실제 장치에 업로드하지 않았다. 이 전체 회귀는 다음 §26 요청 후보를 추가하기 전의 결과다.

## 26. 후속 설계 — 요청 응답과 재출발 증거

[`actuation_host_session.py`](../../scripts/actuation_host_session.py)는 §22의 메시지 보관기를 소유하고 명시 제어 요청의 상관 확인을 추가하는 PC 후보다. 운영 ROS/serial에 연결하지 않으며 DRIVE를 만들지 않는다. 명시 제어의 envelope는 반환하지만 전송하지 않으므로 요청 기록은 실제 송신 확인이 아니다. ACK 자체에는 boot/generation이 없으므로 다음 조건을 함께 요구한다.

1. 연결마다 새 요청 접두부와 단조 순번을 사용하고, 동시에 미확인 제어 요청은 하나만 허용한다. ACK의 요청 ID·정확한 결과 형식·기대 이유·수신 시각을 대조한다. 연결 단절/호스트 시계 역행/MCU boot 변경 시 폐쇄하며 새 인스턴스가 필요하다.
2. ARM 성공은 양의 ACK만으로 확정하지 않는다. 요청한 세대에서 정확히 한 세대 증가한 새 offer의 ARMED/ACTIVE·자기 소유권·출력 준비를 함께 확인한다. ACK와 offer의 도착 순서는 바뀔 수 있으나 둘 다 요청 이후·요청 기한 이전이어야 한다. 새 원천 입력은 두 증거를 모두 확보한 뒤에 생성되어야 한다.
3. STOP 요청 즉시 기존 ARM 근거와 미완료 ARM을 무효화한다. STOP 응답 유실/지연·늦은 ARM ACK·오래된 offer가 이를 복원하지 못한다. 호스트의 STOP 의도도 유지하며 명시 CLEAR의 ACK와 DISARMED 새 세대 확인 전에는 ARM을 받지 않는다. CLEAR는 DISARMED 확인까지만이며 뒤에 별도 ARM이 필요하다.
4. timeout은 성공/실패 확정이 아닌 **결과 미확인**이다. 자동 재시도/자동 ARM을 하지 않는다. 잘못된 ACK, 다른 세대/소유권/출력 준비 상실, 원천 시각 중복/역행은 새 구동 권한을 만들지 않는다.
5. 이 증거와 §21~23 시간 조건을 모두 만족하더라도 실제 출력 감시·취득 시각·링크 정리/상한 검증은 별개다. 후보의 결과는 `ready_to_drive=false`이며 물리 정지/실차 안전 판정이 아니다. 기존 공유 시계 ROS 경로의 동작은 이번 후보 구현으로 바꾸지 않는다.

단일 소유 작업이 처리/요청/판정의 host 시각을 순서대로 제공해야 한다. `observe(received=..., processed=...)`는 원래 수신 구간과 처리 구간을 구별한다. 처리 구간을 생략하면 같은 사건으로 취급한다. 명시할 때 수신 상한이 처리 하한을 넘으면 순서를 확정할 수 없어 닫는다. 큐에 머문 수신 시각을 새로 쓰지 않으며 ARM 기준은 두 증거를 모두 **처리한 구간**이다. 기한 전 수신·기한 후 처리도 결과 미확인이다. 입력 프레임의 수신 순서 역전과 처리 시계 역행은 폐쇄한다. 실제 드라이버의 읽기/큐·시계 취득 어댑터는 여전히 별도다. 소스 시각은 이미 host 구간으로 변환됐다는 외부 전제이며, 성공한 검토의 watermark는 STOP/CLEAR/ARM으로 초기화하지 않는다.

[v1](../../artifacts/validation/2026-10-23/actuation_host_session_v1/README.md)은 테스트 보조 함수의 `request` 인수 중복으로 176통과·1실패였다. 해당 반례는 후보에 도달하지 못했으며 실패와 소스를 보존했다. 인수 이름을 수정하고 STOP 의도 유지 등의 반례를 더한 [v2](../../artifacts/validation/2026-10-23/actuation_host_session_v2/README.md)는 183 관련 회귀를 통과했다. 수신/처리 구간을 구별한 [후속 검증](../../artifacts/validation/2026-10-23/actuation_host_session_mutants_v2/README.md)은 **190 관련 회귀·skip 0·exit 0·소스 불변**, 변이체 **13/13** 검출이다. 첫 변이체 실행의 TypeError 1건은 검출 성공으로 세지 않고 보존했다. 실제 C++ ARM→STOP 교환도 PC에서 확인했다.

## 27. 10/23 후속 — 단일 모터 속도 PI의 C++ 이관

[`speed_pi.hpp`](../../src/arena_vehicle_interface/portable/speed_pi.hpp)는 모터축에서 환산한 평균 바퀴 속도 하나를 받아 정규화 `MotorIntent`를 계산한다. 좌우 바퀴를 독립 관측/제어하지 않는다. 기존 Python 참조의 Kp 0.6·Ki 1.2, 최대 dt 0.05 s, 포화 쪽 적분 억제·정지/비허가 시 초기화를 옮겼다. 이 값은 가상 모터 참조값이며 실차 튜닝값이 아니다.

### 산술 반례와 수정

입력의 유한성만 확인해도 차분·곱셈 결과가 넘칠 수 있다. 수정 전 [반례 3건](../../artifacts/validation/2026-10-23/speed_pi_numeric_v1/README.md)은 모두 출력 0 대신 **effort +1·제동 요청 false**였다. 정상 입력 검사 1건은 통과했다. 이 극단값은 API 고장 시험이며 실제 센서에서 같은 값이 관측됐다는 뜻은 아니다.

Python 참조와 C++ 후보 모두 비유한 중간값에서 출력 0·제동 요청·적분 초기화로 처리한다. Python의 큰 정수→float 환산 예외도 포함한다. C++은 잘못된 설정이면 영구 무효 객체로 남으며 Python 생성자의 ValueError와 구분한다. 후보는 고정 크기 객체이며 ISR/통신 처리, ARM/STOP/watchdog, 실제 PWM/DIR 출력은 없다. 계산 오류가 사라지면 PI 자체는 다시 계산할 수 있으므로 **구동 고장 고정/명시 재허가는 외부 상태기계의 책임**이다.

[148 관련 회귀](../../artifacts/validation/2026-10-23/speed_pi_cpp_v1/README.md)는 exit 0·skip 0·소스 불변이다. ASan/UBSan과 크기 최적화 각각 무작위 순서 4,000표본 및 고장/초기화/경계값을 대조했으며 정상 범위는 수정 전 수식과도 일치했다. 공유 PI 12사례를 검사 앱에 추가해 호스트에서 실행했다. `-ffp-contract=off`, IEEE binary64를 사용하며 비유한 검사를 제거하는 fast-math/finite-math-only 구성은 컴파일에서 거절한다. 이어 [전체 4,072회귀](../../artifacts/validation/2026-10-23/actuation_pi_final_v1/README.md)·[ESP32-S3 링크 v8](../../artifacts/validation/2026-10-23/actuation_idf_smoke_v8/README.md)가 exit 0·소스 불변으로 끝났다. 대상 실행은 하지 않았으며 이 전체 회귀는 아래 제어 주기 결합 후보를 구현하기 전의 결과다.

### 출력 경계의 후속 조건

- PI 계산기의 `enabled=true`만으로 실물 구동을 허가하지 않는다. 수신기 상태·명령 만료·피드백 신선도·제어 tick·독립 출력 준비를 **같은 제어 주기**에서 확인한 뒤 계산해야 한다.
- STOP/CLEAR/ARM 등 세대 변경에서 이전 적분값을 넘기지 않는다. 오래된 상태 사본이나 통신 콜백이 제어 주기를 대신 갱신해서는 안 된다.
- 0목표의 제동 요청, 비허가의 출력 차단, 정상 감속의 음수 effort를 구분해 기록하되 실제 MD20A 역회전/전기 제동의 매핑으로 단정하지 않는다. 실제 게인·출력 준비·핀/전기 STOP은 미검증이다.
- Gazebo 내부 모터 PI는 여전히 기존 모델이다. 이번 Python 참조의 산술 수정과 C++ 이관을 실제 주행 제어 교체나 실차 성능 검증으로 확대하지 않는다.

### 다음 묶음의 수용 조건 — 출력 없는 제어 주기 결합

통신 객체를 실제 출력에 연결하기 전에, 수신기와 PI를 한 소유 작업에서 계산하는 **출력 없는 후보**를 준비한다. 실제 HAL/타이머가 아니라 사건 순서와 고장 경계만 검사한다.

1. 주기 진입 시 receiver의 시계·명령/피드백 만료를 먼저 검사한다. dt는 제어 주기의 원래 시각 차분으로 구하며 통신 도착 간격이나 재사용 상태 사본으로 대체하지 않는다.
2. 허가 세대가 바뀌거나 비허가/0목표가 되면 적분을 초기화한다. 이미 ACTIVE인 수신기에 계산기를 새로 붙이는 것은 정상 부팅 순서가 아니므로 재허가 전 출력하지 않는다.
3. 구동 허가 중 출력 준비 상실·잘못된 주기·PI 산술 고장은 출력 0과 수신기의 고장 고정으로 연결한다. 준비값/숫자가 다시 정상이 되는 것만으로 재출발하지 않는다. 기존 고장/물리 STOP 이유를 새 고장으로 덮지 않는다.
4. 고장 해제는 기존 정상 피드백/정지 준비와 명시 CLEAR→ARM을 요구한다. 새로운 API는 고장 고정만 가능하며 허가를 만드는 우회 경로를 두지 않는다.
5. 산출물은 정규화 계산 의도뿐이다. 실제 UART/엔코더 입력·PWM/DIR·서보·전기 STOP·독립 watchdog은 연결하지 않으며 `output_ready`의 기본은 false다. 제어 작업 자체가 멎으면 같은 작업의 소프트웨어는 출력을 끌 수 없다는 한계를 남긴다.

## 28. 10/23 후속 — 출력 없는 제어 주기 결합

[`motor_control_candidate.hpp`](../../src/arena_vehicle_interface/portable/motor_control_candidate.hpp)는 위 수용 조건에 따라 수신기와 PI를 같은 소유 작업에서 검사하는 후보다. 반환값은 시각·세대·상태·조향·정규화 모터 의도를 담는 `MotorCyclePreview`이며 출력 드라이버가 아니다. 운영 ROS/Gazebo·출력 비활성 벤치에는 연결하지 않았다.

수신기에 새 `report_control_fault` API를 추가했다. 이는 허가를 만드는 기능 없이 출력 준비 상실·PI 설정/주기/산술 고장을 기존 FAULT로 고정하며, 기존 ESTOP/FAULT 이유는 보존한다. 통신 콜백과 달리 후보의 tick에서만 PI 주기를 계산한다. 원래 수신기의 명령/피드백/제어 만료 한도는 그대로다. 다른 호출자가 receiver tick을 잘못 갱신하더라도 후보 자신의 dt 검사로 PI 공백을 거절한다.

최종 검토에서 v1/v2의 **중간 ARMED 직접 관측 강제 조건은 제거**했다. 이 후보는 비신뢰 상태 메시지를 받는 Gazebo `AppliedGuard`와 달리, 같은 소유 작업의 실제 receiver 객체를 읽는다. receiver가 명시 ARM·DRIVE를 이미 검증했어도 두 명령이 제어 주기 사이에 도착하면 v1/v2는 불필요하게 고정 정지했다. 변경본은 새 세대에서 적분을 초기화한 뒤 현재 목표를 계산한다. 중간에 STOP/CLEAR/ARM을 거쳐도 옛 적분은 넘기지 않는다. 반면 이미 ACTIVE인 수신기에 새 계산기를 붙이는 첫 tick, 고장 자동 재개, 오래된 입력은 계속 거절한다. 운영 `AppliedGuard`의 메시지/허가 관측 규칙을 완화한 것이 아니다.

dt는 마지막 실제 계산 주기와 현재 시각의 차분이며, 새 세대에서도 그 dt와 현재 끝점 오차로 PI를 계산한다. 중간 명령별 연속 구동 이력을 복원하는 물리 모델이 아니다. 실제 타이머/입력 반영 순서와 게인 튜닝은 보드 통합 때 확인해야 한다.

| 검사 | 실제 결과와 범위 |
|---|---|
| [v1](../../artifacts/validation/2026-10-23/motor_cycle_v1/README.md) | 제어 주기 20사례×두 빌드 및 기존 관련 검사 **188통과·skip 0·exit 0·소스 불변** |
| [v2](../../artifacts/validation/2026-10-23/motor_cycle_v2/README.md) | 25사례로 확장·공유 검사 앱 포함, 관련 **198통과·skip 0·exit 0·소스 불변**. 모터축 환산값/정상 감속·새 ARM 관측·외부 tick 공백·오래된 피드백 CLEAR 거절 추가 |
| v2 제한 변이 검사 | 정상 대조 통과, 결함 변이 **8/8**이 컴파일 성공 뒤 결합 검사에서 검출됨. 무관한 코어/송수신/벤치/PI 공유 검사는 유지. 이후 중간 ARMED 관측 강제 제거의 검증으로 소급하지 않음 |
| [최종 전체 회귀](../../artifacts/validation/2026-10-23/motor_cycle_final_v1/README.md) | 관측 강제 제거 판본. **4,122통과·skip 0·exit 0·소스 불변**. 변이 8종(`no_arm_observation` 대신 `no_generation_reset`) **8/8 검출**, 정상 대조 통과 |
| [ESP32-S3 링크 v9](../../artifacts/validation/2026-10-23/actuation_idf_smoke_v9/README.md) | 같은 소스 해시. exit 0·소스 불변, BIN 202,064 B·정적 DIRAM 131,039 B. `.su` 함수 프레임 최대 3,952 B는 실제 stack 아님. `ESP_ROM_ELF_DIR` 진단 2건·미해결 SDK 심볼 3개 보존. 대상 실행 없음 |

기본 준비값 false, 모터축 28카운트·15:1·65 mm, STOP/CLEAR/ARM 경계를 유지했다. 고장 정지에서는 적분을 버리고 준비가 돌아와도 자동 재개하지 않는다. 정상 음수 노력값은 **실제 역회전 신호가 아니다**. PC 합성 사건 검사이며 실제 출력 차단·타이머/ISR 동시성·독립 watchdog·MCU 실행 시간·실물 제동은 미검증이다.

## 29. 10/23 후속 — 출력 비활성 UART 주기 어댑터

§15.5의 2·3단계 중 **보드 없이 할 수 있는 부분**만 구현했다. [`actuation_uart_bench.hpp`](../../src/arena_vehicle_interface/portable/actuation_uart_bench.hpp)의 `UartBenchAdapter<Io, Clock>`가 §25의 `ActuationBenchEndpoint` 하나를 한 소유 작업에서 주기적으로 돌린다. 실제 SDK 연결과 목표 컴파일은 별도 [`firmware_bench`](../../src/arena_vehicle_interface/firmware_bench/README.md) 앱에 두었다. 기존 smoke 앱의 사례 수·구성은 바꾸지 않고 공용 헤더만 재사용했다. 이 묶음의 설계·구현·문서는 Claude Opus 5.5, 단순 실행·회수는 Claude Haiku 5.5 별칭 보조가 맡았다.

### 29.1 경계와 순서

- 준비값은 항상 false이고 endpoint는 const로만 노출한다. 피드백 주입·준비값 설정·ARM 성공 경로가 없으며, UART로 받은 ARM/DRIVE는 기존 관문에서 거절된다. PI 후보(§27~28)와 결합하지 않는다.
- 한 주기: 단조 시각 → 기한이면 `control_tick` 1회 → (전 주기에서 미룬 세션 경계) → [사건 확인 → 읽기 → 사건 확인 → `feed`] 반복 → `poll` → `pump` 1회 → 종료 시각 재읽기. 제어 tick은 시간 기한으로만 호출하며 수신/송신 경로는 tick을 호출하지 않는다. 늦으면 한 번만 tick하고 실제 간격과 놓친 기한 수를 기록한다. 지난 기한을 재생하지 않고, 한 주기 이상 늦으면 현재 시각에서 기한을 다시 잡는다. 대상 작업도 `xTaskDelayUntil`이 늦으면 기한을 현재로 바꾸고 한 tick 양보한다.
- 수신 시각은 **바이트가 고정 256 B 버퍼에 들어온 뒤 읽은 단조 시각**이며 선로 도착·원천 취득 시각이 아니다. 주기당 바이트(검사값 1024 B)·새 읽기 시작 시간 상한(첫 읽기 하나는 예외)이 있고 남은 바이트는 드라이버 버퍼에 둔다. 처리하지 못한 적체는 결국 RX 넘침 사건으로 드러나 세션 경계가 된다.
- 주기 실행시간은 송신·입력 폐기·단절 뒤 시각을 다시 읽은 `end_us`까지다. 이 종료 읽기의 역행도 시계 고장이다. 수신 버퍼 시각과 분리하며, `cycle_budget_us`는 새 읽기를 시작하지 않는 상한일 뿐 주기 전체 마감 보장이 아니다. 상한·제어 간격을 넘은 주기는 따로 센다.
- 사건 예산은 **한 주기 전체** 상한이다. 바이트를 드라이버에서 꺼내기 전 그때까지의 사건을 모두 확인해야 하고(예산 안에서 큐가 비지 않으면 읽지 않음), 새 읽기에는 읽은 뒤 확인용 예약분이 남아 있어야 한다. 읽은 뒤 오류 사건이 보이거나 예산 안에서 사건을 다 확인하지 못하면 그 조각은 연결부에 넣지 않고 세션을 끊는다. 큐가 비었다는 응답 직후 도착해 마지막 조회에 보인 사건도 내용 미확인으로 보며, 마지막 조회가 0일 때만 확인 완료다. 한 주기 경계는 한 번이며 두 번째 오류는 다음 주기 시작으로 미룬다.
- 넘침·RX 버퍼 가득 참·프레임/패리티 오류·break·미사용 사건과 **사건 큐 포화**를 세션 경계로 본다. v6.1 소스상 ISR은 큐가 가득 차면 사건을 조용히 버리므로 포화 자체를 유실 가능으로 취급했다. 소비자가 이 작업 하나뿐이면 직전 확인 이후의 포화는 다음 확인 진입 때까지 남으므로 꺼내기 전 진입 검사로 보고, 꺼내는 도중 다시 가득 차면 종료 검사로 본다. **두 검사 사이 가득 찼다가 우리가 꺼내 내려간 순간 포화는 볼 수 없어** 모든 사건 유실을 검출한다고 주장하지 않는다.
- 경계에서는 연결부 단절(부분 프레임·송신 꼬리·ack 폐기, 필요 시 로컬 STOP) → 사건 큐 비움 → RX 폐기 → 새 세션 순서다. 이미 전달한 완전 프레임은 되돌릴 수 없고 늦은 오류는 남은 부분 프레임만 버린다. 이미 TX FIFO에 넣은 바이트는 회수할 수 없다. 잘린 프레임은 호스트 CRC/재동기화가 버리지만 **FIFO에 남은 완전한 이전 프레임은 CRC를 통과**하므로 호스트의 요청 ID·세션·세대 계약(§22·§26)으로 배제해야 한다. 재연결은 CLEAR/ARM을 만들지 않는다.
- 무수신을 USB 단절로 추정하지 않는다. CH343P 쪽 USB 분리가 UART에 사건으로 오지 않을 수 있으므로 명령 만료·독립 watchdog이 맡아야 한다. 송신 0 B·부분 수락은 다음 주기로 넘기고, 오류(-1)·요청보다 큰 수락 보고·수신 오류·입력 폐기 실패·시각 역행/범위 밖은 영구 어댑터 고장이다. 고장 뒤에는 I/O·tick을 하지 않고 재부팅(새 객체)으로만 재개한다.

### 29.2 SDK 의미 확인과 외부 I/O 계획

로컬 ESP-IDF v6.1 소스를 대조했다. `uart_tx_chars`는 TX FIFO 빈 칸만 채워 바이트 수를 반환하며 TX ring 비사용 때만 쓰라고 명시한다. 다만 내부 tx 잠금을 `portMAX_DELAY`로 잡으므로 같은 포트를 다른 작업이 쓰지 않는 단일 소유가 비차단의 전제다. `uart_read_bytes(..., 0)`은 ring에 있는 바이트만 복사하고 잠금 경합도 -1이다. FIFO 넘침 때 ISR은 FIFO를 초기화하고 사건을 보낸다. `uart_driver_install`은 RX ring이 FIFO 128 B보다 커야 한다. v6.1의 `uart_set_pin`은 매크로이고 실제 기호는 `_uart_set_pin6`이다. 바이트가 ring에 들어오는 시점, 사건과 바이트의 비동기 순서, 큐 포화 때 사건 버림은 모두 **소스 해석이며 하드웨어에서 확인하지 않았다**.

`check_uart_plan`의 기본은 `enabled=false`다. 배선 확인 표시, 포트·TX/RX·baud 명시, 진단 콘솔 포트/핀과 분리, ESP32-S3 직접 USB(19/20)·flash/octal PSRAM(26~37)·strapping(0/3/45/46) 회피, 다른 기능에 배정한 핀과 충돌 없음이 필요하다. UART0는 ROM/부트로더 출력 침묵을 별도로 확인해야 한다. **기존 CH343P→UART0 우선안은 현재 SDK 기본 콘솔과 같은 포트**이므로 이 앱은 자동 선택하지 않는다. 콘솔 이동·부팅 출력 처리·다른 UART 사용 여부는 실물 판본 대조 뒤 결정할 하드웨어 질문으로 남긴다.

### 29.3 결과

| 검사 | 실제 결과와 범위 |
|---|---|
| [첫 전체 PC 회귀](../../artifacts/validation/2026-10-23/actuation_uart_bench_v1/README.md) | 아래 검토에서 결함 3건이 발견된 판본. **4,128통과·skip 0·exit 0·소스 불변**. 공유 가짜 I/O 23사례를 ASan/UBSan과 `-Os`에서 통과. 정적 경계 검사 포함 |
| 같은 실행의 변이 검사 | 정상 대조 통과, 결함 변이 **15/15**가 컴파일 성공 뒤 사례 판단 차이로 검출. tick 따라잡기·통신 주기 tick·상한 완화·포화/오류 무시·수신 시각 위장·부분 쓰기 완료 처리 등 |
| [대상 빌드 v1](../../artifacts/validation/2026-10-23/actuation_uart_bench_idf_v1/README.md) | 링크·이미지 생성 뒤 스크립트 기호 기대값 오류(`uart_set_pin`)로 **exit 1**. 실패 근거 보존 |
| [대상 빌드 v2](../../artifacts/validation/2026-10-23/actuation_uart_bench_idf_v2/README.md) | 기대값만 수정, 관련 6회귀 통과 뒤 **exit 0·소스 불변**. BIN 216,144 B·정적 DIRAM 105,022 B, 비활성 기본값에서도 SDK UART 함수 6개 링크 확인. 결함 판본의 빌드. 대상 실행 없음 |
| [검토 수정 v1](../../artifacts/validation/2026-10-23/actuation_uart_bench_review_v1/README.md) | 결함 재현 6/6(결함 판본 실패·수정본 통과), 관련 9회귀, 변이 22/22. 이후 가짜 I/O 수정으로 대체 |
| [대상 빌드 v3](../../artifacts/validation/2026-10-23/actuation_uart_bench_idf_v3/README.md) | exit 0이나 공유 가짜 I/O 재생성의 정적 프레임 **24,832 B** 발견(8,192 B main stack 초과 위험). 미채택·근거 보존 |
| [검토 수정 v2](../../artifacts/validation/2026-10-23/actuation_uart_bench_review_v2/README.md) | 최종 소스. 결함 판본 재현 **6건 실패(exit 1)**·수정본 0건, 관련 **9회귀 통과**(공유 32사례), 변이 **22/22**, exit 0·소스 불변 |
| [대상 빌드 v4](../../artifacts/validation/2026-10-23/actuation_uart_bench_idf_v4/README.md) | review_v2 소스 **exit 0·소스 불변**, BIN 219,904 B·정적 DIRAM 105,182 B, 정적 프레임 2 KiB 초과 0건(최대 496 B), SDK UART 함수 6개 링크. 이후 2차 검토 경쟁 발견. 대상 실행 없음 |
| [검토 수정 v3](../../artifacts/validation/2026-10-23/actuation_uart_bench_review_v3/README.md) | 관측된 pending 경쟁 수정 첫 실행. 재현·9회귀는 기대대로였으나 변이 스크립트 고정 문자열 오류로 **exit 1**. 근거 보존 |
| [검토 수정 v4](../../artifacts/validation/2026-10-23/actuation_uart_bench_review_v4/README.md) | 최종 소스. 재현기 7건: 첫 판본 7건 실패·직전 수정본 새 반례 1건만 실패·수정본 0건, 관련 **9회귀**(공유 33사례), 변이 **23/23**, exit 0·소스 불변 |
| [대상 빌드 v5](../../artifacts/validation/2026-10-23/actuation_uart_bench_idf_v5/README.md) | 최종 소스 **exit 0·소스 불변**, BIN 220,256 B·정적 DIRAM 105,182 B, 정적 프레임 2 KiB 초과 0건, SDK UART 함수 6개 링크. 대상 실행 없음 |

### 29.4 최종 검토에서 찾은 계약 불일치와 수정

GPT 최종 검토가 첫 판본에서 세 가지를 지적했다. (1) 사건을 먼저 꺼낸 뒤 포화를 확인해, 진입 때 정상 DATA로 가득 찬 큐(용량 16)를 8개 꺼낸 뒤 8로 보고 유실 가능성을 놓쳤다. (2) `max_events_per_cycle`이 확인 호출마다 다시 채워져 한 주기에 여러 배를 처리했고, 예산을 다 써 오류 여부를 모르는 채 읽은 바이트를 넣었다. (3) 종료 시각으로 마지막 읽기를 재사용해 송신·입력 폐기·단절 시간이 주기 실행시간에서 빠졌고, 그 사이 시각 역행도 다음 주기에서야 드러났다. 또한 대상 앱은 합성 자기 검사가 실패해도 계획이 준비되면 외부 I/O를 시작할 수 있었다.

첫 회귀 스냅샷의 헤더로 [재현기](../../scripts/uart_bench_defect_repro.cpp)를 빌드해 6건 실패를 보존하고, 위 29.1의 규칙으로 고쳤다. 자기 검사 실패 시 외부 I/O를 시작하지 않는 관문도 넣었다. 수정 중 공유 가짜 I/O 재생성이 대상에서 24,832 B 정적 프레임을 만드는 회귀를 v3 빌드에서 발견했다. 제자리 재생성과 빌드 스크립트의 2 KiB 초과 프레임 거절로 고쳤다. 전체 4,128회귀는 바뀐 파일이 이 묶음의 새 파일뿐이라 반복하지 않았다. 처음의 TX FIFO 문장(“호스트 CRC가 버린다”)은 완전한 이전 프레임에 대해 틀렸으므로 29.1처럼 정정했다.

2차 검토는 수정본의 `check_events`가 `empty || pending == 0`을 Clean으로 판정해, 큐가 비었다는 응답 직후 도착해 마지막 조회에 실제로 보인 사건을 무시할 수 있음을 지적했다. 읽은 뒤 확인에서 이 경로를 타면 오류 꼬리를 모른 채 조각을 넣는다. 마지막 조회가 0일 때만 Clean으로 고치고 필요 없어진 `empty`를 지웠다. 재현 반례를 추가해 직전 판본 실패·수정본 통과를 보존했다. 첫 재실행은 변이 스크립트가 지운 줄을 고정 문자열로 써서 실패했고 근거를 보존했다. 마지막 조회 이후 도착하는 사건은 여전히 다음 확인의 몫이다.

### 29.5 미검증 경계와 다음 단계

실제 UART 타이밍·ISR/드라이버 동시성·FIFO/ring 지연·작업 실행시간과 실제 stack·전기 신호·CH343P USB 사건은 검증하지 않았다. SDK의 첫 바이트·ring 읽기와 사건 비동기 순서는 소스 해석이며, 순간 포화 유실은 검출하지 못할 수 있다. 정적 프레임 검사·자기 검사 통과는 실제 stack 충분성이나 실물 동작 증거가 아니다. 제어 간격 5 ms·수신 상한·큐 길이·offer 20 ms·조립 기한 30 ms·`CONFIG_FREERTOS_HZ=1000`은 검사값이며 운영 baud/timeout이 아니다. 출력 준비 감시·PWM/DIR·물리 STOP은 여전히 없다. 소유 작업이 멎으면 같은 작업의 소프트웨어는 아무것도 할 수 없다.

다음은 보드가 있을 때의 §15.5 3·4단계다. 실물 판본·콘솔/부팅 출력·핀 배정 확인 → 사람이 승인한 계획으로 출력 없는 수신/송신 측정(방향별 바이트·큐 최고 점유·tick 간격·주기 실행시간·리셋 사유) → 포트 재개방/DTR·RTS/케이블 단절/보드 리셋 고장 시험 순서다. 보드 없이 같은 합성 사례를 더 늘리는 것은 이번 묶음 범위에서 멈춘다. `firmware_verified=false`, `ready_to_drive=false`.
