# UNO Q Jetson 호스트 준비 및 장치 없는 검증

> 경기 종료 후 분류: **현장 미사용 대체안/준비 이력**. 대회용 최종 기준은 [ESP32 소스 보존본](../releases/FINAL_ESP32_2026_11_14.md)이다. 아래 내용은 당시 오프라인 준비 기록이며 실차 적용 결과가 아니다.

2026-11-14. 장치는 꺼진 상태이며 이 작업에서는 SSH·직렬 포트·flash·실제 출력에 접근하지 않았다. Python 호스트, 가짜 UART/USB 서보 시험, 설치 후 사용할 명령을 준비했다. 실제 UNO Q 타깃 빌드 결과와 배포 절차는 [배포 문서](UNO_Q_DEPLOYMENT_2026_11_14.md), 배선은 [UNO Q 핀표](schematics/uno_q_pin_table_v1.md)를 따른다.

## 실행 경로

`scripts/uno_q_host.py`가 두 직렬 연결을 각각 한 작업에서 소유한다. MCU USART1과 USB 서보 모두 **1,000,000 baud, 8N1**이다. 공용 ESP32 `SerialIngress`의 921600 기본값은 변경하지 않았다. `pyserial`은 실제 실행할 때만 import한다. 기본 진단에는 ROS와 실측 motor profile이 필요하지 않으며 USB 서보 포트도 생략할 수 있다.

```bash
# 장치를 열지 않는 준비 확인
python3 scripts/uno_q_host.py --dry-run

# 설치·장치 경로 확인 후에만 사용하는 진단 예시; MCU_UART는 실제 확인한 경로
python3 scripts/uno_q_host.py --diagnostics --mcu-port "$MCU_UART" \
  --seconds 30 --output "$NEW_LOG_DIRECTORY"

# USB 서보도 읽으려면 --servo-port "$SERVO_USB" 추가
# 명시 torque OFF 쓰기/ACK 확인을 허용하려면 --release-servo 추가
# ROS 관측을 발행하려면 --ros 추가
```

기본 진단은 서보에 READ만 보낸다. 진단 중 서보 torque가 이미 ON이면 그 상태를 기록하며 자동으로 ON/OFF를 바꾸지 않는다. MCU에는 시작·정상 종료 시 소프트웨어 STOP을 보낸다. 이 ACK는 물리 정지를 입증하지 않는다.

운영은 `--ros --enable-drive-wire --installed-profile-dir ... --mcu-profile ... --mcu-port ... --servo-port ...`를 모두 명시해야 한다. host installed bundle은 기존 `runtime_installed_profile.py` 생성·검증 형식이며, MCU JSON은 firmware 빌드에 실제 사용한 `uno_q_profile_v1`이다. 엔코더 방향, DIR, duty, PI, 속도, 조향 상한, feedback 주기, assembly ID를 대조한다. MCU가 보고한 canonical SHA256까지 일치한 신선한 관측이 있어야 ARM 준비를 시작한다. 서로 다른 프로필은 문서 주의만 남기는 대신 코드에서 거절한다.

현재 저장소에는 확인된 motor PI/방향을 채운 운영 프로필이 없으므로, 이 문서가 해당 값을 대신하지 않는다. 별도 motor feedback 설정을 사용할 때는 MCU의 count에 이미 방향 부호가 적용되므로 host `encoder_sign=1`이어야 한다. 28 counts/motor revolution, gear ratio `2.5*81/17`, wheel radius `0.0325 m`도 현재 MCU 고정값과 대조한다.

## 출력과 만료 계약

- 기존 IA JSON 명령 및 `/drive → 독립 보호층 → /drive/safe`를 유지한다. 구독은 `/drive/safe` 하나이며 이 호스트는 테스트 명령을 직접 발행하지 않는다.
- ARM은 현재 서보 위치 READ → 같은 위치 HOLD 쓰기 ACK → torque ON ACK 순서다. 영점 2811로 자동 이동하지 않는다. 이후 MCU ARM의 ACK·새 세대 offer까지 확인한다.
- DRIVE의 원 ROS header 시각을 보존한다. MCU가 해당 `accepted_seq`를 관측으로 확인한 뒤에만 서보 목표를 보낸다. 목표 ACK가 확인된 seq와 MCU 현재 DRIVE seq가 정확히 일치해야 모터가 허용된다.
- 이미 ACK된 목표 count가 같고 서보 관측이 신선하면 중복 목표 쓰기 없이 적용 seq만 갱신한다. 다른 목표·불명 상태에는 이 생략을 적용하지 않는다.
- USB 요청은 한 번에 하나, enqueue부터 30 ms 제한이다. timeout·손상·다중 ACK·단절 뒤에는 자동 재시도하거나 재ARM하지 않는다. ST 응답에 transaction ID가 없으므로 무분별한 재시도로 늦은 ACK를 새 요청의 ACK로 취급하지 않는다.
- `ESV1`은 IA framing 안의 별도 49-byte payload, `<4s8sQQQIBq`이다. boot, generation, 현재 lease, boot 내 증가 seq, 관측 age, ready, 적용 DRIVE seq를 전달한다. MCU는 원 lease 발행 기준 최대 100 ms로 제한하며 host queue에 머문 프레임이 새 수명을 얻지 않는다. ARM의 단일 +1 세대 전환만 기존 기한을 유지한 carry를 허용한다.
- STOP/CLEAR/DISARM/FAULT/boot 변경은 목표·준비 상태를 폐기한다. host가 살아 있으면 가능한 현재 USB transaction을 마친 뒤 torque OFF를 확인한다. USB 단절·host 사망 시 **서보 torque OFF는 보장하지 못한다**. MCU PWM0와 서보 torque OFF는 별도 판정이다.

`config/hardware/uno_q_clock_policy.template.json`의 100 ms source TTL, 1000 ppm 상대 비율, ROS/단조시계 오차는 준비용 선언값이다. 실물 측정이나 전체 주행 안정성 증거가 아니다.

## 모터 단독 벤치

`--bench`는 별도 bench firmware에만 요청한다. runtime ARM/DRIVE, PI, 서보 torque ON, ESV 준비를 사용하지 않는다. 서보 전원을 분리한 상태에서 USB 서보 포트를 생략할 수 있다.

```bash
# 장치 없는 인수 검증 예시; 실제 실행은 --dry-run을 제거할 때만 가능
python3 scripts/uno_q_host.py --dry-run --bench \
  --bench-duty-permille 50 --bench-direction 0 --bench-duration-ms 1000
```

이 수치는 예시이며 `DIR=0`을 전진이라고 주장하지 않는다. duty 1–150 permille, 기간 1–5000 ms를 명시한다. startup STOP 확인 → 명시 CLEAR → raw count 연속 200 ms 불변 확인 → BEN ARM 관측 → PULSE 한 번 → 20 ms KEEPALIVE → `duration_complete` 관측 순서다. KEEPALIVE는 최초 종료 시각을 늘리지 않는다. 자동 반복이 없고, 수락/완료·HAL·관측 신선도에 문제가 있으면 STOP 종료한다. 펌웨어의 엔코더 진행·과속·독립 deadline 검사와 별개로 host가 물리 안전을 증명하지는 않는다.

## 관측 토픽과 원문

| 토픽 | 형식·범위 |
|---|---|
| `/hardware_runtime/imu` | 기존 gyro-only `sensor_msgs/Imu`; 다른 채널 unavailable |
| `/hardware_runtime/accel` | accel-only `sensor_msgs/Imu`, SI specific force, `lsm6dsox_sensor`; orientation/gyro covariance[0] = -1 |
| `/hardware_runtime/accel_meta` | `String` JSON; boot/sequence, sensor tick, MCU read 구간, 개별 FIFO 시각 추정·generation·receipt age 원형 유지 |
| `/hardware_runtime/steering_observation` | `String`; 실제 count, CAD 중앙 등가각, USB 실제 읽기 receipt, age, valid, `steering_sign_verified`, sign 근거. 바퀴각 실측으로 주장하지 않음 |
| `/hardware_runtime/encoder_observation` | `String`; 원 boot/generation/observation_seq/capture_us/motor_counts/raw_count, receipt·age·추정 stamp, HAL 유효성. 정지후보용이며 속도0/엔코더 방향을 만들어내지 않음 |
| `/actuation/status` | 기존 상태·선택 feedback 변환을 유지 |

가속도는 tag2 전용 FIFO clock에서 처리한다. gyro 결과나 원본 frame을 바꾸거나 이전 accel을 새로운 gyro 메시지에 섞지 않는다. `time_contract_verified=false`를 유지한다. Encoder의 같은 원 샘플 재보고에는 새로운 freshness를 주지 않는다. `events.jsonl`, `uart_rx.bin`, `uart_tx.bin`, 등록 인수와 종료 요약을 기록하며 원 IMU 프레임은 UART 원문에 남는다. 설치 부호가 없는 진단에서도 raw count 불변을 WAIT 정지후보에 사용할 수 있으나 이것을 바퀴속도0으로 발행하지 않는다.

## 장치 없는 검증 결과

현재 전용 host/bench/profile/observations 및 기존 IA session/link/runner/FIFO 회귀는 220개 통과했다. 가짜 USB/MCU 검사는 read→hold ACK→torque ON ACK, 중립 자동이동 없음, MCU 수락 이전 목표 없음, 목표 ACK 이전 motor inhibit, 동일 목표 생략, STOP 경합 및 아직 USB 작업이 가져가지 않은 요청 취소, timeout·늦은 ACK·boot 변경·프로필 불일치, 원천 시각 보존, 단발 bench와 원 관측 보존을 확인한다.

5 ms host tick으로 설정한 결정적 fixture의 PWM inhibit 구간:

| 가짜 USB 응답 지연 / MCU 수락보고 지연 | 새 목표 | 같은 ACK 목표 |
|---|---:|---:|
| 0 / 0 ms | 6 ms | 1 ms |
| 5 / 0 ms | 6 ms | 1 ms |
| 10 / 10 ms | 21 ms | 11 ms |
| 20 / 20 ms | 41 ms | 26 ms |

이는 소프트웨어 fixture의 수치이며 Jetson 스케줄러·USB·UART·MCU 실측이 아니다. MCU offer 40 ms, 수락변경 즉시/최대 20 ms compact 보고, 100 ms lease와 실제 통합 처리율·IMU drop·PWM 공백은 연결 후 확인해야 한다. 보호 조건을 완화하여 지연을 감추지 않는다.
