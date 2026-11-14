# UNO Q MCU 프로토콜

운영 신호는 USART1 D0/PB7 RX, D1/PB6 TX, **1,000,000 baud 8N1**, no-pull이다. Arduino Linux Bridge/Router와 USB Serial을 통하지 않는다. 기존 `portable/actuation_*`의 IA JSON 명령·임대·ACK는 변경하지 않는다. 물리 STOP GPIO는 없다. 소프트웨어 STOP은 유지한다.

## ESV1 외부 서보 증거

IA payload는 little-endian Python `<4s8sQQQIBq`의 **49 B**, IA 헤더/CRC를 포함하면 **57 B**이다.

| offset | 자료형 | 의미 |
|---:|---|---|
| 0 | 4 bytes | `ESV1` |
| 4 | 8 ASCII | MCU offer boot hex |
| 12 | u64 | generation |
| 20 | u64 | MCU가 발행한 현재 lease ID |
| 28 | u64 | MCU 부팅 전체에서 단조 증가하는 status_seq, ≤INT64_MAX |
| 36 | u32 | 송신 시점 USB 서보 증거의 경과 시간 µs |
| 40 | u8 | ready 0/1 |
| 41 | i64 | USB goal ACK가 완료된 DRIVE seq; 아직 없으면 -1 |

MCU 유효 기한은 `min(lease.expires_us, lease.issued_us + 100000 - observation_age_us)`다. 수신 시각으로 새 100 ms를 부여하지 않는다. age≥100000, 잘못된 boot/generation/lease, 만료, 반복·역순 seq는 거절한다. 오래된 ready=false도 최신 ready=true를 무효화하지 않는다. 반대로 **새 seq의 유효한 false는 즉시 준비 해제**다. malformed 프레임/CRC 오류는 준비를 해제한다. lease를 갱신해도 host가 같은 측정의 age를 계속 증가시켜야 하므로 오래된 관측의 수명이 연장되지 않는다.

실제로 수락된 `explicit_arm` 한 번에만 이전 세대의 증거를 새 세대로 넘기며 **원래 절대 기한**을 보존한다. DISARM/CLEAR/STOP/FAULT는 증거를 넘기지 않는다. status_seq는 세대 전환에서도 초기화하지 않는다. 단일 소유 UART이라는 현 구성에서 boot+generation+lease가 세션 경계이며 별도 사용자 인증 프로토콜은 아니다.

MCU가 수락한 **현재 DRIVE accepted_seq와 USB goal ACK seq가 같을 때만** PWM을 허용한다. 새 DRIVE→USB ACK 사이에는 PWM0로 억제하며 이 일시 불일치만으로 FAULT를 만들지 않는다. 서보 증거 만료/준비 해제는 FAULT로 고정하고 CLEAR+새 ARM을 요구한다. 이미 전송된 USB 명령의 물리 효과나 실제 서보 위치 일치를 ACK만으로 보장하지 않는다.

## BNV1 모터 단독 벤치

bench 이미지에서만 해석하는 IA payload는 `<4s8sQQQBBHI`, **44 B / IA 52 B**이다. 앞 36 B는 magic `BNV1`, boot8, generation, lease, bench_seq로 ESV1과 같은 순서다. 이후 action u8, dir u8, duty_permille u16, duration_us u32다.

- action 1 ARM: MCU DISARMED, encoder/HAL 정상, 정지 피드백 200 ms, duty=duration=0. 서보·ESV·IMU 준비는 요구하지 않는다.
- action 2 PULSE: 이미 명시 ARM, duty 1..150 permille, duration 1..5,000,000 µs, dir 0/1. 진행 중 PULSE 재요청으로 기간을 연장하지 않는다.
- action 3 STOP: 안전 정지로 세션과 무관하게 수락하고 disarmed로 복귀한다. bench_seq는 마지막 수락한 non-STOP 번호를 유지한다.
- action 4 KEEPALIVE: duty=duration=0. 새 단조 seq와 새/현재 유효 lease로 host 기한만 갱신하며 최초 PULSE 종료 시각은 유지한다.

전원 투입 기본 disarmed이며 호스트는 시작 시 IA STOP을 명시적으로 보낸다. 따라서 호스트 절차는 `STOP → 정지 피드백 → CLEAR → 새 offer → BNV1 ARM → armed 관측 → PULSE → KEEPALIVE → disarmed`다. host 만료, 절대 종료, encoder/HAL 오류 또는 300 ms 카운트 무진행이면 PWM0/disarmed다. 300 ms는 준비된 검출 설정이며 검증된 모터 시동 물성이 아니다. 자동 반복/증가/재ARM은 없다. 벤치의 raw timer 방향은 차량 전진 부호로 채택하지 않는다.

## 관측과 전송 예산

`type:uno_q,v:1` IA JSON은 20 ms마다, 또는 accepted_seq/generation 변경 시 전송을 시도한다. 필수 키는 `boot,generation,accepted_seq,state,reason,bench_state,bench_seq,bench_reason,mode,profile_sha256,observation_seq,capture_us`이다. `bench_state`는 disarmed/armed/running, 초기 seq=-1이다. 초기 reason은 disarmed다. `motor_counts,raw_count,pwm_duty_ppm,hal_fault,imu_ready,servo_ready,servo_applied_seq,uart_errors,imu_dropped,tx_imu_dropped`는 진단이다.

`observation_seq,capture_us`는 성공한 encoder 읽기/피드백 수락 시의 MCU 시각·번호이며 같은 샘플 재보고 시 유지한다(초기 -1/0). 수신 시각을 취득 시각으로 바꾸거나 미확인 값으로 속도0을 만들지 않는다.

`profile_sha256`은 `json.dumps(profile,sort_keys=True,separators=(',',':'),ensure_ascii=True,allow_nan=False).encode('ascii')`의 SHA256이다. installed ARM 전에 호스트는 이 해시와 로컬 MCU profile, 기존 runtime_board의 대응값이 모두 같음을 확인해야 한다. 실제 설치 프로필은 아직 없으며 테스트용 게인을 채택하지 않는다.

offer는 40 ms, 상세 diag는 1 s다. 68 B IMU1 FIFO frame은 기존 형식·CRC·각 태그/원시 시각을 유지한다. accel/gyro 각208 Hz 및 timestamp 태그를 줄이지 않는다. 1 Mbps 8N1은 최대100,000 B/s다. 대략 IMU42.4 kB/s + compact34 kB/s + offer12 kB/s에 ACK/diag가 추가된다. 큐는 유한하며 drop 계수와 원본 seq를 남긴다. 이 예산은 산술 추정이고 실제 UART drain/IRQ 부하/태그율·지연·무손실 시험 결과가 아니다.
