# MCU 구동 준비 순서 자문

읽기 전용으로 검토했으며 파일 변경·빌드·실행은 하지 않았습니다. 권고의 핵심은 **`output_ready`를 토크와 무관한 "링크 증거"로 정의하고, 토크 ON은 ARM 수락 뒤 HAL이 수행하며, 그동안 모터만 HAL에서 0으로 묶는 것**입니다. 이렇게 하면 순환 의존이 사라지고 receiver/ingress 계약은 그대로 유지됩니다.

## 1. 연결 순서와 최소 상태

구분할 것은 세 가지입니다.

- **`ready` (증거):** 루프마다 한 번 계산하고 저장하지 않습니다. 토크를 요구하지 않습니다.
- **armed/active (허가):** 기존 receiver 그대로입니다.
- **`engaged` (HAL 내부 모터 억제 조건):** 새 허가가 아니라 추가 금지 조건입니다.

`ready`의 구성 조건은 다음과 같습니다.

- 치명 `failed`가 아닐 것
- `ZeroProfile.valid()`, CAD `covers`, MD20A 프로필이 유효할 것
- 서보 mode==0을 확인했을 것
- 서보 Transaction이 Fault가 아닐 것
- 최근 position이 프로필 `[minimum, maximum]` 안일 것

프로필 헤더가 기본값(무효)이면 `ready`는 구조적으로 영구 false이므로, 지금 빌드는 출력 불가 상태로 남습니다.

서보 순서기는 portable의 작은 클래스 하나로 두고, 상태는 4개면 충분합니다.

| 상태 | 서보에 보내는 것 | `ready` | 모터 |
|---|---|---|---|
| Probe (부팅/복구) | read_mode, read_position만 | false | 0 |
| Standby | 주기 read_position만, 쓰기 0 | true | 0 |
| Engage (armed 세대 관측 시) | read_position → 범위 확인 → `goal(현재값)` ACK → `torque(1)` ACK | true | HAL 강제 0 |
| Engaged | 최신 CAD goal 1개(latest-wins)와 read 교대 | true | 매핑 결과 적용 |

- **Engage 시한:** 새 TTL을 만들지 않습니다. 각 단계가 기존 30ms deadline을 넘기면 Transaction Fault → `ready=false` → 다음 5ms tick에서 기존 `output_unavailable` FAULT로 이어집니다. 진행 중이던 read까지 포함해 최대 4단계입니다.
- **모터 0 유지:** r381 매핑의 `output_ready` 인자에 `ready && engaged`를 넘기면 `kInhibited`가 됩니다. Engage 중에 DRIVE가 수락되어 ACTIVE가 되어도 duty는 0입니다. PI 적분은 `[0, duty_cap]` 포화로 묶이므로 별도 처리가 필요 없다고 봅니다.
- **첫 조향:** `receiver.steering()`은 부팅 시 0이고 STOP 뒤에는 옛 값이 남습니다. ARM 직후 이를 쓰면 명령받지 않은 중앙 이동이 됩니다. **해당 세대에서 DRIVE가 수락된 뒤에만** CAD goal을 보내도록 하십시오.
  - 권장: receiver에 `drive_seen()` 접근자를 추가합니다(DRIVE 수락 시 true, `transition`에서 false, 3줄 수준).
  - 무변경 대안: Engaged 시점의 `accepted_seq()`를 기억하고 값이 바뀐 뒤부터 송신합니다. 첫 DRIVE 하나를 놓칠 수 있습니다.
  - `state==ACTIVE` 조건은 쓰지 마십시오. 목표 0으로 감속하며 굴러가는 동안 조향이 얼어붙습니다.
- **매 ARM마다 Engage를 거치는 단일 경로**를 권합니다. STOP 뒤에도 토크는 켜져 있으므로 재ARM의 `goal(현재값)`과 `torque(1)`은 사실상 무동작입니다. STOP 중 서보 전원이 재투입된 경우도 같은 경로로 처리됩니다. 토크 0 쓰기는 넣지 않습니다.

## 2. STOP·만료·서보 실패 시 goal과 late ACK

- **STOP·물리 STOP·명령 만료·DISARM:** 미송신 goal 슬롯만 버립니다. 전송 중인 트랜잭션은 `cancel()`하지 말고 끝까지 두어 ACK를 정상 소비합니다. 이후 쓰기는 0이고 read만 계속합니다. 조향은 마지막으로 송신한 goal에 머뭅니다. 자동 중앙이나 토크 OFF는 없습니다.
- **서보 응답 실패(30ms 초과, status 오류):** 그 goal의 적용 여부는 "미확정"으로 두고 재전송하지 않습니다. `ready=false`가 되어 기존 FAULT 경로로 duty 0이 됩니다.
- **late ACK:** ST 응답에는 요청 ID가 없어 식별할 수 없습니다. 현실적인 처리는 다음 순서입니다.
  1. 실패 뒤 `timeout_us` 이상 무송신으로 기다립니다.
  2. `uart_flush_input` 후 `reset_after_drain`을 호출합니다.
  3. **첫 트랜잭션을 read로** 시작합니다. `parse()`가 길이 불일치를 버리므로(`rx_[3]!=expected_+2`) 늦은 쓰기 ACK(길이 2)는 read 응답(길이 3/4)으로 오인되지 않습니다.
- **복구의 범위:** read 재프로브로 `ready`가 돌아오는 것은 증거의 자동 회복일 뿐입니다. 주행 재개는 기존 CLEAR→ARM→Engage→새 DRIVE가 있어야 합니다.
- **현재 소스의 한계:** `servo_failed`는 재부팅 전까지 래치되고 flush/reset 호출 경로가 없습니다. 호스트 UART의 비-DATA 이벤트 하나도 `vTaskSuspend`로 끝납니다. 운영에서는 전자만이라도 위 복구 경로로 바꾸시기를 권합니다.

## 3. 검증 범위 구분

**지금 (portable g++ + 가짜 서보 UART, ESP32는 빌드만)**

- 미ARM 구간의 서보 송신에 쓰기(instr 3)가 0건인지 확인합니다.
- 무효 프로필, mode≠0, 범위 밖 위치, 무응답에서 ARM이 `output_not_ready`로 거절되는지 확인합니다.
- ARM 뒤 순서가 read → goal==읽은 값 → torque(1)인지, 첫 DRIVE 전 CAD goal이 0건인지, torque ACK 전 duty가 0인지 확인합니다.
- Engage 단계 timeout에서 한 tick 내 FAULT·duty 0·추가 쓰기 0이 되는지 확인합니다.
- STOP, GPIO STOP, 만료, 링크 단절 뒤 새 쓰기 0, torque(0) 없음, 중앙 goal 없음을 확인합니다. GPIO 복귀만으로 재출발하지 않고, CLEAR→ARM→DRIVE로만 재개되는지도 봅니다.
- timeout 뒤 주입한 late ACK가 이후 쓰기의 성공으로 소비되지 않는지 확인합니다.
- 서보가 무응답이어도 5ms tick 간격과 `control_deadline_missed` 0이 유지되는지 확인합니다.
- offer의 `output_ready`가 루프당 `ready` 값과 일치하는지 확인합니다.
- duty→10bit 정수 변환, 상한, DIR 고정을 확인합니다.
- ESP32 빌드는 기본(무효) 프로필로 하고 `ready`가 상수 false임을 확인합니다. 합성 유효 프로필은 컴파일 확인용으로만 쓰고 flash하지 않습니다.

**실장 후 최소 확인**

1. 엔코더 부호와 바퀴 1회전 count를 확인합니다. 28×기어비가 맞다면 약 333.5가 기대값입니다.
2. 서보 영점·방향·끝단을 확인해 export-cpp 헤더로 만들고, ARM 시 hold에서 눈에 띄는 튐이 없는지 봅니다.
3. 바퀴를 띄운 상태에서 전진 DIR 레벨, 낮은 duty 상한, PWM0의 실제 감속을 확인합니다.
4. GPIO8 STOP과 호스트 단절에서 duty 0과 조향 유지를 확인합니다.
5. 모터 구동 중 서보 트랜잭션 오류 수와 tick 지연을 확인합니다. 벤치의 오류 0건은 무부하·모터 전원 분리 조건이었습니다.
6. 주행용 서보 speed/accel을 선정합니다.

## 4. 현 src의 구멍 3개

1. **`ready` 전달:** `runtime_tx.hpp:23`이 `endpoint.pump(now,sink,false)`로 고정되어 있습니다. `sync()`는 호출마다 `tx_.set_output_ready`를 덮어쓰므로 offer는 항상 false가 됩니다. 호스트는 `actuation_ros.py:123`에서 DRIVE를 보내지 않습니다. control_tick/feed/poll/pump 네 곳에 같은 루프의 `ready` 하나를 넘겨야 합니다. `main.cpp:127`의 armed 즉시 `failed` 처리도 함께 제거해야 합니다.
2. **조향 한계 불일치:** `main.cpp:97-98`이 `max_steering_rad`를 설정하지 않아 기본 0.37입니다. 호스트 producer는 조향을 제한하지 않으므로, CAD 중앙각 ±0.45785 기준 명령이 0.37을 넘으면 `invalid_drive` FAULT가 래치됩니다. `ResolveCadServoGoal`의 `command_limit`과 같은 값을 쓰도록 맞춰야 합니다.
3. **엔코더 미연결과 양자화:** `motor_feedback` 호출이 없어 ARM은 항상 `not_ready`입니다. 5ms 차분으로 넣으면 1count가 0.612mm, 즉 0.12m/s입니다. kp 0.6이면 duty가 약 ±0.07 흔들리고, 1count만으로도 200ms 정지 판정이 리셋됩니다. HAL에서 4tick(20ms)마다 공급하면 약 0.031m/s/count이며 fresh 50ms 안에 들어갑니다. 공급 주기는 실측으로 정하시기 바랍니다.

## 대안과 핵심 위험

- **대안 A (부팅 시 hold+토크 ON):** 코드는 가장 단순하지만 명시 허가 전에 통전됩니다. 권하지 않습니다.
- **대안 B (PREPARE 명령 신설):** 새 프로토콜이라 제외했습니다.
- **조향 속도 상한:** `Transaction::goal`의 speed≤400은 서보축 약 35°/s입니다. ±23.5° 전 구간에 약 1.3초가 걸려 주행에는 부족할 가능성이 큽니다. 한계를 프로필 인자로 빼 두되 값은 실측 후 정하시기를 권합니다.
- **미검증 가정:** 토크 ON 직전 goal을 현재 위치로 쓰면 움직임이 없다는 근거는 11/12 무부하 probe(토크0→goal(현재)→토크1)뿐입니다. 재ARM처럼 토크가 켜진 상태에서의 거동과 링크 부하 조건은 확인되지 않았습니다.
- **엄격한 실패 정책의 비용:** 서보 응답 1회 실패가 곧 주행 정지입니다. 재시도 허용 여부는 5번 항목의 실측 뒤에 판단하시는 편이 좋겠습니다.
- **토크 해제 수단:** 자동 OFF가 없으므로 서보 전원 차단이 유일한 해제 수단입니다.

## 읽은 범위와 한계

- **전체 열람:** `main.cpp`, `runtime_tx.hpp`, `fixed_tick.hpp`, `stop_switch.hpp`, preview `README.md`, `sdkconfig.defaults`, portable의 `actuation_motor_endpoint`, `motor_control_candidate`, `actuation_receiver`, `actuation_ingress`, `actuation_tx`, `speed_pi`, `st3215_transaction`, `steering_zero`, `cad_steering`, `cad_servo_goal`, r381의 `README.md`와 staged `md20a_forward_output.hpp`, `docs/sensors/ST3215_HS_HARDWARE_2026_11_12.md`.
- **일부만 열람:** `actuation_ros.py`(100–279행), `actuation_command.hpp`(drive 검사부), `probe_bus_servo.py`(155–194행), `STEERING_FUSION_2026_11_13.md`와 `steering_zero_profile.py`(검색 결과만).
- **읽지 않음:** `actuation_link`/`actuation_wire`/`imu_source.c`, r376 산출물, 실기 Jetson 브리지 스크립트, 배선 문서, r381의 나머지 staged 파일과 시험.
- **한계:** ST3215의 레지스터 동작과 응답 지연은 문서와 과거 probe 기록에 근거한 판단이며 직접 실행해 확인하지 않았습니다. 28 counts/rev가 4체배 기준이라는 점은 소스 설정을 그대로 가정했습니다.
