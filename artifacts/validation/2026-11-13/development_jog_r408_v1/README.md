# r408 기존 개발용 짧은 조작의 상태·종료 연결 보완

상태: **staged 준비만 완료. 공통 미적용·pytest/ROS/WSL/Jetson/실물 미실행.** 기존 `scripts/development_jog.py`의 후속판이며 새 CLI·자율 실행 루프를 추가하지 않는다. 통합·사용 판단은 주 에이전트가 별도로 한다.

`RACE_OPERATOR_RUNBOOK.md` 49행에는 미구현이라는 옛 문장이 남아 있지만, 134행 이후와 r169에는 이미 같은 도구·제한 ROS 검사가 있다. r408은 그 원 기록을 수정하지 않고 현행 r394 host와 맞지 않는 부분만 보완한다. r169의 실제 차량/전체 MCU 전달 미검증도 유지한다.

## 확인한 연결 문제와 변경

- 기존 시작 검사는 virtual receiver의 `owner`, `target_speed_mps`, `feedback_valid`를 요구한다. 현행 host는 `McuStatusClock`의 관측 상태를 발행하며 이 세 필드가 없다. 대신 원 `boot/generation`, `clock_epoch`, 추정 상태 시각과 `motor_feedback.source_valid`가 있다. 두 형식을 구분하고 원 identity/시각·유효한 feedback을 검사한다. unavailable을 0속도로 만들지 않으며 runtime 상태에 없는 owner/target을 생성하지 않는다.
- 현재 Python/C++ receiver의 `ARMED`는 0목표 상태이다. 시작 때 이 상태를 요구하되 실제 속도 0·서보 engaged·host의 DRIVE 수락·실물 구동 허가를 증명한다고 쓰지 않는다. 기존 host/MCU가 source TTL·owner·ready·실제 출력 조건을 계속 검사한다. 기존 관측 최대나이 .25 s를 유지하며 새 MCU TTL이나 시계 보증을 만들지 않는다.
- 실제 정지 상태 이름은 `ESTOP_LATCHED`이다. 기존 jog의 `ESTOP` 검사로는 현행 receiver/host의 STOP 확인이 성립하지 않았다. 동일 boot, 증가한 generation, runtime이면 같은 clock epoch의 새 상태를 확인한다. runtime의 0목표 필드는 없으므로 결과 이름도 `fresh_estop_state_seen`으로 쓰며, 실측 0속도 관측이라고 부르지 않는다.
- 기존 cleanup은 0명령 발행과 STOP 요청을 한 `try` 안에 두어, 0발행 예외가 STOP까지 생략했다. 두 시도를 분리하고 STOP은 한 번만 요청한다. ACK 거절/유실·서비스 실패·관측 미확인은 실패로 기록한다. rclpy의 자동 신호 종료를 끄고 SIGINT/SIGTERM을 잡아 이 종료 경로가 먼저 실행되게 한다.
- `DRIVE`의 속도가 0이어도 receiver는 `steering`과 `drive_seen`을 갱신하고, `runtime_servo`는 `engaged && drive_seen`에서 조향 목표를 계산한다. 따라서 기존 cleanup의 `steering=0`은 중앙 목표가 될 수 있다. 종료 0명령은 마지막 요청 조향을 그대로 유지하고 곧바로 STOP을 요청한다. STOP 자체는 자동 중앙/torque OFF가 아니다. 호스트 STOP 우선 mailbox가 앞선 0명령을 버릴 수 있으므로 ROS 호출 순서가 UART 도착 순서·물리 반응을 보장하지는 않는다.
- 기본 상한과 허용 상한을 .2 m/s로 제한했다(이전 최대 .3). 기본 .15 m/s·.5 s, 허용 기간 .05–2 s, 좌/우 ±.2 rad는 **개발 시험 가정**이며 장착 교정/물리 안전 보증이 아니다. 기존 좌/우도 전진하면서 조향하는 동작이다. 후진·제자리 조향 모드·게인/정지 임계값 변경은 추가하지 않았다.

## I/O와 사용 범위

`--send`가 없으면 ROS를 import하거나 출력 폴더를 만들지 않고 계획 JSON만 출력한다. 실제 호출도 고정된 한 목표만 보내며 WASD/GUI/경기 중 수동 전환은 없다.

```bash
# 미리보기만. 이 명령도 이번 준비에서는 실행하지 않았다.
python3 -B artifacts/validation/2026-11-13/development_jog_r408_v1/staged/scripts/development_jog.py \
  forward --speed .15 --seconds .5 --speed-limit .2

# 후속 검토·실행 배정용 예시. 이번 단계의 실제 장치 사용 허가나 실행 기록이 아니다.
python3 -B artifacts/validation/2026-11-13/development_jog_r408_v1/staged/scripts/development_jog.py \
  left --speed .15 --seconds .5 --speed-limit .2 --send --output /tmp/jog-new-run
```

필요한 외부 구성은 독립 `lidar_safety`, `/drive/safe`를 구독하는 기존 `actuation_host` 또는 `runtime_actuation_host`, `/actuation/stop`의 `std_srvs/Trigger`, 신선한 `/actuation/status`이다. 자율 `/drive` 발행자는 운영자가 별도로 종료한 상태여야 한다. 이 도구는 노드를 죽이거나 초기화하지 않으며 ARM/CLEAR/RESET/래치 해제·host wire enable을 수행하지 않는다. 수신 MCU의 명시 ARM은 별도 기존 절차가 담당한다.

발행은 `AckermannDriveStamped`의 **`/drive`만**, 원 발행시점 ROS stamp와 `base_link`, 고정 speed/steering을 사용한다. `/drive/safe`는 독립 guard만 발행한다. `/actuation/command`, PWM, UART/serial에는 접근하지 않는다. 기본 .05 s 간격으로 요청하며 시간이 늦게 깨어났으면 기간을 넘겨 양수 명령을 추가하지 않는다. 이 간격·벽시계 .5 s가 실제 20 Hz scheduling이나 바퀴 이동거리 보증은 아니다.

처음부터 다른 `/drive` 발행자가 보이면 **명령/STOP 0건으로 거절**한다. 시작 후 다른 발행자, guard/host 연결 소실, MCU latch/ready 소실·boot/generation/clock epoch 변화·관측 만료, SIGINT/SIGTERM이면 양수 발행을 끝내고 0요청+STOP을 시도한다. 정상 기간 종료도 같은 경로다. 신규 graph 발견까지 최대3 s, STOP 확인 회수 최대1 s는 기존 도구의 예산을 유지한다. 원 ROS/OS 호출이 멈추거나 SIGKILL/전원 상실이면 Python cleanup은 보장할 수 없으며 기존 receiver expiry/독립 보호를 대체하지 않는다.

graph 조회는 발견된 endpoint 수와 namespace/name을 확인하는 실수 방지다. DDS 발견 지연 사이의 동시 시작, 조회 직후 발행자 진입, 같은 이름의 위장, service 제공자 인증을 원자적으로 막는 잠금/권한 체계가 아니다. 독립 도메인/자율 노드 OFF 확인은 운영자가 담당하며, 이 도구를 경기 모드에 상주시키지 않는다.

`request.json`과 `result.json`에 요청, 실제 ROS publish 호출, 실패, `zero_published`, `stop_requested`, `stop_ack`, 응답 원문, 새 latch 상태를 따로 남긴다. `zero_published`는 로컬 publish 반환이고 `stop_requested`는 service future 생성이다. `stop_ack`도 기존 host가 보고한 ACK 결과이며 `physical_stop_verified`는 항상 false다. 결과0은 위 종료 확인까지의 소프트웨어 흐름이며 차량이 움직였거나 멈췄다는 판정이 아니다. 출력 경로가 있으면 덮어쓰지 않는다.

## 고정 검사와 수용 조건

실행 담당에게 아래 **순수 검사 한 묶음만** 인계한다. ROS executor·DDS·serial·장치·시뮬을 시작하지 않는다. `FakeROS`는 main의 호출 순서를 메모리에서 기록하고, 별도1검사는 기존 `McuStatusClock`의 실제 변환 함수 결과를 사용한다. 합성 시간으로 실시간 성능/제동을 검증했다는 주장은 하지 않는다.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
python3 -B -m pytest -q -p no:cacheprovider \
  artifacts/validation/2026-11-13/development_jog_r408_v1/test_development_jog_runtime.py
```

수용 범위: 세 방향/상한/기간·invalid 입력, 두 기존 상태 형식·실제 상태 변환, feedback unavailable/추정 시각/세대 소실, 경쟁 발행자 시작 전 무송신, 정상 publish→0(조향 유지)→STOP 순서, 실행 중 경쟁/구동상실/인터럽트/지연, 양수 또는0 publish 오류에도 STOP 요청, STOP 서비스 오류/ACK 거절·유실의 실패 기록, 기본 미리보기 무ROS·무파일이다. AST 정적 준비만 수행했으며 검사 통과는 아직 주장하지 않는다.

`proposed.patch`는 현재 공통 jog 대비 변경이며 `manifest.json`에 준비파일과 읽은 계약의 SHA를 고정한다. 공통 파일/원 r169/원 r406은 수정하지 않았다. 최종 시뮬 짧은 조작·실제 구동 응답·방향/장착 조향·차체 정지는 후속 별도이며 지금 새 통합/사용을 완료한 것으로 표시하지 않는다.
