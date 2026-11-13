# 실제 직렬 호스트 연결 r394

상태: 단일 I/O 소유자와 호스트 계약 core는 독립 검토 및 관련88검사 통과, ROS runner는 작성 중이다. 기본 ARM/DRIVE 차단·명시 STOP/상태 관측을 유지한다. 아직 실물 포트를 열거나 모터/서보를 구동하지 않았다.

미송신 DRIVE는 원 wire 그대로 최신1개만 남기고 제어는 최대8개로 한정한다. STOP은 대기 DRIVE/제어를 비우고 먼저 보내며 이미 I/O가 쓰기 시작한 프레임을 물리적으로 취소하지는 못한다. 각 명령의 원 source/lease/수명은 다시 계산하지 않는다. 준비와 실제 송신·ACK·물리 정지를 구분한다.

`verify_core_v1`에서 core/serial/공통기하/offer 회귀88개가 통과했다. ROS 환경 없이 실행되어 CDR1개는 건너뛰었으며, 해당 단일 검사는 r393/verify_v1에서 설치된 ROS 환경으로 수행해 통과했다. 입력 해시는 전후 동일하다. ROS runner의 종료 STOP 실제 송신 회수·services·sensor 전달 검증은 별도다.

## runner 준비 판본 — 실행 전

`staged/scripts/run_runtime_actuation_host.py`와 `test_runtime_actuation_runner.py`를 준비했다. Windows AST 문법 확인만 수행했으며 새 runner pytest·ROS graph·실물 포트는 실행하지 않았다. 공통 소스와 검증된 `runtime_actuation_link.py`는 수정하지 않았다. 아래 고정 판본을 실행 담당자가 한 번 검증한 뒤 결과를 별도 `verify_runner_v1`에 보존한다.

| 파일 | SHA256 |
|---|---|
| staged/scripts/run_runtime_actuation_host.py | `16f70aad595a71e27447d45ba05efdab0eb264dab4b5421a7b8b3e6490e4a226` |
| test_runtime_actuation_runner.py | `e6909446cb7052e11dc7989ea7d5d837751dca9f05ba3387c53613f1962fa3b3` |
| staged/scripts/runtime_actuation_link.py (기존 동결 core) | `c84e6d77338f0d61b8c18ef9e32579cddb2b88d9f63d2357e9af7e7975336611` |
| common scripts/same_host_command_clock.py | `fbe5bcd108a1c5fcd9c18c9daecf36603099210612a3ddbe03d73227921bd14b` |
| common scripts/receiver_owned_actuation_host.py | `005244d8918d4741c449a8f2f717a674c39cea643b360e52b751897777a6023f` |
| common scripts/runtime_offer_adapter.py | `d299180031713be7e0ab03a1c435494d4d8bcebe7e427d2c1348d11d3e345191` |

runner는 `SingleThreadedExecutor` 한 소유자와 별도 UART I/O thread를 사용한다. async Trigger가 결과를 기다리는 동안 timer가 ACK·offer·자체 기한을 진행하도록 reentrant callback group을 사용하지만 executor thread는 하나다. `/drive/safe`만 구독하고 `/drive`, `/drive/safe`, `/actuation/tx`는 발행하지 않는다. `--enable-drive-wire`가 없으면 ARM/DRIVE가 차단된다. 최초 STOP은 자동 ARM과 구분되며 재연결·자동 CLEAR/ARM은 없다.

`/actuation/{arm,clear_stop,disarm,stop}` Trigger는 원 request, ACK, 일치하는 송신 receipt, ARM/CLEAR/DISARM의 새 세대 offer/outcome과 자체 timeout을 확인한다. 첫 STOP처럼 session이 없어도 무한 대기하지 않는다. STOP pending 중 새 비STOP 요청은 보내지 않는다. 성공 응답도 실제 출력·정지 증명이 아니다. 실제 clock fault, MCU boot 변화, session/link closed를 관측하면 종료 STOP을 요청한다. 종료는 matching STOP tx receipt와 가능하면 ACK를 최대 0.5 s 회수한 뒤 포트를 닫는다. 이 값은 drain 예산이며 Python/OS scheduling의 절대 종료시간 보장은 아니다. 이미 쓰기 시작한 프레임의 취소, serial write 성공의 물리 정지 승격은 하지 않는다.

`/drive/safe` 원 ROS header는 r393의 physical ROS time→같은 host monotonic 구간으로 옮긴 뒤 그대로 r391 수명 계산에 사용한다. 0 stamp, 미래·오래된 source와 불확실 source는 그 명령만 생략한다. transient sample gap/read width는 r393의 두 정상 pair 재수립과 새 source 조건을 유지한다. 원 source를 수신/처리 시각으로 바꾸거나 lease를 다시 계산하지 않는다. 방금 clock capture의 명시 host 오차구간 안에 processing now가 있으면 시각을 미래로 옮기지 않고 그 명령을 생략한다. 실제 관측 역행은 계속 fault다. 알려지지 않은 sample 사이 jump를 완벽 검출한다는 주장은 없다.

control frame은 r392 확장 분리 후 원 UART receipt monotonic과 별도 processing now를 engine에 넘긴다. runtime observation은 로그용이며 session 권한에 넣지 않는다. IMU는 기존 `ImuFifoClock` 추정을 `/hardware_runtime/imu` 또는 명시 `--imu-topic`으로 전달한다. 가속도·자세를 허위 유효값으로 만들지 않고 covariance의 unavailable 표시를 유지한다. `/actuation/status`는 `McuStatusClock` 관측이며 원 feedback count/null을 보존한다. encoder→ROS motion 연결은 이번 범위 밖이다.

`registered.json`에 CLI·policy 원문 해시·명시 가정을 남긴다. `uart_rx.bin`, `uart_tx.bin`과 `events.jsonl`은 실제 chunk receipt/offset, prepared wire/source/lease, written receipt/ACK, clock rejection, 상태, 종료를 분리한다. 기본 IMU 상세 JSON 2행은 생략하며 `--debug-imu-events`에서만 추가한다. raw binary와 chunk 시각은 보존되므로 오프라인 해독이 가능하다. flush는 100 ms 또는 핵심 STOP/clock fault/종료에 묶으며 fsync/전원손실 보장은 아니다. raw UART·상세 JSONL은 기존 원시 자료 정책에 따라 공개 Git에 넣지 않으며, 실물 실행 output은 Git 밖 새 로컬 폴더로 지정한다.

## 검증 담당자 고정 명령

현재 checkout 루트에서 다음 명령만 수행한다. 실제 runner CLI/포트나 ROS graph는 이 단계에서 실행하지 않는다. fake ROS 모듈과 가짜 UART로 main·async service·single I/O lifecycle을 확인한다. core 88검사의 중복 전수 실행은 요구하지 않으며 runner가 사용하는 core 경계만 함께 재검사한다.

```bash
python3 -m pytest -q \
  artifacts/validation/2026-11-13/runtime_serial_host_r394_v1/test_runtime_actuation_runner.py \
  artifacts/validation/2026-11-13/runtime_serial_host_r394_v1/test_runtime_actuation_link.py
```

전후 위 runner/core/공통 3파일 해시를 보존한다. 실패는 원 log와 exit code를 남기고 판정선을 바꾸지 않는다. 검사 범위는 명시 policy/TTL 일치·기본 ARM/DRIVE 차단·원 source age·큰 epoch offset·receipt와 processing 분리·ACK/offer 지연·STOP timeout/tx 회수·boot/clock fault·transient 재수립·원 UART/IMU/status·fake ROS 발행 경계·실제 I/O thread+가짜 UART다. ROS CDR 자체는 r393 기존 통과 결과이며 이 테스트에서 새 ROS 실행으로 주장하지 않는다.

## 명시 clock policy 예제

다음은 하드웨어에서 확인한 값이 아닌 임시 가정 예시다. 사용 시 별도 JSON 파일로 명시해야 하며 runner 기본값은 없다. TTL을 자동 연장하지 않는다. same-host rate와 host/MCU rate는 서로 다른 가정이다. 큰 host timestamp error를 선택하면 capture 직후 명령이 생략될 수 있으며 runner는 시간을 조작해 통과시키지 않는다.

```json
{
  "schema": "runtime_host_clock_policy_v1",
  "receiver_lease": {
    "source_ttl_ns": 100000000,
    "max_receipt_age_ns": 100000000,
    "max_rate_error_ppm": 1000,
    "policy_note": "Temporary declared host/MCU relative-rate bound; hardware unverified"
  },
  "same_host": {
    "source_ttl_ns": 100000000,
    "max_lookback_ns": 100000000,
    "max_sample_gap_ns": 50000000,
    "max_read_width_ns": 1000000,
    "ros_timestamp_error_ns": 1000,
    "host_timestamp_error_ns": 0,
    "max_rate_error_ppm": 1000,
    "assume_no_unsampled_clock_step": true,
    "policy_note": "Temporary declared same-host ROS/monotonic assumptions; reported host reads define bounds; unsampled jumps not excluded by observation"
  },
  "max_lease_us": 100000,
  "request_timeout_ns": 200000000
}
```

실제 runner는 `--port`, `--output`(새 폴더), `--seconds`(1..3600), `--clock-policy`, `--max-speed-mps`, `--max-steering-rad`를 모두 요구한다. 최종 CAD 송신 조향 상한 `0.4578436017036438 rad`와 실제 장착 조향 확인은 별개다. 정책 schema/유한 수치/TTL 일치와 `use_sim_time` 비활성 physical ROS mode를 장치 open 전에 확인한다. 외부 ROS 장치 실행/운영 허가는 이 준비물의 검사 통과만으로 부여되지 않는다.
