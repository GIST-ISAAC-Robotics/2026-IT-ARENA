> 후속 결과: 순수106검사, 실제ROS+가짜MCU PTY 검사, 공통 이관106검사를 완료했다. 원 배열비교 실패는 [검토](PTY_REVIEW.md)에 보존하며 실제 장치/출력 검증은 아니다. [통합 결과](verify_ros_pty_v2/report.json) · [이관](integration.json).

# r394 선택형 모터 피드백 연결 v2

상태: **준비·동결, 미실행**. 기존 v1 runner/core의 `verify_runner_v1` 73 passed와 전후 해시 불변은 보존했다. 이번 판본은 Windows AST 확인만 수행했으며 pytest·WSL·ROS·PTY·실물 포트를 실행하지 않았다. 원 v1 파일과 공통 코드는 수정하지 않았다. 공통 r397 helper의 기존 52검사와 이번 연결 검증을 구분한다.

## 변경 범위

`staged/scripts/run_runtime_actuation_host.py`는 v1 복사본에 `--motor-feedback-config`를 추가한다. 파일을 명시하면 장치 open/ROS 초기화 전에 exact schema를 읽고 `MotorEncoderSpec`와 공통 `MotorFeedbackRuntime`을 만든다. 옵션이 없으면 `/drive_motor/encoder` publisher 자체가 없다. 기본 ARM/DRIVE 차단, 원 `/drive/safe` source, clock policy, STOP/ACK/서비스 및 독립 보호 경계는 v1과 같다.

`McuStatusClock`에서 나온 동일 status를 원 `/actuation/status`에 발행하고 한 번만 변환한다. 유효 결과만 `JointState`로 발행하며 topic `/drive_motor/encoder`, name `drive_motor_shaft`, frame `drive_motor_encoder`, stamp는 원 `estimated_capture_ns`, position/velocity는 모터축 rad/rad/s다. 첫 표본·invalid·gap 등 무출력 조건은 helper를 그대로 따른다. 같은 count의 두 유효 표본이 만든 실제 관측 0속도는 허용한다. 좌우 바퀴 독립값이나 가짜 source를 만들지 않는다.

원 count·취득/receipt 시각·time basis는 기존 status 로그에 남는다. 모터 발행 계수와 첫/마지막 원값, helper의 거절별 계수, 처리 시각 기준 발행 빈도는 최대 1 Hz의 집계와 최종 summary에 남긴다. 이 빈도는 sensor capture rate나 실시간 달성 증명이 아니다. helper의 `maximum_age_ns`는 원 UART receipt−추정 capture에 적용되며 처리 큐 지연을 숨기지 않는다. downstream은 원 stamp의 freshness를 계속 평가해야 한다. 모터 설정값과 source 파일 해시는 실행 등록 자료로 보존한다.

정확한 JSON 형식 예제는 다음과 같다. 부호와 한계는 명시 설정이며 설치/실측 확정값으로 승격하지 않는다.

```json
{
  "schema": "runtime_motor_feedback_v1",
  "counts_per_motor_revolution": 28,
  "gear_ratio": 11.911764705882353,
  "wheel_radius_m": 0.0325,
  "encoder_sign": 1,
  "maximum_interval_ns": 200000000,
  "maximum_age_ns": 200000000,
  "maximum_speed_mps": 5.0
}
```

## 순수·가짜 ROS 검증 인계

checkout root에서 한 번 실행한다. 원 v1 테스트와 같은 이름의 v2 테스트를 **동시에** 수집하지 않는다. core 파일은 원 동결 판본을 재사용한다. 실패 결과를 보존하고 실행 중 소스를 수정하지 않는다.

```bash
python3 -m pytest -q \
  artifacts/validation/2026-11-13/runtime_serial_host_r394_v1/revision_motor_v2/test_runtime_actuation_runner.py \
  artifacts/validation/2026-11-13/runtime_serial_host_r394_v1/test_runtime_actuation_link.py \
  tests/test_motor_feedback_runtime.py
```

기존 runner 회귀에 exact config/장치 전 거절, 옵션 없음 publisher 부재, 첫/invalid 무출력, 2유효 stationary 0, 원 stamp/name/frame, 단일 변환·집계율 검사를 추가했다. 이 단계는 실제 ROS graph·장치를 열지 않는다.

## 실제 ROS + PTY 가짜 MCU 통합 1회 준비

`verify_ros_pty.py`는 **실제 serial 장치 경로를 받지 않는다.** 새 `os.openpty()`의 `/dev/pts/...`만 runner에 넘긴다. 기존 Python `ActuationReceiver`/`ContractConfig`와 실제 `actuation_wire` API로 가짜 MCU를 구성한다. 합성 stationary count와 `output_ready=True`는 PTY 시험 입력이며 실제 encoder·모터·servo HAL이 아니다. source·시계 경계 검증을 위해 이 고립 시험 graph 안에서만 `/drive/safe` 시험 메시지를 발행한다.

같은 ROS domain 217에 runner와 검사 노드를 두고 각 프로세스는 `SingleThreadedExecutor`를 쓴다. 최초 STOP 관측 후 명시 CLEAR→ARM을 요청하며 ARM ACK를 가짜 MCU에서 25 ms 보류한다. 그동안 Trigger가 완료되지 않는지 확인한 뒤 상관 ACK+새 세대 offer로 성공하는지 검사한다. 원 ROS header의 DRIVE가 같은 wire로 PTY 수신기에 도착하고 source가 재작성되지 않는지, 선택형 JointState의 관측된 0/name/frame이 맞는지, SIGINT 뒤 matching STOP tx receipt·ACK·실제 가짜 수신을 회수했는지 확인한다. v1의 SIGINT 종료 code 1은 원 의미대로 검사하고 성공 code 0으로 바꾸지 않는다.

실행 담당자가 WSL/ROS 자원과 domain 217을 독점한 상태에서 **단 한 번** 실행한다. ROS 환경은 기존 설치 setup을 source한 것을 전제로 한다. 출력 경로가 이미 있으면 실행하지 않는다. 외부 timeout은 14 s 후 INT, 추가 1 s 후 KILL로 전체 최대 15 s를 제한한다. 스크립트의 본 검사는 11 s 내에서 종료하고 남은 시간에 STOP·자식/PTY 종료를 회수한다. 만료/거절이 발생해도 TTL/lease/기준을 늘리거나 재시도하지 않는다.

```bash
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 \
timeout --signal=INT --kill-after=1s 14s \
python3 artifacts/validation/2026-11-13/runtime_serial_host_r394_v1/revision_motor_v2/verify_ros_pty.py \
  --output artifacts/validation/2026-11-13/runtime_serial_host_r394_v1/revision_motor_v2/verify_ros_pty_v1
```

`report.json`은 원 오류/검사별 증거와 PTY/실물 구분을, `runner/summary.json`은 실제 종료 확인을 보존한다. raw UART/상세 JSONL은 기존 원시 자료 Git 제외 정책을 유지한다. 이 revision의 로컬 `.gitignore`는 `verify_*` 아래 UART binary를 제외한다. 시험기 echo/ACK나 본 참조 수신기의 성공은 펌웨어/실물 출력·정지·주행 성공이 아니다. 실제 serial 자원, GPIO, flash 또는 기존 runtime 설치는 건드리지 않는다.

## 동결 입력

`source_manifest.json`은 새 3파일·원 v1·공통 helper/실제 import 경로·검사 의존 32파일을 SHA256으로 고정한다. 실행 전후 대조한다. 주요 판본은 다음과 같다.

| 파일 | SHA256 |
|---|---|
| v2 runner | `9a791e7f40781171d810ecb9916edb8efae6e34e85bf216a8cbe00f9018042db` |
| v2 pure/fake tests | `be942b09ff1f34cbf647d7b3624052fa51d88223a2a631b7a2f578e480368b3f` |
| PTY integration script | `3af5a8e076594523f0021620127f21d6cde3e61076c35cfe26d241406379cc99` |
| 원 v1 runner | `16f70aad595a71e27447d45ba05efdab0eb264dab4b5421a7b8b3e6490e4a226` |
| 원 v1 tests | `e6909446cb7052e11dc7989ea7d5d837751dca9f05ba3387c53613f1962fa3b3` |
| common motor_feedback_runtime.py | `7deaf336a07c905152eddbdbeff4ef60f8d75fcd993ed8394f3ed3b42694facc` |
