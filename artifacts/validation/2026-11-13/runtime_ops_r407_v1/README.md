# r407 B3 운영 버튼과 runtime host의 한정 연결 — 실행 준비

2026-11-13. [PLAN](PLAN.md)에 따라 정적 준비만 수행했다. ROS/WSL/Jetson/센서/실물 포트/firmware를 실행하거나 변경하지 않았다. 아래 검증은 지정 실행자에게 인계할 **1회 실제 ROS + 신선한 PTY 가짜 MCU 시험**이며 아직 통과 결과가 없다. 공통 소스 수정은0개다. r405의 동결 소스·문서도 수정하지 않았다.

## 원 근거와 파생 설정

r401 `prepared_v2`의 전체 parameter/bindings/expected header와 r404 RESULT, r366 README, r394 `revision_motor_v2/verify_ros_pty_sequence_v2.py`를 `reference/`에 원문 보존했다. 원 경로·SHA256은 `source_inputs.json`에 있다. r404 실제 수신은 C1183·RGB400·IMU3,795개 및 양쪽0명령/정상 종료까지이며 B3·실물 출력·경로 처리율의 근거는 아니다. 이번 시험에는 그 센서나 저장 센서 표본을 재생하지 않는다.

`prepared/runtime_profile.yaml`의 의미 있는 차이는 정확히 다음4개다. `profile.patch`는 YAML 표현 재정렬과 분리한 부모 검토용 semantic diff다. `prepared/bindings.json`은 원 r401 파일과 바이트 동일하다.

| 노드/매개변수 | 원 r401 | r407 명시 선택 |
|---|---|---|
| planner `race_control_mode` | `legacy` | `b3_candidate` |
| planner `development_speed_limit_enabled` | false | true |
| planner `development_speed_limit_mps` | 없음 | 0.3 m/s |
| guard `race_control_mode` | 없음(기본 legacy) | `b3_candidate` |

나머지 최종 기하·4구간 mask·기어비·센서 위치/축·임시 보정·시간·frame·계획/보호 정책은 유지한다. track `.1239` m, wheel width `.0261` m, radius `.0325` m, gear `2.5*81/17`, 송신 조향 상한 `.4578436017036438` rad이다. `c1_unmounted_preview`, `lsm6dsox_sensor`, `host_estimated_v1`, verified=false를 보존한다. 실장/절대 취득시각 검증으로 승격하지 않는다. 원 bindings의 과거 설명 필드도 덮지 않는다. 실제 노드의 기대 frame은 원 prepared_v2 parameter와 현재 공통 코드로 대조한다.

r405 현재 코너 후보를 켜지 않는다. 기존 runtime 정책 `cad_runtime_r377_v1`와 native 두 계산 슬롯은 r401 그대로다. 일반 운영의 `enable_drive_wire=false`는 bindings에 유지한다. 실제 포트를 지정한 새 실행 명령은 제공하지 않는다. 인계할 운영 대상은 이 명시 프로필과 기존 RaceOps/host API 연결이며 기본 실행값 변경이나 실물 구동 승인이 아니다.

## 대표 시험과 판정

`run_once.py`는 새 domain219/localhost에서 실제 G1G2·LidarSafety·관찰 노드를 SingleThreadedExecutor에 등록하고, RaceOps HTTP와 공통 host runner를 각각 별도 프로세스로 실행한다. RaceOps는 localhost의 자동 할당 포트만 열며 브라우저를 띄우지 않는다. 시험용 토큰·차량 이름을 사용한다. 외부 노드/actuation 서비스가 발견되면 중단한다.

1. 초기 `process_start` 래치, host의 STOP 뒤 ESTOP 상태 관측을 확인한다. `/drive` 발행자는 planner1개, `/drive/safe` 발행자는 실제 guard1개여야 한다. 하네스는 두 drive 토픽에 발행하지 않는다.
2. HTTP 버튼 API에서 정지 중 개발 상한을0.3→0.2 m/s로 변경한다. 값과 래치 유지를 확인하고 CLEAR/ARM을 유발하지 않았는지 본다. 모든 명령이0인 시험이므로 양수 목표의 실제 상한 작용/주행 성능을 새로 증명하지 않는다. 발행 직전 상한의 기존 회귀는 반복하지 않는다.
3. 신호 대기 밖 수동 출발을 HTTP에서 거절하고, 같은 `/race/command` 수신 노드도 현재 래치 상태에서 거절하는지 각각 확인한다. 실제 `WAIT_GREEN` 성공/영상인식·차량 출발은 이번 범위가 아니다. r366의212회귀는 기존 노드 메서드/세대/영상 연결 근거이며 실제 브라우저 클릭·주행 근거가 아니다.
4. **합성 과거 경기 기록**을 기존 `restore_race` API로 명시 복원한다. `prepared/synthetic_saved_race.json`의 synthetic/scope/start_source와 실제 사용 사본을 기록한다. 현재 시험 시각은 기록 신선도 필드에만 넣으며 센서/명령 시각을 다시 쓰지 않는다. 실제 영상 출발 사실을 만들지 않고 `restored_operator_confirmed`와 visual_start_fact=false를 확인한다. 이는 센서 없이 일반 STOP→START 분기를 검사하기 위한 fixture다.
5. **새 PTY에서만** 기존 host CLEAR/ARM Trigger를 명시1회씩 호출한다. planner 래치는 그대로 둔다. host의 `--enable-drive-wire`도 이 내부 생성 PTY 명령에만 붙인다. 센서 부재로 실제 guard가 만드는0명령을 host가 송신하는지 확인한다. r394 FakeMcu의 제어·ACK·세대 로직을 재사용하며 boot 이름과 r401의 속도/조향 상한만 변경했다(`fake_mcu.patch`). 실제 HAL은 없다.
6. RaceOps STOP 버튼→`/race/command` 래치와 `/actuation/stop` 서비스→host 실제 송신 receipt→같은 요청 ACK→가짜 MCU 수신을 대조한다. 이후 별도 RESET 없이 `resume_start`를 요청한다. 준비 수락과 새 입력 경계의 planner/guard 일치를 확인하고, `scan/rgb/gyro/encoder`4개 누락·래치 유지·추가 ARM0을 확인한다. **재개 수락은 주행 허가가 아니다.** 끝에는 새 STOP으로 대기를 취소한다.
7. 모든 관측 drive/가짜 wire 목표가0인지, guard 원 header가 host source와 wire까지 보존됐는지 확인한다. RaceOps를 먼저 종료하고 host에 SIGINT를 보낸 뒤 matching STOP tx/ACK를 회수한다. 두 계산 자식·영상 자식·HTTP/host/PTY thread·프로세스 그룹의 종료를 별도로 기록한다.

가짜 stationary count는 MCU 계약의 CLEAR/ARM 준비에만 필요하다. host에 `--motor-feedback-config`를 넘기지 않으므로 encoder publisher를 만들지 않는다. `/drive_motor/encoder` 발행자0, planner/guard 운동 표본0을 검사한다. 실제 센서와 합성 count를 섞지 않으며 없는 측정속도를0으로 채우지 않는다. IMU publisher의 생성과 실제 표본 수신도 구분한다. 테스트는 C1·카메라·IMU 표본을 발행하지 않는다.

host는 기존 정책의 TTL100ms/relative rate1000ppm/읽기 오차 가정을 그대로 사용한다. 이는 PTY 시험의 임시 가정이며 하드웨어 계측값이 아니다. 스케줄링·DDS·시계 관문에서 실패해도 TTL/한도/상태 조건을 늘려 성공시키지 않는다. host의 의도한 SIGINT는 현재 공통 구현상 `complete=false`, exit1이다. 이를 exit0으로 바꾸지 않고 `signal_2`·error없음·STOP 송신/ACK·잔여 프로세스와 분리해서 판정한다. 실제 차체 정지는 항상 미검증이다.

## 고정 실행과 회수

원 소스/입력은 정적 import 폐포의 `bundle_sources.zip` 및 `bundle_manifest.json`에 고정한다. 지정 실행자는 새 WSL home root에만 추출한다. Python 경로는 독립 소스를 먼저 쓰고 ROS setup의 inherited 경로를 뒤에 유지한다. 실제 프로젝트 import가 독립 root 밖이면 실패한다. 공통 파일을 설치하거나 덮지 않는다.

동결 source/input은208파일이며 bundle SHA256은 `152dd6a50703ca09c6bfdc7363383b1dda017db3c79ebb570c15e184a1384db1`이다. 정적 준비 과정은 source/config를 실행하지 않고 원문 복사·정해진 JSON scalar 프로필 비교·AST·import closure만 사용했다. 준비 호스트의 PyYAML 부재는 일반 YAML 처리를 흉내 내는 fallback으로 숨기지 않고, 동결 r401 생성 형식만 수락하는 좁은 reader로 제한했다. 실제 ROS preflight에서는 설치된 YAML parser로 다시 읽어 전체 parameter를 검사한다.

```bash
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
python3 -B artifacts/validation/2026-11-13/runtime_ops_r407_v1/execute_isolated.py \
  --root /home/jinhyeong/it-arena-r407-ops-20261113-a \
  --receipt artifacts/validation/2026-11-13/runtime_ops_r407_v1/verify_v1
```

단계는 native build120초→클래스 import/전체 parameter/명시 host 시계 정책 preflight30초→실제 ROS/PTY1회(내부45초, 외부70초)다. 별도 전수 pytest나 주행은 없다. 자식 정상 종료 대기 뒤 잔여 process group을 검사하고, 외부 상한 시 SIGINT10초 뒤 필요하면 KILL3초로 회수한다. 강제 회수는 성공으로 표시하지 않는다. 이미 존재하는 root/receipt는 거절하며 삭제·자동 재실행하지 않는다. 실패하면 해당 첫 결과/traceback/로그를 그대로 보존하고 다음 단계로 진행하지 않는다.

회수: `verify_v1/execution.json`, `results.zip`의 `preflight.json`, `results/report.json`, `host/summary.json`, 원 UART binary/events, HTTP/상태/명령·가짜 MCU 기록, 로그. 준비물과 추출 source의 전후 해시, native binary, 실제 import와 PID/exit을 함께 대조한다. 원시/세부 기록과 archive는 기존 Git 제외 정책을 유지한다.

실패 가능성은 ROS 의존 import/노드 초기화, domain discovery, B3 상태 갱신과 MCU 관측 지연, 엄격한 host 시계·lease 관문, ACK/새 세대의 시간 안 확인, 정상 자식 종료다. 어느 실패도 곧바로 차량 물성/보호 기준 문제로 해석하지 않는다. 성공하더라도 이번1회는 운영 요청 연결·입력 없는 준비·PTY 프로토콜까지이며 실물 출력·실차 정지·양수 주행·전 코스 검증으로 합산하지 않는다.

## 이관 검토

공통 코드 이관 후보는 없다. root가 검증 결과를 검토한 뒤 명시 B3 프로필과 운영 실행 연결의 채택을 결정한다. RaceOps의 별도 프로세스·기존 API, 기본 OFF 및 `/drive`→guard→`/drive/safe` 경계는 그대로다. 구동 host를 실물 포트에 연결하거나 motor feedback을 실제 센서로 연결하는 후속은 이번 시험과 별도다.

`prepare.py`는 원문/정적 AST/프로필/closure를 준비하는 스크립트이며 동결 인계 후 다시 실행하지 않는다. 전체 준비물 manifest 생성 후 인계 시점부터 root의 명시 반환 승인 전까지 문서를 포함한 어떤 파일도 수정하지 않는다. 실행 전과 실제 결과를 소급하여 바꾸지 않는다.
