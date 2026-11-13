# 서보 준비·ARM 결합 순서기 r384

2026-11-13, GPT. **Windows 코드 준비만 완료했다. 컴파일·합성 검사·실물·WSL 실행은 아직 하지 않았다.** 공통 소스·기존 firmware는 변경하지 않았다. [r383 자문](../mcu_engagement_advice_r383_v1/ADVICE.md)은 설계 참고이며 실행 결과나 하드웨어 동작 보증으로 취급하지 않는다. 자문의 “Engage 중 적분 누적 허용” 제안은 채택하지 않았으며 모터 PI 준비 중 reset/억제는 주 작업 담당자의 별도 연결 범위다.

## 최소 API와 순서

[`runtime_servo.hpp`](staged/src/arena_vehicle_interface/portable/runtime_servo.hpp)의 `RuntimeServo`는 기존 `Transaction`의 RAM `read_mode`, `read_position`, `goal`, `torque(true)`와 `ResolveCadServoGoal`/`ZeroProfile`만 사용한다. EEPROM, 토크 OFF, 중앙 복귀, 새 receiver 상태/프로토콜은 없다. 클래스는 receiver를 참조하지 않으며 같은 소유 작업이 검증한 snapshot을 전달한다.

```cpp
arena_servo::RuntimeServoSnapshot snapshot{
    endpoint.receiver().armed(), endpoint.receiver().generation(),
    endpoint.receiver().drive_seen(), endpoint.receiver().steering()};
servo.step(now_us, snapshot, rx_bytes, rx_size, nonblocking_uart_sink);
const bool servo_ready = servo.ready(now_us); // 통신·장착 프로필 증거
const bool servo_engaged = servo.engaged();  // 별도 모터 억제 조건
```

`ready`에는 유효 설정·CAD 범위/실장 영점 프로필·mode 0 확인·범위 내 최근 position·Fault 아님을 요구한다. `engaged`는 ARM 세대의 hold/torque ACK 완료 상태다. 모터 허가는 이 클래스가 만들지 않으며 실제 사용은 기존 receiver 허가와 `ready && engaged`를 함께 소비해야 한다. DRIVE 목표 속도가 0이어도 `armed=true, drive_seen=true`이면 조향 목표를 처리한다. 새 `drive_seen()`은 주 작업 담당자의 공통 receiver 연결 범위다.

| 상태 | 요청 | 전이 |
|---|---|---|
| Probe | mode 읽기 → position 읽기 | 유효 응답이면 Standby |
| Standby | 명시 주기의 position 읽기; 쓰기 0 | 새 ARM 세대를 관측하면 Engage |
| Engage | 새 position 읽기 → 읽은 위치 goal ACK → torque ON ACK | ACK 전부 성공 시 Engaged |
| Engaged | 실제 DRIVE 뒤 최신 CAD 목표 1개 및 주기 position 읽기 | STOP/비허가/세대 변경 시 결합 해제 |
| Fault | 새 송신 0, 원목표 재전송 0 | 외부 drain 뒤 명시 reset만 허용 |

ARM 직후에는 이전 `steering_rad`나 부팅 0을 목표로 쓰지 않는다. 첫 DRIVE 전에는 읽은 실제 위치를 hold하는 goal만 존재한다. 목표 큐는 없으며 송신 중의 목표 뒤에는 마지막 snapshot 하나만 계산한다. UART가 0 byte를 수락한 미송신 목표도 새 목표가 오면 교체하되 최초 요청의 기존 30 ms 마감은 그대로 유지한다. 실제 전송을 시작한 목표는 취소/재전송하지 않는다.

STOP·명령 만료·DISARM은 snapshot의 `armed=false`로 전달하고, 세대 변화는 generation으로 전달한다. 이를 ACK 소비보다 먼저 반영한다. 1 byte도 UART에 맡기지 않은 goal/torque는 폐기하며, 일부 또는 전체 송신을 시작한 Transaction은 기존 마감 안에서 ACK를 소비한다. 이후 이전 세대의 다음 쓰기는 시작하지 않는다. STOP과 같은 step의 torque ACK도 Engaged를 복원하지 않는다. 명시 새 ARM 세대는 이전 요청 종료 후 새 position 읽기부터 결합한다.

기존 Transaction의 요청별 `timeout_us=30000`을 그대로 사용하고 새 TTL·재시도는 추가하지 않았다. timeout/응답 status 오류는 Fault이며 목표 적용 여부는 미확정이다. `reset_after_drain(now)`는 외부가 TX/RX drain과 충분한 무송신 시간을 확보한 뒤에만 호출한다. reset 후 read-only Probe로 시작하고, Fault 전 ARM snapshot만 계속 들어오면 쓰기를 재개하지 않는다. 새 ARM 관측이 필요하다. read/write 응답 길이 검사는 기존 Transaction에 그대로 남아 있어 늦은 쓰기 ACK를 mode/position 응답으로 수용하지 않는다. 같은 길이의 늦은 read ACK까지 식별하거나 실제 UART의 모든 잔류 응답 제거를 보증하지는 않는다.

## 명시 설정과 실측 경계

`RuntimeServoConfig`의 기본값은 무효이며 실제 준비 프로필이 없으면 `ready=false`이고 송신도 하지 않는다. `ZeroProfile`과 CAD 표 수명, ID, goal speed/acceleration, position 읽기 주기, 관측 최대 나이, command_limit·wheelbase·track·wheel_limit를 호출자가 명시한다. 영점/끝단/방향/position mode/count scale 확인 없이 유효 프로필을 만들지 않는다.

현 `Transaction::goal` 자체가 speed 1..400·acceleration 5..20만 허용하므로 이 후보도 같은 API 범위만 수용한다. **이는 기존 probe용 소프트웨어 범위이지 주행용 선정·검증값이 아니다.** 값은 raw register 정수이며 실제 장치 설정을 포함한 물리 속도/가속 환산은 미확정이다. r383의 speed 400에 대한 각속도 추산은 사용하지 않았다. 범위 밖 주행 프로필을 지원하려면 Transaction의 별도 명시 변경·검증이 필요하다. 테스트의 speed 120/acceleration 10·100 ms 읽기·200 ms 관측 나이·영점 2000/끝단 1000..3000은 모두 합성 검사 입력이며 운영 기본값으로 채택하지 않았다. 실제 전류·링크 부하·토크가 이미 켜진 재ARM에서의 hold 거동·실장 목표 도달 시간은 실측 대상이다.

## 가짜 UART 검사와 실행 인계

[`tests/servo_runtime_cases.cpp`](tests/servo_runtime_cases.cpp)는 요청을 byte 단위로 모으는 가짜 UART sink와 실제 ST 응답 형식의 합성 ACK를 사용한다. [`tests/test_runtime_servo.py`](tests/test_runtime_servo.py)는 staged header만 include하여 C++17로 빌드하고 27개 사례를 실행한다. 컴파일러 부재는 skip으로 처리하지 않는다.

- 미ARM 읽기/쓰기 0, ARM hold 순서, torque ACK 전 engaged false, 첫 DRIVE 전 중앙 goal 없음, DRIVE 속도와 독립적인 조향 처리.
- 송신 중·전송 수락 0 상태의 latest-wins, 부분 송신 뒤 STOP은 현재 요청만 마침, 이미 송신한 DRIVE 목표 뒤 STOP은 쓰기 0.
- Engage의 position/hold/torque 각 단계에서 STOP·세대 변경·timeout, hold/torque 미송신 폐기.
- timeout 뒤 read-only 복구, 이전 ARM 자동 결합 없음, 늦은 write ACK를 두 종류 read 성공으로 오인하지 않음.
- 무효 기본/실장 프로필·CAD 범위·speed·mode·position·status·목표·역행 시각, 관측 최대 나이 경계.

`manifest.json`은 직접 참조하는 기존 portable 5개 파일과 모든 준비 소스/문서의 해시를 보존하며 검사 전후 동일성을 확인한다. 출력은 새 디렉터리만 사용한다. 준비 중 실행한 검사는 Windows Python AST 구문 검사·`git apply --check proposed.patch`·정적 파일 검사뿐이다. 실제 5 ms 주기 준수나 실시간 성능을 합성 시각 진행으로 검증했다고 주장하지 않는다.

자원 배정받은 실행 담당자가 저장소 루트에서 다음 한 묶음만 수행한다.

```sh
set -eu
A=artifacts/validation/2026-11-13/servo_engagement_r384_v1
test ! -e "$A/verify_v1"
mkdir "$A/verify_v1"
result=0
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q "$A/tests/test_runtime_servo.py" \
  --basetemp "$A/verify_v1/tmp" -o "cache_dir=$A/verify_v1/cache" \
  >"$A/verify_v1/pytest.log" 2>&1 || result=$?
cat "$A/verify_v1/pytest.log"
exit "$result"
```

실패 시 원 로그와 manifest를 보존하며 공통 이관을 중단한다. 제안 [`proposed.patch`](proposed.patch)는 신규 portable header 하나만 추가한다. 실제 ESP32 HAL·receiver/motor/runtime_tx 연결·기본 무효 프로필 ESP32 빌드는 주 작업 담당자가 별도 통합한다.

## 공통 이관 확인

가짜 UART 검사27개를 통과했고 참조/후보 파일의 전후 해시가 일치했다. `verify_v1/`에 전체 결과를 보존한다. 이후 GPT 주 에이전트가 순서기 헤더를 공통 portable 경로에 적용하고 staged와 바이트 동일을 확인했다(`integration.json`). 실제 보드 설치·물리 조향 검증은 하지 않았다.
